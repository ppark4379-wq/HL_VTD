#!/usr/bin/env python3
"""Copy original indexed JPEGs nearest assessment times, without transcoding."""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
from pathlib import Path


def read_complete_jsonl(path):
    rows = []
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if not line.endswith('\n'):
                break  # Live recorder's not-yet-complete final line.
            rows.append(json.loads(line))
    return rows


def jpeg_dimensions(data):
    if not data.startswith(b'\xff\xd8'):
        return None
    index = 2
    while index + 4 <= len(data):
        if data[index] != 0xff:
            return None
        while index < len(data) and data[index] == 0xff:
            index += 1
        if index >= len(data):
            return None
        marker = data[index]
        index += 1
        if marker in (0xd8, 0xd9) or 0xd0 <= marker <= 0xd7:
            continue
        if index + 2 > len(data):
            return None
        length = int.from_bytes(data[index:index + 2], 'big')
        if length < 2 or index + length > len(data):
            return None
        if marker in {0xc0, 0xc1, 0xc2, 0xc3, 0xc5, 0xc6, 0xc7,
                      0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf} and length >= 7:
            return [int.from_bytes(data[index + 5:index + 7], 'big'),
                    int.from_bytes(data[index + 3:index + 5], 'big')]
        index += length
    return None


def nearest(rows, keys, target):
    at = bisect.bisect_left(keys, target)
    choices = [i for i in (at - 1, at) if 0 <= i < len(rows)]
    return rows[min(choices, key=lambda i: abs(keys[i] - target))]


def extract(assessment_path, video_dir, output_dir, times, offsets=(-1.0, 0.0, 1.0), max_skew_s=.25):
    output_dir.mkdir(parents=True, exist_ok=False)
    assessment = [r for r in read_complete_jsonl(assessment_path)
                  if r.get('kind') == 'frame' and r.get('received_monotonic_ns') is not None]
    assessment.sort(key=lambda row: row['sim_time_s'])
    if not assessment:
        raise ValueError('No assessment frames with synchronization timestamps')
    simulation_times = [r['sim_time_s'] for r in assessment]
    report = dict(schema='hl_vtd_assessment_video_evidence/v1',
        assessment=str(assessment_path.resolve()), video_directory=str(video_dir.resolve()),
        clock='host_monotonic_ns', method='unaltered_jpeg_byte_copy',
        max_skew_s=max_skew_s, observations=[], video_indexes={},
        synchronization_limit=('Host receive timestamps select nearby evidence only. '
            'Video rendering, capture and buffering latency are not calibrated; '
            'a small timestamp skew does not establish equal VTD simulation time. '
            'Read the on-screen simulation clock when making event-level comparisons.'))
    for view in ('mainrs', 'birdrs'):
        index_path, data_path = video_dir / f'{view}.frames.jsonl', video_dir / f'{view}.mjpeg'
        if not index_path.exists() or not data_path.exists():
            report['video_indexes'][view] = {'error': 'missing_video_or_index'}
            continue
        frames = read_complete_jsonl(index_path)
        frames.sort(key=lambda row: row['monotonic_ns'])
        if not frames:
            report['video_indexes'][view] = {'error': 'empty_index'}
            continue
        keys = [r['monotonic_ns'] for r in frames]
        size = data_path.stat().st_size
        expected_offset = 0
        index_errors = []
        for frame in frames:
            if frame['offset'] != expected_offset:
                index_errors.append(dict(sequence=frame['sequence'], error='noncontiguous_offset'))
            expected_offset = frame['offset'] + frame['length']
            if expected_offset > size:
                index_errors.append(dict(sequence=frame['sequence'], error='beyond_observed_file_size'))
        report['video_indexes'][view] = dict(frames=len(frames), indexed_bytes=expected_offset,
            observed_data_bytes=size, unindexed_tail_bytes=max(0, size - expected_offset),
            errors=index_errors, first_monotonic_ns=keys[0], last_monotonic_ns=keys[-1])
        with data_path.open('rb') as stream:
            for event_time in times:
                for offset in offsets:
                    requested_time = event_time + offset
                    row = nearest(assessment, simulation_times, requested_time)
                    frame = nearest(frames, keys, row['received_monotonic_ns'])
                    skew_s = (frame['monotonic_ns'] - row['received_monotonic_ns']) / 1e9
                    stream.seek(frame['offset'])
                    jpeg = stream.read(frame['length'])
                    dimensions = jpeg_dimensions(jpeg)
                    reasons = []
                    if len(jpeg) != frame['length'] or not jpeg.endswith(b'\xff\xd9'):
                        reasons.append('incomplete_jpeg')
                    if dimensions is None or dimensions[0] < 320 or dimensions[1] < 180:
                        reasons.append('video_dimensions_too_small')
                    if abs(skew_s) > max_skew_s:
                        reasons.append('video_timestamp_too_far_from_assessment')
                    if abs(row['sim_time_s'] - requested_time) > .1:
                        reasons.append('assessment_time_not_available')
                    source = frame.get('source') or {}
                    if isinstance(source, dict) and source.get('source') == 'window':
                        title = str(source.get('title', '')).lower()
                        if view not in title:
                            reasons.append('window_title_does_not_match_requested_view')
                    status = 'unusable' if reasons else 'review'
                    destination = output_dir / status
                    destination.mkdir(exist_ok=True)
                    filename = f'{view}_event_{event_time:07.2f}_{offset:+.2f}s_frame_{row["frame"]}.jpg'
                    image_path = destination / filename
                    with image_path.open('xb') as output:
                        output.write(jpeg)
                    report['observations'].append(dict(view=view, event_sim_time_s=event_time,
                        offset_sim_s=offset, requested_sim_time_s=requested_time,
                        assessment_sim_time_s=row['sim_time_s'], assessment_frame=row['frame'],
                        assessment_monotonic_ns=row['received_monotonic_ns'],
                        assessment_wall_time_ns=row.get('received_wall_time_ns'),
                        ego_x=row.get('ego_x'), ego_y=row.get('ego_y'), ego_speed=row.get('ego_speed'),
                        video_sequence=frame['sequence'], video_monotonic_ns=frame['monotonic_ns'],
                        video_wall_time_ns=frame.get('wall_time_ns'), video_offset=frame['offset'],
                        video_length=frame['length'], timestamp_skew_s=skew_s,
                        dimensions=dimensions, source=source, status=status,
                        unusable_reasons=reasons, image=str(image_path.resolve()),
                        sha256=hashlib.sha256(jpeg).hexdigest()))
    with (output_dir / 'evidence_index.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(dict(output=str(output_dir.resolve()),
        observations=len(report['observations']),
        usable_for_review=sum(r['status'] == 'review' for r in report['observations']),
        unusable=sum(r['status'] == 'unusable' for r in report['observations']),
        video_indexes=report['video_indexes']), ensure_ascii=False, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assessment', required=True, type=Path)
    parser.add_argument('--video-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--times', required=True, nargs='+', type=float)
    parser.add_argument('--max-skew-s', default=.25, type=float)
    args = parser.parse_args()
    extract(args.assessment, args.video_dir, args.output_dir, args.times,
            max_skew_s=args.max_skew_s)


if __name__ == '__main__':
    main()
