#!/usr/bin/env python3
"""Replay closed RDB evidence and compare every frame to its decoded record."""
import argparse
import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'dashboard'))
from rdb_stream import Decoder


def normalize(frame):
    saved = dict(frame)
    saved['lanes'] = [dict(player_id=k[0], road_id=k[1], lane_id=k[2], **v)
                      for k, v in frame['lanes'].items()]
    return json.loads(json.dumps(saved))


def verify(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    if manifest['active']:
        raise ValueError('Close recorder before final integrity verification')
    decoder = None
    connection_id = None
    offset = chunks = frame_count = 0
    objects_min = None
    objects_max = 0
    errors = []
    connection_stats = {}
    first_frame = last_frame = None
    with gzip.open(directory / 'stream.rdb.gz', 'rb') as raw, \
         gzip.open(directory / 'chunks.jsonl.gz', 'rt') as index, \
         gzip.open(directory / 'frames.jsonl.gz', 'rt') as decoded:
        for line in index:
            chunk = json.loads(line)
            if chunk['offset'] != offset:
                errors.append(dict(kind='chunk_offset', index=chunks, expected=offset,
                                   actual=chunk['offset']))
            if chunk['connection_id'] != connection_id:
                if decoder is not None:
                    connection_stats[str(connection_id)] = dict(decoder.stats,
                        pending_bytes=len(decoder.buffer), pending_frame=decoder.frame)
                connection_id = chunk['connection_id']
                decoder = Decoder()
            data = raw.read(chunk['length'])
            if len(data) != chunk['length']:
                raise ValueError('Raw stream shorter than indexed chunk')
            offset += len(data)
            chunks += 1
            for frame in decoder.feed(data):
                reference_line = decoded.readline()
                if not reference_line:
                    raise ValueError('Replayed frame has no recorded decoded frame')
                reference = json.loads(reference_line)
                for field in ['connection_id', 'received_wall_time_ns', 'received_monotonic_ns']:
                    reference.pop(field, None)
                if normalize(frame) != reference:
                    errors.append(dict(kind='decoded_frame_differs', frame=frame['frame'], sim_time=frame['time']))
                frame_count += 1
                identity = dict(frame=frame['frame'], sim_time=frame['time'], connection_id=connection_id)
                if first_frame is None:
                    first_frame = identity
                last_frame = identity
                count = len(frame['objects'])
                objects_min = count if objects_min is None else min(objects_min, count)
                objects_max = max(objects_max, count)
        if raw.read(1):
            raise ValueError('Raw stream contains unindexed tail bytes')
        if decoded.readline():
            raise ValueError('Decoded recording contains extra frames')
    if decoder is not None:
        connection_stats[str(connection_id)] = dict(decoder.stats,
            pending_bytes=len(decoder.buffer), pending_frame=decoder.frame)
    event_counts = {}
    with gzip.open(directory / 'events.jsonl.gz', 'rt') as stream:
        for line in stream:
            event = json.loads(line)
            kind = event['kind']
            event_counts[kind] = event_counts.get(kind, 0) + 1
    for key, actual in [('received_bytes', offset), ('chunks', chunks), ('frames', frame_count)]:
        if manifest[key] != actual:
            errors.append(dict(kind='manifest_count_differs', field=key, actual=actual,
                               expected=manifest[key]))
    for name, stats in connection_stats.items():
        if any(stats[k] for k in ['resynchronization_bytes', 'incomplete_frames_discarded',
                                  'malformed_messages', 'invalid_simulation_times', 'pending_bytes']):
            errors.append(dict(kind='decoder_problem', connection_id=name, stats=stats))
        if stats['pending_frame'] is not None:
            errors.append(dict(kind='unfinished_frame', connection_id=name))
    if not frame_count:
        errors.append(dict(kind='no_complete_frames'))
    if manifest.get('error'):
        errors.append(dict(kind='recorder_error', error=manifest['error']))
    if manifest.get('observed_missing_frame_numbers'):
        errors.append(dict(kind='missing_frame_numbers', count=manifest['observed_missing_frame_numbers']))
    return dict(schema='hl_vtd_final_rdb_integrity/v1', directory=str(directory.resolve()),
        pass_=not errors, errors=errors, all_four_gzip_streams_reached_validated_eof=True,
        raw_uncompressed_bytes=offset, indexed_chunks=chunks,
        replayed_frames=frame_count, decoded_frames_exactly_match_raw=not any(
            e['kind'] == 'decoded_frame_differs' for e in errors), first_frame=first_frame,
        last_frame=last_frame, object_count_min=objects_min, object_count_max=objects_max,
        event_counts=event_counts, decoder_connections=connection_stats,
        recorded_missing_frame_numbers=manifest['observed_missing_frame_numbers'],
        recorder_error=manifest['error'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = verify(args.directory)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['pass_'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
