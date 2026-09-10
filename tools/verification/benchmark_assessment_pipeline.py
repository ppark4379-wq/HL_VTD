#!/usr/bin/env python3
"""Read-only saved-row benchmark; never starts VTD or touches controller inputs.

This measures batch scoring / dashboard snapshots and the producer's lock wait,
not RDB decoding or raw per-frame GT geometry. Run before and after snapshot-lock
changes on the same evidence and compare the returned measurements.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import cProfile
import copy
import gzip
import hashlib
import io
import json
from pathlib import Path
import pstats
import statistics
import sys
import tempfile
import threading
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
sys.path.insert(0, str(ROOT / 'tools/verification'))
import evaluate
from live_assessment import LiveAssessment


def load_evidence(path):
    session, rows, digest = None, [], hashlib.sha256()
    with Path(path).open('rb') as stream:
        for line in stream:
            digest.update(line)
            record = json.loads(line)
            if record.get('kind') == 'session':
                if session is not None:
                    raise ValueError('expected a single assessment session')
                session = record
            elif record.get('kind') == 'frame':
                rows.append({key: '' if value is None else str(int(value))
                             if isinstance(value, bool) else str(value)
                             for key, value in record.items() if key != 'kind'})
    if session is None or not rows:
        raise ValueError('session and frame records required')
    return session, rows, digest.hexdigest()


def timed(callback, repeats):
    samples, result = [], None
    for _ in range(repeats):
        begin = time.perf_counter()
        result = callback()
        samples.append((time.perf_counter() - begin) * 1000)
    return {'median_ms': statistics.median(samples), 'max_ms': max(samples),
            'samples_ms': samples}, result


def prepare(engine, session, rows):
    engine.rows = list(rows)
    engine.status = 'RUNNING'
    engine.session_id = session['session_id']
    engine.start_evidence = session.get('start_evidence')
    engine.active_profile = session.get('active_profile')
    engine.active_scenario_id = rows[0]['scenario_id']
    engine.last_received = time.monotonic()
    engine.cache = None
    engine.deductions, engine.deducted = [], {}


def contention_probe(engine, session, rows, serialize_snapshot=False):
    """Append one saved row after scoring starts; check snapshot coherence."""
    prepare(engine, session, rows[:-1])
    entered, result, errors = threading.Event(), {}, []
    original = evaluate.evaluate_official_for

    def observed(*args, **kwargs):
        entered.set()
        return original(*args, **kwargs)

    def snapshot_worker():
        try:
            # Counterfactual comparison emulates the old critical-section
            # scope without loading/reverting any previous implementation.
            with engine.lock if serialize_snapshot else nullcontext():
                result['snapshot'] = engine.snapshot()
        except BaseException as error:
            errors.append(error)
            entered.set()

    evaluate.evaluate_official_for = observed
    try:
        worker = threading.Thread(target=snapshot_worker, daemon=True)
        worker.start()
        if not entered.wait(15):
            raise RuntimeError('snapshot did not enter batch scoring')
        start = time.perf_counter()
        with engine.lock:
            wait_ms = (time.perf_counter() - start) * 1000
            engine.rows.append(rows[-1])
            engine.cache = None
        worker.join(15)
        if worker.is_alive():
            raise RuntimeError('snapshot did not complete')
        if errors:
            raise errors[0]
    finally:
        evaluate.evaluate_official_for = original
    snapshot = result['snapshot']
    coherent = snapshot.get('latest', {}).get('frame') == rows[-2]['frame']
    preserved = len(engine.rows) == len(rows) and engine.rows[-1] is rows[-1]
    return {'mode': 'SIMULATED_OUTER_LOCK' if serialize_snapshot else 'CURRENT_IMPLEMENTATION',
            'producer_lock_wait_ms': wait_ms,
            'snapshot_prefix_coherent': coherent, 'appended_row_preserved': preserved,
            'snapshot_latest_frame': snapshot.get('latest', {}).get('frame'),
            'expected_snapshot_latest_frame': rows[-2]['frame'],
            'rows_after_append': len(engine.rows)}


def benchmark(path, repeats=3, profile_lines=18):
    load_start = time.perf_counter()
    session, rows, digest = load_evidence(path)
    report = {'mode': 'OFFLINE_SAVED_ASSESSMENT_ROWS_ONLY',
              'limitations': ['Does not measure RDB decoding or raw GT geometry.',
                              'Producer probe appends a recorded row under the production lock; it does not call on_frame.',
                              'Timing depends on host load; no hard realtime guarantee.'],
              'source': str(Path(path).resolve()), 'source_sha256': digest,
              'assessment_source_sha256': hashlib.sha256(
                  (ROOT / 'tools/dashboard/live_assessment.py').read_bytes()).hexdigest(),
              'row_count': len(rows), 'source_load_ms': (time.perf_counter()-load_start)*1000,
              'session_id': session['session_id'], 'measurements': []}
    with tempfile.TemporaryDirectory(prefix='assessment-benchmark-') as temporary:
        # Ephemeral assessor-only runtime config; source evidence is immutable.
        rules_path = Path(temporary) / 'rules.yaml'
        rules_path.write_text(yaml.safe_dump(session['rules']), encoding='utf-8')
        engine = LiveAssessment(rules_path, Path(temporary) / 'unused_logs')
        engine.file = None
        for count in sorted(set([min(1000, len(rows)), min(5000, len(rows)), len(rows)])):
            selected = rows[:count]
            prepare(engine, session, selected)
            batch, batch_result = timed(lambda: evaluate.evaluate_official_for(
                selected, engine.rules['normal_speed_kmh'], engine.rules['school_speed_kmh']), repeats)

            def fresh_snapshot():
                engine.cache = None
                return engine.snapshot()

            snapshot_timing, snapshot = timed(fresh_snapshot, repeats)
            expected = {item.number: item.penalty for item in batch_result}
            observed = {item['number']: item['penalty'] for item in snapshot['official_grid']}
            report['measurements'].append({'rows': count, 'batch': batch,
                'snapshot': snapshot_timing, 'batch_snapshot_penalties_equal': expected == observed})
        report['contention'] = contention_probe(engine, session, rows)
        report['serialized_lock_counterfactual'] = contention_probe(
            engine, session, rows, serialize_snapshot=True)
        prepare(engine, session, rows)
        profiler = cProfile.Profile()
        profiler.runcall(engine.snapshot)
        buffer = io.StringIO()
        pstats.Stats(profiler, stream=buffer).sort_stats('cumulative').print_stats(profile_lines)
        report['snapshot_profile'] = buffer.getvalue()
    report['checks_passed'] = all(item['batch_snapshot_penalties_equal'] for item in report['measurements']) and all(
        report['contention'][key] for key in ('snapshot_prefix_coherent', 'appended_row_preserved'))
    return report


def benchmark_raw_prefix(evidence_path, rdb_directory, connection_id=2, count=200):
    """Bounded original-wire replay of one connection; no whole-file materialization."""
    from rdb_stream import Decoder
    from rdb_recording import RdbRecordingSession
    with Path(evidence_path).open(encoding='utf-8') as stream:
        session = json.loads(next(stream))
    if session.get('kind') != 'session':
        raise ValueError('first evidence record must be a session')
    rules = copy.deepcopy(session['rules'])
    profile = session.get('active_profile') or {}
    scenario = profile.get('scenario_file')
    if scenario:
        rules.setdefault('metadata', {})['scenario_file'] = scenario
        rules['metadata']['scenario_id'] = profile.get('scenario_id', Path(scenario).stem)
    timings = {'decode_ms': 0.0, 'record_chunk_ms': 0.0, 'record_frame_ms': 0.0,
               'on_frame_ms': 0.0}
    frame_times, wire_bytes, decoded, started = [], 0, 0, False
    with tempfile.TemporaryDirectory(prefix='assessment-prefix-benchmark-') as temporary:
        temporary = Path(temporary)
        rules_path = temporary / 'rules.yaml'
        rules_path.write_text(yaml.safe_dump(rules), encoding='utf-8')
        engine = LiveAssessment(rules_path, temporary / 'assessment')
        recorder = RdbRecordingSession(temporary / 'rdb', 'OFFLINE_BENCHMARK', 0)
        recorder.event('connected')
        decoder = Decoder()
        rdb_directory = Path(rdb_directory)
        prefix_start = time.perf_counter()
        try:
            with gzip.open(rdb_directory / 'stream.rdb.gz', 'rb') as wire, gzip.open(
                    rdb_directory / 'chunks.jsonl.gz', 'rt') as chunks:
                for line in chunks:
                    chunk = json.loads(line)
                    data = wire.read(chunk['length'])
                    if len(data) != chunk['length']:
                        raise ValueError('truncated bounded raw prefix')
                    if chunk['connection_id'] < connection_id:
                        continue
                    if chunk['connection_id'] > connection_id:
                        break
                    wire_bytes += len(data)
                    now = time.perf_counter()
                    recorder.record_chunk(data, chunk['received_wall_time_ns'], chunk['received_monotonic_ns'])
                    timings['record_chunk_ms'] += (time.perf_counter()-now)*1000
                    now = time.perf_counter()
                    frames = list(decoder.feed(data))
                    timings['decode_ms'] += (time.perf_counter()-now)*1000
                    for frame in frames:
                        frame['received_wall_time_ns'] = chunk['received_wall_time_ns']
                        frame['received_monotonic_ns'] = chunk['received_monotonic_ns']
                        now = time.perf_counter()
                        recorder.record_frame(frame)
                        timings['record_frame_ms'] += (time.perf_counter()-now)*1000
                        if not started and frame['time'] > session['start_sim_time']+1e-8:
                            engine.on_control_received(copy.deepcopy(session['start_evidence']))
                            started = True
                        now = time.perf_counter()
                        engine.on_frame(frame)
                        duration = (time.perf_counter()-now)*1000
                        frame_times.append(duration)
                        timings['on_frame_ms'] += duration
                        decoded += 1
                        if decoded >= count:
                            break
                    if decoded >= count:
                        break
            rows = len(engine.rows)
            status = engine.status
        finally:
            recorder.close()
            engine.close()
        elapsed = time.perf_counter()-prefix_start
    if not frame_times:
        raise ValueError('no frames in requested connection prefix')
    sorted_times = sorted(frame_times)
    total_ms = sum(timings.values())
    return {'mode': 'OFFLINE_ORIGINAL_WIRE_BOUNDED_PREFIX', 'connection_id': connection_id,
            'decoded_frames': decoded, 'assessed_rows': rows, 'assessment_status': status,
            'wire_bytes': wire_bytes, 'timings': timings,
            'on_frame_median_ms': statistics.median(frame_times),
            'on_frame_p95_ms': sorted_times[min(len(sorted_times)-1, int(len(sorted_times)*.95))],
            'on_frame_max_ms': max(frame_times),
            'pipeline_average_ms_per_frame': total_ms/decoded,
            'pipeline_processing_frames_per_s': decoded*1000/total_ms,
            'frames_over_40ms_on_frame': sum(value > 40 for value in frame_times),
            'elapsed_including_input_decompression_and_close_s': elapsed,
            'limitations': ['Fixed connection prefix, not a full-run real-time guarantee.',
                            '25 Hz corresponds to 40 ms per frame.',
                            'Processing rate excludes input-file decompression and initialization/close.',
                            'No VTD connection or controller execution; only bounded original evidence replay.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assessment_jsonl', type=Path)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--rdb-dir', type=Path)
    parser.add_argument('--connection-id', type=int, default=2)
    parser.add_argument('--frames', type=int, default=200)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 20:
        parser.error('--repeats must be between 1 and 20')
    if args.output:
        output = args.output.resolve()
        if output.exists() or not (output.is_relative_to(ROOT / 'output') or output.is_relative_to(Path('/tmp'))):
            parser.error('--output must be a new file below output/ or /tmp/')
    if args.rdb_dir:
        if not 10 <= args.frames <= 500:
            parser.error('--frames must be between 10 and 500')
        report = benchmark_raw_prefix(args.assessment_jsonl, args.rdb_dir,
                                      args.connection_id, args.frames)
    else:
        report = benchmark(args.assessment_jsonl, args.repeats)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            stream.write(rendered + '\n')
    print(rendered)
    return 0 if report.get('checks_passed', True) else 1


if __name__ == '__main__':
    raise SystemExit(main())
