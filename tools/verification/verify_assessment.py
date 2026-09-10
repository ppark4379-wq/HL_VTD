#!/usr/bin/env python3
"""Replay captured VTD GT through the assessor and inspect existing geometry.

Never starts a controller, sends SCP/control commands, or regenerates a route,
corridor or gateway. All outputs must go to a new, separate directory.
"""
import argparse
from collections import Counter
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
from live_assessment import LiveAssessment
from rdb_stream import Decoder


def jsonlines(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def replay_frames(directory):
    """Re-decode ALL original bytes, restoring the original completed-frame clocks."""
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest.get('active') or manifest.get('error'):
        raise ValueError('recording must be closed without errors before replay')
    saved = iter(jsonlines(directory/'frames.jsonl.gz'))
    decoder, connection, offset = Decoder(), None, 0
    with gzip.open(directory/'stream.rdb.gz', 'rb') as stream:
        for chunk in jsonlines(directory/'chunks.jsonl.gz'):
            if chunk['offset'] != offset:
                raise ValueError('wire/chunk offsets are not contiguous')
            if connection != chunk['connection_id']:
                if decoder.buffer or decoder.frame:
                    raise ValueError('connection ended with incomplete RDB data')
                decoder, connection = Decoder(), chunk['connection_id']
            data = stream.read(chunk['length'])
            if len(data) != chunk['length']:
                raise ValueError('truncated RDB stream')
            offset += len(data)
            for frame in decoder.feed(data):
                original = next(saved)
                if (frame['frame'], frame['time']) != (original['frame'], original['time']):
                    raise ValueError('wire and recorded frame identities differ')
                for key in ('received_wall_time_ns', 'received_monotonic_ns'):
                    frame[key] = original[key]
                yield frame
        if stream.read(1) or next(saved, None) is not None:
            raise ValueError('unindexed bytes or unmatched decoded frames')
    if offset != manifest['received_bytes'] or decoder.buffer or decoder.frame:
        raise ValueError('recording byte count or final decoder state is invalid')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rdb-dir', type=Path, required=True)
    parser.add_argument('--baseline-assessment', type=Path, required=True)
    parser.add_argument('--scenario', type=Path, required=True, help='Existing scenario XML')
    parser.add_argument('--run-dir', type=Path, required=True, help='Existing verification run directory')
    parser.add_argument('--output', type=Path, required=True, help='New report directory')
    parser.add_argument('--rules', type=Path, default=ROOT/'tools/dashboard/assessment_rules.yaml')
    parser.add_argument('--skip-geometry-report', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error('--output must not already exist; original evidence is never overwritten')
    protected = (ROOT/'src', ROOT/'config', ROOT/'scenario', ROOT/'build', ROOT/'tools')
    if any(args.output.is_relative_to(p) for p in protected):
        parser.error('--output cannot be inside source, scenario, config, tools or build')
    session, baseline = None, None
    for record in jsonlines(args.baseline_assessment):
        if record.get('kind') == 'session':
            if session:
                raise ValueError('exactly one baseline session required')
            session = record
        elif record.get('kind') == 'result':
            baseline = record
    if not session or not baseline:
        parser.error('baseline must contain one session and a final result')
    args.output.mkdir(parents=True)
    rules = yaml.safe_load(args.rules.read_text())
    rules['route'] = copy.deepcopy(session['rules']['route'])
    rules.setdefault('metadata', {})['scenario_file'] = str(args.scenario.resolve())
    rules['metadata']['scenario_id'] = (session.get('active_profile') or {}).get('scenario_id', args.scenario.stem)
    rules_path = args.output/'replay_rules.yaml'
    rules_path.write_text(yaml.safe_dump(rules, allow_unicode=True), encoding='utf-8')
    engine = LiveAssessment(rules_path, args.output/'assessment')
    start, started, count = session['start_sim_time'], False, 0
    next_snapshot = start
    begun = time.monotonic()
    try:
        for frame in replay_frames(args.rdb_dir):
            if not started and frame['time'] > start+1e-8:
                if engine.previous is None or abs(engine.previous['time']-start) > 1e-6:
                    raise ValueError('original start-alignment frame is missing')
                engine.on_control_received(copy.deepcopy(session['start_evidence']))
                started = True
            engine.on_frame(frame)
            count += 1
            if started and frame['time'] >= next_snapshot:
                state = engine.snapshot()
                print(f"[replay] sim={frame['time']:.2f}s status={state['status']} "
                      f"penalty={state['summary']['official_penalty']}", flush=True)
                next_snapshot = frame['time']+30
        if not started:
            raise ValueError('no assessment frames after the original start')
        if engine.status == 'RUNNING':
            engine.finalize_incomplete('replay_eof_before_completion')
        final = engine.snapshot()
    finally:
        engine.close()
    assessment_path = next((args.output/'assessment').glob('*.jsonl'))
    final_path = args.output/'result.json'
    final_path.write_text(json.dumps(final, ensure_ascii=False, indent=2), encoding='utf-8')
    protected_inputs = [args.scenario, Path(rules['route']['csv']), args.rules,
                        args.baseline_assessment, args.rdb_dir/'manifest.json']
    sources = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected_inputs}
    comparison = dict(schema='hl_vtd_assessment_replay/v1', mode='OFFLINE_RECORDED_GT_REPLAY',
                      replay_frames=count, elapsed_processing_s=time.monotonic()-begun,
                      source_sha256=sources, baseline_summary=baseline.get('summary'),
                      revised_summary=final['summary'], assessment_jsonl=str(assessment_path),
                      controller_modified=False, generated_route_modified=False,
                      limitation='Regression on recorded data, not a new live drive or complete rule certification')
    reasons = ('signal_resolution_status', 'signal_resolution_reason',
               'centerline_measurement_reason', 'sidewalk_measurement_reason',
               'lane_transition_reason', 'object_missing_names', 'object_invalid_geometry')
    coverage = dict(
        meaning='Data coverage is not proof of detection accuracy; PASS means no violation in measured samples.',
        active_item_numbers=[item['number'] for item in final['official_grid']],
        excluded_items=[dict(number=3, reason='차로 유지: 기존 사용자 결정으로 제외')],
        reason_counts={field: dict(Counter(r.get(field, '') for r in engine.rows)) for field in reasons},
        exposure_counts={field: sum(r.get(field, '') not in ('', '0', '[]') for r in engine.rows)
                         for field in ('pedestrian_ids', 'crosswalk_active', 'static_collision', 'road_user_collision')},
        observation_gaps=final['observation_gaps'],
        ground_truth_inventory=final['ground_truth_inventory'])
    (args.output/'coverage.json').write_text(json.dumps(coverage, ensure_ascii=False, indent=2), encoding='utf-8')
    if not args.skip_geometry_report:
        from assessment_geometry_report import generate_report
        comparison['geometry_report'] = generate_report(
            run_dir=args.run_dir, report_dir=args.output/'geometry',
            route_csv=Path(rules['route']['csv']),
            checkpoints_csv=args.scenario.parent/'checkpoints.csv',
            assessment_jsonl=assessment_path)
    (args.output/'comparison.json').write_text(json.dumps(comparison, ensure_ascii=False, indent=2, default=str), encoding='utf-8')
    lines = ['# GT 자동채점 재검증', '',
             '저장된 원시 GT 재생 결과입니다. 새 실주행/전체 정확도 인증이 아닙니다.', '',
             f'- 상태: {final["status"]}',
             f'- 이전 감점: {baseline["summary"]["official_penalty"]}; 수정 후 감점: {final["summary"]["official_penalty"]}',
             f'- 종합점수: {final["summary"]["official_score"]}; 보류 사유: {final["summary"]["score_withheld_reasons"]}', '',
             '| 번호 | 상태 | 측정 | 미측정 | 해당 없음 | 감점 |', '|---|---|---:|---:|---:|---:|']
    for item in final['official_grid']:
        lines.append(f'| {item["number"]} | {item["status"]} | {item["measured_frames"]} | {item["missing_frames"]} | {item["not_applicable_frames"]} | {item["penalty"]} |')
    lines += ['', 'PASS는 측정된 샘플에서 위반이 검출되지 않았다는 뜻이며 정확도 인증이 아닙니다.',
              '3번 차로 유지는 기존 사용자 결정으로 제외되어 활성 평가 항목은 14개입니다.',
              '[프레임/판정 상세](result.json)', '[미측정 사유·위반 노출·GT 목록](coverage.json)',
              '[입력 해시와 이전/이후 비교](comparison.json)']
    if not args.skip_geometry_report:
        lines += ['[기존 궤적·콜리더·게이트웨이 시각화](geometry/index.html)']
    (args.output/'README.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(json.dumps({'output': str(args.output), 'result': str(final_path), 'summary': final['summary']}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
