#!/usr/bin/env python3
"""Read-only per-run observations; provisional PDF-item tables, not official scores."""
import argparse
import bisect
import csv
import gzip
import json
import io
import math
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/dashboard'))
from assessment_ground_truth import ScenarioGroundTruth, obb_overlap_3d, valid_geometry
from audit_crosswalk_batch import Auditor


class Episodes:
    def __init__(self):
        self.active = set()
        self.previous = None
        self.events = []

    def step(self, keys, t):
        keys = set(keys)
        if self.previous is None or t < self.previous or t-self.previous > .5:
            self.active.clear()
        for key in keys-self.active:
            self.events.append(dict(time_s=round(t, 3), object=key))
        self.active = keys
        self.previous = t


def independent_contacts(run_id, scenario, source_root):
    path = ROOT / 'output/verification' / run_id / scenario / 'rdb/frames.jsonl.gz'
    zstd = path.with_suffix('.zst')
    if not path.exists() and not zstd.exists():
        return dict(status='UNAVAILABLE')
    inventory = ScenarioGroundTruth(source_root / 'scenario/mixed' / scenario / (scenario+'.xml'))
    groups = {k: Episodes() for k in ('static', 'road_user', 'unknown', 'vtd_collision_flag')}
    frames = complete = 0
    process = subprocess.Popen(['zstd', '-q', '-d', '-c', str(zstd)], stdout=subprocess.PIPE) if zstd.exists() else None
    reader = io.TextIOWrapper(process.stdout) if process else gzip.open(path, 'rt')
    for line in reader:
        frame = json.loads(line)
        egos = [o for o in frame['objects'].values() if o.get('name') == 'Ego']
        if len(egos) != 1 or not valid_geometry(egos[0]):
            continue
        ego = egos[0]
        frames += 1
        complete += inventory.inspect(frame)['complete']
        keys = {k: [] for k in groups}
        for obj in frame['objects'].values():
            if obj['id'] == ego['id'] or not valid_geometry(obj):
                continue
            # Broad phase only; full 3D SAT is the actual contact test.
            if math.hypot(obj['x']-ego['x'], obj['y']-ego['y']) > (
                    obj['length']+ego['length']+abs(obj.get('off_x', 0))+5):
                continue
            if not obb_overlap_3d(ego, obj):
                continue
            role = inventory.roles.get(obj.get('name'))
            group = 'static' if role == 'static_obstacle' else (
                'road_user' if role in ('road_user', 'pedestrian', 'cyclist') else 'unknown')
            keys[group].append(f'{obj["id"]}:{obj.get("name", "")}')
        scoring = frame.get('scoring', {}).get(str(ego['id']), {})
        if scoring.get('collision_reported'):
            keys['vtd_collision_flag'].append('Ego')
        for key, episodes in groups.items():
            episodes.step(keys[key], frame['time'])
    reader.close()
    if process and process.wait():
        raise RuntimeError('zstd evidence decode failed')
    manifest = json.loads((path.parent/'manifest.json').read_text())
    transport = json.loads((path.parent/'transport/transport.json').read_text())
    return dict(status='OBSERVED' if frames else 'UNAVAILABLE', frames=frames,
                capture_incomplete=bool(transport.get('shutdown_incomplete') or transport.get('error')),
                capture_error=transport.get('error'),
                last_sim_time=(manifest.get('last_frame') or {}).get('sim_time'),
                complete_inventory_frames=complete,
                events={k: v.events for k, v in groups.items()},
                note='Independent RDB 3D OBB contacts; persistent VTD flag is not a per-object collision count.')


def stopped_intervals(path):
    rows = list(csv.DictReader(path.open()))
    intervals = []
    start = None
    for row in rows:
        t = float(row['elapsed_wall_time_s'])
        if abs(float(row['ego_speed'])) < .1:
            start = t if start is None else start
        elif start is not None:
            if t-start >= 3:
                intervals.append(dict(start_s=start, end_s=t, duration_s=t-start))
            start = None
    if start is not None and rows:
        end = float(rows[-1]['elapsed_wall_time_s'])
        if end-start >= 3:
            intervals.append(dict(start_s=start, end_s=end, duration_s=end-start))
    return intervals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    out = ROOT / 'output/verification' / args.run_id
    identity = json.loads((out / 'identity.json').read_text())
    batch = json.loads((out / 'batch.json').read_text())
    finished = {r['scenario'] for r in batch if r['status'] == 'FINISHED'}
    auditor = Auditor()
    results = []
    for number in identity['scenarios']:
        scenario = f'M{number:02}'
        if scenario not in finished:
            results.append(dict(scenario=scenario, status='NOT_FINISHED'))
            continue
        result = auditor.audit(scenario, args.run_id)
        result['independent_rdb'] = independent_contacts(args.run_id, scenario, Path(identity['root']))
        run = ROOT / 'output/verification' / scenario / args.run_id
        if (run / 'run.csv').exists():
            intervals = stopped_intervals(run / 'run.csv')
            result['stopped_intervals'] = intervals
            result['metrics']['longest_stop_s'] = round(max((r['duration_s'] for r in intervals), default=0), 2)
        independent = result['independent_rdb']
        if independent['status'] == 'OBSERVED':
            for item, group in ((11, 'static'), (14, 'road_user')):
                events = independent['events'][group]
                complete = (not independent['capture_incomplete'] and
                            independent['frames'] == independent['complete_inventory_frames'])
                result['items'][str(item)].update(
                    count=len(events), status='REVIEW' if events else 'NO_EVENT' if complete else 'PARTIAL',
                    note='Independent RDB 3D OBB + XML semantic role; incomplete transport/inventory is not a pass.')
        results.append(result)
        print(scenario, result['status'], result.get('metrics', {}), flush=True)
    (out / 'comparison_observations.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
    text = ['# '+args.run_id+' — 수정 전후 비교용 관측표', '',
            '공식 점수가 아닌 동일 조건의 관측 결과. NO_EVENT는 관측 구간 미검출이며 안전 보증이 아니다.',
            '차로 유지 세부 정의·공식 구간/종료선·리스폰 확정 자료가 부족하므로 총점은 임의 합산하지 않는다.',
            '정지시간에는 적색·보행자·정상 대기가 포함된다. 길찾기 정지는 decision 로그와 별도 대조한다.', '',
            '|시나리오|도착 상태|실행 초|제어 Hz|최장 정지 초|정적 OBB 접촉|이용자 OBB 접촉|',
            '|---|---|---:|---:|---:|---:|---:|']
    for r in results:
        m = r.get('metrics', {})
        items = r.get('items', {})
        text.append('|'+ '|'.join(str(v) for v in [r['scenario'], r['status'],
            m.get('duration_s', '—'), m.get('control_hz', '—'), m.get('longest_stop_s', '—'),
            items.get('11', {}).get('count', '—'), items.get('14', {}).get('count', '—')])+'|')
    text += ['', '## PDF 15항목 상세', '', '|시나리오|항목|판정|관측 횟수|근거/한계|', '|---|---|---|---:|---|']
    with (out / 'comparison_items.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['scenario', 'item', 'name', 'status', 'count', 'note'])
        for r in results:
            for key, item in r.get('items', {}).items():
                writer.writerow([r['scenario'], key, item['name'], item['status'], item['count'], item['note']])
                text.append(f'|{r["scenario"]}|{key}. {item["name"]}|{item["status"]}|{item["count"]}|{item["note"]}|')
    (out / 'comparison_report.md').write_text('\n'.join(text)+'\n')


if __name__ == '__main__':
    main()
