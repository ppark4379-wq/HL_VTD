"""Read-only start diagnostics. Never enables control or substitutes for VTD."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QPainterPath
from inspect_surface_alignment import area, path


def first_lanes(mission):
    return set(map(tuple, mission.get('corridor_groups', [{}])[0].get('lanes', [])))


def inspect_start(mission, observation, network, background, runtime_lanes=None):
    """Check the actual observed lane, pose and OBB against a planned start.

    Uses complete lane surfaces for body support, not the SVG's CP-centred
    display cuts. Actual runtime membership is a separate required input.
    """
    findings = []
    if not mission.get('initialized_ok'):
        return dict(status='BLOCKED', findings=['MISSION_INITIALIZATION_FAILED'])
    ego = observation.get('ego', {})
    try:
        values = [float(ego[k]) for k in ('x', 'y', 'heading')]
        lane = tuple(int(ego[k]) for k in ('road_id', 'section_index', 'lane_id'))
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite pose')
    except (KeyError, ValueError, TypeError, OverflowError):
        return dict(status='INCOMPLETE', findings=['MISSING_OR_INVALID_EGO_POSE_OR_LANE'])
    x, y, heading = values
    cp = mission['checkpoints'][0]
    if math.hypot(x-cp['x'], y-cp['y']) > 2.:
        findings.append('START_POSE_MISMATCH')
    if 'heading' in cp and abs(math.remainder(heading-cp['heading'], 2*math.pi)) > math.radians(20):
        findings.append('START_HEADING_MISMATCH')
    planned = first_lanes(mission)
    if lane not in planned:
        findings.append('EGO_LANE_MISSING_FROM_FIRST_CORRIDOR')
    if runtime_lanes is not None and lane not in runtime_lanes:
        findings.append('EGO_LANE_MISSING_FROM_RUNTIME_LEG_0')
    permitted = set(planned)
    gate = next((g for g in mission['legs'] if g.get('connection_lanes')), None)
    if gate:
        permitted.update(map(tuple, gate.get('surface_lanes', gate['connection_lanes'])))
        targets = {tuple(a) for a, b in gate['entry_edges']}
    else:
        targets = {tuple(mission['checkpoint_matches'][-1]['nearest_lane'])}
    if runtime_lanes is not None:
        permitted &= runtime_lanes
    seen, pending = set(), [lane]
    while pending:
        k = pending.pop()
        if k in seen or k not in permitted:
            continue
        seen.add(k)
        pending.extend(b for b, cost in network.search_graph[k] if b in permitted and b not in seen)
    if not seen & targets:
        findings.append('NO_DIRECTED_PATH_TO_FIRST_GATEWAY_OR_FINISH')
    # A raw RDB lane is optional in the present logger. If supplied, check it
    # too; world.egoRoadId alone is not proof of VTD's native lane identity.
    raw = ego.get('rdb_lane')
    if raw is not None and tuple(raw) not in planned:
        findings.append('RDB_LANE_MISSING_FROM_FIRST_CORRIDOR')
    if raw is not None and runtime_lanes is not None and tuple(raw) not in runtime_lanes:
        findings.append('RDB_LANE_MISSING_FROM_RUNTIME_LEG_0')
    body_outside = None
    try:
        corners = np.asarray(ego['obb'], dtype=float)
        if corners.ndim != 2 or corners.shape[1] < 2 or len(corners) < 4 or not np.isfinite(corners).all():
            raise ValueError('invalid OBB')
        body = path(corners[:, :2]*[1, -1])
        if area(body) < .5:
            raise ValueError('degenerate OBB')
        support = QPainterPath()
        for k in permitted:
            clip = background.clip(k)
            if clip is not None and clip.boundingRect().intersects(body.boundingRect()):
                support = support.united(clip)
        body_outside = float(area(body.subtracted(support)))
        if not support.contains(QPointF(x, -y)):
            findings.append('CENTER_OUTSIDE_START_SURFACE')
        if body_outside > .02:
            findings.append('VEHICLE_BODY_OUTSIDE_START_SURFACE')
    except (KeyError, ValueError, TypeError):
        findings.append('MISSING_OR_INVALID_OBSERVED_OBB')
    incomplete = [f for f in findings if f.startswith('MISSING_')]
    return dict(status='BLOCKED' if len(findings) > len(incomplete) else ('INCOMPLETE' if incomplete else 'STATIC_START_OK'),
                findings=findings, observed_lane=lane, body_outside_m2=body_outside,
                runtime_membership_checked=runtime_lanes is not None, raw_rdb_lane_checked=raw is not None)


def inspect_window(frames, initial, runtime_available, minimum_seconds=3.):
    """Movement and planning must occur together in fresh, ordered frames."""
    if initial['status'] != 'STATIC_START_OK':
        return dict(status=initial['status'], reason='initial_observation_failed')
    if len(frames) < 4:
        return dict(status='INCOMPLETE', reason='too_few_fresh_frames')
    try:
        times = [float(f['time_s']) for f in frames]
        numbers = [int(f['frame']) for f in frames]
        if not all(math.isfinite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])) or any(b <= a for a, b in zip(numbers, numbers[1:])):
            raise ValueError('stale_or_reordered_frames')
        if times[-1]-times[0] < minimum_seconds:
            return dict(status='INCOMPLETE', reason='observation_window_too_short')
        start = frames[0]['ego']; streak = 0; moved = False; safety_stop = False; any_valid = False
        if abs(float(start['speed'])) > .2:
            return dict(status='INCOMPLETE', reason='capture_started_after_motion')
        for f in frames:
            e, d = f['ego'], f['decision']
            numeric = [float(e[k]) for k in ('x', 'y', 'speed')]+[float(d['target_speed'])]
            if not all(math.isfinite(v) for v in numeric) or type(d['plan_valid']) is not bool or type(d['emergency_brake']) is not bool:
                raise ValueError('invalid_planning_telemetry')
            any_valid |= d['plan_valid']
            if f.get('collision') is True:
                return dict(status='BLOCKED', reason='collision_observed')
            hold = d['emergency_brake'] or int(d['stop_reason']) != 0
            safety_stop |= hold
            okay = d['plan_valid'] and not hold and numeric[2] > .1 and numeric[3] > .1
            streak = streak+1 if okay else 0
            if streak >= 3 and math.hypot(e['x']-start['x'], e['y']-start['y']) >= .5:
                moved = True
        if not any_valid:
            return dict(status='BLOCKED', reason='no_valid_local_plan_observed')
        if moved and runtime_available:
            return dict(status='OBSERVED_START_OK', reason='runtime_membership_and_valid_moving_frames_observed')
        if moved:
            return dict(status='INCOMPLETE', reason='runtime_corridor_snapshot_missing')
        return dict(status='WAIT_CONDITION' if safety_stop else 'BLOCKED',
                    reason='safety_or_traffic_stop_requires_review' if safety_stop else 'no_valid_movement_observed')
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        return dict(status='INCOMPLETE', reason='missing_or_invalid_telemetry: '+str(error))


def runtime_lanes_from_csv(filename, checkpoint=None):
    with filename.open(encoding='utf-8-sig', newline='') as stream:
        rows = csv.DictReader(stream)
        if not {'layer', 'group', 'road_id', 'section_index', 'lane_id'} <= set(rows.fieldnames or []):
            raise ValueError('Unsupported runtime static_map.csv schema; do not infer lane membership.')
        records = list(rows)
        if checkpoint is not None:
            first = next((r for r in records if r['layer'] == 'checkpoint' and r['group'] == '0'), None)
            if first is None or math.hypot(float(first['x'])-checkpoint['x'], float(first['y'])-checkpoint['y']) > .1:
                raise ValueError('Runtime checkpoint 0 does not match the selected test case')
        lanes = {(int(r['road_id']), int(r['section_index']), int(r['lane_id'])) for r in records
                 if r['layer'] == 'corridor' and r['group'].startswith('0:')}
    if not lanes:
        raise ValueError('No runtime leg 0 corridor found')
    return lanes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--case', required=True)
    parser.add_argument('--geometry', type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--frames', type=Path, help='Recorded JSONL frames; not proof of a live VTD run.')
    source.add_argument('--rviz-dir', type=Path, help='Observe fresh latest.json updates after this command starts.')
    parser.add_argument('--runtime-static', type=Path)
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 3:
        parser.error('--seconds must be finite and at least 3')
    if args.output.exists():
        parser.error('Choose a new output file; existing evidence is preserved.')
    cases = json.loads(args.suite.read_text(encoding='utf-8'))['cases']
    case = next(c for c in cases if c['id'] == args.case)
    frames = []
    if args.frames:
        frames = [json.loads(line) for line in args.frames.read_text(encoding='utf-8').splitlines() if line.strip()]
    else:
        latest = args.rviz_dir/'latest.json'
        baseline = latest.stat().st_mtime_ns if latest.exists() else None
        deadline = time.monotonic()+args.seconds
        previous_frame = None
        while time.monotonic() < deadline:
            try:
                stamp = latest.stat().st_mtime_ns
                if stamp != baseline:
                    f = json.loads(latest.read_text(encoding='utf-8'))
                    if f.get('frame') != previous_frame:
                        frames.append(f); previous_frame = f.get('frame')
                    baseline = stamp
            except (OSError, ValueError):
                pass
            time.sleep(.05)
    from whole_mission_corridors import Network
    from render_mission_corridors import load_background
    read = lambda name: json.loads((args.geometry/name).read_text(encoding='utf-8'))
    geo = {tuple(g['key']): dict(g, points=np.asarray(g['points'])) for g in read('lane_geometry.json')}
    network = Network(geo, read('lane_topology.json'), read('lane_graph.json'))
    runtime_file = args.runtime_static or (args.rviz_dir/'static_map.csv' if args.rviz_dir else None)
    lanes, runtime_error = None, None
    try:
        if runtime_file and runtime_file.exists():
            lanes = runtime_lanes_from_csv(runtime_file, case['mission']['checkpoints'][0])
    except (OSError, ValueError, KeyError, TypeError) as error:
        runtime_error = str(error)
    initial = inspect_start(case['mission'], frames[0], network, load_background(args.geometry/'road_map.svg'), lanes) if frames else dict(status='INCOMPLETE', findings=['NO_FRESH_FRAMES'])
    if runtime_error:
        initial = dict(status='INCOMPLETE', findings=['RUNTIME_SNAPSHOT_INVALID: '+runtime_error])
    result = dict(case=args.case, source='recorded_frames' if args.frames else 'observed_latest_json',
                  initial=initial, outcome=inspect_window(frames, initial, lanes is not None),
                  frames=frames, controller_interlock=False, full_drive_validated=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'frames'}, ensure_ascii=False, indent=2))
    return 0 if result['outcome']['status'] == 'OBSERVED_START_OK' else 2


if __name__ == '__main__':
    raise SystemExit(main())
