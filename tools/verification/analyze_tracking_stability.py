#!/usr/bin/env python3
"""Compare recorded tracking geometry, commands and path replacements (read-only).

No simulator/scenario assumptions. Path-change distances compare both paths at
the SAME current pose, avoiding false changes from normal forward motion.
"""
import argparse
import csv
import gzip
import json
import math
from pathlib import Path


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def project(path, ego):
    best = None
    for i, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        ds2 = dx * dx + dy * dy
        if ds2 < 1e-10 or abs(angle(math.atan2(dy, dx) - ego['heading'])) > 1.2:
            continue
        u = max(0., min(1., ((ego['x'] - a[0]) * dx + (ego['y'] - a[1]) * dy) / ds2))
        q = (a[0] + u * dx, a[1] + u * dy)
        d = math.hypot(q[0] - ego['x'], q[1] - ego['y'])
        if best is None or d < best[0]:
            best = (d, i, u, q)
    return best


def ahead(path, projection, distance):
    _, i, _, previous = projection
    for p in path[i + 1:]:
        ds = math.hypot(p[0] - previous[0], p[1] - previous[1])
        if ds >= distance:
            u = distance / max(ds, 1e-12)
            return (previous[0] + u * (p[0] - previous[0]), previous[1] + u * (p[1] - previous[1]))
        distance -= ds
        previous = p
    return None


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values) - 1, int((len(values) - 1) * fraction))] if values else None


def analyze(directory):
    rows = [r for r in csv.DictReader((directory / 'run.csv').open()) if r.get('collision') is not None]
    timing = [r for r in csv.DictReader((directory / 'timing.csv').open()) if r.get('loop_total_ms')]
    frames = []
    with gzip.open(directory / 'rviz/frames.jsonl.gz', 'rt') as stream:
        try:
            for line in stream:
                frames.append(json.loads(line))
        except EOFError:
            pass
    errors, changes, jumps = [], [], []
    previous = None
    for frame in frames:
        ego, path = frame['ego'], frame['tracked_path']
        projection = project(path, ego)
        if projection and ego['speed'] > .55:
            errors.append(projection[0])
        if previous is not None:
            dt = frame['time_s'] - previous['time_s']
            old_ego = previous['ego']
            step = math.hypot(ego['x'] - old_ego['x'], ego['y'] - old_ego['y'])
            yaw_step = angle(ego['heading'] - old_ego['heading'])
            if step > max(1., abs(ego['speed']) * max(dt, 0.) * 3.) or abs(yaw_step) > .5:
                jumps.append(dict(time=frame['time_s'], distance=step, yaw_step=yaw_step,
                                  old_road=old_ego['road_id'], road=ego['road_id']))
            old_projection = project(previous['tracked_path'], ego)
            if projection and old_projection and old_projection[0] < .75 and projection[0] < .75:
                shifts = []
                for distance in (3., 6., 10.):
                    a = ahead(path, projection, distance)
                    b = ahead(previous['tracked_path'], old_projection, distance)
                    if a and b:
                        shifts.append((a[0] - b[0]) * -math.sin(ego['heading']) +
                                      (a[1] - b[1]) * math.cos(ego['heading']))
                if shifts and max(abs(s) for s in shifts) > .15:
                    changes.append(dict(time=frame['time_s'], frame=frame['frame'],
                                        road=ego['road_id'], shifts_3_6_10m=shifts,
                                        phase=frame['decision']['lane_change_phase']))
        previous = frame
    starts = [float(r['loop_wall_start_s']) for r in timing]
    summary = dict(directory=str(directory), duration_s=float(rows[-1]['elapsed_wall_time_s']),
                   goal_reached=rows[-1]['goal_reached'],
                   invalid_path_frames=sum(r['plan_valid'] != '1' for r in rows),
                   logged_collision_frames=sum(r['collision'] == '1' for r in rows),
                   mean_control_hz=(len(starts) - 1) / (starts[-1] - starts[0]),
                   loop_ms_p95=percentile([float(r['loop_total_ms']) for r in timing], .95),
                   tracking_error_m_median=percentile(errors, .5),
                   tracking_error_m_p95=percentile(errors, .95),
                   tracking_error_m_max=max(errors, default=0.),
                   path_changes_over_15cm=changes, pose_discontinuities=jumps)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    analyze(parser.parse_args().directory)
