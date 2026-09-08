"""Read-only scenario catalog and verification-run payload builders."""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Dict, List

import yaml

from assessment_types import RunPayload


REPO_ROOT = Path(__file__).resolve().parents[2]
VERIFICATION_DIR = REPO_ROOT / 'output' / 'verification'
CATALOG_PATH = REPO_ROOT / 'scenario' / 'scenario_catalog.csv'

sys.path.insert(0, str(REPO_ROOT / 'tools' / 'verification'))
import evaluate  # noqa: E402


OFFICIAL_ITEM_LABELS = {
    1: '제한속도 준수',
    2: '보호구역 속도 준수',
    4: '중앙선 침범(우측통행)',
    5: '보도 침범',
    6: '실선 구간 차로변경',
    7: '적색 신호 정지',
    8: '녹색 신호 통과',
    9: '적색 점멸 일시정지',
    10: '보행자 대응',
    11: '장애물 대응',
    12: '횡단보도 정차',
    13: '차선변경 방향지시등',
    14: '도로사용자 충돌',
    15: '리스폰(재출현)',
}


def load_catalog() -> List[Dict[str, str]]:
    with CATALOG_PATH.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def catalog_by_id() -> Dict[str, Dict[str, str]]:
    return {row['scenario_id']: row for row in load_catalog()}


def list_runs(scenario_id: str) -> List[dict]:
    base = VERIFICATION_DIR / scenario_id
    if not base.is_dir():
        return []
    runs = []
    now = time.time()
    for entry in sorted(base.iterdir()):
        run_csv = entry / 'run.csv'
        if not entry.is_dir() or not run_csv.is_file():
            continue
        stat = run_csv.stat()
        runs.append({
            'run_id': entry.name,
            'mtime': stat.st_mtime,
            'size': stat.st_size,
            'live': (now - stat.st_mtime) < 5.0,
        })
    runs.sort(key=lambda row: row['mtime'], reverse=True)
    return runs


def read_run_rows(scenario_id: str, run_id: str) -> List[Dict[str, str]]:
    path = VERIFICATION_DIR / scenario_id / run_id / 'run.csv'
    return evaluate.load_rows(path)


def section_columns(rows: List[Dict[str, str]]) -> List[str]:
    seen = {evaluate.section_key(row) for row in rows}
    return sorted(seen)


def official_grid(
    rows: List[Dict[str, str]],
    speed_limit_kmh,
    school_zone_limit_kmh,
) -> List[dict]:
    results = evaluate.evaluate_official_for(rows, speed_limit_kmh, school_zone_limit_kmh)
    return [
        {
            'number': item.number,
            'label': OFFICIAL_ITEM_LABELS[item.number],
            'status': item.status,
            'penalty': item.penalty,
            'detail': item.detail,
            'by_section': item.severity_by_section,
            'penalty_by_section': item.penalty_by_section,
        }
        for item in results
    ]


def latest_row(rows: List[Dict[str, str]]) -> Dict[str, str]:
    return rows[-1] if rows else {}


def to_float(row: Dict[str, str], field: str):
    return evaluate.parse_float(row, field)


def build_run_payload(
    scenario_id: str,
    run_id: str,
    since_frame: int,
    speed_limit_kmh,
    school_zone_limit_kmh,
) -> RunPayload:
    rows = read_run_rows(scenario_id, run_id)
    last = latest_row(rows)
    sections = section_columns(rows)
    grid = official_grid(rows, speed_limit_kmh, school_zone_limit_kmh)
    arguments = argparse.Namespace(
        official=True,
        require_goal=False,
        min_object_distance=None,
        max_abs_steering=None,
        max_abs_accel=None,
        speed_limit_kmh=speed_limit_kmh,
        school_zone_limit_kmh=school_zone_limit_kmh,
    )
    summary = evaluate.summarize(rows, arguments)
    track = [
        {
            'frame': int(to_float(row, 'frame') or 0),
            't': to_float(row, 'elapsed_wall_time_s'),
            'x': to_float(row, 'ego_x'),
            'y': to_float(row, 'ego_y'),
            'heading': to_float(row, 'ego_heading'),
            'speed': to_float(row, 'ego_speed'),
            'obj_x': to_float(row, 'nearest_object_x'),
            'obj_y': to_float(row, 'nearest_object_y'),
            'obj_dist': to_float(row, 'nearest_object_distance'),
            'obj_long': to_float(row, 'nearest_object_longitudinal_m'),
            'ttc': to_float(row, 'nearest_object_ttc_s'),
            'collision': row.get('collision', '').strip() in {'1', 'true', 'True'},
            'signal': row.get('traffic_light_state', '').strip(),
            'section': evaluate.section_key(row),
        }
        for row in rows
        if int(to_float(row, 'frame') or 0) > since_frame
    ]
    return {
        'scenario_id': scenario_id,
        'run_id': run_id,
        'frame_count': len(rows),
        'sections': sections,
        'official_grid': grid,
        'summary': summary,
        'latest': {
            'frame': int(to_float(last, 'frame') or 0) if last else None,
            'elapsed_wall_time_s': to_float(last, 'elapsed_wall_time_s'),
            'ego_speed_kmh': (to_float(last, 'ego_speed') or 0) * 3.6,
            'evaluation_section_id': evaluate.section_key(last) if last else None,
            'nearest_object_distance': to_float(last, 'nearest_object_distance'),
            'nearest_object_longitudinal_m': to_float(last, 'nearest_object_longitudinal_m'),
            'nearest_object_ttc_s': to_float(last, 'nearest_object_ttc_s'),
            'collision': last.get('collision', '').strip() in {'1', 'true', 'True'} if last else False,
            'traffic_light_state': last.get('traffic_light_state', '').strip() if last else '',
        },
        'track': track,
    }


def build_map_payload(scenario_id: str) -> dict:
    row = catalog_by_id().get(scenario_id)
    if row is None:
        return {'checkpoints': [], 'route': [], 'meta': None}
    scenario_dir = REPO_ROOT / 'scenario' / row['package_path']
    checkpoints = []
    checkpoints_path = scenario_dir / 'checkpoints.csv'
    if checkpoints_path.is_file():
        with checkpoints_path.open(newline='', encoding='utf-8-sig') as stream:
            for record in csv.DictReader(stream):
                checkpoints.append({
                    'seq': record.get('seq'),
                    'x': float(record['x']),
                    'y': float(record['y']),
                })
    route = []
    route_path = scenario_dir / 'route.csv'
    if route_path.is_file():
        with route_path.open(newline='', encoding='utf-8-sig') as stream:
            for record in csv.DictReader(stream):
                route.append({'x': float(record['x']), 'y': float(record['y'])})
    metadata = None
    yaml_path = scenario_dir / 'scenario.yaml'
    if yaml_path.is_file():
        data = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
        scenario_meta = data.get('scenario', {})
        metadata = {
            'name': scenario_meta.get('name'),
            'description': scenario_meta.get('description'),
            'map': scenario_meta.get('map'),
        }
    return {
        'checkpoints': checkpoints,
        'route': route,
        'meta': metadata,
        'category': row.get('category'),
        'notes': row.get('notes'),
    }
