"""Pure payload builders for dashboard HTTP endpoints."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from run_repository import build_map_payload, build_run_payload, list_runs, load_catalog


@dataclass
class DashboardRuntime:
    live_assessment: Any = None
    rdb_observer: Any = None
    control_events: Any = None
    video_relay: Any = None


def scenarios_payload() -> List[dict]:
    catalog = load_catalog()
    for row in catalog:
        row['run_count'] = len(list_runs(row['scenario_id']))
    return catalog


def live_payload(runtime: DashboardRuntime) -> dict:
    if runtime.live_assessment is None:
        return {'enabled': False}
    payload = runtime.live_assessment.snapshot()
    payload['enabled'] = True
    payload['receiver'] = dict(runtime.rdb_observer.status)
    payload['control_receiver'] = (
        dict(runtime.control_events.status)
        if runtime.control_events
        else {'connected': False, 'error': 'control monitor not started'}
    )
    return payload


def runs_payload(query: Dict[str, List[str]]) -> List[dict]:
    scenario_id = query.get('scenario_id', [''])[0]
    return list_runs(scenario_id)


def map_payload(query: Dict[str, List[str]]) -> dict:
    scenario_id = query.get('scenario_id', [''])[0]
    return build_map_payload(scenario_id)


def run_payload(query: Dict[str, List[str]]) -> dict:
    scenario_id = query.get('scenario_id', [''])[0]
    run_id = query.get('run_id', [''])[0]
    since_frame = int(query.get('since', ['-1'])[0])
    speed_limit = query.get('speed_limit_kmh', [None])[0]
    school_zone_limit = query.get('school_zone_limit_kmh', [None])[0]
    return build_run_payload(
        scenario_id,
        run_id,
        since_frame,
        float(speed_limit) if speed_limit else None,
        float(school_zone_limit) if school_zone_limit else None,
    )


def video_status_payload(runtime: DashboardRuntime) -> dict:
    if runtime.video_relay:
        return runtime.video_relay.status()
    return {'found': False, 'error': 'video relay not started'}
