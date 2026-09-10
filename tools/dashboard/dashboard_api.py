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
    video_relays: Any = None
    video_recording: Any = None
    scp_observer: Any = None
    respawn_log_observer: Any = None


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
    receiver = payload['receiver']
    transport = receiver.get('transport') or {}
    incomplete = bool(receiver.get('shutdown_incomplete') or receiver.get('processing_error') or transport.get('error'))
    delayed = receiver.get('processing_lag_s', 0) > 2
    if incomplete or delayed:
        # Do not mutate the assessor's cached prefix. This guard is about live
        # transport completeness, not a new driving violation or deduction.
        summary = dict(payload['summary'])
        reasons = list(summary.get('score_withheld_reasons', []))
        reason = 'incomplete_transport_evidence' if incomplete else 'assessment_processing_backlog'
        if reason not in reasons:
            reasons.append(reason)
        summary.update(official_score=None, score_withheld_reasons=reasons)
        if summary.get('official_evaluation') == 'COMPLETE':
            summary['official_evaluation'] = 'PARTIAL'
        payload['summary'] = summary
        payload['warnings'] = list(payload.get('warnings', [])) + [
            ('GT 수신·처리 오류: 원본 보존 상태 확인 전 점수 보류' if incomplete else
             f"GT는 보존 중이나 채점이 {receiver['processing_lag_s']:.2f}초 지연됨: 처리 완료 전 점수 보류")
        ]
    payload['control_receiver'] = (
        dict(runtime.control_events.status)
        if runtime.control_events
        else {'connected': False, 'error': 'control monitor not started'}
    )
    payload['scp_receiver'] = dict(runtime.scp_observer.status) if runtime.scp_observer else {
        'connected': False, 'receive_only': True, 'error': 'SCP observation disabled',
        'respawn_completion_verified': False}
    payload['respawn_log_receiver'] = dict(runtime.respawn_log_observer.status) if runtime.respawn_log_observer else {
        'connected': False, 'receive_only': True, 'error': 'plugin log observation disabled',
        'respawn_completion_verified': False}
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


def video_status_payload(runtime: DashboardRuntime, view: str = 'mainrs') -> dict:
    relays = runtime.video_relays or {}
    relay = relays.get(view)
    if relay is None and view == 'mainrs':
        relay = runtime.video_relay  # compatibility with the former single relay
    if relay:
        return relay.status()
    return {
        'found': False,
        'view': view,
        'source': None,
        'error': f'{view} video relay not started',
    }


def video_recording_status_payload(runtime: DashboardRuntime) -> dict:
    if runtime.video_recording is None:
        return {
            'enabled': False,
            'status': 'DISABLED',
            'active': False,
        }
    payload = runtime.video_recording.status()
    payload['enabled'] = True
    return payload
