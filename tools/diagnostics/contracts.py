"""Data contracts for field capture and compact GPT evidence. No scoring."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CaptureRequest:
    root: Path
    directory: Path
    binary: Path
    arguments: list[str]
    legacy_decision_log: bool = False


@dataclass
class Incident:
    code: str
    first_frame: int
    last_frame: int
    first_wall_s: float
    last_wall_s: float
    evidence: dict[str, Any] = field(default_factory=dict)


@dataclass
class ScanResult:
    frames: int = 0
    records: int = 0
    send_completed: int = 0
    sends_skipped: int = 0
    sends_failed: int = 0
    issues: list[str] = field(default_factory=list)
    incidents: list[Incident] = field(default_factory=list)
    counts: dict[str, int] = field(default_factory=dict)
    first: dict[str, Any] = field(default_factory=dict)
    last: dict[str, Any] = field(default_factory=dict)
    last_stage: dict[str, Any] = field(default_factory=dict)
    maximum_planning_ms: float = 0.0
    maximum_receive_ms: float = 0.0
    maximum_clock_ms: float = 0.0
    goal_reported: bool = False
    send_interval_count: int = 0
    send_interval_total_ms: float = 0.0
    maximum_send_interval_ms: float = 0.0
    send_intervals_over_55ms: int = 0
    maximum_receive_to_send_ms: float = 0.0
    maximum_diagnostic_capture_ms: float = 0.0
    logger_status: dict[str, Any] = field(default_factory=dict)
    repair_status: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PackRequest:
    directory: Path
    context_seconds: float = 5.0
    max_windows: int = 8
    max_excerpt_records: int = 320


@dataclass(frozen=True)
class ExcerptAnchor:
    frame: int
    wall_s: float
    incident: bool = True


@dataclass
class FrameEvidence:
    frame: int
    first_line: int
    records: int = 0
    candidate: bool = False
    near_incident: bool = False
    distance: float = float("inf")


# These are investigation triggers, NOT official rules or proof of a code defect.
NO_MOTION_SECONDS = 2.0
NO_MOTION_DISTANCE_M = 0.2
PLANNING_SLOW_MS = 100.0
SAMPLE_TIME_GAP_S = 0.5

# Exact current repository locations. All source files are also snapshotted.
SOURCE_MAP = {
    "NO_FRAMES": ["src/main.cpp", "src/network/receive/gt_receiver.cpp", "src/network/receive/packet_parser.cpp", "src/network/vtd_client.cpp"],
    "INPUT_TIMEOUT": ["src/network/receive/gt_receiver.cpp", "src/network/vtd_client.cpp", "src/main.cpp"],
    "SEND_FAILED": ["src/network/send/control_sender.cpp", "src/network/vtd_client.cpp"],
    "RECEIVE_ONLY": ["src/main.cpp"],
    "INVALID_PLAN": ["src/planning/planning_process.cpp", "src/planning/local/corridor_lattice.cpp", "src/data/gt/world_process.cpp"],
    "SAFETY_OVERRIDE": ["src/control/control_process.cpp", "src/control/safety/safety_limiter.cpp"],
    "COMMAND_WITHOUT_MOTION": ["src/network/send/control_sender.cpp", "src/control/longitudinal/longitudinal_controller.cpp", "src/data/gt/world_process.cpp"],
    "ZERO_TARGET_WAIT": ["src/planning/planning_process.cpp", "src/planning/behavior/behavior_planner.cpp", "src/planning/local/corridor_lattice.cpp"],
    "EMERGENCY_BRAKE": ["src/planning/collision/collision_checker.cpp", "src/planning/planning_process.cpp"],
    "COLLISION_LOGGED": ["src/planning/collision/collision_checker.cpp", "src/planning/local/lane_change_planner.cpp"],
    "ROUTE_SEARCH_FAILED": ["src/planning/local/corridor_lattice.cpp", "src/planning/global/mission_corridor.cpp"],
    "POSITION_JUMP": ["src/data/gt/world_process.cpp", "src/main.cpp"],
    "SIM_TIME_FROZEN": ["src/network/receive/rdb_clock.cpp", "src/main.cpp"],
    "PLANNING_SLOW": ["src/planning/planning_process.cpp", "src/planning/local/corridor_lattice.cpp"],
    "CLOCK_SLOW": ["src/network/receive/rdb_clock.cpp", "src/main.cpp"],
    "STEERING_AT_LIMIT": ["src/control/lateral/lateral_controller.cpp", "src/control/control_process.cpp", "src/planning/local/path_planner.cpp"],
    "TRACKING_ERROR": ["src/planning/local/path_planner.cpp", "src/control/lateral/lateral_controller.cpp", "src/planning/local/corridor_lattice.cpp"],
}

MEANINGS = {
    "NO_FRAMES": "최종 제어 프레임 없음: 마지막 실행 단계·console부터 확인",
    "INPUT_TIMEOUT": "GT 수신 제한시간 초과 관측",
    "SEND_FAILED": "제어 TCP 송신 예외 관측",
    "RECEIVE_ONLY": "수신 전용 옵션으로 명령 송신 생략",
    "INVALID_PLAN": "최종 plan.valid=0; 매칭·경로 생성 근거 확인",
    "SAFETY_OVERRIDE": "안전 제한기가 명령을 변경함; invalid/emergency 원인 확인",
    "COMMAND_WITHOUT_MOTION": "2초 이상 거의 이동 없이 양의 목표속도·가속 명령 관측; 실제 송신 및 VTD 적용은 별도 확인",
    "ZERO_TARGET_WAIT": "2초 이상 목표속도 0 근처에서 정지 관측; 정상 신호·장애물 대기도 포함",
    "EMERGENCY_BRAKE": "최종 긴급제동 요청 관측",
    "COLLISION_LOGGED": "제어기 현재 OBB 충돌 값 관측; 미래 충돌과 구분",
    "ROUTE_SEARCH_FAILED": "회랑 전역경로 탐색 실패 상태 관측",
    "POSITION_JUMP": "World 위치 불연속 검출; 공식 리스폰 확정 아님",
    "SIM_TIME_FROZEN": "연속 수신 동안 매칭된 시뮬레이션 시간이 진행하지 않음; 정지·중복 매칭 확인",
    "PLANNING_SLOW": "계획 계산 100ms 초과 표본",
    "CLOCK_SLOW": "RDB 연결·시각 처리 구간 100ms 초과 표본",
    "STEERING_AT_LIMIT": "조향 명령이 설정 한계의 98% 이상; 지속 여부·곡률 확인",
    "TRACKING_ERROR": "샘플된 최종 로컬 경로로부터 횡방향 거리 1m 초과; 경로 다운샘플 오차 고려",
}
