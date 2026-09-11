"""Find debugging symptoms, preserving uncertainty and original frame references."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
from typing import Iterator

from contracts import (
    Incident, ScanResult, NO_MOTION_SECONDS, NO_MOTION_DISTANCE_M,
    PLANNING_SLOW_MS, SAMPLE_TIME_GAP_S,
)


def finite(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def reject_nonfinite_constant(value: str) -> None:
    raise ValueError(f"invalid JSON numeric constant: {value}")


def records(path: Path) -> Iterator[tuple[int, dict | None, str | None]]:
    if not path.is_file():
        yield 0, None, "DIAGNOSTICS_MISSING"
        return
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line_no, line in enumerate(stream, 1):
            try:
                value = json.loads(line, parse_constant=reject_nonfinite_constant)
                if not isinstance(value, dict) or value.get("schema") != 1:
                    yield line_no, None, "UNSUPPORTED_RECORD"
                else:
                    value["_line"] = line_no
                    yield line_no, value, None
            except (ValueError, TypeError):
                yield line_no, None, "TRUNCATED_OR_INVALID_JSON"


def compact_frame(row: dict) -> dict:
    result = {key: row.get(key) for key in (
        "frame", "_line", "wall_s", "sim_s", "process_wall_s", "dt", "receive_only",
        "receive_wait_ms", "clock_ms", "planning_ms", "dropped_gt_total",
        "ego", "planning", "lane_change", "mission", "corridor", "control",
    )}
    result["tracking"] = {key: value for key, value in (row.get("tracking") or {}).items()
                          if key != "points_index_x_y_yaw_s_road_section_lane"}
    return result


def record_incident(result: ScanResult, active: dict[str, Incident], row: dict, code: str) -> None:
    frame = int(row.get("frame", 0))
    wall = finite(row.get("process_wall_s")) or 0.0
    result.counts[code] = result.counts.get(code, 0) + 1
    if code in active:
        active[code].last_frame = frame
        active[code].last_wall_s = wall
        return
    incident = Incident(code, frame, frame, wall, wall, compact_frame(row))
    incident.evidence["detail"] = row.get("detail")
    active[code] = incident
    # Keep first and recent episodes, not unbounded events for every control tick.
    same = [item for item in result.incidents if item.code == code]
    if len(same) >= 3:
        result.incidents.remove(same[1])
    result.incidents.append(incident)


def distance_to_segment(x: float, y: float, start: list, end: list) -> float:
    ax, ay, bx, by = float(start[1]), float(start[2]), float(end[1]), float(end[2])
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / denom)) if denom else 0.0
    return math.hypot(x - ax - t * dx, y - ay - t * dy)


def sampled_path_distance(row: dict) -> float | None:
    ego = row.get("ego", {})
    x, y = finite(ego.get("x")), finite(ego.get("y"))
    points = row.get("tracking", {}).get("points_index_x_y_yaw_s_road_section_lane", [])
    if x is None or y is None or not isinstance(points, list) or len(points) < 2:
        return None
    if any(not isinstance(p, list) or len(p) < 3 or finite(p[1]) is None or finite(p[2]) is None for p in points):
        return None
    return min(distance_to_segment(x, y, a, b) for a, b in zip(points, points[1:]))


def immediate_symptoms(row: dict) -> list[str]:
    codes = []
    planning = row.get("planning", {})
    control = row.get("control", {})
    corridor = row.get("corridor") or {}
    if row.get("receive_only") == 1: codes.append("RECEIVE_ONLY")
    if planning.get("valid") == 0: codes.append("INVALID_PLAN")
    if control.get("overridden") == 1: codes.append("SAFETY_OVERRIDE")
    if planning.get("emergency") == 1: codes.append("EMERGENCY_BRAKE")
    if planning.get("collision") == 1: codes.append("COLLISION_LOGGED")
    if corridor.get("search_failed") == 1: codes.append("ROUTE_SEARCH_FAILED")
    if row.get("ego", {}).get("position_jump") == 1: codes.append("POSITION_JUMP")
    if (finite(row.get("planning_ms")) or 0.0) > PLANNING_SLOW_MS: codes.append("PLANNING_SLOW")
    if (finite(row.get("clock_ms")) or 0.0) > PLANNING_SLOW_MS: codes.append("CLOCK_SLOW")
    steer = finite(control.get("safe", {}).get("steering_rad"))
    limit = finite(control.get("max_steering"))
    if steer is not None and limit is not None and limit > 0 and abs(steer) >= 0.98 * limit:
        codes.append("STEERING_AT_LIMIT")
    distance = sampled_path_distance(row)
    if distance is not None and distance > 1.0:
        row["tracking"]["sampled_path_distance_m"] = distance
        codes.append("TRACKING_ERROR")
    return codes


def update_motion_window(window: deque, row: dict) -> None:
    now = finite(row.get("process_wall_s"))
    if now is None:
        window.clear()
        return
    if window:
        previous = finite(window[-1].get("process_wall_s"))
        if previous is None or not 0 <= now - previous <= SAMPLE_TIME_GAP_S:
            window.clear()
    window.append(row)
    while len(window) > 2 and now - window[1]["process_wall_s"] >= NO_MOTION_SECONDS:
        window.popleft()


def motion_symptoms(window: deque) -> list[str]:
    if len(window) < 2:
        return []
    first, last = window[0], window[-1]
    if last["process_wall_s"] - first["process_wall_s"] < NO_MOTION_SECONDS:
        return []
    positions = [(finite(row.get("ego", {}).get("x")), finite(row.get("ego", {}).get("y"))) for row in window]
    if any(x is None or y is None for x, y in positions):
        return []
    x0, y0 = positions[0]
    if any(math.hypot(x - x0, y - y0) > NO_MOTION_DISTANCE_M for x, y in positions):
        return []
    sims = [finite(row.get("sim_s")) for row in window]
    if all(value is not None for value in sims) and max(sims) - min(sims) < 1e-6:
        return ["SIM_TIME_FROZEN"]
    target_speeds = [finite(row.get("planning", {}).get("target_speed")) for row in window]
    accels = [finite(row.get("control", {}).get("safe", {}).get("accel_mps2")) for row in window]
    sending_enabled = all(row.get("receive_only") != 1 for row in window)
    if sending_enabled and all(value is not None and value > 0.5 for value in target_speeds) and all(value is not None and value > 0.1 for value in accels):
        return ["COMMAND_WITHOUT_MOTION"]
    if all(value is not None and value <= 0.05 for value in target_speeds):
        return ["ZERO_TARGET_WAIT"]
    return []


def scan_frame(result: ScanResult, active: dict, window: deque, row: dict) -> None:
    result.frames += 1
    if not result.first:
        result.first = compact_frame(row)
    if result.last and row["frame"] <= result.last["frame"]:
        result.issues.append(f"FRAME_ORDER_INVALID line={row['_line']}")
        window.clear()
    result.last = compact_frame(row)
    result.goal_reported = result.goal_reported or row.get("planning", {}).get("goal") == 1
    result.maximum_planning_ms = max(result.maximum_planning_ms, finite(row.get("planning_ms")) or 0.0)
    result.maximum_receive_ms = max(result.maximum_receive_ms, finite(row.get("receive_wait_ms")) or 0.0)
    result.maximum_clock_ms = max(result.maximum_clock_ms, finite(row.get("clock_ms")) or 0.0)
    update_motion_window(window, row)
    codes = immediate_symptoms(row) + motion_symptoms(window)
    for code in codes:
        record_incident(result, active, row, code)
    for code in list(active):
        if code not in codes:
            del active[code]


def valid_frame(row: dict) -> bool:
    return (isinstance(row.get("frame"), int) and row["frame"] >= 0
            and isinstance(row.get("process_wall_s"), (float, int))
            and finite(row.get("process_wall_s")) is not None
            and all(isinstance(row.get(key), dict) for key in ("ego", "planning", "control", "tracking"))
            and isinstance(row["control"].get("safe", {}), dict)
            and (row.get("corridor") is None or isinstance(row["corridor"], dict)))


def scan_send_timing(result: ScanResult, row: dict) -> None:
    interval = finite(row.get("send_interval_ms"))
    if row.get("socket_send_completed") == 1 and interval is not None and interval > 0:
        result.send_interval_count += 1
        result.send_interval_total_ms += interval
        result.maximum_send_interval_ms = max(result.maximum_send_interval_ms, interval)
        result.send_intervals_over_55ms += interval > 55.0
    result.maximum_receive_to_send_ms = max(result.maximum_receive_to_send_ms,
                                             finite(row.get("receive_to_send_ms")) or 0.0)
    result.maximum_diagnostic_capture_ms = max(result.maximum_diagnostic_capture_ms,
                                               finite(row.get("diagnostic_capture_ms")) or 0.0)


def scan_logger_health(result: ScanResult, directory: Path) -> None:
    path = directory / "repair_health.json"
    if path.is_file():
        try:
            status = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_nonfinite_constant)
            if not isinstance(status, dict):
                raise ValueError("not an object")
            result.repair_status = status
        except (OSError, ValueError):
            result.issues.append("REPAIR_HEALTH_INVALID")
    async_mode = result.logger_status.get("mode") == "async_bounded" or bool(result.repair_status)
    if not async_mode:
        return
    for name, status in (("LOGGER", result.logger_status), ("REPAIR", result.repair_status)):
        if status.get("final") != 1:
            result.issues.append(f"{name}_NOT_DRAINED: queued tail may be missing")
        for key in ("dropped_records_total", "dropped_events_total", "limited_events_total", "truncated_events_total"):
            value = finite(status.get(key)) or 0
            if value > 0:
                result.issues.append(f"{name}_{key}={int(value)}: evidence is partial")


def scan_run(directory: Path) -> ScanResult:
    result = ScanResult()
    active = {}
    window = deque()
    pending_send = None
    for line, row, error in records(directory / "diagnostics.jsonl"):
        if error:
            result.issues.append(f"{error} line={line}")
            window.clear()
            active.clear()
            continue
        result.records += 1
        kind = row.get("kind")
        if kind == "stage":
            result.last_stage = row
            if row.get("stage") == "process_start" and "mode=async_bounded" in row.get("detail", ""):
                result.logger_status = {"mode": "async_bounded"}
            code = {"gt_timeout": "INPUT_TIMEOUT", "send_failed": "SEND_FAILED"}.get(row.get("stage"))
            if code:
                record_incident(result, active, row, code)
        elif kind == "frame":
            if not valid_frame(row):
                result.issues.append(f"FRAME_INVALID line={line}")
                window.clear()
                active.clear()
                continue
            if pending_send is not None:
                result.issues.append(f"SEND_RESULT_MISSING frame={pending_send}")
            pending_send = row["frame"]
            if result.last and row["frame"] != result.last["frame"] + 1:
                window.clear()
                active.clear()
                result.issues.append("LOGGED_FRAME_GAP: motion window restarted")
            scan_frame(result, active, window, row)
        elif kind == "send_result":
            if pending_send != row.get("frame"):
                result.issues.append(f"SEND_FRAME_MISMATCH line={line}")
            if row.get("socket_send_completed") == 1: result.send_completed += 1
            elif row.get("receive_only") == 0: result.sends_failed += 1
            else: result.sends_skipped += 1
            scan_send_timing(result, row)
            pending_send = None
        elif kind == "logger_status":
            result.logger_status = row
    if pending_send is not None:
        result.issues.append(f"SEND_RESULT_MISSING frame={pending_send}")
    if not result.frames:
        record_incident(result, {}, result.last_stage or {}, "NO_FRAMES")
    if result.last_stage.get("stage") not in {"process_end", "config_check_complete"}:
        result.issues.append("NO_CLEAN_CONTROLLER_END")
    scan_logger_health(result, directory)
    result.issues = list(dict.fromkeys(result.issues))[:40]
    return result
