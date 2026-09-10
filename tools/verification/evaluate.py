#!/usr/bin/env python3
"""HL_VTD 실행 로그 요약과 2026 HL FMA 공식 기준의 가용 항목 평가."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


REQUIRED_COLUMNS = {
    "scenario_id", "run_id", "frame", "elapsed_wall_time_s", "ego_speed",
    "nearest_object_distance", "goal_reached", "command_steering", "target_accel",
}

OFFICIAL_ITEMS = {
    1: "speed_limit",
    2: "school_zone_speed",
    4: "centerline_right_hand_traffic",
    5: "sidewalk_intrusion",
    6: "respawn",
    7: "lane_change_turn_signal",
    8: "solid_line_change",
    9: "red_light_stop",
    10: "green_light_pass",
    11: "flashing_red_stop",
    12: "crosswalk_no_stop",
    13: "static_obstacle_collision",
    14: "pedestrian_response",
    15: "road_user_collision",
}


@dataclass
class OfficialItemResult:
    number: int
    status: str
    penalty: int = 0
    detail: str = ""
    severity_by_section: dict[str, int] = None
    penalty_by_section: dict[str, int] = None

    def __post_init__(self) -> None:
        if self.severity_by_section is None:
            self.severity_by_section = {}
        if self.penalty_by_section is None:
            self.penalty_by_section = {
                key: 3 if level == 1 else 6
                for key, level in self.severity_by_section.items()
            }


def non_negative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("0 이상의 유한한 숫자여야 합니다")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Logger CSV 지표 요약 및 2026 HL FMA 가용 항목 평가"
    )
    parser.add_argument("log", type=Path, help="Logger가 생성한 실행 CSV")
    parser.add_argument("--summary", type=Path, help="실행 요약 CSV 저장 경로")
    parser.add_argument(
        "--official", action="store_true",
        help="공식 15개 항목 중 로그 데이터가 있는 항목을 구간별 평가",
    )
    parser.add_argument(
        "--speed-limit-kmh", type=non_negative_float,
        help="posted_speed_limit_kmh가 없을 때 적용할 법정 제한속도",
    )
    parser.add_argument(
        "--school-zone-limit-kmh", type=non_negative_float,
        help="보호구역 제한속도. school_zone_active 로그가 함께 필요",
    )
    parser.add_argument("--require-goal", action="store_true")
    parser.add_argument("--min-object-distance", type=non_negative_float, metavar="M")
    parser.add_argument("--max-abs-steering", type=non_negative_float, metavar="RAD")
    parser.add_argument("--max-abs-accel", type=non_negative_float, metavar="MPS2")
    return parser.parse_args()


def parse_float(row: dict[str, str], field: str) -> float | None:
    raw_value = row.get(field, "")
    raw = "" if raw_value is None else str(raw_value).strip()
    if not raw:
        return None
    try:
        value = float(raw)
    except ValueError as error:
        raise ValueError(f"{field}에 숫자가 아닌 값이 있습니다: {raw!r}") from error
    return value if math.isfinite(value) else None


def parse_bool(row: dict[str, str], field: str) -> bool | None:
    raw_value = row.get(field, "")
    raw = "" if raw_value is None else str(raw_value).strip().lower()
    if not raw:
        return None
    if raw in {"1", "true", "yes"}:
        return True
    if raw in {"0", "false", "no"}:
        return False
    raise ValueError(f"{field}에 boolean이 아닌 값이 있습니다: {raw!r}")


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or [])
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError("필수 열이 없습니다: " + ", ".join(missing))
        rows = list(reader)
    incomplete = [
        index for index, row in enumerate(rows)
        if None in row or any(value is None for value in row.values())
    ]
    if incomplete == [len(rows)-1]:
        # The dashboard may read while the C++ CSV logger is between writes.
        # A single incomplete physical tail is not evidence and must neither
        # crash the request nor become a synthetic all-empty assessment row.
        rows.pop()
    elif incomplete:
        raise ValueError("CSV 중간에 잘린 데이터 행이 있습니다")
    if not rows:
        raise ValueError("로그에 데이터 행이 없습니다")
    return rows


def finite_values(rows: list[dict[str, str]], field: str) -> list[float]:
    return [value for row in rows if (value := parse_float(row, field)) is not None]


def section_key(row: dict[str, str]) -> str:
    value = row.get("evaluation_section_id", "")
    return ("" if value is None else str(value).strip()) or "unsegmented"


def group_by_section(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[section_key(row)].append(row)
    return dict(grouped)


def severity_result(number: int, severity_by_section: dict[str, int], detail: str) -> OfficialItemResult:
    penalty = sum(3 if severity == 1 else 6 for severity in severity_by_section.values())
    if not severity_by_section:
        return OfficialItemResult(number, "PASS", 0, detail, dict(severity_by_section))
    maximum = max(severity_by_section.values())
    status = "MAJOR" if maximum == 2 else "MINOR"
    sections = ",".join(sorted(severity_by_section))
    return OfficialItemResult(number, status, -penalty, f"sections={sections};{detail}", dict(severity_by_section))


def evaluate_speed_limit(
    rows: list[dict[str, str]], fallback_limit: float | None
) -> OfficialItemResult:
    severity: dict[str, int] = {}
    samples = 0
    maximum_overspeed = 0.0
    for row in rows:
        speed = parse_float(row, "ego_speed")
        # The controller's historical ``current_speed_limit_kmh`` column is a
        # behavioural planner cap.  It also contains curve, gateway and
        # crosswalk approach speeds, so treating it as a posted road limit can
        # manufacture an overspeed violation while the car is legally below
        # the road limit.  Only consume it when the producer explicitly marks
        # the value as regulatory; new evidence should prefer the unambiguous
        # posted_speed_limit_kmh column.
        limit = parse_float(row, "posted_speed_limit_kmh")
        source = row.get("speed_limit_source", "").strip().lower()
        if limit is None and source in {
            "assessment_rule", "map_regulatory", "posted", "regulatory"
        }:
            limit = parse_float(row, "current_speed_limit_kmh")
        if limit is None:
            limit = fallback_limit
        if speed is None or limit is None:
            continue
        samples += 1
        overspeed = speed * 3.6 - limit
        maximum_overspeed = max(maximum_overspeed, overspeed)
        level = 2 if overspeed > 20.0 else (1 if overspeed > 1.0 else 0)
        if level:
            key = section_key(row)
            severity[key] = max(severity.get(key, 0), level)
    if samples == 0:
        return OfficialItemResult(1, "UNAVAILABLE", detail="speed limit unavailable")
    return severity_result(1, severity, f"max_overspeed_kmh={maximum_overspeed:.6g}")


def evaluate_school_zone(
    rows: list[dict[str, str]], school_limit: float | None
) -> OfficialItemResult:
    severity: dict[str, int] = {}
    samples = 0
    maximum_overspeed = 0.0
    for row in rows:
        active = parse_bool(row, "school_zone_active")
        if active is not True:
            continue
        speed = parse_float(row, "ego_speed")
        limit = school_limit if school_limit is not None else 30.0
        if speed is None or limit is None:
            continue
        samples += 1
        overspeed = speed * 3.6 - limit
        maximum_overspeed = max(maximum_overspeed, overspeed)
        level = 2 if overspeed > 5.0 else (1 if overspeed > 1.0 else 0)
        if level:
            key = section_key(row)
            severity[key] = max(severity.get(key, 0), level)
    if samples == 0 and any(parse_bool(row, "school_zone_active") is False for row in rows):
        return severity_result(2, {}, "outside known school zones")
    if samples == 0:
        return OfficialItemResult(2, "UNAVAILABLE", detail="school zone/limit unavailable")
    return severity_result(2, severity, f"max_overspeed_kmh={maximum_overspeed:.6g}")


def evaluate_sustained_intrusion(
    rows: list[dict[str, str]], number: int, field: str,
    distance_threshold: float, duration_threshold: float,
) -> OfficialItemResult:
    if not any(parse_float(row, field) is not None for row in rows):
        return OfficialItemResult(number, "UNAVAILABLE", detail=f"{field} unavailable")
    severity: dict[str, int] = {}
    duration = 0.0
    previous_active = False
    for row, dt in timed_rows(rows):
        key = section_key(row)
        value = parse_float(row, field)
        active = value is not None and value >= distance_threshold
        if active:
            duration = duration + dt if previous_active and dt > 0 else 0.0
            if duration + 1e-9 >= duration_threshold:
                severity[key] = 2
        else:
            duration = 0.0
        previous_active = active
    return severity_result(number, severity, f"threshold={distance_threshold}m/{duration_threshold}s")


def count_rising_edges(rows: list[dict[str, str]], field: str) -> dict[str, int] | None:
    if not any(row.get(field, "").strip() for row in rows):
        return None
    counts: dict[str, int] = defaultdict(int)
    previous: dict[str, bool] = defaultdict(bool)
    for row in rows:
        key = section_key(row)
        active = parse_bool(row, field) is True
        if active and not previous[key]:
            counts[key] += 1
        previous[key] = active
    return dict(counts)


def evaluate_event_count(rows: list[dict[str, str]], number: int, field: str) -> OfficialItemResult:
    counts = count_rising_edges(rows, field)
    if counts is None:
        return OfficialItemResult(number, "UNAVAILABLE", detail=f"{field} unavailable")
    severity = {key: 2 if count >= 2 else 1 for key, count in counts.items() if count}
    return severity_result(number, severity, f"events={sum(counts.values())}")


def evaluate_crosswalk_stop(rows: list[dict[str, str]]) -> OfficialItemResult:
    accepted_sources = {
        "map_polygon_rear_axle",
        "scenario_metadata_rear_axle",
        "rdb_rear_axle_polygon",
    }
    if not any(
        row.get("crosswalk_active", "").strip()
        and row.get("crosswalk_source", "").strip().lower() in accepted_sources
        for row in rows
    ):
        return OfficialItemResult(
            12,
            "UNAVAILABLE",
            detail="rear-axle crosswalk occupancy unavailable",
        )
    severity: dict[str, int] = {}
    duration = 0.0
    previous_active = False
    for row, dt in timed_rows(rows):
        key = section_key(row)
        speed = parse_float(row, "ego_speed")
        trusted = row.get("crosswalk_source", "").strip().lower() in accepted_sources
        active = trusted and parse_bool(row, "crosswalk_active") is True and speed is not None and abs(speed) * 3.6 <= 1.0
        if active:
            duration = duration + dt if previous_active and dt > 0 else 0.0
            if duration + 1e-9 >= 3.0:
                severity[key] = 1
        else:
            duration = 0.0
        previous_active = active
    return severity_result(12, severity, "speed<=1km/h for >=3s")


def evaluate_collision(rows: list[dict[str, str]], scenario_id: str) -> list[OfficialItemResult]:
    if any("road_user_collision" in row or "static_collision" in row for row in rows):
        return [evaluate_boolean(rows, 13, "static_collision", 1),
                evaluate_boolean(rows, 15, "road_user_collision", 2)]
    edges = count_rising_edges(rows, "collision")
    obstacle = OfficialItemResult(13, "UNAVAILABLE", detail="collision unavailable")
    road_user = OfficialItemResult(15, "UNAVAILABLE", detail="collision unavailable")
    if edges is None:
        return [obstacle, road_user]
    event_sections = {key: count for key, count in edges.items() if count}
    if any(parse_bool(row, "collision") is True for row in rows):
        # Scenario names do not identify the object involved in a collision.
        return [OfficialItemResult(13, "UNAVAILABLE", detail="collision object type missing"),
                OfficialItemResult(15, "UNAVAILABLE", detail="collision object type missing")]
    if scenario_id.startswith("O"):
        obstacle = severity_result(13, {key: 1 for key in event_sections}, "obstacle collision")
        road_user = OfficialItemResult(15, "NOT_APPLICABLE", detail="obstacle scenario")
    elif scenario_id.startswith(("V", "P")):
        obstacle = OfficialItemResult(13, "NOT_APPLICABLE", detail="non-obstacle scenario")
        road_user = severity_result(15, {key: 2 for key in event_sections}, "road-user collision")
    else:
        obstacle = OfficialItemResult(13, "NOT_APPLICABLE")
        road_user = OfficialItemResult(15, "NOT_APPLICABLE")
    return [obstacle, road_user]


def evaluate_respawn(rows: list[dict[str, str]]) -> OfficialItemResult:
    values = [(section_key(row), parse_float(row, "respawn_count")) for row in rows]
    available = [(key, value) for key, value in values if value is not None]
    if not available:
        return OfficialItemResult(6, "UNAVAILABLE", detail="confirmed respawn event unavailable")
    maximum: dict[str, int] = defaultdict(int)
    for key, value in available:
        maximum[key] = max(maximum[key], int(value))
    excess = {key: count - 1 for key, count in maximum.items() if count > 1}
    penalty = -6 * sum(excess.values())
    return OfficialItemResult(
        6, "MAJOR" if excess else "PASS", penalty,
        "excess_respawns=" + str(sum(excess.values())),
        {key: 2 for key in excess},
        {key: count * 6 for key, count in excess.items()},
    )


def timed_rows(rows):
    """Only consecutive simulation samples contribute to a duration."""
    previous = None
    previous_section = None
    for row in rows:
        now = parse_float(row, "sim_time_s")
        if now is None:
            now = parse_float(row, "elapsed_wall_time_s")
        gap = parse_float(row, "max_sample_gap_s") or 0.5
        dt = now - previous if now is not None and previous is not None else 0.0
        valid = row.get("data_valid", "1") != "0" and row.get("paused", "0") != "1"
        if not valid or not 0 < dt <= gap or previous_section != section_key(row):
            dt = 0.0
        yield row, dt
        previous = now if valid else None
        previous_section = section_key(row)


def evaluate_boolean(rows, number, field, level):
    values = [(section_key(row), parse_bool(row, field)) for row in rows]
    if not any(value is not None for _, value in values):
        return OfficialItemResult(number, "UNAVAILABLE", detail=f"{field} unavailable")
    return severity_result(number, {key: level for key, value in values if value is True}, field)


def evaluate_signals(rows):
    severities = {9: {}, 10: {}, 11: {}}
    available = {9: False, 10: False, 11: False}
    stop_duration = green_duration = 0.0
    stopped_before = green_before = False
    previous_id = previous_distance = None
    flashing_stop_quality = 0  # 0=no stop, 1=stopped >=2m away, 2=stopped <2m away
    for row, dt in timed_rows(rows):
        signal = row.get("traffic_light_state", "").upper()
        signal_id = row.get("stop_line_id", "")
        distance = parse_float(row, "stop_line_distance_m")
        speed = parse_float(row, "ego_speed")
        if not signal_id or distance is None or speed is None or signal not in {"RED", "GREEN", "YELLOW", "FLASHING_RED"}:
            stop_duration = green_duration = 0.0
            stopped_before = green_before = False
            previous_id = previous_distance = None
            flashing_stop_quality = 0
            continue
        if previous_id != signal_id or dt == 0:
            stop_duration = green_duration = 0.0
            stopped_before = green_before = False
            flashing_stop_quality = 0
            previous_distance = None
        available[9] = available[11] = True
        stopped = abs(speed) <= (parse_float(row, "stop_speed_kmh") or 0.0) / 3.6
        in_approach = 0 <= distance <= 30
        stopped_here = stopped and distance >= 0
        stop_duration = stop_duration + dt if stopped_here and stopped_before else 0.0
        key = section_key(row)
        if signal == "FLASHING_RED" and stop_duration + 1e-9 >= 0.5:
            if 0 <= distance < 2:
                flashing_stop_quality = 2
            elif distance >= 2:
                flashing_stop_quality = max(flashing_stop_quality, 1)
        crossed = previous_distance is not None and previous_distance >= 0 and distance < 0
        if signal == "RED":
            if crossed:
                severities[9][key] = 2
            elif distance >= 2 and stop_duration + 1e-9 >= 0.5:
                severities[9][key] = max(severities[9].get(key, 0), 1)
        if signal == "FLASHING_RED" and crossed:
            if flashing_stop_quality == 0:
                severities[11][key] = 2
            elif flashing_stop_quality == 1:
                severities[11][key] = max(severities[11].get(key, 0), 1)
        justified = parse_bool(row, "justified_stop")
        if justified is not None:
            available[10] = True
        green_now = signal == "GREEN" and stopped and in_approach and justified is False
        green_duration = green_duration + dt if green_now and green_before else 0.0
        if green_duration + 1e-9 >= 5:
            level = 2 if green_duration + 1e-9 >= 10 else 1
            severities[10][key] = max(severities[10].get(key, 0), level)
        stopped_before, green_before = stopped_here, green_now
        previous_id, previous_distance = signal_id, distance
    return tuple(severity_result(n, severities[n], "front bumper / simulation time")
                 if available[n] else OfficialItemResult(n, "UNAVAILABLE", detail="signal mapping or stop reason unavailable")
                 for n in (9, 10, 11))


def evaluate_official_for(
    rows: list[dict[str, str]],
    speed_limit_kmh: float | None = None,
    school_zone_limit_kmh: float | None = None,
) -> list[OfficialItemResult]:
    """Reusable entry point for callers (e.g. the dashboard) that don't have an argparse Namespace."""
    args = argparse.Namespace(speed_limit_kmh=speed_limit_kmh, school_zone_limit_kmh=school_zone_limit_kmh)
    return evaluate_official(rows, args)


def evaluate_official(rows: list[dict[str, str]], args: argparse.Namespace) -> list[OfficialItemResult]:
    results = {number: OfficialItemResult(number, "UNAVAILABLE") for number in OFFICIAL_ITEMS}
    results[1] = evaluate_speed_limit(rows, args.speed_limit_kmh if args.speed_limit_kmh is not None else 50.0)
    results[2] = evaluate_school_zone(rows, args.school_zone_limit_kmh)
    results[4] = evaluate_sustained_intrusion(rows, 4, "centerline_intrusion_m", 0.6, 0.6)
    results[5] = evaluate_sustained_intrusion(rows, 5, "sidewalk_intrusion_m", 0.5, 0.3)
    results[6] = evaluate_respawn(rows)
    results[7] = evaluate_boolean(rows, 7, "turn_signal_violation", 1)
    results[8] = evaluate_event_count(rows, 8, "solid_line_crossing")
    results[9], results[10], results[11] = evaluate_signals(rows)
    results[12] = evaluate_crosswalk_stop(rows)
    results[14] = evaluate_boolean(rows, 14, "pedestrian_violation", 2)
    obstacle, road_user = evaluate_collision(rows, rows[0]["scenario_id"])
    results[13] = obstacle
    results[15] = road_user
    return [results[number] for number in sorted(results)]


def summarize(rows: list[dict[str, str]], args: argparse.Namespace) -> dict[str, str]:
    times = finite_values(rows, "elapsed_wall_time_s")
    speeds = finite_values(rows, "ego_speed")
    distances = finite_values(rows, "nearest_object_distance")
    steering = finite_values(rows, "command_steering")
    acceleration = finite_values(rows, "target_accel")
    ttc = finite_values(rows, "nearest_object_ttc_s")
    path_d = finite_values(rows, "path_d_m")
    goal_reached = any(row["goal_reached"].strip() == "1" for row in rows)

    rules: list[tuple[bool, str]] = []
    criteria = []
    if args.require_goal:
        rules.append((goal_reached, "goal_not_reached")); criteria.append("require_goal")
    if args.min_object_distance is not None:
        rules.append((bool(distances) and min(distances) >= args.min_object_distance,
                      "object_distance_unavailable" if not distances else "minimum_object_distance_below_limit"))
        criteria.append(f"min_object_distance>={args.min_object_distance}")
    if args.max_abs_steering is not None:
        rules.append((bool(steering) and max(map(abs, steering)) <= args.max_abs_steering,
                      "steering_unavailable" if not steering else "steering_limit_exceeded"))
        criteria.append(f"max_abs_steering<={args.max_abs_steering}")
    if args.max_abs_accel is not None:
        rules.append((bool(acceleration) and max(map(abs, acceleration)) <= args.max_abs_accel,
                      "acceleration_unavailable" if not acceleration else "acceleration_limit_exceeded"))
        criteria.append(f"max_abs_accel<={args.max_abs_accel}")

    failures = [reason for passed, reason in rules if not passed]
    result = "NOT_EVALUATED" if not rules else ("FAIL" if failures else "PASS")
    summary = {
        "scenario_id": rows[0]["scenario_id"], "run_id": rows[0]["run_id"],
        "result": result, "failures": ";".join(failures), "criteria": ";".join(criteria),
        "frame_count": str(len(rows)), "duration_s": str(max(times) if times else ""),
        "goal_reached": "1" if goal_reached else "0",
        "max_ego_speed": str(max(speeds) if speeds else ""),
        "min_object_distance": str(min(distances) if distances else ""),
        "min_ttc_s": str(min(ttc) if ttc else ""),
        "max_abs_path_d_m": str(max(map(abs, path_d)) if path_d else ""),
        "max_abs_steering": str(max(map(abs, steering)) if steering else ""),
        "max_abs_accel": str(max(map(abs, acceleration)) if acceleration else ""),
    }
    if args.official:
        official = evaluate_official(rows, args)
        available = [item for item in official if item.status not in {"UNAVAILABLE", "NOT_APPLICABLE"}]
        unavailable = [OFFICIAL_ITEMS[item.number] for item in official if item.status == "UNAVAILABLE"]
        violations = [f"{item.number}:{item.status}" for item in official if item.status in {"MINOR", "MAJOR"}]
        has_section_ids = all(row.get("evaluation_section_id", "").strip() for row in rows)
        penalty = -sum(
            points
            for item in available
            for section, points in item.penalty_by_section.items()
            if section != "unsegmented"
        )
        complete = not unavailable and has_section_ids
        summary.update({
            "official_evaluation": "COMPLETE" if complete else "PARTIAL",
            "major_count": str(sum(points//6 if item.number==6 else int(item.severity_by_section.get(section)==2)
                                   for item in official for section,points in item.penalty_by_section.items()
                                   if section != "unsegmented")),
            "official_base_score": "500",
            "official_penalty": str(penalty),
            "official_score": str(500 + penalty) if complete else None,
            "evidence_only_score": str(500 + sum(item.penalty for item in available)),
            "official_violations": ";".join(violations),
            "official_unavailable_items": ";".join(unavailable),
            "official_item_status": ";".join(
                f"{item.number}:{OFFICIAL_ITEMS[item.number]}={item.status}" for item in official
            ),
        })
    return summary


def write_summary(path: Path, summary: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary))
        writer.writeheader(); writer.writerow(summary)


def main() -> int:
    args = parse_args()
    try:
        rows = load_rows(args.log)
        summary = summarize(rows, args)
        if args.summary:
            write_summary(args.summary, summary)
    except (OSError, ValueError) as error:
        print(f"[evaluate] ERROR: {error}", file=sys.stderr)
        return 2
    for key, value in summary.items():
        print(f"{key}: {value}")
    return 1 if summary["result"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
