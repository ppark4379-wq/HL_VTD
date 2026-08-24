#!/usr/bin/env python3
"""현재 Logger CSV의 지표를 계산하고 명시된 기준만 판정한다.

대회 최종 PASS/FAIL 기준은 아직 고정하지 않는다. 사용자가 옵션으로 전달한
기준이 없으면 결과를 NOT_EVALUATED로 표시한다.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


REQUIRED_COLUMNS = {
    "scenario_id",
    "run_id",
    "frame",
    "elapsed_wall_time_s",
    "ego_speed",
    "nearest_object_distance",
    "goal_reached",
    "command_steering",
    "target_accel",
}


def non_negative_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("0 이상의 유한한 숫자여야 합니다")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Logger CSV 지표 요약 및 명시적 기준 기반 PASS/FAIL 판정"
    )
    parser.add_argument("log", type=Path, help="Logger가 생성한 실행 CSV")
    parser.add_argument("--summary", type=Path, help="실행 요약 CSV 저장 경로")
    parser.add_argument(
        "--require-goal",
        action="store_true",
        help="한 프레임 이상 goal_reached=1이어야 통과",
    )
    parser.add_argument(
        "--min-object-distance",
        type=non_negative_float,
        metavar="M",
        help="최근접 객체와의 최소 허용 거리(m)",
    )
    parser.add_argument(
        "--max-abs-steering",
        type=non_negative_float,
        metavar="RAD",
        help="조향 명령 절댓값의 허용 최댓값(rad)",
    )
    parser.add_argument(
        "--max-abs-accel",
        type=non_negative_float,
        metavar="MPS2",
        help="가속도 명령 절댓값의 허용 최댓값(m/s^2)",
    )
    return parser.parse_args()


def parse_float(row: dict[str, str], field: str) -> float | None:
    raw = row.get(field, "").strip()
    if not raw:
        return None
    try:
        value = float(raw)
    except ValueError as error:
        raise ValueError(f"{field}에 숫자가 아닌 값이 있습니다: {raw!r}") from error
    return value if math.isfinite(value) else None


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or [])
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError("필수 열이 없습니다: " + ", ".join(missing))
        rows = list(reader)
    if not rows:
        raise ValueError("로그에 데이터 행이 없습니다")
    return rows


def finite_values(rows: list[dict[str, str]], field: str) -> list[float]:
    values = []
    for row in rows:
        value = parse_float(row, field)
        if value is not None:
            values.append(value)
    return values


def summarize(rows: list[dict[str, str]], args: argparse.Namespace) -> dict[str, str]:
    times = finite_values(rows, "elapsed_wall_time_s")
    speeds = finite_values(rows, "ego_speed")
    distances = finite_values(rows, "nearest_object_distance")
    steering = finite_values(rows, "command_steering")
    acceleration = finite_values(rows, "target_accel")
    goal_reached = any(row["goal_reached"].strip() == "1" for row in rows)

    rules: list[tuple[bool, str]] = []
    criteria = []
    if args.require_goal:
        rules.append((goal_reached, "goal_not_reached"))
        criteria.append("require_goal")
    if args.min_object_distance is not None:
        rules.append((
            bool(distances) and min(distances) >= args.min_object_distance,
            "object_distance_unavailable"
            if not distances
            else "minimum_object_distance_below_limit",
        ))
        criteria.append(f"min_object_distance>={args.min_object_distance}")
    if args.max_abs_steering is not None:
        rules.append((
            bool(steering) and max(map(abs, steering)) <= args.max_abs_steering,
            "steering_unavailable"
            if not steering
            else "steering_limit_exceeded",
        ))
        criteria.append(f"max_abs_steering<={args.max_abs_steering}")
    if args.max_abs_accel is not None:
        rules.append((
            bool(acceleration)
            and max(map(abs, acceleration)) <= args.max_abs_accel,
            "acceleration_unavailable"
            if not acceleration
            else "acceleration_limit_exceeded",
        ))
        criteria.append(f"max_abs_accel<={args.max_abs_accel}")

    failures = [reason for passed, reason in rules if not passed]
    result = "NOT_EVALUATED" if not rules else ("FAIL" if failures else "PASS")

    return {
        "scenario_id": rows[0]["scenario_id"],
        "run_id": rows[0]["run_id"],
        "result": result,
        "failures": ";".join(failures),
        "criteria": ";".join(criteria),
        "frame_count": str(len(rows)),
        "duration_s": str(max(times) if times else ""),
        "goal_reached": "1" if goal_reached else "0",
        "max_ego_speed": str(max(speeds) if speeds else ""),
        "min_object_distance": str(min(distances) if distances else ""),
        "max_abs_steering": str(max(map(abs, steering)) if steering else ""),
        "max_abs_accel": str(max(map(abs, acceleration)) if acceleration else ""),
    }


def write_summary(path: Path, summary: dict[str, str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary))
        writer.writeheader()
        writer.writerow(summary)


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
