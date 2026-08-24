#!/usr/bin/env python3
"""evaluate.py가 만든 여러 실행 요약 CSV를 하나로 집계한다."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


REQUIRED_COLUMNS = {"scenario_id", "run_id", "result", "failures"}
VALID_RESULTS = {"PASS", "FAIL", "NOT_EVALUATED"}
METRIC_COLUMNS = {
    "duration_s",
    "max_ego_speed",
    "min_object_distance",
    "max_abs_steering",
    "max_abs_accel",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="여러 시나리오 실행의 PASS/FAIL 및 주요 수치 집계"
    )
    parser.add_argument(
        "summaries",
        type=Path,
        nargs="+",
        help="evaluate.py가 생성한 summary CSV 파일들",
    )
    parser.add_argument("--output", type=Path, help="전체 요약 CSV 저장 경로")
    return parser.parse_args()


def load_summaries(paths: list[Path]) -> list[dict[str, str]]:
    rows = []
    for path in paths:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            columns = set(reader.fieldnames or [])
            missing = sorted(REQUIRED_COLUMNS - columns)
            if missing:
                raise ValueError(
                    f"{path}: 필수 열이 없습니다: " + ", ".join(missing)
                )
            for row in reader:
                result = row["result"].strip()
                if result not in VALID_RESULTS:
                    raise ValueError(f"{path}: 알 수 없는 result 값: {result!r}")
                rows.append(row)
    if not rows:
        raise ValueError("요약할 실행 결과가 없습니다")
    return rows


def metric_values(rows: list[dict[str, str]], field: str) -> list[float]:
    values = []
    for row in rows:
        raw = row.get(field, "").strip()
        if not raw:
            continue
        try:
            value = float(raw)
        except ValueError as error:
            raise ValueError(f"{field}에 숫자가 아닌 값이 있습니다: {raw!r}") from error
        if math.isfinite(value):
            values.append(value)
    return values


def aggregate(rows: list[dict[str, str]]) -> dict[str, str]:
    passed = [row for row in rows if row["result"].strip() == "PASS"]
    failed = [row for row in rows if row["result"].strip() == "FAIL"]
    not_evaluated = [
        row for row in rows if row["result"].strip() == "NOT_EVALUATED"
    ]

    failed_scenarios = sorted({
        row["scenario_id"].strip() for row in failed if row["scenario_id"].strip()
    })
    failed_runs = [
        f"{row['scenario_id'].strip()}/{row['run_id'].strip()}"
        for row in failed
    ]
    failure_reasons = sorted({
        reason
        for row in failed
        for reason in row.get("failures", "").split(";")
        if reason
    })

    durations = metric_values(rows, "duration_s")
    speeds = metric_values(rows, "max_ego_speed")
    distances = metric_values(rows, "min_object_distance")
    steering = metric_values(rows, "max_abs_steering")
    acceleration = metric_values(rows, "max_abs_accel")

    return {
        "total_runs": str(len(rows)),
        "pass_count": str(len(passed)),
        "fail_count": str(len(failed)),
        "not_evaluated_count": str(len(not_evaluated)),
        "failed_scenarios": ";".join(failed_scenarios),
        "failed_runs": ";".join(failed_runs),
        "failure_reasons": ";".join(failure_reasons),
        "total_duration_s": str(sum(durations) if durations else ""),
        "max_ego_speed": str(max(speeds) if speeds else ""),
        "min_object_distance": str(min(distances) if distances else ""),
        "max_abs_steering": str(max(steering) if steering else ""),
        "max_abs_accel": str(max(acceleration) if acceleration else ""),
    }


def write_aggregate(path: Path, summary: dict[str, str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary))
        writer.writeheader()
        writer.writerow(summary)


def main() -> int:
    args = parse_args()
    try:
        rows = load_summaries(args.summaries)
        summary = aggregate(rows)
        if args.output:
            write_aggregate(args.output, summary)
    except (OSError, ValueError) as error:
        print(f"[summarize_results] ERROR: {error}", file=sys.stderr)
        return 2

    for key, value in summary.items():
        print(f"{key}: {value}")
    return 1 if int(summary["fail_count"]) > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
