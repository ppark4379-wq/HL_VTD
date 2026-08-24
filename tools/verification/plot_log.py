#!/usr/bin/env python3
"""현재 Logger CSV에서 확보되는 주요 시계열을 PNG로 그린다.

TTC, 충돌 여부와 객체 종류는 아직 입력에 없으므로 시각화하지 않는다.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


PLOT_COLUMNS = {
    "elapsed_wall_time_s",
    "ego_speed",
    "target_speed",
    "nearest_object_distance",
    "planned_steering",
    "command_steering",
    "target_accel",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Logger CSV 시계열 그래프 생성")
    parser.add_argument("log", type=Path, help="Logger가 생성한 실행 CSV")
    parser.add_argument(
        "--output",
        type=Path,
        help="PNG 저장 경로. 생략하면 입력 파일 옆에 _plot.png로 저장",
    )
    return parser.parse_args()


def number(row: dict[str, str], field: str) -> float:
    raw = row.get(field, "").strip()
    if not raw:
        return math.nan
    try:
        value = float(raw)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or [])
        missing = sorted(PLOT_COLUMNS - columns)
        if missing:
            raise ValueError("필수 열이 없습니다: " + ", ".join(missing))
        rows = list(reader)
    if not rows:
        raise ValueError("로그에 데이터 행이 없습니다")
    return rows


def main() -> int:
    args = parse_args()
    output = args.output or args.log.with_name(args.log.stem + "_plot.png")

    try:
        rows = load_rows(args.log)
    except (OSError, ValueError) as error:
        print(f"[plot_log] ERROR: {error}", file=sys.stderr)
        return 2

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "[plot_log] ERROR: matplotlib가 필요합니다 "
            "(python3 -m pip install matplotlib)",
            file=sys.stderr,
        )
        return 2

    time = [number(row, "elapsed_wall_time_s") for row in rows]
    ego_speed = [number(row, "ego_speed") for row in rows]
    target_speed = [number(row, "target_speed") for row in rows]
    distance = [number(row, "nearest_object_distance") for row in rows]
    planned_steering = [number(row, "planned_steering") for row in rows]
    command_steering = [number(row, "command_steering") for row in rows]
    target_accel = [number(row, "target_accel") for row in rows]

    figure, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    axes[0].plot(time, ego_speed, label="ego speed")
    axes[0].plot(time, target_speed, label="target speed")
    axes[0].set_ylabel("speed [m/s]")
    axes[0].legend()

    axes[1].plot(time, distance, color="tab:orange", label="nearest object")
    axes[1].set_ylabel("distance [m]")
    axes[1].legend()

    axes[2].plot(time, planned_steering, label="planned steering")
    axes[2].plot(time, command_steering, label="command steering")
    axes[2].set_ylabel("steering [rad]")
    axes[2].legend()

    axes[3].plot(time, target_accel, color="tab:red", label="target accel")
    axes[3].set_ylabel("accel [m/s²]")
    axes[3].set_xlabel("elapsed wall time [s]")
    axes[3].legend()

    scenario_id = rows[0].get("scenario_id", "unspecified")
    run_id = rows[0].get("run_id", "unspecified")
    figure.suptitle(f"{scenario_id} / {run_id}")
    figure.tight_layout()
    try:
        figure.savefig(output, dpi=150)
    except OSError as error:
        print(f"[plot_log] ERROR: {error}", file=sys.stderr)
        return 2
    finally:
        plt.close(figure)

    print(f"[plot_log] saved: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
