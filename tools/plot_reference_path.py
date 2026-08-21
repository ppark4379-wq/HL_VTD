#!/usr/bin/env python3
"""외부 패키지 없이 reference path와 checkpoint를 SVG로 시각화한다."""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

def read_xy(csv_path: Path) -> list[tuple[float, float]]:
    """x/y 헤더가 있는 CSV에서 월드 좌표를 읽는다."""
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        rows = csv.DictReader(stream)
        if not rows.fieldnames or "x" not in rows.fieldnames or "y" not in rows.fieldnames:
            raise ValueError(f"{csv_path}: CSV header must contain x,y")
        return [(float(row["x"]), float(row["y"])) for row in rows]


def write_svg(path_points: list[tuple[float, float]],
              checkpoints: list[tuple[float, float]],
              lane_segments: list[list[tuple[float, float]]], output: Path) -> None:
    """월드 좌표 비율을 유지하면서 경로와 checkpoint를 SVG로 저장한다."""
    map_points = [point for segment in lane_segments for point in segment]
    all_points = path_points + checkpoints + map_points
    if not all_points:
        raise ValueError("no points to draw")

    width, height, margin = 1200.0, 800.0, 50.0
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min((width - 2 * margin) / span_x,
                (height - 2 * margin) / span_y)

    def screen(point: tuple[float, float]) -> tuple[float, float]:
        x = margin + (point[0] - min_x) * scale
        y = height - margin - (point[1] - min_y) * scale
        return x, y

    path_polyline = " ".join(
        f"{x:.2f},{y:.2f}" for x, y in map(screen, path_points))
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" '
        f'height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]

    # 지도 차선을 먼저 그려 reference path가 위에 보이게 한다.
    for segment in lane_segments:
        polyline = " ".join(
            f"{x:.2f},{y:.2f}" for x, y in map(screen, segment))
        svg.append(f'<polyline points="{polyline}" fill="none" '
                   'stroke="#c4c8cc" stroke-width="0.45"/>')

    svg.append(f'<polyline points="{path_polyline}" fill="none" '
               'stroke="#1769e0" stroke-width="0.55"/>')

    for sequence, point in enumerate(checkpoints, start=1):
        x, y = screen(point)
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="0.9" fill="#e53935"/>')
        svg.append(f'<text x="{x + 1.8:.2f}" y="{y - 1.8:.2f}" '
                   f'font-family="sans-serif" font-size="5">{sequence}</text>')

    svg += [
        f'<text x="{margin}" y="25" font-family="sans-serif" font-size="16">'
        f'VTD world path: X [{min_x:.1f}, {max_x:.1f}] m, '
        f'Y [{min_y:.1f}, {max_y:.1f}] m</text>',
        '</svg>',
    ]
    output.write_text("\n".join(svg), encoding="utf-8")


def read_lane_map(csv_path: Path) -> list[list[tuple[float, float]]]:
    """C++ 생성기가 저장한 lane-map CSV를 lane segment별로 묶는다."""
    segments: dict[tuple[int, int, int], list[tuple[int, float, float]]] = defaultdict(list)
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["road_id"]), int(row["lane_id"]),
                   int(row["section_index"]))
            segments[key].append((int(row["point_index"]),
                                  float(row["x"]), float(row["y"])))
    return [[(x, y) for _, x, y in sorted(points)]
            for points in segments.values()]


def main() -> None:
    """명령행 인자를 읽어 reference_path.svg를 생성한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument("path_csv", type=Path, nargs="?")
    parser.add_argument("--waypoints", type=Path)
    parser.add_argument("--map", dest="lane_map", type=Path)
    parser.add_argument("--output", type=Path, default=Path("reference_path.svg"))
    args = parser.parse_args()

    path_points = read_xy(args.path_csv) if args.path_csv else []
    checkpoints = read_xy(args.waypoints) if args.waypoints else []
    lane_segments = read_lane_map(args.lane_map) if args.lane_map else []
    write_svg(path_points, checkpoints, lane_segments, args.output)
    print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
