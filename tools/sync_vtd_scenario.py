#!/usr/bin/env python3
"""VTD 원본 Scenario XML을 저장소 패키지로 동기화한다.

Ego가 PosRoute/PathRef를 사용하면 TrackId/road s/StartLane을 추출해 기존
OpenDRIVE lane graph로 checkpoints.csv, route.csv, route.svg까지 재생성한다.
PosAbsolute Scenario는 기존 checkpoints.csv의 목적지를 유지하고 Ego 시작 좌표를
갱신해 OpenDRIVE 경로를 다시 생성한다. scenario.yaml의
route.preserve_checkpoint_coordinates가 true면 대회 제공 좌표 전체를 보존하고,
route.linear_checkpoint_segments에 지정한 구간은 checkpoint 직선 형상을 보존한다.
route.smooth_checkpoint_segments는 차선 graph로 연결할 수 없는 인접
checkpoint 구간을 양쪽 경로의 방향을 이은 Bezier 곡선으로 연결한다.
"""

from __future__ import annotations

import argparse
import copy
import csv
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


CATEGORY_BY_PREFIX = {
    "B": "basic",
    "T": "traffic",
    "V": "vehicle",
    "P": "pedestrian",
    "O": "obstacle",
    "M": "mixed",
}
SCENARIO_NAME_PATTERN = re.compile(r"^([BTVPOMbtvpom])(\d+)(.*)$")


class SyncError(RuntimeError):
    """사용자가 수정할 수 있는 Scenario 동기화 오류."""


@dataclass(frozen=True)
class VtdPathPoint:
    sequence: int
    road_id: int
    road_s: float


@dataclass(frozen=True)
class EgoRoute:
    path_id: int
    start_lane: int
    points: tuple[VtdPathPoint, ...]


@dataclass(frozen=True)
class EgoAbsolutePosition:
    x: float
    y: float
    heading: float | None = None


@dataclass(frozen=True)
class RouteSyncOptions:
    linear_checkpoint_segments: tuple[int, ...] = ()
    smooth_checkpoint_segments: tuple[int, ...] = ()
    smooth_checkpoint_handle_m: float = 8.0
    preserve_checkpoint_coordinates: bool = False
    path_shape_lane_snap: bool = False
    trim_route_to_ego_start: bool = False
    trim_max_distance_m: float = 5.0
    bridge_distant_ego_start: bool = False


@dataclass(frozen=True)
class SyncConfig:
    repo_root: Path
    map_path: Path
    generator: Path
    checkpoint_generator: Path
    path_shape_generator: Path
    plotter: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="VTD Scenario XML 복사 및 Ego reference path 자동 생성"
    )
    parser.add_argument("source", type=Path, help="VTD XML 파일 또는 감시할 폴더")
    parser.add_argument(
        "--watch", action="store_true", help="폴더/XML 변경을 계속 감시"
    )
    parser.add_argument(
        "--repo-root", type=Path, help="HL_VTD 저장소 루트(기본: 스크립트 상위)"
    )
    parser.add_argument("--map", dest="map_path", type=Path, help="XODR 경로")
    parser.add_argument(
        "--generator", type=Path, help="vtd_path_to_checkpoints 실행 파일"
    )
    parser.add_argument(
        "--poll-interval", type=float, default=1.0, help="감시 주기(초)"
    )
    parser.add_argument(
        "--settle-time", type=float, default=0.5, help="저장 완료 대기 시간(초)"
    )
    parser.add_argument(
        "--no-initial-sync",
        action="store_true",
        help="watch 시작 시 기존 XML은 동기화하지 않음",
    )
    parser.add_argument(
        "--require-route",
        action="store_true",
        help="Ego PosRoute가 없으면 XML만 복사하지 않고 실패 처리",
    )
    return parser.parse_args()


def scenario_identity(stem: str) -> tuple[str, str, str]:
    match = SCENARIO_NAME_PATTERN.match(stem)
    if not match:
        raise SyncError(
            f"Scenario 파일명은 B/T/V/P/O/M + 숫자로 시작해야 합니다: {stem}"
        )
    prefix = match.group(1).upper()
    scenario_id = f"{prefix}{int(match.group(2)):02d}"
    normalized_stem = scenario_id + match.group(3)
    return scenario_id, normalized_stem, CATEGORY_BY_PREFIX[prefix]


def resolve_destination(source: Path, repo_root: Path) -> tuple[Path, str]:
    scenario_id, normalized_stem, category = scenario_identity(source.stem)
    category_dir = repo_root / "scenario" / category
    existing = []
    if category_dir.is_dir():
        for path in category_dir.iterdir():
            name = path.name.upper()
            if path.is_dir() and (
                name == scenario_id or name.startswith(scenario_id + "_")
            ):
                existing.append(path)
    if len(existing) > 1:
        names = ", ".join(sorted(path.name for path in existing))
        raise SyncError(f"{scenario_id} 대상 폴더가 여러 개입니다: {names}")
    destination = existing[0] if existing else category_dir / normalized_stem
    return destination, destination.name


def find_ego_route(root: ET.Element) -> EgoRoute | None:
    traffic_control = root.find("TrafficControl")
    if traffic_control is None:
        raise SyncError("TrafficControl이 없습니다")

    ego_player = None
    for player in traffic_control.findall("Player"):
        description = player.find("Description")
        if description is not None and description.get("Name") == "Ego":
            ego_player = player
            break
    if ego_player is None:
        raise SyncError('Description Name="Ego"인 Player가 없습니다')

    init = ego_player.find("Init")
    if init is None:
        raise SyncError("Ego Init이 없습니다")
    path_ref = init.find("PathRef")
    if init.find("PosRoute") is None or path_ref is None:
        return None

    try:
        path_id = int(path_ref.attrib["PathId"])
        start_lane = int(path_ref.attrib["StartLane"])
    except (KeyError, ValueError) as error:
        raise SyncError("Ego PathRef의 PathId/StartLane이 올바르지 않습니다") from error

    selected_path = None
    for path in traffic_control.findall("Path"):
        try:
            candidate_id = int(path.attrib.get("PathId", ""))
        except ValueError:
            continue
        if candidate_id == path_id:
            selected_path = path
            break
    if selected_path is None:
        raise SyncError(f"Ego가 참조하는 PathId={path_id}를 찾지 못했습니다")

    points = []
    for sequence, waypoint in enumerate(selected_path.findall("Waypoint"), start=1):
        try:
            road_id = int(waypoint.attrib["TrackId"])
            road_s = float(waypoint.attrib["s"])
        except (KeyError, ValueError) as error:
            raise SyncError(
                f"PathId={path_id} waypoint {sequence}의 TrackId/s가 올바르지 않습니다"
            ) from error
        points.append(VtdPathPoint(sequence, road_id, road_s))
    if len(points) < 2:
        raise SyncError(f"PathId={path_id}에는 waypoint가 2개 이상 필요합니다")
    return EgoRoute(path_id, start_lane, tuple(points))


def find_ego_absolute(root: ET.Element) -> EgoAbsolutePosition | None:
    for player in root.findall("./TrafficControl/Player"):
        description = player.find("Description")
        if description is None or description.get("Name") != "Ego":
            continue
        position = player.find("./Init/PosAbsolute")
        if position is None:
            return None
        try:
            heading = position.get("Direction")
            return EgoAbsolutePosition(
                float(position.attrib["X"]),
                float(position.attrib["Y"]),
                float(heading) if heading is not None else None,
            )
        except (KeyError, ValueError) as error:
            raise SyncError("Ego PosAbsolute X/Y가 올바르지 않습니다") from error
    return None


def parse_xml(
    path: Path,
) -> tuple[ET.ElementTree, EgoRoute | None, EgoAbsolutePosition | None]:
    try:
        tree = ET.parse(path)
    except (OSError, ET.ParseError) as error:
        raise SyncError(f"XML을 읽을 수 없습니다: {path}: {error}") from error
    root = tree.getroot()
    return tree, find_ego_route(root), find_ego_absolute(root)


def locate_generator(repo_root: Path, requested: Path | None) -> Path:
    if requested:
        candidates = [requested]
    else:
        executable = "vtd_path_to_checkpoints.exe" if os.name == "nt" else "vtd_path_to_checkpoints"
        candidates = [
            repo_root / "build" / executable,
            repo_root / "build-wsl" / executable,
            repo_root / executable,
        ]
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved.is_file():
            return resolved
    searched = ", ".join(str(path) for path in candidates)
    raise SyncError(
        "vtd_path_to_checkpoints를 찾지 못했습니다. 먼저 scripts/build.sh를 "
        f"실행하세요. 검색 위치: {searched}"
    )


def make_config(args: argparse.Namespace) -> SyncConfig:
    script_path = Path(__file__).resolve()
    repo_root = (args.repo_root or script_path.parent.parent).expanduser().resolve()
    map_path = (
        args.map_path
        or repo_root / "src" / "data" / "map" / "HL_FMA_VTD_LivingLab.xodr"
    ).expanduser().resolve()
    if not map_path.is_file():
        raise SyncError(f"XODR을 찾지 못했습니다: {map_path}")
    generator = locate_generator(repo_root, args.generator)
    checkpoint_generator = repo_root / "build" / "checkpoints_to_route"
    if not checkpoint_generator.is_file():
        raise SyncError(
            "checkpoints_to_route를 찾지 못했습니다. 먼저 scripts/build.sh를 "
            "실행하세요"
        )
    path_shape_executable = (
        "path_shape_to_route.exe" if os.name == "nt" else "path_shape_to_route"
    )
    path_shape_generator = repo_root / "build" / path_shape_executable
    if not path_shape_generator.is_file():
        raise SyncError(
            "path_shape_to_route를 찾지 못했습니다. 먼저 scripts/build.sh를 "
            "실행하세요"
        )
    plotter = repo_root / "tools" / "plot_reference_path.py"
    if not plotter.is_file():
        raise SyncError(f"경로 SVG 도구를 찾지 못했습니다: {plotter}")
    return SyncConfig(
        repo_root, map_path, generator, checkpoint_generator,
        path_shape_generator, plotter
    )


def write_absolute_checkpoints(
    source: Path,
    destination: Path,
    ego: EgoAbsolutePosition,
    preserve_coordinates: bool = False,
) -> int:
    try:
        with source.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    except OSError as error:
        raise SyncError(f"기존 checkpoints.csv를 읽을 수 없습니다: {source}") from error
    if len(rows) < 2 or not {"seq", "x", "y"}.issubset(rows[0]):
        raise SyncError(
            "PosAbsolute 경로 재생성에는 seq,x,y 형식의 체크포인트 2개 이상이 필요합니다"
        )
    if not preserve_coordinates:
        rows[0]["x"] = f"{ego.x:.8f}"
        rows[0]["y"] = f"{ego.y:.8f}"
    with destination.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["seq", "x", "y"], lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(
            {"seq": row["seq"], "x": row["x"], "y": row["y"]}
            for row in rows
        )
    return len(rows)


def read_route_sync_options(package: Path) -> RouteSyncOptions:
    """Read checkpoint preservation and linear-segment settings."""
    manifest_path = package / "scenario.yaml"
    if not manifest_path.is_file():
        return RouteSyncOptions()
    try:
        import yaml
    except ImportError as error:
        raise SyncError("scenario.yaml을 읽으려면 PyYAML이 필요합니다") from error
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise SyncError(f"scenario.yaml을 읽을 수 없습니다: {manifest_path}") from error
    if not isinstance(manifest, dict):
        raise SyncError(f"scenario.yaml 루트는 mapping이어야 합니다: {manifest_path}")
    route = manifest.get("route", {})
    if not isinstance(route, dict):
        raise SyncError(f"scenario.yaml route는 mapping이어야 합니다: {manifest_path}")
    raw_segments = route.get("linear_checkpoint_segments", [])
    if not isinstance(raw_segments, list):
        raise SyncError("route.linear_checkpoint_segments는 정수 배열이어야 합니다")
    preserve_coordinates = route.get("preserve_checkpoint_coordinates", False)
    if not isinstance(preserve_coordinates, bool):
        raise SyncError("route.preserve_checkpoint_coordinates는 bool이어야 합니다")
    path_shape_lane_snap = route.get("path_shape_lane_snap", False)
    if not isinstance(path_shape_lane_snap, bool):
        raise SyncError("route.path_shape_lane_snap은 bool이어야 합니다")
    trim_route_to_ego_start = route.get("trim_route_to_ego_start", False)
    if not isinstance(trim_route_to_ego_start, bool):
        raise SyncError("route.trim_route_to_ego_start는 bool이어야 합니다")
    trim_max_distance = route.get("trim_max_distance_m", 5.0)
    if (isinstance(trim_max_distance, bool)
            or not isinstance(trim_max_distance, (int, float))
            or not math.isfinite(float(trim_max_distance))
            or trim_max_distance <= 0.0):
        raise SyncError("route.trim_max_distance_m는 유한한 양수여야 합니다")
    bridge_distant_start = route.get("bridge_distant_ego_start", False)
    if not isinstance(bridge_distant_start, bool):
        raise SyncError("route.bridge_distant_ego_start는 bool이어야 합니다")

    segments = []
    for value in raw_segments:
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise SyncError(
                "route.linear_checkpoint_segments에는 1 이상의 정수만 사용할 수 있습니다"
            )
        segments.append(value)
    raw_smooth_segments = route.get("smooth_checkpoint_segments", [])
    if not isinstance(raw_smooth_segments, list):
        raise SyncError("route.smooth_checkpoint_segments는 정수 배열이어야 합니다")
    smooth_segments = []
    for value in raw_smooth_segments:
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise SyncError(
                "route.smooth_checkpoint_segments에는 1 이상의 정수만 사용할 수 있습니다"
            )
        smooth_segments.append(value)
    smooth_handle = route.get("smooth_checkpoint_handle_m", 8.0)
    if (
        isinstance(smooth_handle, bool)
        or not isinstance(smooth_handle, (int, float))
        or not math.isfinite(float(smooth_handle))
        or float(smooth_handle) <= 0.0
    ):
        raise SyncError("route.smooth_checkpoint_handle_m는 0보다 큰 수여야 합니다")
    if set(segments) & set(smooth_segments):
        raise SyncError(
            "하나의 checkpoint 구간에 linear과 smooth 설정을 동시에 사용할 수 없습니다"
        )
    if path_shape_lane_snap and (segments or smooth_segments):
        raise SyncError(
            "path_shape_lane_snap은 linear/smooth checkpoint 설정과 "
            "함께 사용할 수 없습니다"
        )
    return RouteSyncOptions(
        linear_checkpoint_segments=tuple(sorted(set(segments))),
        smooth_checkpoint_segments=tuple(sorted(set(smooth_segments))),
        smooth_checkpoint_handle_m=float(smooth_handle),
        preserve_checkpoint_coordinates=preserve_coordinates,
        path_shape_lane_snap=path_shape_lane_snap,
        trim_route_to_ego_start=trim_route_to_ego_start,
        trim_max_distance_m=float(trim_max_distance),
        bridge_distant_ego_start=bridge_distant_start,
    )


def read_xy_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
    except OSError as error:
        raise SyncError(f"CSV를 읽을 수 없습니다: {path}") from error
    if not {"x", "y"}.issubset(fieldnames) or not rows:
        raise SyncError(f"x,y 열이 있는 CSV가 필요합니다: {path}")
    return fieldnames, rows


def xy(row: dict[str, str]) -> tuple[float, float]:
    try:
        return float(row["x"]), float(row["y"])
    except (KeyError, ValueError) as error:
        raise SyncError("경로 CSV의 x,y 값이 올바르지 않습니다") from error


def nearest_route_indices(
    checkpoints: list[dict[str, str]],
    route: list[dict[str, str]],
    max_distance_m: float = 2.0,
) -> list[int]:
    indices = []
    search_begin = 0
    for checkpoint in checkpoints:
        checkpoint_x, checkpoint_y = xy(checkpoint)
        nearest = min(
            range(search_begin, len(route)),
            key=lambda index: (
                (xy(route[index])[0] - checkpoint_x) ** 2
                + (xy(route[index])[1] - checkpoint_y) ** 2
            ),
        )
        route_x, route_y = xy(route[nearest])
        distance = math.hypot(route_x - checkpoint_x, route_y - checkpoint_y)
        if distance > max_distance_m:
            raise SyncError(
                f"checkpoint를 route에서 찾을 수 없습니다: "
                f"({checkpoint_x:.3f}, {checkpoint_y:.3f}), distance={distance:.3f}m"
            )
        indices.append(nearest)
        search_begin = min(nearest + 1, len(route) - 1)
    return indices


def contiguous_segment_groups(segments: tuple[int, ...]) -> list[tuple[int, int]]:
    groups: list[tuple[int, int]] = []
    for segment in segments:
        if groups and segment == groups[-1][1] + 1:
            groups[-1] = (groups[-1][0], segment)
        else:
            groups.append((segment, segment))
    return groups


def update_route_yaw_and_s(rows: list[dict[str, str]]) -> None:
    cumulative_s = 0.0
    previous_yaw = 0.0
    for index, row in enumerate(rows):
        current_x, current_y = xy(row)
        if index > 0:
            previous_x, previous_y = xy(rows[index - 1])
            step = math.hypot(current_x - previous_x, current_y - previous_y)
            if step > 2.0:
                raise SyncError(
                    f"직선 구간 병합 후 route 간격이 너무 큽니다: {step:.3f}m"
                )
            cumulative_s += step
        if index + 1 < len(rows):
            next_x, next_y = xy(rows[index + 1])
            if math.hypot(next_x - current_x, next_y - current_y) > 1e-9:
                previous_yaw = math.atan2(next_y - current_y, next_x - current_x)
        row["yaw"] = f"{previous_yaw:.8f}"
        row["s"] = f"{cumulative_s:.8f}"


def trim_route_to_ego_start(
    route_path: Path,
    ego: EgoAbsolutePosition,
    maximum_distance_m: float = 5.0,
    checkpoints_path: Path | None = None,
    bridge_distant_start: bool = False,
) -> int:
    fieldnames, rows = read_xy_rows(route_path)
    nearest_index = min(
        range(len(rows)),
        key=lambda index: (
            (xy(rows[index])[0] - ego.x) ** 2
            + (xy(rows[index])[1] - ego.y) ** 2
        ),
    )
    nearest_x, nearest_y = xy(rows[nearest_index])
    distance = math.hypot(nearest_x - ego.x, nearest_y - ego.y)
    join_index = nearest_index
    if distance > maximum_distance_m and not bridge_distant_start:
        raise SyncError(
            "Ego 중간 시작 위치가 경로에서 너무 멉니다: "
            f"distance={distance:.3f}m"
        )
    if distance > maximum_distance_m:
        if ego.heading is None or not math.isfinite(ego.heading):
            raise SyncError(
                "차선 밖 Ego 시작 연결에는 PosAbsolute Direction이 필요합니다"
            )
        # Some authored VTD scenarios start on a staging apron rather than a
        # driving lane. Search only forward on the validated mission route and
        # connect the exact XML pose with a curvature-limited cubic curve.
        best: tuple[float, int, list[dict[str, str]], float] | None = None
        search_end = min(len(rows) - 1, nearest_index + 700)
        for candidate_index in range(nearest_index, search_end):
            candidate_x, candidate_y = xy(rows[candidate_index])
            delta_x = candidate_x - ego.x
            delta_y = candidate_y - ego.y
            chord = math.hypot(delta_x, delta_y)
            if chord < maximum_distance_m or chord > 250.0:
                continue
            forward = (
                delta_x * math.cos(ego.heading)
                + delta_y * math.sin(ego.heading)
            )
            if forward <= 0.0:
                continue
            start = dict(rows[candidate_index])
            start.update({
                "x": f"{ego.x:.8f}",
                "y": f"{ego.y:.8f}",
                "yaw": f"{ego.heading:.8f}",
                "s": "0",
                "road_id": "0",
                "lane_id": "0",
                "road_s": "0",
            })
            for handle_fraction in (0.20, 0.30, 0.40, 0.50, 0.60):
                handle = min(100.0, chord * handle_fraction)
                bridge = bezier_bridge_rows(
                    start, rows[candidate_index], handle
                )
                candidate = [dict(start), *bridge]
                candidate.extend(dict(row) for row in rows[candidate_index:])
                update_route_yaw_and_s(candidate)
                try:
                    validate_smoothed_route(candidate)
                except SyncError:
                    continue
                score = chord + 0.01 * (candidate_index - nearest_index)
                if best is None or score < best[0]:
                    best = (score, candidate_index, candidate, chord)
        if best is None:
            raise SyncError(
                "XML Ego 좌표에서 전역경로로 연결되는 곡률 안전 경로를 "
                "만들 수 없습니다"
            )
        _, join_index, trimmed, bridge_distance = best
        print(
            "[sync] XML Ego off-lane bridge: "
            f"raw_distance={distance:.3f}m join_distance={bridge_distance:.3f}m "
            f"join_index={join_index}",
            flush=True,
        )
    else:
        trimmed = [dict(row) for row in rows[nearest_index:]]
    if len(trimmed) < 2:
        raise SyncError("Ego 중간 시작 위치 뒤에 남은 경로점이 부족합니다")
    # Find the active mission before rebasing route s. Anchors are checkpoint
    # 1, 3, 5, ... and the finish, matching missionAnchorIndices in C++.
    # Project in checkpoint order so repeated/overlapping roads keep their
    # occurrence in this mission instead of jumping to a later lap.
    start_leg = 0
    if checkpoints_path is not None:
        _, checkpoints = read_xy_rows(checkpoints_path)
        anchors = list(range(0, len(checkpoints), 2))
        if anchors[-1] != len(checkpoints) - 1:
            anchors.append(len(checkpoints) - 1)
        cursor = 0
        for leg_index, checkpoint_index in enumerate(anchors):
            checkpoint_x, checkpoint_y = xy(checkpoints[checkpoint_index])
            cursor = min(
                range(cursor, len(rows)),
                key=lambda index: (
                    (xy(rows[index])[0] - checkpoint_x) ** 2
                    + (xy(rows[index])[1] - checkpoint_y) ** 2
                ),
            )
            if cursor <= join_index:
                start_leg = min(leg_index, len(anchors) - 2)
    update_route_yaw_and_s(trimmed)
    with route_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(trimmed)
    print(
        f"[sync] route trimmed to Ego start: removed={join_index} "
        f"distance={distance:.3f}m remaining={len(trimmed)} "
        f"mission_start_leg={start_leg}",
        flush=True,
    )
    return start_leg


def write_checkpoint_slice(
    rows: list[dict[str, str]], begin: int, end: int, output: Path
) -> None:
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["seq", "x", "y"], lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(
            {"seq": index + 1, "x": row["x"], "y": row["y"]}
            for index, row in enumerate(rows[begin:end])
        )


def bezier_bridge_rows(
    start: dict[str, str],
    end: dict[str, str],
    requested_handle_m: float,
    sample_interval_m: float = 0.5,
) -> list[dict[str, str]]:
    start_x, start_y = xy(start)
    end_x, end_y = xy(end)
    start_yaw = float(start["yaw"])
    end_yaw = float(end["yaw"])
    chord = math.hypot(end_x - start_x, end_y - start_y)
    if chord <= 1e-6:
        raise SyncError("smooth checkpoint 연결 끝점이 서로 같습니다")
    handle = min(requested_handle_m, chord * 0.45)
    control1 = (
        start_x + handle * math.cos(start_yaw),
        start_y + handle * math.sin(start_yaw),
    )
    control2 = (
        end_x - handle * math.cos(end_yaw),
        end_y - handle * math.sin(end_yaw),
    )
    control_length = (
        handle
        + math.hypot(control2[0] - control1[0], control2[1] - control1[1])
        + handle
    )
    steps = max(2, math.ceil(control_length / sample_interval_m))
    bridge = []
    for step in range(1, steps):
        progress = step / steps
        inverse = 1.0 - progress
        point_x = (
            inverse ** 3 * start_x
            + 3.0 * inverse ** 2 * progress * control1[0]
            + 3.0 * inverse * progress ** 2 * control2[0]
            + progress ** 3 * end_x
        )
        point_y = (
            inverse ** 3 * start_y
            + 3.0 * inverse ** 2 * progress * control1[1]
            + 3.0 * inverse * progress ** 2 * control2[1]
            + progress ** 3 * end_y
        )
        bridge.append({
            "x": f"{point_x:.8f}",
            "y": f"{point_y:.8f}",
            "yaw": "0",
            "s": "0",
            # A bridge intentionally leaves the legal lane graph.  Zero lane
            # metadata makes downstream code use its normal world matching.
            "road_id": "0",
            "lane_id": "0",
            "road_s": "0",
        })
    return bridge


def validate_smoothed_route(rows: list[dict[str, str]]) -> None:
    maximum_curvature = 0.0
    for current, following in zip(rows, rows[1:]):
        current_x, current_y = xy(current)
        following_x, following_y = xy(following)
        spacing = math.hypot(following_x - current_x, following_y - current_y)
        if spacing <= 1e-9 or spacing > 2.0:
            raise SyncError(
                f"smooth checkpoint 경로 간격이 허용 범위를 벗어났습니다: {spacing:.3f}m"
            )
        yaw_delta = (
            float(following["yaw"]) - float(current["yaw"]) + math.pi
        ) % (2.0 * math.pi) - math.pi
        maximum_curvature = max(maximum_curvature, abs(yaw_delta) / spacing)
    if maximum_curvature > 0.20:
        raise SyncError(
            "smooth checkpoint 경로의 최대 곡률이 허용치를 초과합니다: "
            f"{maximum_curvature:.3f} 1/m"
        )


def generate_smooth_checkpoint_route(
    checkpoints_path: Path,
    route_path: Path,
    lane_map_path: Path,
    staging: Path,
    config: SyncConfig,
    bridge_segments: tuple[int, ...],
    handle_m: float,
) -> None:
    _, checkpoints = read_xy_rows(checkpoints_path)
    checkpoint_count = len(checkpoints)
    if any(segment >= checkpoint_count for segment in bridge_segments):
        raise SyncError(
            "route.smooth_checkpoint_segments 값은 checkpoint 개수보다 작아야 합니다"
        )
    boundaries = (0,) + bridge_segments + (checkpoint_count,)
    if any(end - begin < 2 for begin, end in zip(boundaries, boundaries[1:])):
        raise SyncError(
            "smooth checkpoint 구간 사이에는 차선 경로용 checkpoint가 "
            "2개 이상 필요합니다"
        )

    route_parts: list[list[dict[str, str]]] = []
    fieldnames: list[str] = []
    for part_index, (begin, end) in enumerate(
        zip(boundaries, boundaries[1:]), start=1
    ):
        part_checkpoints = staging / f"checkpoint_part_{part_index}.csv"
        part_route = staging / f"route_part_{part_index}.csv"
        write_checkpoint_slice(checkpoints, begin, end, part_checkpoints)
        run_checked(
            [
                str(config.checkpoint_generator),
                str(config.map_path),
                str(part_checkpoints),
                str(part_route),
                str(lane_map_path),
            ],
            config.repo_root,
        )
        current_fieldnames, current_rows = read_xy_rows(part_route)
        if not fieldnames:
            fieldnames = current_fieldnames
        route_parts.append(current_rows)

    merged = list(route_parts[0])
    for next_part in route_parts[1:]:
        merged.extend(bezier_bridge_rows(merged[-1], next_part[0], handle_m))
        merged.extend(next_part)
    update_route_yaw_and_s(merged)
    validate_smoothed_route(merged)
    with route_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(merged)
    joined = ",".join(str(segment) for segment in bridge_segments)
    print(
        f"[sync] smoothed disconnected checkpoint segments: {joined} "
        f"({len(merged)} route points)",
        flush=True,
    )


def splice_linear_checkpoint_segments(
    checkpoints_path: Path,
    lane_graph_route_path: Path,
    path_shape_route_path: Path,
    output_path: Path,
    segments: tuple[int, ...],
) -> None:
    """Replace selected lane-graph spans with straight checkpoint interpolation."""
    _, checkpoints = read_xy_rows(checkpoints_path)
    fieldnames, lane_graph_route = read_xy_rows(lane_graph_route_path)
    _, path_shape_route = read_xy_rows(path_shape_route_path)
    if any(segment >= len(checkpoints) for segment in segments):
        raise SyncError(
            "route.linear_checkpoint_segments 값은 checkpoint 개수보다 작아야 합니다"
        )

    # Checkpoints may lie away from a lane centerline (for example near a lane
    # edge), so allow one lane-width of offset when locating splice boundaries.
    # The PathShape route still has to pass within the stricter default limit.
    graph_indices = nearest_route_indices(
        checkpoints, lane_graph_route, max_distance_m=5.0
    )
    shape_indices = nearest_route_indices(checkpoints, path_shape_route)
    merged: list[dict[str, str]] = []
    graph_cursor = 0
    for first_segment, last_segment in contiguous_segment_groups(segments):
        first_checkpoint = first_segment - 1
        last_checkpoint = last_segment
        graph_start = graph_indices[first_checkpoint]
        graph_end = graph_indices[last_checkpoint]
        shape_start = shape_indices[first_checkpoint]
        shape_end = shape_indices[last_checkpoint]
        merged.extend(lane_graph_route[graph_cursor:graph_start])
        linear_rows = [
            dict(row) for row in path_shape_route[shape_start : shape_end + 1]
        ]
        graph_start_x, graph_start_y = xy(lane_graph_route[graph_start])
        graph_end_x, graph_end_y = xy(lane_graph_route[graph_end])
        shape_start_x, shape_start_y = xy(linear_rows[0])
        shape_end_x, shape_end_y = xy(linear_rows[-1])
        denominator = max(1, len(linear_rows) - 1)
        for index, row in enumerate(linear_rows):
            progress = index / denominator
            row_x, row_y = xy(row)
            offset_x = (
                (graph_start_x - shape_start_x) * (1.0 - progress)
                + (graph_end_x - shape_end_x) * progress
            )
            offset_y = (
                (graph_start_y - shape_start_y) * (1.0 - progress)
                + (graph_end_y - shape_end_y) * progress
            )
            row["x"] = f"{row_x + offset_x:.8f}"
            row["y"] = f"{row_y + offset_y:.8f}"
        for field in ("road_id", "lane_id", "road_s"):
            linear_rows[0][field] = lane_graph_route[graph_start][field]
            linear_rows[-1][field] = lane_graph_route[graph_end][field]
        merged.extend(linear_rows)
        graph_cursor = graph_end + 1
    merged.extend(lane_graph_route[graph_cursor:])

    filtered = []
    for row in merged:
        if filtered:
            current_x, current_y = xy(row)
            previous_x, previous_y = xy(filtered[-1])
            if math.hypot(current_x - previous_x, current_y - previous_y) <= 1e-4:
                continue
        filtered.append(row)
    if len(filtered) < 2:
        raise SyncError("직선 구간 병합 결과에 route point가 부족합니다")
    update_route_yaw_and_s(filtered)

    try:
        with output_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(filtered)
    except OSError as error:
        raise SyncError(f"병합 route를 저장할 수 없습니다: {output_path}") from error
    joined = ",".join(str(segment) for segment in segments)
    print(
        f"[sync] preserved linear checkpoint segments: {joined} "
        f"({len(filtered)} route points)",
        flush=True,
    )


def write_vtd_path(path: Path, route: EgoRoute) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "road_id", "road_s"])
        for point in route.points:
            writer.writerow([point.sequence, point.road_id, f"{point.road_s:.12f}"])


def run_checked(command: list[str], repo_root: Path) -> None:
    print("[sync] run:", " ".join(command), flush=True)
    try:
        subprocess.run(command, cwd=repo_root, check=True)
    except (OSError, subprocess.CalledProcessError) as error:
        raise SyncError(f"명령 실행 실패: {command[0]}: {error}") from error


def atomic_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    os.close(handle)
    temporary = Path(temporary_name)
    try:
        shutil.copy2(source, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def direct_child(element: ET.Element, name: str) -> ET.Element | None:
    """Return a direct child while tolerating an XML namespace."""
    return next(
        (child for child in element if child.tag.rsplit("}", 1)[-1] == name),
        None,
    )


def xml_manifest_entities(root: ET.Element) -> list[dict[str, object]]:
    """Extract the XML-backed portion of scenario.yaml entities."""
    entities: list[dict[str, object]] = []
    for element in root.iter():
        tag = element.tag.rsplit("}", 1)[-1]
        if tag == "Player":
            description = direct_child(element, "Description")
            if description is None:
                continue
            name = description.get("Name", "")
            entity_type = "vehicle"
        elif tag == "Character":
            description = None
            name = element.get("Name", "")
            entity_type = "pedestrian"
        elif tag == "Object":
            description = None
            name = element.get("Name", "")
            entity_type = "obstacle"
        else:
            continue
        if not name.strip():
            continue

        entity: dict[str, object] = {"name": name, "type": entity_type}
        position = None
        if tag == "Player":
            init = direct_child(element, "Init")
            position = direct_child(init, "PosAbsolute") if init is not None else None
            path_ref = direct_child(init, "PathRef") if init is not None else None
            path_shape_ref = (
                direct_child(init, "PathShapeRef") if init is not None else None
            )
            if position is None and path_ref is not None:
                try:
                    entity["initial_route"] = {
                        "path_id": int(path_ref.attrib["PathId"]),
                        "start_s_m": float(path_ref.attrib["StartS"]),
                        "target_s_m": float(path_ref.attrib["TargetS"]),
                        "start_lane": int(path_ref.attrib["StartLane"]),
                    }
                except (KeyError, ValueError):
                    pass
            elif position is None and path_shape_ref is not None:
                try:
                    entity["initial_path_shape"] = {
                        "path_shape_id": int(path_shape_ref.attrib["PathShapeId"]),
                        "start_s_m": float(path_shape_ref.attrib["StartS"]),
                        "target_s_m": float(path_shape_ref.attrib["TargetS"]),
                    }
                except (KeyError, ValueError):
                    pass
        else:
            position = direct_child(element, "StartPosAbs")

        if position is not None:
            try:
                entity["initial_position"] = {
                    "x": float(position.attrib["X"]),
                    "y": float(position.attrib["Y"]),
                    "z": float(position.attrib["Z"]),
                    "heading_rad": float(position.attrib["Direction"]),
                }
            except (KeyError, ValueError):
                pass
        if tag == "Object":
            scaling = direct_child(element, "Scaling")
            if scaling is not None:
                try:
                    entity["scale"] = {
                        axis.lower(): float(scaling.attrib[axis])
                        for axis in ("X", "Y", "Z")
                    }
                except (KeyError, ValueError):
                    pass
        entities.append(entity)
    return entities


def sync_manifest_from_xml(
    package: Path,
    scenario_file_name: str,
    root: ET.Element,
    mission_start_leg: int | None = None,
) -> None:
    """Reconcile XML-owned manifest fields while preserving authored metadata."""
    manifest_path = package / "scenario.yaml"
    if not manifest_path.is_file():
        return
    try:
        import yaml
    except ImportError as error:
        raise SyncError("scenario.yaml을 갱신하려면 PyYAML이 필요합니다") from error
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise SyncError(f"scenario.yaml을 읽을 수 없습니다: {manifest_path}") from error
    if not isinstance(manifest, dict):
        raise SyncError(f"scenario.yaml 루트는 mapping이어야 합니다: {manifest_path}")

    updated = copy.deepcopy(manifest)
    if mission_start_leg is not None:
        updated.setdefault("runtime", {})["mission_start_leg"] = mission_start_leg
    scenario = updated.setdefault("scenario", {})
    if not isinstance(scenario, dict):
        raise SyncError(f"scenario.yaml scenario는 mapping이어야 합니다: {manifest_path}")
    scenario["file"] = scenario_file_name

    extracted = xml_manifest_entities(root)
    existing_entities = manifest.get("entities", [])
    existing_by_name = {
        entity.get("name"): entity
        for entity in existing_entities
        if isinstance(entity, dict) and isinstance(entity.get("name"), str)
    } if isinstance(existing_entities, list) else {}
    reconciled = []
    for xml_entity in extracted:
        name = str(xml_entity["name"])
        entity = copy.deepcopy(existing_by_name.get(name, {}))
        for key in ("initial_position", "initial_route", "initial_path_shape", "scale"):
            entity.pop(key, None)
        entity.update(xml_entity)
        if name == "Ego":
            entity["role"] = "ego"
        else:
            entity.setdefault("role", "relevant_object")
        reconciled.append(entity)
    updated["entities"] = reconciled

    entity_names = {str(entity["name"]) for entity in extracted}
    events = updated.get("events")
    if isinstance(events, list):
        updated["events"] = [
            event
            for event in events
            if isinstance(event, dict)
            and (event.get("subject") is None or event.get("subject") in entity_names)
            and (
                not isinstance(event.get("trigger"), dict)
                or event["trigger"].get("reference") is None
                or event["trigger"].get("reference") in entity_names
            )
        ]

    ego_player = next(
        (
            element
            for element in root.iter()
            if element.tag.rsplit("}", 1)[-1] == "Player"
            and direct_child(element, "Description") is not None
            and direct_child(element, "Description").get("Name") == "Ego"
        ),
        None,
    )
    if ego_player is not None:
        description = direct_child(ego_player, "Description")
        validation = updated.setdefault("validation", {})
        if isinstance(validation, dict) and description is not None:
            control = description.get("Control")
            if control:
                validation["ego_control"] = control

    if updated == manifest:
        return
    handle, temporary_name = tempfile.mkstemp(
        prefix=".scenario.yaml.", suffix=".tmp", dir=package
    )
    os.close(handle)
    temporary = Path(temporary_name)
    try:
        temporary.write_text(
            yaml.safe_dump(updated, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        os.replace(temporary, manifest_path)
    except OSError as error:
        raise SyncError(f"scenario.yaml을 저장할 수 없습니다: {manifest_path}") from error
    finally:
        temporary.unlink(missing_ok=True)
    print(f"[sync] updated manifest: {manifest_path}", flush=True)


def sync_one(source: Path, config: SyncConfig, require_route: bool) -> None:
    source = source.expanduser().resolve()
    if not source.is_file() or source.suffix.lower() != ".xml":
        raise SyncError(f"VTD XML 파일이 아닙니다: {source}")
    tree, ego_route, ego_absolute = parse_xml(source)
    root = tree.getroot()
    destination, destination_stem = resolve_destination(source, config.repo_root)
    xml_destination = destination / f"{destination_stem}.xml"
    if source == xml_destination.resolve():
        raise SyncError("VTD 원본과 저장소 복사본이 같은 파일입니다")

    if ego_route is None:
        if require_route:
            raise SyncError(
                "Ego가 PosRoute/PathRef를 사용하지 않아 경로를 자동 생성할 수 없습니다"
            )
        checkpoints = destination / "checkpoints.csv"
        if ego_absolute is None or not checkpoints.is_file():
            atomic_copy(source, xml_destination)
            sync_manifest_from_xml(destination, xml_destination.name, root)
            print(
                f"[sync] XML copied: {xml_destination}\n"
                "[sync] PosAbsolute checkpoints missing: existing route was kept",
                flush=True,
            )
            return

        route_options = read_route_sync_options(destination)
        linear_segments = route_options.linear_checkpoint_segments
        smooth_segments = route_options.smooth_checkpoint_segments
        destination.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(
            tempfile.mkdtemp(prefix=f".{destination_stem}.sync-", dir=destination.parent)
        )
        try:
            staged_xml = staging / f"{destination_stem}.xml"
            staged_checkpoints = staging / "checkpoints.csv"
            staged_route = staging / "route.csv"
            staged_path_shape_route = staging / "path_shape_route.csv"
            staged_merged_route = staging / "merged_route.csv"
            staged_lane_map = staging / "lane_map.csv"
            staged_svg = staging / "route.svg"
            shutil.copy2(source, staged_xml)
            checkpoint_count = write_absolute_checkpoints(
                checkpoints,
                staged_checkpoints,
                ego_absolute,
                route_options.preserve_checkpoint_coordinates,
            )
            if route_options.path_shape_lane_snap:
                run_checked(
                    [
                        str(config.checkpoint_generator),
                        str(config.map_path),
                        str(staged_lane_map),
                    ],
                    config.repo_root,
                )
                run_checked(
                    [
                        str(config.path_shape_generator),
                        str(config.map_path),
                        str(staged_checkpoints),
                        str(staged_route),
                        "--lane-snap",
                    ],
                    config.repo_root,
                )
            elif smooth_segments:
                if linear_segments:
                    raise SyncError(
                        "smooth checkpoint 경로와 linear checkpoint 경로는 "
                        "한 시나리오에서 함께 사용할 수 없습니다"
                    )
                generate_smooth_checkpoint_route(
                    staged_checkpoints,
                    staged_route,
                    staged_lane_map,
                    staging,
                    config,
                    smooth_segments,
                    route_options.smooth_checkpoint_handle_m,
                )
            else:
                run_checked(
                    [
                        str(config.checkpoint_generator),
                        str(config.map_path),
                        str(staged_checkpoints),
                        str(staged_route),
                        str(staged_lane_map),
                    ],
                    config.repo_root,
                )
                if linear_segments:
                    run_checked(
                        [
                            str(config.path_shape_generator),
                            str(config.map_path),
                            str(staged_checkpoints),
                            str(staged_path_shape_route),
                        ],
                        config.repo_root,
                    )
                    splice_linear_checkpoint_segments(
                        staged_checkpoints,
                        staged_route,
                        staged_path_shape_route,
                        staged_merged_route,
                        linear_segments,
                    )
                    os.replace(staged_merged_route, staged_route)
            start_leg = 0
            if route_options.trim_route_to_ego_start:
                start_leg = trim_route_to_ego_start(
                    staged_route, ego_absolute,
                    route_options.trim_max_distance_m, staged_checkpoints,
                    route_options.bridge_distant_ego_start,
                )
            run_checked(
                [
                    sys.executable,
                    str(config.plotter),
                    str(staged_route),
                    "--waypoints",
                    str(staged_checkpoints),
                    "--map",
                    str(staged_lane_map),
                    "--output",
                    str(staged_svg),
                ],
                config.repo_root,
            )
            destination.mkdir(parents=True, exist_ok=True)
            for staged, name in (
                (staged_checkpoints, "checkpoints.csv"),
                (staged_route, "route.csv"),
                (staged_svg, "route.svg"),
                (staged_xml, f"{destination_stem}.xml"),
            ):
                os.replace(staged, destination / name)
            sync_manifest_from_xml(
                destination, xml_destination.name, root,
                mission_start_leg=start_leg,
            )
            print(
                f"[sync] synced PosAbsolute: {source} -> {destination}\n"
                f"[sync] Ego start=({ego_absolute.x:.3f}, {ego_absolute.y:.3f}) "
                f"checkpoints={checkpoint_count} "
                f"preserve_checkpoint_coordinates="
                f"{route_options.preserve_checkpoint_coordinates}",
                flush=True,
            )
        finally:
            shutil.rmtree(staging, ignore_errors=True)
        return

    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f".{destination_stem}.sync-", dir=destination.parent)
    )
    try:
        staged_xml = staging / f"{destination_stem}.xml"
        staged_vtd_path = staging / "vtd_path.csv"
        staged_checkpoints = staging / "checkpoints.csv"
        staged_route = staging / "route.csv"
        staged_lane_map = staging / "lane_map.csv"
        staged_svg = staging / "route.svg"
        shutil.copy2(source, staged_xml)
        write_vtd_path(staged_vtd_path, ego_route)
        run_checked(
            [
                str(config.generator),
                str(config.map_path),
                str(staged_vtd_path),
                str(ego_route.start_lane),
                str(staged_checkpoints),
                str(staged_route),
                str(staged_lane_map),
            ],
            config.repo_root,
        )
        run_checked(
            [
                sys.executable,
                str(config.plotter),
                str(staged_route),
                "--waypoints",
                str(staged_checkpoints),
                "--map",
                str(staged_lane_map),
                "--output",
                str(staged_svg),
            ],
            config.repo_root,
        )

        destination.mkdir(parents=True, exist_ok=True)
        for staged, name in (
            (staged_checkpoints, "checkpoints.csv"),
            (staged_route, "route.csv"),
            (staged_svg, "route.svg"),
            (staged_xml, f"{destination_stem}.xml"),
        ):
            os.replace(staged, destination / name)
        sync_manifest_from_xml(destination, xml_destination.name, root)
        print(
            f"[sync] synced: {source} -> {destination}\n"
            f"[sync] PathId={ego_route.path_id} StartLane={ego_route.start_lane} "
            f"waypoints={len(ego_route.points)}",
            flush=True,
        )
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def xml_files(source: Path) -> list[Path]:
    if source.is_file():
        return [source] if source.suffix.lower() == ".xml" else []
    if source.is_dir():
        return sorted(path for path in source.rglob("*.xml") if path.is_file())
    return []


def signature(path: Path) -> tuple[int, int]:
    stat = path.stat()
    return stat.st_mtime_ns, stat.st_size


def wait_until_stable(path: Path, settle_time: float, attempts: int = 8) -> None:
    for _ in range(attempts):
        before = signature(path)
        time.sleep(settle_time)
        after = signature(path)
        if before != after:
            continue
        try:
            ET.parse(path)
            return
        except (OSError, ET.ParseError):
            continue
    raise SyncError(f"VTD가 XML 저장을 완료하지 않았습니다: {path}")


def watch(source: Path, config: SyncConfig, args: argparse.Namespace) -> None:
    if not source.exists():
        raise SyncError(f"감시 대상이 없습니다: {source}")
    known: dict[Path, tuple[int, int]] = {}
    initial = xml_files(source)
    if args.no_initial_sync:
        known = {path: signature(path) for path in initial}
    else:
        for path in initial:
            try:
                wait_until_stable(path, args.settle_time)
                sync_one(path, config, args.require_route)
            except (OSError, SyncError) as error:
                print(f"[sync] ERROR: {error}", file=sys.stderr, flush=True)
            known[path] = signature(path)

    print(f"[sync] watching: {source} (Ctrl+C to stop)", flush=True)
    while True:
        current_files = xml_files(source)
        current_set = set(current_files)
        for removed in set(known) - current_set:
            del known[removed]
        for path in current_files:
            current_signature = signature(path)
            if known.get(path) == current_signature:
                continue
            try:
                wait_until_stable(path, args.settle_time)
                sync_one(path, config, args.require_route)
                known[path] = signature(path)
            except (OSError, SyncError) as error:
                print(f"[sync] ERROR: {error}", file=sys.stderr, flush=True)
                known[path] = signature(path)
        time.sleep(args.poll_interval)


def main() -> int:
    args = parse_args()
    if args.poll_interval <= 0.0 or args.settle_time <= 0.0:
        print("[sync] ERROR: poll/settle 시간은 0보다 커야 합니다", file=sys.stderr)
        return 2
    try:
        config = make_config(args)
        source = args.source.expanduser().resolve()
        if args.watch:
            watch(source, config, args)
        else:
            if not source.is_file():
                raise SyncError("수동 동기화는 XML 파일 하나를 지정해야 합니다")
            wait_until_stable(source, args.settle_time)
            sync_one(source, config, args.require_route)
    except KeyboardInterrupt:
        print("\n[sync] stopped")
        return 0
    except (OSError, SyncError) as error:
        print(f"[sync] ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
