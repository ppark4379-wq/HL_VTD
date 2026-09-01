#!/usr/bin/env python3
"""VTD 원본 Scenario XML을 저장소 패키지로 동기화한다.

Ego가 PosRoute/PathRef를 사용하면 TrackId/road s/StartLane을 추출해 기존
OpenDRIVE lane graph로 checkpoints.csv, route.csv, route.svg까지 재생성한다.
PosAbsolute Scenario는 기존 checkpoints.csv의 목적지를 유지하고 Ego 시작 좌표를
갱신해 OpenDRIVE 경로를 다시 생성한다.
"""

from __future__ import annotations

import argparse
import csv
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


@dataclass(frozen=True)
class SyncConfig:
    repo_root: Path
    map_path: Path
    generator: Path
    checkpoint_generator: Path
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
            return EgoAbsolutePosition(float(position.attrib["X"]),
                                       float(position.attrib["Y"]))
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
    plotter = repo_root / "tools" / "plot_reference_path.py"
    if not plotter.is_file():
        raise SyncError(f"경로 SVG 도구를 찾지 못했습니다: {plotter}")
    return SyncConfig(
        repo_root, map_path, generator, checkpoint_generator, plotter
    )


def write_absolute_checkpoints(
    source: Path, destination: Path, ego: EgoAbsolutePosition
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


def sync_one(source: Path, config: SyncConfig, require_route: bool) -> None:
    source = source.expanduser().resolve()
    if not source.is_file() or source.suffix.lower() != ".xml":
        raise SyncError(f"VTD XML 파일이 아닙니다: {source}")
    _, ego_route, ego_absolute = parse_xml(source)
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
            print(
                f"[sync] XML copied: {xml_destination}\n"
                "[sync] PosAbsolute checkpoints missing: existing route was kept",
                flush=True,
            )
            return

        destination.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(
            tempfile.mkdtemp(prefix=f".{destination_stem}.sync-", dir=destination.parent)
        )
        try:
            staged_xml = staging / f"{destination_stem}.xml"
            staged_checkpoints = staging / "checkpoints.csv"
            staged_route = staging / "route.csv"
            staged_lane_map = staging / "lane_map.csv"
            staged_svg = staging / "route.svg"
            shutil.copy2(source, staged_xml)
            checkpoint_count = write_absolute_checkpoints(
                checkpoints, staged_checkpoints, ego_absolute
            )
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
            print(
                f"[sync] synced PosAbsolute: {source} -> {destination}\n"
                f"[sync] Ego start=({ego_absolute.x:.3f}, {ego_absolute.y:.3f}) "
                f"checkpoints={checkpoint_count}",
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
