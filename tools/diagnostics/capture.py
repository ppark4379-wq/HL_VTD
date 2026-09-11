"""Freeze source/config provenance before launch; keep original evidence local."""

from __future__ import annotations

import hashlib
import json
import importlib.metadata
import platform
from pathlib import Path
import shutil
import subprocess
from datetime import datetime, timezone

from contracts import CaptureRequest


INPUT_DEFAULTS = {
    "--config": "config/network.yaml",
    "--control-config": "config/control.yaml",
    "--map-config": "config/maps/living_lab.yaml",
    "--vehicle-config": "config/vehicle.yaml",
    "--path": "output/reference_path.csv",
    "--checkpoints": None,
}
MANAGED_OPTIONS = {"--diagnostic-dir", "--log", "--decision-log", "--ready-file", "--mission-output"}
SOURCE_SUFFIXES = {".cpp", ".c", ".hpp", ".h", ".inl", ".inc", ".py", ".sh", ".ps1", ".bat", ".cmake", ".in"}
FIXTURE_SUFFIXES = {".csv", ".json", ".yaml", ".yml", ".xodr"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def save_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def git_output(root: Path, arguments: list[str]) -> str:
    try:
        result = subprocess.run(["git", "-C", str(root), *arguments], capture_output=True,
                                text=True, encoding="utf-8", errors="replace", timeout=8)
        return result.stdout if result.returncode == 0 else "UNAVAILABLE: " + result.stderr
    except (OSError, subprocess.TimeoutExpired) as error:
        return "UNAVAILABLE: " + str(error)


def argument_value(arguments: list[str], option: str, default: str | None = None) -> str | None:
    positions = [i for i, value in enumerate(arguments) if value == option]
    if len(positions) > 1:
        raise ValueError(f"duplicate option: {option}")
    if not positions:
        return default
    index = positions[0] + 1
    if index >= len(arguments) or arguments[index].startswith("--"):
        raise ValueError(f"missing value: {option}")
    return arguments[index]


def resolve_input(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def snapshot_file(source: Path, target: Path) -> dict:
    result = {"original_path": str(source), "snapshot": str(target), "exists": source.is_file()}
    if not source.is_file():
        return result
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    result.update(size=target.stat().st_size, sha256=sha256(target))
    return result


def snapshot_sources(request: CaptureRequest) -> list[dict]:
    records = []
    for directory, suffixes in (("src", SOURCE_SUFFIXES | FIXTURE_SUFFIXES),
                                ("config", {".yaml", ".yml"}),
                                ("tools", SOURCE_SUFFIXES | FIXTURE_SUFFIXES),
                                ("tests", SOURCE_SUFFIXES | FIXTURE_SUFFIXES),
                                ("scripts", SOURCE_SUFFIXES)):
        for path in sorted((request.root / directory).rglob("*")):
            if path.is_file() and not path.is_symlink() and path.suffix in suffixes:
                target = request.directory / "snapshot" / path.relative_to(request.root)
                record = snapshot_file(path, target)
                record["snapshot"] = str(target.relative_to(request.directory))
                records.append(record)
    root_files = ["CMakeLists.txt"]
    root_files.extend(path.name for path in request.root.glob("requirements*.txt") if path.is_file())
    root_files.extend(name for name in ("pyproject.toml", "setup.cfg", "CMakePresets.json")
                      if (request.root / name).is_file())
    for name in root_files:
        source = request.root / name
        target = request.directory / "snapshot" / name
        record = snapshot_file(source, target)
        record["snapshot"] = str(target.relative_to(request.directory))
        records.append(record)
    return records


def capture_environment(directory: Path) -> None:
    packages = {}
    for name in ("numpy", "scipy", "PyYAML"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = "NOT_INSTALLED"
    save_json(directory / "environment.json", {
        "python_version": platform.python_version(), "platform": platform.platform(),
        "python_packages": packages,
        "scope": "capture-tool environment; original binary compiler/build options are not verified",
    })


def snapshot_inputs(request: CaptureRequest) -> dict:
    records = {}
    for option, default in INPUT_DEFAULTS.items():
        value = argument_value(request.arguments, option, default)
        if value is None:
            continue
        source = resolve_input(request.root, value)
        target = request.directory / "inputs" / (option[2:] + source.suffix)
        record = snapshot_file(source, target)
        record["snapshot"] = str(target.relative_to(request.directory))
        records[option] = record
    # Map loader uses these paths relative to repository cwd, even for external YAML.
    map_record = records.get("--map-config", {})
    if map_record.get("exists"):
        mapping = read_map_dependencies(request.directory / map_record["snapshot"], map_record)
        for key in ("xodr", "static_features", "school_zone_reference_points"):
            value = mapping.get(key)
            if not isinstance(value, str):
                continue
            source = resolve_input(request.root, value)
            target = request.directory / "inputs" / (key + source.suffix)
            record = snapshot_file(source, target)
            record["snapshot"] = str(target.relative_to(request.directory))
            records[key] = record
    source = request.root / "output/precomputed_lane_network/manifest.json"
    target = request.directory / "inputs/precomputed_manifest.json"
    records["precomputed_manifest"] = snapshot_file(source, target)
    records["precomputed_manifest"]["snapshot"] = str(target.relative_to(request.directory))
    return records


def read_map_dependencies(path: Path, record: dict) -> dict:
    try:
        import yaml
    except ImportError:
        record["dependency_error"] = "PyYAML unavailable; map YAML retained but dependencies not resolved"
        return {}
    try:
        content = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (yaml.YAMLError, UnicodeError) as error:
        record["dependency_error"] = str(error)
        return {}
    mapping = content.get("map", {}) if isinstance(content, dict) else {}
    return mapping if isinstance(mapping, dict) else {}


def build_command(request: CaptureRequest) -> list[str]:
    for argument in request.arguments:
        if argument.split("=", 1)[0] in MANAGED_OPTIONS:
            raise ValueError(f"{argument} is managed by run_field.py; remove it from forwarded arguments")
    directory = request.directory
    command = [str(request.binary), *request.arguments,
            "--diagnostic-dir", str(directory), "--log", str(directory / "run.csv"),
            "--ready-file", str(directory / "connected.flag"),
            "--mission-output", str(directory / "mission.svg")]
    if request.legacy_decision_log:
        command.extend(["--decision-log", str(directory / "decision.log")])
    return command


def prepare_capture(request: CaptureRequest) -> dict:
    command = build_command(request)
    # Fail before touching an existing attempt, even if it contains partial evidence.
    request.directory.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema": 1, "status": "PREPARING", "created_at": datetime.now(timezone.utc).isoformat(),
        "directory": str(request.directory), "cwd": str(request.root), "command": command,
        "source_head": git_output(request.root, ["rev-parse", "HEAD"]).strip(),
        "source_binary_match": "UNVERIFIED", "notes": [],
        "diagnostic_mode": "async_bounded", "legacy_decision_log": request.legacy_decision_log,
    }
    save_json(request.directory / "capture_manifest.json", manifest)
    (request.directory / "changes.patch").write_text(git_output(request.root, ["diff", "HEAD", "--", "src", "config", "CMakeLists.txt", "tools/diagnostics"]), encoding="utf-8")
    (request.directory / "git_status.txt").write_text(git_output(request.root, ["status", "--short"]), encoding="utf-8")
    manifest["source_files"] = snapshot_sources(request)
    capture_environment(request.directory)
    manifest["inputs"] = snapshot_inputs(request)
    for key, record in manifest["inputs"].items():
        if not record.get("exists"):
            manifest["notes"].append(f"INPUT_MISSING: {key} {record.get('original_path')}")
        if record.get("dependency_error"):
            manifest["notes"].append(f"MAP_DEPENDENCIES_UNRESOLVED: {record['dependency_error']}")
    manifest["binary"] = {"path": str(request.binary), "exists": request.binary.is_file()}
    if request.binary.is_file():
        manifest["binary"].update(sha256=sha256(request.binary), size=request.binary.stat().st_size)
        newer = [entry["original_path"] for entry in manifest["source_files"]
                 if entry["exists"] and entry["snapshot"].startswith("snapshot/src/")
                 and Path(entry["original_path"]).stat().st_mtime > request.binary.stat().st_mtime]
        if newer:
            manifest["notes"].append("SOURCE_NEWER_THAN_BINARY: " + ", ".join(newer[:10]))
    else:
        manifest["notes"].append("CONTROLLER_BINARY_MISSING")
    save_json(request.directory / "capture_manifest.json", manifest)
    return manifest
