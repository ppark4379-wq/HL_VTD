#!/usr/bin/env python3
# Validate a Phase 1 self-contained Scenario Package before a VTD run.

from __future__ import annotations

import argparse
import csv
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

try:
    import yaml
except ImportError as error:
    yaml = None
    YAML_IMPORT_ERROR = error
else:
    YAML_IMPORT_ERROR = None

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_ROOT = REPO_ROOT / "scenario"
CATALOG_PATH = SCENARIO_ROOT / "scenario_catalog.csv"
REQUIRED_ROUTE_COLUMNS = (
    "x", "y", "yaw", "s", "road_id", "lane_id", "road_s"
)
SCENARIO_ID_PATTERN = re.compile(r"^[A-Z][0-9]{2}(?:-[0-9]{2})?$")
PARAMETER_TYPES = {"float", "int", "string", "bool"}


@dataclass
class ValidationReport:
    package: Path
    messages: list[tuple[str, str]] = field(default_factory=list)
    manifest: dict[str, Any] | None = None
    scenario_file: Path | None = None
    route_file: Path | None = None
    tool_error: bool = False

    def add(self, level: str, message: str) -> None:
        self.messages.append((level, message))

    @property
    def failure_count(self) -> int:
        return sum(level == "FAIL" for level, _ in self.messages)

    @property
    def exit_code(self) -> int:
        if self.tool_error:
            return 2
        return 1 if self.failure_count else 0

    def print(self) -> None:
        for level, message in self.messages:
            print(f"[{level}] {message}")
        result = "PASS" if self.exit_code == 0 else "FAIL"
        print()
        print(f"RESULT: {result}")


if yaml is not None:

    class UniqueKeyLoader(yaml.SafeLoader):
        pass


    def construct_unique_mapping(
        loader: UniqueKeyLoader,
        node: yaml.MappingNode,
        deep: bool = False,
    ) -> dict[Any, Any]:
        mapping: dict[Any, Any] = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    f"duplicate key: {key!r}",
                    key_node.start_mark,
                )
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping


    UniqueKeyLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_unique_mapping,
    )


def load_manifest(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError(
            "PyYAML is required. Install it with "
            "'python3 -m pip install PyYAML'."
        ) from YAML_IMPORT_ERROR
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.load(stream, Loader=UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot load YAML: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("manifest root must be a mapping")
    return document


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def child_by_name(element: ET.Element, name: str) -> ET.Element | None:
    return next(
        (child for child in element if local_name(child.tag) == name),
        None,
    )


def elements_by_name(root: ET.Element, name: str) -> Iterator[ET.Element]:
    return (element for element in root.iter() if local_name(element.tag) == name)


def non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def path_is_within(path: Path, directory: Path) -> bool:
    try:
        path.relative_to(directory)
    except ValueError:
        return False
    return True


def is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def parameter_value_matches(type_name: str, value: Any) -> bool:
    if type_name == "float":
        return is_number(value)
    if type_name == "int":
        return isinstance(value, int) and not isinstance(value, bool)
    if type_name == "string":
        return isinstance(value, str)
    if type_name == "bool":
        return isinstance(value, bool)
    return False


def parameter_references(node: Any) -> Iterator[tuple[str, Any]]:
    if isinstance(node, dict):
        for key, value in node.items():
            key_text = str(key)
            if key_text == "parameter" or key_text.endswith("_parameter"):
                yield key_text, value
            yield from parameter_references(value)
    elif isinstance(node, list):
        for item in node:
            yield from parameter_references(item)


def validate_manifest_structure(
    report: ValidationReport,
    manifest: dict[str, Any],
) -> None:
    failures_before = report.failure_count
    if manifest.get("schema_version") != 1:
        report.add("FAIL", "schema_version must be 1")

    scenario = manifest.get("scenario")
    if not isinstance(scenario, dict):
        report.add("FAIL", "scenario must be a mapping")
        scenario = {}
    for key in ("id", "category", "file", "format"):
        if not non_empty_string(scenario.get(key)):
            report.add("FAIL", f"scenario.{key} is required")

    route = manifest.get("route")
    if not isinstance(route, dict):
        report.add("FAIL", "route must be a mapping")
        route = {}
    if not non_empty_string(route.get("default")):
        report.add("FAIL", "route.default is required")
    if route.get("mode") != "packaged":
        report.add("FAIL", "Phase 1 requires route.mode=packaged")
    if not isinstance(route.get("compatible_routes"), list):
        report.add("FAIL", "route.compatible_routes must be an array")

    parameters = manifest.get("parameters")
    if not isinstance(parameters, dict):
        report.add("FAIL", "parameters must be a mapping")
        parameters = {}
    for name, definition in parameters.items():
        if not non_empty_string(name) or not isinstance(definition, dict):
            report.add("FAIL", f"invalid parameter declaration: {name!r}")
            continue
        type_name = definition.get("type")
        if type_name not in PARAMETER_TYPES:
            report.add("FAIL", f"parameter {name}: unsupported type {type_name!r}")
            continue
        if "default" not in definition:
            report.add("FAIL", f"parameter {name}: default is required")
        elif not parameter_value_matches(type_name, definition["default"]):
            report.add(
                "FAIL", f"parameter {name}: default does not match type {type_name}"
            )

    for key in ("entities", "events", "expected_outcomes"):
        if not isinstance(manifest.get(key), list):
            report.add("FAIL", f"{key} must be an array")

    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        report.add("FAIL", "evaluation must be a mapping")
    else:
        for key in ("codes", "required_metrics"):
            if not isinstance(evaluation.get(key), list):
                report.add("FAIL", f"evaluation.{key} must be an array")
        if not non_empty_string(evaluation.get("thresholds_status")):
            report.add("FAIL", "evaluation.thresholds_status is required")

    validation = manifest.get("validation")
    if not isinstance(validation, dict):
        report.add("FAIL", "validation must be a mapping")
    else:
        for key in (
            "start_position_tolerance_m",
            "start_heading_tolerance_deg",
        ):
            value = validation.get(key)
            if not is_number(value) or float(value) < 0.0:
                report.add("FAIL", f"validation.{key} must be non-negative")

    if report.failure_count == failures_before:
        report.add("PASS", "manifest structure")


def validate_entities_and_events(
    report: ValidationReport,
    manifest: dict[str, Any],
) -> None:
    parameters = manifest.get("parameters")
    parameter_names = set(parameters) if isinstance(parameters, dict) else set()
    entities = manifest.get("entities")
    events = manifest.get("events")
    if not isinstance(entities, list) or not isinstance(events, list):
        return

    entity_names: set[str] = set()
    for index, entity in enumerate(entities):
        if not isinstance(entity, dict) or not non_empty_string(entity.get("name")):
            report.add("FAIL", f"entities[{index}].name is required")
            continue
        name = entity["name"]
        if name in entity_names:
            report.add("FAIL", f"duplicate entity name: {name}")
        entity_names.add(name)

    event_names: set[str] = set()
    for index, event in enumerate(events):
        if not isinstance(event, dict) or not non_empty_string(event.get("name")):
            report.add("FAIL", f"events[{index}].name is required")
            continue
        name = event["name"]
        if name in event_names:
            report.add("FAIL", f"duplicate event name: {name}")
        event_names.add(name)

        subject = event.get("subject")
        if subject is not None:
            if not non_empty_string(subject) or subject not in entity_names:
                report.add("FAIL", f"event {name}: unknown subject {subject!r}")
            else:
                report.add("PASS", f"event subject: {subject}")

        trigger = event.get("trigger")
        if isinstance(trigger, dict) and "reference" in trigger:
            reference = trigger["reference"]
            if reference not in entity_names:
                report.add("FAIL", f"event {name}: unknown reference {reference!r}")
            else:
                report.add("PASS", f"trigger reference: {reference}")

        for field_name, parameter_name in parameter_references(event):
            if not non_empty_string(parameter_name):
                report.add(
                    "FAIL", f"event {name}: {field_name} must name a parameter"
                )
            elif parameter_name not in parameter_names:
                report.add(
                    "FAIL", f"event {name}: unknown parameter {parameter_name}"
                )
            else:
                report.add("PASS", f"parameter reference: {parameter_name}")


def parse_scenario_xml(
    report: ValidationReport,
    manifest: dict[str, Any],
) -> ET.Element | None:
    scenario = manifest.get("scenario", {})
    file_name = scenario.get("file")
    if not non_empty_string(file_name):
        return None
    scenario_file = (report.package / file_name).resolve()
    report.scenario_file = scenario_file
    if not path_is_within(scenario_file, report.package):
        report.add("FAIL", "scenario.file must stay inside the Package")
        return None
    if not scenario_file.is_file():
        report.add("FAIL", f"scenario file missing: {scenario_file}")
        return None
    report.add("PASS", f"scenario file: {scenario_file.name}")

    scenario_format = scenario.get("format")
    if scenario_format != "vtd_native":
        report.add(
            "WARN",
            f"scenario format is not validated in Phase 1: {scenario_format}",
        )
        return None
    if scenario_file.suffix.lower() != ".xml":
        report.add("FAIL", "vtd_native scenario file must use .xml")
        return None
    try:
        root = ET.parse(scenario_file).getroot()
    except (OSError, ET.ParseError) as error:
        report.add("FAIL", f"scenario XML parse error: {error}")
        return None
    if local_name(root.tag) != "Scenario":
        report.add("FAIL", f"unexpected VTD XML root: {local_name(root.tag)}")
        return None
    report.add("PASS", "scenario XML syntax and VTD root")
    return root


def player_descriptions(root: ET.Element) -> dict[str, ET.Element]:
    descriptions: dict[str, ET.Element] = {}
    for player in elements_by_name(root, "Player"):
        description = child_by_name(player, "Description")
        if description is None:
            continue
        name = description.get("Name")
        if name:
            descriptions[name] = description
    return descriptions


def validate_xml_entities(
    report: ValidationReport,
    manifest: dict[str, Any],
    root: ET.Element,
) -> None:
    descriptions = player_descriptions(root)
    ego = descriptions.get("Ego")
    if ego is None:
        report.add("FAIL", "Ego player not found in Scenario XML")
    else:
        report.add("PASS", "Ego player found")
        expected_control = manifest.get("validation", {}).get("ego_control")
        actual_control = ego.get("Control")
        if expected_control and actual_control != expected_control:
            report.add(
                "FAIL",
                f"Ego control: {actual_control!r}, expected {expected_control!r}",
            )
        else:
            report.add("PASS", f"Ego control: {actual_control}")

    signal_ids = {
        element.get("Id")
        for element in elements_by_name(root, "SignalController")
        if element.get("Id") is not None
    }
    for entity in manifest.get("entities", []):
        if not isinstance(entity, dict):
            continue
        name = entity.get("name")
        source_id = entity.get("source_id")
        if source_id is not None:
            if str(source_id) in signal_ids:
                report.add(
                    "PASS",
                    f"entity found: {name} (SignalController {source_id})",
                )
            else:
                report.add(
                    "FAIL",
                    f"entity not found: {name} (SignalController {source_id})",
                )
        elif name in descriptions:
            report.add("PASS", f"entity found: {name}")
        else:
            report.add("FAIL", f"entity not found in Scenario XML: {name}")


def parse_route_csv(
    report: ValidationReport,
    route_file: Path,
) -> list[dict[str, float]]:
    try:
        with route_file.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.reader(stream)
            header = next(reader, None)
            if header is None:
                report.add("FAIL", "route CSV is empty")
                return []
            if tuple(header[: len(REQUIRED_ROUTE_COLUMNS)]) != REQUIRED_ROUTE_COLUMNS:
                report.add(
                    "FAIL",
                    "route CSV header must start with: "
                    + ",".join(REQUIRED_ROUTE_COLUMNS),
                )
                return []

            points: list[dict[str, float]] = []
            for line_number, row in enumerate(reader, start=2):
                if len(row) < len(REQUIRED_ROUTE_COLUMNS):
                    report.add(
                        "FAIL",
                        f"route CSV line {line_number}: expected at least 7 fields",
                    )
                    return []
                point: dict[str, float] = {}
                for index, column in enumerate(REQUIRED_ROUTE_COLUMNS):
                    try:
                        value = float(row[index])
                    except ValueError:
                        report.add(
                            "FAIL",
                            f"route CSV line {line_number}: {column} is not numeric",
                        )
                        return []
                    if not math.isfinite(value):
                        report.add(
                            "FAIL",
                            f"route CSV line {line_number}: {column} is not finite",
                        )
                        return []
                    point[column] = value
                points.append(point)
    except OSError as error:
        report.add("FAIL", f"cannot read route CSV: {error}")
        return []

    if not points:
        report.add("FAIL", "route CSV has no path points")
        return []
    report.add("PASS", f"route CSV format: {len(points)} points")
    return points


def resolve_packaged_route(
    report: ValidationReport,
    manifest: dict[str, Any],
) -> Path | None:
    route = manifest.get("route", {})
    if route.get("mode") != "packaged":
        return None
    default_route = route.get("default")
    if not non_empty_string(default_route):
        return None
    route_file = (report.package / default_route).resolve()
    report.route_file = route_file
    if not path_is_within(route_file, report.package):
        report.add("FAIL", "packaged route.default must stay inside the Package")
        return None
    if not route_file.is_file():
        report.add("FAIL", f"default route missing: {route_file}")
        return None
    report.add("PASS", f"default route: {route_file.name}")

    compatible_routes = route.get("compatible_routes", [])
    if isinstance(compatible_routes, list):
        for compatible in compatible_routes:
            if not non_empty_string(compatible):
                report.add("FAIL", "compatible route path must be a string")
                continue
            compatible_file = (report.package / compatible).resolve()
            if not path_is_within(compatible_file, report.package):
                report.add(
                    "FAIL",
                    f"packaged compatible route escapes Package: {compatible}",
                )
                continue
            if compatible_file.is_file():
                report.add("PASS", f"compatible route exists: {compatible}")
            else:
                report.add("FAIL", f"compatible route missing: {compatible}")
    return route_file


def ego_initial_pose(root: ET.Element) -> tuple[float, float, float] | None:
    for player in elements_by_name(root, "Player"):
        description = child_by_name(player, "Description")
        if description is None or description.get("Name") != "Ego":
            continue
        init = child_by_name(player, "Init")
        if init is None:
            return None
        position = child_by_name(init, "PosAbsolute")
        if position is None:
            return None
        try:
            return (
                float(position.attrib["X"]),
                float(position.attrib["Y"]),
                float(position.attrib["Direction"]),
            )
        except (KeyError, ValueError):
            return None
    return None


def angle_difference_degrees(left: float, right: float) -> float:
    difference = math.atan2(math.sin(left - right), math.cos(left - right))
    return abs(math.degrees(difference))


def validate_start_alignment(
    report: ValidationReport,
    manifest: dict[str, Any],
    root: ET.Element | None,
    points: list[dict[str, float]],
) -> None:
    if root is None or not points:
        report.add("SKIP", "start alignment unavailable")
        return
    pose = ego_initial_pose(root)
    if pose is None:
        report.add("SKIP", "Ego absolute initial pose unavailable")
        return
    validation = manifest.get("validation", {})
    distance_tolerance = validation.get("start_position_tolerance_m")
    heading_tolerance = validation.get("start_heading_tolerance_deg")
    if not is_number(distance_tolerance) or not is_number(heading_tolerance):
        report.add("SKIP", "start alignment tolerances unavailable")
        return

    first = points[0]
    distance = math.hypot(pose[0] - first["x"], pose[1] - first["y"])
    distance_limit = float(distance_tolerance)
    if distance <= distance_limit:
        report.add(
            "PASS",
            f"start distance: {distance:.3f} m <= {distance_limit:.3f} m",
        )
    else:
        report.add(
            "FAIL",
            f"start distance: {distance:.3f} m > {distance_limit:.3f} m",
        )

    heading_difference = angle_difference_degrees(pose[2], first["yaw"])
    heading_limit = float(heading_tolerance)
    if heading_difference <= heading_limit:
        report.add(
            "PASS",
            f"heading difference: {heading_difference:.3f} deg "
            f"<= {heading_limit:.3f} deg",
        )
    else:
        report.add(
            "FAIL",
            f"heading difference: {heading_difference:.3f} deg "
            f"> {heading_limit:.3f} deg",
        )


def validate_naming(
    report: ValidationReport,
    manifest: dict[str, Any],
) -> None:
    scenario = manifest.get("scenario", {})
    scenario_id = scenario.get("id")
    scenario_file = scenario.get("file")
    if not non_empty_string(scenario_id):
        return
    if not SCENARIO_ID_PATTERN.fullmatch(scenario_id):
        report.add("FAIL", f"invalid scenario id format: {scenario_id}")
    else:
        report.add("PASS", f"scenario id format: {scenario_id}")
    expected_prefix = f"{scenario_id}_"
    if not report.package.name.startswith(expected_prefix):
        report.add("FAIL", f"package name must start with {expected_prefix!r}")
    else:
        report.add("PASS", f"package naming: {report.package.name}")
    if non_empty_string(scenario_file):
        if not Path(scenario_file).stem.startswith(expected_prefix):
            report.add(
                "FAIL",
                f"scenario file name must start with {expected_prefix!r}",
            )
        else:
            report.add("PASS", f"scenario file naming: {scenario_file}")
    category = scenario.get("category")
    if non_empty_string(category) and report.package.parent.name != category:
        report.add(
            "FAIL",
            f"package category is {report.package.parent.name!r}, "
            f"expected {category!r}",
        )


def validate_catalog(
    report: ValidationReport,
    manifest: dict[str, Any],
) -> None:
    if not CATALOG_PATH.is_file():
        report.add("FAIL", f"catalog missing: {CATALOG_PATH}")
        return
    try:
        with CATALOG_PATH.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            required = {
                "scenario_id",
                "category",
                "package_path",
                "eval_codes",
                "status",
                "notes",
            }
            if set(reader.fieldnames or []) != required:
                report.add("FAIL", "catalog header does not match Phase 1 schema")
                return
            rows = list(reader)
    except OSError as error:
        report.add("FAIL", f"cannot read catalog: {error}")
        return

    identifiers: set[str] = set()
    duplicates: set[str] = set()
    for row in rows:
        identifier = row["scenario_id"].strip()
        if identifier in identifiers:
            duplicates.add(identifier)
        identifiers.add(identifier)
        package_path = row["package_path"].strip()
        status = row["status"].strip()
        if package_path:
            catalog_package = (SCENARIO_ROOT / package_path).resolve()
            if not catalog_package.is_dir():
                report.add(
                    "FAIL",
                    f"catalog package_path missing for {identifier}: {package_path}",
                )
        elif status != "planned":
            report.add(
                "FAIL",
                f"catalog package_path required for {identifier} status={status}",
            )
    if duplicates:
        report.add(
            "FAIL",
            "duplicate scenario_id in catalog: " + ", ".join(sorted(duplicates)),
        )
        return

    scenario_id = manifest.get("scenario", {}).get("id")
    matching = [row for row in rows if row["scenario_id"].strip() == scenario_id]
    if len(matching) != 1:
        report.add("FAIL", f"catalog must contain exactly one row for {scenario_id}")
        return
    expected_package = (SCENARIO_ROOT / matching[0]["package_path"]).resolve()
    if expected_package != report.package:
        report.add("FAIL", f"catalog package_path mismatch for {scenario_id}")
        return
    report.add("PASS", f"catalog entry: {scenario_id}")


def validate_scenario(package_path: Path) -> ValidationReport:
    package = package_path.expanduser().resolve()
    report = ValidationReport(package=package)
    if not package.is_dir():
        report.tool_error = True
        report.add("FAIL", f"package directory not found: {package}")
        return report

    manifest_path = package / "scenario.yaml"
    if not manifest_path.is_file():
        report.add("FAIL", f"manifest missing: {manifest_path}")
        return report
    try:
        manifest = load_manifest(manifest_path)
    except RuntimeError as error:
        report.tool_error = True
        report.add("FAIL", str(error))
        return report
    except ValueError as error:
        report.add("FAIL", str(error))
        return report
    report.manifest = manifest
    report.add("PASS", "manifest: scenario.yaml")

    validate_manifest_structure(report, manifest)
    validate_entities_and_events(report, manifest)
    validate_naming(report, manifest)
    root = parse_scenario_xml(report, manifest)
    if root is not None:
        validate_xml_entities(report, manifest, root)
    route_file = resolve_packaged_route(report, manifest)
    points = parse_route_csv(report, route_file) if route_file else []
    validate_start_alignment(report, manifest, root, points)
    validate_catalog(report, manifest)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a Phase 1 Scenario Package."
    )
    parser.add_argument(
        "package",
        type=Path,
        help="Package directory containing scenario.yaml, XML, and route.csv",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = validate_scenario(args.package)
    report.print()
    return report.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
