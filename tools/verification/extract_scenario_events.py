#!/usr/bin/env python3
"""Inventory VTD native mixed scenarios registered in scenario_catalog.csv.

See verification/README.md for the exact meaning of each presence flag.
Uses only the Python standard library; does not modify source scenarios.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COLUMNS = (
    "Scenario", "Vehicle", "Pedestrian", "Static obstacle",
    "Lane change", "Speed change", "Traffic light", "Trigger",
)
CHECK = "\u2713"
# VTD native condition elements observed under Action, not Init/waypoints.
TRIGGER_TAGS = {"PosRelative", "PosAbsolute"}
OBJECT_MOVEMENT_TAGS = {"Motion", "ObjectPath", "Path", "SpeedChange", "LaneChange"}


def extract_events(root: ET.Element, ego_name: str = "Ego") -> dict[str, bool]:
    """Read explicit XML definitions, including namespace-qualified exports."""
    for element in root.iter():
        element.tag = element.tag.rsplit("}", 1)[-1]
    if root.tag != "Scenario":
        raise ValueError(f"expected VTD native Scenario root, found {root.tag!r}")

    players = root.findall("./TrafficControl/Player/Description")
    moving = root.find("MovingObjectsControl")
    characters = [] if moving is None else moving.findall("Character")
    objects = [] if moving is None else moving.findall("Object")
    moving_objects = set()
    if moving is not None:
        for actions in moving.findall("ObjectActions"):
            if any(e.tag in OBJECT_MOVEMENT_TAGS for e in actions.iter()):
                moving_objects.add(actions.get("Object"))

    actions = root.findall(".//Action")
    event_tags = {child.tag for action in actions for child in action}
    return {
        "Vehicle": any(p.get("Name") != ego_name for p in players),
        "Pedestrian": any(c.get("Class", "").lower() == "pedestrian" for c in characters),
        "Static obstacle": any(o.get("Name") not in moving_objects for o in objects),
        "Lane change": "LaneChange" in event_tags,
        "Speed change": "SpeedChange" in event_tags,
        "Traffic light": bool(
            root.findall("./LightSigns/SignalController")
            or root.findall("./LightSigns/Signal")
        ),
        "Trigger": bool(event_tags & TRIGGER_TAGS),
    }


def read_catalog(catalog: Path) -> list[tuple[str, Path]]:
    """Resolve each catalog package to its sole XML, rejecting ambiguity."""
    scenarios = []
    seen = set()
    with catalog.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"scenario_id", "package_path"}.issubset(reader.fieldnames or []):
            raise ValueError("catalog requires scenario_id and package_path columns")
        for row in reader:
            scenario_id = (row.get("scenario_id") or "").strip()
            if not re.fullmatch(r"M\d{2}", scenario_id):
                continue
            if scenario_id in seen:
                raise ValueError(f"duplicate catalog scenario: {scenario_id}")
            seen.add(scenario_id)
            package_path = (row.get("package_path") or "").strip()
            if not package_path:
                raise ValueError(f"{scenario_id}: empty package_path")
            package = catalog.parent / package_path
            candidates = sorted(package.glob("*.xml"))
            if len(candidates) != 1:
                raise ValueError(
                    f"{scenario_id}: expected exactly one XML in {package}; "
                    f"found {len(candidates)}"
                )
            scenarios.append((scenario_id, candidates[0]))
    if not scenarios:
        raise ValueError("no M scenarios found in catalog")
    return sorted(scenarios)


def build_registry(catalog: Path, ego_name: str = "Ego") -> list[dict[str, str]]:
    rows = []
    for scenario_id, xml_path in read_catalog(catalog):
        try:
            flags = extract_events(ET.parse(xml_path).getroot(), ego_name)
        except (ET.ParseError, ValueError) as error:
            raise ValueError(f"{scenario_id} ({xml_path}): {error}") from error
        rows.append({"Scenario": scenario_id, **{
            name: CHECK if present else "" for name, present in flags.items()
        }})
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path,
                        default=REPO_ROOT / "scenario/scenario_catalog.csv")
    parser.add_argument("--output", type=Path,
                        default=REPO_ROOT / "verification/m_scenario_registry.csv")
    parser.add_argument("--ego-name", default="Ego", help="vehicle name to exclude")
    args = parser.parse_args(argv)
    try:
        # Parse every source before opening the output: errors cannot produce
        # a partial registry or replace an existing report with false blanks.
        rows = build_registry(args.catalog, args.ego_name)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=COLUMNS, lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)
    except (OSError, ValueError, csv.Error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    registered = {row["Scenario"] for row in rows}
    missing = [f"M{i:02d}" for i in range(1, 21) if f"M{i:02d}" not in registered]
    if missing:
        print("WARNING: not registered in catalog: " + ", ".join(missing), file=sys.stderr)
    print(f"Wrote {len(rows)} scenarios to {args.output}")
    print(" | ".join(COLUMNS))
    for row in rows:
        # ASCII preview remains readable in Windows terminals using legacy encodings.
        print(" | ".join(row[c].replace(CHECK, "yes") or "-" for c in COLUMNS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
