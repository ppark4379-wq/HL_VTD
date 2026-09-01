#!/usr/bin/env python3
"""Render manifest parameters into a temporary VTD Scenario XML."""

from __future__ import annotations

import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


class ParameterError(ValueError):
    pass


def _parse_value(name: str, definition: dict[str, Any], raw: str) -> Any:
    type_name = definition.get("type")
    try:
        if type_name == "float":
            value: Any = float(raw)
        elif type_name == "int":
            value = int(raw)
        elif type_name == "bool":
            lowered = raw.lower()
            if lowered not in {"true", "false"}:
                raise ValueError
            value = lowered == "true"
        elif type_name == "string":
            value = raw
        else:
            raise ParameterError(f"parameter {name}: unsupported type {type_name!r}")
    except ValueError as error:
        raise ParameterError(
            f"parameter {name}: {raw!r} is not a valid {type_name}"
        ) from error
    if isinstance(value, float) and not math.isfinite(value):
        raise ParameterError(f"parameter {name}: value must be finite")
    return value


def resolve_parameters(
    manifest: dict[str, Any], overrides: list[str], randomize: bool, seed: int | None
) -> dict[str, Any]:
    definitions = manifest.get("parameters", {})
    values = {name: definition["default"] for name, definition in definitions.items()}
    if randomize:
        generator = random.Random(seed)
        for name, definition in definitions.items():
            random_spec = definition.get("random")
            if not isinstance(random_spec, dict):
                continue
            minimum = random_spec.get("min")
            maximum = random_spec.get("max")
            if not isinstance(minimum, (int, float)) or not isinstance(
                maximum, (int, float)
            ):
                raise ParameterError(f"parameter {name}: random min/max are required")
            if minimum > maximum:
                raise ParameterError(f"parameter {name}: random min exceeds max")
            if definition.get("type") == "int":
                values[name] = generator.randint(int(minimum), int(maximum))
            else:
                values[name] = generator.uniform(float(minimum), float(maximum))

    for item in overrides:
        if "=" not in item:
            raise ParameterError(f"--param requires NAME=VALUE: {item!r}")
        name, raw = item.split("=", 1)
        if name not in definitions:
            raise ParameterError(f"unknown parameter: {name}")
        values[name] = _parse_value(name, definitions[name], raw)
    return values


def _player(root: ET.Element, name: str) -> ET.Element:
    for player in root.iter("Player"):
        description = player.find("Description")
        if description is not None and description.get("Name") == name:
            return player
    raise ParameterError(f"VTD Player not found: {name}")


def render_scenario(
    source: Path,
    destination: Path,
    manifest: dict[str, Any],
    values: dict[str, Any],
) -> dict[str, Any]:
    bindings = manifest.get("runtime_bindings", [])
    if not isinstance(bindings, list):
        raise ParameterError("runtime_bindings must be an array")
    tree = ET.parse(source)
    root = tree.getroot()
    applied: dict[str, Any] = {}
    for binding in bindings:
        if not isinstance(binding, dict):
            raise ParameterError("runtime binding must be a mapping")
        parameter = binding.get("parameter")
        target = binding.get("target")
        if parameter not in values or not isinstance(target, dict):
            raise ParameterError(f"invalid runtime binding for {parameter!r}")
        if target.get("type") != "player_init_speed":
            raise ParameterError(
                f"parameter {parameter}: unsupported target {target.get('type')!r}"
            )
        player_name = target.get("player")
        player = _player(root, player_name)
        init = player.find("Init")
        speed = init.find("Speed") if init is not None else None
        if speed is None:
            raise ParameterError(f"Player {player_name}: Init/Speed not found")
        value = float(values[parameter])
        if value < 0.0 or not math.isfinite(value):
            raise ParameterError(f"parameter {parameter}: speed must be non-negative")
        speed.set("Value", f"{value:.16e}")
        applied[parameter] = value

    destination.parent.mkdir(parents=True, exist_ok=True)
    xml_body = ET.tostring(root, encoding="unicode")
    destination.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        "<!DOCTYPE Scenario>\n"
        + xml_body
        + "\n",
        encoding="utf-8",
    )
    return applied
