#!/usr/bin/env python3
# Validate one Phase 1 Scenario Package and start HL_VTD with its default route.

from __future__ import annotations

import argparse
import re
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from validate_scenario import REPO_ROOT, validate_scenario


RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")


def default_run_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a Scenario Package and run HL_VTD with route.default."
        )
    )
    parser.add_argument(
        "package",
        type=Path,
        help="Phase 1 Scenario Package directory",
    )
    parser.add_argument(
        "--run-id",
        default=None,
        help="Logger run ID (letters, digits, dot, underscore, hyphen)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved files and command without starting HL_VTD",
    )
    return parser.parse_args()


def build_command(
    binary: Path,
    route_file: Path,
    scenario_id: str,
    run_id: str,
    log_file: Path,
) -> list[str]:
    return [
        str(binary),
        "--path",
        str(route_file),
        "--scenario-id",
        scenario_id,
        "--run-id",
        run_id,
        "--log",
        str(log_file),
    ]


def main() -> int:
    args = parse_args()
    report = validate_scenario(args.package)
    report.print()
    if report.exit_code != 0:
        print(
            "[run_scenario] validation failed; HL_VTD was not started.",
            file=sys.stderr,
        )
        return report.exit_code
    if report.manifest is None or report.scenario_file is None:
        print("[run_scenario] validated Scenario metadata is missing.", file=sys.stderr)
        return 2
    if report.route_file is None:
        print("[run_scenario] validated default route is missing.", file=sys.stderr)
        return 2

    scenario_id = report.manifest["scenario"]["id"]
    run_id = args.run_id or default_run_id()
    if not RUN_ID_PATTERN.fullmatch(run_id):
        print(
            "[run_scenario] --run-id may contain only letters, digits, "
            "dot, underscore, and hyphen.",
            file=sys.stderr,
        )
        return 2

    binary = (REPO_ROOT / "build" / "hl_vtd").resolve()
    log_file = (
        REPO_ROOT
        / "output"
        / "verification"
        / scenario_id
        / run_id
        / "run.csv"
    ).resolve()
    command = build_command(
        binary,
        report.route_file,
        scenario_id,
        run_id,
        log_file,
    )

    print()
    print(f"Scenario : {scenario_id}")
    print(f"File     : {report.scenario_file}")
    print(f"Route    : {report.route_file}")
    print(f"Log      : {log_file}")
    print(f"Command  : {shlex.join(command)}")
    print("VTD      : Load the Scenario file manually; GUI Load/Start is not automated.")

    if args.dry_run:
        print("[run_scenario] dry-run; HL_VTD was not started.")
        return 0
    if not binary.is_file():
        print(
            f"[run_scenario] HL_VTD executable not found: {binary}",
            file=sys.stderr,
        )
        return 2

    log_file.parent.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
