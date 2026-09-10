#!/usr/bin/env python3
"""Run a fixed YAML suite through run_scenario.py, without driving assessment.

Usage:
    python tools/verification/run_regression.py
    python tools/verification/run_regression.py verification/suites/m_all.yaml
    python tools/verification/run_regression.py --dry-run

Execution logs and summary.csv:
    output/verification/regression/<campaign_id>/
Scenario artifacts (owned by run_scenario.py):
    output/verification/<scenario_id>/<campaign_id>/

Exit codes: 0 all completed, 1 any execution failure, 2 invalid input/setup,
130 user interruption. No Scenario Registry or driving PASS/FAIL is consulted.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("run_scenario.py")
OUTPUT_ROOT = REPO_ROOT / "output/verification"
RESULT_COLUMNS = ("scenario_id", "status", "exit_code", "duration_s", "runner_log")


def load_suite(path: Path) -> dict:
    with path.open(encoding="utf-8-sig") as stream:
        suite = yaml.safe_load(stream)
    if not isinstance(suite, dict):
        raise ValueError("suite must be a YAML mapping")
    if not isinstance(suite.get("name"), str) or not suite["name"].strip():
        raise ValueError("suite.name must be a non-empty string")
    scenarios = suite.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("suite.scenarios must be a non-empty list")
    if any(not isinstance(s, str) or not re.fullmatch(r"[A-Z][0-9]{2}(?:-[0-9]{2})?", s)
           for s in scenarios):
        raise ValueError("suite.scenarios must contain scenario IDs such as M01")
    if len(set(scenarios)) != len(scenarios):
        raise ValueError("suite.scenarios contains duplicate IDs")
    if not isinstance(suite.get("continue_on_failure"), bool):
        raise ValueError("continue_on_failure must be true or false")
    timeout = suite.get("run_timeout_s")
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout) or timeout <= 0):
        raise ValueError("run_timeout_s must be a positive finite number")
    return suite


def load_catalog(path: Path) -> dict[str, Path]:
    packages = {}
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"scenario_id", "package_path"}.issubset(reader.fieldnames or []):
            raise ValueError("catalog requires scenario_id and package_path columns")
        for row in reader:
            scenario_id = (row.get("scenario_id") or "").strip()
            package_path = (row.get("package_path") or "").strip()
            if not scenario_id or not package_path:
                raise ValueError("catalog contains an empty ID or package_path")
            if scenario_id in packages:
                raise ValueError(f"duplicate catalog ID: {scenario_id}")
            packages[scenario_id] = (path.parent / package_path).resolve()
    return packages


def build_command(package: Path, campaign_id: str, timeout: float,
                  config: Path | None = None) -> list[str]:
    command = [
        sys.executable, "-u", str(RUNNER), str(package),
        "--run-id", campaign_id, "--auto-vtd", "--run-timeout", f"{timeout:g}",
    ]
    if config is not None:
        command.extend(["--config", str(config.resolve())])
    return command


def execution_status(exit_code: int, output: str) -> str:
    # Code 2 is shared by multiple runner errors. Match its final diagnostic,
    # not arbitrary controller output, assessment text or recoverable warnings.
    if exit_code == 0:
        return "COMPLETE"
    if exit_code == 130:
        return "INTERRUPTED"
    errors = [line for line in output.splitlines() if line.startswith("[run_scenario]")]
    diagnostic = errors[-1] if errors else ""
    if "scenario exceeded --run-timeout" in diagnostic:
        return "TIMEOUT"
    if "validation failed;" in diagnostic:
        return "VALIDATION_ERROR"
    if "controller launch failed:" in diagnostic or "HL_VTD exited before Play" in diagnostic:
        return "CONTROLLER_ERROR"
    if "automation failed:" in diagnostic:
        return "AUTOMATION_ERROR"
    if diagnostic:
        return "RUNNER_ERROR"
    if "[auto-vtd] VTD is RUNNING; CSV logging is active" in output:
        return "CONTROLLER_ERROR"
    return "RUNNER_ERROR"


def run_one(command: list[str], log_path: Path) -> tuple[str, int | None]:
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    with log_path.open("wb") as log:
        try:
            # Do not impose subprocess timeout=run_timeout_s: that would kill
            # the runner before its Controller termination / VTD Stop cleanup.
            completed = subprocess.run(
                command, cwd=REPO_ROOT, env=env, stdout=log,
                stderr=subprocess.STDOUT, check=False,
            )
        except OSError as error:
            log.write(f"Cannot launch runner: {error}\n".encode("utf-8"))
            return "LAUNCH_ERROR", None
    return execution_status(
        completed.returncode, log_path.read_text(encoding="utf-8", errors="replace")
    ), completed.returncode


def print_summary(campaign_id: str, results: list[dict], total: int) -> None:
    print("\n" + "=" * 40)
    print(f"Regression Campaign: {campaign_id}")
    print("=" * 40)
    for result in results:
        print(f"{result['scenario_id']}  {result['status']}  (exit={result['exit_code']})")
    completed = sum(r["status"] == "COMPLETE" for r in results)
    print("-" * 40)
    print(f"Total      : {total}")
    print(f"Completed  : {completed}")
    print(f"Failed     : {len(results) - completed}")
    if total != len(results):
        print(f"Not run    : {total - len(results)}")
    print("-" * 40)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("suite", nargs="?", type=Path,
                        default=REPO_ROOT / "verification/suites/m_all.yaml")
    parser.add_argument("--catalog", type=Path,
                        default=REPO_ROOT / "scenario/scenario_catalog.csv")
    parser.add_argument("--config", type=Path, help="Network config forwarded to run_scenario.py")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print commands only; do not invoke runner, VTD, or write output")
    args = parser.parse_args(argv)
    try:
        suite = load_suite(args.suite)
        packages = load_catalog(args.catalog)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    campaign_id = datetime.now().strftime("REG_%Y%m%d_%H%M%S")
    if args.dry_run:
        print(f"Dry run: {suite['name']} / {campaign_id}", flush=True)
        missing = False
        for scenario_id in suite["scenarios"]:
            if scenario_id not in packages:
                print(f"{scenario_id}: CATALOG_ERROR")
                missing = True
                continue
            command = build_command(packages[scenario_id], campaign_id,
                                    suite["run_timeout_s"], args.config)
            print(f"{scenario_id}: {subprocess.list2cmdline(command)}")
        return 2 if missing else 0

    # Reserve an unused campaign directory, including same-second invocations.
    directory = OUTPUT_ROOT / "regression" / campaign_id
    try:
        try:
            directory.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            campaign_id += "_" + datetime.now().strftime("%f")
            directory = directory.with_name(campaign_id)
            directory.mkdir(parents=True, exist_ok=False)
        summary = (directory / "summary.csv").open("w", encoding="utf-8", newline="")
    except OSError as error:
        print(f"ERROR: cannot create campaign output: {error}", file=sys.stderr)
        return 2

    print(f"Regression Campaign: {campaign_id} ({suite['name']})", flush=True)
    print(f"Results: {directory}", flush=True)
    results = []
    interrupted = False
    with summary:
        writer = csv.DictWriter(summary, fieldnames=RESULT_COLUMNS)
        writer.writeheader()
        summary.flush()
        for index, scenario_id in enumerate(suite["scenarios"], 1):
            print(f"[{index}/{len(suite['scenarios'])}] {scenario_id} RUNNING", flush=True)
            started = time.monotonic()
            log_path = directory / f"{scenario_id}.runner.log"
            try:
                if scenario_id not in packages:
                    status, exit_code = "CATALOG_ERROR", None
                elif not packages[scenario_id].is_dir():
                    status, exit_code = "PACKAGE_ERROR", None
                else:
                    status, exit_code = run_one(
                        build_command(packages[scenario_id], campaign_id,
                                      suite["run_timeout_s"], args.config), log_path
                    )
            except KeyboardInterrupt:
                status, exit_code = "INTERRUPTED", 130
            except OSError as error:
                print(f"{scenario_id}: {error}", file=sys.stderr)
                status, exit_code = "RUNNER_ERROR", None
            result = dict(scenario_id=scenario_id, status=status, exit_code=exit_code,
                          duration_s=f"{time.monotonic() - started:.3f}",
                          runner_log=log_path.name if log_path.exists() else "")
            results.append(result)
            writer.writerow(result)
            summary.flush()
            print(f"{scenario_id} {status} (exit={exit_code})", flush=True)
            if status == "INTERRUPTED":
                interrupted = True
                break
            if status != "COMPLETE" and not suite["continue_on_failure"]:
                break

    print_summary(campaign_id, results, len(suite["scenarios"]))
    print(f"Summary: {directory / 'summary.csv'}")
    if interrupted:
        return 130
    return 0 if all(r["status"] == "COMPLETE" for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
