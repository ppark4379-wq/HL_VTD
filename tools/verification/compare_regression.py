#!/usr/bin/env python3
"""Compare execution health only; never run VTD or modify input campaigns.

Reads run_regression.py's summary.csv and run_scenario.py's per-scenario run.csv.
goal_reached is true if any valid logged frame has 1/true. Missing, empty,
malformed or mismatched logs are UNAVAILABLE, never assumed false.

Precedence: definite REGRESSION, insufficient data INCONCLUSIVE, goal improvement
IMPROVED, otherwise SAME. Both executions must be COMPLETE with exit code 0 for
SAME/IMPROVED. A failed baseline cannot establish a healthy execution reference.

Exit codes: 0 SAME/IMPROVED only; 1 any REGRESSION; 2 input/output error;
3 INCONCLUSIVE without a definite regression. Only regression_summary.csv is
written, in the current campaign directory. Official assessment is not read.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import os
from pathlib import Path
import re
import sys
import tempfile

from run_regression import OUTPUT_ROOT


FIELDS = (
    "scenario_id", "baseline_execution_status", "current_execution_status",
    "baseline_exit_code", "current_exit_code", "baseline_log_exists",
    "current_log_exists", "baseline_goal_reached", "current_goal_reached",
    "result", "reason", "baseline_campaign_id", "current_campaign_id",
)
FAILURE_STATUSES = {
    "TIMEOUT", "CONTROLLER_ERROR", "AUTOMATION_ERROR", "VALIDATION_ERROR",
    "RUNNER_ERROR", "LAUNCH_ERROR", "CATALOG_ERROR", "PACKAGE_ERROR", "INTERRUPTED",
}
UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class Execution:
    status: str
    exit_code: int | None

    @property
    def complete(self) -> bool:
        return self.status == "COMPLETE" and self.exit_code == 0

    @property
    def failed(self) -> bool:
        return self.status in FAILURE_STATUSES or self.exit_code not in (None, 0)


@dataclass(frozen=True)
class LogHealth:
    exists: bool | None
    goal: bool | None = None
    issue: str = ""


def campaign_directory(root: Path, campaign_id: str) -> Path:
    # Generated REG IDs use letters/digits/_/-. Exclude separators, dots,
    # drive prefixes and Windows alternate data streams before joining paths.
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", campaign_id):
        raise ValueError(f"invalid Campaign ID: {campaign_id!r}")
    parent = (root / "regression").resolve()
    directory = parent / campaign_id
    if directory.resolve() != directory:
        raise ValueError(f"Campaign directory must not redirect to another path: {campaign_id}")
    if not directory.is_dir():
        raise ValueError(f"Campaign not found: {campaign_id} ({directory})")
    return directory


def read_executions(directory: Path) -> dict[str, Execution]:
    """One parser, reused for both run_regression summary.csv inputs."""
    path = directory / "summary.csv"
    if not path.is_file():
        raise ValueError(f"Campaign execution summary not found: {path}")
    executions = {}
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, strict=True)
        fields = reader.fieldnames or []
        if (not {"scenario_id", "status", "exit_code"}.issubset(fields)
                or len(fields) != len(set(fields))):
            raise ValueError(f"invalid execution summary columns: {path}")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError(f"incomplete execution summary row {reader.line_num}: {path}")
            scenario_id = row["scenario_id"].strip()
            if not re.fullmatch(r"[A-Z][0-9]{2}(?:-[0-9]{2})?", scenario_id):
                raise ValueError(f"invalid scenario ID {scenario_id!r} in {path}")
            if scenario_id in executions:
                raise ValueError(f"duplicate scenario ID {scenario_id} in {path}")
            code = row["exit_code"].strip()
            if code and not re.fullmatch(r"-?[0-9]+", code):
                raise ValueError(f"invalid exit code {code!r} for {scenario_id} in {path}")
            executions[scenario_id] = Execution(
                row["status"].strip() or UNAVAILABLE, int(code) if code else None
            )
    return executions


def read_log(root: Path, scenario_id: str, campaign_id: str) -> LogHealth:
    path = root / scenario_id / campaign_id / "run.csv"
    if not path.resolve().is_relative_to(root.resolve()):
        return LogHealth(None, issue="run.csv redirects outside verification directory")
    try:
        stream = path.open(encoding="utf-8-sig", newline="")
    except FileNotFoundError:
        return LogHealth(False, issue="run.csv missing")
    except OSError as error:
        return LogHealth(None, issue=f"run.csv unreadable: {error}")
    try:
        with stream:
            reader = csv.DictReader(stream, strict=True)
            fields = reader.fieldnames or []
            if not fields:
                return LogHealth(True, issue="run.csv empty")
            if len(fields) != len(set(fields)):
                return LogHealth(True, issue="run.csv duplicate columns")
            if "goal_reached" not in fields:
                return LogHealth(True, issue="goal_reached column missing")
            goal = False
            count = 0
            for row in reader:
                count += 1
                if None in row or any(v is None for v in row.values()):
                    return LogHealth(True, issue="run.csv malformed or truncated row")
                # Identity columns exist in the real logger. Older logs without
                # them remain usable, but mismatches must not look like evidence.
                for field, expected in (("scenario_id", scenario_id), ("run_id", campaign_id)):
                    if field in row and row[field].strip() != expected:
                        return LogHealth(True, issue=f"run.csv {field} mismatch")
                value = row["goal_reached"].strip().lower()
                if value not in {"0", "1", "false", "true"}:
                    return LogHealth(True, issue="goal_reached value invalid or missing")
                goal = goal or value in {"1", "true"}
            if not count:
                return LogHealth(True, issue="run.csv has no data rows")
            return LogHealth(True, goal)
    except (OSError, UnicodeError, csv.Error) as error:
        return LogHealth(True, issue=f"run.csv cannot be parsed: {error}")


def decide(baseline: Execution | None, current: Execution | None,
           baseline_log: LogHealth, current_log: LogHealth) -> tuple[str, str]:
    if baseline is None:
        return "INCONCLUSIVE", "scenario only in current; no baseline reference"
    if current is None:
        return "REGRESSION", "scenario missing from current campaign summary"
    regressions = []
    if baseline.complete and current.failed:
        regressions.append(
            f"execution {baseline.status} -> {current.status} "
            f"(exit {baseline.exit_code} -> {current.exit_code})"
        )
    if baseline_log.exists is True and current_log.exists is False:
        regressions.append("run.csv present -> missing")
    if baseline_log.goal is True and current_log.goal is False:
        regressions.append("goal_reached true -> false")
    if regressions:
        return "REGRESSION", "; ".join(regressions)

    unavailable = []
    if not baseline.complete:
        unavailable.append(f"baseline execution is not COMPLETE/0 ({baseline.status}, exit={baseline.exit_code})")
    if not current.complete:
        unavailable.append(f"current execution is not COMPLETE/0 ({current.status}, exit={current.exit_code})")
    for name, log in (("baseline", baseline_log), ("current", current_log)):
        if log.goal is None:
            unavailable.append(f"{name}: {log.issue or 'goal_reached unavailable'}")
    if unavailable:
        return "INCONCLUSIVE", "; ".join(unavailable)
    if baseline_log.goal is False and current_log.goal is True:
        return "IMPROVED", "goal_reached false -> true"
    return "SAME", ""


def flag(value: bool | None) -> str:
    return UNAVAILABLE if value is None else str(value).lower()


def compare_campaigns(root: Path, baseline_id: str, current_id: str) -> tuple[Path, list[dict]]:
    if baseline_id.casefold() == current_id.casefold():
        raise ValueError("Baseline and Current must be different Campaign IDs")
    baseline_dir = campaign_directory(root, baseline_id)
    current_dir = campaign_directory(root, current_id)
    if baseline_dir.samefile(current_dir):
        raise ValueError("Baseline and Current resolve to the same directory")
    baseline = read_executions(baseline_dir)
    current = read_executions(current_dir)
    if not baseline and not current:
        raise ValueError("Both campaign summaries contain no scenarios")
    results = []
    # Baseline order first, then newly added scenarios in current order.
    for scenario_id in dict.fromkeys([*baseline, *current]):
        before, after = baseline.get(scenario_id), current.get(scenario_id)
        before_log = read_log(root, scenario_id, baseline_id)
        after_log = read_log(root, scenario_id, current_id)
        result, reason = decide(before, after, before_log, after_log)
        results.append(dict(
            scenario_id=scenario_id,
            baseline_execution_status=before.status if before else "MISSING",
            current_execution_status=after.status if after else "MISSING",
            baseline_exit_code=before.exit_code if before and before.exit_code is not None else UNAVAILABLE,
            current_exit_code=after.exit_code if after and after.exit_code is not None else UNAVAILABLE,
            baseline_log_exists=flag(before_log.exists), current_log_exists=flag(after_log.exists),
            baseline_goal_reached=flag(before_log.goal), current_goal_reached=flag(after_log.goal),
            result=result, reason=reason, baseline_campaign_id=baseline_id, current_campaign_id=current_id,
        ))
    return current_dir / "regression_summary.csv", results


def write_results(path: Path, results: list[dict]) -> None:
    # Replace the report atomically: a hardlink to an input cannot overwrite
    # that input's bytes. Explicitly reject symbolic-link output destinations.
    if path.is_symlink():
        raise ValueError(f"comparison report must not be a symbolic link: {path}")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8-sig", newline="",
                                         dir=path.parent, suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(results)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--current", required=True)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT,
                        help="Existing verification output root (default: output/verification)")
    args = parser.parse_args(argv)
    try:
        path, results = compare_campaigns(args.output_root.resolve(), args.baseline, args.current)
        write_results(path, results)
    except (OSError, ValueError, csv.Error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print("=" * 60)
    print(f"Regression Comparison\nBaseline : {args.baseline}\nCurrent  : {args.current}")
    print("=" * 60)
    for row in results:
        print(f"{row['scenario_id']}  {row['result']}  {row['reason']}".rstrip())
    print("-" * 60)
    counts = {status: sum(r["result"] == status for r in results)
              for status in ("SAME", "IMPROVED", "REGRESSION", "INCONCLUSIVE")}
    for status, count in counts.items():
        print(f"{status:<13}: {count}")
    print("-" * 60)
    print(f"Report: {path}")
    if counts["REGRESSION"]:
        print(f"*** REGRESSION DETECTED: {counts['REGRESSION']} scenario(s) ***")
        return 1
    return 3 if counts["INCONCLUSIVE"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
