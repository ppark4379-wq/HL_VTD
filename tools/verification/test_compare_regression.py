"""Offline fixtures using the existing campaign and logger CSV formats."""

import contextlib
import csv
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import compare_regression as compare
from run_regression import RESULT_COLUMNS


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.baseline = "REG_20260910_001"
        self.current = "REG_20260910_002"
        self.campaign(self.baseline)
        self.campaign(self.current)

    def campaign(self, campaign_id, scenarios=None):
        scenarios = scenarios if scenarios is not None else [("M01", "COMPLETE", 0)]
        directory = self.root / "regression" / campaign_id
        directory.mkdir(parents=True, exist_ok=True)
        with (directory / "summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=RESULT_COLUMNS)
            writer.writeheader()
            for scenario_id, status, code in scenarios:
                writer.writerow(dict(scenario_id=scenario_id, status=status, exit_code=code,
                                     duration_s="12.345", runner_log=f"{scenario_id}.runner.log"))

    def log(self, campaign_id, values=(0, 1), scenario_id="M01", extra=None):
        path = self.root / scenario_id / campaign_id / "run.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = ["scenario_id", "run_id", "frame", "goal_reached", *(extra or {})]
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index, value in enumerate(values):
                writer.writerow(dict(scenario_id=scenario_id, run_id=campaign_id,
                                     frame=index, goal_reached=value, **(extra or {})))
        return path

    def rows(self):
        return compare.compare_campaigns(self.root, self.baseline, self.current)[1]

    def pair(self, before=(0, 1), after=(0, 1)):
        self.log(self.baseline, before)
        self.log(self.current, after)

    def test_complete_and_goal_true_same(self):
        self.pair()
        row = self.rows()[0]
        self.assertEqual(row["result"], "SAME")
        self.assertEqual(row["baseline_goal_reached"], "true")
        self.assertEqual(row["baseline_exit_code"], 0)

    def test_complete_to_timeout(self):
        self.pair()
        self.campaign(self.current, [("M01", "TIMEOUT", 2)])
        row = self.rows()[0]
        self.assertEqual(row["result"], "REGRESSION")
        self.assertIn("COMPLETE -> TIMEOUT", row["reason"])

    def test_complete_to_controller_error(self):
        self.pair()
        self.campaign(self.current, [("M01", "CONTROLLER_ERROR", 7)])
        row = self.rows()[0]
        self.assertEqual(row["result"], "REGRESSION")
        self.assertEqual(row["current_exit_code"], 7)

    def test_goal_true_to_false(self):
        self.pair(after=(0, 0))
        self.assertEqual(self.rows()[0]["result"], "REGRESSION")
        self.assertIn("goal_reached true -> false", self.rows()[0]["reason"])

    def test_goal_false_to_true(self):
        self.pair(before=(0, 0))
        self.assertEqual(self.rows()[0]["result"], "IMPROVED")

    def test_current_log_missing(self):
        self.log(self.baseline)
        row = self.rows()[0]
        self.assertEqual(row["result"], "REGRESSION")
        self.assertEqual(row["current_log_exists"], "false")
        self.assertEqual(row["current_goal_reached"], "UNAVAILABLE")

    def test_baseline_log_missing(self):
        self.log(self.current)
        self.assertEqual(self.rows()[0]["result"], "INCONCLUSIVE")

    def test_new_current_scenario(self):
        self.pair()
        self.campaign(self.current, [("M01", "COMPLETE", 0), ("M18", "COMPLETE", 0)])
        self.log(self.current, scenario_id="M18")
        row = self.rows()[1]
        self.assertEqual(row["scenario_id"], "M18")
        self.assertEqual(row["result"], "INCONCLUSIVE")
        self.assertEqual(row["baseline_execution_status"], "MISSING")

    def test_scenario_removed_from_current(self):
        self.pair()
        self.campaign(self.current, [])
        row = self.rows()[0]
        self.assertEqual(row["result"], "REGRESSION")
        self.assertEqual(row["current_execution_status"], "MISSING")

    def test_invalid_and_same_ids(self):
        for invalid in ("../REG_x", "..\\REG_x", "C:\\temp", "/tmp", "REG_x:stream", ".", ""):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "invalid Campaign ID"):
                compare.compare_campaigns(self.root, invalid, self.current)
        for current in (self.baseline, self.baseline.lower()):
            with self.assertRaisesRegex(ValueError, "different Campaign IDs"):
                compare.compare_campaigns(self.root, self.baseline, current)

    def test_campaign_and_summary_missing(self):
        with self.assertRaisesRegex(ValueError, "Campaign not found"):
            compare.compare_campaigns(self.root, "REG_absent", self.current)
        (self.root / "regression" / self.baseline / "summary.csv").unlink()
        with self.assertRaisesRegex(ValueError, "execution summary not found"):
            self.rows()

    def test_unhealthy_baseline_is_inconclusive(self):
        self.pair(before=(0,), after=(1,))
        for status in ("TIMEOUT", "CONTROLLER_ERROR"):
            with self.subTest(status=status):
                self.campaign(self.baseline, [("M01", status, 2)])
                self.assertEqual(self.rows()[0]["result"], "INCONCLUSIVE")

    def test_definite_regression_beats_unhealthy_baseline(self):
        self.log(self.baseline)
        self.campaign(self.baseline, [("M01", "TIMEOUT", 2)])
        self.assertEqual(self.rows()[0]["result"], "REGRESSION")

    def test_execution_regression_beats_goal_improvement(self):
        self.pair(before=(0,), after=(1,))
        self.campaign(self.current, [("M01", "TIMEOUT", 2)])
        self.assertEqual(self.rows()[0]["result"], "REGRESSION")

    def test_empty_missing_column_and_bad_log_are_unavailable(self):
        self.log(self.baseline)
        path = self.log(self.current)
        for content in ("", "goal_reached\n", "frame,speed\n0,2\n", "goal_reached\n\"unterminated",
                        "frame,goal_reached\n0,\n", "frame,goal_reached\n0,1\n1\n",
                        "goal_reached,goal_reached\n0,1\n", "goal_reached\nyes\n"):
            with self.subTest(content=content):
                path.write_text(content, encoding="utf-8")
                row = self.rows()[0]
                self.assertEqual(row["result"], "INCONCLUSIVE")
                self.assertEqual(row["current_log_exists"], "true")
                self.assertEqual(row["current_goal_reached"], "UNAVAILABLE")

    def test_invalid_utf8_is_unavailable(self):
        self.pair()
        self.log(self.current).write_bytes(b"goal_reached\n\xff\n")
        self.assertEqual(self.rows()[0]["result"], "INCONCLUSIVE")

    def test_log_identity_mismatch(self):
        self.pair()
        path = self.log(self.current)
        path.write_text("scenario_id,run_id,goal_reached\nM02,REG_other,1\n")
        self.assertEqual(self.rows()[0]["result"], "INCONCLUSIVE")
        self.assertIn("mismatch", self.rows()[0]["reason"])

    def test_goal_true_in_any_frame_and_boolean_strings(self):
        self.pair(before=("false", "TRUE", "0"), after=("1", "0"))
        self.assertEqual(self.rows()[0]["result"], "SAME")

    def test_both_complete_goal_false_is_same(self):
        self.pair(before=(0,), after=(0,))
        self.assertEqual(self.rows()[0]["result"], "SAME")

    def test_status_code_inconsistency_and_unknown(self):
        self.pair()
        for code, expected in ((9, "REGRESSION"), (None, "INCONCLUSIVE")):
            with self.subTest(code=code):
                self.campaign(self.current, [("M01", "COMPLETE", code)])
                self.assertEqual(self.rows()[0]["result"], expected)
        self.campaign(self.current, [("M01", "FUTURE_STATUS", 0)])
        self.assertEqual(self.rows()[0]["result"], "INCONCLUSIVE")

    def test_launch_error_with_blank_exit_code(self):
        self.pair()
        self.campaign(self.current, [("M01", "LAUNCH_ERROR", None)])
        self.assertEqual(self.rows()[0]["result"], "REGRESSION")

    def test_duplicate_scenario_and_unsafe_scenario_rejected(self):
        for rows in ([('M01', 'COMPLETE', 0)] * 2, [('../M01', 'COMPLETE', 0)]):
            self.campaign(self.current, rows)
            with self.assertRaises(ValueError):
                self.rows()

    def test_invalid_summary_rejected_without_report(self):
        summary = self.root / "regression" / self.current / "summary.csv"
        for content in ("x\n1\n", "scenario_id,status,exit_code\nM01,COMPLETE\n",
                        "scenario_id,status,exit_code\nM01,COMPLETE,bad\n"):
            summary.write_text(content)
            with self.assertRaises(ValueError):
                self.rows()
        self.assertFalse(summary.with_name("regression_summary.csv").exists())

    def test_all_regression_reasons_are_kept(self):
        self.pair(after=(0,))
        self.campaign(self.current, [("M01", "CONTROLLER_ERROR", 2)])
        reason = self.rows()[0]["reason"]
        self.assertIn("execution COMPLETE -> CONTROLLER_ERROR", reason)
        self.assertIn("goal_reached true -> false", reason)

    def test_official_assessment_columns_are_ignored(self):
        self.log(self.baseline, extra={"collision": "0", "nearest_object_ttc_s": "99"})
        self.log(self.current, extra={"collision": "1", "nearest_object_ttc_s": "0"})
        self.assertEqual(self.rows()[0]["result"], "SAME")

    def test_cli_writes_only_report_and_preserves_all_sources(self):
        self.pair(after=(0,))
        snapshots = {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        completed = subprocess.run(
            [sys.executable, str(Path(compare.__file__).resolve()), "--baseline", self.baseline,
             "--current", self.current, "--output-root", str(self.root)],
            cwd=self.root, capture_output=True, text=True, encoding="utf-8", check=False,
        )
        self.assertEqual(completed.returncode, 1, completed.stderr)
        self.assertIn("*** REGRESSION DETECTED: 1 scenario(s) ***", completed.stdout)
        report = self.root / "regression" / self.current / "regression_summary.csv"
        self.assertEqual({p for p in self.root.rglob("*") if p.is_file()}, {*snapshots, report})
        for path, content in snapshots.items():
            self.assertEqual(path.read_bytes(), content, str(path))
        with report.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]["result"], "REGRESSION")
        self.assertEqual(rows[0]["baseline_campaign_id"], self.baseline)

    def test_cli_exit_codes(self):
        args = ["--baseline", self.baseline, "--current", self.current, "--output-root", str(self.root)]
        for before, after, expected in (((1,), (1,), 0), ((0,), (1,), 0), ((1,), (0,), 1), ((), (), 3)):
            self.pair(before, after)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(compare.main(args), expected)
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(compare.main([*args, "--current", self.baseline]), 2)

    def test_report_hardlink_cannot_modify_baseline(self):
        self.pair()
        original = self.root / "regression" / self.baseline / "summary.csv"
        path, rows = compare.compare_campaigns(self.root, self.baseline, self.current)
        content = original.read_bytes()
        try:
            os.link(original, path)
        except OSError as error:
            self.skipTest(f"hardlinks unavailable: {error}")
        compare.write_results(path, rows)
        self.assertEqual(original.read_bytes(), content)
        self.assertFalse(path.samefile(original))

    def test_campaign_symlink_escape_rejected(self):
        alias = self.root / "regression" / "REG_alias"
        try:
            alias.symlink_to(self.root, target_is_directory=True)
        except OSError as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaisesRegex(ValueError, "redirect"):
            compare.compare_campaigns(self.root, self.baseline, "REG_alias")

    def test_report_symlink_rejected(self):
        self.pair()
        path, rows = compare.compare_campaigns(self.root, self.baseline, self.current)
        source = self.root / "regression" / self.baseline / "summary.csv"
        try:
            path.symlink_to(source)
        except OSError as error:
            self.skipTest(f"symlinks unavailable: {error}")
        with self.assertRaisesRegex(ValueError, "symbolic link"):
            compare.write_results(path, rows)


if __name__ == "__main__":
    unittest.main()
