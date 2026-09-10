#!/usr/bin/env python3

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import gzip
import io
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "verification"))

import run_scenario as runner  # noqa: E402
from run_scenario import (  # noqa: E402
    ControllerExitBeforePlay,
    EvidenceBundle,
    EvidenceExistsError,
    RvizHistoryRecorder,
    TrafficHealthMonitor,
    run_controller,
    scenario_requires_traffic,
)
from analyze_planner_replay import analyze  # noqa: E402


class TrafficHealthMonitorTests(unittest.TestCase):
    def test_old_warnings_are_ignored_when_monitor_is_armed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.write_text(
                "no traffic info available for 5000 wait frames\n",
                encoding="utf-8",
            )
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            self.assertIsNone(monitor.poll(now=10.0))

    def test_single_transient_warning_recovers_without_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.touch()
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            with log.open("a", encoding="utf-8") as stream:
                stream.write("no traffic info available for 100 wait frames\n")
            self.assertIsNone(monitor.poll(now=10.0))
            self.assertIsNone(monitor.poll(now=10.7))
            self.assertIsNone(monitor.poll(now=11.2))

    def test_repeated_missing_traffic_fails_after_grace_period(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.touch()
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            for now in (10.0, 10.3, 10.6, 10.9):
                with log.open("a", encoding="utf-8") as stream:
                    stream.write(
                        "no traffic info available for 5000 wait frames\n"
                    )
                self.assertIsNone(monitor.poll(now=now))
            with log.open("a", encoding="utf-8") as stream:
                stream.write("waiting for connection <Traffic>....\n")
            failure = monitor.poll(now=11.1)
            self.assertIsNotNone(failure)
            self.assertIn("Traffic health check failed", failure)

    def test_connected_message_clears_pending_loss(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.touch()
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            with log.open("a", encoding="utf-8") as stream:
                stream.write("waiting for connection <Traffic>....\n")
            self.assertIsNone(monitor.poll(now=10.0))
            with log.open("a", encoding="utf-8") as stream:
                stream.write('NOTICE: "<Traffic> connected"\n')
            self.assertIsNone(monitor.poll(now=11.5))

    def test_internal_player_requires_traffic_but_empty_scene_does_not(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            internal = Path(directory) / "internal.xml"
            internal.write_text(
                "<Scenario><TrafficControl><Player><Description "
                'Name="NPC" Control="internal"/></Player>'
                "</TrafficControl></Scenario>",
                encoding="utf-8",
            )
            empty = Path(directory) / "empty.xml"
            empty.write_text(
                "<Scenario><TrafficControl/></Scenario>", encoding="utf-8"
            )
            self.assertTrue(scenario_requires_traffic(internal))
            self.assertFalse(scenario_requires_traffic(empty))


class EvidenceBundleTests(unittest.TestCase):
    def test_active_assessment_profile_is_atomic_and_owner_scoped(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            route=root/'route.csv'; route.write_text('x,y\n0,0\n',encoding='utf-8')
            scenario=root/'scenario.xml'; scenario.write_text('<Scenario/>',encoding='utf-8')
            checkpoints=root/'checkpoints.csv'; checkpoints.write_text('seq,x,y\n1,0,0\n',encoding='utf-8')
            profile=root/'active.json'
            token=runner.publish_active_assessment_profile(
                profile,'M16','run',route,scenario,checkpoints,1,
            )
            payload=json.loads(profile.read_text(encoding='utf-8'))
            self.assertEqual(payload['scenario_id'],'M16')
            self.assertEqual(payload['mission_start_leg'],1)
            self.assertEqual(payload['ownership_token'],token)
            runner.clear_active_assessment_profile(profile,'another-run')
            self.assertTrue(profile.exists())
            runner.clear_active_assessment_profile(profile,token)
            self.assertFalse(profile.exists())

    def test_main_dry_run_builds_bundle_and_rejects_reuse(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            package.mkdir()
            scenario = package / "scenario.xml"
            scenario.write_text("<Scenario/>", encoding="utf-8")
            scenario_manifest = package / "scenario.yaml"
            scenario_manifest.write_text("scenario:\n  id: TEST\n", encoding="utf-8")
            route = package / "route.csv"
            route.write_text("x,y\n0,0\n", encoding="utf-8")
            config = root / "network.yaml"
            config.write_text("vtd: {}\n", encoding="utf-8")
            manifest = {"scenario": {"id": "TEST"}, "runtime": {}}
            report = SimpleNamespace(
                exit_code=0,
                manifest=manifest,
                scenario_file=scenario,
                route_file=route,
                package=package,
                print=lambda: None,
            )
            args = argparse.Namespace(
                package=package,
                skip_source_sync=True,
                run_id="same-id",
                config=config,
                evidence=True,
                rviz_data=False,
                decision_log=None,
                auto_vtd=False,
                dry_run=True,
                param=[],
                randomize=False,
                seed=None,
            )
            output = io.StringIO()
            errors = io.StringIO()
            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                mock.patch.object(runner, "parse_args", return_value=args),
                mock.patch.object(runner, "validate_scenario", return_value=report),
                redirect_stdout(output),
                redirect_stderr(errors),
            ):
                self.assertEqual(runner.main(), 0)
                run = root / "output" / "verification" / "TEST" / "same-id"
                manifest_path = run / "evidence_manifest.json"
                first_manifest = manifest_path.read_bytes()
                payload = json.loads(first_manifest)
                self.assertEqual(payload["status"], "DRY_RUN")
                self.assertIn("--decision-log", payload["command"])
                self.assertIn("--visualization-dir", payload["command"])
                self.assertEqual(runner.main(), 2)
                self.assertEqual(manifest_path.read_bytes(), first_manifest)
            self.assertIn("choose a unique run ID", errors.getvalue())

    def test_manifest_hashes_artifacts_and_refuses_run_id_reuse(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "scenario.xml"
            source.write_text("<Scenario/>", encoding="utf-8")
            run = root / "run"
            bundle = EvidenceBundle(run, "M15", "proof", {"scenario": source})
            (run / "decision.log").write_text("decision\n", encoding="utf-8")
            bundle.set_command(["hl_vtd", "--run-id", "proof"])
            bundle.finalize(7)

            manifest = json.loads(
                (run / "evidence_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["exit_code"], 7)
            self.assertEqual(manifest["status"], "FAILED")
            self.assertFalse(manifest["video_evidence"]["included"])
            self.assertIn(
                "--record-video-dir",
                manifest["video_evidence"]["required_dashboard_integration"],
            )
            self.assertIn(
                "separate-empty-video-directory",
                manifest["video_evidence"]["required_dashboard_integration"],
            )
            self.assertIn(
                "output/video_evidence",
                manifest["video_evidence"]["directory_constraint"],
            )
            self.assertEqual(manifest["command"][-1], "proof")
            artifacts = {record["path"]: record for record in manifest["artifacts"]}
            self.assertEqual(artifacts["decision.log"]["size"], 9)
            self.assertEqual(len(artifacts["decision.log"]["sha256"]), 64)
            with self.assertRaises(EvidenceExistsError):
                EvidenceBundle(run, "M15", "proof", {"scenario": source})
            self.assertEqual((run / "decision.log").read_text(), "decision\n")

    def test_pre_play_clean_exit_is_an_automation_failure(self) -> None:
        error = ControllerExitBeforePlay(0)
        self.assertEqual(error.controller_exit_code, 0)
        self.assertEqual(error.runner_exit_code, 2)
        self.assertEqual(ControllerExitBeforePlay(7).runner_exit_code, 7)

        with tempfile.TemporaryDirectory() as directory:
            ready_file = Path(directory) / "controller.ready"
            controller = mock.Mock()
            controller.wait.return_value = 0
            controller.poll.return_value = 0
            scp = mock.MagicMock()
            context = mock.MagicMock()
            context.__enter__.return_value = scp
            args = SimpleNamespace(
                scp_port=48179,
                vtd_timeout=1.0,
                inline_scenario=False,
            )
            with (
                mock.patch.object(runner, "VtdScpClient", return_value=context),
                mock.patch.object(
                    runner,
                    "start_controller",
                    return_value=(controller, (None, None)),
                ),
                redirect_stdout(io.StringIO()),
                redirect_stderr(io.StringIO()),
                self.assertRaises(ControllerExitBeforePlay) as raised,
            ):
                runner.run_automated(
                    args,
                    ["hl_vtd"],
                    Path(directory) / "scenario.xml",
                    "127.0.0.1",
                    ready_file,
                    False,
                    Path(directory),
                )
            self.assertEqual(raised.exception.runner_exit_code, 2)
            scp.start.assert_not_called()

    def test_main_records_pre_play_clean_exit_as_automation_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            package.mkdir()
            scenario = package / "scenario.xml"
            scenario.write_text("<Scenario/>", encoding="utf-8")
            (package / "scenario.yaml").write_text(
                "scenario:\n  id: TEST\n", encoding="utf-8"
            )
            route = package / "route.csv"
            route.write_text("x,y\n0,0\n", encoding="utf-8")
            config = root / "network.yaml"
            config.write_text("vtd: {}\n", encoding="utf-8")
            binary = root / "build" / "hl_vtd"
            binary.parent.mkdir()
            binary.write_text("", encoding="utf-8")
            report = SimpleNamespace(
                exit_code=0,
                manifest={"scenario": {"id": "TEST"}, "runtime": {}},
                scenario_file=scenario,
                route_file=route,
                package=package,
                print=lambda: None,
            )
            args = argparse.Namespace(
                package=package,
                skip_source_sync=True,
                run_id="pre-play-zero",
                config=config,
                evidence=True,
                rviz_data=False,
                decision_log=None,
                auto_vtd=True,
                dry_run=False,
                param=[],
                randomize=False,
                seed=None,
                scp_host="127.0.0.1",
                scp_port=48179,
                vtd_scenario_file=None,
                inline_scenario=False,
            )
            with (
                mock.patch.object(runner, "REPO_ROOT", root),
                mock.patch.object(runner, "parse_args", return_value=args),
                mock.patch.object(runner, "validate_scenario", return_value=report),
                mock.patch.object(
                    runner,
                    "run_automated",
                    side_effect=ControllerExitBeforePlay(0),
                ),
                redirect_stdout(io.StringIO()),
                redirect_stderr(io.StringIO()),
            ):
                self.assertEqual(runner.main(), 2)

            manifest = json.loads(
                (
                    root
                    / "output"
                    / "verification"
                    / "TEST"
                    / "pre-play-zero"
                    / "evidence_manifest.json"
                ).read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["status"], "AUTOMATION_FAILED")
            self.assertEqual(manifest["exit_code"], 2)
            self.assertEqual(manifest["controller_exit_code"], 0)

    def test_controller_console_capture_preserves_exit_code(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            command = [
                sys.executable,
                "-c",
                (
                    "import sys; print('controller-out'); "
                    "print('controller-err', file=sys.stderr); raise SystemExit(7)"
                ),
            ]
            self.assertEqual(run_controller(command, run), 7)
            self.assertEqual(
                (run / "console.stdout.log").read_text().strip(),
                "controller-out",
            )
            self.assertEqual(
                (run / "console.stderr.log").read_text().strip(),
                "controller-err",
            )

    def test_rviz_latest_is_preserved_as_compressed_frame_history(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            rviz = Path(directory) / "rviz"
            recorder = RvizHistoryRecorder(rviz, poll_interval=10)
            recorder.start()
            try:
                recorder.latest.write_text(
                    json.dumps({"frame": 1, "time_s": .1, "objects": []})
                )
                recorder._capture()
                recorder._capture()  # unchanged latest.json is not duplicated
                recorder.latest.write_text(
                    json.dumps({"frame": 3, "time_s": .3, "objects": []})
                )
                recorder._capture()
            finally:
                recorder.close()
            with gzip.open(recorder.history, "rt", encoding="utf-8") as stream:
                frames = [json.loads(line) for line in stream]
            self.assertEqual([frame["frame"] for frame in frames], [1, 3])
            self.assertEqual(recorder.status()["captured_frames"], 2)
            self.assertEqual(recorder.status()["estimated_missed_frames"], 1)
            self.assertEqual(analyze(Path(directory))["history_last_frame"], 3)


if __name__ == "__main__":
    unittest.main()
