"""Evidence-boundary and process-lifecycle tests, with no VTD dependency."""

import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
import time
import unittest
import zipfile

from analyze import scan_run
from capture import prepare_capture, build_command
from contracts import CaptureRequest, PackRequest, Incident
from pack import pack_run, write_decision_excerpt, write_frame_excerpt, select_incidents
from reproduction import missing_build_sources
from run_field import run_capture


def sample(frame, **planning):
    return {"schema": 1, "kind": "frame", "frame": frame, "process_wall_s": frame * .1,
            "wall_s": frame * .1, "sim_s": frame * .1, "dt": .1,
            "ego": {"x": 0., "y": 0., "speed": 0.},
            "planning": {"valid": 1, "target_speed": 3., **planning},
            "control": {"safe": {"accel_mps2": 1.}}, "tracking": {}}


def write_samples(directory, rows):
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / "diagnostics.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")
            if row.get("kind") == "frame":
                stream.write(json.dumps({"schema": 1, "kind": "send_result", "frame": row["frame"],
                                         "process_wall_s": row["process_wall_s"], "socket_send_completed": 1}) + "\n")


class FieldDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_fast_default_avoids_legacy_synchronous_decision_log(self):
        request = CaptureRequest(self.root, self.root / "attempt", Path(sys.executable), [])
        command = build_command(request)
        self.assertIn("--diagnostic-dir", command)
        self.assertIn("--log", command)
        self.assertNotIn("--decision-log", command)
        request = CaptureRequest(self.root, self.root / "attempt", Path(sys.executable), [], True)
        self.assertIn("--decision-log", build_command(request))

    def test_async_loss_and_send_period_are_visible_in_brief(self):
        write_samples(self.root, [sample(0)])
        with (self.root / "diagnostics.jsonl").open("a") as stream:
            for frame, interval in ((1, 50.0), (2, 70.0)):
                stream.write(json.dumps(sample(frame)) + "\n")
                stream.write(json.dumps({"schema": 1, "kind": "send_result", "frame": frame,
                    "socket_send_completed": 1, "send_interval_ms": interval,
                    "diagnostic_capture_ms": .02, "receive_to_send_ms": 8.0}) + "\n")
            stream.write(json.dumps({"schema": 1, "kind": "logger_status", "mode": "async_bounded",
                                    "dropped_records_total": 5, "final": 1}) + "\n")
        (self.root / "repair_health.json").write_text(json.dumps({"limited_events_total": 3, "final": 0}))
        result = scan_run(self.root)
        self.assertEqual(result.send_interval_count, 2)
        self.assertEqual(result.send_interval_total_ms, 120.)
        self.assertEqual(result.send_intervals_over_55ms, 1)
        self.assertTrue(any("dropped_records_total=5" in issue for issue in result.issues))
        self.assertTrue(any("REPAIR_NOT_DRAINED" in issue for issue in result.issues))
        archive = pack_run(PackRequest(self.root))
        with zipfile.ZipFile(archive) as zipped:
            self.assertIn("repair_health.json", zipped.namelist())
            self.assertIn("16.67Hz", zipped.read("GPT_BRIEF.md").decode())

    def test_missing_async_tail_is_not_assumed_complete(self):
        write_samples(self.root, [{"schema": 1, "kind": "stage", "stage": "process_start",
                                   "detail": "mode=async_bounded"}])
        result = scan_run(self.root)
        self.assertTrue(any("LOGGER_NOT_DRAINED" in issue for issue in result.issues))

    def test_missing_frame_breaks_motion_even_with_short_wall_gap(self):
        rows = [sample(i) for i in range(25) if i != 10]
        write_samples(self.root, rows)
        result = scan_run(self.root)
        self.assertNotIn("COMMAND_WITHOUT_MOTION", result.counts)
        self.assertTrue(any("LOGGED_FRAME_GAP" in issue for issue in result.issues))

    def test_acceleration_without_motion_and_raw_evidence(self):
        write_samples(self.root, [sample(i) for i in range(35)])
        result = scan_run(self.root)
        self.assertIn("COMMAND_WITHOUT_MOTION", result.counts)
        self.assertEqual(result.send_completed, 35)
        incident = next(item for item in result.incidents if item.code == "COMMAND_WITHOUT_MOTION")
        self.assertEqual(incident.first_frame, 20)
        self.assertEqual(incident.evidence["control"]["safe"]["accel_mps2"], 1.)

    def test_signal_wait_is_observation_not_bug_or_score(self):
        rows = [sample(i, target_speed=0., stop_reason=1, stop_source_id=12) for i in range(35)]
        write_samples(self.root, rows)
        result = scan_run(self.root)
        self.assertIn("ZERO_TARGET_WAIT", result.counts)
        self.assertNotIn("COMMAND_WITHOUT_MOTION", result.counts)
        self.assertNotIn("VIOLATED", json.dumps(result.counts))

    def test_frozen_sim_time_does_not_accuse_acceleration_controller(self):
        rows = [sample(i) for i in range(35)]
        for row in rows: row["sim_s"] = 4.0
        write_samples(self.root, rows)
        result = scan_run(self.root)
        self.assertIn("SIM_TIME_FROZEN", result.counts)
        self.assertNotIn("COMMAND_WITHOUT_MOTION", result.counts)

    def test_gap_breaks_standstill_window(self):
        rows = [sample(i) for i in range(10)] + [sample(i) for i in range(100, 110)]
        write_samples(self.root, rows)
        self.assertNotIn("COMMAND_WITHOUT_MOTION", scan_run(self.root).counts)

    def test_partial_tail_keeps_prior_evidence(self):
        write_samples(self.root, [sample(0, emergency=1)])
        with (self.root / "diagnostics.jsonl").open("a") as stream: stream.write('{"kind":')
        result = scan_run(self.root)
        self.assertEqual(result.frames, 1)
        self.assertIn("EMERGENCY_BRAKE", result.counts)
        self.assertTrue(any("TRUNCATED" in issue for issue in result.issues))

    def test_nonfinite_json_is_reported_without_losing_bundle(self):
        write_samples(self.root, [sample(0)])
        with (self.root / "diagnostics.jsonl").open("a") as stream:
            stream.write('{"schema":1,"kind":"frame","process_wall_s":NaN}\n')
        self.assertTrue(pack_run(PackRequest(self.root)).is_file())

    def test_receive_only_is_not_presented_as_failed_actuation(self):
        rows = [sample(i) for i in range(35)]
        for row in rows:
            row["receive_only"] = 1
        write_samples(self.root, rows)
        result = scan_run(self.root)
        self.assertIn("RECEIVE_ONLY", result.counts)
        self.assertNotIn("COMMAND_WITHOUT_MOTION", result.counts)

    def test_pack_has_actual_sources_settings_raw_and_excerpts(self):
        (self.root / "src").mkdir()
        (self.root / "src/control.cpp").write_text("// version before drive\n")
        args = CaptureRequest(self.root, self.root / "attempt", Path(sys.executable), [])
        prepare_capture(args)
        (self.root / "src/control.cpp").write_text("// modified after drive\n")
        write_samples(args.directory, [sample(i) for i in range(35)])
        (args.directory / "repair_decision.log").write_text("0.000 frame=0 [lane_topology] side_scanned | side=left\n")
        archive = pack_run(PackRequest(args.directory))
        with zipfile.ZipFile(archive) as stream:
            self.assertIn("diagnostics.jsonl", stream.namelist())
            self.assertIn("focus_frames.jsonl", stream.namelist())
            self.assertIn("GPT_BRIEF.md", stream.namelist())
            self.assertEqual(stream.read("snapshot/src/control.cpp"), b"// version before drive\n")
            self.assertIn("bundle_manifest.json", stream.namelist())
            self.assertIn("repair_decision.log", stream.namelist())
            self.assertIn(b"repair_decision.log:1", stream.read("focus_decisions.log"))

    def test_excerpt_uses_separate_repair_file_without_modifying_legacy(self):
        (self.root / "decision.log").write_text("0.000 frame=1 [legacy] original\n")
        (self.root / "repair_decision.log").write_text("0.000 frame=1 [repair] new_detail\n")
        metadata = write_decision_excerpt(self.root, {1})
        self.assertEqual(metadata["full_file"], "repair_decision.log")
        self.assertEqual((self.root / "decision.log").read_text(), "0.000 frame=1 [legacy] original\n")
        self.assertIn("new_detail", (self.root / "focus_decisions.log").read_text())

    def test_middle_failure_and_neighbours_survive_long_run_excerpt(self):
        rows = []
        for index in range(3000):
            row = sample(index, valid=0 if index == 1000 else 1)
            row.update(process_wall_s=index * .05, sim_s=index * .05)
            row["ego"]["x"] = index * .1
            rows.append(row)
        write_samples(self.root, rows)
        scan = scan_run(self.root)
        frames, metadata = write_frame_excerpt(PackRequest(self.root), scan, select_incidents(scan, 8))
        excerpt = [json.loads(line) for line in (self.root / "focus_frames.jsonl").read_text().splitlines()]
        self.assertTrue({999, 1000, 1001} <= frames)
        self.assertEqual(frames, {row["frame"] for row in excerpt})
        for frame in (999, 1000, 1001):
            self.assertEqual({row["kind"] for row in excerpt if row["frame"] == frame}, {"frame", "send_result"})
        self.assertTrue(metadata["truncated"])
        self.assertLessEqual(len(excerpt), 320)

    def test_all_incident_boundaries_override_tiny_excerpt_budget(self):
        rows = [sample(index) for index in range(300)]
        for row in rows:
            row["ego"]["x"] = row["frame"]
        write_samples(self.root, rows)
        selected = [Incident(f"TEST_{index}", index * 30, index * 30 + 10,
                             index * 3., index * 3. + 1.) for index in range(1, 9)]
        frames, metadata = write_frame_excerpt(PackRequest(self.root, max_excerpt_records=10), scan_run(self.root), selected)
        for incident in selected:
            self.assertTrue({incident.first_frame - 1, incident.first_frame, incident.first_frame + 1,
                             incident.last_frame - 1, incident.last_frame, incident.last_frame + 1} <= frames)
        self.assertTrue(metadata["record_budget_exceeded"])

    def test_anchor_keeps_early_input_and_failure_stage_in_same_frame(self):
        row = sample(100)
        write_samples(self.root, [row])
        with (self.root / "diagnostics.jsonl").open("a") as stream:
            stream.write(json.dumps({"schema": 1, "kind": "input", "frame": 100, "process_wall_s": 0.}) + "\n")
            stream.write(json.dumps({"schema": 1, "kind": "stage", "frame": 100,
                                     "stage": "send_failed", "process_wall_s": 10., "detail": "TCP failure"}) + "\n")
        scan = scan_run(self.root)
        write_frame_excerpt(PackRequest(self.root), scan, select_incidents(scan, 8))
        excerpt = [json.loads(line) for line in (self.root / "focus_frames.jsonl").read_text().splitlines()]
        self.assertEqual({row["kind"] for row in excerpt}, {"frame", "send_result", "input", "stage"})

    def test_protected_decision_frame_keeps_all_events_over_budget(self):
        lines = ([f"0 frame=1 [normal] before | event={i}\n" for i in range(300)]
                 + [f"1 frame=1000 [failure] reason | event={i}\n" for i in range(250)]
                 + [f"2 frame=2000 [normal] after | event={i}\n" for i in range(300)])
        (self.root / "repair_decision.log").write_text("".join(lines))
        metadata = write_decision_excerpt(self.root, {1, 1000, 2000}, {1000})
        excerpt = (self.root / "focus_decisions.log").read_text()
        self.assertEqual(excerpt.count("frame=1000 "), 250)
        self.assertNotIn("frame=1 ", excerpt)
        self.assertTrue(metadata["line_budget_exceeded"])

    def test_build_tools_tests_and_runtime_dependencies_are_frozen(self):
        sources = {
            "src/main.cpp": "// main\n",
            "tests/controller_logic_tests.cpp": "// controller tests before run\n",
            "tests/fixture.csv": "x,y\n1,2\n",
            "tools/verification/replay_corridor_scene.cpp": "// replay\n",
            "tools/verification/prepare_runtime_mission.py": "from whole_mission_corridors import Network\n",
            "tools/verification/whole_mission_corridors.py": "from corridor_surface_bounds import apply_surface_bounds\n",
            "tools/verification/corridor_surface_bounds.py": "# geometry\n",
            "tools/map_precompute/precompute_lane_network.py": "from xodr_network import LaneNetwork\n",
            "tools/map_precompute/xodr_network.py": "# map\n",
            "scripts/build.sh": "cmake -S . -B build\n",
            "CMakeLists.txt": "add_executable(app src/main.cpp tests/controller_logic_tests.cpp tools/verification/replay_corridor_scene.cpp)\n",
        }
        for name, content in sources.items():
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        request = CaptureRequest(self.root, self.root / "attempt", Path(sys.executable), [])
        prepare_capture(request)
        (self.root / "tools/verification/prepare_runtime_mission.py").write_text("# changed after launch\n")
        archive = pack_run(PackRequest(request.directory))
        self.assertEqual(missing_build_sources(request.directory / "snapshot"), [])
        with zipfile.ZipFile(archive) as stream:
            for name, content in sources.items():
                self.assertEqual(stream.read("snapshot/" + name).decode(), content)
            self.assertIn("environment.json", stream.namelist())
            guide = stream.read("REPRODUCE.md").decode()
            self.assertIn("CMake에 명시된 로컬 소스 누락: 없음", guide)
            self.assertIn("주요 미션/지도 Python 소스 누락: 없음", guide)

    def test_old_incomplete_snapshot_is_not_reported_as_build_complete(self):
        snapshot = self.root / "snapshot"
        snapshot.mkdir()
        (snapshot / "CMakeLists.txt").write_text("add_executable(test tests/missing.cpp)\n")
        archive = pack_run(PackRequest(self.root))
        with zipfile.ZipFile(archive) as stream:
            guide = stream.read("REPRODUCE.md").decode()
            self.assertIn("tests/missing.cpp", guide)
            self.assertIn("prepare_runtime_mission.py", guide)

    def test_invalid_map_yaml_does_not_prevent_error_capture(self):
        (self.root / "broken.yaml").write_text("map: [\n")
        args = CaptureRequest(self.root, self.root / "attempt", Path(sys.executable), ["--map-config", "broken.yaml"])
        manifest = prepare_capture(args)
        self.assertIn("dependency_error", manifest["inputs"]["--map-config"])

    def test_existing_attempt_is_never_overwritten(self):
        directory = self.root / "attempt"
        directory.mkdir()
        (directory / "keep").write_text("original")
        with self.assertRaises(FileExistsError):
            prepare_capture(CaptureRequest(self.root, directory, Path(sys.executable), []))
        self.assertEqual((directory / "keep").read_text(), "original")

    def test_missing_binary_still_generates_launch_evidence(self):
        args = CaptureRequest(self.root, self.root / "attempt", self.root / "missing_binary", [])
        self.assertEqual(run_capture(args), 1)
        self.assertTrue((args.directory / "gpt_bundle.zip").is_file())
        manifest = json.loads((args.directory / "capture_manifest.json").read_text())
        self.assertEqual(manifest["status"], "LAUNCH_FAILED")

    def test_running_attempt_is_not_packed(self):
        (self.root / ".capture_running").write_text("123")
        with self.assertRaisesRegex(ValueError, "still running"):
            pack_run(PackRequest(self.root))

    def test_controller_exit_preserved_and_console_captured(self):
        script = self.root / "fake.py"
        script.write_text("import sys\nprint('TCP failed example')\nsys.exit(7)\n")
        args = CaptureRequest(self.root, self.root / "attempt", Path(sys.executable), [str(script)])
        self.assertEqual(run_capture(args), 7)
        self.assertIn("TCP failed example", (args.directory / "console.log").read_text())
        self.assertTrue((args.directory / "GPT_BRIEF.md").is_file())

    @unittest.skipIf(os.name == "nt", "POSIX process-group interrupt test")
    def test_operator_interrupt_stops_controller_and_packs(self):
        directory = self.root / "interrupt_attempt"
        runner = Path(__file__).with_name("run_field.py")
        process = subprocess.Popen([
            sys.executable, str(runner), "--output", str(directory),
            "--binary", sys.executable, "--", "-c", "import time; time.sleep(15)",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        child_pid = None
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                path = directory / "capture_manifest.json"
                manifest = json.loads(path.read_text()) if path.is_file() else {}
                if manifest.get("status") == "RUNNING":
                    child_pid = manifest["controller_pid"]
                    break
                time.sleep(.05)
            self.assertIsNotNone(child_pid, "controller did not start")
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 130, output)
            self.assertTrue((directory / "gpt_bundle.zip").is_file())
            self.assertFalse((directory / ".capture_running").exists())
            self.assertEqual(json.loads((directory / "capture_manifest.json").read_text())["status"], "INTERRUPTED")
            with self.assertRaises(ProcessLookupError):
                os.kill(child_pid, 0)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
            if child_pid is not None:
                try:
                    os.killpg(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


if __name__ == "__main__":
    unittest.main()
