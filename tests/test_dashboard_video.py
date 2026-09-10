"""Dual-view discovery, relay metadata, and HTTP compatibility tests."""
from __future__ import annotations

import io
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "dashboard"))

import server  # noqa: E402
from video_relay import VideoRelay  # noqa: E402
from video_recording import (  # noqa: E402
    VideoRecordingExistsError,
    VideoRecordingSession,
)
from video_source import (  # noqa: E402
    VideoSourceConfig,
    parse_window_tree,
    select_vtd_windows,
    video_capture_command,
)


WINDOW_TREE = """
  0x100001 "cam1": ("osgViewer" "osgViewer") 1920x1080+0+0 +0+0
  0x100002 "mainRS": ("Qt" "Qt") 1280x720+10+20 +10+20
  0x100003 "birdRS": ("osgViewer" "osgViewer") 1024x768+30+40 +30+40
  0x100004 "not-mainRS-camera": ("osgViewer" "osgViewer") 4096x2160+0+0 +0+0
"""


class WindowDiscoveryTests(unittest.TestCase):
    def test_exact_main_and_bird_are_distinct_and_keep_identity(self):
        selected = select_vtd_windows(parse_window_tree(WINDOW_TREE))
        self.assertEqual(selected["mainrs"]["window_id"], 0x100002)
        self.assertEqual(selected["mainrs"]["title"], "mainRS")
        self.assertEqual(selected["mainrs"]["matched_title"], "mainRS")
        self.assertEqual(selected["birdrs"]["window_id"], 0x100003)
        self.assertNotEqual(
            selected["mainrs"]["window_id"], selected["birdrs"]["window_id"]
        )
        self.assertEqual(selected["birdrs"]["x"], 30)
        self.assertEqual(selected["birdrs"]["height"], 768)

    def test_main_fallback_is_ordered_and_bird_has_no_fallback(self):
        windows = parse_window_tree(
            '0x20 "igVisualBroadcast": ("osg" "osg") 1600x900+0+0 +0+0\n'
            '0x21 "cam1": ("osg" "osg") 800x600+0+0 +0+0\n'
        )
        selected = select_vtd_windows(windows)
        self.assertEqual(selected["mainrs"]["title"], "cam1")
        self.assertEqual(selected["mainrs"]["matched_title"], "cam1")
        self.assertIsNone(selected["birdrs"])

    def test_duplicate_xid_is_never_assigned_to_both_views(self):
        windows = parse_window_tree(
            '0x30 "mainRS birdRS": ("osg" "osg") 800x600+0+0 +0+0\n'
            '0x31 "cam1": ("osg" "osg") 800x600+0+0 +0+0\n'
        )
        selected = select_vtd_windows(windows)
        self.assertEqual(selected["birdrs"]["window_id"], 0x30)
        self.assertEqual(selected["mainrs"]["window_id"], 0x31)

    @mock.patch("video_source.shutil.which", return_value="/usr/bin/ffmpeg")
    def test_capture_command_reports_actual_window_metadata(self, _which):
        window = select_vtd_windows(parse_window_tree(WINDOW_TREE))["birdrs"]
        command, source, error = video_capture_command(
            VideoSourceConfig("window", None, 10),
            lambda: window,
            view="birdrs",
        )
        self.assertIsNone(error)
        self.assertEqual(command[command.index("-window_id") + 1], str(0x100003))
        self.assertEqual(source["source"], "window")
        self.assertEqual(source["view"], "birdrs")
        self.assertEqual(source["title"], "birdRS")
        self.assertEqual(source["geometry"]["width"], 1024)

    @mock.patch("video_source.shutil.which", return_value="/usr/bin/ffmpeg")
    def test_rtsp_is_per_view_and_missing_bird_is_truthful(self, _which):
        command, source, error = video_capture_command(
            VideoSourceConfig("rtsp", "rtsp://example/bird", 5),
            lambda: None,
            view="birdrs",
        )
        self.assertIsNone(error)
        self.assertIn("rtsp://example/bird", command)
        self.assertEqual(source, {
            "source": "rtsp", "view": "birdrs", "url": "rtsp://example/bird"
        })
        secret_url = (
            "rtsp://camera-user:camera-secret@example/bird"
            "?access_token=query-secret#fragment-secret"
        )
        command, source, error = video_capture_command(
            VideoSourceConfig("rtsp", secret_url, 5),
            lambda: None,
            view="birdrs",
        )
        self.assertIsNone(error)
        self.assertIn(secret_url, command)
        self.assertEqual(
            source["url"],
            "rtsp://<redacted>@example/bird?<redacted>#<redacted>",
        )
        self.assertNotIn("camera-secret", json.dumps(source))
        self.assertNotIn("query-secret", json.dumps(source))
        self.assertNotIn("fragment-secret", json.dumps(source))
        command, source, error = video_capture_command(
            VideoSourceConfig("rtsp", None, 5), lambda: None, view="birdrs"
        )
        self.assertIsNone(command)
        self.assertEqual(source["view"], "birdrs")
        self.assertIn("not configured", error)


class RelayMetadataTests(unittest.TestCase):
    def test_source_metadata_is_exposed_in_status(self):
        command = [
            sys.executable,
            "-u",
            "-c",
            "import sys,time;sys.stdout.buffer.write(bytes.fromhex('ffd8ffd9'));"
            "sys.stdout.flush();time.sleep(10)",
        ]
        source = {
            "source": "window",
            "view": "mainrs",
            "title": "mainRS",
            "window_id": 42,
            "geometry": {"x": 1, "y": 2, "width": 3, "height": 4},
        }
        relay = VideoRelay(lambda: (command, source, None))
        relay.start()
        try:
            _, frame = relay.wait_frame(0, 3)
            self.assertEqual(frame, bytes.fromhex("ffd8ffd9"))
            status = relay.status()
            self.assertTrue(status["found"])
            self.assertEqual(status["title"], "mainRS")
            self.assertEqual(status["window_id"], 42)
            self.assertEqual(status["geometry"]["height"], 4)
        finally:
            relay.close()


class VideoRecordingTests(unittest.TestCase):
    @staticmethod
    def abundant_space(_path):
        return SimpleNamespace(free=10 * 1024 * 1024)

    def test_separate_raw_streams_and_indexes_retain_frame_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "evidence"
            recording = VideoRecordingSession(
                target,
                1024,
                disk_usage=self.abundant_space,
                disk_check_interval_s=0,
                manifest_interval_s=0,
            )
            main_source = {
                "source": "window",
                "view": "mainrs",
                "title": "mainRS",
                "window_id": 0x101,
                "geometry": {"x": 1, "y": 2, "width": 3, "height": 4},
            }
            bird_source = {
                "source": "window",
                "view": "birdrs",
                "title": "birdRS",
                "window_id": 0x202,
            }
            main_jpeg = b"\xff\xd8main\xff\xd9"
            main_jpeg_2 = b"\xff\xd8main-2\xff\xd9"
            bird_jpeg = b"\xff\xd8bird\xff\xd9"
            self.assertTrue(recording.record_frame(
                "mainrs", 11, main_jpeg, main_source,
                wall_time_ns=1_700_000_000_000_000_000,
                monotonic_ns=8_000_000_000,
            ))
            self.assertTrue(recording.record_frame(
                "mainrs", 12, main_jpeg_2, main_source,
                wall_time_ns=1_700_000_000_050_000_000,
                monotonic_ns=8_050_000_000,
            ))
            self.assertTrue(recording.record_frame(
                "birdrs", 21, bird_jpeg, bird_source,
                wall_time_ns=1_700_000_000_100_000_000,
                monotonic_ns=8_100_000_000,
            ))
            session_id = recording.session_id
            recording.close()

            self.assertEqual(
                (target / "mainrs.mjpeg").read_bytes(),
                main_jpeg + main_jpeg_2,
            )
            self.assertEqual((target / "birdrs.mjpeg").read_bytes(), bird_jpeg)
            main_indexes = [
                json.loads(line)
                for line in (target / "mainrs.frames.jsonl")
                .read_text()
                .splitlines()
            ]
            main_index = main_indexes[0]
            bird_index = json.loads(
                (target / "birdrs.frames.jsonl").read_text().strip()
            )
            self.assertEqual(main_index["session_id"], session_id)
            self.assertEqual(main_index["sequence"], 11)
            self.assertEqual(main_index["offset"], 0)
            self.assertEqual(main_index["length"], len(main_jpeg))
            self.assertEqual(main_index["wall_time_ns"], 1_700_000_000_000_000_000)
            self.assertEqual(main_index["monotonic_ns"], 8_000_000_000)
            self.assertEqual(main_index["source"]["window_id"], 0x101)
            self.assertEqual(main_indexes[1]["sequence"], 12)
            self.assertEqual(main_indexes[1]["offset"], len(main_jpeg))
            self.assertEqual(main_indexes[1]["length"], len(main_jpeg_2))
            self.assertEqual(bird_index["view"], "birdrs")
            self.assertEqual(bird_index["source"]["window_id"], 0x202)

            manifest = json.loads(
                (target / "recording_manifest.json").read_text()
            )
            self.assertEqual(manifest["status"], "CLOSED")
            self.assertFalse(manifest["active"])
            self.assertEqual(manifest["views"]["mainrs"]["frame_count"], 2)
            self.assertEqual(manifest["views"]["birdrs"]["frame_count"], 1)
            self.assertEqual(
                manifest["views"]["mainrs"]["source_segments"][0]
                ["source"]["title"],
                "mainRS",
            )
            self.assertEqual(
                manifest["views"]["mainrs"]["source_segments"][0]
                ["frame_count"],
                2,
            )
            self.assertTrue(
                manifest["views"]["mainrs"]["index_authoritative"]
            )
            self.assertEqual(
                manifest["views"]["mainrs"]["orphan_tail_bytes"], 0
            )

    def test_index_failure_marks_unindexed_raw_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "evidence"
            recording = VideoRecordingSession(
                target,
                1024,
                disk_usage=self.abundant_space,
                disk_check_interval_s=0,
            )
            recording._views["mainrs"]["index_stream"].close()
            jpeg = b"\xff\xd8orphan\xff\xd9"
            self.assertFalse(recording.record_frame(
                "mainrs",
                1,
                jpeg,
                {"source": "window", "window_id": 1},
            ))
            status = recording.status()
            self.assertEqual(status["status"], "ERROR")
            self.assertIn("index is authoritative", status["error"])
            self.assertEqual(
                status["views"]["mainrs"]["orphan_tail_bytes"], len(jpeg)
            )
            self.assertEqual(status["views"]["mainrs"]["frame_count"], 0)
            manifest = json.loads(
                (target / "recording_manifest.json").read_text()
            )
            self.assertEqual(manifest["status"], "ERROR")
            self.assertIn("index is authoritative", manifest["error"])
            self.assertEqual(
                manifest["views"]["mainrs"]["orphan_tail_bytes"], len(jpeg)
            )
            recording.close()

    def test_nonempty_directory_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "evidence"
            target.mkdir()
            marker = target / "keep.txt"
            marker.write_text("do not replace", encoding="utf-8")
            with self.assertRaises(VideoRecordingExistsError):
                VideoRecordingSession(
                    target, 1024, disk_usage=self.abundant_space
                )
            self.assertEqual(marker.read_text(), "do not replace")
            self.assertEqual(list(target.iterdir()), [marker])

    def test_verification_run_parent_is_never_preclaimed_for_video(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            verification = root / "output" / "verification"
            inside = verification / "M15" / "same-run" / "video"
            outside = root / "output" / "video_evidence" / "same-run"
            with mock.patch.object(server, "VERIFICATION_DIR", verification):
                with self.assertRaisesRegex(ValueError, "must be outside"):
                    server.resolve_video_recording_directory(inside)
                self.assertEqual(
                    server.resolve_video_recording_directory(outside),
                    outside.resolve(),
                )
            self.assertFalse(inside.exists())

    def test_low_disk_stops_recording_but_relay_keeps_streaming(self):
        free_values = iter((10 * 1024 * 1024, 16))

        def disk_usage(_path):
            return SimpleNamespace(free=next(free_values, 16))

        command = [
            sys.executable,
            "-u",
            "-c",
            "import sys,time;sys.stdout.buffer.write(bytes.fromhex('ffd8ffd9'));"
            "sys.stdout.flush();time.sleep(10)",
        ]
        with tempfile.TemporaryDirectory() as directory:
            recording = VideoRecordingSession(
                Path(directory) / "evidence",
                1024,
                disk_usage=disk_usage,
                disk_check_interval_s=0,
            )
            relay = VideoRelay(
                lambda: (command, {
                    "source": "window", "view": "mainrs", "title": "mainRS",
                    "window_id": 7,
                }, None),
                recording=recording,
                view="mainrs",
            )
            relay.start()
            try:
                _, frame = relay.wait_frame(0, 3)
                self.assertEqual(frame, bytes.fromhex("ffd8ffd9"))
                deadline = time.monotonic() + 2
                while recording.status()["active"] and time.monotonic() < deadline:
                    time.sleep(.01)
                status = recording.status()
                self.assertEqual(status["status"], "STOPPED_LOW_DISK")
                self.assertFalse(status["active"])
                self.assertTrue(relay.status()["found"])
                self.assertEqual(
                    relay.status()["recording"]["status"],
                    "STOPPED_LOW_DISK",
                )
            finally:
                relay.close()
                recording.close()

    def test_unexpected_recorder_error_isolated_from_live_stream(self):
        class BrokenRecording:
            @staticmethod
            def record_frame(*_args, **_kwargs):
                raise OSError("evidence disk failed")

            @staticmethod
            def status(_view):
                return {"status": "ERROR", "active": False, "error": "failed"}

        command = [
            sys.executable,
            "-u",
            "-c",
            "import sys,time;sys.stdout.buffer.write(bytes.fromhex('ffd8ffd9'));"
            "sys.stdout.flush();time.sleep(10)",
        ]
        relay = VideoRelay(
            lambda: (command, {"source": "window", "view": "mainrs"}, None),
            recording=BrokenRecording(),
            view="mainrs",
        )
        relay.start()
        try:
            _, frame = relay.wait_frame(0, 3)
            self.assertEqual(frame, bytes.fromhex("ffd8ffd9"))
            deadline = time.monotonic() + 2
            status = relay.status()
            while (
                "relay_error" not in status["recording"]
                and time.monotonic() < deadline
            ):
                time.sleep(.01)
                status = relay.status()
            self.assertTrue(status["found"])
            self.assertIn("evidence disk failed", status["recording"]["relay_error"])
        finally:
            relay.close()


class RelaySourceChangeTests(unittest.TestCase):
    def test_source_change_never_relabels_a_stale_frame_as_live(self):
        command = [
            sys.executable,
            "-u",
            "-c",
            "import sys;sys.stdout.buffer.write(bytes.fromhex('ffd8ffd9'));"
            "sys.stdout.flush();import time;time.sleep(.2)",
        ]
        attempts = []

        def command_factory():
            attempts.append(len(attempts) + 1)
            if len(attempts) == 1:
                return command, {
                    "source": "window", "view": "mainrs", "title": "old",
                }, None
            return None, {
                "source": "window", "view": "mainrs", "title": "new",
            }, "new source unavailable"

        relay = VideoRelay(command_factory)
        relay.start()
        try:
            _, frame = relay.wait_frame(0, 3)
            self.assertEqual(frame, bytes.fromhex("ffd8ffd9"))
            deadline = time.monotonic() + 4
            status = relay.status()
            while status.get("title") != "new" and time.monotonic() < deadline:
                time.sleep(.01)
                status = relay.status()
            self.assertEqual(status.get("title"), "new")
            self.assertFalse(status["found"])
            self.assertEqual(status["error"], "new source unavailable")
            self.assertIsNone(relay.frame)
        finally:
            relay.close()


class FakeRelay:
    def __init__(self, view, marker):
        self.view = view
        self.marker = marker

    def status(self):
        return {
            "found": True,
            "view": self.view,
            "source": "window",
            "title": self.view,
            "window_id": self.marker,
            "geometry": {"x": 0, "y": 0, "width": 10, "height": 10},
            "frame_count": 1,
            "error": None,
        }

    def wait_frame(self, sequence, timeout=2):
        if sequence < 1:
            return 1, b"\xff\xd8" + bytes([self.marker]) + b"\xff\xd9"
        return sequence, None


class FakeRecording:
    def status(self):
        return {
            "session_id": "recording-test",
            "status": "RECORDING",
            "active": True,
            "views": {
                "mainrs": {"frame_count": 3},
                "birdrs": {"frame_count": 4},
            },
        }


class VideoEndpointTests(unittest.TestCase):
    def setUp(self):
        self.old_relays = server.VIDEO_RELAYS
        self.old_relay = server.VIDEO_RELAY
        self.old_recording = server.VIDEO_RECORDING
        self.main = FakeRelay("mainrs", 1)
        self.bird = FakeRelay("birdrs", 2)
        server.VIDEO_RELAYS = {"mainrs": self.main, "birdrs": self.bird}
        server.VIDEO_RELAY = self.main
        server.VIDEO_RECORDING = FakeRecording()

    def tearDown(self):
        server.VIDEO_RELAYS = self.old_relays
        server.VIDEO_RELAY = self.old_relay
        server.VIDEO_RECORDING = self.old_recording

    def route(self, path):
        handler = object.__new__(server.Handler)
        handler.path = path
        result = {}
        handler._send_json = lambda payload, status=200: result.update(
            payload=payload, status=status
        )
        handler._stream_shared_video = lambda view="mainrs": result.update(
            stream=view
        )
        handler.do_GET()
        return result

    def stream(self, view):
        handler = object.__new__(server.Handler)
        handler.wfile = io.BytesIO()
        response = {"headers": {}}
        handler.send_response = lambda status: response.update(status=status)
        handler.send_header = lambda key, value: response["headers"].update(
            {key: value}
        )
        handler.end_headers = lambda: None
        handler._stream_shared_video(view)
        response["body"] = handler.wfile.getvalue()
        return response

    def test_view_status_and_legacy_main_alias(self):
        main = self.route("/api/video/mainrs/status")["payload"]
        bird = self.route("/api/video/birdrs/status")["payload"]
        legacy = self.route("/api/mainrs_status")["payload"]
        self.assertEqual(main["window_id"], 1)
        self.assertEqual(bird["window_id"], 2)
        self.assertEqual(main, legacy)

    def test_view_streams_do_not_mix_and_main_alias_remains(self):
        self.assertEqual(self.route("/api/video/mainrs/stream")["stream"], "mainrs")
        self.assertEqual(self.route("/api/video/birdrs/stream")["stream"], "birdrs")
        self.assertEqual(self.route("/api/mainrs_stream")["stream"], "mainrs")
        main = self.stream("mainrs")
        bird = self.stream("birdrs")
        self.assertIn("multipart/x-mixed-replace", main["headers"]["Content-Type"])
        self.assertIn(b"\xff\xd8\x01\xff\xd9", main["body"])
        self.assertIn(b"\xff\xd8\x02\xff\xd9", bird["body"])

    def test_recording_status_reports_both_independent_views(self):
        payload = self.route("/api/video/recording/status")["payload"]
        self.assertTrue(payload["enabled"])
        self.assertEqual(payload["session_id"], "recording-test")
        self.assertEqual(payload["views"]["mainrs"]["frame_count"], 3)
        self.assertEqual(payload["views"]["birdrs"]["frame_count"], 4)


class VideoUiContractTests(unittest.TestCase):
    def test_dual_and_split_controls_use_view_endpoints(self):
        html = (ROOT / "tools/dashboard/static/index.html").read_text(
            encoding="utf-8"
        )
        javascript = (ROOT / "tools/dashboard/static/app.js").read_text(
            encoding="utf-8"
        )
        stylesheet = (ROOT / "tools/dashboard/static/style.css").read_text(
            encoding="utf-8"
        )
        for identity in (
            "tabMainrs", "tabBirdrs", "tabSplit", "tabMap",
            "mainrsImg", "birdrsImg", "videoGrid",
        ):
            self.assertIn(f'id="{identity}"', html)
        self.assertIn("/api/video/${view}/stream", javascript)
        self.assertIn("/api/video/${view}/status", javascript)
        self.assertIn('setViewTab("split")', javascript)
        self.assertIn(".video-grid.mode-split", stylesheet)

    def test_remaining_time_and_data_source_are_explicit(self):
        html = (ROOT / "tools/dashboard/static/index.html").read_text(
            encoding="utf-8"
        )
        javascript = (ROOT / "tools/dashboard/static/app.js").read_text(
            encoding="utf-8"
        )
        stylesheet = (ROOT / "tools/dashboard/static/style.css").read_text(
            encoding="utf-8"
        )
        self.assertIn('<p>남은 시간</p>', html)
        self.assertIn('id="dataSourceBadge"', html)
        self.assertIn('id="finalizeBtn"', html)
        self.assertEqual(html.count("<article"), html.count("</article>"))
        self.assertIn('summary.official_evaluation === "CONFIG_MISMATCH"', javascript)
        self.assertIn("평가 경로 불일치 · 점수 보류", javascript)
        self.assertIn("decoderStatus.unframed_bytes", javascript)
        self.assertIn("TCP SYN 없이 관측된", javascript)
        self.assertIn("white-space: nowrap", stylesheet)


class FakeAssessment:
    def __init__(self):
        self.calls = []

    def finalize_incomplete(self, reason="manual_stop"):
        self.calls.append(reason)
        return True

    def snapshot(self):
        return {"status": "ABORTED", "finish_reason": "manual_stop"}


class FinalizeEndpointTests(unittest.TestCase):
    def setUp(self):
        self.old_assessment = server.LIVE_ASSESSMENT
        self.assessment = FakeAssessment()
        server.LIVE_ASSESSMENT = self.assessment

    def tearDown(self):
        server.LIVE_ASSESSMENT = self.old_assessment

    def request(self, address="127.0.0.1", action="finalize"):
        handler = object.__new__(server.Handler)
        handler.path = "/api/live/finalize"
        handler.client_address = (address, 12345)
        handler.headers = {"X-HL-VTD-Action": action}
        result = {}
        handler._send_json = lambda payload, status=200: result.update(
            payload=payload, status=status
        )
        handler.do_POST()
        return result

    def test_confirmed_loopback_request_finalizes_as_incomplete(self):
        result = self.request()
        self.assertEqual(result["status"], 200)
        self.assertTrue(result["payload"]["finalized"])
        self.assertEqual(result["payload"]["assessment"]["status"], "ABORTED")
        self.assertEqual(self.assessment.calls, ["manual_stop"])

    def test_remote_or_unconfirmed_request_cannot_finalize(self):
        self.assertEqual(self.request("192.0.2.10")["status"], 403)
        self.assertEqual(self.request(action="")["status"], 403)
        self.assertEqual(self.assessment.calls, [])


if __name__ == "__main__":
    unittest.main()
