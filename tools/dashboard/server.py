#!/usr/bin/env python3
"""Passive VTD RDB assessment + shared local video + optional CSV history.

Runs on the VTD PC, independent of the participant controller. Requires Python
and PyYAML; local X11 video additionally requires ffmpeg and xwininfo.
See docs/dashboard_auto_assessment.md for scenario metadata and unresolved rules.
"""

from __future__ import annotations

import argparse
import json
import shlex
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
STATIC_DIR = Path(__file__).resolve().parent / "static"
NETWORK_CONFIG_PATH = REPO_ROOT / "config" / "network.yaml"

VTD_WINDOW_TITLES = ("igVisualBroadcast", "cam1", "mainRS", "birdRS")
VIDEO_FRAMERATE = 10
VIDEO_SOURCE_MODE = "auto"
VIDEO_RTSP_URL = None

from dashboard_api import (
    DashboardRuntime,
    live_payload,
    map_payload,
    run_payload,
    runs_payload,
    scenarios_payload,
    video_status_payload,
)
from live_assessment import LiveAssessment
from rdb_stream import RdbObserver
from run_repository import (
    CATALOG_PATH,
    OFFICIAL_ITEM_LABELS,
    VERIFICATION_DIR,
    build_map_payload,
    build_run_payload,
    catalog_by_id,
    latest_row,
    list_runs,
    load_catalog,
    official_grid,
    read_run_rows,
    section_columns,
    to_float,
)
from video_relay import VideoRelay
from video_source import (
    VideoSourceConfig,
    configured_rtsp_url as read_configured_rtsp_url,
    find_mainrs_window as discover_mainrs_window,
    video_capture_command as build_video_capture_command,
)
from control_monitor import ControlEvents

LIVE_ASSESSMENT = None
RDB_OBSERVER = None
VIDEO_RELAY = None
CONTROL_EVENTS = None

def find_mainrs_window() -> dict | None:
    return discover_mainrs_window(VTD_WINDOW_TITLES)


def configured_rtsp_url() -> str | None:
    return read_configured_rtsp_url(NETWORK_CONFIG_PATH)


def video_capture_command():
    config = VideoSourceConfig(
        mode=VIDEO_SOURCE_MODE,
        rtsp_url=VIDEO_RTSP_URL,
        framerate=VIDEO_FRAMERATE,
    )
    return build_video_capture_command(config, find_mainrs_window)


def dashboard_runtime() -> DashboardRuntime:
    return DashboardRuntime(
        live_assessment=LIVE_ASSESSMENT,
        rdb_observer=RDB_OBSERVER,
        control_events=CONTROL_EVENTS,
        video_relay=VIDEO_RELAY,
    )


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:  # noqa: A002
        sys.stderr.write(f"[dashboard] {self.address_string()} {format % args}\n")

    def _send_json(self, payload, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, content_type: str) -> None:
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)

        try:
            if parsed.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
            elif parsed.path == "/" or parsed.path == "/index.html":
                self._send_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
            elif parsed.path == "/static/app.js":
                self._send_file(STATIC_DIR / "app.js", "application/javascript; charset=utf-8")
            elif parsed.path == "/static/style.css":
                self._send_file(STATIC_DIR / "style.css", "text/css; charset=utf-8")
            elif parsed.path == "/api/scenarios":
                self._send_json(scenarios_payload())
            elif parsed.path == "/api/live":
                self._send_json(live_payload(dashboard_runtime()))
            elif parsed.path == "/api/criteria.md":
                self._send_file(REPO_ROOT / 'docs' / 'HL_VTD_자동채점_최종평가기준.md', 'text/markdown; charset=utf-8')
            elif parsed.path == "/api/runs":
                self._send_json(runs_payload(query))
            elif parsed.path == "/api/map":
                self._send_json(map_payload(query))
            elif parsed.path == "/api/run":
                self._send_json(run_payload(query))
            elif parsed.path == "/api/mainrs_status":
                self._send_json(video_status_payload(dashboard_runtime()))
            elif parsed.path == "/api/mainrs_stream":
                self._stream_shared_video()
            else:
                self._send_json({"error": "not found"}, status=404)
        except FileNotFoundError as error:
            self._send_json({"error": str(error)}, status=404)
        except ValueError as error:
            self._send_json({"error": str(error)}, status=400)

    def _stream_shared_video(self):
        sequence,frame = VIDEO_RELAY.wait_frame(-1) if VIDEO_RELAY else (-1,None)
        if frame is None:
            self._send_json(VIDEO_RELAY.status() if VIDEO_RELAY else {"error":"video unavailable"},503)
            return
        self.send_response(200)
        self.send_header("Content-Type","multipart/x-mixed-replace; boundary=hlvtdframe")
        self.send_header("Cache-Control","no-store")
        self.end_headers()
        try:
            while frame is not None:
                self.wfile.write((f"--hlvtdframe\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n").encode()+frame+b"\r\n")
                self.wfile.flush()
                sequence,frame=VIDEO_RELAY.wait_frame(sequence)
        except (BrokenPipeError,ConnectionResetError):
            pass



def main() -> int:
    global VIDEO_SOURCE_MODE, VIDEO_RTSP_URL
    global LIVE_ASSESSMENT, RDB_OBSERVER, CONTROL_EVENTS
    global VIDEO_RELAY

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-live", action="store_true", help="CSV history mode only")
    parser.add_argument("--rdb-host", default="127.0.0.1", help="VTD state stream host (dashboard runs on VTD PC)")
    parser.add_argument("--rdb-port", type=int, default=48190)
    parser.add_argument("--assessment-config", type=Path, default=Path(__file__).with_name("assessment_rules.yaml"))
    parser.add_argument(
        "--video-source",
        choices=("auto", "rtsp", "window"),
        default="auto",
        help="VTD video input (default: auto)",
    )
    parser.add_argument(
        "--rtsp-url",
        help="VTD RTSP URL (default: config/network.yaml + /broadcast)",
    )
    args = parser.parse_args()
    if args.rdb_port == 9910:
        parser.error("Use the read-only RDB output port, not the controller port 9910")

    VIDEO_SOURCE_MODE = args.video_source
    VIDEO_RTSP_URL = args.rtsp_url or configured_rtsp_url()
    if VIDEO_RTSP_URL and not VIDEO_RTSP_URL.startswith("rtsp://"):
        parser.error("--rtsp-url must start with rtsp://")
    if VIDEO_SOURCE_MODE == "rtsp" and not VIDEO_RTSP_URL:
        parser.error("RTSP source selected but no RTSP URL is configured")

    if not args.no_live:
        LIVE_ASSESSMENT = LiveAssessment(args.assessment_config, REPO_ROOT / "output" / "assessment")
        RDB_OBSERVER = RdbObserver(args.rdb_host, args.rdb_port, LIVE_ASSESSMENT.on_frame, LIVE_ASSESSMENT.on_gap)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    VIDEO_RELAY = VideoRelay(video_capture_command)
    VIDEO_RELAY.start()
    if RDB_OBSERVER is not None:
        RDB_OBSERVER.start()
        CONTROL_EVENTS=ControlEvents(LIVE_ASSESSMENT.on_control_received)
        CONTROL_EVENTS.start()
    print(f"[dashboard] http://{args.host}:{args.port} 에서 서비스 중 (Ctrl+C 로 종료)",flush=True)
    if CONTROL_EVENTS and CONTROL_EVENTS.status['socket']:
        print('[dashboard] VTD PC의 다른 터미널에서 제어 수신 감지기를 실행하세요 (웹 서버에는 sudo 불필요):',flush=True)
        print('sudo python3 '+shlex.quote(str(Path(__file__).with_name('control_monitor.py')))+' --event-socket '+shlex.quote(CONTROL_EVENTS.status['socket']),flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if CONTROL_EVENTS is not None:
            CONTROL_EVENTS.close()
        if VIDEO_RELAY is not None:
            VIDEO_RELAY.close()
        if RDB_OBSERVER is not None:
            RDB_OBSERVER.close()
        if LIVE_ASSESSMENT is not None:
            LIVE_ASSESSMENT.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
