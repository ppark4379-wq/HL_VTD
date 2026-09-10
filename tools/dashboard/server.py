#!/usr/bin/env python3
"""Passive VTD RDB assessment + shared local video + optional CSV history.

Runs on the VTD PC, independent of the participant controller. Requires Python
and PyYAML; local X11 video additionally requires ffmpeg and xwininfo.
See docs/dashboard_auto_assessment.md for scenario metadata and unresolved rules.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import shlex
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
STATIC_DIR = Path(__file__).resolve().parent / "static"
NETWORK_CONFIG_PATH = REPO_ROOT / "config" / "network.yaml"
ACTIVE_ASSESSMENT_PROFILE = Path("/tmp/hl_vtd_active_assessment.json")

VTD_WINDOW_TITLES = ("igVisualBroadcast", "cam1", "mainRS", "birdRS")
VIDEO_FRAMERATE = 10
VIDEO_SOURCE_MODE = "auto"
VIDEO_RTSP_URL = None
VIDEO_RTSP_URLS = {"mainrs": None, "birdrs": None}

from dashboard_api import (
    DashboardRuntime,
    live_payload,
    map_payload,
    run_payload,
    runs_payload,
    scenarios_payload,
    video_recording_status_payload,
    video_status_payload,
)
from live_assessment import LiveAssessment
from rdb_stream import RdbObserver
from rdb_recording import RdbRecordingSession
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
from video_recording import VideoRecordingExistsError, VideoRecordingSession
from video_source import (
    VideoSourceConfig,
    configured_rtsp_url as read_configured_rtsp_url,
    find_vtd_windows as discover_vtd_windows,
    video_capture_command as build_video_capture_command,
)
from control_monitor import ControlEvents
from geometry_service import GeometryReports
from scp_observation import PassiveScpObserver, PluginRespawnLogObserver

LIVE_ASSESSMENT = None
RDB_OBSERVER = None
SCP_OBSERVER = None
RESPAWN_LOG_OBSERVER = None
VIDEO_RELAY = None
VIDEO_RELAYS = {}
VIDEO_RECORDING = None
CONTROL_EVENTS = None
GEOMETRY_REPORTS = None
GEOMETRY_INIT_LOCK = threading.Lock()


def geometry_reports():
    global GEOMETRY_REPORTS
    with GEOMETRY_INIT_LOCK:
        if GEOMETRY_REPORTS is None:
            GEOMETRY_REPORTS = GeometryReports()
        return GEOMETRY_REPORTS

def find_mainrs_window() -> dict | None:
    return discover_vtd_windows()["mainrs"]


def find_video_window(view: str) -> dict | None:
    if view not in {"mainrs", "birdrs"}:
        raise ValueError(f"unsupported video view: {view}")
    return discover_vtd_windows()[view]


def configured_rtsp_url() -> str | None:
    return read_configured_rtsp_url(NETWORK_CONFIG_PATH)


def video_capture_command(view: str = "mainrs"):
    rtsp_url = VIDEO_RTSP_URLS.get(view)
    if view == "mainrs" and rtsp_url is None:
        rtsp_url = VIDEO_RTSP_URL
    config = VideoSourceConfig(
        mode=VIDEO_SOURCE_MODE,
        rtsp_url=rtsp_url,
        framerate=VIDEO_FRAMERATE,
    )
    return build_video_capture_command(
        config,
        lambda: find_video_window(view),
        view=view,
    )


def dashboard_runtime() -> DashboardRuntime:
    return DashboardRuntime(
        live_assessment=LIVE_ASSESSMENT,
        rdb_observer=RDB_OBSERVER,
        control_events=CONTROL_EVENTS,
        video_relay=VIDEO_RELAY,
        video_relays=VIDEO_RELAYS,
        video_recording=VIDEO_RECORDING,
        scp_observer=SCP_OBSERVER,
        respawn_log_observer=RESPAWN_LOG_OBSERVER,
    )


def get_video_relay(view: str):
    relay = VIDEO_RELAYS.get(view)
    if relay is None and view == "mainrs":
        relay = VIDEO_RELAY
    return relay


def close_runtime_component(label: str, component) -> None:
    if component is None:
        return
    try:
        component.close()
    except Exception as error:
        print(f"[dashboard] warning: cannot close {label}: {error}", file=sys.stderr)


def resolve_video_recording_directory(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    verification_root = VERIFICATION_DIR.resolve()
    if resolved == verification_root or resolved.is_relative_to(verification_root):
        raise ValueError(
            "--record-video-dir must be outside output/verification; use a "
            "separate unique directory such as output/video_evidence/<run-id>"
        )
    return resolved


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
        # The dashboard is restarted in place while validating VTD runs. Do
        # not let a browser keep an older app.js/style.css after a server-side
        # fix or mix assets from two revisions.
        self.send_header("Cache-Control", "no-store")
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
            elif parsed.path == "/api/geometry":
                self._send_json(geometry_reports().status(query, LIVE_ASSESSMENT))
            elif parsed.path.startswith('/geometry/'):
                path, content_type = geometry_reports().asset(parsed.path)
                self._send_file(path, content_type)
            elif parsed.path == "/api/criteria.md":
                self._send_file(REPO_ROOT / 'docs' / 'HL_VTD_자동채점_최종평가기준.md', 'text/markdown; charset=utf-8')
            elif parsed.path == "/api/runs":
                self._send_json(runs_payload(query))
            elif parsed.path == "/api/map":
                self._send_json(map_payload(query))
            elif parsed.path == "/api/run":
                self._send_json(run_payload(query))
            elif parsed.path in {"/api/mainrs_status", "/api/video/mainrs/status"}:
                self._send_json(video_status_payload(dashboard_runtime(), "mainrs"))
            elif parsed.path == "/api/video/birdrs/status":
                self._send_json(video_status_payload(dashboard_runtime(), "birdrs"))
            elif parsed.path == "/api/video/recording/status":
                self._send_json(video_recording_status_payload(dashboard_runtime()))
            elif parsed.path in {"/api/mainrs_stream", "/api/video/mainrs/stream"}:
                self._stream_shared_video("mainrs")
            elif parsed.path == "/api/video/birdrs/stream":
                self._stream_shared_video("birdrs")
            else:
                self._send_json({"error": "not found"}, status=404)
        except FileNotFoundError as error:
            self._send_json({"error": str(error)}, status=404)
        except ValueError as error:
            self._send_json({"error": str(error)}, status=400)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != "/api/live/finalize":
            self._send_json({"error": "not found"}, status=404)
            return
        try:
            peer = ipaddress.ip_address(self.client_address[0])
        except (ValueError, IndexError):
            self._send_json({"error": "invalid client address"}, status=403)
            return
        # This endpoint mutates assessment state.  Loopback-only access plus a
        # custom header prevents LAN clients and ordinary cross-origin forms
        # from ending a run accidentally.
        if not peer.is_loopback or self.headers.get("X-HL-VTD-Action") != "finalize":
            self._send_json({"error": "local dashboard confirmation required"}, status=403)
            return
        if LIVE_ASSESSMENT is None:
            self._send_json({"error": "live assessment is disabled"}, status=409)
            return
        finalize = getattr(LIVE_ASSESSMENT, "finalize_incomplete", None)
        if finalize is None:
            self._send_json({"error": "live finalization is unavailable"}, status=501)
            return
        changed = bool(finalize(reason="manual_stop"))
        self._send_json({"finalized": changed, "assessment": LIVE_ASSESSMENT.snapshot()})

    def _stream_shared_video(self, view="mainrs"):
        relay = get_video_relay(view)
        sequence,frame = relay.wait_frame(-1) if relay else (-1,None)
        if frame is None:
            self._send_json(
                relay.status() if relay else {
                    "found": False, "view": view, "source": None,
                    "error": f"{view} video unavailable",
                },
                503,
            )
            return
        self.send_response(200)
        self.send_header("Content-Type","multipart/x-mixed-replace; boundary=hlvtdframe")
        self.send_header("Cache-Control","no-store")
        self.end_headers()
        try:
            while frame is not None:
                self.wfile.write((f"--hlvtdframe\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n").encode()+frame+b"\r\n")
                self.wfile.flush()
                sequence,frame=relay.wait_frame(sequence)
        except (BrokenPipeError,ConnectionResetError):
            pass



def main() -> int:
    global VIDEO_SOURCE_MODE, VIDEO_RTSP_URL, VIDEO_RTSP_URLS
    global LIVE_ASSESSMENT, RDB_OBSERVER, CONTROL_EVENTS
    global VIDEO_RELAY, VIDEO_RELAYS, VIDEO_RECORDING
    global GEOMETRY_REPORTS
    global SCP_OBSERVER
    global RESPAWN_LOG_OBSERVER

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-live", action="store_true", help="CSV history mode only")
    parser.add_argument("--rdb-host", default="127.0.0.1", help="VTD state stream host (dashboard runs on VTD PC)")
    parser.add_argument("--rdb-port", type=int, default=48190)
    parser.add_argument('--scp-host', help='Receive-only VTD SCP host (default: --rdb-host)')
    parser.add_argument('--scp-port', type=int, default=48179)
    parser.add_argument('--no-scp-observation', action='store_true', help='Disable receive-only SCP cause/event logging')
    parser.add_argument('--respawn-log', type=Path, default=Path('/tmp/taskRec_ModuleManager.txt'),
                        help='Read new HLVTD [RESPAWN] lines without changing the simulator')
    parser.add_argument('--no-respawn-log-observation', action='store_true')
    parser.add_argument('--record-rdb-dir', type=Path,
        help='Record unmodified RDB bytes and all decoded frames in a new/empty evidence directory')
    parser.add_argument("--assessment-config", type=Path, default=Path(__file__).with_name("assessment_rules.yaml"))
    parser.add_argument(
        "--active-assessment-profile",
        type=Path,
        default=ACTIVE_ASSESSMENT_PROFILE,
        help=(
            "Route profile published by tools/verification/run_scenario.py "
            f"(default: {ACTIVE_ASSESSMENT_PROFILE})"
        ),
    )
    parser.add_argument(
        "--video-source",
        choices=("auto", "rtsp", "window"),
        default="auto",
        help="VTD video input (default: auto)",
    )
    parser.add_argument(
        "--rtsp-url",
        help="Legacy mainRS RTSP URL (default: config/network.yaml + /broadcast)",
    )
    parser.add_argument(
        "--mainrs-rtsp-url",
        help="mainRS RTSP URL (overrides --rtsp-url)",
    )
    parser.add_argument(
        "--birdrs-rtsp-url",
        help="birdRS RTSP URL; omitted means birdRS is unavailable without its X11 window",
    )
    parser.add_argument(
        "--record-video-dir",
        type=Path,
        help=(
            "Record mainRS and birdRS as separate raw MJPEG + JSONL evidence "
            "files in a new or empty directory"
        ),
    )
    parser.add_argument(
        "--record-video-min-free-mb",
        type=float,
        default=1024.0,
        help=(
            "Stop video recording before free space falls below this many MiB "
            "(default: 1024)"
        ),
    )
    args = parser.parse_args()
    if args.scp_port == 9910:
        parser.error('SCP observation must not use the controller port 9910')
    GEOMETRY_REPORTS = GeometryReports(profile_path=args.active_assessment_profile)
    if args.rdb_port == 9910:
        parser.error("Use the read-only RDB output port, not the controller port 9910")
    record_rdb_dir = None
    if args.record_rdb_dir is not None:
        if args.no_live:
            parser.error('--record-rdb-dir requires live RDB observation')
        record_rdb_dir = args.record_rdb_dir.expanduser().resolve()
        if record_rdb_dir == VERIFICATION_DIR.resolve() or record_rdb_dir.is_relative_to(VERIFICATION_DIR.resolve()):
            parser.error('--record-rdb-dir must be outside output/verification, e.g. output/rdb_evidence/<run-id>')

    VIDEO_SOURCE_MODE = args.video_source
    VIDEO_RTSP_URL = args.mainrs_rtsp_url or args.rtsp_url or configured_rtsp_url()
    VIDEO_RTSP_URLS = {
        "mainrs": VIDEO_RTSP_URL,
        "birdrs": args.birdrs_rtsp_url,
    }
    for view, url in VIDEO_RTSP_URLS.items():
        if url and not url.startswith("rtsp://"):
            parser.error(f"--{view}-rtsp-url must start with rtsp://")
    if VIDEO_SOURCE_MODE == "rtsp" and not VIDEO_RTSP_URLS["mainrs"]:
        parser.error("RTSP source selected but no mainRS RTSP URL is configured")
    if args.record_video_min_free_mb <= 0:
        parser.error("--record-video-min-free-mb must be greater than zero")
    record_video_dir = None
    if args.record_video_dir is not None:
        try:
            record_video_dir = resolve_video_recording_directory(
                args.record_video_dir
            )
        except ValueError as error:
            parser.error(str(error))

    LIVE_ASSESSMENT = None
    RDB_OBSERVER = None
    SCP_OBSERVER = None
    RESPAWN_LOG_OBSERVER = None
    CONTROL_EVENTS = None
    VIDEO_RELAY = None
    VIDEO_RELAYS = {}
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    VIDEO_RECORDING = None
    if record_video_dir is not None:
        try:
            VIDEO_RECORDING = VideoRecordingSession(
                record_video_dir,
                int(args.record_video_min_free_mb * 1024 * 1024),
            )
        except (OSError, ValueError, VideoRecordingExistsError) as error:
            server.server_close()
            print(
                f"[dashboard] video recording was not started: {error}",
                file=sys.stderr,
            )
            return 2
    try:
        rdb_recording = None
        if not args.no_live:
            LIVE_ASSESSMENT = LiveAssessment(
                args.assessment_config,
                REPO_ROOT / "output" / "assessment",
                args.active_assessment_profile,
            )
            if record_rdb_dir is not None:
                rdb_recording = RdbRecordingSession(record_rdb_dir, args.rdb_host, args.rdb_port)
            RDB_OBSERVER = RdbObserver(
                args.rdb_host,
                args.rdb_port,
                LIVE_ASSESSMENT.on_frame,
                LIVE_ASSESSMENT.on_gap,
                recorder=rdb_recording,
            )
    except BaseException:
        if rdb_recording is not None:
            rdb_recording.close()
        if VIDEO_RECORDING is not None:
            VIDEO_RECORDING.close()
        server.server_close()
        raise
    VIDEO_RELAYS = {
        view: VideoRelay(
            lambda view=view: video_capture_command(view),
            recording=VIDEO_RECORDING,
            view=view,
        )
        for view in ("mainrs", "birdrs")
    }
    VIDEO_RELAY = VIDEO_RELAYS["mainrs"]  # legacy API/tests
    started_relays = []
    try:
        for relay in VIDEO_RELAYS.values():
            started_relays.append(relay)
            relay.start()
        if RDB_OBSERVER is not None:
            RDB_OBSERVER.start()
            if not args.no_scp_observation:
                SCP_OBSERVER = PassiveScpObserver(args.scp_host or args.rdb_host, args.scp_port,
                                                  LIVE_ASSESSMENT.on_scp_event)
                SCP_OBSERVER.start()
            if not args.no_respawn_log_observation:
                RESPAWN_LOG_OBSERVER = PluginRespawnLogObserver(args.respawn_log, LIVE_ASSESSMENT.on_scp_event)
                RESPAWN_LOG_OBSERVER.start()
            CONTROL_EVENTS=ControlEvents(LIVE_ASSESSMENT.on_control_received)
            CONTROL_EVENTS.start()
    except BaseException:
        close_runtime_component("control receiver", CONTROL_EVENTS)
        close_runtime_component("SCP observer", SCP_OBSERVER)
        close_runtime_component("respawn log observer", RESPAWN_LOG_OBSERVER)
        for relay in started_relays:
            close_runtime_component("video relay", relay)
        close_runtime_component("video recording", VIDEO_RECORDING)
        close_runtime_component("RDB observer", RDB_OBSERVER)
        close_runtime_component("live assessment", LIVE_ASSESSMENT)
        server.server_close()
        raise
    print(f"[dashboard] http://{args.host}:{args.port} 에서 서비스 중 (Ctrl+C 로 종료)",flush=True)
    if rdb_recording is not None:
        print(f'[dashboard] complete RDB evidence: {rdb_recording.directory}', flush=True)
    if VIDEO_RECORDING is not None:
        recording_status = VIDEO_RECORDING.status()
        print(
            "[dashboard] video evidence: "
            f"{recording_status['directory']} "
            f"({recording_status['status']}, session "
            f"{recording_status['session_id']})",
            flush=True,
        )
        if not recording_status["active"]:
            print(
                "[dashboard] warning: video recording is not active: "
                f"{recording_status['error']}",
                file=sys.stderr,
                flush=True,
            )
    if CONTROL_EVENTS and CONTROL_EVENTS.status['socket']:
        print('[dashboard] 제어기를 켜기 전, VTD PC의 다른 터미널에서 수신 감지기를 먼저 실행하세요 (웹 서버에는 sudo 불필요):',flush=True)
        control_log_dir=REPO_ROOT/'output'/'control_evidence'
        try:
            control_log_dir.mkdir(parents=True,exist_ok=True)
            control_log=control_log_dir/(
                time.strftime('%Y%m%d_%H%M%S')+'_'+
                Path(CONTROL_EVENTS.status['socket']).parent.name+'.jsonl'
            )
            log_option=' --log '+shlex.quote(str(control_log))
        except OSError as error:
            log_option=''
            print(
                f'[dashboard] 주의: 제어 증거 로그 디렉터리를 만들 수 없습니다: {error}',
                file=sys.stderr,flush=True,
            )
        print('sudo python3 '+shlex.quote(str(Path(__file__).with_name('control_monitor.py')))+' --event-socket '+shlex.quote(CONTROL_EVENTS.status['socket'])+log_option,flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        close_runtime_component("geometry reports", GEOMETRY_REPORTS)
        close_runtime_component("SCP observer", SCP_OBSERVER)
        close_runtime_component("respawn log observer", RESPAWN_LOG_OBSERVER)
        close_runtime_component("control receiver", CONTROL_EVENTS)
        for view, relay in zip(VIDEO_RELAYS, started_relays):
            close_runtime_component(f"{view} video relay", relay)
        close_runtime_component("video recording", VIDEO_RECORDING)
        close_runtime_component("RDB observer", RDB_OBSERVER)
        close_runtime_component("live assessment", LIVE_ASSESSMENT)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
