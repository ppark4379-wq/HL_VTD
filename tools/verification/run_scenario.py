#!/usr/bin/env python3
# Validate one Phase 1 Scenario Package and start HL_VTD with its default route.

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import re
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

from validate_scenario import REPO_ROOT, load_manifest, validate_scenario
from vtd_scp import ScpError, VtdScpClient
from scenario_parameters import ParameterError, render_scenario, resolve_parameters

import yaml


RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")
DEFAULT_TASK_CONTROL_LOG = Path("/tmp/taskRec_TaskControl.txt")
TRAFFIC_LOSS_MARKERS = (
    "no traffic info available",
    "waiting for connection <traffic>",
    "<traffic> disconnected",
    "connection <traffic> lost",
)
TRAFFIC_CONNECTED_MARKER = '"<traffic> connected"'


def scenario_requires_traffic(scenario_file: Path) -> bool:
    root = ET.parse(scenario_file).getroot()
    return any(
        description.get("Control", "").lower() == "internal"
        for description in root.findall(".//TrafficControl/Player/Description")
    )


def ghostdriver_pids() -> set[int]:
    pids: set[int] = set()
    for proc_entry in Path("/proc").glob("[0-9]*"):
        try:
            command_name = (proc_entry / "comm").read_text(
                encoding="utf-8"
            ).strip()
        except (OSError, UnicodeError):
            continue
        if command_name.startswith("ghostdriver"):
            pids.add(int(proc_entry.name))
    return pids


@dataclass
class TrafficHealthMonitor:
    log_path: Path
    timeout: float
    monitor_process: bool = True
    recovery_quiet_period: float = 0.6
    offset: int = 0
    file_identity: tuple[int, int] | None = None
    fragment: str = ""
    initial_pids: set[int] = field(default_factory=set)
    loss_started: float | None = None
    last_loss_signal: float | None = None
    last_reason: str = ""

    def arm(self) -> None:
        try:
            stat = self.log_path.stat()
            self.offset = stat.st_size
            self.file_identity = (stat.st_dev, stat.st_ino)
        except OSError:
            self.offset = 0
            self.file_identity = None
        self.fragment = ""
        self.initial_pids = ghostdriver_pids() if self.monitor_process else set()
        self.loss_started = None
        self.last_loss_signal = None
        self.last_reason = ""

    def _new_log_lines(self) -> list[str]:
        try:
            stat = self.log_path.stat()
        except OSError:
            return []
        identity = (stat.st_dev, stat.st_ino)
        if self.file_identity != identity or stat.st_size < self.offset:
            self.offset = 0
            self.fragment = ""
            self.file_identity = identity
        try:
            with self.log_path.open("rb") as stream:
                stream.seek(self.offset)
                data = stream.read()
                self.offset = stream.tell()
        except OSError:
            return []
        if not data:
            return []
        text = self.fragment + data.decode("utf-8", errors="replace")
        lines = text.splitlines(keepends=True)
        if lines and not lines[-1].endswith(("\n", "\r")):
            self.fragment = lines.pop()
        else:
            self.fragment = ""
        return [line.rstrip("\r\n") for line in lines]

    def _mark_loss(self, now: float, reason: str) -> None:
        if self.loss_started is None:
            self.loss_started = now
        self.last_loss_signal = now
        self.last_reason = reason

    def _mark_recovered(self) -> None:
        self.loss_started = None
        self.last_loss_signal = None
        self.last_reason = ""

    def poll(self, now: float | None = None) -> str | None:
        current_time = time.monotonic() if now is None else now
        for line in self._new_log_lines():
            normalized = line.lower()
            if TRAFFIC_CONNECTED_MARKER in normalized:
                self._mark_recovered()
            for marker in TRAFFIC_LOSS_MARKERS:
                if marker in normalized:
                    self._mark_loss(current_time, line.strip())
                    break

        if self.monitor_process and self.initial_pids:
            current_pids = ghostdriver_pids()
            if not current_pids:
                self._mark_loss(current_time, "ghostdriver process exited")

        if self.loss_started is None or self.last_loss_signal is None:
            return None
        if current_time - self.last_loss_signal > self.recovery_quiet_period:
            self._mark_recovered()
            return None
        if current_time - self.loss_started < self.timeout:
            return None
        return (
            f"Traffic health check failed after {self.timeout:g}s: "
            f"{self.last_reason}"
        )


def default_run_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def sync_vtd_source_before_run(package: Path) -> bool:
    manifest_path = package.resolve() / "scenario.yaml"
    try:
        manifest = load_manifest(manifest_path)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"[run_scenario] cannot read source-sync setting: {error}",
              file=sys.stderr)
        return False
    runtime = manifest.get("runtime", {})
    if not isinstance(runtime, dict):
        return True
    raw_source = runtime.get("sync_source_before_run")
    if raw_source is None:
        return True
    if not isinstance(raw_source, str) or not raw_source.strip():
        print(
            "[run_scenario] runtime.sync_source_before_run must be a path",
            file=sys.stderr,
        )
        return False
    source = Path(raw_source).expanduser().resolve()
    if not source.is_file():
        print(f"[run_scenario] VTD sync source not found: {source}",
              file=sys.stderr)
        return False
    sync_tool = REPO_ROOT / "tools" / "sync_vtd_scenario.py"
    print(f"[run_scenario] syncing VTD source before run: {source}", flush=True)
    completed = subprocess.run(
        [sys.executable, str(sync_tool), str(source)],
        cwd=REPO_ROOT,
        check=False,
    )
    if completed.returncode != 0:
        print(
            "[run_scenario] VTD source sync failed; run was not started.",
            file=sys.stderr,
        )
        return False
    return True


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
        "--config",
        type=Path,
        default=REPO_ROOT / "config" / "network.yaml",
        help="HL_VTD network config (default: config/network.yaml)",
    )
    parser.add_argument(
        "--auto-vtd",
        action="store_true",
        help="Load, Apply, Play, and Stop VTD through SCP",
    )
    parser.add_argument(
        "--scp-host",
        default=None,
        help="VTD SCP host (default: vtd.host_ip from --config)",
    )
    parser.add_argument(
        "--scp-port",
        type=int,
        default=48179,
        help="VTD SCP port (default: 48179)",
    )
    parser.add_argument(
        "--vtd-scenario-file",
        type=Path,
        default=None,
        help=(
            "Scenario path as seen by the VTD PC "
            "(default: validated local XML path)"
        ),
    )
    parser.add_argument(
        "--inline-scenario",
        action="store_true",
        help="Embed Scenario XML in SCP instead of loading a VTD-side file",
    )
    parser.add_argument(
        "--skip-source-sync",
        action="store_true",
        help="Run the existing local package without overwriting it from VTD source",
    )
    parser.add_argument(
        "--vtd-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for each VTD state transition",
    )
    parser.add_argument(
        "--run-timeout",
        type=float,
        default=None,
        help="Maximum scenario runtime in seconds (default: unlimited)",
    )
    parser.add_argument(
        "--traffic-loss-timeout",
        type=float,
        default=1.0,
        help=(
            "Fail an auto-VTD run when Traffic is disconnected for this many "
            "seconds; 0 disables the check (default: 1.0)"
        ),
    )
    parser.add_argument(
        "--task-control-log",
        type=Path,
        default=DEFAULT_TASK_CONTROL_LOG,
        help=(
            "Local TaskControl log used by the Traffic health check "
            f"(default: {DEFAULT_TASK_CONTROL_LOG})"
        ),
    )
    parser.add_argument(
        "--decision-log",
        type=Path,
        default=None,
        help="Write the detailed planning decision trace to this file",
    )
    parser.add_argument(
        "--rviz-data",
        action="store_true",
        help=(
            "Write static map and live planner state for the RViz bridge "
            "under this run's rviz directory"
        ),
    )
    parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="Override a bound scenario parameter (repeatable)",
    )
    parser.add_argument(
        "--randomize",
        action="store_true",
        help="Randomize parameters that declare a random min/max range",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed used with --randomize",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved files and command without starting HL_VTD",
    )
    args = parser.parse_args()
    if not 1 <= args.scp_port <= 65535:
        parser.error("--scp-port must be between 1 and 65535")
    if args.vtd_timeout <= 0:
        parser.error("--vtd-timeout must be greater than zero")
    if args.run_timeout is not None and args.run_timeout <= 0:
        parser.error("--run-timeout must be greater than zero")
    if args.traffic_loss_timeout < 0:
        parser.error("--traffic-loss-timeout must be zero or greater")
    return args


def build_command(
    binary: Path,
    network_config: Path,
    route_file: Path,
    scenario_id: str,
    run_id: str,
    log_file: Path,
    ready_file: Path | None = None,
    decision_log: Path | None = None,
    ignore_traffic_lights: bool = False,
    control_config: Path | None = None,
    checkpoints_file: Path | None = None,
    mission_output: Path | None = None,
    visualization_dir: Path | None = None,
    mission_start_leg: int = 0,
) -> list[str]:
    command = [
        str(binary),
        "--config",
        str(network_config),
        "--path",
        str(route_file),
        "--scenario-id",
        scenario_id,
        "--run-id",
        run_id,
        "--log",
        str(log_file),
    ]
    if ready_file is not None:
        command.extend(("--ready-file", str(ready_file)))
    if decision_log is not None:
        command.extend(("--decision-log", str(decision_log)))
    if ignore_traffic_lights:
        command.append("--ignore-traffic-lights")
    if control_config is not None:
        command.extend(("--control-config", str(control_config)))
    if checkpoints_file is not None:
        command.extend(("--checkpoints", str(checkpoints_file)))
        command.extend(("--mission-start-leg", str(mission_start_leg)))
    if mission_output is not None:
        command.extend(("--mission-output", str(mission_output)))
    if visualization_dir is not None:
        command.extend(("--visualization-dir", str(visualization_dir)))
    return command


def render_control_config(manifest: dict, destination: Path) -> Path | None:
    control = manifest.get("control", {})
    target_kph = control.get("target_speed_kph") if isinstance(control, dict) else None
    if target_kph is None:
        return None
    if not isinstance(target_kph, (int, float)) or target_kph <= 0:
        raise ValueError("control.target_speed_kph must be positive")
    config = load_manifest(REPO_ROOT / "config" / "control.yaml")
    speed = config.get("speed")
    if not isinstance(speed, dict):
        raise ValueError("speed mapping is missing from config/control.yaml")
    for key in (
        "straight_target_kph",
        "curve_target_kph",
        "sharp_curve_target_kph",
        "school_zone_target_kph",
    ):
        speed[key] = float(target_kph)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return destination


def load_vtd_host(network_config: Path) -> str:
    config = load_manifest(network_config)
    vtd = config.get("vtd")
    if not isinstance(vtd, dict) or not isinstance(vtd.get("host_ip"), str):
        raise ValueError(f"vtd.host_ip is missing from network config: {network_config}")
    return vtd["host_ip"]


def terminate_process(process: subprocess.Popen[bytes], timeout: float = 5.0) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def run_automated(
    args: argparse.Namespace,
    command: list[str],
    scenario_file: Path,
    scp_host: str,
    ready_file: Path,
    requires_traffic: bool,
) -> int:
    controller: subprocess.Popen[bytes] | None = None
    traffic_monitor: TrafficHealthMonitor | None = None
    with VtdScpClient(scp_host, args.scp_port, args.vtd_timeout) as scp:
        try:
            print(f"[auto-vtd] connected to SCP {scp_host}:{args.scp_port}")
            print(f"[auto-vtd] loading and applying {scenario_file.name}")
            scp.configure_scenario(
                scenario_file,
                args.vtd_timeout,
                inline=args.inline_scenario,
                # Reload the active Setup on every automated run so changes to
                # IG render surfaces (follower + bird popups) are applied even
                # when TaskControl was already sitting in READY.
                force_reconfigure=True,
            )
            print("[auto-vtd] VTD is READY; initializing scenario")
            try:
                # Image generators can need substantially longer than 15 seconds
                # to load the road and visual databases.  Use the configured VTD
                # transition timeout here as well; otherwise a healthy IG is torn
                # down while it is still initializing.
                scp.initialize(args.vtd_timeout)
            except ScpError:
                print(
                    "[auto-vtd] IG did not initialize; rebuilding the VTD setup"
                )
                scp.configure_scenario(
                    scenario_file,
                    args.vtd_timeout,
                    inline=args.inline_scenario,
                    force_reconfigure=True,
                )
                print("[auto-vtd] VTD setup recovered; retrying initialization")
                scp.initialize(args.vtd_timeout)
            scp.set_relative_camera(
                "VIEW_CAMERA",
                "Ego",
                dx=-6.5,
                dy=0.0,
                dz=2.6,
                channel_mask=0x08,
                view_name="cam1",
            )
            print(
                "[auto-vtd] follower and bird view popups configured for Ego"
            )
            print("[auto-vtd] VTD is initialized; starting HL_VTD")
            controller = subprocess.Popen(command, cwd=REPO_ROOT)
            try:
                controller.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                pass
            deadline = time.monotonic() + args.vtd_timeout
            while not ready_file.is_file():
                early_exit = controller.poll()
                if early_exit is not None:
                    raise ScpError(
                        f"HL_VTD exited before Play with status {early_exit}"
                    )
                if time.monotonic() >= deadline:
                    raise ScpError(
                        "HL_VTD did not connect to the VTD GT port within "
                        f"{args.vtd_timeout:g}s"
                    )
                try:
                    controller.wait(timeout=0.1)
                except subprocess.TimeoutExpired:
                    pass
            if (
                args.traffic_loss_timeout > 0
                and requires_traffic
            ):
                traffic_monitor = TrafficHealthMonitor(
                    args.task_control_log,
                    args.traffic_loss_timeout,
                    monitor_process=scp_host in {"127.0.0.1", "localhost", "::1"},
                )
                traffic_monitor.arm()
                print(
                    "[auto-vtd] Traffic health check enabled: "
                    f"{args.traffic_loss_timeout:g}s grace, "
                    f"log={args.task_control_log}"
                )
            print("[auto-vtd] HL_VTD is connected; sending Play")
            scp.start(args.vtd_timeout)
            # External Ego control must not fall back to Traffic/ghostdriver.
            # Besides competing with the HL_VTD participant, VTD 2025.2's
            # obstacle-bypass path can crash inside ghostdriver for this map.
            scp.set_player_driver_control("Ego", "extern")
            print("[auto-vtd] Ego driver control source set to extern")
            print("[auto-vtd] VTD is RUNNING; CSV logging is active")
            run_started = time.monotonic()
            while True:
                if traffic_monitor is not None:
                    traffic_failure = traffic_monitor.poll()
                    if traffic_failure is not None:
                        raise ScpError(traffic_failure)
                exit_status = controller.poll()
                if exit_status is not None:
                    return exit_status
                if (
                    args.run_timeout is not None
                    and time.monotonic() - run_started >= args.run_timeout
                ):
                    raise ScpError(
                        f"scenario exceeded --run-timeout {args.run_timeout:g}s"
                    )
                time.sleep(0.1)
        finally:
            if controller is not None:
                terminate_process(controller)
            ready_file.unlink(missing_ok=True)
            try:
                print("[auto-vtd] stopping VTD")
                scp.stop(args.vtd_timeout)
            except ScpError as error:
                print(f"[auto-vtd] warning: {error}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    if not args.skip_source_sync and not sync_vtd_source_before_run(args.package):
        return 2
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
    # Rendered scenario/config and the mission corridor visualization all belong
    # to this run. Create the run directory before any of them are generated.
    log_file.parent.mkdir(parents=True, exist_ok=True)
    checkpoints_file = (report.package / "checkpoints.csv").resolve()
    if not checkpoints_file.is_file():
        checkpoints_file = None
    mission_output = (
        log_file.parent / "mission_corridor.svg"
        if checkpoints_file is not None else None
    )
    visualization_dir = log_file.parent / "rviz" if args.rviz_data else None
    scenario_file = report.scenario_file.resolve()
    try:
        parameter_values = resolve_parameters(
            report.manifest, args.param, args.randomize, args.seed
        )
        bindings = report.manifest.get("runtime_bindings", [])
        if bindings:
            rendered_file = log_file.parent / "scenario.rendered.xml"
            applied = render_scenario(
                scenario_file, rendered_file, report.manifest, parameter_values
            )
            scenario_file = rendered_file
            for name, value in applied.items():
                print(f"[parameter] {name}={value:g}")
    except (OSError, ET.ParseError, ParameterError) as error:
        print(f"[run_scenario] parameter rendering failed: {error}", file=sys.stderr)
        return 2
    ready_file = log_file.parent / "controller.ready" if args.auto_vtd else None
    try:
        runtime_control_config = render_control_config(
            report.manifest, log_file.parent / "control.rendered.yaml"
        )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"[run_scenario] control config rendering failed: {error}", file=sys.stderr)
        return 2
    command = build_command(
        binary,
        args.config.resolve(),
        report.route_file,
        scenario_id,
        run_id,
        log_file,
        ready_file,
        args.decision_log.resolve() if args.decision_log else None,
        bool(report.manifest.get("control", {}).get("ignore_traffic_lights", False)),
        runtime_control_config,
        checkpoints_file,
        mission_output,
        visualization_dir,
        mission_start_leg=report.manifest.get("runtime", {}).get(
            "mission_start_leg", 0),
    )

    # Build the checkpoint corridor and its image before touching VTD. This
    # catches a disconnected mission early and guarantees that dry-runs also
    # leave the requested artifact in the run output directory.
    if mission_output is not None:
        if not binary.is_file():
            print(
                f"[run_scenario] HL_VTD executable not found: {binary}",
                file=sys.stderr,
            )
            return 2
        mission_check = command + ["--check-config"]
        completed = subprocess.run(
            mission_check, cwd=REPO_ROOT, check=False,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        if completed.returncode != 0:
            print(
                "[run_scenario] checkpoint corridor generation failed:\n"
                + completed.stdout + completed.stderr,
                file=sys.stderr,
            )
            return 2
        print(f"[mission] corridor map saved: {mission_output}")

    print()
    print(f"Scenario : {scenario_id}")
    print(f"File     : {scenario_file}")
    print(f"Route    : {report.route_file}")
    if checkpoints_file is not None:
        print(f"Mission  : {checkpoints_file}")
        print(f"Start leg: {report.manifest.get('runtime', {}).get('mission_start_leg', 0)} (0-based)")
        print(f"Map SVG  : {mission_output}")
    if visualization_dir is not None:
        print(f"RViz data: {visualization_dir}")
    print(f"Log      : {log_file}")
    print(f"Command  : {shlex.join(command)}")
    if args.auto_vtd:
        try:
            scp_host = args.scp_host or load_vtd_host(args.config.resolve())
        except (OSError, ValueError, RuntimeError) as error:
            print(f"[run_scenario] {error}", file=sys.stderr)
            return 2
        print(f"VTD SCP  : {scp_host}:{args.scp_port} (automatic Load/Apply/Play/Stop)")
        vtd_scenario_file = (
            args.vtd_scenario_file or scenario_file
        )
        delivery = "inline XML" if args.inline_scenario else "VTD file path"
        print(f"VTD file : {vtd_scenario_file} ({delivery})")
    else:
        scp_host = ""
        vtd_scenario_file = scenario_file
        print("VTD      : Load/Apply/Play manually (use --auto-vtd to automate).")

    if args.dry_run:
        print("[run_scenario] dry-run; HL_VTD was not started.")
        return 0
    if not binary.is_file():
        print(
            f"[run_scenario] HL_VTD executable not found: {binary}",
            file=sys.stderr,
        )
        return 2

    try:
        if args.auto_vtd:
            assert ready_file is not None
            ready_file.unlink(missing_ok=True)
            return run_automated(
                args, command, vtd_scenario_file, scp_host,
                ready_file, scenario_requires_traffic(scenario_file),
            )
        completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        print("\n[run_scenario] interrupted", file=sys.stderr)
        return 130
    except ScpError as error:
        print(f"[run_scenario] automation failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
