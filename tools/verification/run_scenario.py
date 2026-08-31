#!/usr/bin/env python3
# Validate one Phase 1 Scenario Package and start HL_VTD with its default route.

from __future__ import annotations

import argparse
import re
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from validate_scenario import REPO_ROOT, load_manifest, validate_scenario
from vtd_scp import ScpError, VtdScpClient


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
    return args


def build_command(
    binary: Path,
    network_config: Path,
    route_file: Path,
    scenario_id: str,
    run_id: str,
    log_file: Path,
    ready_file: Path | None = None,
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
    return command


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
) -> int:
    controller: subprocess.Popen[bytes] | None = None
    with VtdScpClient(scp_host, args.scp_port, args.vtd_timeout) as scp:
        try:
            print(f"[auto-vtd] connected to SCP {scp_host}:{args.scp_port}")
            print(f"[auto-vtd] loading and applying {scenario_file.name}")
            scp.configure_scenario(
                scenario_file, args.vtd_timeout, inline=args.inline_scenario
            )
            print("[auto-vtd] VTD is READY; initializing scenario")
            try:
                scp.initialize(min(15.0, args.vtd_timeout))
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
            scp.show_camera_owner("birdCam")
            print("[auto-vtd] birdCam configured to show Ego")
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
            print("[auto-vtd] HL_VTD is connected; sending Play")
            scp.start(args.vtd_timeout)
            print("[auto-vtd] VTD is RUNNING; CSV logging is active")
            try:
                return controller.wait(timeout=args.run_timeout)
            except subprocess.TimeoutExpired as error:
                raise ScpError(
                    f"scenario exceeded --run-timeout {args.run_timeout:g}s"
                ) from error
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
    ready_file = log_file.parent / "controller.ready" if args.auto_vtd else None
    command = build_command(
        binary,
        args.config.resolve(),
        report.route_file,
        scenario_id,
        run_id,
        log_file,
        ready_file,
    )

    print()
    print(f"Scenario : {scenario_id}")
    print(f"File     : {report.scenario_file}")
    print(f"Route    : {report.route_file}")
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
            args.vtd_scenario_file or report.scenario_file.resolve()
        )
        delivery = "inline XML" if args.inline_scenario else "VTD file path"
        print(f"VTD file : {vtd_scenario_file} ({delivery})")
    else:
        scp_host = ""
        vtd_scenario_file = report.scenario_file.resolve()
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

    log_file.parent.mkdir(parents=True, exist_ok=True)
    try:
        if args.auto_vtd:
            assert ready_file is not None
            ready_file.unlink(missing_ok=True)
            return run_automated(
                args, command, vtd_scenario_file, scp_host,
                ready_file,
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
