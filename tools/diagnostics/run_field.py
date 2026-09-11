#!/usr/bin/env python3
"""Run only the controller; automatically capture and pack repair evidence."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from capture import prepare_capture, save_json
from contracts import CaptureRequest, PackRequest
from pack import pack_run


REPO_ROOT = Path(__file__).resolve().parents[2]


def echo_new_console(stream) -> None:
    content = stream.read()
    if content:
        print(content, end="", flush=True)


def stop_requested_process(process: subprocess.Popen) -> int:
    if process.poll() is not None:
        return process.returncode
    try:
        if os.name == "nt":
            process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        if os.name == "nt":
            process.terminate()
        else:
            os.killpg(process.pid, signal.SIGTERM)
    try:
        return process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        if os.name == "nt":
            process.kill()
        else:
            os.killpg(process.pid, signal.SIGKILL)
        return process.wait()


def run_controller(request: CaptureRequest, manifest: dict) -> int:
    console = request.directory / "console.log"
    with console.open("wb") as output:
        process = subprocess.Popen(
            manifest["command"], cwd=request.root, stdout=output, stderr=subprocess.STDOUT,
            start_new_session=os.name != "nt",
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
        )
        try:
            manifest.update(status="RUNNING", controller_pid=process.pid)
            save_json(request.directory / "capture_manifest.json", manifest)
            with console.open(encoding="utf-8", errors="replace") as reader:
                while process.poll() is None:
                    echo_new_console(reader)
                    time.sleep(0.05)
                echo_new_console(reader)
            return process.returncode
        except KeyboardInterrupt:
            print("\n[field] operator interrupt; stopping controller and packing evidence")
            stop_requested_process(process)
            return 130
        except BaseException:
            stop_requested_process(process)
            raise


def run_capture(request: CaptureRequest) -> int:
    manifest = prepare_capture(request)
    lock = request.directory / ".capture_running"
    lock.write_text(str(os.getpid()), encoding="ascii")
    code = 1
    try:
        print(f"[field] Attempt: {request.directory}", flush=True)
        print("[field] Controller only. Brief/ZIP will be generated after it exits.", flush=True)
        code = run_controller(request, manifest)
        manifest.update(status="INTERRUPTED" if code == 130 else "EXITED", exit_code=code)
    except OSError as error:
        manifest.update(status="LAUNCH_FAILED", error=str(error), exit_code=1)
        print(f"[field] launch failed: {error}")
    finally:
        manifest["ended_at"] = datetime.now(timezone.utc).isoformat()
        save_json(request.directory / "capture_manifest.json", manifest)
        lock.unlink(missing_ok=True)
        pack_run(PackRequest(request.directory))
    return code


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new attempt directory; never reused")
    parser.add_argument("--binary", type=Path, default=Path("build/hl_vtd"))
    parser.add_argument("--legacy-decision-log", action="store_true",
                        help="also enable legacy synchronous decision.log/timing.csv (extra loop cost)")
    parser.add_argument("controller_args", nargs=argparse.REMAINDER, help="-- followed by normal controller arguments")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    arguments = args.controller_args
    if arguments[:1] == ["--"]:
        arguments = arguments[1:]
    request = CaptureRequest(REPO_ROOT, args.output.resolve(), args.binary.resolve(), arguments,
                             args.legacy_decision_log)
    try:
        return run_capture(request)
    except (OSError, ValueError, ImportError) as error:
        print(f"[field] failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
