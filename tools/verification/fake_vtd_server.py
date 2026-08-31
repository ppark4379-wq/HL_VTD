#!/usr/bin/env python3
"""VTD 없이 제어기 전체 경로를 검증하는 루프백 서버.

VTD 역할을 대신한다.
  1) TCP 9910에서 제어기 접속을 기다린다
  2) 1109 Byte GT 프레임을 20 Hz로 보낸다
  3) 제어기가 돌려준 9 Byte 제어 명령(steering/targetAccel/turnSignal)을 읽는다
  4) 자전거 모델로 Ego를 움직여 다음 프레임에 반영한다  <- 폐루프

실제 VTD가 아니므로 물리·객체 거동은 근사다. 확인할 수 있는 것은
"패킷 파싱 -> World -> Planning -> Control -> 송신"이 끊김 없이 도는지와
조향/가속 명령이 경로를 실제로 따라가는지다.

사용법:
  python tools/verification/fake_vtd_server.py --route scenario/basic/B01_lane_keep/route.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time
from pathlib import Path

GT_FRAME_SIZE = 1109
CONTROL_PACKET_SIZE = 9
OBJECT_COUNT = 30
WHEELBASE = 2.944


def load_route(path: Path) -> list[tuple[float, float, float]]:
    points = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            points.append((float(row["x"]), float(row["y"]), float(row["yaw"])))
    if len(points) < 2:
        raise SystemExit(f"route too short: {path}")
    return points


def build_gt_frame(ego, objects, light_id: int, light_state: int) -> bytes:
    payload = struct.pack(
        "<6f", ego["x"], ego["y"], 0.0, ego["heading"], 0.0, 0.0
    )
    for index in range(OBJECT_COUNT):
        if index < len(objects):
            obj = objects[index]
            payload += struct.pack(
                "<I8f",
                obj["id"], obj["x"], obj["y"], 0.0, obj["heading"],
                obj["speed"], obj["length"], obj["width"], obj["height"],
            )
        else:
            payload += struct.pack("<I8f", 0, 0, 0, 0, 0, 0, 0, 0, 0)
    payload += struct.pack("<iB", light_id, light_state)
    assert len(payload) == GT_FRAME_SIZE, len(payload)
    return payload


def nearest_route_error(route, x: float, y: float) -> float:
    best = float("inf")
    for px, py, _ in route:
        best = min(best, math.hypot(px - x, py - y))
    return best


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fake VTD loopback server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9910)
    parser.add_argument(
        "--route", type=Path,
        default=Path("scenario/basic/B01_lane_keep/route.csv"))
    parser.add_argument("--rate", type=float, default=20.0, help="Hz")
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--light-state", type=int, default=3,
                        help="0 unset 1 red 2 yellow 3 green 6 flashing")
    parser.add_argument("--light-id", type=int, default=11)
    parser.add_argument("--lead-vehicle", action="store_true",
                        help="place a stopped vehicle 40 m ahead on the route")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    route = load_route(args.route)

    objects = []
    if args.lead_vehicle:
        index = min(len(route) - 1, 80)  # 약 40 m 앞 (0.5 m 간격)
        px, py, pyaw = route[index]
        objects.append({"id": 101, "x": px, "y": py, "heading": pyaw,
                        "speed": 0.0, "length": 4.5, "width": 1.9,
                        "height": 1.5})

    ego = {"x": route[0][0], "y": route[0][1],
           "heading": route[0][2], "speed": 0.0}

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f"[fake-vtd] listening on {args.host}:{args.port}")
    print(f"[fake-vtd] route {args.route} ({len(route)} points)")
    print("[fake-vtd] waiting for controller ...")

    connection, peer = server.accept()
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[fake-vtd] controller connected from {peer}")

    dt = 1.0 / args.rate
    max_error = 0.0
    commands = 0
    signals = set()
    try:
        for frame in range(args.frames):
            connection.sendall(
                build_gt_frame(ego, objects, args.light_id, args.light_state))

            reply = b""
            while len(reply) < CONTROL_PACKET_SIZE:
                chunk = connection.recv(CONTROL_PACKET_SIZE - len(reply))
                if not chunk:
                    print("[fake-vtd] controller closed the connection")
                    return 1
                reply += chunk
            steering, accel = struct.unpack("<2f", reply[:8])
            turn_signal = reply[8]
            commands += 1
            signals.add(turn_signal)

            # 자전거 모델로 Ego를 전진시킨다.
            ego["speed"] = max(0.0, ego["speed"] + accel * dt)
            ego["heading"] += (ego["speed"] / WHEELBASE) * math.tan(steering) * dt
            ego["x"] += ego["speed"] * math.cos(ego["heading"]) * dt
            ego["y"] += ego["speed"] * math.sin(ego["heading"]) * dt

            error = nearest_route_error(route, ego["x"], ego["y"])
            max_error = max(max_error, error)

            if frame % 40 == 0:
                print(f"[fake-vtd] f={frame:4d} "
                      f"speed={ego['speed'] * 3.6:5.1f} km/h "
                      f"steer={steering:+.4f} accel={accel:+.2f} "
                      f"signal={turn_signal} lat_err={error:.3f} m")
            time.sleep(dt)
    except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
        # 제어기가 goal_reached로 정상 종료하면 여기로 온다.
        print("[fake-vtd] controller disconnected (goal reached or stopped)")
    finally:
        connection.close()
        server.close()

    print("\n[fake-vtd] ---- summary ----")
    print(f"[fake-vtd] control packets received : {commands}")
    print(f"[fake-vtd] final speed              : {ego['speed'] * 3.6:.1f} km/h")
    print(f"[fake-vtd] max lateral error        : {max_error:.3f} m")
    print(f"[fake-vtd] turn signal values seen  : {sorted(signals)}")
    ok = commands > 0 and max_error < 2.0
    print(f"[fake-vtd] RESULT: {'PASS' if ok else 'CHECK'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
