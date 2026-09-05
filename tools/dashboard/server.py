#!/usr/bin/env python3
"""HL_VTD 대회 검증 평가 대시보드.

output/verification/<scenario_id>/<run_id>/run.csv 를 읽어 2026 HL FMA 공식
15개 항목 기준으로 자동 채점하고, 경로/이동 궤적을 재생하는 로컬 웹 대시보드.
run_scenario.py 로 시나리오를 돌리는 동안 run.csv 가 계속 갱신되므로, 이 화면을
VTD 메인 화면 옆에 띄워두고 계속 새로고침(폴링)하며 실시간으로 채점 결과를
확인할 수 있다.

standard library + PyYAML 만 사용한다 (repo 에 pip/Flask 가 설치되어 있지 않음).
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
STATIC_DIR = Path(__file__).resolve().parent / "static"
VERIFICATION_DIR = REPO_ROOT / "output" / "verification"
CATALOG_PATH = REPO_ROOT / "scenario" / "scenario_catalog.csv"

sys.path.insert(0, str(REPO_ROOT / "tools" / "verification"))
import evaluate  # noqa: E402  (경로 설정 이후 import)

OFFICIAL_ITEM_LABELS = {
    1: "제한속도 준수",
    2: "보호구역 속도 준수",
    3: "차로 유지",
    4: "중앙선 침범(우측통행)",
    5: "보도 침범",
    6: "실선 구간 차로변경",
    7: "적색 신호 정지",
    8: "녹색 신호 통과",
    9: "적색 점멸 일시정지",
    10: "보행자 대응",
    11: "장애물 대응",
    12: "횡단보도 정차",
    13: "차선변경 방향지시등",
    14: "도로사용자 충돌",
    15: "리스폰(재출현)",
}


def load_catalog() -> list[dict[str, str]]:
    with CATALOG_PATH.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def catalog_by_id() -> dict[str, dict[str, str]]:
    return {row["scenario_id"]: row for row in load_catalog()}


def list_runs(scenario_id: str) -> list[dict]:
    base = VERIFICATION_DIR / scenario_id
    if not base.is_dir():
        return []
    runs = []
    now = time.time()
    for entry in sorted(base.iterdir()):
        run_csv = entry / "run.csv"
        if not entry.is_dir() or not run_csv.is_file():
            continue
        stat = run_csv.stat()
        runs.append({
            "run_id": entry.name,
            "mtime": stat.st_mtime,
            "size": stat.st_size,
            "live": (now - stat.st_mtime) < 5.0,
        })
    runs.sort(key=lambda r: r["mtime"], reverse=True)
    return runs


def read_run_rows(scenario_id: str, run_id: str) -> list[dict[str, str]]:
    path = VERIFICATION_DIR / scenario_id / run_id / "run.csv"
    return evaluate.load_rows(path)


def section_columns(rows: list[dict[str, str]]) -> list[str]:
    seen = {evaluate.section_key(row) for row in rows}
    return sorted(seen)


def official_grid(rows: list[dict[str, str]], speed_limit_kmh, school_zone_limit_kmh) -> list[dict]:
    results = evaluate.evaluate_official_for(rows, speed_limit_kmh, school_zone_limit_kmh)
    grid = []
    for item in results:
        grid.append({
            "number": item.number,
            "label": OFFICIAL_ITEM_LABELS[item.number],
            "status": item.status,
            "penalty": item.penalty,
            "detail": item.detail,
            "by_section": item.severity_by_section,
        })
    return grid


def latest_row(rows: list[dict[str, str]]) -> dict[str, str]:
    return rows[-1] if rows else {}


def to_float(row: dict[str, str], field: str):
    return evaluate.parse_float(row, field)


def build_run_payload(scenario_id: str, run_id: str, since_frame: int, speed_limit_kmh, school_zone_limit_kmh) -> dict:
    rows = read_run_rows(scenario_id, run_id)
    last = latest_row(rows)
    sections = section_columns(rows)
    grid = official_grid(rows, speed_limit_kmh, school_zone_limit_kmh)

    args = argparse.Namespace(
        official=True, require_goal=False, min_object_distance=None,
        max_abs_steering=None, max_abs_accel=None,
        speed_limit_kmh=speed_limit_kmh, school_zone_limit_kmh=school_zone_limit_kmh,
    )
    summary = evaluate.summarize(rows, args)

    track = [
        {
            "frame": int(to_float(row, "frame") or 0),
            "t": to_float(row, "elapsed_wall_time_s"),
            "x": to_float(row, "ego_x"),
            "y": to_float(row, "ego_y"),
            "heading": to_float(row, "ego_heading"),
            "speed": to_float(row, "ego_speed"),
            "obj_x": to_float(row, "nearest_object_x"),
            "obj_y": to_float(row, "nearest_object_y"),
            "obj_dist": to_float(row, "nearest_object_distance"),
            "obj_long": to_float(row, "nearest_object_longitudinal_m"),
            "ttc": to_float(row, "nearest_object_ttc_s"),
            "collision": row.get("collision", "").strip() in {"1", "true", "True"},
            "signal": row.get("traffic_light_state", "").strip(),
            "section": evaluate.section_key(row),
        }
        for row in rows
        if int(to_float(row, "frame") or 0) > since_frame
    ]

    return {
        "scenario_id": scenario_id,
        "run_id": run_id,
        "frame_count": len(rows),
        "sections": sections,
        "official_grid": grid,
        "summary": summary,
        "latest": {
            "frame": int(to_float(last, "frame") or 0) if last else None,
            "elapsed_wall_time_s": to_float(last, "elapsed_wall_time_s"),
            "ego_speed_kmh": (to_float(last, "ego_speed") or 0) * 3.6,
            "evaluation_section_id": evaluate.section_key(last) if last else None,
            "nearest_object_distance": to_float(last, "nearest_object_distance"),
            "nearest_object_longitudinal_m": to_float(last, "nearest_object_longitudinal_m"),
            "nearest_object_ttc_s": to_float(last, "nearest_object_ttc_s"),
            "collision": last.get("collision", "").strip() in {"1", "true", "True"} if last else False,
            "traffic_light_state": last.get("traffic_light_state", "").strip() if last else "",
        },
        "track": track,
    }


def build_map_payload(scenario_id: str) -> dict:
    catalog = catalog_by_id()
    row = catalog.get(scenario_id)
    if row is None:
        return {"checkpoints": [], "route": [], "meta": None}
    scenario_dir = REPO_ROOT / "scenario" / row["package_path"]

    checkpoints = []
    checkpoints_path = scenario_dir / "checkpoints.csv"
    if checkpoints_path.is_file():
        with checkpoints_path.open(newline="", encoding="utf-8-sig") as stream:
            for r in csv.DictReader(stream):
                checkpoints.append({"seq": r.get("seq"), "x": float(r["x"]), "y": float(r["y"])})

    route = []
    route_path = scenario_dir / "route.csv"
    if route_path.is_file():
        with route_path.open(newline="", encoding="utf-8-sig") as stream:
            for r in csv.DictReader(stream):
                route.append({"x": float(r["x"]), "y": float(r["y"])})

    meta = None
    yaml_path = scenario_dir / "scenario.yaml"
    if yaml_path.is_file():
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        scenario_meta = data.get("scenario", {})
        meta = {
            "name": scenario_meta.get("name"),
            "description": scenario_meta.get("description"),
            "map": scenario_meta.get("map"),
        }

    return {"checkpoints": checkpoints, "route": route, "meta": meta,
            "category": row.get("category"), "notes": row.get("notes")}


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
                catalog = load_catalog()
                for row in catalog:
                    row["run_count"] = len(list_runs(row["scenario_id"]))
                self._send_json(catalog)
            elif parsed.path == "/api/runs":
                scenario_id = query.get("scenario_id", [""])[0]
                self._send_json(list_runs(scenario_id))
            elif parsed.path == "/api/map":
                scenario_id = query.get("scenario_id", [""])[0]
                self._send_json(build_map_payload(scenario_id))
            elif parsed.path == "/api/run":
                scenario_id = query.get("scenario_id", [""])[0]
                run_id = query.get("run_id", [""])[0]
                since_frame = int(query.get("since", ["-1"])[0])
                speed_limit = query.get("speed_limit_kmh", [None])[0]
                school_zone_limit = query.get("school_zone_limit_kmh", [None])[0]
                payload = build_run_payload(
                    scenario_id, run_id, since_frame,
                    float(speed_limit) if speed_limit else None,
                    float(school_zone_limit) if school_zone_limit else None,
                )
                self._send_json(payload)
            else:
                self._send_json({"error": "not found"}, status=404)
        except FileNotFoundError as error:
            self._send_json({"error": str(error)}, status=404)
        except ValueError as error:
            self._send_json({"error": str(error)}, status=400)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[dashboard] http://{args.host}:{args.port} 에서 서비스 중 (Ctrl+C 로 종료)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
