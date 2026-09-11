"""Create an evidence brief and ZIP after the operator ends an attempt."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict
import json
from pathlib import Path
import time
import zipfile

from analyze import scan_run
from capture import save_json, sha256
from contracts import Incident, MEANINGS, PackRequest, ScanResult, SOURCE_MAP
from excerpts import write_frame_excerpt, write_decision_excerpt
from reproduction import write_reproduction_guide


PRIORITY = ["NO_FRAMES", "SEND_FAILED", "INPUT_TIMEOUT", "COMMAND_WITHOUT_MOTION",
            "INVALID_PLAN", "ZERO_TARGET_WAIT", "ROUTE_SEARCH_FAILED", "COLLISION_LOGGED",
            "SIM_TIME_FROZEN", "EMERGENCY_BRAKE", "SAFETY_OVERRIDE", "PLANNING_SLOW",
            "CLOCK_SLOW", "RECEIVE_ONLY", "TRACKING_ERROR", "STEERING_AT_LIMIT", "POSITION_JUMP"]


def select_incidents(scan: ScanResult, maximum: int) -> list[Incident]:
    selected = []
    for code in PRIORITY:
        candidates = [item for item in scan.incidents if item.code == code]
        if candidates:
            selected.append(candidates[0])
        if len(selected) == maximum:
            break
    return selected




def read_tail(path: Path, count: int = 45) -> str:
    if not path.is_file():
        return "(파일 없음)"
    with path.open(encoding="utf-8", errors="replace") as stream:
        return "".join(deque(stream, maxlen=count))


def source_locations(directory: Path, selected: list[Incident]) -> list[str]:
    locations = []
    for incident in selected:
        for source in SOURCE_MAP.get(incident.code, []):
            path = directory / "snapshot" / source
            if source not in locations and path.is_file():
                locations.append(source)
    return locations


def compact_evidence(incident: Incident) -> dict:
    row = incident.evidence
    ego, planning, control = row.get("ego") or {}, row.get("planning") or {}, row.get("control") or {}
    return {
        "diagnostics_line": row.get("_line"), "sim_s": row.get("sim_s"),
        "x": ego.get("x"), "y": ego.get("y"), "speed": ego.get("speed"),
        "road_section_lane": [ego.get(key) for key in ("road", "section", "lane")],
        "target_speed": planning.get("target_speed"), "stop_reason_mask": planning.get("stop_reason"),
        "stop_source": planning.get("stop_source_id"), "risk_object": planning.get("risk_object"),
        "safe_command": control.get("safe"), "safety_invalid_input": control.get("invalid_input"),
        "detail": row.get("detail"),
    }


def timing_brief(scan: ScanResult) -> list[str]:
    if not scan.send_interval_count or not scan.send_interval_total_ms:
        return ["- 송신 주기: 측정 표본 없음 (20Hz 확인 불가)"]
    average = scan.send_interval_total_ms / scan.send_interval_count
    return [
        f"- 관측 송신 평균: {1000.0 / average:.2f}Hz / {average:.3f}ms; 최장 간격 {scan.maximum_send_interval_ms:.3f}ms",
        f"- 55ms 초과 송신 간격: {scan.send_intervals_over_55ms}/{scan.send_interval_count} (55ms는 지터 확인용, 제어 기준 변경 아님)",
        f"- 수신 후 송신 완료까지 최대: {scan.maximum_receive_to_send_ms:.3f}ms; 송신 후 진단 복사 최대: {scan.maximum_diagnostic_capture_ms:.3f}ms",
        "- 관측 표본 통계이며 누락 시 편향될 수 있다. 느린 송신 간격만으로 로깅이 원인이라고 단정하지 않는다.",
    ]


def render_brief(request: PackRequest, scan: ScanResult, manifest: dict,
                 selected: list[Incident], metadata: dict) -> str:
    locations = source_locations(request.directory, selected)
    lines = [
        "# 현장 재시도용 GPT 진단 브리프", "",
        "이 문서는 감점·준수 평가가 아니라 제어기 수정에 필요한 주행 증거다.",
        "15분은 연결·1차 주행·수정·빌드·2차 주행을 합친 전체 시간이다. 남은 시간은 운영측 기준으로 확인한다.",
        "", f"- 실행 상태: {manifest.get('status', 'UNKNOWN')}; exit={manifest.get('exit_code')}",
        f"- 당시 소스 HEAD: {manifest.get('source_head', 'UNKNOWN')}",
        f"- 실행 바이너리 SHA256: {manifest.get('binary', {}).get('sha256', 'UNKNOWN')}",
        "- 소스와 바이너리의 동일 빌드 여부: UNVERIFIED (해시·빌드 시각·변경 파일 대조 필요)",
        f"- 프레임: {scan.frames}; TCP 송신 완료: {scan.send_completed}; 송신 생략: {scan.sends_skipped}; 실패: {scan.sends_failed}",
        f"- 마지막 단계: {scan.last_stage.get('stage', 'UNKNOWN')} / {scan.last_stage.get('detail', '')}",
        f"- 최대 planning/clock/receive 대기: {scan.maximum_planning_ms:.2f} / {scan.maximum_clock_ms:.2f} / {scan.maximum_receive_ms:.2f} ms",
        f"- 제어기 goal 보고: {scan.goal_reported}; 공식 완주 확인과는 별도",
        "- socket_send_completed는 OS 송신 완료이며 VTD 적용 확인은 아니다. 실제 적용 ACK는 수집되지 않는다.",
        *timing_brief(scan),
        f"- 비동기 저장 상태: {json.dumps(scan.logger_status, ensure_ascii=False)}",
        f"- 상세 판단 저장 상태: {json.dumps(scan.repair_status, ensure_ascii=False)}",
        "", "## 우선 확인할 증상", "",
    ]
    for item in selected:
        lines.extend([
            f"### {item.code}: {MEANINGS.get(item.code, item.code)}", "",
            f"프레임 {item.first_frame}~{item.last_frame}; process wall {item.first_wall_s:.3f}~{item.last_wall_s:.3f}s.",
            "```json", json.dumps(compact_evidence(item), ensure_ascii=False, separators=(",", ":")), "```", "",
        ])
    if not selected:
        lines.append("정의된 증상 트리거에 걸린 표본 없음. 정상 주행 보증이 아니며 마지막 프레임·console 확인 필요.")
    lines.extend(["", "## 읽을 파일 순서", "",
                  "1. 이 브리프 → focus_frames.jsonl → focus_decisions.log",
                  "2. inputs/ 설정·체크포인트·경로, snapshot/ 당시 코드, changes.patch",
                  "3. 필요한 프레임만 diagnostics.jsonl·repair_decision.log에서 추가 확인 (기존 로그가 있으면 함께 참고)",
                  "", "우선 검토할 소스 (추정 원인 후보 위치이며 버그 확정 아님):", ""])
    lines.extend(f"- snapshot/{source}" for source in locations)
    lines.extend(["", "## 해석 규칙·누락", "",
                  "- ego speed는 제어기가 계산한 속도. GT 원위치와 시간도 함께 확인한다.",
                  "- sim_s=null이면 시뮬레이션 시각 미확인. 2초 정체는 수신 wall time 기반 증상이며 시뮬레이션 pause와 구별이 필요하다.",
                  "- stop_reason 비트: 1 적색, 2 적색점멸, 4 보행자, 16 횡단보도, 32 정적 경로 막힘.",
                  "- lane_change.phase: 0 None, 1 Prepare, 2 Execute, 3 Settle; route_mode: 0 ReturnToSource, 1 KeepTargetLane.",
                  "- planning.health: 0 Ok, 1 Recovery; indicator: 0 OFF, 1 LEFT, 2 RIGHT; 객체 유형 추정 없음.",
                  "- diagnostics.planning은 runPlanningProcess 반환 후 최종 결과. 기존 decision의 frame_result는 후속 긴급제동 적용 전 값일 수 있다.",
                  "- 단위: m, m/s, m/s², rad, s; 성능 필드는 ms. 객체 OBB는 2D. async_bounded 모드 local path는 최대 17점(구버전 약65점). 샘플 사이 형상은 생략됨.",
                  "- 2초/0.2m 정체, 100ms 계산 지연, 1m 추종 오차는 디버깅 탐지값이며 공식 규칙·수정 파라미터가 아니다.",
                  "- async_bounded는 수신 Ego/신호와 최종 World 객체 최대30개를 복사한다. 중복 GT 30슬롯은 생략한다. 객체/정지 요청/목표 차로 전체 수와 보존 수가 다르면 일부가 잘린 것이다.",
                  "- 상세 이벤트는 프레임당 후보 상세32개·핵심64개까지. JSON 중복 frame_result는 생략한다. 누락·축약 기록은 정상 판정 근거로 사용할 수 없다.",
                  "- 저장 스레드는 약100ms마다 flush. 정상 종료는 큐를 비우지만 강제 종료/저장 오류는 꼬리 기록을 잃을 수 있다.",
                  "- kernel에서 버린 과거 GT는 복구할 수 없다. dropped_gt_total 및 수신 공백 확인.",
                  f"- 발췌 한도 정보: {json.dumps(metadata, ensure_ascii=False)}",
                  "- 오류 시작/끝 및 바로 전후 관측 프레임은 전체 기록을 보존한다. 핵심 기록이 많으면 발췌 한도를 초과할 수 있다.",
                  "- 발췌되지 않은 원본 로그도 ZIP에 포함된다. 전체 로그를 처음부터 읽을 필요는 없다.",
                  "- REPRODUCE.md에 포함 소스·의존성·오프라인 빌드/검사 방법 및 재현 한계를 명시한다."])
    lines.extend(f"- {issue}" for issue in [*manifest.get("notes", []), *scan.issues])
    lines.extend(["", "## GPT에게 요청할 작업", "",
                  "남은 현장 시간: 사용자가 기입. 아래 증거로 가장 먼저 발생한 문제를 추적하라.",
                  "관측 사실 / 원인 추론 / 아직 확인되지 않은 외부 조건을 구분하고 근거 frame·필드·소스 함수를 명시하라.",
                  "현재 작업 소스가 snapshot과 같은지 먼저 확인하고 최소한의 수정안을 적용하라. 정상 신호 대기나 안전 제동은 로그만 보고 삭제하지 말라.",
                  "TCP 미수신·송신 실패·VTD 미적용이면 해당 근거를 먼저 확인하라. 원인과 무관한 제어 파라미터를 여러 개 바꾸지 말라.",
                  "수정 파일·이유, 빠른 빌드/관련 테스트 명령, 다음 주행에서 바뀌어야 하는 로그 필드를 제시하라.",
                  "한 번의 패치로 해결됐다는 단정은 실제 재주행 근거가 있어야 한다.",
                  "", "## 종료 직전 console", "", "```text", read_tail(request.directory / "console.log"), "```", ""])
    return "\n".join(lines)


def archive_paths(directory: Path) -> list[Path]:
    names = ["GPT_BRIEF.md", "REPRODUCE.md", "environment.json", "diagnosis.json", "capture_manifest.json", "changes.patch", "git_status.txt",
             "focus_frames.jsonl", "focus_decisions.log", "diagnostics.jsonl", "repair_decision.log", "repair_health.json", "decision.log", "timing.csv",
             "run.csv", "console.log", "mission.svg", "mission.csv"]
    paths = [directory / name for name in names if (directory / name).is_file()]
    for folder in ("snapshot", "inputs"):
        paths.extend(path for path in (directory / folder).rglob("*") if path.is_file() and not path.is_symlink())
    return paths


def write_archive(directory: Path) -> None:
    paths = archive_paths(directory)
    checksums = {str(path.relative_to(directory)): {"sha256": sha256(path), "size": path.stat().st_size} for path in paths}
    save_json(directory / "bundle_manifest.json", {"schema": 1, "files": checksums})
    target = directory / "gpt_bundle.zip"
    temporary = directory / "gpt_bundle.zip.tmp"
    with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        for path in [*paths, directory / "bundle_manifest.json"]:
            archive.write(path, path.relative_to(directory).as_posix())
    temporary.replace(target)


def pack_run(request: PackRequest) -> Path:
    if (request.directory / ".capture_running").exists():
        raise ValueError("capture is still running; end the attempt before packing")
    started = time.monotonic()
    manifest_path = request.directory / "capture_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {"notes": ["CAPTURE_MANIFEST_MISSING"]}
    scan = scan_run(request.directory)
    selected = select_incidents(scan, request.max_windows)
    frames, frame_meta = write_frame_excerpt(request, scan, selected)
    decision_meta = write_decision_excerpt(request.directory, frames, set(frame_meta["protected_frames"]))
    metadata = {"frames": frame_meta, "decisions": {key: value for key, value in decision_meta.items() if key != "event_counts"}}
    save_json(request.directory / "diagnosis.json", {"schema": 1, "scan": asdict(scan), "excerpts": metadata, "decision_events": decision_meta["event_counts"]})
    (request.directory / "GPT_BRIEF.md").write_text(render_brief(request, scan, manifest, selected, metadata), encoding="utf-8")
    write_reproduction_guide(request.directory, manifest)
    write_archive(request.directory)
    elapsed = time.monotonic() - started
    print(f"[field] GPT brief: {request.directory / 'GPT_BRIEF.md'}")
    print(f"[field] GPT bundle: {request.directory / 'gpt_bundle.zip'} ({elapsed:.2f}s)")
    return request.directory / "gpt_bundle.zip"
