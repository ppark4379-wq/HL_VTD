"""Document offline build/inspection from frozen evidence; never run a vehicle."""

from __future__ import annotations

from pathlib import Path
import re


RUNTIME_SOURCES = (
    "tools/verification/prepare_runtime_mission.py",
    "tools/verification/whole_mission_corridors.py",
    "tools/verification/corridor_surface_bounds.py",
    "tools/map_precompute/precompute_lane_network.py",
    "tools/map_precompute/xodr_network.py",
)


def missing_build_sources(snapshot: Path) -> list[str]:
    cmake = snapshot / "CMakeLists.txt"
    if not cmake.is_file():
        return ["CMakeLists.txt"]
    # Check literal local source paths used by this repository's CMake targets.
    paths = set(re.findall(r"(?:src|tools|tests)/[\w./-]+\.(?:cpp|c|hpp|h)\b", cmake.read_text(encoding="utf-8")))
    return sorted(path for path in paths if not (snapshot / path).is_file())


def input_inventory(manifest: dict) -> list[str]:
    lines = []
    for option, record in manifest.get("inputs", {}).items():
        state = "보존됨" if record.get("exists") else "누락"
        lines.append(f"- `{option}`: {state}; ZIP 경로 `{record.get('snapshot', '?')}`; 원래 경로 `{record.get('original_path', '?')}`")
    return lines or ["- 입력 manifest 없음. 실행 당시 파일을 사후 추정하지 말 것."]


def write_reproduction_guide(directory: Path, manifest: dict) -> None:
    snapshot = directory / "snapshot"
    missing_build = missing_build_sources(snapshot)
    missing_runtime = [path for path in RUNTIME_SOURCES if not (snapshot / path).is_file()]
    lines = [
        "# GPT용 오프라인 빌드·재현 안내", "",
        "이 ZIP은 수정용 증거 묶음이다. 아래 명령은 VTD를 시작하거나 차량에 명령을 보내지 않는다.",
        "현장에서는 원래 저장소에서 최소 수정·증분 빌드·검사 후 운영측 허가에 따라 재주행한다.",
        "ZIP만 받은 조사자는 snapshot/의 당시 코드부터 확인한다. ZIP 자체를 Git 저장소로 가정하지 않는다.",
        "", "## 1. 포함 상태", "",
        f"- CMake에 명시된 로컬 소스 누락: {', '.join(missing_build) if missing_build else '없음'}",
        f"- 주요 미션/지도 Python 소스 누락: {', '.join(missing_runtime) if missing_runtime else '없음'}",
        "- src/, config/, tools/, tests/, scripts/의 소스·관련 로컬 입력 파일을 실행 전에 보존한다.",
        "- environment.json: 수집 도구의 OS/Python/NumPy/SciPy/PyYAML 버전. 컴파일러·라이브러리 전체 환경을 고정한 컨테이너는 아니다.",
        "- capture_manifest.json의 원래 실행 명령은 증거다. 원래 절대 경로와 IP를 새 환경에서 그대로 실행하지 말 것.",
        "", "## 2. ZIP만으로 빌드·로그 단위 테스트", "",
        "Ubuntu/Linux 기준. C++17 컴파일러, CMake, Boost 헤더, Python 3.10 이상 필요.",
        "미션/지도 계산에는 NumPy·SciPy, YAML 의존성 해석에는 PyYAML도 필요하다. environment.json과 버전을 대조한다.",
        "ZIP을 푼 최상위 폴더에서, 누락 소스가 없을 때만:", "",
        "```bash", "cd snapshot",
        "cmake -S . -B build -DCMAKE_BUILD_TYPE=Release &&",
        "cmake --build build --target hl_vtd field_diagnostics_tests --parallel 2 &&",
        "ctest --test-dir build -R '^field_diagnostics_tests$' --output-on-failure",
        "python3 -m unittest discover -s tools/diagnostics -p 'test_*.py' -v", "```", "",
        "이 테스트는 로그 보존·포장 검증이지 실제 주행 성공 검증이 아니다.",
        "전체 C++ 회귀 테스트는 추가 빌드 타깃 및 scenario/·output/의 외부 시험 자료가 필요할 수 있다.",
        "대시보드 등 다른 하위 시스템의 테스트까지 ZIP 단독 실행을 보장하지 않는다.",
        "", "## 3. 미션 생성·설정 오류 조사", "",
        "1. console.log와 diagnostics.jsonl의 마지막 stage를 확인한다.",
        "2. snapshot/src/planning/global/mission_precompute.cpp 및 tools/verification/prepare_runtime_mission.py와 종속 모듈을 대조한다.",
        "3. 실제 입력은 아래 목록을 사용한다. 외부 절대 경로는 재현용 복사본에서만 수정하고 원본 증거는 유지한다.",
        "4. map-config의 map.xodr/static_features/school_zone_reference_points 경로는 snapshot 작업 디렉터리에서 해석된다. inputs/의 복사본 위치와 일치하게 재현용 YAML을 준비한다.",
        "5. output/precomputed_lane_network의 대용량 CSV 전체는 포함하지 않는다. inputs/precomputed_manifest.json의 해시·파라미터와 일치하는 기존 데이터를 가져오거나, 보존된 지도와 tools/map_precompute 소스로 별도 재생성한다.",
        "6. 위 입력·사전계산 데이터가 준비된 후에만 기존 실행 인자를 재현용 경로로 바꿔 --check-config로 검사한다. 실제 제어기 실행과 혼동하지 말 것.",
        "", *input_inventory(manifest), "",
        "## 4. 증거 범위", "",
        "- focus_frames.jsonl은 선택된 증상의 시작/끝 프레임과 바로 전후 관측 프레임을 통째로 보존한다. _line은 원본 diagnostics.jsonl의 행 번호다.",
        "- focus_decisions.log는 보호된 프레임의 판단 이벤트를 모두 보존한다. 파일 한도보다 핵심 증거를 우선한다.",
        "- 원본은 diagnostics.jsonl/repair_decision.log, 무결성 목록은 bundle_manifest.json이다.",
        "- 소스와 실제 실행 바이너리의 동일 빌드 여부는 UNVERIFIED다. 새로 빌드한 결과와 당시 실행 파일을 동일하다고 단정하지 않는다.",
        "- 영상·VTD 프로젝트·엔진·명령 적용 ACK·완전한 시뮬레이션 리플레이는 포함되지 않는다.",
        "- 기존 폴더를 재포장해도 누락된 실행 전 소스는 자동 복원하지 않는다. 누락이 표시되면 동일 커밋/변경사항을 별도로 확보한다.", "",
    ]
    (directory / "REPRODUCE.md").write_text("\n".join(lines), encoding="utf-8")
