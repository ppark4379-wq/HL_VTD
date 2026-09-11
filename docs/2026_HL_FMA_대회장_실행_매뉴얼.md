# 2026 HL FMA 대회장 실행 매뉴얼

**Ubuntu/Linux의 Bash 터미널용. 모든 명령은 실제 HL_VTD 저장소에서, 같은 터미널에 순서대로 입력한다.** Windows PowerShell용이 아니다. 현재 Windows 실행 파일은 `--checkpoints` 모드를 지원하지 않는다.

> **사용 전 확인:** 아래는 현재 코드에 맞춘 실행 절차이며, 현장 완주 보증은 아니다. 현재 코드에는 공식 포트 외 별도 RDB 연결 시도와 사용되지 않는 UDP 수신 설정이 남아 있다. 또한 체크포인트 모드의 완료 판정이 공식 후륜축 종료선 통과와 일치하는지 확인되지 않았다. 대회 전 실제 PC로 공식 통신·출발·종료선 통과까지 검증해야 한다. 이 문제는 IP 설정만 바꿔서 해결되지 않는다.

## 0. 랜 연결 전에 준비 — 대회 전에 한 번

파일 관리자에서 **실제 HL_VTD 폴더 → 터미널에서 열기**. 다음 출력에 `src`, `config`, `scripts`가 있어야 한다.

```bash
pwd
ls -d src config scripts
```

의존성 설치·빌드·지도 사전 계산. **오류가 나면 현장에 가기 전에 해결한다.** 행사 중에는 설치·git pull·전체 지도 사전 계산을 하지 않는다.

```bash
sudo apt update &&
sudo apt install -y build-essential cmake libboost-dev python3-numpy python3-scipy python3-yaml nano xdg-utils &&
bash scripts/build.sh &&
python3 tools/map_precompute/precompute_lane_network.py --no-png &&
./build/check_precomputed_network output/precomputed_lane_network &&
ctest --test-dir build --output-on-failure &&
echo "PREBUILD PASS"
```

아래 파일을 연다.

```bash
nano config/network.competition.yaml
```

내용을 아래와 같게 만든다. 저장: **Ctrl+O → Enter → Ctrl+X**.

```yaml
vtd:
  host_ip: "192.168.50.11"
  tcp_port: 9910
  rtsp_port: 8554
controller:
  ip: "192.168.50.10"
  udp_port: 9912
```

**Ubuntu 설정 → 네트워크 → 유선 → IPv4 → 수동**에서 다음을 설정한다. YAML 수정만으로 PC IP가 바뀌지 않는다.

| 항목 | 값 |
|---|---|
| 제어기 PC 유선 IPv4 | 192.168.50.10 |
| 서브넷 마스크 | 255.255.255.0 |
| 게이트웨이·DNS | 직결망에서는 비워 둠 |
| 운영측 VTD PC | 192.168.50.11 / 같은 마스크 |
| 공식 통신 | TCP 9910 양방향 / RTSP 8554 제어기→VTD / UDP 9912 VTD→제어기 |

제어기 PC 방화벽 확인. 활성 상태라면 공식 UDP 수신을 허용한다. 단, 포트를 열어도 코드에 없는 수신 기능이 생기지는 않는다.

```bash
sudo ufw status
```

활성일 때만:

```bash
sudo ufw allow from 192.168.50.11 to any port 9912 proto udp
```

### 준비·검사·실행 명령 등록

**아래 블록 전체를 한 번 붙여 넣는다.** 터미널을 닫으면 다시 등록해야 한다. 이 함수들은 명령을 묶을 뿐, 제어기 소스를 수정하지 않는다.

```bash
comp_controller() {
  ./build/hl_vtd \
    --config config/network.competition.yaml \
    --control-config config/control.yaml \
    --map-config config/maps/living_lab.yaml \
    --vehicle-config config/vehicle.yaml \
    --path "$COMP_DIR/route.csv" \
    --checkpoints "$COMP_DIR/checkpoints.csv" "$@"
}

comp_check() (
  set -e -o pipefail
  comp_controller --mission-output "$COMP_DIR/mission.svg" --check-config \
    2>&1 | tee "$COMP_DIR/config_check.log"
  test -s "$COMP_DIR/mission.svg"
  test -s "$COMP_DIR/mission.csv"
  sha256sum build/hl_vtd config/network.competition.yaml config/control.yaml \
    config/maps/living_lab.yaml config/maps/living_lab.generated.yaml \
    config/maps/living_lab_school_zone_points.yaml config/vehicle.yaml \
    "$COMP_DIR/route.csv" "$COMP_DIR/checkpoints.csv" \
    > "$COMP_DIR/checked.sha256"
  echo "CHECK PASS"
)

comp_prepare() (
  set -e -o pipefail
  python3 - "$COMP_DIR/checkpoints.csv" <<'PY'
import csv, math, sys
with open(sys.argv[1], encoding="utf-8-sig", newline="") as f:
    reader = csv.DictReader(f)
    assert {"seq", "x", "y"} <= set(reader.fieldnames or []), "필수 헤더: seq,x,y"
    rows = list(reader)
assert len(rows) >= 2, "체크포인트가 2개 미만"
assert [int(r["seq"]) for r in rows] == list(range(1, len(rows)+1)), "seq는 파일 순서대로 1,2,3..."
assert all(math.isfinite(float(r[k])) for r in rows for k in ("x", "y")), "좌표 오류"
assert all(not r.get("heading", "").strip() or math.isfinite(float(r["heading"])) for r in rows), "heading 오류"
print("CSV PASS:", len(rows), "points")
PY
  ./build/checkpoints_to_route \
    src/data/map/HL_FMA_VTD_LivingLab.xodr "$COMP_DIR/checkpoints.csv" \
    "$COMP_DIR/route.csv" "$COMP_DIR/lane_map.csv" \
    2>&1 | tee "$COMP_DIR/route_build.log"
  test -s "$COMP_DIR/route.csv"
  test -s "$COMP_DIR/lane_map.csv"
  comp_check
  python3 tools/plot_reference_path.py "$COMP_DIR/route.csv" \
    --waypoints "$COMP_DIR/checkpoints.csv" --map "$COMP_DIR/lane_map.csv" \
    --output "$COMP_DIR/route.svg"
  echo "PREPARE PASS"
)

comp_run() (
  set -e -o pipefail
  [[ "$1" =~ ^[0-9][0-9]$ ]]
  sha256sum --check --status "$COMP_DIR/checked.sha256"
  attempt="$COMP_DIR/run_$1"
  python3 tools/diagnostics/run_field.py \
    --output "$attempt" --binary ./build/hl_vtd -- \
    --config config/network.competition.yaml \
    --control-config config/control.yaml \
    --map-config config/maps/living_lab.yaml \
    --vehicle-config config/vehicle.yaml \
    --path "$COMP_DIR/route.csv" \
    --checkpoints "$COMP_DIR/checkpoints.csv" \
    --scenario-id COMPETITION --run-id "run_$1"
)
```

대회 전에 **샘플 CSV를 별도 폴더에 복사하여 아래 2~4단계를 실제로 리허설**한다. 경로 생성·검사·실행 시작까지 걸리는 시간도 측정한다. 실행 때 미션을 다시 생성하므로 검사 직후라도 연결까지 추가 시간이 든다.

## 1. 랜선 연결 → 통신 확인

**랜선 연결 시 15분 시작.** 운영측에 VTD IP가 `192.168.50.11`인지 확인한다.

```bash
ip -4 addr
ip route get 192.168.50.11
ping -c 4 -W 1 192.168.50.11
timeout 3 bash -c 'exec 3<>/dev/tcp/192.168.50.11/9910' && echo "TCP 9910 OPEN"
```

- 경로 출력: **유선 인터페이스 + src 192.168.50.10**이어야 한다.
- TCP 검사는 **제어기 실행 전에 한 번만** 한다. 시험 접속으로 운영측 서버가 연결을 소모했다면 재연결 대기를 요청한다.
- TCP 실패: IP·랜선·VTD 서비스·운영측 방화벽을 확인한다. 제어 로직부터 고치지 않는다.
- ping은 차단될 수 있다. TCP 성공도 데이터 형식·주행 성공을 보장하지 않는다.

## 2. USB 체크포인트 CSV 복사

USB에서 받은 **정확한 CSV 파일의 전체 경로**를 확인한다. 아래 질문에는 따옴표 없이 경로만 입력한다. 공백·한글은 가능하다.

```bash
mkdir -p output
COMP_DIR=$(mktemp -d "$PWD/output/competition_XXXXXX")
read -r -p "USB 체크포인트 CSV 전체 경로: " COMP_USB
test -f "$COMP_USB" &&
cp -- "$COMP_USB" "$COMP_DIR/checkpoints.csv" &&
echo "COPY PASS: $COMP_DIR"
```

**COPY PASS가 없으면 진행하지 않는다.** 원본 CSV를 보존하고 열 이름·좌표를 임의로 바꾸지 않는다. 현재 입력 형식은 UTF-8 CSV, `seq,x,y` 헤더와 1부터 순서대로 증가하는 seq다. 다르면 운영측에 좌표계·열 의미를 확인한 후 별도 변환이 필요하다.

## 3. 경로·콜리더·게이트웨이 생성 및 확인

```bash
comp_prepare
```

**마지막에 PREPARE PASS가 있어야 진행한다.** 실패하면 아래 로그에서 원인을 확인하고 수정 후 같은 명령을 다시 실행한다.

```bash
tail -n 25 "$COMP_DIR/route_build.log" "$COMP_DIR/config_check.log"
```

성공하면 경로 그림 두 개를 연다.

```bash
xdg-open "$COMP_DIR/route.svg"
xdg-open "$COMP_DIR/mission.svg"
```

확인: **출발 위치·방향 / 체크포인트 순서 / 교차로 진입·진출 방향 / 최종 도로가 운영측 경로와 맞는가?** 체크포인트는 도로 힌트이므로 다른 차로의 점을 정확히 밟으려고 변경하지 않는다.

`route.svg`는 기준경로, `mission.svg`는 실제 체크포인트 모드의 콜리더·게이트웨이 확인용이다. **CHECK PASS는 정적 생성·설정 검사이지, 실제 Ego 출발 정합·완주·공식 종료선 검증이 아니다.** 불일치하면 실행하지 말고 CSV·지도·출발 위치를 확인한다.

## 4. 운영측 시뮬레이션 시작 → 제어기 실행

1. 운영측에 **시나리오 초기화, Ego 출발 위치·방향, 외부 제어 연결 준비**를 확인한다.
2. 운영측이 **시뮬레이션을 시작하고 실행을 지시하면**, 아래 명령을 한 번 실행한다.
3. 주행이 시작되면 PC를 조작하지 않는다. 시나리오 자동화 명령은 사용하지 않는다.

```bash
comp_run 01
```

이 명령은 미션 생성 → TCP 연결 → 데이터 수신 → 제어 명령 송신을 수행한다. 별도 송신 명령은 없다. `connected.flag`는 TCP 연결 표시일 뿐, 데이터 수신·차량 이동 증거는 아니다.

**현재 창은 실행 상태로 둔다. 주행 중 추가 명령·수정·Ctrl+C 금지.** 운영측이 시험 종료 및 PC 조작을 허용한 뒤에도 프로그램이 남아 있을 때만 Ctrl+C로 정리한다. 프로그램 종료나 `goal_reached`만으로 공식 완주라고 판단하지 않는다.

## 5. 안 움직임·실패 → 수정 → 재시도

**먼저 운영측에 해당 시험 종료·시뮬레이션 정지·PC 조작 허가를 받는다.** 그다음 실행 중인 제어기가 있으면 Ctrl+C. 남은 시간 안에 수정·빌드·검사·초기화·주행이 가능한 경우에만 재시도한다.

### 5-1. 먼저 로그 확인

첫 시도 기준:

```bash
cat "$COMP_DIR/run_01/GPT_BRIEF.md"
tail -n 40 "$COMP_DIR/run_01/console.log"
tail -n 20 "$COMP_DIR/run_01/repair_decision.log"
```

제어기 종료 후 요약과 ZIP이 자동 생성된다. **GPT에는 `$COMP_DIR/run_01/gpt_bundle.zip`을 첨부하고 증상·남은 시간을 알려준다.** ZIP에는 별도 수정용 로그, 실행 당시 소스·설정·경로가 들어 있다. 터미널 주행 출력·기존 CSV는 유지한다. GPT 로그는 비동기 저장하며 중복 `decision.log`·`timing.csv`는 기본으로 생성하지 않는다. 주행 중에는 요약 생성이나 수정 작업을 하지 않는다.

종료된 시도의 ZIP만 다시 만들 때:

```bash
python3 tools/diagnostics/pack_field.py "$COMP_DIR/run_01"
```

| 확인된 현상 | 먼저 할 일 |
|---|---|
| 미션 생성 오류 / CHECK 실패 | CSV 형식·지도·사전 계산 파일 확인. 경로를 바꾸었으면 `comp_prepare` 재실행 |
| TCP 오류 / 주행 로그가 없음 | IP·VTD 서비스·Play·Ego 외부 제어·데이터 송신 확인. RDB 연결 대기도 점검 |
| targetSpeed=0 | `zeroSpeedCause`, `stopReason` 확인. 신호·보행자·장애물에 의한 정상 정지인지 구분 |
| invalid_plan_or_input | 출발 위치·방향, 체크포인트·콜리더와 실제 Ego 위치 확인 |
| targetSpeed와 targetAccel이 양수인데 정지 | 운영측의 제어 수신·Ego 연결 상태부터 확인 |
| 이동하지만 조향·가감속이 잘못됨 | 아래 설정/해당 C++ 로직 수정 |

`run.csv`: 주행 값, `repair_decision.log`: 핵심 판단 근거, `diagnostics.jsonl`: 최종 상태·명령·송신 주기. `GPT_BRIEF.md`에서 평균 Hz·최장 송신 간격·누락 수를 확인한다. 로그에 근거가 없으면 원인을 확정하지 않는다. 움직이게 하려고 신호·충돌 안전 로직을 임의로 끄지 않는다.

### 5-2. 필요한 것만 수정

파라미터 수정:

```bash
nano config/control.yaml
```

C++ 수정은 원인에 해당하는 **파일 하나를 선택해** 연다. 코드를 고치는 내용 자체는 증상·로그에 따라 달라지며 아래 명령이 자동으로 고쳐 주지는 않는다.

| 수정 대상 | 여는 명령 |
|---|---|
| 목표 속도·정지·주행 판단 | `nano src/planning/planning_process.cpp` |
| 조향 제어 | `nano src/control/lateral/lateral_controller.cpp` |
| 가감속 제어 | `nano src/control/longitudinal/longitudinal_controller.cpp` |

C++를 바꿨다면 빌드. **BUILD PASS가 없으면 실행 금지.** 빌드 실패 시 남아 있는 실행 파일은 이전 버전일 수 있다.

```bash
cmake --build build --target hl_vtd --parallel 2 && echo "BUILD PASS"
```

YAML만 바꿨으면 빌드는 생략한다. 어느 경우든 다시 검사한다.

```bash
comp_check
```

**CHECK PASS 확인.** CSV·지도·경로를 바꾼 경우에는 `comp_check` 대신 `comp_prepare`와 그림 확인을 다시 한다. XODR 자체 변경은 전체 지도 사전 계산도 필요하므로 현장 즉석 변경을 피한다.

### 5-3. 재시도

운영측에 **처음 출발점으로 재초기화 → 시뮬레이션 시작**을 요청하고, 실행 지시 후:

```bash
comp_run 02
```

세 번째는 `comp_run 03`. 같은 번호를 다시 쓰면 로그 덮어쓰기를 막기 위해 중단된다. 다음 진단 때는 `run_01` 대신 해당 번호의 로그를 본다. **주행 시작 후 PC 조작 금지는 재시도에도 동일하다.**

## 추가되면 좋은 기능 — 아직 없는 기능, 현장에서 쓸 명령 아님

- 공식 통신만 사용하는 모드: 별도 RDB 접속 차단·접속 시간 제한, 필요한 공식 UDP 수신 지원.
- 출발·종료 검증: 실제 Ego 위치/방향과 미션 시작 비교, 공식 후륜축 종료선 통과와 완료 판정 일치.
- 검증된 미션 재사용: 실행마다 콜리더를 다시 만드는 시간을 줄이는 캐시.
- 수정 전후 실행의 자동 비교·검증된 이전 버전 복구. 진단 브리프·GPT용 ZIP은 위 실행 명령에 포함됨.

---
기준: 공식 안내문 IP·포트 표 + 후속 공지의 15분/USB/주행 중 조작 금지 규칙. 별도 수정용 로그를 지원하는 최신 소스로 대회 전에 다시 빌드한다.
