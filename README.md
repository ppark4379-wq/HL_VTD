# HL_VTD

VTD Ground Truth를 수신해 경로를 계획하고 Ego 제어 명령을 전송하는 HL FMA 자율주행 제어기 프로젝트입니다.

```text
VTD → Network → GT Data → Planning → Control → VTD
                              ↓
                            Logger
```

## 디렉터리 구조

```text
HL_VTD/
├── config/                 네트워크·차량·제어 설정
├── docs/                   인터페이스·평가기준·네트워크 문서
├── scenario/               VTD XML, manifest, route를 묶은 시나리오 패키지
│   ├── basic/
│   ├── traffic/
│   ├── vehicle/
│   ├── pedestrian/
│   ├── obstacle/
│   └── mixed/
├── scripts/                빌드·기본 실행 스크립트
├── src/
│   ├── network/            VTD 연결, GT 수신, 제어 명령 송신
│   ├── data/               GT 처리, OpenDRIVE와 경로 데이터
│   ├── planning/           참조경로 생성과 주행 계획
│   ├── control/            횡·종방향 제어
│   ├── logger/             실행 CSV 기록
│   └── main.cpp
├── tools/verification/     패키지 검증·실행·평가·그래프 도구
└── output/verification/    시나리오별 실행 결과
```

## 빌드

최초 실행 또는 C++ 코드 변경 후 저장소 루트에서 빌드합니다. XML·YAML·CSV만 수정한 경우에는 다시 빌드할 필요가 없습니다.

```bash
cd /home/autonav/바탕화면/HL_VTD
bash scripts/build.sh
```

## 시나리오 실행

1. VTD GUI에서 `00_HL_VTD` Setup을 선택합니다.
2. 실행할 패키지와 같은 XML을 선택하고 초록 체크 Apply를 누릅니다.
3. 아직 VTD Play는 누르지 않습니다.
4. 터미널에서 Runner를 실행합니다.

```bash
cd /home/autonav/바탕화면/HL_VTD

python3 tools/verification/run_scenario.py \
  scenario/vehicle/V04_cut_in \
  --run-id test_01
```

Validator가 `RESULT: PASS`를 출력하고 HL_VTD가 GT를 기다리면 VTD Play를 누릅니다. 경로 끝에 도달하면 `[drive] goal reached`와 함께 종료됩니다.

### VTD 시나리오 자동 실행

VTD 프로그램과 `00_HL_VTD` Setup을 실행해 둔 상태라면 SCP 자동화로
Scenario 선택, Configure/Apply, Play와 Stop을 한 명령에서 처리할 수 있습니다.

```bash
python3 tools/verification/run_scenario.py \
  scenario/vehicle/V04_cut_in \
  --run-id test_01 \
  --auto-vtd
```

자동 실행 순서는 다음과 같습니다.

```text
Scenario 검증 및 XML 전송
→ VTD Configure/Apply 완료 대기
→ Scenario Init 완료 대기
→ HL_VTD 실행 및 TCP 연결 대기
→ VTD Play
→ CSV 로깅
→ 제어기 종료 후 VTD Stop
```

초기화 직후 `00_HL_VTD`의 Image Generator 두 개가 팝업을 하나씩 띄우고
Ego에 자동 연결한다. `igVisualBroadcast/cam1`은 대회용 follower view
(`x=-6.5 m`, `z=2.6 m`), `igVisualStudent/birdCam`은 기존 bird view를
사용한다. VTD 2025.2의 공유 메모리 제약 때문에 두 RenderSurface를 한 IG에
넣지 않고 별도 프로세스로 실행한다.

Runner는 Play 직후 External Ego의 driver input을
`ctrlLatLong="extern"`으로 고정한다. `00_HL_VTD` Setup에서도
`Config/ModuleManager/moduleManager.xml`의 Ego를 담당하는
`DynamicsPlugin`에 다음 항목을 넣어야 재초기화와 외부 제어기
재연결 시 ghostdriver fallback이 다시 활성화되지 않는다.

```xml
<Player default="true" />
<Driver ctrlLatLong="extern" />
```

Traffic 프로세스는 계속 실행되며 Internal NPC를 제어한다. 위 설정은
Ego만 HL_VTD 외부 제어 명령을 사용하게 한다.

제어기 PC와 VTD PC가 분리되어 있으면 `config/network.yaml`의
`vtd.host_ip`를 VTD PC 주소로 설정합니다. SCP는 같은 주소의 TCP 48179번을
사용하므로 VTD PC 방화벽에서도 이 포트를 허용해야 합니다. 두 PC를 사용할
때는 Scenario XML도 VTD PC에 배치하고 `--vtd-scenario-file`에 VTD PC에서
보이는 경로를 지정해야 합니다. XML이 참조하는 VTD Project·Distro 자산도
VTD PC에 설치되어 있어야 합니다.

별도 네트워크 설정 파일이나 SCP 주소를 지정할 수도 있습니다.

```bash
python3 tools/verification/run_scenario.py \
  scenario/vehicle/V04_cut_in \
  --config config/network.lab.yaml \
  --scp-host 165.246.170.76 \
  --vtd-scenario-file /home/vtd/HL_VTD/scenario/vehicle/V04_cut_in/V04_cut_in.xml \
  --auto-vtd \
  --run-timeout 180
```

`--auto-vtd`를 생략하면 기존 수동 Load/Apply/Play 방식으로 동작합니다.

다음 시험에서는 기존 로그 덮어쓰기를 피하도록 실행 ID를 변경합니다.

```bash
python3 tools/verification/run_scenario.py \
  scenario/vehicle/V04_cut_in \
  --run-id test_02
```

## 실행 전 패키지 검사

VTD나 HL_VTD를 실행하지 않고 XML·YAML·route 정합성만 검사합니다.

```bash
python3 tools/verification/validate_scenario.py \
  scenario/vehicle/V04_cut_in
```

실행할 파일, 경로와 명령만 확인하려면 다음을 사용합니다.

```bash
python3 tools/verification/run_scenario.py \
  scenario/vehicle/V04_cut_in \
  --run-id test_01 \
  --dry-run
```

## 출력 구조

Runner는 시나리오 ID와 실행 ID를 사용해 결과를 분리합니다.

```text
output/verification/
└── V04/
    ├── test_01/
    │   ├── run.csv
    │   ├── summary.csv       evaluate.py 실행 시 생성
    │   └── run_plot.png      plot_log.py 실행 시 생성
    └── test_02/
        └── run.csv
```

실행 요약과 그래프는 다음 명령으로 생성합니다.

```bash
python3 tools/verification/evaluate.py \
  output/verification/V04/test_01/run.csv \
  --summary output/verification/V04/test_01/summary.csv

python3 tools/verification/plot_log.py \
  output/verification/V04/test_01/run.csv \
  --output output/verification/V04/test_01/run_plot.png
```

공식 기준 중 로그에서 계산 가능한 항목을 평가하려면 `--official`을 사용합니다. 제한속도가 로그에 없으면 시험 구간 제한속도를 직접 전달할 수 있습니다.

```bash
python3 tools/verification/evaluate.py \
  output/verification/B02/test_01/run.csv \
  --official \
  --speed-limit-kmh 30 \
  --summary output/verification/B02/test_01/summary.csv
```

`evaluate.py`는 차로 유지를 제외한 14개 항목의 판정 입력으로 구간별 감점을 계산합니다. 500점에서 시작하며 데이터가 없는 항목은 `UNAVAILABLE`로 남깁니다. 최신 사용자 결정·출처·항목별 기준·실행 절차는 [최종 자동채점 기준](docs/HL_VTD_자동채점_최종평가기준.md)을 우선 확인하세요.

### VTD PC의 실시간 자동채점 대시보드

VTD와 같은 Linux PC에서 실행합니다. 별도 제어기 PC의 CSV가 없어도 읽기 전용 RDB 출력을 수신합니다.

```bash
python3 tools/dashboard/check_environment.py --probe-rdb
python3 tools/dashboard/server.py --host 127.0.0.1 --port 8765 --video-source window
```

브라우저: `http://127.0.0.1:8765/`. Python/PyYAML과 영상용 FFmpeg/X11 환경이 필요합니다. 서버가 출력하는 `control_monitor.py` 명령을 VTD PC의 다른 터미널에서 실행해야 첫 제어값 수신부터 채점됩니다(0 제어값 포함, Play/움직임만으로 시작하지 않음). 웹 서버에는 sudo를 사용하지 않습니다. [assessment_rules.yaml](tools/dashboard/assessment_rules.yaml)의 경로/신호 매핑과 실제 객체 송출을 확인하세요. 종료는 마지막 체크포인트 도달(현재 구현 허용거리 1.5m)이며 종료 정지선 지정은 필요 없습니다. 보호구역 43개 적용·15/28번 보류([연결 보완 근거](docs/living_lab_school_zone_review.md)), 횡단보도 197개 추출 상태이며 실제 VTD 종단 간 검증은 별도입니다.

## 네트워크

현재 `config/network.yaml`은 연구실 연동 환경을 사용합니다. 2026 대회 공지 네트워크는 다음과 같습니다.

| 구분 | 주소·포트 |
|---|---|
| 제어기 PC | `192.168.50.10/24` |
| VTD Host | `192.168.50.11/24` |
| VTD TCP | `9910` |
| VTD RTSP | `8554` |
| Controller UDP | `9912` |

대회 당일에는 연구실 설정과 대회 설정을 별도 파일로 관리해야 합니다. HL_VTD 바이너리는 `--config` 선택을 지원하지만 현재 `run_scenario.py`에는 해당 옵션이 없으므로, Runner의 설정 선택 기능을 추가하기 전까지는 실행 전에 `config/network.yaml`의 환경을 반드시 확인해야 합니다.

자세한 내용은 [docs/network_setup.md](docs/network_setup.md), 패킷 구조는 [docs/interface_api.md](docs/interface_api.md), 시나리오 설계는 [scenario/README.md](scenario/README.md)를 참고합니다.
