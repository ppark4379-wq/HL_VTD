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
→ HL_VTD 실행 및 TCP 연결 대기
→ VTD Play
→ CSV 로깅
→ 제어기 종료 후 VTD Stop
```

제어기 PC와 VTD PC가 분리되어 있으면 `config/network.yaml`의
`vtd.host_ip`를 VTD PC 주소로 설정합니다. SCP는 같은 주소의 TCP 48179번을
사용하므로 VTD PC 방화벽에서도 이 포트를 허용해야 합니다. Scenario XML은
SCP 메시지에 포함해 전송하므로 두 PC가 Scenario 폴더를 공유할 필요는
없지만, XML이 참조하는 VTD Project·Distro 자산은 VTD PC에 설치되어 있어야
합니다.

별도 네트워크 설정 파일이나 SCP 주소를 지정할 수도 있습니다.

```bash
python3 tools/verification/run_scenario.py \
  scenario/vehicle/V04_cut_in \
  --config config/network.lab.yaml \
  --scp-host 165.246.170.76 \
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

현재 `evaluate.py`는 제한속도·보호구역 속도·중앙선/보도 지속 침범·실선 변경·횡단보도 정차·충돌·리스폰을 해당 로그 필드가 있을 때 구간별로 평가합니다. 입력 데이터가 없는 공식 항목은 `UNAVAILABLE`로 표시하며 임의로 PASS 처리하지 않습니다. 일반 기준과 공식 기준을 모두 지정하지 않으면 기존 `result`는 `NOT_EVALUATED`가 정상입니다. 자세한 내용은 [docs/eval_criteria.md](docs/eval_criteria.md)에 정리합니다.

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
