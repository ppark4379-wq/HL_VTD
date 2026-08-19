## Architecture

```text
VTD
 ↓
Network
 ↓
Data
 ↓
Planning
 ↓
Control
 ↓
VTD
```

Logger는 각 단계의 상태와 제어값을 기록합니다.

---

## Directory Structure

```text
repo/
├── docs/                 # 인터페이스 / 평가항목 / 연동 문서
├── config/               # 네트워크 / 차량 / 제어 파라미터
│
├── src/
│   ├── network/          # VTD 통신
│   ├── data/             # GT 및 도로 데이터 관리
│   ├── planning/         # 상황 판단 및 경로/속도 계획
│   ├── control/          # 횡/종방향 제어
│   └── logger/           # 주행 로그
│
├── scenario/             # 자체 VTD 검증 시나리오
├── tools/                # 로그 분석 및 평가 도구
└── scripts/              # 빌드 / 실행
```

---

## Development

### Build

```bash
./scripts/build.sh
```

### Run

```bash
./scripts/run.sh
```

---

## Git Rule

`main` 브랜치는 항상 실행 가능한 코드를 유지합니다.

기능 개발 시 별도 브랜치를 생성합니다.

예:

```text
feat/vtd-network
feat/planning-pedestrian
feat/lateral-control
fix/traffic-light
```
