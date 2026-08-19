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
## Directory Structure

repo/
├── README.md
│   # 프로젝트 소개 / 실행 방법 / Git 협업 규칙
│
├── CMakeLists.txt
│   # C++ 프로젝트 빌드 설정
│
├── docs/
│   ├── interface_api.md
│   │   # VTD → Controller / Controller → VTD 인터페이스 정리
│   │
│   ├── eval_criteria.md
│   │   # 대회 평가항목 / 감점 조건 / 대응 방향 정리
│   │
│   └── network_setup.md
│       # VTD IP / TCP·UDP Port / 연동 절차 정리
│
├── config/
│   ├── network.yaml
│   │   # Host IP / TCP·UDP Port 등 통신 설정
│   │
│   ├── vehicle.yaml
│   │   # 차량 제원 / 최대 조향각 / 가감속 제한값
│   │
│   └── control.yaml
│       # PID Gain / Lookahead / TTC 기준값 등
│       # Planning·Control 튜닝 파라미터 관리
│
├── src/
│   │
│   ├── network/
│   │   ├── vtd_client.hpp
│   │   ├── vtd_client.cpp
│   │   │   # VTD 연결 / 상태 수신 / 제어 명령 송신
│   │   │
│   │   ├── packet_parser.hpp
│   │   └── packet_parser.cpp
│   │       # 수신 패킷을 Ego / Object / TrafficLight 데이터로 변환
│   │
│   ├── data/
│   │   ├── types.hpp
│   │   │   # EgoState / ObjectState / TrafficLightState
│   │   │   # ControlCommand 등 공통 데이터 구조 정의
│   │   │
│   │   ├── state_manager.hpp
│   │   ├── state_manager.cpp
│   │   │   # 현재 Ego / 주변 객체 / 신호등 상태 관리
│   │   │
│   │   ├── map_loader.hpp
│   │   ├── map_loader.cpp
│   │   │   # OpenDRIVE(.xodr) 도로 데이터 로드
│   │   │
│   │   ├── lane_data.hpp
│   │   └── lane_data.cpp
│   │       # 현재 차선 / 차선 중심선 / 실선·점선
│   │       # 제한속도 등 도로정보 관리
│   │
│   ├── planning/
│   │   ├── risk_assessment.hpp
│   │   ├── risk_assessment.cpp
│   │   │   # 차량·보행자 거리 / 상대속도 / TTC / 충돌위험 계산
│   │   │
│   │   ├── traffic_rule.hpp
│   │   ├── traffic_rule.cpp
│   │   │   # 신호등 / 제한속도 / 스쿨존 / 차선 규칙 판단
│   │   │
│   │   ├── event_handler.hpp
│   │   ├── event_handler.cpp
│   │   │   # 보행자 / 앞차 급정거 / 장애물 등 이벤트 대응 판단
│   │   │
│   │   ├── path_planner.hpp
│   │   └── path_planner.cpp
│   │       # 주행 경로 / 목표 차선 / 목표 속도 생성
│   │
│   ├── control/
│   │   ├── lateral_controller.hpp
│   │   ├── lateral_controller.cpp
│   │   │   # 횡방향 제어 / Steering 계산
│   │   │
│   │   ├── longitudinal_controller.hpp
│   │   ├── longitudinal_controller.cpp
│   │   │   # 종방향 제어 / 속도 유지 / 감속 / 정지
│   │   │
│   │   ├── controller.hpp
│   │   └── controller.cpp
│   │       # Planning 결과를 받아 최종 Steering / Accel / TurnSignal 생성
│   │
│   ├── logger/
│   │   ├── logger.hpp
│   │   └── logger.cpp
│   │       # Ego / Object / Planning / Control 데이터를 로그로 저장
│   │
│   └── main.cpp
│       # 전체 프로그램 실행 및 모듈 연결
│
├── scenario/
│   ├── basic/
│   │   # 차선 유지 / 제한속도 / 신호등 등 기본 시나리오
│   │
│   ├── pedestrian/
│   │   # 보행자 출현 / 횡단 등 보행자 시나리오
│   │
│   ├── vehicle/
│   │   # 선행차 / 차량 등장 / 급정거 등 차량 시나리오
│   │
│   ├── obstacle/
│   │   # 정적 장애물 / 도로 장애물 시나리오
│   │
│   └── mixed/
│       # 신호 + 보행자 / 실선 + 장애물 등 복합 시나리오
│
├── tools/
│   ├── plot_log.py
│   │   # Logger 데이터를 그래프로 시각화
│   │
│   └── evaluate.py
│       # 시나리오별 주행 결과 및 평가 결과 정리
│
└── scripts/
    ├── build.sh
    │   # 프로젝트 빌드
    │
    └── run.sh
        # 프로그램 실행
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
```
