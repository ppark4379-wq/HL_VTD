# HL FMA 제어기 최종 구조

## 설계 원칙

- 클래스 대신 설정·유지 상태·프레임 결과를 구조체에 둔다.
- 최상위 Process 함수는 중간값을 반환하지 않고, 읽을 구조체와 갱신할 구조체를 인자로 명시한다.
- `main.cpp`의 반복문은 `World -> Planning -> Control -> TCP Send` 순서만 유지한다.
- 객체 종류는 추정하지 않는다. 수신한 위치·heading·length·width로 OBB를 만들고 경로·횡단보도·충돌 관계만 판단한다.
- Safety는 FSM 상태가 아니라 Control이 최종 명령을 제한한 결과인 `SafetyReport`에 둔다.

## 폴더와 파일

### `src/network`

- `network_structs.hpp`: 1109바이트 GT 프레임, TCP 연결, 수신 버퍼 구조체를 정의한다.
- `network_config.*`: `network.yaml`에서 VTD 주소와 포트를 읽고 검증한다.
- `vtd_client.*`: TCP 9910 연결·수신·송신·수신 타임아웃을 처리한다.
- `receive/packet_parser.*`: 1109바이트를 Ego 1대, 객체 30칸, 신호등 1개로 역직렬화한다.
- `receive/gt_receiver.*`: TCP 스트림을 정확히 1109바이트 프레임 단위로 조립한다.
- `receive/gt_printer.*`: 선택적으로 수신 객체 상태를 화면에 표시한다.
- `send/control_sender.*`: steering 4B, targetAccel 4B, turnSignal 1B를 padding 없는 9바이트로 만들어 동일한 TCP 9910에 보낸다.

### `src/data`

- `gt/gt_structs.hpp`: 원본 GT와 Planning이 읽는 `WorldState` 구조체를 정의한다.
- `gt/gt_processor.*`: Ego·객체를 차선 및 전역경로에 투영하고 객체 OBB를 만든다.
- `gt/world_process.*`: 속도 추정, 리스폰 감지, 앞범퍼 위치, 상대거리, TTC, 횡단보도 점유를 순서대로 갱신한다.
- `map/map_structs.hpp`: Road, Lane, Geometry, LaneGraph, LaneMatch, TrafficController 등 XODR 지도 구조체를 정의한다.
- `map/map_loader.*`: XODR XML 문서를 읽는 진입점이다.
- `map/map_matcher.*`: 월드 좌표를 현재 road/section/lane에 매칭한다.
- `map/static_zones.*`: 자동 추출 정적 지형과 수동 설정을 합치고 현재 전역경로의 routeS로 다시 정렬한다.
- `vehicle/vehicle_structs.hpp`: wheelbase, 폭, 길이, 앞·뒤 오버행 구조체를 정의한다.
- `vehicle/vehicle_config.*`: `vehicle.yaml`을 읽고 차량 제원을 검증한다.

### `src/planning`

- `planning_structs.hpp`: Planning 설정, 유지 상태, 프레임 중간값, 최종 결과 구조체를 정의한다.
- `planning_process.*`: 정적 제약, 정지 요청, 차선변경, 행동, 지역경로, 충돌검사를 명시된 순서로 호출한다.
- `global/reference_path_generator.*`: XODR의 line/arc/spiral/poly3와 lane 폭을 계산해 차선 중심선과 lane graph를 만든다.
- `global/global_path_planner.*`: 체크포인트를 차선에 매칭하고 lane graph를 따라 전역경로를 연결한다.
- `local/path_planner.*`: 전역경로 최근접점, lookahead, 곡률, Pure Pursuit 조향을 계산한다.
- `local/local_path_planner.*`: 현재 위치 앞쪽의 충돌예측용 지역경로를 잘라낸다.
- `local/lane_change_structs.hpp`: 차선변경 설정과 `None/Prepare/Execute/Settle` 유지 상태를 정의한다.
- `local/lane_change_planner.*`: 정적 장애물 회피, 다음 road 연결 시 목표 차선 유지, 필요 시 원래 경로 복귀와 대기를 처리한다.
- `behavior/behavior_structs.hpp`: `Cruise/Follow/Stop`, 차선변경 단계, `Ok/Recovery`, 정지 요청 구조체를 정의한다.
- `behavior/behavior_planner.*`: 제한속도·정지점·앞차를 이용해 종방향 모드와 목표속도를 정한다.
- `collision/collision_checker.*`: 현재 OBB 겹침, TTC, 선택 경로 위 미래 OBB 충돌을 검사한다.

### `src/control`

- `control_structs.hpp`: 제어 설정·PD 유지 상태·nominal/safe 명령·SafetyReport를 정의한다.
- `config/control_config.*`: `control.yaml`의 모든 숫자 설정을 구조체에 넣고 범위를 검증한다.
- `lateral/lateral_controller.*`: Planning 조향값을 차량 최대 조향각 안으로 제한한다.
- `longitudinal/longitudinal_controller.*`: 현재속도와 목표속도로 PD 가속도 명령을 만든다.
- `safety/safety_limiter.*`: 무효 입력·NaN 명령·긴급제동 요청을 최종 안전 명령으로 덮는다.
- `control_process.*`: 횡제어, 종제어, 방향지시등, Safety limiter를 순서대로 호출한다.

### 설정·도구·테스트

- `config/control.yaml`: 속도, 정지, 충돌예측, 차선변경, 제어기, watchdog 값을 둔다.
- `config/vehicle.yaml`: Ego 후륜축 기준 차량 제원을 둔다.
- `config/maps/living_lab.generated.yaml`: XODR에서 자동 추출한 정지선·횡단보도 원본이다.
- `config/maps/living_lab.yaml`: 운영측 값과 신호 점멸 색 등 사람이 확인해야 하는 값을 덧붙인다.
- `config/maps/living_lab_school_zone_points.yaml`: VTD에서 측정한 보호구역 기준점 46개의 원본이다. `living_lab.yaml`이 이 파일을 로드하지만 시작·종료 관계가 확정되지 않은 기준점을 속도구역으로 임의 활성화하지는 않는다.
- `tools/map_extraction/extract_static_zones.cpp`: XODR 정적 객체를 generated YAML로 다시 만든다.
- `tools/path_generation/`: Scenario의 roadId+s 체크포인트를 월드 좌표 경로로 바꾸는 오프라인 도구다.
- `tests/controller_logic_tests.cpp`: 지도 routeS 정렬, 현재/미래 충돌 분리, 차선변경 허용·차단을 검사한다.
- `src/logger/`: 기존 시나리오 검증용 선택 기능이며 Planning 판단에는 사용하지 않는다.

## 실행 흐름

시작할 때 한 번 수행한다.

```text
network/control/vehicle/map 설정 로드
  -> reference_path.csv 로드
  -> XODR 로드 및 lane center/lane graph 생성
  -> generated 정적 지형 + manual 설정 병합
  -> 현재 reference path 기준 routeS 재정렬
  -> TCP 9910 연결 및 수신 watchdog 설정
```

매 GT 프레임은 다음 순서로 처리한다.

```cpp
while (receiveGtFrame(receiver)) {
    const GtFrame& gt = getLatestGtFrame(receiver);
    runWorldProcess(world, gt, receiveTime, dt, mapData, globalPath, vehicle);
    runPlanningProcess(planning, gt, world.state, mapData, dt);
    runControlProcess(control, world.state, planning.result, dt);
    sendControlCommand(receiver.client, control.safeCommand);
}
```

각 Process 내부는 구조체를 직접 갱신하지만 읽기·쓰기 대상이 함수 인자에 보인다.

```text
WorldProcessData
  -> update ego motion
  -> map/route matching
  -> build OBB
  -> update relative S/D and TTC

PlanningProcessData
  -> evaluate constraints and static zones
  -> prepare/advance obstacle lane change
  -> update stop requests
  -> select Cruise/Follow/Stop
  -> generate selected local path
  -> check current/predicted collision

ControlProcessData
  -> lateral control
  -> longitudinal control
  -> apply turn signal
  -> apply safety limiter
```

## 장애물 회피 차선변경

차선변경은 일반 추월이 아니라 정적 장애물 때문에 완주가 막힐 때만 사용한다.

```text
정적 장애물 18~45 m 전방 감지
  -> 같은 방향의 인접 driving lane 탐색
  -> 목표 차선 앞·뒤 간격과 TTC 확인
  -> 기존 전역경로에서 현재 road 다음의 정확한 LaneKey 확인
  -> 목표 차선에서 횡방향 edge를 제외한 successor/junction 연결만 탐색

  -> 다음 LaneKey로 연결됨
       -> 목표 차선을 유지하고 다음 LaneKey부터 기존 전역경로 재사용

  -> 다음 LaneKey로 연결되지 않음
       -> 같은 road/section의 원래 차선으로 복귀하는 경로 생성
       -> 복귀 경로가 막히면 먼저 2 m/s로 감속
       -> 복귀 시작점 직전의 목표 차선 대기 위치에서 정지
       -> 그 전이라도 선택 경로 3초 안에 OBB 충돌이 예측되면 즉시 정지
       -> 안전해지면 복귀 재개
```

여기서 forward 연결은 별도 그래프가 아니다. 기존 lane graph에서 같은 road/section의 옆 차선 이동 edge만 제외한 lane successor, road successor, junction laneLink를 뜻한다. 다음 road의 `roadId/sectionIndex/laneId`가 기존 전역경로와 정확히 같을 때만 목표 차선을 유지한다. 임의의 다른 road는 탐색하지 않는다.

목표 차선 유지와 원래 차선 복귀 후보 모두 전체 경로 OBB 검사를 통과해야 시작한다. 실행 중 복귀가 막히면 즉시 경로를 꺾지 않고 감속하며, 가까운 충돌 위험이면 정지한다.

## 신호등 처리

- 상태 `1`: 적색이므로 정지선 앞에서 정지하고 신호가 바뀔 때까지 기다린다.
- 상태 `2`: 기본 정지하되 이미 넘었거나 안전 감속으로 못 멈추면 그 교차로는 통과한다.
- 상태 `3/4/5`: 진행 가능한 신호로 처리한다.
- 상태 `6`: 색 정보가 아니라 점멸 단계다. `living_lab.yaml`에서 해당 패킷 ID를 `flashing_type: red`로 지정한 경우에만 1초 정지 후 통과한다.
- XODR의 개별 signal은 controller 아래에 묶인다. 예를 들어 controller 11은 signal 54/55/58을 소유한다.
- GT `trafficLight.id`는 개별 signal id가 아니라 controller id이므로 정지선도 `controller_id`로 연결한다.
- controller가 자동 추정되지 않은 정지선만 같은 road/lane의 가장 가까운 정지선을 보조 사용한다.

패킷에는 빨간색 램프의 signal id가 오는 것이 아니라 `controller ID 4B + 그 controller의 현재 논리 상태 1B` 한 세트가 들어 있다. 따라서 한 프레임의 1109바이트에는 controller 상태 레코드가 하나뿐이다.

## 현재 완료와 남은 입력값

코드 대응 완료: 경로추종, 제한속도 상한, 차선 유지 기반 경로, 실선·중앙선 차선변경 금지, 적색·황색·적색점멸 판단, 횡단보도 객체 점유, 앞차 추종, 정적 장애물 정지/회피, 방향지시등, 현재·미래 OBB 충돌제동, 리스폰/맵매칭 Recovery, GT 수신 watchdog, 공식 종료 좌표 통과 판정 로직.

대회 값이 있어야 활성화되는 항목:

- `traffic_lights`: 상태 6이 적색점멸인 VTD controller ID
- `speed_zones`: 46개 기준점에서 확정한 붉은 보호구역의 정확한 polygon 또는 전역경로 `route_s` 시작·종료와 제한속도
- `finish`: 운영측 공식 종료점 x/y
- `vehicle.yaml`: 운영측 차량의 정확한 앞·뒤 오버행
- VTD 실차 시나리오 확인: controller/정지선 대응, 조향 부호, 방향지시등 값, 충돌 이벤트, 차선변경 튜닝

이 값들이 비어 있는 상태에서는 관련 로직은 준비되어 있어도 모든 공식 평가항목을 만족한다고 확정할 수 없다.
