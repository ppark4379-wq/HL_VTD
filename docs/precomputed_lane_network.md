# XODR 사전 차선 점 네트워크

현재 지도 전체의 **0이 아닌 driving lane**을 샘플링하고, 도로 내부에서 차선이 생기거나 사라지는 구간에는 XODR 형상·통행 조건을 검사한 합류/분기 경로를 추가한다. 시나리오 XML이나 장애물 위치는 입력하지 않는다. 출력은 전역 완주 경로 하나가 아니라 선택 가능한 점 네트워크다. 일반 장애물 회피용 차선변경과 Corridor/Gateway의 선택 로직은 이후 통합 대상이다.

## 생성 / 검수

```bash
python3 tools/map_precompute/precompute_lane_network.py
```

기본 원본은 `src/data/map/HL_FMA_VTD_LivingLab.xodr`, 출력은 `output/precomputed_lane_network/`다. numpy/scipy/matplotlib이 필요하다. `--interval 0.5`, `--png-pixels 8000`, `--xodr`, `--output`을 지정할 수 있다. 전체 지도 생성은 기존 제어기의 실행 경로를 바꾸지 않는다.

| 파일 | 용도 |
|---|---|
| `index.html` | 독립 실행 검수 화면: 확대/이동, road ID 찾기, 경로 선택, 실제 점 표시, 레이어 전환, 검토점 순회 |
| `road_map.png`, `road_map.svg` | 도로 전체, 차선 경계, 노면 표시 |
| `lane_network.png`, `lane_network.svg` | 같은 도로 위에 노란 차선 중앙점 연결선과 주행 방향 |
| `details/*.png` | 합류/불연속 사례 80m × 80m 확대 |
| `lane_points.csv` | 모든 점과 다음/좌우 인접 점 참조 |
| `lane_connections.csv` | 차선 끝의 모든 명시적 종방향 연결과 검사 결과 |
| `lane_transitions.csv` | 생성한 합류/분기의 종류, 공통 시작/종료 점 ID, 출처와 검사값 |
| `transition_points.csv` | 실제 합류/분기 곡선의 점·방향·곡률·누적거리 |
| `replaced_points.csv` | 합류/분기로 대체된 원본 차선의 막다른 꼬리 점 ID |
| `transition_checks.csv` | 실선·차선변경 제한, 차체 검사 결과, 플래닝 검증 필요 여부 |
| `lane_access.csv` | 차선별 도로망 진입/진출 가능 여부(위상 검사), 노란 경로 제외 사유 |
| `manifest.json` | 원본 SHA256, 단위, 차량 검사값, 차선별 점 범위/연결 인덱스 |
| `validation.json` | 연결/형상 진단, 노면 표시 수량, 생략/기호 표현의 사유 |
| `full_network_audit.json` | 전체 방향·공유 앵커·숨겨진 대체 경로 검사, 잔여 원본 단절 |
| `reviewed_17/index.html` | 기존 번호 2·3 제외한 17건 수정 전후 사진 |
| `coverage/index.html` | 빈 차선 복원 및 같은 도로 내 원본 단절의 전후 사진 |

PNG 긴 변은 기본 8,000픽셀이다. 도로가 넓으므로 상세 검수는 SVG 또는 HTML을 확대해서 한다. 흰 노면 화살표와 노란 진행방향 화살표를 구분한다. 기본 노란 선은 유효 원본 중앙선과 생성한 합류/분기 경로다. 대체된 막다른 꼬리는 노란 선에서 제외하고, HTML의 `원본 중앙선` 레이어에서 원본 그대로 비교할 수 있다. 미해결 원본 불연속은 숨기거나 임의 연결하지 않는다.

## 좌표 생성

`line/arc/spiral/poly3` 기준선을 계산하고 `laneOffset`과 안쪽 차선들의 `width` 다항식을 더해 차선 중앙의 횡좌표를 구한다. spiral은 수치적분, poly3는 **곡선의 호 길이를 역산**해서 평가한다(`u=road_s`로 취급하지 않음). 미지원 geometry, LHT, 차선 방향 override, border 기반 폭/뱅크 등은 임의 대체하지 않고 오류로 끝낸다. 현재 지도는 지원 범위인 RHT/width/평면 형상을 사용한다.

중앙점 위치는 `reference + t × normal`이다. heading과 curvature는 reference heading을 복사하지 않고, `t(s)`의 1·2차 미분과 reference curvature를 포함한 중앙선 자체의 미분으로 구한다. 따라서 폭이 변하거나 중앙 offset이 움직이는 합류/분기부의 기울기를 반영한다.

road_s 간격 0.5m를 시작으로 실제 중앙점 간 거리가 목표 간격의 0.1% 허용오차를 넘으면 추가 평가한다. geometry/width/offset/roadMark 시작점도 포함하며 불연속을 찾기 위해 knot 직전도 평가한다. 양수 lane은 점 순서를 뒤집어 주행 방향으로 저장한다. 두 section의 양 끝점은 각각 보존한다. 같은 위치라고 해도 서로 다른 LaneKey이며 연결 테이블에서 관계를 표현한다.

## 연결과 검증

원본 종방향 연결 테이블은 차선 section의 successor/predecessor, road link + lane link, junction connection/laneLink만 사용한다. 근접 차선, 같은 ID, 시각적으로 가까운 교차로 사이에 원본 연결을 추가하지 않는다. junction에서 연결 대상의 `contactPoint`와 주행 방향이 맞는지 확인하고 원본 junction ID/connection ID를 보존한다. 이 규칙은 [ASAM OpenDRIVE lane linkage](https://www.asam.net/fileadmin/Standards/OpenDRIVE/ASAM_OpenDRIVE_BS_V1-7-0.html)의 명시적 lane 연결 모델을 따른다.

`status=continuous`는 위치 오차 ≤ 0.05m, heading 차이 ≤ 5°, 곡률 차이 ≤ 0.05/m, 폭이 0이 아닌 연결이다. 검사 한계는 지도 불연속 검수용이며 C2 수학적 증명이나 차량 통과 보증이 아니다. 연결별 실제 측정값을 모두 저장한다. 부족한 원본을 부드럽게 보이도록 보간하지 않는다.

원본 점의 `usable`은 현재 설정의 차량 폭(1.880m), wheelbase(2.944m), 최대 조향각(0.48rad)에 의한 곡률 상한만 검사한다. 원본 점 전체에 차량 길이·실선 침범 검사를 수행한 것은 아니다. `no_declared_successor`는 도로 끝/차선 소멸에도 정상적으로 생길 수 있으므로 모두 지도 오류라는 의미는 아니다. 폭이 0으로 수렴하는 차선 끝을 차량이 그대로 주행할 수 있는 것도 아니다.

좌우 인접점은 같은 road/section에서 주행방향이 같은 인접 lane의 동일 road_s에 가까운 샘플만 참조한다. `left/rightChangeAllowed`는 그 점의 roadMark가 broken이고 laneChange 방향이 허용되는지 나타낸다. 이것은 횡방향 next 연결이 아니며, 전체 차선변경 구간의 안전/합법성을 보증하지 않는다.

## 합류/분기: 실제 공유 점과 매끄러운 곡선

차선 폭이 0이 되는 지점의 중앙점은 옆 차선의 중앙점이 아니라 경계에 놓인다. 따라서 원본 중앙점만 그리면 두 선이 합쳐지지 않는다. 이를 그림으로만 잇지 않고, 별도의 **실제 주행 점 경로**로 생성한다.

1. 도로 내부의 차선 생성/소멸을 찾는다. 명시적 종방향 연결이 있는 끝은 기존 연결 검사 대상으로 남긴다. 동일 위치의 단순 laneLink 누락을 합류라고 가정하지 않는다.
2. 같은 방향의 바로 인접한 driving lane만 연결 후보로 삼는다. section을 넘는 후보 탐색은 원본 laneLink를 따라가므로 차선 번호 변경도 반영한다. 반대편 도로·임의의 교차로 방향으로 연결하지 않는다.
3. 폭이 0에서 늘어나거나 0으로 줄어드는 실제 taper 범위를 원본 width로 찾는다. 그 범위 전후 가까운 점을 앵커로 사용하며, 고정된 먼 look-ahead 지점을 먼저 선택하지 않는다. 폭 변화 없이 driving→none으로 끝나는 차선은 도로 끝 이전에 차체가 빠져나오는 국소 연결을 탐색한다.
4. taper 중간 60%는 인접 통과 차선 반대쪽 경계선에서 일정 횡간격을 둔 가이드를 따라간다. 따라서 XODR 기준선 좌표계에서 경계선의 `t'/t''`를 그대로 반영한다. 기본 간격은 완성된 차선 폭의 절반이고, 차체 여유가 부족하면 도로 안쪽으로 최대 0.45m를 더 둔다. 접근/이탈 부분은 각각 5차 다항식으로 가이드 및 원본 중앙선의 `t/t'/t''`를 맞춘다. 실제 앵커 위치·방향·곡률 일치와 기준선 불연속도 검사한다. 가이드 후보가 모두 탈락하면 국소 단일 5차 연결을 검사하며, 선택 방식은 `shape_method`에 기록한다.
5. 각 출력 점에서 길이 방향 0.5m 이하 간격으로 직사각형 차체를 검사한다. 폭 1.880m, 기준점 뒤 1.055m/앞 3.800m, 도로 가장자리 여유 0.05m를 사용한다. 실선/차선변경 금지는 후보 삭제 조건이 아니라 `transition_checks.csv`의 플래닝 판단용 플래그다. 짧은 13번 구간은 차량 전체 검사 미통과 시 중심점의 driving 영역 및 곡률을 검사한 기하 후보만 보존하며 `vehicle_footprint_valid=0`과 실패 사유를 기록한다. 검증 전 주행용으로 사용하면 안 된다. 경로 점 간격은 최대 0.51m다.
6. 시작/종료는 원본 점 ID를 공유한다. 합류 후 막다른 꼬리와 분기 전 고립된 새 차선 꼬리는 `replaced_points.csv`로 대체 처리한다. 원본 파일과 원본 점 좌표는 보존한다.

이 곡선은 원본 XML에 존재하던 laneLink가 아니라, 원본 도로 조건에서 생성한 횡방향 주행 경로이며 `provenance`로 구분한다. 샘플링된 차체 검사이지 연속 swept-volume의 수학적 보증은 아니다. 속도별 조향 변화율, 장애물, 실제 제어기 추종 검증도 아직 포함하지 않는다. 후보 길이/공간 부족, 실선, 원본 형상 불연속, 다른 경로 앵커와의 충돌 등으로 생성하지 못한 곳은 `validation.json`의 `transitions/status=unresolved`에 남긴다. 도로/교차로 사이의 원본 불연속까지 전부 복구했다는 뜻은 아니다.

### 회전교차로 등 주변 도로망에 연결되지 않은 차선

원본 laneLink와 생성한 합류/분기를 방향 그래프로 검사한다. 다른 road로 이어지는 명시적 연결 또는 실제 열린 지도 끝에서 들어오고 나갈 수 있어야 `lane_access.csv`의 `status=connected`다. junction으로 이어진 road 끝에 해당 laneLink가 없으면 열린 지도 끝이라고 간주하지 않는다. `no_entry/no_exit/no_entry_or_exit`도 노란 기하 후보로 표시하며 위상 미연결을 이유로 차선 전체를 숨기지 않는다. 기존 검토 14~17·19번처럼 사용자가 명시적으로 제외한 차선만 `excluded_by_review`로 숨긴다. 원본 점은 보존한다. 실제 사용 여부는 플래닝에서 위상·실선·차체·장애물 제약으로 결정한다.

이는 차선 단위 위상 검사이며, 원본 링크의 위치/방향 불연속은 별도 진단으로 보존한다. 따라서 `connected`가 모든 실제 점의 기하 연속성 또는 주행 안전성을 보증하지 않는다. 회전교차로에서 가까운 선에 임의로 붙여 원본에 없는 진출입을 만드는 일도 하지 않는다.

같은 road의 명시적 laneLink 또는 단일 차선 내부에서 5cm 초과 위치 단절이 있으면 별도의 `continuation` 후보를 탐색한다. 기존 원본 `nextPoint=-1`과 source 진단은 변경하지 않는다. 방향이 같은 선언된 차선 트랙 안에서 실제 공유 앵커를 선택하고, world-space C2 곡선의 도로 영역·곡률·샘플 간격을 검사한다. 전체 차체가 통과하지 못한 기하 후보는 별도 플래그로 남긴다. 다른 road 사이로 임의 연결하지 않으며 실패한 곳은 `source_gap_repairs`에 보존한다. 늦은 국소 분기는 다른 연결의 앵커와 도착점을 유지한 채 분기 시작점만 taper 앞쪽으로 조정한다.

노면 화살표 모델의 원본 heading과 차선 진행 접선이 90° 넘게 다르면 `pavement_lane_direction_conflict`로 보고한다. 외부 모델의 방향만 보고 lane 방향이나 junction 연결을 덮어쓰지는 않는다. 실제 확인 사례인 road 211의 object 679(`RM_537_LT`, s≈4.872, t≈−1.593)는 +s 방향이지만, 해당 `211:0:1`은 양수 lane이고 junction 12에 from=1 연결이 없다. 따라서 기존 노란 선의 −s 방향과 노면 표시가 충돌하며 정상 주행 후보로 해석하면 안 된다. 차량 화면 좌표 s=15.307, t=1.197은 별도의 정상 차선 `211:2:2`이고 x/y, 폭, 중앙 offset이 파서 결과와 표시 정밀도 내에서 일치한다. 높이 및 외부 FLT 텍스처의 완전 일치까지 검사한 것은 아니다.

## 노면 시각화의 출처와 한계

실선/점선/이중선의 위치·색·폭은 XODR `roadMark`, 노면 object의 위치·방향·크기는 `s/t/hdg/length/width`에서 읽는다. `type/line`에 패턴이 있으면 사용하고, 현재 지도처럼 없으면 점선의 3m 실획/6m 공백과 이중선 분리는 시각화 기본값을 사용하여 보고서에 기록한다.

외부 FLT 텍스처 자체는 XODR 안에 없다. `RM_537_ST/LT/RT`, `RM_538_SLT/SRT`, `RM_539_LUT/UT`는 이름별 방향 기호, `RM_532_*`와 crosswalk 모델은 횡단보도 기호로 표시한다. stop line은 원본 크기의 사각형, crosswalk warning은 마름모로 그린다. 의미가 불분명한 나머지 노면 모델은 원본 크기의 윤곽과 모델명을 보존한다. 이 화면을 실제 VTD 렌더링의 픽셀 단위 복원으로 해석하면 안 된다. 수치가 깨진 원본 object는 `invalid_object`로 보고하며 지도 전체를 덮는 비정상 사각형을 그리지 않는다.

## C++ 구조체와 로드

`src/data/map/precomputed_lane_network.hpp`에 다음 구조체가 있다.

```text
PrecomputedLaneNetwork
├── points[id]: PrecomputedLanePoint
│   ├── pose: Waypoint (x, y, yaw, s, roadS, roadId, sectionIndex, laneId)
│   ├── curvature, width, pointIndex
│   ├── nextPoint, leftPoint, rightPoint
│   ├── leftChangeAllowed, rightChangeAllowed, usable
│   └── outgoingTransitions[], replacedByTransition
├── lanes[LaneKey]: PrecomputedLane
│   ├── pointIds (주행 순서)
│   ├── outgoingConnections (분기 시 여러 개)
│   └── hasTopologicalEntry, hasTopologicalExit, topologyStatus
├── connections[]: PrecomputedLaneConnection
│   ├── from/to LaneKey 및 fromPoint/toPoint
│   ├── provenance, contactPoint, junctionId, connectionId
│   └── gap, headingDelta, curvatureDelta, status
└── transitions[]: PrecomputedLaneTransition
    ├── kind, eventLane, fromPoint, toPoint, provenance
    ├── minClearance, maxCurvature
    ├── crossesSolid, violatesLaneChangePermission, requiresPlannerValidation
    ├── vehicleFootprintValid, geometryOnlyReasons, roadMarkConflicts
    └── points[]: roadS, s, x, y, yaw, curvature
```

`pose.s`는 **차선 내부 주행 누적거리**로, 기존 global route_s와 섞으면 안 된다. section/road 끝에서 `nextPoint=-1`이며 `outgoingConnections`로 분기를 찾는다. 원본 내부 불연속에서도 `nextPoint=-1`이다. 하나의 `successorLane` 정수만 저장해서 junction의 여러 합법적인 방향을 잃지 않도록 했다.

합류/분기는 차선 내부의 점에서도 출발하므로 `points[id].outgoingTransitions`를 사용한다. 선택한 transition의 점들을 따라간 뒤 `toPoint`에서 원본 차선을 계속 따라간다. 분기에서는 원래 차선의 직진과 transition 두 선택이 가능하다. 합류에서는 `replacedByTransition >= 0`인 원본 꼬리를 경로로 선택하면 안 된다. 기존 `nextPoint`만 따라가는 소비자는 새 연결을 사용하지 못한다. transition의 `s`는 해당 곡선 내부 누적거리다. 이 선택 처리는 아직 라이브 플래너에 통합하지 않았다.

새 출력은 가시화용 기하 후보를 넓게 보존한다. 플래닝에서 목적지 도달성, 실선/권한, 차량 전체 통과, 장애물을 검사해야 하며 노란색이라는 이유만으로 사용하면 안 된다. `vehicleFootprintValid=false`는 차량 전체 검사 미통과 또는 미검증이다. `excluded_by_review`는 명시적 제외이며, 위상 미연결은 경로 탐색 시 별도 판단한다. 이전 출력처럼 `lane_access.csv`가 없으면 `not_audited`로 표시한다.

```cpp
PrecomputedLaneNetwork network;
std::string error;
loadPrecomputedLaneNetwork("output/precomputed_lane_network", network, &error);
```

로드는 잘못된 참조, 숫자, 열 구성, 역순 점, transition의 앵커 위치/방향/곡률 불일치, 샘플 간격, 대체된 앵커를 검사하고 실패 시 기존 객체를 유지한다. 현재 로더는 CSV 참조 정합성 검사용이며 도로 영역 검사를 재실행하지 않는다. 런타임 통합 시 manifest의 source hash 및 차량/형상 설정 일치를 추가로 확인해야 한다. schema 2의 세 transition 파일이 모두 없는 schema 1 출력도 읽을 수 있다.

```bash
cmake -S . -B build
cmake --build build --target check_precomputed_network -j 4
build/check_precomputed_network output/precomputed_lane_network
python3 -m unittest discover -s tests -p 'test_precomputed_lane_network.py' -v
```
