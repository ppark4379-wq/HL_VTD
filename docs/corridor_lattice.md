# Corridor 전체 차선 후보 플래너

## 적용 범위

`--checkpoints`로 실행하는 mission에서 `output/precomputed_lane_network`를 읽는다. 프로젝트 루트에서 실행해야 한다. 체크포인트가 없는 기존 실행은 기존 플래너를 사용한다. 지도 생성/누락 검사는 `precomputed_lane_network.md`를 참고한다.

- 정상 추종: 현재 차선의 사전 샘플과 명시적 종방향 연결을 따라 pure pursuit 경로를 공급한다. road ID 끝에서 lookahead를 끊지 않는다.
- 정적 장애물 접근: 현재 corridor의 **모든 차선**을 먼저 열거한다. 각 차선 × 완료거리 12/20/30m의 5차 곡선을 만들고, 이후 실선·진행방향·차량 footprint·충돌을 검사한다. 이때 gateway 연결성은 후보 탈락 조건으로 사용하지 않는다. 눈앞 장애물의 즉시 회피가 우선이다.
- 선택: 충분한 후속 여유가 있는 후보 중 차선 이동량·곡률 비용 최소. 모두 앞쪽이 막혀 있으면 충돌 없는 기동 중 후속 여유 최대, 비슷하면 이동 비용 최소. 곡선 자체가 충돌하는 후보를 강제로 선택하지 않는다.
- 회피 후 목표 차선을 계속 추종한다. 원래 차선으로 돌아가는 5차 복귀 경로를 강제하지 않는다.
- gateway: 허용된 종방향/차선변경 그래프에서 target lane까지 남은 횡이동 단계 수를 계산한다. 안전한 장애물 회피 경로가 gateway 단계도 줄이면 두 목적을 함께 달성한다. 회피 기동 중이라도 목표 차선 중심에 안착하고 다음 gateway 단계가 장애물 여유를 통과하면 즉시 연속 연결한다. 단, gateway 때문에 유일한 안전 회피 후보를 버리지는 않는다. 남은 단계당 20m의 계획 여유를 예약하고, gateway 후보는 기본 12/20/24m 완료거리 중 가장 짧은 안전 후보를 쓴다.
- 최종 gateway 후보의 tail은 현재 corridor 끝에서 끝날 수 있다. 이를 경로 실패로 오판하지 않고 gateway까지의 부드러운 prefix를 유지한 뒤, target road가 관측되면 다음 corridor가 tail을 이어받는다. 장애물 여유는 잘라지지 않은 물리 lane continuation으로 검사한다.
- 기존 신호/보행자/횡단보도/학교구역/긴급제동/조향 rate limiter는 유지한다. 안전 정지 중에도 경로를 충돌 방향으로 강제하지 않는다. 안전 경로가 없으면 검색 실패를 기록한다.

## 자료 구조와 코드

| 위치 | 역할 |
|---|---|
| `src/planning/local/corridor_lattice.{hpp,cpp}` | 점 그래프, corridor mask, gateway 역탐색, 전체 차선 후보/5차/footprint/충돌/비용 |
| `PathGeneratorData::corridorRuntime` | 공유 런타임 지도, 공간 인덱스, forward/reverse edge, gatewayCost, 현재 leg 캐시 |
| `PrecomputedLaneNetwork` | CSV에서 읽은 원본 점/명시적 연결/검증된 합류·분기 transition |
| `LaneChangeData` | 선택된 실제 waypoint 열, `KeepTargetLane`, 완료 s/차선, 게이트웨이 기동 여부 |
| `lane_change_planner.cpp` | mission이 있는 경우 위 구현으로 3개 진입점 전달; 기존 상태 전이는 유지 |
| `planning_process.cpp` | 장애물 회피 우선 확정, 차선 중심 안착 시 gateway 단계 연속 연결 |
| `visualization_logger.cpp`, `rviz_bridge.py` | 전체 사전 경로 노란 선, 체크포인트 분홍 원, 기존 레이어 유지 |

## 조정할 파라미터 (`config/control.yaml`)

| 파라미터 | 기본값 | 의미/주의 |
|---|---:|---|
| `lattice_short_distance` | 12m | 가장 짧은 횡이동 완료거리. 작게 하면 좁은 공간 후보 증가, 곡률 증가. gateway 횡이동 그래프 거리에도 적용 |
| `lattice_medium_distance` | 20m | 중간 후보 완료거리 |
| `lattice_long_distance` | 30m | 긴 후보 완료거리. 멀리 잡으면 완만하지만 다음 장애물/실선까지 지나갈 수 있음 |
| `lattice_lane_cost` | 2 | 횡이동량 비용. 높이면 현재/가까운 차선 선호 |
| `lattice_curvature_cost` | 30 | 곡률 비용. 높이면 완만한 후보 선호 |
| `lattice_progress_cost` | 1 | 후속 장애물 여유 비용. 충돌 hard filter를 완화하지 않음 |
| `lattice_obstacle_clearance` | 0.30m | 정적 장애물 검사 시 차량 박스를 사방으로 확장하는 여유. M08의 10cm 미만 근접 통과 후보를 거르기 위해 추가. 낮추면 접촉 위험, 높이면 후보 부족 |
| `gateway_reserve_distance` | 8m | 남은 단계별 20m 계획 거리 뒤에 더하는 마지막 횡이동 여유 |
| `obstacle_trigger_maximum_distance` | 70m | 정적 장애물 후보 검색 시작 범위 |
| `maximum_lane_change_speed_kph` | 15km/h | 기존 횡이동 속도 제한 |
| `prepare_time` | 0.5s | 준비 거리 계산에도 반영되는 기존 선행 시간 |

5차 완료거리는 pure pursuit lookahead와 별개다. 곡률 폭발/경로 단절은 파라미터 튜닝으로 숨기지 말고 점 간격·방향·연결부터 고쳐야 한다. EBS/신호 정지 임계값을 낮춰 회피 실패를 감추지 않는다.

## 검증 실행

```bash
cmake --build build --target hl_vtd corridor_lattice_tests controller_logic_tests -j4
build/corridor_lattice_tests
build/corridor_lattice_tests --m05-tail-regression
build/corridor_lattice_tests --m08-obstacle-replay
build/corridor_lattice_tests --m08-gateway-replay
build/corridor_lattice_tests --m08-final-gateway-replay
build/corridor_lattice_tests --m08-corridor-handoff-replay
build/corridor_lattice_tests --all-scenario-handoff-audit
build/controller_logic_tests --preserved-safety
python3 tools/verification/verify_corridor_batch.py --tag UNIQUE_RUN_TAG --first 5 --last 15 --timeout 420
```

배치는 로컬 XML을 VTD에 직접 전송하며 사용자 시나리오를 외부 원본으로 덮어쓰지 않는다. 각 실행에 CSV, 결정 trace, RViz 데이터, console log를 남긴다. 주행 데이터 없는 환경 실패는 별도로 중단한다. 주행 실패가 4개면 이후 시나리오를 중단한다. 실행 도중 바이너리가 바뀌면 별도 버전으로 검증해야 한다.

`collision`은 현재 객체 OBB 겹침 판정이다. VTD 네이티브 접촉 센서 전체를 검증했다는 뜻이 아니다. 빈 CSV 항목인 실선/중앙선 침범 수치를 0으로 해석하면 안 된다. 조향은 명령과 계획 각각의 변화량을 기록하며 wall-time 변화율을 simulation-time 변화율로 혼동하지 않는다.

## 검증 한계 / 알려진 기존 회귀

- 안전 회귀 부분집합 9개가 통과해도 전체 테스트가 통과한 것은 아니다. assertion을 활성화한 전체 기존 테스트는 `testCloseMovingLeadMayCommandStop`에서 실패한다. 변경 전 behavior 소스만으로도 재현됐다.
- 별도 `--traffic-stop-regression`은 기존 코드가 traffic stop 중 회피 준비를 유지하는 동작과 테스트의 취소 기대가 달라 실패한다. 이 작업에서 그 안전 정책을 임의로 바꾸지 않았다.
- footprint 검사는 사전 샘플 strip 위 차량 길이/폭 샘플 검사이며 정확한 연속 폴리곤 충돌 증명은 아니다. 도로/section 경계의 road-mark 횡단 검사는 추가 보강 여지가 있다.
- XODR 미분 경계의 1mm 미만 중복점은 제어 경로에서 생략하고, 방향/거리 곡률 제한을 넘는 연결은 corridor 내부 5차 보간으로 검사한다. 회피 후 추종 tail에도 적용한다. 보간이 불가능하면 후보를 거르므로, 별도의 대체 경로가 없으면 주행이 멈출 수 있다.
- `--m08-gateway-replay`는 위험한 직행 후보를 버리고 30cm 장애물 여유를 만족하는 중간 단계를 선택한다. `--m08-final-gateway-replay`는 기존에 `tracking_tail_not_smoothable`로 잘못 버리던 마지막 `429:0:2` 후보를 고정한다.
- gateway 역탐색은 도로/실선 기반 도달성이다. 아직 먼 장애물 배치를 포함한 gateway까지의 전 구간 collision-free 도달성을 증명하지 않는다. 가까운 회피가 성공해도 마지막 정렬 공간이 부족할 수 있다.
- 실제 배치 결과와 최신 수정의 재현 테스트를 혼동하지 않는다. `output/verification/corridor_lattice_v2_inline_20260908/README.md`에 버전과 실패 결과를 구분해 기록했다.
- 오프라인에 보이는 모든 노란 선이 실행 가능하다는 뜻이 아니다. geometry-only/실선 통과/footprint 실패 transition은 실행 그래프에서 제외한다.
- 임의 장애물 배치에서 정지 없이 반드시 통과할 수 있다는 보장은 없다. 물리적으로 막힌 구간에서 충돌을 강제하지 않으며, 재탐색·정지 후 재출발 성능은 실제 시나리오 결과로 판단해야 한다.
