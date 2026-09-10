# M17 체크포인트 및 검증 상태

VTD `PathShape02` 원본의 웨이포인트 6개를 `checkpoints.csv`에
소수점 셋째 자리까지 그대로 반영했습니다. Z와 Yaw를 포함한 원본값은
`path_shape_waypoints.csv`에 별도로 보존했습니다.

- 체크포인트 구조: 시작 P1, 진입·진출 P2→P3 / P4→P5, 도착 P6.
- XODR 연결 검사: 콜리더 3개, 게이트웨이 2개, 순서 연결·게이트웨이
  인터페이스·표면 연결 모두 통과. `mission_network.json`과
  `mission_corridor.svg`에 결과를 저장했습니다.
- P6은 junction 10 내부이며, 방향성 그래프로 도달 가능한
  connector `(road 781, section 0, lane -1)`의 `road_s=33.366 m`를
  종료 target으로 사용합니다. 체크포인트와 차선 표면 거리는
  0.058 m입니다.
- `checkpoint_shape.csv`/`.svg`는 VTD PathShape의 직선 보간 형상
  (2,364개 점, 1,179.63 m)입니다. 차선 중심 주행 경로가 아니므로
  `route.csv`로 사용하지 않습니다.
- 차선 그래프 경로 생성은 P4 근처에서 연결 차선 후보를 찾지 못해
  실패했습니다. lane-snap 시험도 최대 차선 이탈 49.57 m,
  곡률 1.969 1/m로 검증 기준을 초과했습니다.
- VTD 원본 Ego 시작은 (377.530, -258.977)이며 P1에서 약 380 m
  떨어져 있습니다. 유효한 `route.csv` 생성과 Ego 시작 위치·방향 정렬 전에는
  실제 주행을 시작하면 안 됩니다.

검사를 우회하거나 곡률/거리 허용치를 늘리지 않았습니다.
