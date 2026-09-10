# M18 체크포인트 경로

VTD `PathShape01` 화면의 웨이포인트 10개를 `checkpoints.csv`에 반영했습니다.
Z·Yaw·Time 원본 표기값은 `path_shape_waypoints.csv`에 별도로 보존했습니다.

- 체크포인트 구조: 시작 P1, 진입·진출 P2→P3 / P4→P5 / P6→P7 /
  P8→P9, 도착 P10.
- `checkpoint_shape.csv`/`.svg`: 체크포인트 직선 보간 PathShape 미리보기
  (1,265점, 629.624 m). 차선 중심 주행 경로가 아닙니다.
- `route.csv`/`.svg`: XODR 차선 그래프 경로. 기본 그래프가 끊기는 P4→P5만
  15 m 핸들의 곡률 제한 연결로 이었고, 현재 XML Ego 위치부터 잘라 1,241점을
  저장했습니다.
- `mission_corridor.csv`/`.svg`: XODR 기반 콜리더 5개와 게이트웨이 4개.
  현재 Ego 시작은 두 번째 콜리더이므로 `mission_start_leg: 1`입니다.
- P10은 junction 9의 겹치는 connector road 775/4471 위에 있습니다. 마지막
  PathShape 방향(23.90°)과 일치하는 road 775를 종료 target으로 선택합니다.

현재 VTD 원본 Ego는 `(377.530, -258.977)`, heading `225.07°`입니다. 생성된
경로 시작 heading `40.63°`와 반대여서 시나리오 패키지 구조·경로·미션 네트워크는
생성됐지만 `run_scenario.py`의 시작 방향 검증은 실패합니다. 실제 주행 전 VTD에서
Ego 방향을 경로 진행 방향으로 돌려 저장해야 합니다.
