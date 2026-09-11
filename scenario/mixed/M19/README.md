# M19 체크포인트 경로

VTD `PathShape01` 화면의 웨이포인트 4개를 `checkpoints.csv`에
소수점 셋째 자리까지 그대로 반영했습니다. Z·Yaw·Time 원본 표기값은
`path_shape_waypoints.csv`에 별도로 보존합니다.

- 체크포인트 구조: 시작 P1, 교차로 진입·진출 P2→P3, 도착 P4.
- `checkpoint_shape.csv`/`.svg`: VTD PathShape 직선 보간 미리보기
  (285개 점, 141.534 m). 차선 중심 주행 경로가 아닙니다.
- `route.csv`/`.svg`: OpenDRIVE 차선 그래프 경로. 그래프가 끊기는
  P2→P3 구간만 8 m 핸들의 곡률 제한 연결로 이었습니다.
- `mission_corridor.csv`/`.svg`: XODR 기반 콜리더 2개와
  게이트웨이 1개의 사전 미션 검증 산출물.

원본 Ego 시작은 `(388.169, -335.616)`, heading `129.10°`입니다.
P1 좌표와는 약 0.96 m 이내이며 첫 구간 방향과 일치합니다.
