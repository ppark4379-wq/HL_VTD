# M19-02 체크포인트 경로

VTD `M19-2.xml` `PathShape02` 화면의 웨이포인트 4개를
`checkpoints.csv`에 소수점 셋째 자리까지 반영했습니다.
Z·Yaw·Time은 `path_shape_waypoints.csv`에 보존합니다.

- 구조: 시작 P1, 교차로 진입·진출 P2→P3, 도착 P4.
- `checkpoint_shape.csv`/`.svg`: PathShape 직선 보간 미리보기.
- `route.csv`/`.svg`: OpenDRIVE 차선 그래프 주행 경로.
- P2→P3 단절 구간은 8 m 핸들 곡률 제한 연결을 사용합니다.
