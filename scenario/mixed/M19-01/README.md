# M19-01 체크포인트 경로

VTD `M19-1.xml`의 `PathShape01` 화면에 표시된 웨이포인트
4개를 `checkpoints.csv`에 소수점 셋째 자리까지 그대로 반영했습니다.
Z·Yaw·Time 원본 표기값은 `path_shape_waypoints.csv`에 보존합니다.

- 체크포인트 구조: 시작 P1, 교차로 진입·진출 P2→P3, 도착 P4.
- `checkpoint_shape.csv`/`.svg`: VTD PathShape 직선 보간 미리보기.
- `route.csv`/`.svg`: OpenDRIVE 차선 그래프 경로. 그래프가 끊기는
  P2→P3 구간만 8 m 핸들의 곡률 제한 연결로 이었습니다.
- `mission_corridor.csv`/`.svg`: XODR 기반 사전 미션 검증 산출물.

저장소 시나리오 ID 규칙에 맞춰 `M19-01`로 정규화했으며,
원본 `M19-1.xml`은 WATCH 및 `run_scenario.py`의 실행 전
동기화 대상입니다.
