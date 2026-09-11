# VTD Scenario 자동 동기화

VTD Editor의 원본 XML은 VTD 프로젝트 폴더에 그대로 둔다. 이 도구는 원본을
읽기만 하고 저장소의 `scenario/<category>/<scenario>/`로 단방향 복사한다.
Git add/commit/push는 수행하지 않는다.

## 빌드

```bash
bash scripts/build.sh
```

빌드 결과에 `build/vtd_path_to_checkpoints`가 추가된다.

## 한 파일 동기화

WSL/Linux:

```bash
python3 tools/sync_vtd_scenario.py /path/to/VTD/P01_pedestrian_crossing.xml
```

Windows PowerShell에서 WSL을 사용할 때:

```powershell
bash -lc "python3 tools/sync_vtd_scenario.py '/mnt/c/VTD/Scenarios/P01_pedestrian_crossing.xml'"
```

파일명의 첫 글자로 category를 선택한다.

```text
B -> basic       T -> traffic      V -> vehicle
P -> pedestrian  O -> obstacle     M -> mixed
```

`O1_...`처럼 한 자리 ID는 저장소에서 `O01_...`로 정규화한다. 같은 ID의 기존
폴더가 하나 있으면 그 폴더를 갱신한다.

`M19-1.xml`처럼 숫자 파생 ID는 `M19-01`로 정규화하여 기본
`M19`과 다른 패키지로 동기화한다.

## 저장 감시

VTD Scenario 원본만 모아 둔 폴더를 지정한다.

```bash
python3 tools/sync_vtd_scenario.py /path/to/VTD/Scenarios --watch
```

기본적으로 시작할 때 기존 XML을 한 번 동기화하고 이후 1초마다 변경을 확인한다.
기존 파일은 건너뛰고 새 저장부터 감시하려면 `--no-initial-sync`를 추가한다.

VTD가 큰 XML을 여러 번 나누어 쓰는 동안 읽지 않도록 파일 크기와 수정 시간이
안정된 뒤 XML 파싱에 성공할 때만 반영한다.

## 경로 생성 규칙

Ego가 다음 구조를 사용할 때 완전 자동 생성한다.

```xml
<PosRoute/>
<PathRef PathId="1" StartLane="3" .../>
```

연결된 `<Path>`의 모든 `TrackId`와 `s`를 읽고, `TrackId == XODR road id`로
해석한다. 첫 road는 `StartLane`으로 고정하고 이후 road는 기존 successor,
junction laneLink와 lane-change graph에서 전체 연결거리가 가장 짧은 lane
조합을 선택한다.

성공 시 다음 네 파일을 원자적으로 교체한다.

```text
<scenario>.xml
checkpoints.csv
route.csv
route.svg
```

대상 Package에 `scenario.yaml`이 있으면 XML 저장 시 다음 XML 연동 필드도
자동으로 맞춘다.

- `scenario.file`
- `validation.ego_control`
- `entities`의 차량·보행자·정적 객체 목록
- Ego의 `initial_position` (`PosAbsolute`인 경우)

기존 엔티티의 역할과 추가 메타데이터는 이름이 같은 동안 유지한다. XML에서
삭제된 엔티티와 해당 엔티티를 참조하는 manifest event는 함께 제거한다. 설명,
평가 코드, 경로 옵션처럼 XML에서 결정할 수 없는 값은 변경하지 않는다.

PosAbsolute checkpoint 중 일부 구간이 유효한 차선 graph로 연결되지 않는
시나리오는 `scenario.yaml` route에 1부터 시작하는 구간 번호를 지정한다.
도구는 양쪽의 정상 차선 경로를 각각 생성한 뒤 진입·진출 방향을
Bezier 곡선으로 연결하고, 생성 경로의 간격과 곡률을 검증한다.

```yaml
route:
  smooth_checkpoint_segments: [2]
  smooth_checkpoint_handle_m: 8.0
```

Ego가 `PosAbsolute`이면 경로 끝점을 XML만으로 알 수 없으므로 XML만 복사하고
기존 `checkpoints.csv`, `route.csv`, `route.svg`는 보존한다. 이 경우도 실패로
처리하려면 `--require-route`를 사용한다.

## Ego를 이동해서 중간 구간 디버깅

PosAbsolute 패키지에서 다음 옵션을 켜면 실행 전 동기화할 때마다 XML의
Ego 위치에 가까운 경로점부터 `route.csv`/`route.svg`를 다시 생성한다.
체크포인트 원본은 보존하고, 전체 경로에서 이미 지난 진출 체크포인트를
계산해 `runtime.mission_start_leg`도 자동 갱신한다(0부터 시작).
러너는 이 값을 제어기의 `--mission-start-leg`로 전달한다.
Ego를 원래 출발점으로 돌려놓으면 시작 구간도 자동으로 0으로 돌아간다.

```yaml
route:
  preserve_checkpoint_coordinates: true
  trim_route_to_ego_start: true
  trim_max_distance_m: 8.0
validation:
  start_position_tolerance_m: 8.0
  start_heading_tolerance_deg: 10.0
```

M13에는 이 설정이 적용되어 있다. VTD Editor에서 Ego 위치/방향을 변경해
원본 XML을 저장하고 기존 `run_scenario.py scenario/mixed/M13 --auto-vtd`
명령을 사용하면 된다. `mission_start_leg`를 직접 편집할 필요는 없다.
경로에서 8m를 넘게 벗어나거나 진행방향이 맞지 않으면 검증은 계속 실패한다.
기존 주행 로그 보존을 위해 재실행 시 `--run-id`를 생략하거나 새 값을 사용한다.

## 주의사항

- VTD XML의 `TrackId`와 저장소 XODR의 road id가 다른 맵에서는 변환 테이블이
  필요하며 현재 도구는 명확한 오류로 중단한다.
- `StartLane`이 해당 road/s에서 driving lane이 아니면 자동 추측하지 않고
  중단한다.
- 감시 도구는 원격 저장소에 자동 push하지 않는다. 생성 결과를 검토한 뒤
  Git 작업은 사람이 수행한다.
