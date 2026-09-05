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

Ego가 `PosAbsolute`이면 경로 끝점을 XML만으로 알 수 없으므로 XML만 복사하고
기존 `checkpoints.csv`, `route.csv`, `route.svg`는 보존한다. 이 경우도 실패로
처리하려면 `--require-route`를 사용한다.

## 주의사항

- VTD XML의 `TrackId`와 저장소 XODR의 road id가 다른 맵에서는 변환 테이블이
  필요하며 현재 도구는 명확한 오류로 중단한다.
- `StartLane`이 해당 road/s에서 driving lane이 아니면 자동 추측하지 않고
  중단한다.
- 감시 도구는 원격 저장소에 자동 push하지 않는다. 생성 결과를 검토한 뒤
  Git 작업은 사람이 수행한다.
