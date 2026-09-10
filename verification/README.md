# M 시나리오 구성 목록

프로젝트 루트에서 실행:

```powershell
python tools/verification/extract_scenario_events.py
```

`scenario/scenario_catalog.csv`의 `M숫자2자리` ID와 `package_path`를 읽고,
각 패키지의 XML을 파싱해 `verification/m_scenario_registry.csv`를 생성한다.
추가 라이브러리는 필요 없다. CSV는 Excel에서도 체크 표시를 읽을 수 있는
UTF-8 BOM 형식이며 `✓`는 포함, 빈칸은 해당 정의가 없음을 뜻한다.
기본 경로는 실행 위치와 무관하다. `--catalog`, `--output`, `--ego-name`으로
경로와 제외할 Ego 이름을 지정할 수 있다.

| 열 | XML 판별 기준 |
|---|---|
| Vehicle | `TrafficControl/Player/Description` 중 이름이 `Ego`가 아닌 차량. 정차 차량·이륜차·external 차량도 포함 |
| Pedestrian | `MovingObjectsControl/Character`의 `Class="pedestrian"` |
| Static obstacle | `MovingObjectsControl/Object` 중 연결된 `ObjectActions`에 이동 동작(`Motion`, `ObjectPath`, `Path`, `SpeedChange`, `LaneChange`)이 없는 객체. `Type="sign"`으로 저장된 콘·타이어 등도 포함. 정차 차량은 Vehicle로 집계 |
| Lane change | `Action` 바로 아래 `LaneChange` |
| Speed change | `Action` 바로 아래 `SpeedChange`. 초기 `Init/Speed`와 보행자 `Motion Speed`는 제외 |
| Traffic light | `LightSigns` 안에 `SignalController` 또는 `Signal` 정의가 존재 |
| Trigger | `Action` 바로 아래 `PosRelative` 또는 `PosAbsolute` 조건. 초기 위치·경로 좌표·`EditorPos`는 제외 |

이 표는 현재 VTD native XML의 **구성 유무**를 정리한다. 이벤트의 실제 발동,
경로상의 장애물 접촉·회피 또는 신호등 조우 여부를 검증한 결과는 아니다.
외부 SCP 명령 문자열은 해석하지 않는다. 새로운 이벤트 형식을 도입하면
판별 규칙도 함께 확장해야 한다. `VehicleList`, `CharacterList`, `ObjectList`
같은 리소스 목록 선언과 빈 `ObjectActions`는 객체·이벤트로 세지 않는다.

카탈로그 미등록 M01~M18 번호는 경고로 알리고 행을 만들지 않는다.
등록된 패키지의 XML이 없거나 여러 개이거나 파싱에 실패하면 오류로 종료하고
기존 CSV를 유지한다. 표 수정이 필요하면 원본 XML 또는 판별 규칙을 수정한 뒤
재실행한다. CSV에 직접 쓴 값은 재실행하면 덮어쓴다.

## 현재 XML 대조 결과

- 현재 카탈로그의 M01~M18 전체 18개를 포함한다.
- M02에는 차선 변경이 있지만 `SpeedChange`가 없다.
- M05에는 보행자와 차량이 있지만 정적 `Object`와 `LaneChange`, `SpeedChange`는 없다.
- M14에는 정적 객체와 차량이 있지만 보행자가 없다.
- M15~M17에는 7개 항목이 모두 포함돼 있다.
- M18에는 차량·보행자·정적 객체·속도 변경·신호등·트리거가 있고 `LaneChange`는 없다.
- M03과 M15를 포함한 모든 XML에 신호등 설정이 있으므로 Traffic light는 모두 체크된다.

검증 실행:

```powershell
python -m unittest discover -s tools/verification -p test_extract_scenario_events.py
```

## 수정 전·후 Regression Health 비교

기존 실행 결과 두 개를 비교한다. VTD 실행이나 공식 평가 조회 없이
Runner의 summary.csv와 시나리오별 run.csv만 읽는다.

```powershell
python tools/verification/compare_regression.py --baseline REG_20260910_001 --current REG_20260910_002
```

입력은 output/verification/regression/<Campaign ID>/summary.csv와
output/verification/<Scenario ID>/<Campaign ID>/run.csv다.
출력은 Current의 regression_summary.csv 한 개이며 두 Campaign ID도 기록한다.
Baseline과 run.csv 원본은 수정하지 않고 기존 비교 보고서만 교체한다.
다른 위치의 결과는 --output-root <verification 디렉터리>로 지정한다.

| 판정 | 조건 |
|---|---|
| REGRESSION | COMPLETE/exit 0 → 실행 실패·비정상 종료, 기존 run.csv 또는 시나리오 행 누락, goal_reached true → false 중 하나라도 존재. 다른 조건보다 우선 |
| INCONCLUSIVE | 명확한 악화 없이 Baseline 자체가 실패했거나, 완료·exit code·goal 데이터 부족, 또는 Current에만 새 시나리오 존재 |
| IMPROVED | 양쪽 COMPLETE/exit 0, 유효한 goal_reached false → true |
| SAME | 양쪽 COMPLETE/exit 0, 유효한 goal_reached 동일. 둘 다 false여도 SAME이며 목표 도달 성공을 뜻하지 않음 |

goal_reached는 실제 Logger의 컬럼이며 유효한 로그에서 한 프레임이라도 1이면
true다. 0/1과 false/true 값을 읽는다. 파일 누락·빈 파일·헤더만 있는 로그·컬럼
누락·잘린 행·잘못된 값·scenario/run ID 불일치는 false로 추정하지 않고
UNAVAILABLE로 표시한다. 진행도·공식 점수·위반 횟수·TTC·jerk는 비교하지 않는다.
현재 작업 폴더에는 실제 Campaign/run.csv가 없으므로 기존 저장 코드와 Logger
스키마를 따른 임시 fixture로 검증했다.

CSV 예시(주요 열 발췌):

```csv
scenario_id,baseline_execution_status,current_execution_status,baseline_goal_reached,current_goal_reached,result,reason
M01,COMPLETE,COMPLETE,true,true,SAME,
M02,COMPLETE,TIMEOUT,true,true,REGRESSION,execution COMPLETE -> TIMEOUT (exit 0 -> 2)
M03,COMPLETE,COMPLETE,false,true,IMPROVED,goal_reached false -> true
```

종료 코드: 0은 SAME/IMPROVED만 있음, 1은 REGRESSION 존재, 2는 입력·출력 오류,
3은 REGRESSION 없이 INCONCLUSIVE 존재.
미완성 실행의 summary 행이 없으면 실행 중·의도적 생략을 구별할 수 없으므로
두 Campaign이 종료된 후 비교한다.

```powershell
python -m unittest discover -s tools/verification -p test_compare_regression.py
```
