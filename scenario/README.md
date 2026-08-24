# HL FMA Scenario Design

## 1. 목적

HL FMA 설명회 자료에서 도로교통법 기반 10개 법규 평가항목과
보행자·차량·장애물·방향지시등·충돌 관련 동적 이벤트 대응 항목이
확인되었다.

다만 각 항목의 세부 판정 Threshold, 감점 기준과 배점은 현재 자료만으로
모두 확정할 수 없다.

따라서 Scenario는 현재 확인된 평가항목과 우선 Mapping하여 구성하고,
세부 평가기준이 추가 공개되면 Parameter와 PASS/FAIL 기준을 보완한다.
기존 개발·검증용 Scenario는 폐기하지 않고 평가 대응 Scenario의 Variation
또는 후속 개발 Scenario로 재사용한다.

## 2. 설계 원칙

Scenario는 다음 순서로 확장한다.

```text
Level 0  정상 주행
   ↓
Level 1  단일 위험상황
   ↓
Level 2  속도·거리·Trigger 등을 변화시킨 경계조건
   ↓
Level 3  두 개 이상의 상황을 결합한 복합상황
```

처음부터 모든 Scenario를 대량 제작하지 않는다.

**대표 Scenario를 먼저 구현한 뒤 Parameter / Trigger를 변경하여 여러 Variation Case를 만드는 방식**으로 진행한다.

예:

```text
V03 : 앞차 급정거
 ├─ V03-01 : Nominal Speed + Far Gap
 ├─ V03-02 : Nominal Speed + Near Gap
 ├─ V03-03 : High Speed + Near Gap
 └─ V03-04 : High Speed + Near Gap + Late Trigger
```

### Scenario 재사용 원칙

Scenario의 재사용 대상은 단순히 동일한 XML 파일 자체가 아니라
`테스트 목적 + Event Logic + Parameter 구조`다.

VTD Scenario Editor 자료에서 다음 기능을 확인했다.

1. Player의 Position Type으로 Relative coordinates를 사용할 수 있다.
2. 기존 Player를 Pivot으로 지정할 수 있다.
3. Relative Player는 Pivot 기준 앞·뒤 거리, 상대 Lane, Lane Offset과
   진행방향을 지정할 수 있다.
4. Relative Trigger는 selected player와 Pivot player 사이의 거리가
   Activation Radius보다 작아질 때 발동한다.
5. TTC Relative Trigger도 제공된다.

가능한 경우 Relative Position과 Relative Trigger를 이용하여 절대 World
Coordinate 의존성을 줄인다.

예:

```text
V03 : Lead Vehicle Sudden Brake

NPC Initial Position
- Pivot = Ego
- Ego 기준 일정 거리 앞
- Same Lane
- Same Direction

Trigger
- Ego와 NPC 사이 거리가 설정값 이하

Action
- NPC Speed Change / Deceleration
```

이 구조는 NPC 위치와 Trigger를 특정 x/y에 고정하는 방식보다 다른 호환
Zone에서 동일한 테스트 로직을 재사용하기 쉽다.

단, 모든 Scenario를 모든 Zone에서 사용할 수 있는 것은 아니다. Traffic
Light, Intersection, Lane Change처럼 도로 구조에 종속되는 Scenario는
Compatible Zone별 설정이 필요하다.

## 3. 설명회에서 확인된 평가항목

### 3.1 도로교통법 기반 평가항목

| 평가 코드 | 평가 항목 |
|---|---|
| `S1.1.01` | 제한속도 준수 |
| `S1.1.02` | 보호구역(스쿨존) 속도 준수 |
| `S2.1.01` | 차로 유지 |
| `S2.1.02` | 중앙선 침범·우측통행 |
| `S2.1.03` | 보도 침범 금지 |
| `S2.2.05` | 실선 차로변경 금지 |
| `S5.1.01` | 적색신호 정지 |
| `S5.1.03` | 녹색신호 통과 |
| `S6.1.01` | 도로 파손·장애물 대응 |
| `S6.3.03` | 횡단보도 정차 금지 |

### 3.2 동적 이벤트와 상시 로그 평가

설명회에서 보행자 출현, 차량 출현·급정거, 장애물 출현, 방향지시등과
충돌 관련 대응 항목도 확인되었다.

```text
보행자 출현
- 거리, 감속, TTC, 정지 여부

차량 출현 / 급정거
- 안전거리 확보, 적절한 대응

장애물 출현
- 감속, 정지, 회피

방향지시등
- 회전 / 차로변경 전 점등 시점

충돌
- 발생 시 감점
```

차로 유지, 중앙선 침범, 보도 침범, 실선 침범과 우측통행은 특정 이벤트가
없어도 여러 Scenario의 주행 로그에서 상시 측정하는 항목으로 관리한다.
각 항목마다 독립 XML을 반드시 하나씩 만드는 구조로 제한하지 않는다.

## 4. Core Scenario 재구성

기존 18개 Scenario를 전부 폐기하지 않는다. 공식 평가에 직접 대응하는
Scenario를 Core로 올리고, 기존 개발용 세부 Case는 Variation 또는 후속
개발 Scenario로 재사용한다.

### 4.1 Basic / Road Rule

| ID | 상황 | 대응 평가·검증 목적 |
|---|---|---|
| B01 | 직선 차로 유지 | `S2.1.01`, 기본 경로추종과 상시 법규 로그 |
| B02 | 제한속도 준수 | `S1.1.01`, 현재 제한속도에 맞춘 주행 |
| B03 | 커브 차로 유지 | `S2.1.01`, 중앙선·보도 침범도 상시 확인 |
| B04 | 보호구역(스쿨존) 속도 준수 | `S1.1.02` |
| B05 | 교차로 / 우측통행 | `S2.1.02`, Junction 경로추종과 우측통행 |
| B06 | 차로변경 + 방향지시등 | 차로변경 전 방향지시등 점등 시점 |
| B07 | 실선 차로변경 금지 | `S2.2.05`, 실선 구간의 불필요한 Lane Change 방지 |

`S2.1.02` 중앙선 침범과 `S2.1.03` 보도 침범은 B01, B03, B05를
포함한 여러 Scenario에서 상시 로그 평가 항목으로 함께 확인한다.

### 4.2 Traffic

| ID | 상황 | 대응 평가·검증 목적 |
|---|---|---|
| T01 | 녹색신호 통과 | `S5.1.03` |
| T02 | 적색신호 정지 | `S5.1.01` |
| T03 | 가까운 거리에서 적색신호 | 적색신호 대응의 제동 경계조건 |
| T04 | 적색 정지 후 녹색 재출발 | 정지 후 정상 재출발 |
| T05 | 횡단보도 정차 금지 | `S6.3.03`, 적색신호·정체·선행차 조건에서 횡단보도 위 정차 방지 |

### 4.3 Vehicle

| ID | 상황 | 대응 평가·검증 목적 |
|---|---|---|
| V01 | 선행차 안전거리 유지 | 차량 출현과 안전거리 확보 |
| V02 | 정지차량 대응 | 정지차량 발견 후 감속·정지 |
| V03 | 선행차 급정거 | 차량 급정거에 대한 긴급 종방향 대응 |
| V04 | Cut-in 대응 | 옆 차선 차량 진입 대응 |
| V05 | 교차로 측면차량 대응 | 측면 충돌위험 대응 |

동일속도 선행차와 저속 선행차는 삭제하지 않고 V01의 Variation 또는
후속 개발 Scenario로 관리한다.

### 4.4 Pedestrian

`pedestrian/`은 2차 후보가 아니라 설명회의 보행자 출현 평가에 대응하는
Core Scenario 범주다.

| ID | 상황 | 대응 평가·검증 목적 |
|---|---|---|
| P01 | 충분한 거리에서 보행자 횡단 | 정상 감속·정지 |
| P02 | 가까운 거리에서 보행자 출현 | Distance·TTC·정지 여부의 긴급 대응 경계조건 |

`P03 신호 교차로 + 보행자`는 단일 Scenario 검증 이후의 후속 후보로 둔다.

### 4.5 Obstacle

`obstacle/`도 `S6.1.01`에 대응하는 Core Scenario 범주다.

| ID | 상황 | 대응 평가·검증 목적 |
|---|---|---|
| O01 | 차로 전방 장애물 / 도로 파손 대응 | 감속·정지 또는 안전 회피 |
| O02 | 가까운 거리에서 장애물 출현 | 긴급 대응 경계조건 |

기존 O01~O03 개발 후보의 세부 배치는 필요하면 위 Core의 Variation으로
재사용한다.

### 4.6 Mixed

| ID | 상황 | 주요 검증 목적 |
|---|---|---|
| M01 | 신호 + 교차차량 충돌위험 | 신호와 측면차량 복합 판단 |
| M02 | 적색신호 + 정지 선행차 | 신호와 선행차 동시 판단 |
| M03 | 커브 + 선행차 급정거 | 횡·종방향 제어 동시 안정성 |

Mixed는 단일 평가항목 대응 Scenario가 동작한 뒤 복합상황과 Regression
검증에 사용한다. M01은 HL FMA 교육자료의 Case 201과 유사한 개발 검증
Scenario로 볼 수 있다.

### 4.7 기존 Scenario 역할 정리

| 기존 Scenario | 재구성 결과 |
|---|---|
| B01 직선 정상주행 | B01 직선 차로 유지로 목적 구체화 |
| B02 고속 직선주행 | B02 제한속도 준수로 변경, 고속 안정성은 Variation |
| B03 완만한 커브 | B03 커브 차로 유지로 유지 |
| B04 고곡률 커브 | B03 Variation 또는 후속 개발 Scenario |
| B05 교차로 단독 통과 | B05 교차로 / 우측통행으로 확장 |
| V01 동일속도·V02 저속 선행차 | V01 안전거리 유지의 Variation |
| V03 정지차량 | V02 정지차량 대응으로 재배치 |
| V04 선행차 급감속 | V03 선행차 급정거로 재배치 |
| V05 Cut-in | V04 Cut-in 대응으로 재배치 |
| V06 교차로 측면차량 | V05 교차로 측면차량 대응으로 재배치 |
| M01~M03 | 복합상황 / Regression 용도로 유지 |

### 4.8 구현 우선순위

1차 구현 우선 대상은 평가항목 대응 중심의 다음 8개다.

| ID | 상황 | 선택 이유 |
|---|---|---|
| B01 | 차로 유지 | 기본 주행과 상시 법규 로그 |
| B02 | 제한속도 준수 | `S1.1.01` 직접 대응 |
| B04 | 스쿨존 속도 준수 | `S1.1.02` 직접 대응 |
| T01 | 녹색신호 통과 | `S5.1.03` 직접 대응 |
| T02 | 적색신호 정지 | `S5.1.01` 직접 대응 |
| V03 | 선행차 급정거 | 차량 급정거 동적 이벤트 |
| P01 | 보행자 출현 | 보행자 동적 이벤트 |
| O01 | 장애물 / 도로 파손 대응 | `S6.1.01` 직접 대응 |

다음 단계에서는 B06, B07, T05와 M01을 추가한다.

```text
1차 : 평가항목 직접 대응 Scenario 8개
   ↓
2차 : 방향지시등·실선·횡단보도와 복합위험 Scenario
   ↓
3차 : 기존 개발 Scenario와 각 Core의 Variation 확대
   ↓
4차 : 추가 공개되는 공식 Threshold와 판정 기준 반영
```

## 5. Case Variation 규칙

각 Scenario는 별도의 완전히 새로운 파일을 계속 만드는 방식보다, 기본 구조를 유지하면서 조건을 변화시키는 방식으로 확장한다.

기존 Variation 축:

```text
- Ego Speed
- Object Distance / Initial Gap
- Object Speed
- Relative Speed
- Trigger Timing
- TTC
- Road Geometry
- Scenario Location
```

### 위치 독립적으로 변경 가능한 Parameter

- Ego Speed
- Object Initial Gap
- Object Speed
- Relative Speed
- Relative Trigger Distance
- Trigger Timing
- TTC / TTC Relative 조건

### 맵 구조에 영향을 받는 Parameter

- Road Geometry
- Scenario Location
- Lane Count
- Intersection Structure
- Traffic Light 존재 여부
- NPC Route

가능한 경우 절대 x/y Trigger보다 Relative Trigger를 우선 검토한다.
다만 실제 Scenario 목적에 Absolute Trigger 또는 Track Trigger가 더 적절한
경우에는 해당 방식을 사용한다.

숫자 값은 현재 임의로 대회 기준처럼 고정하지 않는다.

판단/제어 알고리즘의 Threshold가 정해지면 그 주변을 중심으로 경계조건을 테스트한다.

예:

```text
TTC Threshold = T

Case A : TTC = T + Δ
Case B : TTC = T
Case C : TTC = T - Δ
```

목적은 단순 PASS/FAIL 확인이 아니라 **제어기가 어느 조건에서 실패하기 시작하는지 찾는 것**이다.

## 6. Scenario Zone

Zone은 전역경로 자체가 아니라 특정 종류의 Scenario를 검증하기 적합한 실제
맵 구간을 의미한다.

후보 Zone:

```text
ZONE_STRAIGHT
ZONE_CURVE_LOW
ZONE_CURVE_HIGH
ZONE_INTERSECTION
ZONE_INTERSECTION_CONFLICT
ZONE_LANE_CHANGE
```

Scenario와 Zone의 관계는 `모든 Scenario × 모든 Zone`이 아니라
`Scenario × Compatible Zone`으로 관리한다.

| Scenario | Compatible Zone 예 |
|---|---|
| B01 | Straight Zone |
| B03 | Curve Zone |
| T02 | Traffic Light가 존재하는 Intersection Zone |
| B07 | Solid-Line / Lane-Change 관련 Zone |
| V03 | Straight / 일부 Curve Zone |
| V04 | 다차로 및 Lane Change가 가능한 Zone |
| P01 | Pedestrian Event를 구성할 수 있는 Zone |
| O01 | Obstacle Event를 구성할 수 있는 Zone |
| M01 | Conflict 구성이 가능한 Signal Intersection Zone |

### Global Reference Path와 Scenario 관계

Ego가 실제로 따라가는 Global Reference Path는 Planning에서 별도로
로드한다. Scenario 파일과 Global Reference Path를 하나의 파일로 합치는
구조를 기본으로 하지 않는다.

실제 검증은 다음처럼 구성한다.

```text
VTD Scenario
- Ego / NPC 초기조건
- Traffic Light
- Object
- Trigger
- Action

+

Global Reference Path
- Planning이 사용하는 Ego 주행 경로

↓

동시에 실행하여 판단 / 제어 검증
```

Scenario를 제작할 때는 해당 Scenario와 호환되는 Zone을 지나가는 Global
Reference Path를 사용한다.

```text
Scenario
= 어떤 상황을 시험할 것인가

Zone
= 맵의 어느 구간에서 시험할 것인가

Global Reference Path
= Ego가 실제로 어느 경로를 따라 해당 Zone을 통과할 것인가
```

### Relative Scenario 예시: V03

V03 Lead Vehicle Sudden Brake의 가능한 구성 예:

1. Ego가 해당 Zone을 통과하는 Global Reference Path를 사용한다.
2. NPC를 Ego 기준 상대 위치로 배치한다.
3. NPC는 Ego와 동일 차선·동일 방향으로 설정한다.
4. Ego-NPC 간격을 Initial Gap Parameter로 설정한다.
5. Relative Trigger로 일정 거리 이하에서 Action을 발생시킨다.
6. NPC Speed Change로 급감속 Event를 실행한다.

Variation 예:

```text
V03-01
- Normal Ego Speed
- Far Initial Gap
- Early Trigger

V03-02
- Normal Ego Speed
- Near Initial Gap
- Early Trigger

V03-03
- High Ego Speed
- Near Initial Gap
- Late Trigger
```

같은 Event Logic을 유지하면서 Parameter와 Compatible Zone을 변경하여
제어기의 실패 경계조건을 확인한다.

### 위치 독립성 관련 주의사항

Relative Position과 Relative Trigger를 사용한다고 해서 모든 Scenario가
위치와 완전히 독립적인 것은 아니다.

다음 Scenario는 맵 구조의 영향을 크게 받는다.

- Traffic Light Scenario
- Intersection Conflict Scenario
- Cut-in Scenario
- 특정 Lane 수가 필요한 Scenario
- 특정 NPC Route가 필요한 Scenario

이러한 Scenario는 Compatible Zone을 먼저 선정하고, 필요한 경우 해당 Zone에
맞게 위치, Lane, Route와 Trigger 설정을 조정한다.

동일 Scenario 파일을 모든 Zone에 그대로 재사용한다는 의미로 해석하지 않는다.
Scenario의 Event Logic과 Parameter 구조를 재사용하고, Relative Position과
Relative Trigger를 활용할 수 있는 Scenario는 위치 의존성을 낮추며, 도로
구조 종속 Scenario는 Compatible Zone별로 필요한 설정을 조정한다.

```text
Global Reference Path
      +
Compatible Zone
      +
V03 Event Logic
      ↓
Relative Position / Trigger 활용
      ↓
실제 VTD Test Case
```

## 7. 설명회 평가항목 Mapping

### 7.1 도로교통법 기반 항목

| 평가 코드 | 평가 항목 | 대응 Scenario | 평가 방식 |
|---|---|---|---|
| `S1.1.01` | 제한속도 준수 | B02 | Scenario + 주행 로그 |
| `S1.1.02` | 스쿨존 속도 준수 | B04 | Scenario + 주행 로그 |
| `S2.1.01` | 차로 유지 | B01/B03 등 | 상시 로그 |
| `S2.1.02` | 중앙선 침범·우측통행 | B03/B05 등 | 상시 로그 |
| `S2.1.03` | 보도 침범 금지 | B03/B05 등 | 상시 로그 |
| `S2.2.05` | 실선 차로변경 금지 | B07 | Scenario + 상시 로그 |
| `S5.1.01` | 적색신호 정지 | T02/T03 | Scenario |
| `S5.1.03` | 녹색신호 통과 | T01 | Scenario |
| `S6.1.01` | 도로 파손·장애물 대응 | O01/O02 | Scenario |
| `S6.3.03` | 횡단보도 정차 금지 | T05 | Scenario |

### 7.2 동적 이벤트

| 동적 이벤트 | 대응 Scenario |
|---|---|
| 보행자 출현 | P01/P02 |
| 차량 출현·급정거 | V01/V02/V03 |
| 장애물 출현 | O01/O02 |
| 방향지시등 | B06 |
| 충돌 | 모든 위험 Scenario의 공통 Result Metric |

이 Mapping은 어떤 상황을 어떤 Scenario로 검증할지를 정한 것이다.
세부 Threshold, 감점 기준과 배점까지 확정한 것은 아니다.

## 8. Scenario와 Logging / Verification 연결

Scenario를 설계할 때는 해당 Case에서 무엇을 확인할지도 함께 정의한다.

기존 공통 Logging 후보는 다음과 같다.

```text
Ego
- x, y, heading
- speed

Object
- id
- x, y
- heading
- speed

Risk
- distance
- relative speed
- TTC

Signal
- traffic light id/state

Planning / Control
- behavior / decision
- target speed
- steering
- targetAccel
- turnSignal

Result
- minimum distance
- minimum TTC
- goal 도달 여부
- collision 여부
```

### 8.1 상시 법규 평가를 위해 향후 필요한 데이터

설명회 평가항목을 검증하려면 다음 데이터가 추가로 필요하다.

```text
Road / Lane
- current road / lane
- lane center 대비 lateral error
- lane boundary / road mark
- centerline crossing 여부
- sidewalk intrusion 여부
- solid line crossing 여부
- right-side driving 여부

Speed Rule
- current ego speed
- current speed limit
- school zone 여부

Traffic / Stop
- traffic light state
- stop line 위치 / 통과 여부
- crosswalk 위치
- crosswalk 정차 여부

Turn Signal
- turnSignal
- lane change / turn 시작 시점
- turnSignal ON 시점

Dynamic Safety
- relevant object
- distance / gap
- relative speed
- TTC
- collision
```

이 목록은 향후 평가에 필요한 데이터 정의다. 일부 값은 VTD Interface,
XODR, Planning 출력과 팀 합의가 필요하므로 이번 작업에서 Logger 필드나
계산식을 즉시 확정하지 않는다.

### 8.2 Logger 보류 항목

VTD 또는 참가자 Interface에서 실제 제공 여부를 먼저 확인할 값:

```text
- Simulation Time / VTD Frame
- Object Valid Count / Flag
- Object Type
- Collision / Contact
- Ego Speed 직접 제공 여부
```

우리 코드에서 정의하거나 계산할 값:

```text
- Relevant Object Selection
- Distance / Gap
- Relative Speed
- Closing Speed 필요 여부와 정의
- TTC 정의
```

객체 선택 기준, 거리 기준, 부호 규칙과 유효하지 않은 TTC 표현 방식이
정해져야 각 Scenario의 결과를 일관되게 비교할 수 있다.

### 8.3 Collision 처리 원칙

Collision은 특정 Scenario 하나가 아니라 모든 위험 Scenario의 공통 Result
Metric으로 관리한다. P01, P02, V02, V03, V04, O01, O02, M01 등을
평가할 때 공식 Collision 값이 확보되면 `collision == false`를 중요한
성공 조건으로 사용할 수 있다.

현재 참가자 Interface에서 공식 Collision / Contact 값이 확인되지 않았으므로
`distance < 임계값`을 실제 Collision으로 임의 확정하지 않는다. VTD에서
공식 정보를 받을 수 있는지 먼저 확인한다.

### 8.4 PASS / FAIL 적용 원칙

현재 `tools/verification/evaluate.py`의 다음 구조는 유지한다.

```text
명시된 기준 없음
→ NOT_EVALUATED

명시된 기준 모두 만족
→ PASS

하나 이상의 기준 위반
→ FAIL
```

설명회 평가항목이 확인되었더라도 공식 판정 방식이 공개되기 전에는 TTC,
최소거리, 정지선 허용오차와 방향지시등 선행시간 등의 숫자를 임의로
기본값에 넣지 않는다. 현재는 Scenario와 평가항목 Mapping을 먼저 확정하고,
필요 데이터와 공식 기준이 확보된 뒤 Scenario별 판정 규칙을 추가한다.

## 9. Scenario Naming Rule

기본 Scenario ID:

```text
B01, B02 ...
T01, T02 ...
V01, V02 ...
M01, M02 ...
P01, P02 ...
O01, O02 ...
```

Variation은 뒤에 번호를 추가한다.

```text
V03-01
V03-02
V03-03
```

예:

```text
V03      = 선행차 급정거 Scenario Template
V03-01   = Nominal Speed + Far Gap
V03-02   = Nominal Speed + Near Gap
V03-03   = High Speed + Near Gap
```

## 10. Repository 적용

현재 Scenario 설계 문서는 다음 위치에서 관리한다.

```text
scenario/README.md
```

현재 Scenario 폴더 구조는 검증 목적에 따라 다음과 같이 구분한다.

```text
scenario/
├── README.md
├── scenario_catalog.csv
├── basic/
├── traffic/
├── vehicle/
├── mixed/
├── pedestrian/
└── obstacle/
```

`basic/`은 외부 이벤트가 거의 없는 경로추종과 기본 제어를 검증한다.
`traffic/`은 Traffic Light state에 따른 통과, 정지와 재출발 판단 및
종방향 제어를 검증한다.

교차로가 포함되어도 핵심 검증 목적이 junction 경로추종이면 `basic/`에,
신호 상태에 따른 판단과 제어이면 `traffic/`에 둔다.

```text
B05
→ basic/: 신호 판단이 핵심이 아닌 교차로 경로추종

T01 ~ T05
→ traffic/: 신호 상태에 따른 통과, 정지와 재출발
```

`traffic/` 분리는 HL FMA 공식 폴더 구조가 아니라, 단위 기능별 실패 원인을
명확하게 관리하기 위한 팀 Repository 설계다.

실제 구현 현황과 Variation은 다음 파일에서 관리한다.

```text
scenario/scenario_catalog.csv
```

역할:

```text
scenario/README.md
→ Scenario 설계 원칙 / 분류 / Case / Variation 기준

scenario_catalog.csv
→ Scenario·Variation의 구현 상태와 평가 코드 Mapping 관리
```

Catalog의 현재 스키마:

```text
scenario_id
category
scenario_file
map_area
eval_codes
test_purpose
ego_speed_mps
object_condition
expected_behavior
status
notes
```

하나의 Scenario가 여러 평가항목과 연결될 수 있으므로 `eval_codes`에는
세미콜론으로 구분한 복수 코드를 기록할 수 있다. 현재 실제 VTD Scenario
파일은 없으므로 Catalog 행은 구조 확인용 `planned` 예시이며, 실제
제작된 Case로 해석하지 않는다.

검증 결과는 기존 구조대로 `output/verification/` 아래에 저장한다.

```text
output/verification/
├── <scenario_id>/
│   ├── run.csv
│   ├── summary.csv
│   └── plot.png
└── overall_summary.csv
```

`run.csv`는 Logger의 프레임별 Raw Log, `summary.csv`는 단일 Run
평가 결과, `plot.png`는 선택적인 시각화 결과,
`overall_summary.csv`는 여러 Run의 통합 결과다. 이 목적을 위해
`output/path/` 같은 새 중간 구조를 만들지 않는다.

## 11. 현재 확정된 내용과 보류된 내용

현재 확정된 내용:

- 설명회에서 확인된 평가항목과 Scenario의 Mapping
- Pedestrian과 Obstacle의 Core 평가 대응 범주 승격
- 1차 구현 우선순위
- Scenario × Compatible Zone
- Global Reference Path와 VTD Scenario의 역할 분리
- Relative Position / Relative Trigger 우선 검토 원칙
- Collision을 모든 위험 Scenario의 공통 Result Metric으로 관리
- Catalog의 `eval_codes` 연결 구조

추가 확인 또는 팀 합의가 필요한 내용:

- 평가항목별 공식 Threshold, 감점 기준과 배점
- VTD의 Simulation Time / Frame과 Object Valid / Type 제공 방식
- 공식 Collision / Contact 정보 제공 여부
- Ego Speed의 직접 제공 여부
- Relevant Object, Distance / Gap, Relative Speed, Closing Speed와 TTC 정의
- 도로·차선·속도제한·정지선·횡단보도 정보를 Logger에 연결하는 Interface

실제 VTD Scenario XML과 대량 Catalog Case는 아직 만들지 않는다. 필요한
Interface와 판정 기준이 확정되면 Core Scenario를 구현하고 Catalog의
`planned` 상태를 실제 진행 상태로 갱신한다.
