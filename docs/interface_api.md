# VTD Interface API

본 문서는 VTD Host와 참가자 제어 프로그램 간의 통신 인터페이스를 정리한 문서입니다.

참가자 프로그램은 VTD에서 제공되는 **Ego Vehicle, Object, Traffic Light 상태 정보**를 수신하고, 이를 기반으로 Planning 및 Control을 수행한 뒤 **Steering, Target Acceleration, Turn Signal**을 VTD로 전송합니다.

---

## 1. Interface Overview

```text
                         VTD Host
                            │
                            │ Ego State
                            │ Object State
                            │ Traffic Light State
                            ▼
                    Participant Program
                            │
                            │ Planning
                            │ Control
                            ▼
                     Control Command
                            │
                            │ Steering
                            │ Target Acceleration
                            │ Turn Signal
                            ▼
                         VTD Host
```

### 데이터 방향 요약

| 방향        | 데이터                       |
| --------- | ------------------------- |
| VTD → 참가자 | Ego Vehicle State         |
| VTD → 참가자 | Object State (Array = 30) |
| VTD → 참가자 | Traffic Light State       |
| 참가자 → VTD | Steering                  |
| 참가자 → VTD | Target Acceleration       |
| 참가자 → VTD | Turn Signal               |

---

# 2. VTD → Participant

VTD에서 참가자 프로그램으로 전달되는 상태 정보입니다.

---

## 2.1 Ego Vehicle

Ego Vehicle의 현재 위치 및 자세 정보를 제공합니다.

| 항목              | Field        |  단위 | Type    |   Size |
| --------------- | ------------ | --: | ------- | -----: |
| Ego X 좌표        | `egoX`       |   m | `float` | 32 bit |
| Ego Y 좌표        | `egoY`       |   m | `float` | 32 bit |
| Ego Z 좌표        | `egoZ`       |   m | `float` | 32 bit |
| Ego 요각(Heading) | `egoHeading` | rad | `float` | 32 bit |
| Ego Pitch       | `egoPitch`   | rad | `float` | 32 bit |
| Ego Roll        | `egoRoll`    | rad | `float` | 32 bit |

### 의미

```text
egoX, egoY, egoZ
→ Ego Vehicle의 위치

egoHeading
→ 차량이 바라보는 방향(Yaw)

egoPitch
→ 차량의 앞뒤 기울기

egoRoll
→ 차량의 좌우 기울기
```

Ego State 전체 필드 크기를 단순 합산하면:

```text
6 × 32 bit
= 192 bit
= 24 Byte
```

> 주의: 위 값은 각 필드 크기를 단순 합산한 값입니다.
> 실제 네트워크 Packet 전체 크기는 Header, Padding, Packing 방식 등에 따라 달라질 수 있습니다.

---

## 2.2 Object

주변 객체 정보는 최대 **30개의 Object Array** 형태로 제공됩니다.

```text
objects[0]
objects[1]
...
objects[29]
```

각 객체는 다음 정보를 포함합니다.

| 항목         | Field               |  단위 | Type       |   Size |
| ---------- | ------------------- | --: | ---------- | -----: |
| Object ID  | `objects[].id`      |   - | `uint32_t` | 32 bit |
| X 좌표       | `objects[].x`       |   m | `float`    | 32 bit |
| Y 좌표       | `objects[].y`       |   m | `float`    | 32 bit |
| Z 좌표       | `objects[].z`       |   m | `float`    | 32 bit |
| 객체 Heading | `objects[].heading` | rad | `float`    | 32 bit |
| 객체 속도      | `objects[].speed`   | m/s | `float`    | 32 bit |
| 객체 길이      | `objects[].length`  |   m | `float`    | 32 bit |
| 객체 폭       | `objects[].width`   |   m | `float`    | 32 bit |
| 객체 높이      | `objects[].height`  |   m | `float`    | 32 bit |

### Object 정보 활용

Planning에서는 Object 데이터를 이용해 다음과 같은 정보를 계산할 수 있습니다.

```text
Object Global Position
        ↓
Ego와의 상대 위치
        ↓
상대 거리 / 상대 속도
        ↓
충돌 위험도 / TTC
        ↓
감속 / 정지 / 회피 여부 결정
```

예:

```text
objects[i].x
objects[i].y
        ↓
Ego와 객체 사이 거리 계산

objects[i].speed
        ↓
상대 속도 계산

거리 + 상대 속도
        ↓
TTC(Time To Collision) 계산
```

### Object 1개의 데이터 크기

Object 하나에는:

```text
ID       : 32 bit
float × 8: 256 bit
```

따라서 단순 합산 기준:

```text
288 bit
= 36 Byte / Object
```

Object가 30개 모두 고정 전송된다고 가정한 단순 필드 합계는:

```text
36 Byte × 30
= 1080 Byte
```

> 실제 Packet이 항상 30개의 Object를 모두 포함하는지, 유효 객체 개수를 별도로 전달하는지는 현재 인터페이스 표만으로 확인할 수 없습니다.

---

## 2.3 Traffic Light

신호등의 ID와 현재 상태를 전달합니다.

| 항목     | Field                   | 단위 | Type    |   Size |
| ------ | ----------------------- | -: | ------- | -----: |
| 신호등 ID | `trafficLights[].id`    |  - | `int32` | 32 bit |
| 신호등 상태 | `trafficLights[].state` |  - | `uint8` |  8 bit |

### Traffic Light State

| State | 의미       |
| ----: | -------- |
|   `0` | 미할당      |
|   `1` | 적색       |
|   `2` | 황색       |
|   `3` | 녹색       |
|   `4` | 좌회전      |
|   `5` | 녹색 + 좌회전 |
|   `6` | 점멸       |

Planning에서는 일반적으로 다음과 같이 사용합니다.

```text
trafficLights[].id
        ↓
현재 차량과 관련된 신호등 식별
        ↓
trafficLights[].state 확인
        ↓
정지 / 출발 / 좌회전 등의 주행 판단
```

> `trafficLights[]`의 최대 개수는 현재 인터페이스 표에 명시되어 있지 않습니다.

---

# 3. Participant → VTD

참가자 프로그램에서 Planning 및 Control 결과를 VTD로 전달합니다.

---

## 3.1 Control Command

| 항목      | Field         |   단위 | Type     |   Size |
| ------- | ------------- | ---: | -------- | -----: |
| 조향각     | `steering`    |  rad | `float`  | 32 bit |
| 종방향 가속도 | `targetAccel` | m/s² | `float`  | 32 bit |
| 방향지시등   | `turnSignal`  |    - | `unit8`* |  8 bit |

* 원본 인터페이스 문서에는 `unit8`로 표기되어 있음.

### Steering

```text
steering [rad]
```

차량의 조향각 명령입니다.

횡방향 Controller에서 계산한 최종 조향 명령을 VTD로 전달합니다.

```text
Target Path
     ↓
Lateral Controller
     ↓
Steering
     ↓
VTD
```

---

### Target Acceleration

```text
targetAccel [m/s²]
```

차량의 **종방향 가속도 명령**입니다.

목표 속도 자체를 보내는 것이 아니라 가속도 값을 전달합니다.

```text
Target Speed
     ↓
Current Speed와 비교
     ↓
Longitudinal Controller
     ↓
targetAccel
```

이를 통해:

* 가속
* 정속 주행
* 감속
* 정지

등을 구현합니다.

---

### Turn Signal

```text
turnSignal
```

| Value | 의미       |
| ----: | -------- |
|   `0` | OFF      |
|   `1` | 좌측 방향지시등 |
|   `2` | 우측 방향지시등 |

예:

```cpp
turnSignal = 0;   // OFF
turnSignal = 1;   // LEFT
turnSignal = 2;   // RIGHT
```

---

## 3.2 Control Command Size

필드 크기만 단순 합산하면:

```text
steering    32 bit
targetAccel 32 bit
turnSignal   8 bit
-------------------
Total       72 bit
            9 Byte
```

> C/C++ Structure를 그대로 전송할 경우 Padding 때문에 실제 `sizeof()` 값은 9 Byte가 아닐 수 있습니다.
> 실제 Packet 구조와 Packing 규칙은 별도 확인이 필요합니다.

---

# 4. Network Interface

원본 인터페이스 문서에 정의된 Port 정보입니다.

|   Port | Protocol | 접속 방향 / 비고      |
| -----: | -------- | --------------- |
| `9910` | TCP      | 학생 PC ↔ Host 접속 |
| `8554` | RTSP     | 학생 PC → Host 접속 |
| `9912` | UDP      | Host → 학생 PC 접속 |

### Port 구조

```text
Participant PC                    VTD Host

      │                              │
      ├──────── TCP 9910 ────────────┤
      │                              │
      ├──────── RTSP 8554 ──────────>│
      │                              │
      │<─────── UDP 9912 ────────────┤
      │                              │
```

> 위 화살표는 원본 인터페이스 표의 접속 방향 표기를 기준으로 작성했습니다.

현재 Excel 인터페이스 표에는 각 Port에서 **정확히 어떤 Packet이 전달되는지에 대한 상세 매핑은 별도로 명시되어 있지 않습니다.**

---

# 5. Recommended Internal Data Structure

아래 구조는 **공식 통신 Packet 정의가 아니라**, 수신한 데이터를 프로그램 내부에서 관리하기 위한 권장 구조 예시입니다.

```cpp
struct EgoState
{
    float x;
    float y;
    float z;

    float heading;
    float pitch;
    float roll;
};
```

```cpp
struct ObjectState
{
    uint32_t id;

    float x;
    float y;
    float z;

    float heading;
    float speed;

    float length;
    float width;
    float height;
};
```

```cpp
struct TrafficLightState
{
    int32_t id;
    uint8_t state;
};
```

```cpp
struct ControlCommand
{
    float steering;
    float targetAccel;
    uint8_t turnSignal;
};
```

프로그램 내부에서는 다음과 같이 하나의 State로 관리할 수 있습니다.

```cpp
struct VehicleState
{
    EgoState ego;
    std::array<ObjectState, 30> objects;
    std::vector<TrafficLightState> trafficLights;
};
```

> `TrafficLight`의 개수와 실제 Packet 구조가 확인된 이후 자료구조는 수정할 수 있습니다.

---

# 6. Program Data Flow

현재 Repository 구조를 기준으로 Interface 데이터는 다음과 같이 처리합니다.

```text
VTD
 │
 │ Raw Packet
 ▼
src/network/
 │
 │ Packet Parsing
 ▼
src/data/
 │
 │ Ego / Object / TrafficLight State
 ▼
src/planning/
 │
 │ Risk Assessment
 │ Traffic Rule
 │ Event Handling
 │ Path / Speed Planning
 ▼
src/control/
 │
 │ Lateral Control
 │ Longitudinal Control
 ▼
ControlCommand
 │
 │ steering
 │ targetAccel
 │ turnSignal
 ▼
src/network/
 │
 ▼
VTD
```

---

# 7. Module Responsibility

## `network/`

통신 자체를 담당합니다.

```text
- VTD Host 연결
- TCP / UDP 데이터 송수신
- Raw Packet 수신
- Packet Parsing
- ControlCommand 송신
```

Planning이나 Control 판단 로직은 `network/`에 작성하지 않습니다.

---

## `data/`

수신한 데이터를 프로그램 내부에서 사용하기 쉬운 형태로 관리합니다.

```text
EgoState
ObjectState
TrafficLightState
Map / Lane Data
```

---

## `planning/`

수신된 상태를 기반으로 주행 방법을 결정합니다.

예:

```text
앞 차량과 거리가 가까움
        ↓
TTC 계산
        ↓
감속 필요 판단
        ↓
Target Speed 감소
```

또는:

```text
Traffic Light = RED
        ↓
정지 필요 판단
        ↓
Target Speed = 0
```

---

## `control/`

Planning에서 결정한 목표를 실제 VTD Control Command로 변환합니다.

```text
Planning

Target Path
Target Speed
     │
     ▼
Control

Lateral Controller
→ steering

Longitudinal Controller
→ targetAccel

Event / Lane Change
→ turnSignal
```

---

# 8. Important Notes

현재 제공된 Interface Excel만으로 **확정할 수 없는 항목**들이 있습니다.

실제 VTD 연동 전에 반드시 확인해야 합니다.

### Packet 관련

* Packet Header 구조
* Packet 전체 Layout
* 필드 전송 순서
* Endianness
* Structure Packing / Alignment
* TCP Message 경계 처리 방식
* Packet Length 정보 존재 여부
* Frame ID 존재 여부
* Simulation Timestamp 존재 여부
* 데이터 송신 주기(Hz)

### Object 관련

* Object Array가 항상 30개 고정 전송되는지
* 현재 유효한 Object 개수를 어떻게 판단하는지
* 사용되지 않는 Object Slot의 값
* Object Type 정보 제공 여부

  * Vehicle
  * Pedestrian
  * Obstacle 등
* Object ID의 생성 및 유지 규칙

특히 현재 제공된 필드에는 **Object Type이 명시되어 있지 않습니다.**

따라서 보행자 / 차량 / 기타 장애물을 어떠한 방식으로 구분하는지 확인이 필요합니다.

### Ego 관련

현재 인터페이스 표에는 Ego의 다음 정보가 별도로 명시되어 있지 않습니다.

```text
Ego Speed
Ego Acceleration
Yaw Rate
```

따라서 Ego Speed 등이 별도 API에서 제공되는지 확인이 필요합니다.

제공되지 않는 경우 위치와 시간 정보를 이용한 추정이 필요할 수 있습니다.

### Traffic Light 관련

* `trafficLights[]` 최대 개수
* Traffic Light ID와 OpenDRIVE Signal ID의 대응 관계
* Ego Vehicle이 따라야 할 신호등을 판단하는 방법
* 점멸 상태에서 적색/황색 구분 여부

등의 확인이 필요합니다.

### Coordinate 관련

다음 좌표계 정의도 확인이 필요합니다.

* X / Y / Z Axis 방향
* Global Coordinate 기준
* Heading의 0 rad 기준 방향
* Heading 증가 방향
* Object 좌표 기준점
* Ego 좌표 기준점

---

# 9. Interface Checklist

VTD 실제 연동 전 다음 항목을 확인합니다.

* [ ] TCP 9910 연결 확인
* [ ] UDP 9912 수신 확인
* [ ] RTSP 8554 연결 확인
* [ ] Ego X/Y/Z 정상 수신 확인
* [ ] Ego Heading/Pitch/Roll 정상 수신 확인
* [ ] Object Array 정상 수신 확인
* [ ] Object ID 유지 여부 확인
* [ ] Object Type 구분 방법 확인
* [ ] Traffic Light ID/State 확인
* [ ] Ego Speed 제공 여부 확인
* [ ] Packet 송신 주기 확인
* [ ] Timestamp / Frame ID 확인
* [ ] Coordinate System 확인
* [ ] Steering 부호 방향 확인
* [ ] Steering 최대/최소 범위 확인
* [ ] targetAccel 최대/최소 범위 확인
* [ ] Turn Signal 정상 동작 확인

---

## Source

본 문서는 대회에서 제공된 `인터페이스 API.xlsx` 내용을 기준으로 작성하였습니다.

인터페이스 규격이 변경될 경우 본 문서와 `src/network/`, `src/data/` 구현을 함께 업데이트합니다.
