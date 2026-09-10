# VTD Ego 차량 제원 혼용 진단 — 2026-09-10

## 결론

설치된 **VTD 2025.2.438 `viTrafficDyn` 플러그인**에서 `Player default="true"`이면
다른 Player의 `VehicleDef`를 Ego의 물리 모델에 적용하는 동작을 **별도 프로세스에서 재현했다**.
이는 제어기의 LD/조향 이득만으로 해결할 문제가 아니다.

- 제어기: `src/network/send/control_sender.cpp`는 조향 float를 그대로 전송한다.
- 커스텀 `HLVTD` 플러그인: `ControlApplier::buildDriverControl`은 조향을 복사하거나
  제한한다. 축거를 섞거나 축거 비율로 조향을 변환하는 코드는 이 경로에 없다.
- `viTrafficDyn`: 물리 모델의 `VehicleDef` 갱신 시 이름이 다르더라도
  `useDefaultPlayer=true`이면 갱신을 허용한다. 이 경로에서 `wheelBase`가 덮어써진다.
- NPC의 `external` 설정은 이 문제를 노출하는 조건이지, 여러 external 차량 자체가
  금지되었다는 뜻은 아니다. NPC를 일괄 internal로 바꾸면 시나리오 행동이 달라질 수 있다.

현재 설정 파일:
`/home/autonav/VIRES/VTD.2025.2/Data/Setups/00_HL_VTD/Config/ModuleManager/moduleManager.xml`
의 HLVTD(8행), viTrafficDyn(14행)에 진단 당시 각각 `<Player default="true"/>`가 있었다.
두 플러그인이 Ego를 대상으로 하는 것 자체는 역할 중복 버그가 아니다.
HLVTD는 참가자 명령을 RDB 제어 명령으로 변환하고, viTrafficDyn은 물리 계산을 한다.

## 기존 주행 로그와의 일치

모든 시나리오의 Ego는 Ioniq 6이며 제어기 축거는 2.944 m다.
기존 로그의 실제 회전 반응은 마지막 external NPC의 축거와 일치하는 방향으로 달랐다.

| 시나리오 | 마지막 external NPC | NPC 축거 | `2.944 / NPC 축거` | 실제/명령 기반 회전 반응 근사 |
|---|---|---:|---:|---:|
| M09 | Alfa Romeo Brera | 2.536 m | 1.161 | 약 1.1 |
| M17 | ViresTruck01 | 3.696 m | 0.797 | 약 0.775 |
| M18 | Mercedes Travego | 7.317 m | 0.402 | 약 0.36–0.40 |

M18은 최대 조향을 보내도 실제로 크게 도는 반경이 너무 넓어졌으며,
문제 코너 이전부터 회전 응답이 작았다. M09에는 비슷한 최소 반경의 코너가 있었고
비슷한 진입 속도와 LD로 통과했다. 따라서 M18 코너 형상이나 LD만 원인으로
설명하는 것은 불충분하다. 회전 반응 수치는 로그 추정이며 아래 시험은
모델 내부 축거가 실제로 바뀌는지를 따로 확인한 것이다.

## 독립 재현 방법

`tools/verification/probe_vtd_vehicle_definition.cpp`는 설치된 플러그인의
`makeModule()`로 **자기 프로세스 안의 새 인스턴스**를 만든다.
주행 중인 VTD에 연결하지 않고, 설정·시나리오·플러그인 파일을 변경하지 않는다.
Player를 Ego로 지정한 뒤 Ego와 NPC의 `VehicleDef` 명령을 순서대로 전달하고
그 인스턴스의 축거를 읽는다.

이 도구는 일반 공개 ABI 진단기가 아니다. 아래 SHA-256과 일치하는 x86-64 cxx11
바이너리만 허용하며, 다른 파일은 로딩 전에 거부한다. 동일 설치본의 의존 라이브러리를
사용해야 한다. 내부 오프셋은 이 정확한 빌드를 분석하여 확인했다.

```text
libModuleTrafficDyn.so.2025.2.438
SHA-256: 5889bdd536f67589fd4a038973c2d1ed956494790c31492643d100b592a7d0ed
```

저장소 루트에서 실행(OpenSSL 개발 라이브러리 필요):

```bash
g++ -std=c++17 -O0 -g tools/verification/probe_vtd_vehicle_definition.cpp -ldl -lcrypto -o /tmp/probe_vtd_vehicle_definition
LD_LIBRARY_PATH=/home/autonav/VIRES/VTD.2025.2/Runtime/Core/ModuleManager.cxx11/lib:/home/autonav/VIRES/VTD.2025.2/Runtime/Core/Lib /tmp/probe_vtd_vehicle_definition /home/autonav/VIRES/VTD.2025.2/Data/Distros/Distro/Plugins/ModuleManager.cxx11/libModuleTrafficDyn.so.2025.2.438
```

실측 결과:

```text
defaultPlayer=1 player=Ego
incoming=Ego wheelbase=2.944 parsed=1 model_wheelbase=2.944
incoming=NewPlayer09 wheelbase=7.317 parsed=1 model_wheelbase=7.317
incoming=NewPlayer13 wheelbase=3.696 parsed=1 model_wheelbase=3.696
defaultPlayer=0 player=Ego
incoming=Ego wheelbase=2.944 parsed=1 model_wheelbase=2.944
incoming=NewPlayer09 wheelbase=7.317 parsed=1 model_wheelbase=2.944
incoming=NewPlayer13 wheelbase=3.696 parsed=1 model_wheelbase=2.944
REPRODUCED
```

`parsed=1`은 SCP 문자열을 파싱했다는 뜻이며, 특정 NPC 제원이 적용되었다는 뜻이 아니다.
적용 여부는 마지막 `model_wheelbase` 값으로 확인한다.

## 수정 권고와 미검증 범위

viTrafficDyn의 Player를 다음처럼 **Ego로 명시하고 default를 끈다**.
HLVTD도 동일한 바인딩으로 맞추는 것이 좋다.

```xml
<Player name="Ego" default="false" />
```

기존에 오염된 물리 모델이 남지 않도록 새 INIT/시나리오 로딩 이후 확인해야 한다.
NPC 제원에 맞춰 제어기의 축거를 변경하는 보상은 하지 않는다.
설정 변경 후에는 M18 코너뿐 아니라 M17/M09의 추종·회피를 다시 검증해야 한다.
지금까지의 제어 튜닝이 잘못된 물리 제원에 맞춰져 있었을 가능성 때문이다.

**이 진단에서는 외부 VTD 설정을 변경하지 않았고, 이 설정 수정 후 Live M18 완주도
검증하지 않았다.** 독립 시험은 제원 혼용과 명시적 바인딩에 의한 차단을 검증한 것이다.

## 후속 요청에 따른 설정 적용

2026-09-10 사용자 요청으로 위 `00_HL_VTD/Config/ModuleManager/moduleManager.xml`의
HLVTD와 viTrafficDyn 두 Player를 실제로 `<Player name="Ego" default="false" />`로 수정했다.
`Driver ctrlLatLong="extern"`, EgoID, 제어 코드, M18/NPC 시나리오 설정은 그대로다.
원본 백업은 `/tmp/hl_vtd_moduleManager_before_ego_binding_20260910.xml`에 있다.
XML 조각 구문과 두 플러그인 속성을 검사했으며, 변경은 두 줄뿐이다.

설정 파일을 다시 읽고 기존 모델 인스턴스를 버리도록 **VTD를 완전히 종료한 뒤
동일한 00_HL_VTD Setup으로 재시작하고 M18을 Load/Apply/Init**해야 한다.
기존 실행에서 Play만 다시 누르는 것은 적용 확인이 아니다.
사용자가 주행을 검증하기로 하여 VTD 재시작/실주행은 수행하지 않았다.
다른 Setup이나 다른 PC의 설정 파일은 이번 변경 대상이 아니다.

## 구현 및 공식 설치본 근거

- `Develop/Modules/Common/inc/ModulePlugin.hh:248`은 default player를 첫 external로 설명한다.
- `Doc/Wiki/FAQ_ModuleManager_2025.2.pdf`, 4쪽도 default가 첫 external을 선택하며
  `<Player name="..."/>`로 명시할 수 있다고 설명한다.
- `libVTDModulePlugin.so.2025.2.438`: `useDefaultPlayer`는 객체의 `+0x135`에 bool을 저장한다.
- `libModuleTrafficDyn.so.2025.2.438`: `handleSCPCommand`의 `0xa454–0xa472`는
  Player 이름이 불일치해도 위 bool이 참이면 VehicleDef 갱신으로 진입한다.
  `0xb531–0xb575`에서 wheelBase를 읽고 `0xb918–0xb925`에서 같은 모델에 적용한다.
  모델 포인터 오프셋은 `+0x380`, VehicleDef 내 wheelBase 오프셋은 `+0xc8`이다.
