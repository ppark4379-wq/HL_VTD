#pragma once

#include <cstdint>
#include <vector>

// =============================================================================
// 행동 상태 축
//
// 하나의 enum으로 합치지 않고 서로 독립적인 축으로 나눈다.
// "차선변경 중 적색신호 정지" 같은 상황은 단일 enum으로 표현할 수 없다.
//
//   종방향   LonMode          지금 가속/추종/정지 중 무엇을 하는가
//   횡방향   LaneChangePhase  차선변경 기동의 어느 단계인가
//   시스템   SystemHealth     입력이 정상인가, 복구 중인가
//
// Safety는 여기에 넣지 않는다. 상태가 아니라 Control이 최종 명령을 검사한
// 결과이므로 control_structs.hpp의 SafetyReport에 기록한다.
// =============================================================================

// 종방향 행동. 목표속도를 어떤 근거로 정하는지를 나타낸다.
enum class LonMode {
    Cruise,  // 제한속도 이내 순항
    Follow,  // 앞차 속도에 맞춰 추종
    Stop,    // 정지점까지 감속 후 정지 유지
};

// 횡방향 기동 단계.
//
// 차선변경은 완주가 불가능한 상황(정적 장애물이 차선을 완전히 막음)에서만
// 쓰는 마지막 수단이다. lane_change_planner가 상태를 소유한다.
enum class LaneChangePhase {
    None,     // 현재 차선 유지
    Prepare,  // 목표 차선 고정, 방향지시등 점등, 진입 시점 대기
    Execute,  // 실제 횡이동 중. 완료 전까지 목표 차선을 바꾸지 않는다
    Settle,   // 목표 차선 진입 후 횡오차·heading 오차가 수렴할 때까지
};

// 시스템 상태. 리스폰이나 입력 이상을 만나면 Recovery로 내려간다.
enum class SystemHealth {
    Ok,
    Recovery,
};

// 정지 이유. 동시에 여러 개가 성립할 수 있으므로 비트마스크로 쓴다.
enum class StopReason : std::uint32_t {
    None = 0,
    RedLight = 1U << 0,
    FlashingRed = 1U << 1,
    Pedestrian = 1U << 2,
    Obstacle = 1U << 3,
    CrosswalkBlocked = 1U << 4,
};

// 황색신호는 매 프레임 판단이 뒤집히면 안 되므로 한 번 정한 결과를 고정한다.
// 관련 신호등이 바뀌면 Undecided로 되돌린다.
enum class YellowDecision {
    Undecided,
    Stop,
    Pass,
};

// 정지 요청 하나. "왜"뿐 아니라 "어디에" 서는지를 함께 가진다.
// 해제는 위치가 아니라 원인이 사라질 때만 이루어져야 한다.
struct StopRequest {
    StopReason reason{StopReason::None};
    bool active{};
    double stopRouteS{};       // 전역경로 누적거리 기준 정지 위치
    double requiredHoldTime{}; // 이 정지에 필요한 최소 정차시간
    int sourceId{-1};          // 신호등 id 또는 객체 id
};

// 정지선 앞에서 실제로 얼마나 서 있었는지 센다.
// 평가항목 7(적색신호)과 9(적색점멸)가 "0.5초 이상 정지"를 요구하므로
// 이 값이 없으면 감속만 하고 지나가 중대 위반이 된다.
struct StopHoldState {
    double stoppedTime{};
    bool holdSatisfied{};
    int stopLineId{-1};
};

struct YellowSignalState {
    int controllerId{-1};
    YellowDecision decision{YellowDecision::Undecided};
};

// 프레임 간 유지하는 정지 관리 상태.
struct StopManagerData {
    std::vector<StopRequest> requests;
    StopHoldState hold;
    YellowSignalState yellow;
};

// 모든 후보에 항상 적용되는 제약.
struct ConstraintSet {
    bool mapMatchValid{};
    double speedLimit{};
};

// 이번 프레임에 실제로 적용할 정지점 하나.
// distanceToStop은 정지점을 이미 지났으면 음수가 된다.
struct StopDecision {
    bool valid{};
    StopReason reason{StopReason::None};
    double stopRouteS{};
    double distanceToStop{};
    int sourceId{-1};
};

// 프레임 간 유지하는 행동 상태. hysteresis와 최소 유지시간에 쓴다.
struct BehaviorState {
    LonMode longitudinal{LonMode::Cruise};
    LaneChangePhase laneChange{LaneChangePhase::None};
    SystemHealth health{SystemHealth::Ok};
    double recoveryHoldTime{};
    // 모든 객체에 대한 OBB 검사를 통과한 정적 장애물 회피 경로가 활성 상태다.
    // 하나의 장애물이 여러 VTD 객체로 구성돼도 그 정적 조각을 앞차로 잡지 않는다.
    bool staticObstacleAvoidanceActive{};
    // 선택했던 객체가 재출발해도 현재 회피 경로가 안전한 동안에는 기동을
    // 유지한다. 경로가 위험해지는 즉시 -1로 해제되어 Follow/TTC가 복귀한다.
    int safelyAvoidedObjectId{-1};
};

// 이번 프레임의 행동 결정 결과.
struct BehaviorDecision {
    LonMode longitudinal{LonMode::Cruise};
    LaneChangePhase laneChange{LaneChangePhase::None};
    SystemHealth health{SystemHealth::Ok};
    double targetSpeed{};
    bool emergencyBrake{};
    std::uint8_t turnSignal{};  // 0=OFF, 1=LEFT, 2=RIGHT
};
