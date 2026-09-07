#pragma once

#include "planning/planning_structs.hpp"

#include <cstdint>

// 속도 PD 제어기와 VTD 명령 제한값.
struct ControllerConfig {
    double kp{1.0};
    double kd{0.15};
    double minAcceleration{-4.0};
    double maxAcceleration{2.0};
    // 이 속도 이하에서 목표속도도 0이면 PD 대신 확정 감속으로 붙잡는다.
    double stopHoldSpeedThreshold{0.8};
    double maxErrorDerivative{10.0};
    double maxAccelerationRate{6.0};
    // 조향에도 변화율 제한을 둔다. 가속도에만 있고 조향에는 없어서
    // 저속·짧은 Ld 구간에서 앞바퀴가 프레임마다 크게 튄다.
    double maxSteeringRate{0.9};   // [rad/s]
    double maxSteering{0.48};
};

// PD 미분항 계산을 위해 프레임 사이에 유지하는 상태.
struct ControllerData {
    ControllerConfig config;
    double previousSpeedError{};
    double previousAcceleration{};
    double previousSteering{};
    bool hasPreviousSteering{};
    bool hasPreviousError{};
    bool hasPreviousAcceleration{};
};

// VTD로 전송할 제어 명령. 직렬화 시 padding 없이 별도로 패킷화해야 한다.
struct ControlCommand {
    float steering{};
    float targetAccel{};
    std::uint8_t turnSignal{};
};

struct SafetyReport {
    bool commandOverridden{};
    bool emergencyBrakeApplied{};
    bool invalidInput{};
    bool invalidCommand{};
};

// 제어 프로세스의 설정, 프레임 간 유지 상태, 이번 프레임 결과.
// Ego 속도 추정은 World process가 담당하므로 여기서는 읽기만 한다.
struct ControlProcessData {
    ControllerData controller;   // 유지 상태 (PD 이전값)
    double egoSpeed{};           // World에서 받은 값 (읽기 전용 사본)
    ControlCommand nominalCommand;  // Safety 적용 전
    ControlCommand safeCommand;     // VTD로 실제 전송하는 명령
    ControlCommand latestCommand;   // 로깅용 = safeCommand
    SafetyReport safety;
    // 비상제동 진입 순간의 조향을 유지한다. 제동 중 속도가
    // 낮아진 상태에서 lookahead 대상이 바뀌어 조향이 포화되는 것을 막는다.
    bool emergencySteeringLatched{};
    float emergencySteering{};
};

// GT 프레임 기반 제어 주기와 속도 필터 설정.
struct ControlRuntimeConfig {
    double nominalDt{0.05};
    double speedFilterAlpha{0.25};
    double maxValidPositionSpeed{80.0};
    double inputTimeout{0.25};
};

// control.yaml에서 읽은 전체 주행 설정.
struct DrivingConfig {
    PathPlannerConfig planner;
    PlanningBehaviorConfig behavior;
    LaneChangeConfig laneChange;
    ControllerConfig controller;
    ControlRuntimeConfig runtime;
};
