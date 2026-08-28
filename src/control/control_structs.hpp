#pragma once

#include "planning/planning_structs.hpp"

#include <cstdint>

// 속도 PD 제어기와 VTD 명령 제한값.
struct ControllerConfig {
    double kp{1.0};
    double kd{0.15};
    double minAcceleration{-4.0};
    double maxAcceleration{2.0};
    double maxErrorDerivative{10.0};
    double maxAccelerationRate{6.0};
    double maxSteering{0.48};
};

// PD 미분항 계산을 위해 프레임 사이에 유지하는 상태.
struct ControllerData {
    ControllerConfig config;
    double previousSpeedError{};
    double previousAcceleration{};
    bool hasPreviousError{};
    bool hasPreviousAcceleration{};
};

// VTD로 전송할 제어 명령. 직렬화 시 padding 없이 별도로 패킷화해야 한다.
struct ControlCommand {
    float steering{};
    float targetAccel{};
    std::uint8_t turnSignal{};
};

// GT에 Ego speed가 없어 위치 차분으로 속도를 계산하기 위한 상태.
struct ControlProcessData {
    ControllerData controller;
    double previousX{};
    double previousY{};
    double rawEgoSpeed{};
    double egoSpeed{};
    bool hasPreviousPosition{};
    bool hasSpeedEstimate{};
    ControlCommand latestCommand;
};

// GT 프레임 기반 제어 주기와 속도 필터 설정.
struct ControlRuntimeConfig {
    double nominalDt{0.05};
    double speedFilterAlpha{0.25};
    double maxValidPositionSpeed{80.0};
};

// control.yaml에서 읽은 전체 주행 설정.
struct DrivingConfig {
    PathPlannerConfig planner;
    ControllerConfig controller;
    ControlRuntimeConfig runtime;
};
