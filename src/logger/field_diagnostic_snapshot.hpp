#pragma once

#include "control/control_structs.hpp"

#include <array>
#include <optional>

// Owned, bounded copies only. No planner/map/worker pointers cross threads.
struct DiagnosticMission {
    bool valid{}, topologyAuthority{}, targetRoadObserved{}, legPresent{};
    std::size_t activeLegIndex{}, legCount{}, fromCheckpoint{}, gatewayCheckpoint{}, toCheckpoint{};
    std::size_t allowedLaneCount{}, targetLaneTotal{}, targetLaneCount{};
    double minimumTargetDistance{};
    int targetRoadStableFrames{};
    std::array<LaneKey, 32> targetLanes{};
};

struct DiagnosticCorridor {
    bool present{}, committedGlobalPathReachesGateway{}, committedGlobalPathHasExitPreview{};
    bool trackingPathApproved{}, globalRouteSearchFailed{}, globalRouteReplanRequested{};
    bool workerPresent{}, localFallbackPending{}, localFallbackEngaged{}, localFallbackRelocalizationRequested{};
    std::uint64_t routeRevision{}, globalRouteRequestRevision{};
    std::size_t routePoints{}, committedGlobalPathProgress{}, committedTransitionProgress{};
    int trackingPathBlockingObject{}, committedTransition{};
    double trackingPathValidatedClearance{};
};

struct DiagnosticPlanning {
    PlanningResult result;
    StopDecision stop;
    StopHoldState hold;
    std::array<StopRequest, 16> requests{};
    std::size_t requestCount{}, requestTotal{}, pointTotal{}, pointStride{};
    double emergencyBrakeHoldRemaining{}, recoveryBlockedTime{}, recoveryNoMotionTime{};
};

struct DiagnosticFrameSnapshot {
    std::size_t frame{}, droppedGt{}, bufferBytes{}, objectTotal{};
    double wallTime{}, processWall{}, dt{}, receiveWaitMs{}, clockMs{}, worldMs{}, planningMs{}, controlMs{};
    std::optional<double> simulationTime;
    bool receiveOnly{}, sendCompleted{};
    double sendMs{}, sendIntervalMs{}, receiveToSendMs{}, captureMs{};
    WorldState world;
    DiagnosticPlanning planning;
    LaneChangeData lane;
    DiagnosticMission mission;
    DiagnosticCorridor corridor;
    ControlProcessData control;
};
