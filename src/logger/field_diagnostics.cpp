#include "logger/field_diagnostics.hpp"
#include "planning/local/corridor_lattice.hpp"
#include "logger/diagnostic_queue.hpp"
#include "logger/field_diagnostic_snapshot.hpp"

#include <cstring>
#include <fstream>
#include <thread>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

enum class DiagnosticRecordKind { Stage, Input, Frame };

struct DiagnosticRecord {
    DiagnosticRecordKind kind{};
    std::size_t frame{};
    double processWall{}, receiveTime{};
    EgoState ego;
    TrafficLightState signal;
    std::array<char, 64> stage{};
    std::array<char, 768> detail{};
    bool detailTruncated{};
    DiagnosticFrameSnapshot snapshot;
};

struct FieldDiagnosticWriter {
    DiagnosticQueue<DiagnosticRecord, 128> queue;
    std::ofstream output;
    std::chrono::steady_clock::time_point started;
    std::thread thread;
    std::atomic<bool> stopping{false}, failed{false};
    std::size_t sequence{};
    ~FieldDiagnosticWriter();
};

namespace {

void writeString(std::ostream& out, const std::string& value) {
    out << '"';
    for (const unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 0x20) out << "\\u00" << std::hex << std::setw(2)
                               << std::setfill('0') << static_cast<unsigned int>(c)
                               << std::dec << std::setfill(' ');
        else out << c;
    }
    out << '"';
}

void number(std::ostream& out, double value) {
    if (std::isfinite(value)) out << std::setprecision(10) << value;
    else out << "null";
}

void field(std::ostream& out, const char* name, double value) {
    out << ",\"" << name << "\":";
    number(out, value);
}

void writeObb(std::ostream& out, const Obb2D& box) {
    out << '[';
    for (std::size_t i = 0; i < box.corners.size(); ++i) {
        if (i) out << ',';
        out << '['; number(out, box.corners[i].x);
        out << ','; number(out, box.corners[i].y); out << ']';
    }
    out << ']';
}

void writeCommand(std::ostream& out, const ControlCommand& command) {
    out << "{\"steering_rad\":"; number(out, command.steering);
    field(out, "accel_mps2", command.targetAccel);
    field(out, "indicator", command.turnSignal); out << '}';
}

void writeRawObject(std::ostream& out, const ObjectState& object) {
    out << "{\"id\":" << object.id;
    field(out, "x", object.x); field(out, "y", object.y); field(out, "z", object.z);
    field(out, "heading", object.heading); field(out, "speed", object.speed);
    field(out, "length", object.length); field(out, "width", object.width);
    field(out, "height", object.height); out << '}';
}

void writeWorldObjects(std::ostream& out, const WorldState& world) {
    out << ",\"objects\":[";
    for (std::size_t i = 0; i < world.objects.size(); ++i) {
        if (i) out << ',';
        const auto& object = world.objects[i];
        out << "{\"raw\":"; writeRawObject(out, object.raw);
        field(out, "lane_valid", object.laneValid);
        field(out, "road", object.roadId); field(out, "section", object.sectionIndex);
        field(out, "lane", object.laneId); field(out, "road_s", object.roadS);
        field(out, "path_valid", object.pathValid);
        field(out, "path_s", object.pathS); field(out, "path_d", object.pathD);
        field(out, "relative_s", object.relativeS); field(out, "relative_d", object.relativeD);
        field(out, "ahead", object.isAhead); field(out, "on_path", object.isOnEgoPath);
        field(out, "crosswalk", object.isInsideCrosswalk);
        field(out, "risk", object.isCollisionRisk); field(out, "ttc", object.ttc);
        field(out, "vx", object.velocityX); field(out, "vy", object.velocityY);
        field(out, "velocity_valid", object.velocityValid);
        out << ",\"obb\":"; writeObb(out, object.obb); out << '}';
    }
    out << ']';
}

void writeEgo(std::ostream& out, const WorldState& world) {
    out << ",\"ego\":{\"x\":"; number(out, world.ego.x);
    field(out, "y", world.ego.y); field(out, "z", world.ego.z);
    field(out, "heading", world.ego.heading); field(out, "pitch", world.ego.pitch);
    field(out, "roll", world.ego.roll); field(out, "speed", world.egoSpeed);
    field(out, "input_valid", world.inputValid);
    field(out, "position_jump", world.respawnDetected);
    field(out, "lane_valid", world.egoLaneValid);
    field(out, "road", world.egoRoadId); field(out, "section", world.egoSectionIndex);
    field(out, "lane", world.egoLaneId); field(out, "road_s", world.egoRoadS);
    field(out, "lane_width", world.egoLaneWidth);
    field(out, "lane_distance", world.egoLaneCenterDistance);
    field(out, "path_valid", world.egoPathValid);
    field(out, "path_s", world.egoPathS); field(out, "path_d", world.egoPathD);
    field(out, "front_bumper_path_s", world.frontBumperPathS);
    out << ",\"obb\":"; writeObb(out, world.egoObb); out << '}';
}

void writeTracking(std::ostream& out, const DiagnosticPlanning& snapshot) {
    const auto& plan = snapshot.result;
    const auto& tracking = plan.trackingPlan;
    out << ",\"tracking\":{\"valid\":" << tracking.valid;
    field(out, "nearest_index", tracking.nearestIndex);
    field(out, "target_index", tracking.targetIndex);
    field(out, "lookahead", tracking.lookahead);
    field(out, "max_curvature", tracking.maxCurvature);
    field(out, "steering_rad", tracking.steering);
    field(out, "local_valid", plan.localPath.valid);
    field(out, "local_begin_index", plan.localPath.beginIndex);
    field(out, "local_end_index", plan.localPath.endIndex);
    const auto& points = plan.localPath.points;
    field(out, "point_count", snapshot.pointTotal);
    const std::size_t stride = snapshot.pointStride;
    field(out, "sample_stride", stride);
    out << ",\"points_index_x_y_yaw_s_road_section_lane\":[";
    bool first = true;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!first) out << ',';
        first = false;
        const auto& p = points[i];
        const auto index = i + 1 == points.size() ? snapshot.pointTotal - 1 : i * stride;
        out << '[' << index << ','; number(out, p.x); out << ','; number(out, p.y);
        out << ','; number(out, p.yaw); out << ','; number(out, p.s);
        out << ',' << p.roadId << ',' << p.sectionIndex << ',' << p.laneId << ']';
    }
    out << "]}";
}

void writePlanning(std::ostream& out, const DiagnosticPlanning& planning) {
    const auto& p = planning.result;
    out << ",\"planning\":{\"valid\":" << p.valid;
    field(out, "goal", p.goalReached); field(out, "target_speed", p.targetSpeed);
    field(out, "stop_reason", static_cast<unsigned>(p.stopReason));
    field(out, "health", static_cast<int>(p.health));
    field(out, "lon_mode", static_cast<int>(p.longitudinal));
    field(out, "emergency", p.emergencyBrake);
    field(out, "emergency_hold", planning.emergencyBrakeHoldRemaining);
    field(out, "speed_constraint", p.currentSpeedLimit);
    field(out, "school_zone", p.schoolZoneActive); field(out, "crosswalk", p.crosswalkActive);
    field(out, "stop_valid", p.stopDistanceValid); field(out, "stop_distance", p.stopDistance);
    field(out, "stop_source_id", planning.stop.sourceId);
    field(out, "stop_route_s", planning.stop.stopRouteS);
    field(out, "stop_hold_s", planning.hold.stoppedTime);
    field(out, "stop_hold_satisfied", planning.hold.holdSatisfied);
    field(out, "collision", p.pathSafety.collision);
    field(out, "predicted_collision", p.pathSafety.predictedCollision);
    field(out, "predicted_ttc", p.pathSafety.predictedCollisionTTC);
    field(out, "min_ttc", p.pathSafety.minimumTTC);
    field(out, "risk_object", p.pathSafety.riskObjectId);
    field(out, "recovery", p.progressRecoveryActive);
    field(out, "recovery_contact_expected", p.progressRecoveryContactExpected);
    field(out, "recovery_emergency_overridden", p.progressRecoveryEmergencyOverridden);
    field(out, "recovery_overridden_stop", static_cast<unsigned>(p.progressRecoveryOverriddenStop));
    field(out, "recovery_blocked_s", planning.recoveryBlockedTime);
    field(out, "recovery_no_motion_s", planning.recoveryNoMotionTime);
    out << ",\"stop_requests\":[";
    bool first = true;
    for (std::size_t i = 0; i < planning.requestCount; ++i) {
        const auto& request = planning.requests[i];
        if (!first) out << ',';
        first = false;
        out << "{\"reason\":" << static_cast<unsigned>(request.reason);
        field(out, "active", request.active); field(out, "source_id", request.sourceId);
        field(out, "route_s", request.stopRouteS); field(out, "hold_s", request.requiredHoldTime);
        out << '}';
    }
    out << "]}";
}

void writeLaneChange(std::ostream& out, const LaneChangeData& lane) {
    out << ",\"lane_change\":{\"phase\":" << static_cast<int>(lane.phase);
    field(out, "phase_elapsed", lane.phaseElapsed); field(out, "obstacle", lane.obstacleId);
    field(out, "source_road", lane.sourceRoadId); field(out, "source_section", lane.sourceSectionIndex);
    field(out, "source_lane", lane.sourceLaneId); field(out, "target_lane", lane.targetLaneId);
    field(out, "route_mode", static_cast<int>(lane.routeMode));
    field(out, "waiting_return", lane.waitingForReturn);
    field(out, "reconnect_valid", lane.reconnectLaneValid);
    field(out, "reconnect_road", lane.reconnectLane.roadId);
    field(out, "reconnect_section", lane.reconnectLane.sectionIndex);
    field(out, "reconnect_lane", lane.reconnectLane.laneId);
    field(out, "reconnect_index", lane.reconnectGlobalIndex);
    field(out, "local_avoidance", lane.localCorridorAvoidance);
    field(out, "clearance", lane.validatedObstacleClearance);
    field(out, "failed_obstacle", lane.failedObstacleId);
    field(out, "retry_after", lane.retrySearchAfter); out << '}';
}

void writeMission(std::ostream& out, const DiagnosticMission& mission) {
    out << ",\"mission\":{\"valid\":" << mission.valid;
    field(out, "topology_authority", mission.topologyAuthority);
    field(out, "leg", mission.activeLegIndex); field(out, "leg_count", mission.legCount);
    field(out, "target_distance", mission.minimumTargetDistance);
    field(out, "target_road_observed", mission.targetRoadObserved);
    field(out, "target_stable_frames", mission.targetRoadStableFrames);
    if (mission.legPresent) {
        const auto& leg = mission;
        field(out, "from_checkpoint", leg.fromCheckpoint);
        field(out, "gateway_checkpoint", leg.gatewayCheckpoint);
        field(out, "to_checkpoint", leg.toCheckpoint);
        field(out, "allowed_lane_count", leg.allowedLaneCount);
        out << ",\"target_road_section_lane\":[";
        bool first = true;
        for (std::size_t i = 0; i < leg.targetLaneCount; ++i) {
            const auto& lane = leg.targetLanes[i];
            if (!first) out << ',';
            first = false;
            out << '[' << lane.roadId << ',' << lane.sectionIndex << ',' << lane.laneId << ']';
        }
        out << ']';
    }
    out << '}';
}

void writeCorridor(std::ostream& out, const DiagnosticCorridor& c) {
    out << ",\"corridor\":";
    if (!c.present) { out << "null"; return; }
    out << "{\"route_revision\":" << c.routeRevision;
    field(out, "request_revision", c.globalRouteRequestRevision);
    field(out, "route_points", c.routePoints);
    field(out, "route_progress", c.committedGlobalPathProgress);
    field(out, "reaches_gateway", c.committedGlobalPathReachesGateway);
    field(out, "exit_preview", c.committedGlobalPathHasExitPreview);
    field(out, "approved", c.trackingPathApproved);
    field(out, "blocker", c.trackingPathBlockingObject);
    field(out, "clearance", c.trackingPathValidatedClearance);
    field(out, "search_failed", c.globalRouteSearchFailed);
    field(out, "replan_requested", c.globalRouteReplanRequested);
    field(out, "global_worker_present", c.workerPresent);
    field(out, "local_pending", c.localFallbackPending);
    field(out, "local_engaged", c.localFallbackEngaged);
    field(out, "local_relocalization", c.localFallbackRelocalizationRequested);
    field(out, "committed_transition", c.committedTransition);
    field(out, "transition_progress", c.committedTransitionProgress); out << '}';
}

void writeControl(std::ostream& out, const ControlProcessData& control) {
    out << ",\"control\":{\"nominal\":"; writeCommand(out, control.nominalCommand);
    out << ",\"safe\":"; writeCommand(out, control.safeCommand);
    field(out, "overridden", control.safety.commandOverridden);
    field(out, "invalid_input", control.safety.invalidInput);
    field(out, "invalid_command", control.safety.invalidCommand);
    field(out, "emergency_applied", control.safety.emergencyBrakeApplied);
    field(out, "emergency_steering_latched", control.emergencySteeringLatched);
    field(out, "previous_speed_error", control.controller.previousSpeedError);
    field(out, "previous_acceleration", control.controller.previousAcceleration);
    field(out, "max_steering", control.controller.config.maxSteering);
    field(out, "min_acceleration", control.controller.config.minAcceleration);
    field(out, "max_acceleration", control.controller.config.maxAcceleration); out << '}';
}


double elapsed(const FieldDiagnosticWriter& writer) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - writer.started).count();
}

void copyText(char* destination, std::size_t capacity, const char* text, std::size_t size) {
    const auto count = std::min(capacity - 1, size);
    std::memcpy(destination, text, count);
    destination[count] = '\0';
}

void copyWorld(DiagnosticFrameSnapshot& out, const WorldState& world) {
    out.world.ego = world.ego;
    out.world.egoObb = world.egoObb;
    out.world.receiveTime = world.receiveTime;
    out.world.egoSpeed = world.egoSpeed;
    out.world.inputValid = world.inputValid;
    out.world.respawnDetected = world.respawnDetected;
    out.world.egoLaneValid = world.egoLaneValid;
    out.world.egoRoadId = world.egoRoadId;
    out.world.egoLaneId = world.egoLaneId;
    out.world.egoSectionIndex = world.egoSectionIndex;
    out.world.egoRoadS = world.egoRoadS;
    out.world.egoLaneWidth = world.egoLaneWidth;
    out.world.egoLaneCenterDistance = world.egoLaneCenterDistance;
    out.world.egoPathValid = world.egoPathValid;
    out.world.egoPathS = world.egoPathS;
    out.world.egoPathD = world.egoPathD;
    out.world.frontBumperOffset = world.frontBumperOffset;
    out.world.frontBumperPathS = world.frontBumperPathS;
    out.world.trafficLight = world.trafficLight;
    out.objectTotal = world.objects.size();
    const auto count = std::min<std::size_t>(30, world.objects.size());
    out.world.objects.assign(world.objects.begin(), world.objects.begin() + count);
}

void copyPlan(DiagnosticPlanning& out, const PlanningProcessData& planning) {
    out.result.valid = planning.result.valid;
    out.result.goalReached = planning.result.goalReached;
    out.result.targetSpeed = planning.result.targetSpeed;
    out.result.stopReason = planning.result.stopReason;
    out.result.longitudinal = planning.result.longitudinal;
    out.result.health = planning.result.health;
    out.result.trackingPlan = planning.result.trackingPlan;
    out.result.pathSafety = planning.result.pathSafety;
    out.result.emergencyBrake = planning.result.emergencyBrake;
    out.result.turnSignal = planning.result.turnSignal;
    out.result.currentSpeedLimit = planning.result.currentSpeedLimit;
    out.result.schoolZoneActive = planning.result.schoolZoneActive;
    out.result.stopDistanceValid = planning.result.stopDistanceValid;
    out.result.stopDistance = planning.result.stopDistance;
    out.result.crosswalkActive = planning.result.crosswalkActive;
    out.result.laneChangeActive = planning.result.laneChangeActive;
    out.result.progressRecoveryActive = planning.result.progressRecoveryActive;
    out.result.progressRecoveryContactExpected = planning.result.progressRecoveryContactExpected;
    out.result.progressRecoveryEmergencyOverridden = planning.result.progressRecoveryEmergencyOverridden;
    out.result.progressRecoveryOverriddenStop = planning.result.progressRecoveryOverriddenStop;
    out.stop = planning.frame.stop;
    out.hold = planning.stopManager.hold;
    out.requestTotal = planning.stopManager.requests.size();
    out.requestCount = std::min(out.requests.size(), out.requestTotal);
    std::copy_n(planning.stopManager.requests.begin(), out.requestCount, out.requests.begin());
    out.emergencyBrakeHoldRemaining = planning.emergencyBrakeHoldRemaining;
    out.recoveryBlockedTime = planning.progressRecovery.blockedTime;
    out.recoveryNoMotionTime = planning.progressRecovery.noMotionTime;
    const auto& path = planning.result.localPath;
    out.result.localPath.valid = path.valid;
    out.result.localPath.beginIndex = path.beginIndex;
    out.result.localPath.endIndex = path.endIndex;
    out.pointTotal = path.points.size();
    out.pointStride = std::max<std::size_t>(1, (out.pointTotal + 15) / 16);
    auto& points = out.result.localPath.points;
    points.clear();
    for (std::size_t i = 0; i < out.pointTotal; i += out.pointStride) points.push_back(path.points[i]);
    if (out.pointTotal && (out.pointTotal - 1) % out.pointStride) points.push_back(path.points.back());
}

void copyLane(LaneChangeData& out, const LaneChangeData& lane) {
    out.phase = lane.phase;
    out.phaseElapsed = lane.phaseElapsed;
    out.obstacleId = lane.obstacleId;
    out.sourceRoadId = lane.sourceRoadId;
    out.sourceSectionIndex = lane.sourceSectionIndex;
    out.sourceLaneId = lane.sourceLaneId;
    out.targetLaneId = lane.targetLaneId;
    out.routeMode = lane.routeMode;
    out.waitingForReturn = lane.waitingForReturn;
    out.reconnectLaneValid = lane.reconnectLaneValid;
    out.reconnectLane = lane.reconnectLane;
    out.reconnectGlobalIndex = lane.reconnectGlobalIndex;
    out.localCorridorAvoidance = lane.localCorridorAvoidance;
    out.validatedObstacleClearance = lane.validatedObstacleClearance;
    out.failedObstacleId = lane.failedObstacleId;
    out.retrySearchAfter = lane.retrySearchAfter;
}

void copyMission(DiagnosticMission& out, const MissionCorridorData& mission) {
    out = {};
    out.valid = mission.valid;
    out.topologyAuthority = mission.topologyAuthority;
    out.targetRoadObserved = mission.targetRoadObserved;
    out.activeLegIndex = mission.activeLegIndex;
    out.minimumTargetDistance = mission.minimumTargetDistance;
    out.targetRoadStableFrames = mission.targetRoadStableFrames;
    out.legCount = mission.legs.size();
    out.legPresent = mission.activeLegIndex < mission.legs.size();
    if (!out.legPresent) return;
    const auto& leg = mission.legs[mission.activeLegIndex];
    out.fromCheckpoint = leg.fromCheckpoint;
    out.gatewayCheckpoint = leg.gatewayCheckpoint;
    out.toCheckpoint = leg.toCheckpoint;
    out.allowedLaneCount = leg.allowedLanes.size();
    out.targetLaneTotal = leg.targetLanes.size();
    out.targetLaneCount = std::min(out.targetLanes.size(), out.targetLaneTotal);
    std::copy_n(leg.targetLanes.begin(), out.targetLaneCount, out.targetLanes.begin());
}

void copyCorridor(DiagnosticCorridor& out, const PathGeneratorData& map) {
    out.present = map.corridorRuntime != nullptr;
    if (!out.present) return;
    const auto& c = *map.corridorRuntime;
    out.routeRevision = c.routeRevision;
    out.globalRouteRequestRevision = c.globalRouteRequestRevision;
    out.committedGlobalPathProgress = c.committedGlobalPathProgress;
    out.committedGlobalPathReachesGateway = c.committedGlobalPathReachesGateway;
    out.committedGlobalPathHasExitPreview = c.committedGlobalPathHasExitPreview;
    out.trackingPathApproved = c.trackingPathApproved;
    out.trackingPathBlockingObject = c.trackingPathBlockingObject;
    out.trackingPathValidatedClearance = c.trackingPathValidatedClearance;
    out.globalRouteSearchFailed = c.globalRouteSearchFailed;
    out.globalRouteReplanRequested = c.globalRouteReplanRequested;
    out.localFallbackPending = c.localFallbackPending;
    out.localFallbackEngaged = c.localFallbackEngaged;
    out.localFallbackRelocalizationRequested = c.localFallbackRelocalizationRequested;
    out.committedTransition = c.committedTransition;
    out.committedTransitionProgress = c.committedTransitionProgress;
    out.routePoints = c.committedGlobalPath.size();
    out.workerPresent = c.globalRouteWorker != nullptr;
}

void copyFrame(DiagnosticFrameSnapshot& out, const FieldDiagnosticFrame& input) {
    out.frame = input.frame;
    out.wallTime = input.wallTime;
    out.simulationTime = input.simulationTime;
    out.dt = input.dt;
    out.receiveWaitMs = input.receiveWaitMs;
    out.clockMs = input.clockMs;
    out.worldMs = input.worldMs;
    out.planningMs = input.planningMs;
    out.controlMs = input.controlMs;
    out.receiveOnly = input.receiveOnly;
    out.sendCompleted = input.sendCompleted;
    out.sendMs = input.sendMs;
    out.sendIntervalMs = input.sendIntervalMs;
    out.receiveToSendMs = input.receiveToSendMs;
    out.droppedGt = input.receiver.droppedFrames;
    out.bufferBytes = input.receiver.streamBuffer.size();
    out.control = input.control;
    copyWorld(out, input.world);
    copyPlan(out.planning, input.planning);
    copyLane(out.lane, input.planning.laneChange);
    copyMission(out.mission, input.planning.missionCorridor);
    copyCorridor(out.corridor, input.map);
}

void serializeFrame(std::ostream& out, FieldDiagnosticWriter& writer, const DiagnosticFrameSnapshot& input) {
    out << "{\"schema\":1,\"kind\":\"frame\",\"seq\":" << writer.sequence++;
    field(out, "frame", input.frame); field(out, "wall_s", input.wallTime);
    field(out, "process_wall_s", input.processWall);
    out << ",\"sim_s\":"; if (input.simulationTime) number(out, *input.simulationTime); else out << "null";
    field(out, "dt", input.dt); field(out, "receive_time_s", input.world.receiveTime);
    field(out, "receive_wait_ms", input.receiveWaitMs); field(out, "clock_ms", input.clockMs);
    field(out, "world_ms", input.worldMs); field(out, "planning_ms", input.planningMs);
    field(out, "control_ms", input.controlMs); field(out, "receive_only", input.receiveOnly);
    field(out, "dropped_gt_total", input.droppedGt); field(out, "buffer_bytes", input.bufferBytes);
    field(out, "signal_id", input.world.trafficLight.id);
    field(out, "signal_raw_state", input.world.trafficLight.state);
    field(out, "objects_total", input.objectTotal);
    field(out, "stop_requests_total", input.planning.requestTotal);
    field(out, "target_lanes_total", input.mission.targetLaneTotal);
    writeEgo(out, input.world); writePlanning(out, input.planning);
    writeTracking(out, input.planning); writeLaneChange(out, input.lane);
    writeMission(out, input.mission); writeCorridor(out, input.corridor);
    writeControl(out, input.control); writeWorldObjects(out, input.world);
    out << "}\n";
    out << "{\"schema\":1,\"kind\":\"send_result\",\"seq\":" << writer.sequence++;
    field(out, "frame", input.frame); field(out, "process_wall_s", input.processWall);
    field(out, "socket_send_completed", input.sendCompleted); field(out, "receive_only", input.receiveOnly);
    field(out, "send_ms", input.sendMs); field(out, "send_interval_ms", input.sendIntervalMs);
    field(out, "receive_to_send_ms", input.receiveToSendMs);
    field(out, "diagnostic_capture_ms", input.captureMs);
    out << ",\"vtd_applied_ack\":null}\n";
}

void serializeRecord(FieldDiagnosticWriter& writer, const DiagnosticRecord& record) {
    auto& out = writer.output;
    if (record.kind == DiagnosticRecordKind::Frame) {
        serializeFrame(out, writer, record.snapshot);
        return;
    }
    out << "{\"schema\":1,\"kind\":";
    writeString(out, record.kind == DiagnosticRecordKind::Stage ? "stage" : "input");
    field(out, "seq", writer.sequence++);
    field(out, "frame", record.frame); field(out, "process_wall_s", record.processWall);
    if (record.kind == DiagnosticRecordKind::Stage) {
        out << ",\"stage\":"; writeString(out, record.stage.data());
        out << ",\"detail\":"; writeString(out, record.detail.data());
        field(out, "detail_truncated", record.detailTruncated);
    } else {
        field(out, "receive_time_s", record.receiveTime);
        field(out, "ego_x", record.ego.x); field(out, "ego_y", record.ego.y);
        field(out, "ego_z", record.ego.z); field(out, "ego_heading", record.ego.heading);
        field(out, "ego_pitch", record.ego.pitch); field(out, "ego_roll", record.ego.roll);
        field(out, "signal_id", record.signal.id); field(out, "signal_raw_state", record.signal.state);
    }
    out << "}\n";
}

void writeWriterStatus(FieldDiagnosticWriter& writer, bool final) {
    auto& out = writer.output;
    out << "{\"schema\":1,\"kind\":\"logger_status\",\"mode\":\"async_bounded\"";
    field(out, "dropped_records_total", writer.queue.dropped.load());
    field(out, "queued_total", writer.queue.written.load());
    field(out, "written_total", writer.queue.read.load());
    field(out, "final", final);
    out << "}\n";
    out.flush();
}

void runFieldWriter(FieldDiagnosticWriter* writer) {
    try {
        auto flushed = std::chrono::steady_clock::now();
        while (true) {
            const auto* record = peekDiagnostic(writer->queue);
            if (record) {
                serializeRecord(*writer, *record);
                releaseDiagnostic(writer->queue);
            } else {
                if (writer->stopping.load(std::memory_order_acquire)) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - flushed >= std::chrono::milliseconds(100)) {
                writeWriterStatus(*writer, false);
                flushed = now;
            }
            if (!writer->output) throw std::runtime_error("diagnostic output failed");
        }
        writeWriterStatus(*writer, true);
        if (!writer->output) throw std::runtime_error("diagnostic final flush failed");
    } catch (...) {
        writer->failed.store(true, std::memory_order_release);
        std::cerr << "[diagnostics-error] asynchronous writer failed; evidence is incomplete\n";
    }
}

DiagnosticRecord* reserveRecord(FieldDiagnosticData& data, DiagnosticRecordKind kind, std::size_t frame) {
    if (!data.enabled || data.writer->failed.load(std::memory_order_acquire)) return nullptr;
    auto* record = reserveDiagnostic(data.writer->queue);
    if (!record) return nullptr;
    record->kind = kind;
    record->frame = frame;
    record->processWall = elapsed(*data.writer);
    return record;
}

void stopWriter(FieldDiagnosticWriter& writer) {
    writer.stopping.store(true, std::memory_order_release);
    if (writer.thread.joinable()) writer.thread.join();
    writer.output.close();
}

}  // namespace

FieldDiagnosticWriter::~FieldDiagnosticWriter() { stopWriter(*this); }

void initializeFieldDiagnostics(FieldDiagnosticData& data, const std::filesystem::path& directory) {
    if (data.enabled) throw std::runtime_error("diagnostics already initialized");
    std::filesystem::create_directories(directory);
    const auto path = directory / "diagnostics.jsonl";
    if (std::filesystem::exists(path)) throw std::runtime_error("diagnostic log already exists: " + path.string());
    auto writer = std::make_shared<FieldDiagnosticWriter>();
    writer->output.open(path);
    if (!writer->output) throw std::runtime_error("cannot open diagnostic log: " + path.string());
    for (auto& record : *writer->queue.slots) {
        record.snapshot.world.objects.reserve(30);
        record.snapshot.planning.result.localPath.points.reserve(17);
    }
    writer->started = std::chrono::steady_clock::now();
    writer->thread = std::thread(runFieldWriter, writer.get());
    data.writer = std::move(writer);
    data.enabled = true;
    writeFieldStage(data, "process_start", 0, "schema=1 mode=async_bounded compiled=" __DATE__ " " __TIME__);
}

void writeFieldStage(FieldDiagnosticData& data, const char* stage, std::size_t frame, const std::string& detail) {
    auto* record = reserveRecord(data, DiagnosticRecordKind::Stage, frame);
    if (!record) return;
    copyText(record->stage.data(), record->stage.size(), stage, std::strlen(stage));
    copyText(record->detail.data(), record->detail.size(), detail.data(), detail.size());
    record->detailTruncated = detail.size() >= record->detail.size();
    commitDiagnostic(data.writer->queue);
}

void writeFieldReceived(FieldDiagnosticData& data, const GtReceiverData& receiver, std::size_t frame) {
    auto* record = reserveRecord(data, DiagnosticRecordKind::Input, frame);
    if (!record) return;
    record->ego = receiver.latestGt.ego;
    record->signal = receiver.latestGt.trafficLight;
    record->receiveTime = receiver.latestReceiveTime;
    commitDiagnostic(data.writer->queue);
}

void writeFieldFrame(FieldDiagnosticData& data, const FieldDiagnosticFrame& input) {
    if (!data.enabled) return;
    const auto started = std::chrono::steady_clock::now();
    auto* record = reserveRecord(data, DiagnosticRecordKind::Frame, input.frame);
    if (!record) return;
    copyFrame(record->snapshot, input);
    record->snapshot.processWall = record->processWall;
    record->snapshot.captureMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    // Frame and send_result share one slot: overload cannot separate the pair.
    commitDiagnostic(data.writer->queue);
}

void finishFieldDiagnostics(FieldDiagnosticData& data) {
    data.enabled = false;
    if (data.writer) stopWriter(*data.writer);
}
