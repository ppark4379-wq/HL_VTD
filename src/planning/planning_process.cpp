#include "planning/planning_process.hpp"

#include "planning/behavior/behavior_planner.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/local/local_path_planner.hpp"
#include "planning/local/lane_change_planner.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

// GT trafficLight.id는 XODR <controller> id다. 개별 signal id가 아니다.
//   <controller id="11"> <control signalId="54"/> <control signalId="55"/>
//                        <control signalId="58"/> </controller>
// signal 54/55/58은 적·황·녹 개별 램프이고, 패킷이 보내는 11이 논리 신호등이다.
//
// 신호 상태로 정지를 만들 정지선은 반드시 controller id가 일치해야 한다.
// road/lane만으로 보조 매칭하면, 무관한 신호등이 적색일 때 같은 도로의
// 무신호 정지선(일시정지선 등)을 붙잡고 멈추는 오검출이 생긴다.
const StopLine* findActiveStopLine(const WorldState& world,
                                   const StaticZoneData& zones,
                                   double behindWindow,
                                   double aheadWindow) {
    // 같은 road/lane에 있는 정지선을 우선하되, 못 찾으면 controller만 맞는
    // 정지선을 쓴다.
    //
    // 정지선 바로 앞은 교차로 진입부라서 맵매칭이 현재 road와 연결 road
    // 사이에서 흔들린다. 실측에서 route_s 109.1 지점에 정지한 순간 매칭이
    // road 418/lane 4 -> road 3340/lane -3 으로 넘어갔고, road/lane 조건만
    // 쓰면 그 프레임에 정지 요청이 사라져 적색신호에 그대로 출발했다.
    // controller id는 패킷이 준 값과 정확히 대응하므로 이것만으로도
    // "지금 지켜야 할 신호"는 확정된다.
    const StopLine* laneMatched = nullptr;
    const StopLine* controllerMatched = nullptr;
    double laneMatchedDistance = std::numeric_limits<double>::infinity();
    double controllerMatchedDistance = std::numeric_limits<double>::infinity();

    for (const auto& line : zones.stopLines) {
        if (line.controllerId < 0) continue;
        if (line.controllerId != world.trafficLight.id) continue;
        const double distance = line.routeS - world.frontBumperPathS;
        if (distance < -behindWindow || distance > aheadWindow) continue;

        if (distance < controllerMatchedDistance) {
            controllerMatched = &line;
            controllerMatchedDistance = distance;
        }

        const bool roadMatches =
            line.roadId < 0 || line.roadId == world.egoRoadId;
        const bool laneMatches =
            line.laneIds.empty() ||
            std::find(line.laneIds.begin(), line.laneIds.end(),
                      world.egoLaneId) != line.laneIds.end();
        if (roadMatches && laneMatches && distance < laneMatchedDistance) {
            laneMatched = &line;
            laneMatchedDistance = distance;
        }
    }
    return laneMatched != nullptr ? laneMatched : controllerMatched;
}

void addStopRequest(StopManagerData& manager, StopReason reason,
                    double routeS, int sourceId, double holdTime = 0.0) {
    manager.requests.push_back({reason, true, routeS, holdTime, sourceId});
}

bool crosswalkHasRiskObject(const Crosswalk& crosswalk,
                            const WorldState& world,
                            double predictionHorizon) {
    return std::any_of(
        world.objects.begin(), world.objects.end(),
        [&](const WorldObjectState& object) {
            if (containsPoint(crosswalk.polygon,
                              {object.raw.x, object.raw.y})) return true;
            if (std::any_of(
                    object.obb.corners.begin(), object.obb.corners.end(),
                    [&](const Point2D& point) {
                        return containsPoint(crosswalk.polygon, point);
                    })) {
                return true;
            }
            if (predictionHorizon <= 0.0 ||
                std::abs(object.raw.speed) < 0.3) return false;
            const double predictedX =
                object.raw.x + object.raw.speed *
                                   std::cos(object.raw.heading) *
                                   predictionHorizon;
            const double predictedY =
                object.raw.y + object.raw.speed *
                                   std::sin(object.raw.heading) *
                                   predictionHorizon;
            return containsPoint(crosswalk.polygon,
                                 {predictedX, predictedY});
        });
}

double pointToSegmentDistance(const Point2D& point,
                              const Point2D& begin,
                              const Point2D& end) {
    const double dx = end.x - begin.x;
    const double dy = end.y - begin.y;
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 1e-9) {
        return std::hypot(point.x - begin.x, point.y - begin.y);
    }
    const double t = std::clamp(
        ((point.x - begin.x) * dx + (point.y - begin.y) * dy) /
            lengthSquared,
        0.0, 1.0);
    return std::hypot(point.x - (begin.x + t * dx),
                      point.y - (begin.y + t * dy));
}

bool pointNearCrosswalk(const Crosswalk& crosswalk,
                        const Point2D& point,
                        double margin) {
    if (containsPoint(crosswalk.polygon, point)) return true;
    if (margin <= 0.0 || crosswalk.polygon.size() < 2) return false;
    for (std::size_t index = 0; index < crosswalk.polygon.size(); ++index) {
        const Point2D& begin = crosswalk.polygon[index];
        const Point2D& end =
            crosswalk.polygon[(index + 1) % crosswalk.polygon.size()];
        if (pointToSegmentDistance(point, begin, end) <= margin) return true;
    }
    return false;
}

bool crosswalkHasNearbyObject(const Crosswalk& crosswalk,
                              const WorldState& world,
                              double margin) {
    return std::any_of(
        world.objects.begin(), world.objects.end(),
        [&](const WorldObjectState& object) {
            if (pointNearCrosswalk(crosswalk,
                                   {object.raw.x, object.raw.y}, margin)) {
                return true;
            }
            return std::any_of(
                object.obb.corners.begin(), object.obb.corners.end(),
                [&](const Point2D& corner) {
                    return pointNearCrosswalk(crosswalk, corner, margin);
                });
        });
}

bool shouldStopForYellow(StopManagerData& manager,
                         const WorldState& world,
                         const StopLine& line,
                         const PlanningBehaviorConfig& config) {
    auto& yellow = manager.yellow;
    if (yellow.controllerId != world.trafficLight.id) {
        yellow = {world.trafficLight.id, YellowDecision::Undecided};
    }
    if (yellow.decision == YellowDecision::Undecided) {
        const double distance = line.routeS - world.frontBumperPathS;
        const double speed = std::max(0.0, world.egoSpeed);
        const double stoppingDistance =
            speed * speed / (2.0 * config.comfortableDeceleration) +
            config.stopSafetyMargin;
        yellow.decision = distance > 0.0 && distance >= stoppingDistance
                              ? YellowDecision::Stop
                              : YellowDecision::Pass;
    }
    return yellow.decision == YellowDecision::Stop;
}

bool flashingRedHoldComplete(StopManagerData& manager,
                             const WorldState& world,
                             const StopLine& line,
                             const PlanningBehaviorConfig& config,
                             double dt) {
    auto& hold = manager.hold;
    if (hold.stopLineId != line.id) {
        hold = {};
        hold.stopLineId = line.id;
    }
    const double distance = line.routeS - world.frontBumperPathS;
    if (distance >= -0.5 && distance <= 1.5 &&
        std::abs(world.egoSpeed) <= config.stoppedSpeedThreshold) {
        hold.stoppedTime += std::max(0.0, dt);
        hold.holdSatisfied = hold.stoppedTime >= config.stopHoldTime;
    }
    return hold.holdSatisfied;
}

void updateStopRequests(PlanningProcessData& data,
                        const WorldState& world,
                        const StaticZoneData& zones,
                        double dt) {
    data.stopManager.requests.clear();
    const StopLine* line =
        findActiveStopLine(world, zones, data.config.stopLineBehindWindow,
                           data.config.signalStopSearchDistance);
    const int signalState = static_cast<int>(world.trafficLight.state);
    if (signalState != 2) data.stopManager.yellow = {};
    if (signalState != 6 || line == nullptr) data.stopManager.hold = {};

    if (line != nullptr) {
        if (signalState == 1) {
            addStopRequest(data.stopManager, StopReason::RedLight,
                           line->routeS, line->id);
        } else if (signalState == 2 &&
                   shouldStopForYellow(data.stopManager, world, *line,
                                       data.config)) {
            addStopRequest(data.stopManager, StopReason::RedLight,
                           line->routeS, line->id);
        } else if (signalState == 6 &&
                   isFlashingRedController(zones, world.trafficLight.id) &&
                   !flashingRedHoldComplete(data.stopManager, world, *line,
                                             data.config, dt)) {
            addStopRequest(data.stopManager, StopReason::FlashingRed,
                           line->routeS, line->id,
                           data.config.stopHoldTime);
        }
    }

    for (const auto& crosswalk : zones.crosswalks) {
        // 앞범퍼가 진입한 뒤에도 후륜축이 횡단보도를 완전히 빠져나갈 때까지
        // 점유 검사를 유지한다. 진입점을 넘었다는 이유로 보행자 정지를
        // 해제하면 횡단 중인 보행자 앞으로 다시 출발할 수 있다.
        if (crosswalk.exitRouteS < world.egoPathS) continue;
        const bool occupied = crosswalkHasRiskObject(
            crosswalk, world, data.config.crosswalkPedestrianPrediction);
        if (occupied) {
            // 횡단보도 위에 1 km/h 이하로 3초 이상 서 있으면 감점이다.
            // 횡단보도 입구를 목표로 삼으면 제동 오차만큼 밀려 횡단보도
            // 위에 서게 되므로, 그 앞 정지선이 있으면 정지선을 목표로 한다.
            // 여유는 정지 프로파일의 stop_safety_margin이 이미 적용한다.
            double target = crosswalk.entryRouteS;
            for (const auto& line : zones.stopLines) {
                if (line.routeS > crosswalk.entryRouteS + 1e-3) continue;
                if (crosswalk.entryRouteS - line.routeS >
                    data.config.crosswalkStopLineSearch) continue;
                target = std::min(target, line.routeS);
            }
            addStopRequest(data.stopManager, StopReason::CrosswalkBlocked,
                           std::max(target, world.frontBumperPathS),
                           crosswalk.id);
        }
    }

    for (const auto& object : world.objects) {
        if (isAvoidedLaneChangeObject(data.laneChange,
                                      static_cast<int>(object.raw.id))) {
            continue;
        }
        if (object.isAhead && object.isOnEgoPath &&
            std::abs(object.raw.speed) < 0.3 && object.relativeS < 30.0) {
            addStopRequest(data.stopManager, StopReason::Obstacle,
                           std::max(world.frontBumperPathS,
                                    object.pathS -
                                        data.config.minimumFollowDistance),
                           static_cast<int>(object.raw.id));
        }
    }

    // 정지 목표가 횡단보도 안이면, 아직 진입하지 않은 경우에 한해 앞범퍼
    // 목표를 횡단보도 입구로 당긴다. 이미 진입했다면 안전 정지를 우선한다.
    for (auto& request : data.stopManager.requests) {
        for (const auto& crosswalk : zones.crosswalks) {
            const bool targetInside =
                request.stopRouteS > crosswalk.entryRouteS &&
                request.stopRouteS < crosswalk.exitRouteS;
            if (targetInside &&
                world.frontBumperPathS < crosswalk.entryRouteS) {
                request.stopRouteS = crosswalk.entryRouteS;
            }
        }
    }
}

bool hasLaneChangeBlockingStop(const PlanningProcessData& data) {
    for (const auto& request : data.stopManager.requests) {
        if (!request.active) continue;
        if (request.reason == StopReason::Obstacle &&
            request.sourceId == data.laneChange.obstacleId) continue;
        return true;
    }
    return false;
}

bool evaluateLaneChangeCandidate(PathSafetyResult& safety,
                                 const LaneChangeData& laneChange,
                                 const WorldState& world,
                                 const PlanningBehaviorConfig& config,
                                 const LaneChangeConfig& laneChangeConfig) {
    LocalPathData candidate;
    makeLaneChangeLocalPath(candidate, laneChange);
    resetPathSafety(safety);
    if (!candidate.valid) return false;
    WorldState predictionWorld = world;
    predictionWorld.egoSpeed = std::max(
        {1.0, std::abs(world.egoSpeed),
         laneChangeConfig.maximumLaneChangeSpeed});
    const double remainingLength =
        std::max(0.0, candidate.points.back().s - world.egoPathS);
    const double fullManeuverHorizon =
        remainingLength / predictionWorld.egoSpeed +
        config.collisionPredictionStep;
    checkCurrentCollisions(safety, world.egoObb, world.objects);
    checkPredictedCollisions(safety, predictionWorld, candidate,
                             std::max(config.collisionPredictionHorizon,
                                      fullManeuverHorizon),
                             config.collisionPredictionStep);
    return !safety.collision && !safety.predictedCollision;
}

bool crossedFinishLine(PlanningProcessData& data,
                       const WorldState& world,
                       const StaticZoneData& zones,
                       bool pathGoalReached) {
    bool crossed = pathGoalReached;
    if (zones.finish.valid && world.egoPathValid) {
        crossed = data.hasPreviousRouteS &&
                  data.previousRouteS < zones.finish.routeS &&
                  world.egoPathS >= zones.finish.routeS;
    }
    if (world.egoPathValid) {
        data.previousRouteS = world.egoPathS;
        data.hasPreviousRouteS = true;
    }
    return crossed;
}

// 프레임 중간값을 0이 아니라 NaN/-1로 초기화한다.
// 0으로 두면 "쓰기 전에 읽음" 버그가 그럴듯한 값(정지거리 0 등)으로 조용히
// 통과하지만, NaN이면 즉시 드러난다.
void resetPlanningFrame(PlanningProcessData& data) {
    data.frame = {};
    data.result = {};
    const double poison = std::numeric_limits<double>::quiet_NaN();
    data.frame.constraints.speedLimit = poison;
    data.frame.stop.stopRouteS = poison;
    data.frame.stop.distanceToStop = poison;
    data.frame.behavior.targetSpeed = poison;
    data.frame.pathSafety.minimumTTC = poison;
    data.frame.pathSafety.riskObjectId = -1;
}

}  // namespace

bool initializePlanning(PlanningProcessData& data,
                        const PathPlannerConfig& plannerConfig,
                        const PlanningBehaviorConfig& behaviorConfig,
                        const LaneChangeConfig& laneChangeConfig,
                        const std::string& referencePathCsv) {
    data = {};
    data.pathPlanner.config = plannerConfig;
    data.config = behaviorConfig;
    data.laneChangeConfig = laneChangeConfig;
    return loadWaypointsCsv(data.pathPlanner.waypoints, referencePathCsv);
}

void runPlanningProcess(PlanningProcessData& data,
                        const GtFrame& gt,
                        const WorldState& world,
                        const PathGeneratorData& map,
                        double dt) {
    resetPlanningFrame(data);

    PathPlan tracking = runPurePursuit(data.pathPlanner, gt, world.egoSpeed);
    data.frame.constraints.mapMatchValid = world.inputValid;
    data.frame.constraints.speedLimit = findZoneSpeedLimit(
        map.staticZones, {world.ego.x, world.ego.y},
        world.egoPathValid ? world.egoPathS : -1.0,
        tracking.targetSpeed);

    // 위험 객체가 있는 횡단보도에만 접근 감속을 적용한다. 처음부터 빈
    // 횡단보도는 원래 경로 목표속도로 통과한다. 객체가 완전히 빠져나간 뒤에는
    // 별도의 clearance 속도로 횡단보도를 빠져나가고 원래 속도로 복귀한다.
    for (const auto& crosswalk : map.staticZones.crosswalks) {
        auto active = std::find(data.activeCrosswalkIds.begin(),
                                data.activeCrosswalkIds.end(), crosswalk.id);
        if (crosswalk.exitRouteS < world.egoPathS) {
            if (active != data.activeCrosswalkIds.end()) {
                data.activeCrosswalkIds.erase(active);
            }
            continue;
        }
        const bool risk = crosswalkHasRiskObject(
            crosswalk, world, data.config.crosswalkPedestrianPrediction);
        const bool nearby = crosswalkHasNearbyObject(
            crosswalk, world, data.config.crosswalkNearbyObjectDistance);
        if (risk && active == data.activeCrosswalkIds.end()) {
            data.activeCrosswalkIds.push_back(crosswalk.id);
        }
        if (!risk && !nearby && active == data.activeCrosswalkIds.end()) continue;
        const double distance = crosswalk.entryRouteS - world.frontBumperPathS;
        if ((risk || nearby) &&
            distance > data.config.crosswalkApproachDistance) continue;
        const double limit = (risk || nearby)
                                 ? data.config.crosswalkApproachSpeed
                                 : data.config.crosswalkClearanceSpeed;
        data.frame.constraints.speedLimit =
            data.frame.constraints.speedLimit > 0.0
                ? std::min(data.frame.constraints.speedLimit, limit)
                : limit;
    }

    tryPrepareObstacleLaneChange(
        data.laneChange, data.laneChangeConfig, data.config, world, map,
        data.pathPlanner, tracking.nearestIndex);

    updateStopRequests(data, world, map.staticZones, dt);
    selectStopDecision(data.frame.stop, data.stopManager,
                       world.frontBumperPathS);

    PathSafetyResult candidateSafety;
    const bool candidateSafe =
        data.laneChange.phase == LaneChangePhase::None ||
        evaluateLaneChangeCandidate(candidateSafety, data.laneChange,
                                    world, data.config,
                                    data.laneChangeConfig);
    advanceLaneChangePhase(data.laneChange, data.laneChangeConfig, world, dt,
                           hasLaneChangeBlockingStop(data), candidateSafe);

    if (data.laneChange.phase == LaneChangePhase::Execute) {
        tracking = trackLaneChangePath(
            data.laneChange, data.pathPlanner.config, gt,
            data.laneChangeConfig.maximumLaneChangeSpeed, world.egoSpeed);
        makeLaneChangeLocalPath(data.frame.localPath, data.laneChange);
    } else {
        if (data.laneChange.phase == LaneChangePhase::Settle &&
            data.laneChange.routeMode ==
                LaneChangeRouteMode::KeepTargetLane &&
            data.laneChange.reconnectLaneValid) {
            data.pathPlanner.nearestIndex =
                data.laneChange.reconnectGlobalIndex;
            tracking = runPurePursuit(data.pathPlanner, gt, world.egoSpeed);
        }
        generateLocalPath(data.frame.localPath, data.pathPlanner,
                          tracking.nearestIndex,
                          data.config.localPathHorizon);
    }
    if (!world.inputValid || world.respawnDetected) {
        data.behaviorState.health = SystemHealth::Recovery;
    }
    data.behaviorState.avoidedObstacleId =
        data.laneChange.phase == LaneChangePhase::Execute
            ? data.laneChange.obstacleId
            : -1;
    updateBehavior(data.behaviorState, data.frame.behavior,
                   data.frame.constraints, data.frame.stop, world,
                   tracking.targetSpeed, data.config, dt);
    data.behaviorState.laneChange = data.laneChange.phase;
    data.frame.behavior.laneChange = data.laneChange.phase;
    data.frame.behavior.turnSignal = data.laneChange.turnSignal;
    if (data.laneChange.phase == LaneChangePhase::Execute) {
        data.frame.behavior.targetSpeed = std::min(
            data.frame.behavior.targetSpeed,
            data.laneChangeConfig.maximumLaneChangeSpeed);
        if (data.laneChange.waitingForReturn) {
            const double distanceToWaitingPoint = std::max(
                0.0, data.laneChange.returnStartRouteS -
                         world.frontBumperPathS -
                         data.config.stopSafetyMargin);
            const double waitingPointSpeed = std::sqrt(
                2.0 * data.config.comfortableDeceleration *
                distanceToWaitingPoint);
            data.frame.behavior.targetSpeed = std::min(
                {data.frame.behavior.targetSpeed,
                 data.laneChangeConfig.returnWaitingSpeed,
                 waitingPointSpeed});
        }
    }

    // 미래 충돌검사가 선택된 지역경로를 사용한다.
    resetPathSafety(data.frame.pathSafety);
    checkCurrentCollisions(data.frame.pathSafety, world.egoObb,
                           world.objects);
    updateMinimumTTC(data.frame.pathSafety, world.objects);
    checkPredictedCollisions(data.frame.pathSafety, world,
                             data.frame.localPath,
                             data.config.collisionPredictionHorizon,
                             data.config.collisionPredictionStep);
    const bool blockedReturnNow =
        data.laneChange.waitingForReturn &&
        data.frame.pathSafety.predictedCollision;
    const bool emergency = data.frame.pathSafety.collision ||
                           blockedReturnNow ||
                           (data.frame.pathSafety.predictedCollision &&
                            data.frame.pathSafety.predictedCollisionTTC <=
                                data.config.emergencyTTC) ||
                           data.frame.pathSafety.minimumTTC <=
                               data.config.emergencyTTC;
    data.frame.behavior.emergencyBrake = emergency;
    if (emergency) data.frame.behavior.targetSpeed = 0.0;

    tracking.targetSpeed = data.frame.behavior.targetSpeed;
    tracking.goalReached = crossedFinishLine(
        data, world, map.staticZones, tracking.goalReached);
    data.pathPlanner.latestPlan = tracking;

    data.result.valid = tracking.valid && world.inputValid;
    data.result.goalReached = tracking.goalReached;
    data.result.targetSpeed = tracking.targetSpeed;
    data.result.stopReason = data.frame.stop.reason;
    data.result.longitudinal = data.frame.behavior.longitudinal;
    data.result.health = data.frame.behavior.health;
    data.result.trackingPlan = tracking;
    data.result.localPath = data.frame.localPath;
    data.result.pathSafety = data.frame.pathSafety;
    data.result.emergencyBrake = emergency;
    data.result.turnSignal = data.frame.behavior.turnSignal;
}
