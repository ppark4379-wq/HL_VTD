#include "planning/local/lane_change_planner.hpp"

#include "planning/collision/collision_checker.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>

namespace {

double smoothStep(double value) {
    const double t = std::clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

void resetLaneChange(LaneChangeData& state) {
    state = {};
}

const WorldObjectState* findBlockingObstacle(
    const WorldState& world, const LaneChangeConfig& config) {
    const WorldObjectState* result = nullptr;
    for (const auto& object : world.objects) {
        if (!object.isAhead || !object.isOnEgoPath ||
            std::abs(object.raw.speed) >= 0.3 ||
            object.relativeS < config.obstacleTriggerMinimumDistance ||
            object.relativeS > config.obstacleTriggerMaximumDistance) {
            continue;
        }
        if (result == nullptr || object.relativeS < result->relativeS) {
            result = &object;
        }
    }
    return result;
}

bool targetLaneHasSafeGap(const WorldState& world,
                          int roadId,
                          int sectionIndex,
                          int targetLaneId,
                          const LaneChangeConfig& config) {
    for (const auto& object : world.objects) {
        if (!object.laneValid || object.roadId != roadId ||
            object.sectionIndex != sectionIndex ||
            object.laneId != targetLaneId || !object.pathValid) continue;
        const double gap = object.pathS - world.egoPathS;
        if (gap >= 0.0 && gap < config.minimumTargetAheadGap) return false;
        if (gap < 0.0 && -gap < config.minimumTargetBehindGap) return false;

        const double objectLongitudinalSpeed =
            static_cast<double>(object.raw.speed) *
            std::cos(object.raw.heading - world.ego.heading);
        const double closingSpeed = gap >= 0.0
                                        ? world.egoSpeed - objectLongitudinalSpeed
                                        : objectLongitudinalSpeed - world.egoSpeed;
        if (calculateTTC(std::abs(gap), closingSpeed) <
            config.minimumTargetTTC) return false;
    }
    return true;
}

std::uint8_t turnSignalTowardLane(const WorldState& world,
                                  const PathGeneratorData& map,
                                  int targetLaneId) {
    const auto roadFound = map.roads.find(world.egoRoadId);
    if (roadFound == map.roads.end() || world.egoSectionIndex < 0 ||
        world.egoSectionIndex >=
            static_cast<int>(roadFound->second.sections.size())) return 0;
    const Road& road = roadFound->second;
    const LaneSection& section = road.sections[world.egoSectionIndex];
    const RoadPose reference = roadToWorld(road, world.egoRoadS);
    const double offset =
        laneOffset(road, section, targetLaneId, world.egoRoadS);
    const double targetX = reference.x - offset * std::sin(reference.yaw);
    const double targetY = reference.y + offset * std::cos(reference.yaw);
    const double relativeX = targetX - world.ego.x;
    const double relativeY = targetY - world.ego.y;
    const double left = -std::sin(world.ego.heading) * relativeX +
                         std::cos(world.ego.heading) * relativeY;
    return left >= 0.0 ? 1U : 2U;
}

int sectionIndexAt(const Road& road, double roadS) {
    int result = 0;
    for (std::size_t index = 0; index < road.sections.size(); ++index) {
        if (road.sections[index].s <= roadS + 1e-6) {
            result = static_cast<int>(index);
        } else {
            break;
        }
    }
    return result;
}

bool findNextGlobalRoadLane(LaneKey& output,
                            std::size_t& reconnectIndex,
                            const PathPlannerData& globalPath,
                            const PathGeneratorData& map,
                            std::size_t nearestGlobalIndex,
                            int currentRoadId) {
    for (std::size_t index = nearestGlobalIndex + 1;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& point = globalPath.waypoints[index];
        if (point.roadId == currentRoadId) continue;
        const auto road = map.roads.find(point.roadId);
        if (road == map.roads.end() || road->second.sections.empty()) {
            return false;
        }
        output = {point.roadId,
                  sectionIndexAt(road->second, point.roadS),
                  point.laneId};
        reconnectIndex = index;
        return map.laneSegments.count(output) != 0;
    }
    return false;
}

// 기존 laneGraph에서 같은 road/section의 횡방향 차선변경 edge만 제외한다.
// 남는 것은 lane successor, road successor, junction laneLink 연결이다.
std::vector<LaneKey> findForwardLaneRoute(const PathGeneratorData& map,
                                          const LaneKey& start,
                                          const LaneKey& goal) {
    std::queue<LaneKey> pending;
    std::map<LaneKey, bool> visited;
    std::map<LaneKey, LaneKey> previous;
    pending.push(start);
    visited[start] = true;
    while (!pending.empty()) {
        const LaneKey current = pending.front();
        pending.pop();
        if (current == goal) break;
        const auto neighbors = map.laneGraph.find(current);
        if (neighbors == map.laneGraph.end()) continue;
        for (const LaneKey& next : neighbors->second) {
            const bool lateralLaneChange =
                current.roadId == next.roadId &&
                current.sectionIndex == next.sectionIndex &&
                current.laneId != next.laneId;
            const bool unexpectedRoad =
                next.roadId != start.roadId &&
                next.roadId != goal.roadId;
            if (lateralLaneChange || unexpectedRoad || visited.count(next)) {
                continue;
            }
            visited[next] = true;
            previous[next] = current;
            pending.push(next);
        }
    }
    if (!visited.count(goal)) return {};
    std::vector<LaneKey> route;
    for (LaneKey key = goal;; key = previous.at(key)) {
        route.push_back(key);
        if (key == start) break;
    }
    std::reverse(route.begin(), route.end());
    return route;
}

void appendUnique(std::vector<Waypoint>& output, const Waypoint& point) {
    if (!output.empty() &&
        std::hypot(output.back().x - point.x,
                   output.back().y - point.y) < 0.05) return;
    output.push_back(point);
}

void updateCandidateGeometry(std::vector<Waypoint>& points,
                             double startRouteS) {
    if (points.empty()) return;
    points.front().s = startRouteS;
    for (std::size_t index = 1; index < points.size(); ++index) {
        points[index].s = points[index - 1].s +
            std::hypot(points[index].x - points[index - 1].x,
                       points[index].y - points[index - 1].y);
        points[index - 1].yaw =
            std::atan2(points[index].y - points[index - 1].y,
                       points[index].x - points[index - 1].x);
    }
    if (points.size() > 1) points.back().yaw = points[points.size() - 2].yaw;
}

bool buildKeepTargetPath(std::vector<Waypoint>& output,
                         LaneKey& reconnectLane,
                         std::size_t& reconnectGlobalIndex,
                         int targetLaneId,
                         const LaneChangeConfig& config,
                         const WorldState& world,
                         const PathGeneratorData& map,
                         const PathPlannerData& globalPath,
                         std::size_t nearestGlobalIndex) {
    const LaneKey targetKey{world.egoRoadId, world.egoSectionIndex,
                            targetLaneId};
    if (!findNextGlobalRoadLane(reconnectLane, reconnectGlobalIndex,
                                globalPath, map, nearestGlobalIndex,
                                world.egoRoadId)) return false;
    const std::vector<LaneKey> forwardRoute =
        findForwardLaneRoute(map, targetKey, reconnectLane);
    if (forwardRoute.empty()) return false;

    const auto roadFound = map.roads.find(world.egoRoadId);
    if (roadFound == map.roads.end()) return false;
    const Road& road = roadFound->second;
    const LaneSection& section = road.sections[world.egoSectionIndex];
    if (!laneChangeAllowed(section, world.egoLaneId, targetLaneId,
                           world.egoRoadS)) return false;
    if (!targetLaneHasSafeGap(world, world.egoRoadId,
                              world.egoSectionIndex, targetLaneId,
                              config)) return false;

    output.clear();
    const double transitionEndS = world.egoPathS + config.transitionLength;
    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& original = globalPath.waypoints[index];
        if (original.s > transitionEndS) break;
        if (original.roadId != world.egoRoadId ||
            original.laneId != world.egoLaneId ||
            sectionIndexAt(road, original.roadS) != world.egoSectionIndex) {
            return false;
        }
        const RoadPose reference = roadToWorld(road, original.roadS);
        const double offset =
            laneOffset(road, section, targetLaneId, original.roadS);
        const double targetX = reference.x - offset * std::sin(reference.yaw);
        const double targetY = reference.y + offset * std::cos(reference.yaw);
        const double blend = smoothStep(
            (original.s - world.egoPathS) / config.transitionLength);
        Waypoint point = original;
        point.x += blend * (targetX - original.x);
        point.y += blend * (targetY - original.y);
        if (blend >= 0.5) point.laneId = targetLaneId;
        appendUnique(output, point);
    }
    if (output.size() < 2) return false;

    const auto targetSegment = map.laneSegments.find(targetKey);
    if (targetSegment == map.laneSegments.end()) return false;
    const auto& targetPoints = targetSegment->second.points;
    std::size_t targetBegin = 0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < targetPoints.size(); ++index) {
        const double distance = std::hypot(
            targetPoints[index].x - output.back().x,
            targetPoints[index].y - output.back().y);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            targetBegin = index;
        }
    }
    for (std::size_t index = targetBegin + 1;
         index < targetPoints.size(); ++index) {
        appendUnique(output, targetPoints[index]);
    }
    for (std::size_t routeIndex = 1;
         routeIndex + 1 < forwardRoute.size(); ++routeIndex) {
        const auto segment = map.laneSegments.find(forwardRoute[routeIndex]);
        if (segment == map.laneSegments.end()) return false;
        for (const Waypoint& point : segment->second.points) {
            appendUnique(output, point);
        }
    }

    const double appendUntil =
        globalPath.waypoints[reconnectGlobalIndex].s + 30.0;
    for (std::size_t index = reconnectGlobalIndex;
         index < globalPath.waypoints.size() &&
         globalPath.waypoints[index].s <= appendUntil; ++index) {
        appendUnique(output, globalPath.waypoints[index]);
    }
    updateCandidateGeometry(output, world.egoPathS);
    return output.size() >= 3;
}

bool buildReturnCandidatePath(std::vector<Waypoint>& output,
                        std::uint8_t& turnSignal,
                        double& returnStartRouteS,
                        double& rejoinRouteS,
                        int targetLaneId,
                        const WorldObjectState& obstacle,
                        const LaneChangeConfig& config,
                        const WorldState& world,
                        const PathGeneratorData& map,
                        const PathPlannerData& globalPath,
                        std::size_t nearestGlobalIndex) {
    const auto roadFound = map.roads.find(world.egoRoadId);
    if (roadFound == map.roads.end() || world.egoSectionIndex < 0 ||
        world.egoSectionIndex >=
            static_cast<int>(roadFound->second.sections.size())) return false;
    const Road& road = roadFound->second;
    const LaneSection& section = road.sections[world.egoSectionIndex];
    const double sectionEndS =
        world.egoSectionIndex + 1 < static_cast<int>(road.sections.size())
            ? road.sections[world.egoSectionIndex + 1].s
            : road.length;
    const auto targetLane = section.lanes.find(targetLaneId);
    if (targetLane == section.lanes.end() ||
        targetLane->second.type != "driving") return false;
    if (!laneChangeAllowed(section, world.egoLaneId, targetLaneId,
                           world.egoRoadS)) return false;

    const LaneKey targetKey{world.egoRoadId, world.egoSectionIndex,
                            targetLaneId};
    const LaneKey sourceKey{world.egoRoadId, world.egoSectionIndex,
                            world.egoLaneId};
    if (connectLanes(map, targetKey, sourceKey).empty()) return false;
    if (!targetLaneHasSafeGap(world, world.egoRoadId,
                              world.egoSectionIndex, targetLaneId,
                              config)) return false;

    const double startRouteS = world.egoPathS;
    const double leaveTargetS = obstacle.pathS + config.obstacleClearance;
    returnStartRouteS = leaveTargetS;
    rejoinRouteS = leaveTargetS + config.transitionLength;
    const double pathEndS = rejoinRouteS + 10.0;
    output.clear();

    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& original = globalPath.waypoints[index];
        if (original.s > pathEndS) break;
        if (original.roadId != world.egoRoadId ||
            original.laneId != world.egoLaneId ||
            original.roadS < section.s || original.roadS > sectionEndS) {
            return false;
        }

        double blend = 0.0;
        if (original.s <= startRouteS + config.transitionLength) {
            blend = smoothStep((original.s - startRouteS) /
                               config.transitionLength);
        } else if (original.s < leaveTargetS) {
            blend = 1.0;
        } else {
            blend = 1.0 - smoothStep((original.s - leaveTargetS) /
                                     config.transitionLength);
        }

        const RoadPose reference = roadToWorld(road, original.roadS);
        const double targetOffset =
            laneOffset(road, section, targetLaneId, original.roadS);
        const double targetX = reference.x -
                               targetOffset * std::sin(reference.yaw);
        const double targetY = reference.y +
                               targetOffset * std::cos(reference.yaw);
        Waypoint point = original;
        point.x = original.x + blend * (targetX - original.x);
        point.y = original.y + blend * (targetY - original.y);
        if (blend > 0.5) point.laneId = targetLaneId;
        output.push_back(point);
    }
    if (output.size() < 3 || output.back().s < rejoinRouteS) return false;
    const auto rejoinPoint = std::min_element(
        output.begin(), output.end(),
        [&](const Waypoint& leftPoint, const Waypoint& rightPoint) {
            return std::abs(leftPoint.s - rejoinRouteS) <
                   std::abs(rightPoint.s - rejoinRouteS);
        });
    if (rejoinPoint == output.end() ||
        !laneChangeAllowed(section, targetLaneId, world.egoLaneId,
                           rejoinPoint->roadS)) return false;

    for (std::size_t index = 0; index + 1 < output.size(); ++index) {
        output[index].yaw = std::atan2(output[index + 1].y - output[index].y,
                                      output[index + 1].x - output[index].x);
    }
    output.back().yaw = output[output.size() - 2].yaw;

    turnSignal = turnSignalTowardLane(world, map, targetLaneId);
    return true;
}

bool candidatePathIsSafe(const std::vector<Waypoint>& points,
                         const PlanningBehaviorConfig& config,
                         const LaneChangeConfig& laneChangeConfig,
                         const WorldState& world) {
    LocalPathData candidate;
    candidate.valid = points.size() >= 2;
    candidate.points = points;
    if (!candidate.valid || !world.egoPathValid) return false;

    // 일반 주행의 3초 예측만으로는 긴 회피 경로의 복귀 구간을 놓칠 수 있다.
    // 차선변경 최대속도로 경로 끝까지 가는 시간을 계산해 전체 기동을 검사한다.
    WorldState predictionWorld = world;
    predictionWorld.egoSpeed = std::max(
        {1.0, std::abs(world.egoSpeed),
         laneChangeConfig.maximumLaneChangeSpeed});
    const double remainingLength =
        std::max(0.0, points.back().s - world.egoPathS);
    const double fullManeuverHorizon =
        remainingLength / predictionWorld.egoSpeed +
        config.collisionPredictionStep;
    const double predictionHorizon =
        std::max(config.collisionPredictionHorizon, fullManeuverHorizon);

    PathSafetyResult safety;
    resetPathSafety(safety);
    checkCurrentCollisions(safety, world.egoObb, world.objects);
    checkPredictedCollisions(safety, predictionWorld, candidate,
                             predictionHorizon,
                             config.collisionPredictionStep);
    return !safety.collision && !safety.predictedCollision;
}

}  // namespace

bool tryPrepareObstacleLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex) {
    if (state.phase != LaneChangePhase::None || !world.egoLaneValid ||
        !world.egoPathValid) return false;
    const WorldObjectState* obstacle = findBlockingObstacle(world, config);
    if (obstacle == nullptr) return false;

    const std::array<int, 2> adjacent{{world.egoLaneId + 1,
                                       world.egoLaneId - 1}};
    for (int targetLaneId : adjacent) {
        if (targetLaneId == 0 ||
            (targetLaneId > 0) != (world.egoLaneId > 0)) continue;
        std::vector<Waypoint> candidate;
        const std::uint8_t signal =
            turnSignalTowardLane(world, map, targetLaneId);
        LaneKey reconnectLane;
        std::size_t reconnectIndex{};
        if (signal != 0 &&
            buildKeepTargetPath(candidate, reconnectLane, reconnectIndex,
                                targetLaneId, config, world, map,
                                globalPath, nearestGlobalIndex) &&
            candidatePathIsSafe(candidate, safetyConfig, config, world)) {
            state = {};
            state.phase = LaneChangePhase::Prepare;
            state.routeMode = LaneChangeRouteMode::KeepTargetLane;
            state.obstacleId = static_cast<int>(obstacle->raw.id);
            state.sourceRoadId = world.egoRoadId;
            state.sourceLaneId = world.egoLaneId;
            state.sourceSectionIndex = world.egoSectionIndex;
            state.targetLaneId = targetLaneId;
            state.reconnectLane = reconnectLane;
            state.reconnectLaneValid = true;
            state.reconnectGlobalIndex = reconnectIndex;
            state.turnSignal = signal;
            state.outwardTurnSignal = signal;
            state.startRouteS = world.egoPathS;
            state.rejoinRouteS = candidate.back().s;
            state.path = std::move(candidate);
            return true;
        }

        std::uint8_t returnCandidateSignal{};
        double returnStartS{};
        double rejoinS{};
        candidate.clear();
        if (!buildReturnCandidatePath(
                candidate, returnCandidateSignal, returnStartS, rejoinS,
                targetLaneId, *obstacle, config, world, map, globalPath,
                nearestGlobalIndex)) continue;
        if (!candidatePathIsSafe(candidate, safetyConfig, config, world)) {
            continue;
        }
        state = {};
        state.phase = LaneChangePhase::Prepare;
        state.routeMode = LaneChangeRouteMode::ReturnToSource;
        state.obstacleId = static_cast<int>(obstacle->raw.id);
        state.sourceRoadId = world.egoRoadId;
        state.sourceLaneId = world.egoLaneId;
        state.sourceSectionIndex = world.egoSectionIndex;
        state.targetLaneId = targetLaneId;
        state.turnSignal = returnCandidateSignal;
        state.outwardTurnSignal = returnCandidateSignal;
        state.returnTurnSignal = returnCandidateSignal == 1U ? 2U : 1U;
        state.startRouteS = world.egoPathS;
        state.returnStartRouteS = returnStartS;
        state.rejoinRouteS = rejoinS;
        state.path = std::move(candidate);
        return true;
    }
    return false;
}

void advanceLaneChangePhase(LaneChangeData& state,
                            const LaneChangeConfig& config,
                            const WorldState& world,
                            double dt,
                            bool executionBlocked,
                            bool candidatePathSafe) {
    if (state.phase == LaneChangePhase::None) return;
    if (state.phase == LaneChangePhase::Prepare) {
        if (executionBlocked || !candidatePathSafe) {
            state.phaseElapsed = 0.0;
            if (!candidatePathSafe) resetLaneChange(state);
            return;
        }
        state.phaseElapsed += std::max(0.0, dt);
        if (state.phaseElapsed >= config.prepareTime) {
            state.phase = LaneChangePhase::Execute;
            state.phaseElapsed = 0.0;
        }
        return;
    }
    if (state.phase == LaneChangePhase::Execute &&
        world.egoPathValid) {
        if (state.routeMode == LaneChangeRouteMode::KeepTargetLane) {
            state.waitingForReturn = false;
            if (world.egoPathS >=
                state.startRouteS + config.transitionLength) {
                state.turnSignal = 0;
            }
            if (state.reconnectLaneValid && world.egoLaneValid &&
                world.egoRoadId == state.reconnectLane.roadId &&
                world.egoSectionIndex == state.reconnectLane.sectionIndex &&
                world.egoLaneId == state.reconnectLane.laneId) {
                state.phase = LaneChangePhase::Settle;
                state.phaseElapsed = 0.0;
                state.turnSignal = 0;
            }
            return;
        }

        state.waitingForReturn = !candidatePathSafe;
        if (world.egoPathS >= state.returnStartRouteS - 5.0) {
            state.turnSignal = state.returnTurnSignal;
        }
        if (world.egoPathS >= state.rejoinRouteS - 1.0) {
            state.phase = LaneChangePhase::Settle;
            state.phaseElapsed = 0.0;
            state.waitingForReturn = false;
            state.turnSignal = 0;
        }
        return;
    }
    if (state.phase == LaneChangePhase::Settle) {
        state.phaseElapsed += std::max(0.0, dt);
        if (state.phaseElapsed >= config.settleTime) resetLaneChange(state);
    }
}

bool isAvoidedLaneChangeObject(const LaneChangeData& state, int objectId) {
    return state.phase == LaneChangePhase::Execute &&
           state.obstacleId == objectId;
}

void makeLaneChangeLocalPath(LocalPathData& output,
                             const LaneChangeData& state) {
    output = {};
    if (state.path.size() < 2) return;
    output.valid = true;
    output.beginIndex = state.nearestIndex;
    output.endIndex = state.path.size() - 1;
    output.points = state.path;
}

PathPlan trackLaneChangePath(LaneChangeData& state,
                             const PathPlannerConfig& plannerConfig,
                             const GtFrame& gt,
                             double speedLimit) {
    PathPlannerData temporary;
    temporary.config = plannerConfig;
    temporary.waypoints = state.path;
    temporary.nearestIndex = state.nearestIndex;
    PathPlan plan = runPurePursuit(temporary, gt);
    state.nearestIndex = temporary.nearestIndex;
    plan.goalReached = false;
    plan.targetSpeed = std::min(plan.targetSpeed, speedLimit);
    return plan;
}
