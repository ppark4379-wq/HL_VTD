#include "planning/local/lane_change_planner.hpp"

#include "logger/decision_trace.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>

namespace {

double smoothStep(double value) {
    const double t = std::clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

void resetLaneChange(LaneChangeData& state) {
    state = {};
}

const WorldObjectState* findBlockingObstacle(
    const WorldState& world, const LaneChangeConfig& config,
    DecisionTraceData* trace) {
    const WorldObjectState* result = nullptr;
    for (const auto& object : world.objects) {
        std::ostringstream detail;
        detail << "object=" << object.raw.id
               << " relative_s=" << object.relativeS
               << " relative_d=" << object.relativeD
               << " speed=" << object.raw.speed
               << " road=" << object.roadId
               << " section=" << object.sectionIndex
               << " lane=" << object.laneId;
        if (!object.isAhead || !object.isOnEgoPath ||
            std::abs(object.raw.speed) >= 0.3 ||
            object.relativeS < config.obstacleTriggerMinimumDistance ||
            object.relativeS > config.obstacleTriggerMaximumDistance) {
            detail << " result=reject reasons="
                   << (!object.isAhead ? "not_ahead;" : "")
                   << (!object.isOnEgoPath ? "not_on_ego_path;" : "")
                   << (std::abs(object.raw.speed) >= 0.3 ? "moving;" : "")
                   << (object.relativeS < config.obstacleTriggerMinimumDistance
                           ? "too_close_for_new_lane_change;" : "")
                   << (object.relativeS > config.obstacleTriggerMaximumDistance
                           ? "beyond_trigger_range;" : "");
            traceDecision(trace, "obstacle_scan", "candidate_checked",
                          detail.str());
            continue;
        }
        detail << " result=static_blocker_candidate";
        traceDecision(trace, "obstacle_scan", "candidate_checked",
                      detail.str());
        if (result == nullptr || object.relativeS < result->relativeS) {
            result = &object;
        }
    }
    if (result == nullptr) {
        traceDecision(trace, "obstacle_scan", "no_blocker",
                      "no static object satisfies path/speed/12-45m trigger");
    } else {
        std::ostringstream detail;
        detail << "object=" << result->raw.id
               << " relative_s=" << result->relativeS
               << " relative_d=" << result->relativeD
               << " road=" << result->roadId
               << " section=" << result->sectionIndex
               << " lane=" << result->laneId;
        traceDecision(trace, "obstacle_scan", "blocker_selected", detail.str());
    }
    return result;
}

bool targetLaneHasSafeGap(const WorldState& world,
                          int roadId,
                          int sectionIndex,
                          int targetLaneId,
                          const LaneChangeConfig& config,
                          DecisionTraceData* trace) {
    bool foundObject = false;
    for (const auto& object : world.objects) {
        if (!object.laneValid || object.roadId != roadId ||
            object.sectionIndex != sectionIndex ||
            object.laneId != targetLaneId || !object.pathValid) continue;
        foundObject = true;
        const double gap = object.pathS - world.egoPathS;
        const double objectLongitudinalSpeed =
            static_cast<double>(object.raw.speed) *
            std::cos(object.raw.heading - world.ego.heading);
        const double closingSpeed = gap >= 0.0
                                        ? world.egoSpeed - objectLongitudinalSpeed
                                        : objectLongitudinalSpeed - world.egoSpeed;
        const double ttc = calculateTTC(std::abs(gap), closingSpeed);
        std::ostringstream detail;
        detail << "target=" << roadId << '/' << sectionIndex << '/'
               << targetLaneId << " object=" << object.raw.id
               << " gap=" << gap
               << " object_longitudinal_speed=" << objectLongitudinalSpeed
               << " closing_speed=" << closingSpeed
               << " ttc=" << ttc;
        if (gap >= 0.0 && gap < config.minimumTargetAheadGap) {
            detail << " result=reject reason=ahead_gap_below_"
                   << config.minimumTargetAheadGap;
            traceDecision(trace, "target_lane", "gap_ttc_checked", detail.str());
            return false;
        }
        // 뒤쪽 간격 기준은 "따라오는 차"를 위한 것이다. 이미 지나쳐 온 정지
        // 물체는 접근해 오지 않으므로 이 기준을 적용하지 않는다. 적용하면
        // 옆 차선의 정지 장애물을 지난 직후 12 m 동안 회피가 막힌다.
        // 옆으로 나가다 실제로 스칠 위험은 candidatePathIsSafe의 OBB
        // 예측 검사가 그대로 걸러 준다.
        const bool objectIsStatic = std::abs(object.raw.speed) < 0.3;
        if (gap < 0.0 && !objectIsStatic &&
            -gap < config.minimumTargetBehindGap) {
            detail << " result=reject reason=behind_gap_below_"
                   << config.minimumTargetBehindGap;
            traceDecision(trace, "target_lane", "gap_ttc_checked", detail.str());
            return false;
        }
        if (ttc < config.minimumTargetTTC) {
            detail << " result=reject reason=ttc_below_"
                   << config.minimumTargetTTC;
            traceDecision(trace, "target_lane", "gap_ttc_checked", detail.str());
            return false;
        }
        detail << " result=pass";
        traceDecision(trace, "target_lane", "gap_ttc_checked", detail.str());
    }
    if (!foundObject) {
        std::ostringstream detail;
        detail << "target=" << roadId << '/' << sectionIndex << '/'
               << targetLaneId
               << " result=pass reason=no_object_matched_target_lane";
        traceDecision(trace, "target_lane", "gap_ttc_checked", detail.str());
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

std::optional<int> adjacentDrivingLaneOnSide(
    const Road& road,
    const LaneSection& section,
    int sourceLaneId,
    double roadS,
    double drivingYaw,
    bool targetOnLeft);

std::optional<bool> laneIsOnDrivingLeft(const Road& road,
                                        const LaneSection& section,
                                        int sourceLaneId,
                                        int targetLaneId,
                                        double roadS);

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
                         std::size_t nearestGlobalIndex,
                         DecisionTraceData* trace) {
    const LaneKey targetKey{world.egoRoadId, world.egoSectionIndex,
                            targetLaneId};
    if (!findNextGlobalRoadLane(reconnectLane, reconnectGlobalIndex,
                                globalPath, map, nearestGlobalIndex,
                                world.egoRoadId)) {
        traceDecision(trace, "target_lane", "keep_route_rejected",
                      "reason=no_next_global_road_lane");
        return false;
    }
    {
        std::ostringstream detail;
        detail << "target=" << targetKey.roadId << '/' << targetKey.sectionIndex
               << '/' << targetKey.laneId << " reconnect="
               << reconnectLane.roadId << '/' << reconnectLane.sectionIndex
               << '/' << reconnectLane.laneId
               << " global_index=" << reconnectGlobalIndex;
        traceDecision(trace, "target_lane", "keep_route_goal", detail.str());
    }
    const std::vector<LaneKey> forwardRoute =
        findForwardLaneRoute(map, targetKey, reconnectLane);
    if (forwardRoute.empty()) {
        traceDecision(trace, "target_lane", "keep_route_rejected",
                      "reason=target_lane_cannot_reach_exact_next_global_lane_without_lateral_or_detour_edges");
        return false;
    }

    const auto roadFound = map.roads.find(world.egoRoadId);
    if (roadFound == map.roads.end()) return false;
    const Road& road = roadFound->second;
    const LaneSection& section = road.sections[world.egoSectionIndex];
    if (!laneChangeAllowed(section, world.egoLaneId, targetLaneId,
                           world.egoRoadS)) {
        traceDecision(trace, "target_lane", "keep_route_rejected",
                      "reason=road_mark_forbids_lane_change_at_ego_road_s");
        return false;
    }
    if (!targetLaneHasSafeGap(world, world.egoRoadId,
                              world.egoSectionIndex, targetLaneId,
                              config, trace)) return false;

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


// 회피 기동은 "Ego가 지금 있는 도로"가 아니라 "장애물이 있는 도로/차선"을
// 기준으로 세운다. 장애물 앞 도로가 교차로 연결로처럼 편도 1차선이어도,
// 장애물이 놓인 도로에 옆 차선이 있으면 미리 계획해 두었다가 그 도로에
// 들어서는 순간 횡이동을 시작할 수 있다.
bool buildReturnCandidatePath(std::vector<Waypoint>& output,
                        std::uint8_t& turnSignal,
                        double& returnStartRouteS,
                        double& rejoinRouteS,
                        int targetObstacleLaneId,
                        int targetEgoLaneId,
                        bool targetOnLeft,
                        const WorldObjectState& obstacle,
                        const LaneChangeConfig& config,
                        const WorldState& world,
                        const PathGeneratorData& map,
                        const PathPlannerData& globalPath,
                        std::size_t nearestGlobalIndex,
                        DecisionTraceData* trace) {
    if (!obstacle.laneValid || !obstacle.pathValid) return false;
    const auto obstacleRoadFound = map.roads.find(obstacle.roadId);
    if (obstacleRoadFound == map.roads.end() || obstacle.sectionIndex < 0 ||
        obstacle.sectionIndex >=
            static_cast<int>(obstacleRoadFound->second.sections.size())) {
        return false;
    }
    const Road& obstacleRoad = obstacleRoadFound->second;
    const LaneSection& obstacleSection =
        obstacleRoad.sections[obstacle.sectionIndex];

    // 목표 차선은 장애물이 있는 도로/구간에서 판정한다.
    const auto targetLane =
        obstacleSection.lanes.find(targetObstacleLaneId);
    if (targetLane == obstacleSection.lanes.end() ||
        targetLane->second.type != "driving") {
        return false;
    }
    if (!laneChangeAllowed(obstacleSection, obstacle.laneId,
                           targetObstacleLaneId, obstacle.roadS)) {
        return false;
    }
    const LaneKey targetKey{obstacle.roadId, obstacle.sectionIndex,
                             targetObstacleLaneId};
    const LaneKey sourceKey{obstacle.roadId, obstacle.sectionIndex,
                            obstacle.laneId};
    if (connectLanes(map, targetKey, sourceKey).empty()) {
        return false;
    }
    if (!targetLaneHasSafeGap(world, obstacle.roadId, obstacle.sectionIndex,
                              targetObstacleLaneId, config, trace)) {
        return false;
    }
    if ((world.egoRoadId != obstacle.roadId ||
         world.egoSectionIndex != obstacle.sectionIndex) &&
        !targetLaneHasSafeGap(world, world.egoRoadId, world.egoSectionIndex,
                              targetEgoLaneId, config, trace)) {
        return false;
    }

    // 경로에서 장애물의 도로가 시작되는 지점. 여기부터 목표 차선이 존재한다.
    double targetRoadStartS = std::numeric_limits<double>::infinity();
    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& point = globalPath.waypoints[index];
        if (point.roadId == obstacle.roadId) {
            targetRoadStartS = point.s;
            break;
        }
        if (point.s > obstacle.pathS) break;
    }
    if (!std::isfinite(targetRoadStartS)) {
        return false;
    }

    // 횡이동은 목표 차선이 생기는 지점부터 시작하고, 장애물에 닿기 전에 끝낸다.
    // 횡이동은 장애물 직전에서 시작한다. Ego 위치에서 곧바로 시작하면
    // 장애물이 45 m 앞일 때 45 m 내내 옆 차선에 걸친 경로가 되고, 그 긴
    // 구간에서 lane section 경계나 경로 자체의 차선 전환을 만나 후보 경로가
    // 끊긴다. 실제 주행 관점에서도 미리 45 m 전부터 나갈 이유가 없다.
    const double shiftStartS = std::max(
        {world.egoPathS, targetRoadStartS,
         obstacle.pathS - config.transitionLength});
    const double availableShift = obstacle.pathS - shiftStartS;
    if (availableShift <= 0.5) {
        return false;
    }
    const double shiftLength = std::min(config.transitionLength, availableShift);

    // 회피 방향은 장애물 구간에서 정한 물리적 좌/우로 고정한다.
    // 각 경로 지점에서는 그 방향의 인접 driving lane을 다시 찾는다.
    const double leaveTargetS = obstacle.pathS + config.obstacleClearance;
    returnStartRouteS = leaveTargetS;
    rejoinRouteS = leaveTargetS + config.transitionLength;
    const double pathEndS = rejoinRouteS + 10.0;
    output.clear();

    const Road* rejoinRoadPtr = nullptr;
    const LaneSection* rejoinSectionPtr = nullptr;
    int rejoinRouteLaneId = 0;
    double rejoinPointRoadS = 0.0;
    double rejoinBestDelta = std::numeric_limits<double>::infinity();

    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& original = globalPath.waypoints[index];
        if (original.s > pathEndS) break;
        // nearestIndex가 Ego보다 약간 뒤를 가리킬 수 있다.
        if (original.s < world.egoPathS) continue;

        // 이 웨이포인트가 속한 road/section. 횡이동 판정과 복귀 판정이
        // 같은 구간 기준을 쓰도록 여기서 한 번만 구한다.
        const Road* pointRoadPtr = nullptr;
        const LaneSection* pointSectionPtr = nullptr;
        {
            const auto found = map.roads.find(original.roadId);
            if (found != map.roads.end() && !found->second.sections.empty()) {
                const int index2 = sectionIndexAt(found->second, original.roadS);
                if (index2 >= 0 &&
                    index2 < static_cast<int>(found->second.sections.size())) {
                    pointRoadPtr = &found->second;
                    pointSectionPtr = &found->second.sections[index2];
                }
            }
        }

        double blend = 0.0;
        if (original.s <= shiftStartS) {
            blend = 0.0;
        } else if (original.s <= shiftStartS + shiftLength) {
            blend = smoothStep((original.s - shiftStartS) / shiftLength);
        } else if (original.s < leaveTargetS) {
            blend = 1.0;
        } else {
            blend = 1.0 - smoothStep((original.s - leaveTargetS) /
                                     config.transitionLength);
        }

        // 아직 옆으로 나가지 않는 구간은 기준 경로를 그대로 쓴다. 이 구간이
        // 편도 1차선(연결로)이어도 문제되지 않는다.
        if (blend <= 1e-6) {
            output.push_back(original);
        } else {
            // 횡이동 구간은 웨이포인트마다 자기 road/section에서 목표 차선을
            // 찾는다. lane section 경계(road 429는 6개로 쪼개져 있다)나
            // 경로 자체의 차선 전환을 만나도 기동이 끊기지 않아야 한다.
            if (pointRoadPtr == nullptr || pointSectionPtr == nullptr) break;
            const Road& pointRoad = *pointRoadPtr;
            const LaneSection& pointSection = *pointSectionPtr;

            // 같은 road 위에서는 목표 차선을 고정한다. OpenDRIVE에서 한 road의
            // lane id는 section이 달라도 같은 차선을 가리키므로(구간마다 가장자리
            // 차선이 늘거나 줄 뿐이다) 번호를 그대로 유지하면 된다.
            // 여기서 "경로차선 + 1"로 다시 계산하면, 경로가 스스로 바깥 차선으로
            // laneSection 경계에서는 같은 물리 차선의 ID가 바뀔 수 있으므로
            // 장애물 구간의 목표 ID를 복사하지 않고 현재 지점에서 다시 매핑한다.
            const auto pointTargetLane = adjacentDrivingLaneOnSide(
                pointRoad, pointSection, original.laneId, original.roadS,
                original.yaw, targetOnLeft);
            if (!pointTargetLane) break;
            const int pointTargetLaneId = *pointTargetLane;
            // 경로가 이미 목표 차선을 쓰고 있으면 그 지점에서는 차선을 바꾸는
            // 것이 아니다. 도로표시 검사는 실제로 차선을 넘을 때만 한다.
            const bool outwardTransition =
                original.s > shiftStartS &&
                original.s <= shiftStartS + shiftLength;
            const bool returnTransition =
                original.s >= leaveTargetS && original.s < rejoinRouteS;
            if ((outwardTransition || returnTransition) &&
                !laneChangeAllowed(pointSection, original.laneId,
                                   pointTargetLaneId, original.roadS)) break;

            const RoadPose reference = roadToWorld(pointRoad, original.roadS);
            const double targetOffset = laneOffset(
                pointRoad, pointSection, pointTargetLaneId, original.roadS);
            const double targetX =
                reference.x - targetOffset * std::sin(reference.yaw);
            const double targetY =
                reference.y + targetOffset * std::cos(reference.yaw);
            Waypoint point = original;
            point.x = original.x + blend * (targetX - original.x);
            point.y = original.y + blend * (targetY - original.y);
            if (blend > 0.5) point.laneId = pointTargetLaneId;
            output.push_back(point);
        }

        const double rejoinDelta = std::abs(original.s - rejoinRouteS);
        if (rejoinDelta < rejoinBestDelta && pointSectionPtr != nullptr) {
            rejoinBestDelta = rejoinDelta;
            rejoinRoadPtr = pointRoadPtr;
            rejoinSectionPtr = pointSectionPtr;
            rejoinRouteLaneId = original.laneId;
            rejoinPointRoadS = original.roadS;
        }
    }
    if (output.size() < 3 || output.back().s < rejoinRouteS) {
        return false;
    }
    if (rejoinRoadPtr == nullptr || rejoinSectionPtr == nullptr) return false;
    // 복귀(회피차선 -> 원래차선) 판정. 회피 차선은 경로 생성 루프와 같은
    // 규칙으로 구하고, 경로가 이미 그 차선을 쓰고 있으면 바꿀 것이 없으므로
    // 도로표시 검사를 하지 않는다.
    const RoadPose rejoinReference =
        roadToWorld(*rejoinRoadPtr, rejoinPointRoadS);
    const double rejoinDrivingYaw =
        rejoinRouteLaneId < 0
            ? rejoinReference.yaw
            : normalizeAngle(rejoinReference.yaw + 3.14159265358979323846);
    const auto rejoinAvoidLane = adjacentDrivingLaneOnSide(
        *rejoinRoadPtr, *rejoinSectionPtr, rejoinRouteLaneId,
        rejoinPointRoadS, rejoinDrivingYaw, targetOnLeft);
    if (!rejoinAvoidLane ||
        !laneChangeAllowed(*rejoinSectionPtr, *rejoinAvoidLane,
                           rejoinRouteLaneId, rejoinPointRoadS)) {
        return false;
    }

    for (std::size_t index = 0; index + 1 < output.size(); ++index) {
        output[index].yaw = std::atan2(output[index + 1].y - output[index].y,
                                      output[index + 1].x - output[index].x);
    }
    output.back().yaw = output[output.size() - 2].yaw;

    // 최초에 판정한 물리적 회피 방향을 방향지시등에도 그대로 사용한다.
    turnSignal = targetOnLeft ? 1U : 2U;
    return true;
}


bool candidatePathIsSafe(const std::vector<Waypoint>& points,
                         const PlanningBehaviorConfig& config,
                          const LaneChangeConfig& laneChangeConfig,
                          const WorldState& world,
                          DecisionTraceData* trace) {
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
    {
        std::ostringstream detail;
        detail << "points=" << points.size()
               << " remaining_length=" << remainingLength
               << " assumed_ego_speed=" << predictionWorld.egoSpeed
               << " horizon=" << predictionHorizon
               << " step=" << config.collisionPredictionStep
               << " current_collision=" << safety.collision
               << " predicted_collision=" << safety.predictedCollision
               << " predicted_ttc=" << safety.predictedCollisionTTC
               << " risk_object=" << safety.riskObjectId
               << " result="
               << (!safety.collision && !safety.predictedCollision
                       ? "pass" : "reject");
        traceDecision(trace, "candidate_obb", "full_path_checked", detail.str());
    }
    return !safety.collision && !safety.predictedCollision;
}

// laneId 숫자는 laneSection 경계에서 바뀔 수 있다. 따라서 장애물 구간의
// `laneId +/- 1`을 Ego 구간에 그대로 복사하지 않고, 실제 월드 좌표에서
// 주행방향 기준 같은 쪽(왼쪽/오른쪽)에 있는 인접 driving lane을 다시 찾는다.
std::optional<int> adjacentDrivingLaneOnSide(
    const Road& road,
    const LaneSection& section,
    int sourceLaneId,
    double roadS,
    double drivingYaw,
    bool targetOnLeft) {
    const auto source = section.lanes.find(sourceLaneId);
    if (source == section.lanes.end() || source->second.type != "driving") {
        return std::nullopt;
    }
    const RoadPose reference = roadToWorld(road, roadS);
    const double sourceOffset = laneOffset(road, section, sourceLaneId, roadS);
    const double sourceX = reference.x - sourceOffset * std::sin(reference.yaw);
    const double sourceY = reference.y + sourceOffset * std::cos(reference.yaw);
    for (const int candidateId : {sourceLaneId + 1, sourceLaneId - 1}) {
        if (candidateId == 0 ||
            (candidateId > 0) != (sourceLaneId > 0)) {
            continue;
        }
        const auto candidate = section.lanes.find(candidateId);
        if (candidate == section.lanes.end() ||
            candidate->second.type != "driving") {
            continue;
        }
        const double candidateOffset =
            laneOffset(road, section, candidateId, roadS);
        const double candidateX =
            reference.x - candidateOffset * std::sin(reference.yaw);
        const double candidateY =
            reference.y + candidateOffset * std::cos(reference.yaw);
        const double leftDistance =
            -std::sin(drivingYaw) * (candidateX - sourceX) +
             std::cos(drivingYaw) * (candidateY - sourceY);
        if ((targetOnLeft && leftDistance > 0.1) ||
            (!targetOnLeft && leftDistance < -0.1)) {
            return candidateId;
        }
    }
    return std::nullopt;
}

std::optional<bool> laneIsOnDrivingLeft(const Road& road,
                                        const LaneSection& section,
                                        int sourceLaneId,
                                        int targetLaneId,
                                        double roadS) {
    const RoadPose reference = roadToWorld(road, roadS);
    const double drivingYaw = sourceLaneId < 0
                                  ? reference.yaw
                                  : normalizeAngle(reference.yaw + 3.14159265358979323846);
    const auto left = adjacentDrivingLaneOnSide(
        road, section, sourceLaneId, roadS, drivingYaw, true);
    if (left && *left == targetLaneId) return true;
    const auto right = adjacentDrivingLaneOnSide(
        road, section, sourceLaneId, roadS, drivingYaw, false);
    if (right && *right == targetLaneId) return false;
    return std::nullopt;
}

}  // namespace

bool tryPrepareObstacleLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    DecisionTraceData* trace) {
    if (state.phase != LaneChangePhase::None || !world.egoLaneValid ||
        !world.egoPathValid) return false;
    const WorldObjectState* obstacle = findBlockingObstacle(world, config, trace);
    if (obstacle == nullptr) return false;

    // 후보 차선은 Ego가 아니라 "장애물이 놓인 차선"의 양옆이다. 장애물 앞
    // 구간이 편도 1차선이어도, 장애물이 있는 도로에 옆 차선이 있으면 회피할 수
    // 있어야 한다.
    const int obstacleLaneId =
        obstacle->laneValid ? obstacle->laneId : world.egoLaneId;
    std::array<int, 2> adjacent{{obstacleLaneId + 1, obstacleLaneId - 1}};

    // 양쪽이 모두 비어 있으면 아무 쪽이나 고르면 안 된다. 회피를 끝낸 뒤
    // 기준 경로가 지나가는 차선 쪽으로 나가야, 복귀한 다음 다시 가로질러
    // 갈 필요가 없다. 그 사이에 장애물이 하나만 더 있어도 경로 차선으로
    // 못 넘어가 교차로 진입을 놓칠 수 있다.
    // 기동을 마친 지점에서 경로가 쓰는 차선을 찾아, 그쪽에 가까운 후보를
    // 먼저 시도한다.
    {
        // 한 지점만 보면 경로가 아직 같은 차선이라 양쪽이 동점이 되어 판단이
        // 안 선다. 장애물 이후 구간에서 경로가 "처음으로 옮겨 가는 차선"을
        // 찾아 그쪽을 선호한다. 교차로 진입 차선처럼 수십 m 뒤에 나오는
        // 차선 변경까지 반영해야, 반대쪽으로 나갔다가 되돌아오는 낭비와
        // 그 사이 장애물에 막혀 경로 차선을 놓치는 위험을 피할 수 있다.
        constexpr double kRouteLaneLookahead = 150.0;
        int routeLaneAhead = obstacleLaneId;
        for (std::size_t index = nearestGlobalIndex;
             index < globalPath.waypoints.size(); ++index) {
            const Waypoint& point = globalPath.waypoints[index];
            if (point.s < obstacle->pathS) continue;
            if (point.s > obstacle->pathS + kRouteLaneLookahead) break;
            if (point.laneId != obstacleLaneId && point.laneId != 0 &&
                (point.laneId > 0) == (obstacleLaneId > 0)) {
                routeLaneAhead = point.laneId;
                break;
            }
        }
        if (std::abs(adjacent[1] - routeLaneAhead) <
            std::abs(adjacent[0] - routeLaneAhead)) {
            std::swap(adjacent[0], adjacent[1]);
        }
    }

    const auto obstacleRoadFound = map.roads.find(obstacle->roadId);
    const auto egoRoadFound = map.roads.find(world.egoRoadId);
    if (obstacleRoadFound == map.roads.end() ||
        egoRoadFound == map.roads.end() || obstacle->sectionIndex < 0 ||
        obstacle->sectionIndex >= static_cast<int>(
            obstacleRoadFound->second.sections.size()) ||
        world.egoSectionIndex < 0 ||
        world.egoSectionIndex >= static_cast<int>(
            egoRoadFound->second.sections.size())) {
        traceDecision(trace, "lane_change", "no_safe_candidate",
                      "reason=invalid_ego_or_obstacle_road_section");
        return false;
    }
    const Road& obstacleRoad = obstacleRoadFound->second;
    const LaneSection& obstacleSection =
        obstacleRoad.sections[obstacle->sectionIndex];
    const Road& egoRoad = egoRoadFound->second;
    const LaneSection& egoSection = egoRoad.sections[world.egoSectionIndex];

    for (int targetObstacleLaneId : adjacent) {
        if (targetObstacleLaneId == 0 ||
            (targetObstacleLaneId > 0) != (obstacleLaneId > 0)) {
            traceDecision(trace, "lane_change", "target_candidate_rejected",
                          "reason=center_or_opposite_direction_lane");
            continue;
        }

        const auto targetOnLeft = laneIsOnDrivingLeft(
            obstacleRoad, obstacleSection, obstacleLaneId,
            targetObstacleLaneId, obstacle->roadS);
        if (!targetOnLeft) {
            traceDecision(trace, "lane_change", "target_candidate_rejected",
                          "reason=not_an_adjacent_driving_lane_at_obstacle_section");
            continue;
        }
        const auto targetEgoLane = adjacentDrivingLaneOnSide(
            egoRoad, egoSection, world.egoLaneId, world.egoRoadS,
            world.ego.heading, *targetOnLeft);
        if (!targetEgoLane) {
            traceDecision(trace, "lane_change", "target_candidate_rejected",
                          "reason=same_physical_side_has_no_adjacent_driving_lane_at_ego_section");
            continue;
        }
        const int targetEgoLaneId = *targetEgoLane;
        {
            std::ostringstream detail;
            detail << "obstacle=" << obstacle->raw.id
                   << " obstacle_source_lane=" << obstacleLaneId
                   << " obstacle_target_lane=" << targetObstacleLaneId
                   << " ego_source_lane=" << world.egoLaneId
                   << " ego_target_lane=" << targetEgoLaneId
                   << " side=" << (*targetOnLeft ? "left" : "right");
            traceDecision(trace, "lane_change", "target_candidate_started",
                          detail.str());
        }
        std::vector<Waypoint> candidate;
        const std::uint8_t signal =
            turnSignalTowardLane(world, map, targetEgoLaneId);
        LaneKey reconnectLane;
        std::size_t reconnectIndex{};
        if (signal != 0 &&
            buildKeepTargetPath(candidate, reconnectLane, reconnectIndex,
                                targetEgoLaneId, config, world, map,
                                globalPath, nearestGlobalIndex, trace) &&
            candidatePathIsSafe(candidate, safetyConfig, config, world,
                                trace)) {
            state = {};
            state.phase = LaneChangePhase::Prepare;
            state.routeMode = LaneChangeRouteMode::KeepTargetLane;
            state.obstacleId = static_cast<int>(obstacle->raw.id);
            state.sourceRoadId = world.egoRoadId;
            state.sourceLaneId = world.egoLaneId;
            state.sourceSectionIndex = world.egoSectionIndex;
            state.targetLaneId = targetEgoLaneId;
            state.reconnectLane = reconnectLane;
            state.reconnectLaneValid = true;
            state.reconnectGlobalIndex = reconnectIndex;
            state.turnSignal = signal;
            state.outwardTurnSignal = signal;
            state.startRouteS = world.egoPathS;
            state.rejoinRouteS = candidate.back().s;
            state.path = std::move(candidate);
            traceDecision(trace, "lane_change", "candidate_selected",
                          "mode=keep_target_lane phase=prepare");
            return true;
        }

        traceDecision(trace, "lane_change", "return_candidate_started",
                      "keep-target route unavailable or unsafe; checking out-and-back route");

        std::uint8_t returnCandidateSignal{};
        double returnStartS{};
        double rejoinS{};
        candidate.clear();
        if (!buildReturnCandidatePath(
                candidate, returnCandidateSignal, returnStartS, rejoinS,
                targetObstacleLaneId, targetEgoLaneId, *targetOnLeft,
                *obstacle, config, world, map, globalPath,
                nearestGlobalIndex, trace)) {
            traceDecision(trace, "lane_change", "return_candidate_rejected",
                          "reason=route_geometry_gap_or_road_mark_check_failed");
            continue;
        }
        if (!candidatePathIsSafe(candidate, safetyConfig, config, world,
                                 trace)) {
            traceDecision(trace, "lane_change", "return_candidate_rejected",
                          "reason=obb_collision_prediction");
            continue;
        }
        state = {};
        state.phase = LaneChangePhase::Prepare;
        state.routeMode = LaneChangeRouteMode::ReturnToSource;
        state.obstacleId = static_cast<int>(obstacle->raw.id);
        state.sourceRoadId = world.egoRoadId;
        state.sourceLaneId = world.egoLaneId;
        state.sourceSectionIndex = world.egoSectionIndex;
        state.targetLaneId = targetEgoLaneId;
        state.turnSignal = returnCandidateSignal;
        state.outwardTurnSignal = returnCandidateSignal;
        state.returnTurnSignal = returnCandidateSignal == 1U ? 2U : 1U;
        state.startRouteS = world.egoPathS;
        state.returnStartRouteS = returnStartS;
        state.rejoinRouteS = rejoinS;
        state.path = std::move(candidate);
        traceDecision(trace, "lane_change", "candidate_selected",
                      "mode=return_to_source phase=prepare");
        return true;
    }
    traceDecision(trace, "lane_change", "no_safe_candidate",
                  "all adjacent lane candidates were rejected");
    return false;
}

void advanceLaneChangePhase(LaneChangeData& state,
                            const LaneChangeConfig& config,
                            const WorldState& world,
                            double dt,
                            bool executionBlocked,
                            bool candidatePathSafe,
                            DecisionTraceData* trace) {
    if (state.phase == LaneChangePhase::None) return;
    if (state.phase == LaneChangePhase::Prepare) {
        if (executionBlocked || !candidatePathSafe) {
            std::ostringstream detail;
            detail << "execution_blocked=" << executionBlocked
                   << " candidate_path_safe=" << candidatePathSafe;
            traceDecision(trace, "lane_change", "prepare_wait_or_cancel",
                          detail.str());
            state.phaseElapsed = 0.0;
            if (!candidatePathSafe) {
                traceDecision(trace, "lane_change", "phase_transition",
                              "prepare->none reason=candidate_became_unsafe");
                resetLaneChange(state);
            }
            return;
        }
        state.phaseElapsed += std::max(0.0, dt);
        if (state.phaseElapsed >= config.prepareTime) {
            state.phase = LaneChangePhase::Execute;
            state.phaseElapsed = 0.0;
            traceDecision(trace, "lane_change", "phase_transition",
                          "prepare->execute");
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
                traceDecision(trace, "lane_change", "phase_transition",
                              "execute->settle mode=keep_target_lane exact_reconnect_lane_reached");
            }
            return;
        }

        const bool wasWaiting = state.waitingForReturn;
        state.waitingForReturn = !candidatePathSafe;
        if (state.waitingForReturn != wasWaiting) {
            traceDecision(trace, "lane_change", "return_wait_changed",
                          state.waitingForReturn
                              ? "waiting=true reason=remaining_return_path_unsafe"
                              : "waiting=false reason=remaining_return_path_safe");
        }
        if (world.egoPathS >= state.returnStartRouteS - 5.0) {
            state.turnSignal = state.returnTurnSignal;
        }
        if (world.egoPathS >= state.rejoinRouteS - 1.0) {
            state.phase = LaneChangePhase::Settle;
            state.phaseElapsed = 0.0;
            state.waitingForReturn = false;
            state.turnSignal = 0;
            traceDecision(trace, "lane_change", "phase_transition",
                          "execute->settle mode=return_to_source");
        }
        return;
    }
    if (state.phase == LaneChangePhase::Settle) {
        state.phaseElapsed += std::max(0.0, dt);
        if (state.phaseElapsed >= config.settleTime) {
            traceDecision(trace, "lane_change", "phase_transition",
                          "settle->none maneuver_complete");
            resetLaneChange(state);
        }
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
                             double speedLimit,
                             double egoSpeed) {
    PathPlannerData temporary;
    temporary.config = plannerConfig;
    temporary.waypoints = state.path;
    temporary.nearestIndex = state.nearestIndex;
    PathPlan plan = runPurePursuit(temporary, gt, egoSpeed);
    state.nearestIndex = temporary.nearestIndex;
    plan.goalReached = false;
    plan.targetSpeed = std::min(plan.targetSpeed, speedLimit);
    return plan;
}
