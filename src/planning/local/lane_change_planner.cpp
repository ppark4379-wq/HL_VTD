#include "planning/local/lane_change_planner.hpp"

#include "logger/decision_trace.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <vector>

namespace {

bool hasForwardLaneConnection(const PathGeneratorData& map,
                              const LaneKey& start,
                              const LaneKey& goal);

double smoothStep(double value) {
    const double t = std::clamp(value, 0.0, 1.0);
    // 위치뿐 아니라 속도와 가속도까지 양 끝에서 0이 되는 quintic blend.
    // cubic blend의 곡률 hand-off로 생기던 순간 조향 변화를 줄인다.
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

void resetLaneChange(LaneChangeData& state) {
    state = {};
}

const WorldObjectState* findBlockingObstacle(
    const WorldState& world, const LaneChangeConfig& config,
    const PathGeneratorData& map,
    const std::vector<int>* confirmedStationaryObjectIds,
    DecisionTraceData* trace) {
    const WorldObjectState* result = nullptr;
    double resultDistance = std::numeric_limits<double>::infinity();
    for (const auto& object : world.objects) {
        // 도로 안에서 자유 차선을 타면 global route의 차선과 Ego의
        // 실제 차선이 달라질 수 있다. 이때 isOnEgoPath만 믿으면 현재
        // 차선 중앙에 있는 정지차량도 회피 검색에서 누락된다.
        const bool onCurrentPhysicalLane =
            world.egoLaneValid && object.laneValid &&
            object.roadId == world.egoRoadId &&
            object.sectionIndex == world.egoSectionIndex &&
            object.laneId == world.egoLaneId;
        // lane section 경계에서 차선 번호가 바뀌거나 차선이 넓어지고
        // 좁아져도, 현재 물리 차선의 종방향 successor라면 같은 전방
        // 주행 회랑이다. exact section/lane 비교만 하면 다음 section의
        // 연속 장애물을 회피 완료 직전까지 보지 못한다.
        const bool onCurrentLaneContinuation =
            world.egoLaneValid && object.laneValid &&
            object.roadId == world.egoRoadId &&
            hasForwardLaneConnection(
                map,
                {world.egoRoadId, world.egoSectionIndex, world.egoLaneId},
                {object.roadId, object.sectionIndex, object.laneId});
        const double longitudinalDistance = object.relativeS;
        bool physicallyAhead = false;
        if (onCurrentPhysicalLane) {
            const double physicalLongitudinalDistance = world.egoLaneId < 0
                ? object.roadS - world.egoRoadS
                : world.egoRoadS - object.roadS;
            physicallyAhead = physicalLongitudinalDistance > 0.0;
        }
        const bool onTrackedLane = object.isOnEgoPath ||
                                   onCurrentPhysicalLane ||
                                   onCurrentLaneContinuation;
        const bool ahead = object.isAhead || physicallyAhead;
        const bool stationaryConfirmed =
            std::abs(object.raw.speed) < 0.3 &&
            (confirmedStationaryObjectIds == nullptr ||
             std::find(confirmedStationaryObjectIds->begin(),
                       confirmedStationaryObjectIds->end(),
                       static_cast<int>(object.raw.id)) !=
                 confirmedStationaryObjectIds->end());
        std::ostringstream detail;
        detail << "object=" << object.raw.id
               << " relative_s=" << object.relativeS
               << " relative_d=" << object.relativeD
               << " speed=" << object.raw.speed
               << " road=" << object.roadId
               << " section=" << object.sectionIndex
               << " lane=" << object.laneId;
        if (!ahead || !onTrackedLane ||
            !stationaryConfirmed ||
            longitudinalDistance < config.obstacleTriggerMinimumDistance ||
            longitudinalDistance > config.obstacleTriggerMaximumDistance) {
            detail << " result=reject reasons="
                   << (!ahead ? "not_ahead;" : "")
                   << (!onTrackedLane ? "not_on_tracked_lane;" : "")
                   << (!stationaryConfirmed
                           ? "moving_or_stationary_not_confirmed;" : "")
                   << (longitudinalDistance < config.obstacleTriggerMinimumDistance
                           ? "too_close_for_new_lane_change;" : "")
                   << (longitudinalDistance > config.obstacleTriggerMaximumDistance
                           ? "beyond_trigger_range;" : "");
            traceDecision(trace, "obstacle_scan", "candidate_checked",
                          detail.str());
            continue;
        }
        detail << " result=stationary_blocker_candidate";
        traceDecision(trace, "obstacle_scan", "candidate_checked",
                      detail.str());
        if (result == nullptr || longitudinalDistance < resultDistance) {
            result = &object;
            resultDistance = longitudinalDistance;
        }
    }
    if (result == nullptr) {
        traceDecision(trace, "obstacle_scan", "no_blocker",
                      "no confirmed stationary object satisfies path/distance trigger");
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
        const bool objectIsStatic = std::abs(object.raw.speed) < 0.3;
        std::ostringstream detail;
        detail << "target=" << roadId << '/' << sectionIndex << '/'
               << targetLaneId << " object=" << object.raw.id
               << " gap=" << gap
               << " object_longitudinal_speed=" << objectLongitudinalSpeed
               << " closing_speed=" << closingSpeed
               << " ttc=" << ttc;
        // 정지 객체는 현재 15 m 안에 있더라도 Ego가 그 객체를 지난 뒤
        // 빈 차선으로 들어갈 수 있다. 고정 간격으로 미리 탈락시키지 않고
        // 아래에서 생성한 전체 경로와 실제 OBB 충돌로 판정한다.
        if (gap >= 0.0 && !objectIsStatic &&
            gap < config.minimumTargetAheadGap) {
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
        if (gap < 0.0 && !objectIsStatic &&
            -gap < config.minimumTargetBehindGap) {
            detail << " result=reject reason=behind_gap_below_"
                   << config.minimumTargetBehindGap;
            traceDecision(trace, "target_lane", "gap_ttc_checked", detail.str());
            return false;
        }
        if (!objectIsStatic && ttc < config.minimumTargetTTC) {
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
    const Road& road, const LaneSection& section, int sourceLaneId,
    double roadS, double drivingYaw, bool targetOnLeft) {
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
            (candidateId > 0) != (sourceLaneId > 0)) continue;
        const auto candidate = section.lanes.find(candidateId);
        if (candidate == section.lanes.end() ||
            candidate->second.type != "driving") continue;
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
            (!targetOnLeft && leftDistance < -0.1)) return candidateId;
    }
    return std::nullopt;
}

std::optional<int> drivingLaneOnSide(
    const Road& road, const LaneSection& section, int sourceLaneId,
    double roadS, double drivingYaw, bool targetOnLeft, int laneHops) {
    if (laneHops < 1) return std::nullopt;
    int laneId = sourceLaneId;
    for (int hop = 0; hop < laneHops; ++hop) {
        const auto next = adjacentDrivingLaneOnSide(
            road, section, laneId, roadS, drivingYaw, targetOnLeft);
        if (!next) return std::nullopt;
        laneId = *next;
    }
    return laneId;
}

bool crossedLaneBoundariesAreAllowed(const LaneSection& section,
                                     int fromLaneId, int toLaneId,
                                     double roadS, double previousProgress,
                                     double progress) {
    const int laneHops = std::abs(toLaneId - fromLaneId);
    if (laneHops < 1) return false;
    const int step = toLaneId > fromLaneId ? 1 : -1;
    for (int boundary = 0; boundary < laneHops; ++boundary) {
        const double crossingProgress =
            (static_cast<double>(boundary) + 0.5) / laneHops;
        if (previousProgress < crossingProgress &&
            progress >= crossingProgress &&
            !laneChangeAllowed(section, fromLaneId + boundary * step,
                               fromLaneId + (boundary + 1) * step, roadS))
            return false;
    }
    return true;
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
// 한 번의 회피 경로가 짧은 connecting road 여러 개를 통과할 수 있으므로
// start/goal 이외의 중간 road도 정상적인 종방향 연결로 허용한다.
//
// 먼저 다음 전역경로 도로 id에 종방향 edge만으로 도달 가능한지 확인한다.
// 호출부는 도착 lane도 확인한다. 다른 lane으로 도착한 뒤 전역경로에 곧바로
// 보간하면 차선을 가로지르면서 도로표시 검사를 우회할 수 있기 때문이다.
bool findForwardLaneRoute(const PathGeneratorData& map,
                          const LaneKey& start,
                          int goalRoadId,
                          std::vector<LaneKey>& route,
                          LaneKey& reachedLane) {
    std::queue<LaneKey> pending;
    std::map<LaneKey, bool> visited;
    std::map<LaneKey, LaneKey> previous;
    pending.push(start);
    visited[start] = true;
    bool found = start.roadId == goalRoadId;
    LaneKey goal = start;
    while (!found && !pending.empty()) {
        const LaneKey current = pending.front();
        pending.pop();
        const auto neighbors = map.laneGraph.find(current);
        if (neighbors == map.laneGraph.end()) continue;
        for (const LaneKey& next : neighbors->second) {
            const bool lateralLaneChange =
                current.roadId == next.roadId &&
                current.sectionIndex == next.sectionIndex &&
                current.laneId != next.laneId;
            if (lateralLaneChange || visited.count(next)) {
                continue;
            }
            visited[next] = true;
            previous[next] = current;
            if (next.roadId == goalRoadId) {
                found = true;
                goal = next;
                break;
            }
            pending.push(next);
        }
    }
    if (!found) return false;
    reachedLane = goal;
    route.clear();
    for (LaneKey key = goal;; key = previous.at(key)) {
        route.push_back(key);
        if (key == start) break;
    }
    std::reverse(route.begin(), route.end());
    return true;
}

bool hasForwardLaneConnection(const PathGeneratorData& map,
                              const LaneKey& start,
                              const LaneKey& goal) {
    std::queue<LaneKey> pending;
    std::map<LaneKey, bool> visited;
    pending.push(start);
    visited[start] = true;
    while (!pending.empty()) {
        const LaneKey current = pending.front();
        pending.pop();
        if (current == goal) return true;
        const auto neighbors = map.laneGraph.find(current);
        if (neighbors == map.laneGraph.end()) continue;
        for (const LaneKey& next : neighbors->second) {
            const bool lateralLaneChange =
                current.roadId == next.roadId &&
                current.sectionIndex == next.sectionIndex &&
                current.laneId != next.laneId;
            if (lateralLaneChange || visited.count(next)) continue;
            visited[next] = true;
            pending.push(next);
        }
    }
    return false;
}

const WorldObjectState* nearestStaticObjectOnForwardLane(
    const WorldState& world,
    const PathGeneratorData& map,
    const LaneKey& startLane,
    double afterPathS,
    int ignoredObjectId = -1) {
    const WorldObjectState* nearest = nullptr;
    double nearestPathS = std::numeric_limits<double>::infinity();
    for (const WorldObjectState& object : world.objects) {
        if (static_cast<int>(object.raw.id) == ignoredObjectId ||
            std::abs(object.raw.speed) >= 0.3 ||
            !object.laneValid || !object.pathValid ||
            object.pathS <= afterPathS + 1e-6) {
            continue;
        }
        const LaneKey objectLane{
            object.roadId, object.sectionIndex, object.laneId};
        if (!hasForwardLaneConnection(map, startLane, objectLane)) continue;
        if (object.pathS < nearestPathS) {
            nearest = &object;
            nearestPathS = object.pathS;
        }
    }
    return nearest;
}

double forwardStaticClearance(const WorldState& world,
                              const PathGeneratorData& map,
                              const LaneKey& startLane,
                              double afterPathS,
                              int ignoredObjectId = -1) {
    const WorldObjectState* obstacle = nearestStaticObjectOnForwardLane(
        world, map, startLane, afterPathS, ignoredObjectId);
    return obstacle == nullptr
        ? std::numeric_limits<double>::infinity()
        : obstacle->pathS - afterPathS;
}

LaneKey laneKeyAtPathS(const std::vector<Waypoint>& path, double routeS) {
    if (path.empty()) return {};
    const auto nearest = std::min_element(
        path.begin(), path.end(),
        [&](const Waypoint& lhs, const Waypoint& rhs) {
            return std::abs(lhs.s - routeS) < std::abs(rhs.s - routeS);
        });
    return {nearest->roadId, nearest->sectionIndex, nearest->laneId};
}

bool findForwardLaneRouteToLane(const PathGeneratorData& map,
                                const LaneKey& start,
                                const LaneKey& goal,
                                std::vector<LaneKey>& route) {
    std::queue<LaneKey> pending;
    std::map<LaneKey, bool> visited;
    std::map<LaneKey, LaneKey> previous;
    pending.push(start);
    visited[start] = true;
    while (!pending.empty()) {
        const LaneKey current = pending.front();
        pending.pop();
        if (current == goal) {
            route.clear();
            for (LaneKey key = goal;; key = previous.at(key)) {
                route.push_back(key);
                if (key == start) break;
            }
            std::reverse(route.begin(), route.end());
            return true;
        }
        const auto neighbors = map.laneGraph.find(current);
        if (neighbors == map.laneGraph.end()) continue;
        for (const LaneKey& next : neighbors->second) {
            const bool lateralLaneChange =
                current.roadId == next.roadId &&
                current.sectionIndex == next.sectionIndex &&
                current.laneId != next.laneId;
            if (lateralLaneChange || visited.count(next)) continue;
            visited[next] = true;
            previous[next] = current;
            pending.push(next);
        }
    }
    return false;
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

// Directly appending a global-route continuation to a lane centerline can
// create a lateral kink when their boundary samples are slightly offset.
// Replace both sides of that join with a quintic Hermite curve whose position,
// tangent, and acceleration are continuous at the endpoints.
bool appendQuinticReconnect(std::vector<Waypoint>& output,
                            const std::vector<Waypoint>& continuation,
                            double transitionLength,
                            double sampleInterval,
                            DecisionTraceData* trace) {
    if (output.size() < 3 || continuation.size() < 3) return false;

    const auto headingBetween = [](const Waypoint& from,
                                   const Waypoint& to) {
        return std::atan2(to.y - from.y, to.x - from.x);
    };
    const double outgoingYaw = headingBetween(
        output[output.size() - 2], output.back());

    std::size_t continuationBegin = 0;
    while (continuationBegin + 1 < continuation.size() &&
           std::hypot(continuation[continuationBegin].x - output.back().x,
                      continuation[continuationBegin].y - output.back().y) <
               0.05) {
        ++continuationBegin;
    }
    if (continuationBegin + 1 >= continuation.size()) return false;

    const Waypoint& firstContinuation = continuation[continuationBegin];
    const double joinGap = std::hypot(firstContinuation.x - output.back().x,
                                      firstContinuation.y - output.back().y);
    const double joinYaw = headingBetween(output.back(), firstContinuation);
    const double incomingYaw = headingBetween(
        firstContinuation, continuation[continuationBegin + 1]);
    constexpr double kDirectJoinHeadingTolerance =
        10.0 * 3.14159265358979323846 / 180.0;
    const bool smoothJoinNeeded =
        joinGap > std::max(0.75, 1.5 * sampleInterval) ||
        std::abs(normalizeAngle(joinYaw - outgoingYaw)) >
            kDirectJoinHeadingTolerance ||
        std::abs(normalizeAngle(incomingYaw - joinYaw)) >
            kDirectJoinHeadingTolerance;
    if (!smoothJoinNeeded) {
        for (std::size_t index = continuationBegin;
             index < continuation.size(); ++index) {
            appendUnique(output, continuation[index]);
        }
        return true;
    }

    const double halfTransition = 0.5 * transitionLength;
    std::size_t startIndex = output.size() - 1;
    double startDistance = 0.0;
    while (startIndex > 1 && startDistance < halfTransition) {
        startDistance += std::hypot(
            output[startIndex].x - output[startIndex - 1].x,
            output[startIndex].y - output[startIndex - 1].y);
        --startIndex;
    }

    std::size_t endIndex = continuationBegin;
    double endDistance = 0.0;
    while (endIndex + 1 < continuation.size() &&
           endDistance < halfTransition) {
        endDistance += std::hypot(
            continuation[endIndex + 1].x - continuation[endIndex].x,
            continuation[endIndex + 1].y - continuation[endIndex].y);
        ++endIndex;
    }
    if (startDistance + 1e-6 < halfTransition ||
        endDistance + 1e-6 < halfTransition ||
        endIndex + 1 >= continuation.size()) {
        traceDecision(trace, "target_lane", "smooth_reconnect_rejected",
                      "reason=insufficient_path_length_for_quintic_transition");
        return false;
    }

    const Waypoint start = output[startIndex];
    const Waypoint end = continuation[endIndex];
    const double chordX = end.x - start.x;
    const double chordY = end.y - start.y;
    const double chordLength = std::hypot(chordX, chordY);
    if (!std::isfinite(chordLength) || chordLength < 0.5 ||
        chordLength > 2.0 * transitionLength) {
        traceDecision(trace, "target_lane", "smooth_reconnect_rejected",
                      "reason=invalid_quintic_endpoint_distance");
        return false;
    }

    const double startYaw = headingBetween(output[startIndex - 1], start);
    const double endYaw = headingBetween(
        continuation[endIndex], continuation[endIndex + 1]);
    const double chordYaw = std::atan2(chordY, chordX);
    constexpr double kMaximumEndpointHeadingError =
        75.0 * 3.14159265358979323846 / 180.0;
    if (std::abs(normalizeAngle(startYaw - chordYaw)) >
            kMaximumEndpointHeadingError ||
        std::abs(normalizeAngle(endYaw - chordYaw)) >
            kMaximumEndpointHeadingError) {
        traceDecision(trace, "target_lane", "smooth_reconnect_rejected",
                      "reason=quintic_endpoints_face_away_from_join");
        return false;
    }

    // t ∈ [0, 1]의 5차 Hermite. 양 끝의 2차 미분을 0으로 두어
    // 직선/완만한 차선 중심선에서 조향이 튀지 않게 한다.
    const double v0x = chordLength * std::cos(startYaw);
    const double v0y = chordLength * std::sin(startYaw);
    const double v1x = chordLength * std::cos(endYaw);
    const double v1y = chordLength * std::sin(endYaw);
    const auto interpolate = [](double p0, double p1,
                                double v0, double v1, double t) {
        const double delta = p1 - p0;
        const double c3 = 10.0 * delta - 6.0 * v0 - 4.0 * v1;
        const double c4 = -15.0 * delta + 8.0 * v0 + 7.0 * v1;
        const double c5 = 6.0 * delta - 3.0 * v0 - 3.0 * v1;
        return p0 + v0 * t + c3 * t * t * t +
               c4 * t * t * t * t + c5 * t * t * t * t * t;
    };

    output.resize(startIndex + 1);
    const double estimatedLength = startDistance + joinGap + endDistance;
    const int sampleCount = std::max(
        3, static_cast<int>(std::ceil(
               estimatedLength / std::max(0.2, sampleInterval))));
    for (int index = 1; index <= sampleCount; ++index) {
        const double t = static_cast<double>(index) / sampleCount;
        Waypoint point = t < 0.5 ? start : end;
        point.x = interpolate(start.x, end.x, v0x, v1x, t);
        point.y = interpolate(start.y, end.y, v0y, v1y, t);
        point.s = start.s + t * (end.s - start.s);
        if (start.roadId == end.roadId) {
            point.roadS = start.roadS + t * (end.roadS - start.roadS);
        }
        output.push_back(point);
    }
    for (std::size_t index = endIndex + 1;
         index < continuation.size(); ++index) {
        appendUnique(output, continuation[index]);
    }

    std::ostringstream detail;
    detail << "join_gap=" << joinGap
           << " replaced_length=" << estimatedLength
           << " transition_points=" << sampleCount;
    traceDecision(trace, "target_lane", "smooth_reconnect_generated",
                  detail.str());
    return true;
}

bool buildKeepTargetPath(std::vector<Waypoint>& output,
                         LaneKey& reconnectLane,
                         std::size_t& reconnectGlobalIndex,
                         bool& reconnectLaneValid,
                         bool& awaitExactReconnect,
                         double& shiftCompleteRouteS,
                         double transitionLength,
                         double desiredCompletionDistance,
                         bool targetOnLeft,
                         int laneHops,
                         const LaneKey* requiredForwardLane,
                         const LaneChangeConfig& config,
                         const WorldState& world,
                         const PathGeneratorData& map,
                         const PathPlannerData& globalPath,
                         std::size_t nearestGlobalIndex,
                         DecisionTraceData* trace) {
    const auto roadFound = map.roads.find(world.egoRoadId);
    if (roadFound == map.roads.end()) return false;
    const Road& road = roadFound->second;
    // 두 차선 이상 회피에서 from/to만 한 번에 비교하면 중간
    // 경계의 roadMark를 잘못 읽는다. 아래에서 blend가 각 경계를
    // 실제로 통과하는 road_s마다 모든 경계를 개별 검사한다.
    // 현재 차선의 종방향 successor를 따라 다음 corridor road까지 펼친다.
    // 기존에는 현재 road 끝에서 잘라 버려, 도로 경계를 걸쳐 완성할 수 있는
    // 지그재그 회피도 남은 길이 부족으로 폐기됐다. 횡방향 경계 통과가 점선
    // 구간에서 시작됐는지는 아래 웨이포인트별 검사로 계속 보장한다.
    const LaneKey sourceStart{world.egoRoadId, world.egoSectionIndex,
                              world.egoLaneId};
    std::vector<LaneKey> sourceRoute{sourceStart};
    LaneKey requestedNextRoadLane;
    std::size_t requestedNextRoadIndex{};
    LaneKey reachedRoadLane;
    std::vector<LaneKey> routeToNextRoad;
    if (findNextGlobalRoadLane(requestedNextRoadLane, requestedNextRoadIndex,
                               globalPath, map, nearestGlobalIndex,
                               world.egoRoadId) &&
        findForwardLaneRoute(map, sourceStart, requestedNextRoadLane.roadId,
                             routeToNextRoad, reachedRoadLane)) {
        sourceRoute = std::move(routeToNextRoad);
    }
    for (std::size_t guard = 0; guard < road.sections.size() + 2; ++guard) {
        const LaneKey current = sourceRoute.back();
        const auto edges = map.laneGraph.find(current);
        if (edges == map.laneGraph.end()) break;
        const auto next = std::find_if(
            edges->second.begin(), edges->second.end(),
            [&](const LaneKey& key) {
                return !(key.roadId == current.roadId &&
                         key.sectionIndex == current.sectionIndex &&
                         key.laneId != current.laneId) &&
                       std::find(sourceRoute.begin(), sourceRoute.end(), key) ==
                           sourceRoute.end();
            });
        if (next == edges->second.end()) break;
        sourceRoute.push_back(*next);
    }

    std::vector<Waypoint> sourcePoints;
    for (std::size_t routeIndex = 0; routeIndex < sourceRoute.size(); ++routeIndex) {
        const auto segment = map.laneSegments.find(sourceRoute[routeIndex]);
        if (segment == map.laneSegments.end()) return false;
        std::size_t begin = 0;
        if (routeIndex == 0) {
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t index = 0;
                 index < segment->second.points.size(); ++index) {
                const double distance = std::hypot(
                    segment->second.points[index].x - world.ego.x,
                    segment->second.points[index].y - world.ego.y);
                if (distance < nearest) {
                    nearest = distance;
                    begin = index;
                }
            }
        }
        for (std::size_t index = begin;
             index < segment->second.points.size(); ++index) {
            appendUnique(sourcePoints, segment->second.points[index]);
        }
    }
    if (sourcePoints.size() < 3) return false;

    std::vector<double> sourceProgress(sourcePoints.size(), 0.0);
    for (std::size_t index = 1; index < sourcePoints.size(); ++index) {
        sourceProgress[index] = sourceProgress[index - 1] + std::hypot(
            sourcePoints[index].x - sourcePoints[index - 1].x,
            sourcePoints[index].y - sourcePoints[index - 1].y);
    }
    std::size_t shiftBegin = sourcePoints.size();
    std::size_t shiftEnd = sourcePoints.size();
    double bestCompletionScore = std::numeric_limits<double>::infinity();
    // Prepare 동안에는 방향지시등만 켜고 현재 차선으로 계속 전진한다.
    // 후보를 현재 위치부터 횡이동하도록 만들면 Execute 전환 시 이미 옆으로
    // 이동했어야 할 경로 중간을 쫓으며 조향이 튄다. 준비시간 동안 예상되는
    // 전진거리 뒤에서부터 횡이동을 시작하도록 경로 자체에 직선 prefix를 둔다.
    const double minimumShiftBeginProgress =
        std::max(0.0, std::abs(world.egoSpeed) * config.prepareTime);
    for (std::size_t candidateBegin = 0;
         candidateBegin + 1 < sourcePoints.size(); ++candidateBegin) {
        if (sourceProgress[candidateBegin] + 1e-6 <
            minimumShiftBeginProgress) {
            continue;
        }
        double distance = 0.0;
        double previousProgress = 0.0;
        bool valid = true;
        for (std::size_t index = candidateBegin;
             index < sourcePoints.size(); ++index) {
            if (index > candidateBegin) {
                distance += std::hypot(
                    sourcePoints[index].x - sourcePoints[index - 1].x,
                    sourcePoints[index].y - sourcePoints[index - 1].y);
            }
            const Waypoint& point = sourcePoints[index];
            const auto localRoadFound = map.roads.find(point.roadId);
            if (localRoadFound == map.roads.end()) {
                valid = false;
                break;
            }
            const Road& localRoad = localRoadFound->second;
            const int sectionIndex = sectionIndexAt(localRoad, point.roadS);
            const LaneSection& localSection = localRoad.sections[sectionIndex];
            const auto localTarget = drivingLaneOnSide(
                localRoad, localSection, point.laneId, point.roadS, point.yaw,
                targetOnLeft, laneHops);
            const double progress = smoothStep(
                distance / transitionLength);
            if (!localTarget || !crossedLaneBoundariesAreAllowed(
                    localSection, point.laneId, *localTarget, point.roadS,
                    previousProgress, progress)) {
                valid = false;
                break;
            }
            previousProgress = progress;
            if (distance >= transitionLength) {
                const double score = desiredCompletionDistance < 0.0
                    ? sourceProgress[index]
                    : std::abs(sourceProgress[index] -
                               desiredCompletionDistance);
                if (score < bestCompletionScore) {
                    bestCompletionScore = score;
                    shiftBegin = candidateBegin;
                    shiftEnd = index;
                }
                break;
            }
        }
        if (!valid) continue;
        if (desiredCompletionDistance < 0.0 &&
            shiftEnd < sourcePoints.size()) {
            break;
        }
    }
    if (shiftEnd >= sourcePoints.size()) {
        traceDecision(trace, "target_lane", "keep_route_rejected",
                      "reason=no_forward_dashed_window_for_lane_shift");
        return false;
    }

    output.clear();
    double shiftDistance = 0.0;
    LaneKey completedTarget;
    for (std::size_t index = 0; index <= shiftEnd; ++index) {
        const Waypoint& source = sourcePoints[index];
        if (index > shiftBegin) {
            shiftDistance += std::hypot(
                sourcePoints[index].x - sourcePoints[index - 1].x,
                sourcePoints[index].y - sourcePoints[index - 1].y);
        }
        Waypoint point = source;
        if (index >= shiftBegin) {
            const auto localRoadFound = map.roads.find(source.roadId);
            if (localRoadFound == map.roads.end()) return false;
            const Road& localRoad = localRoadFound->second;
            const int sectionIndex = sectionIndexAt(localRoad, source.roadS);
            const LaneSection& localSection = localRoad.sections[sectionIndex];
            const auto localTarget = drivingLaneOnSide(
                localRoad, localSection, source.laneId, source.roadS, source.yaw,
                targetOnLeft, laneHops);
            if (!localTarget) return false;
            const RoadPose reference = roadToWorld(localRoad, source.roadS);
            const double offset = laneOffset(
                localRoad, localSection, *localTarget, source.roadS);
            const double targetX = reference.x - offset * std::sin(reference.yaw);
            const double targetY = reference.y + offset * std::cos(reference.yaw);
            const double blend = smoothStep(
                shiftDistance / transitionLength);
            point.x += blend * (targetX - source.x);
            point.y += blend * (targetY - source.y);
            if (blend >= 0.5) point.laneId = *localTarget;
            completedTarget = {source.roadId, sectionIndex, *localTarget};
        }
        appendUnique(output, point);
    }
    const std::size_t shiftCompletionIndex = output.size() - 1;

    // 현재 section이 1차로여도 앞 section에서 회피 차선이 새로 생길 수
    // 있다. 이 경우 현재 위치의 존재하지 않는 target lane을 검사하지 말고,
    // 실제 횡이동이 끝나는 road/section/lane의 간격을 검사한다.
    if (!targetLaneHasSafeGap(world, completedTarget.roadId,
                              completedTarget.sectionIndex,
                              completedTarget.laneId,
                              config, trace)) {
        return false;
    }

    const auto targetSegment = map.laneSegments.find(completedTarget);
    if (targetSegment == map.laneSegments.end()) return false;
    std::size_t targetBegin = 0;
    double nearestTarget = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0;
         index < targetSegment->second.points.size(); ++index) {
        const double distance = std::hypot(
            targetSegment->second.points[index].x - output.back().x,
            targetSegment->second.points[index].y - output.back().y);
        if (distance < nearestTarget) {
            nearestTarget = distance;
            targetBegin = index;
        }
    }
    for (std::size_t index = targetBegin + 1;
         index < targetSegment->second.points.size(); ++index) {
        appendUnique(output, targetSegment->second.points[index]);
    }

    awaitExactReconnect = false;
    std::vector<LaneKey> forwardRoute;
    LaneKey reachedLane;
    if (requiredForwardLane != nullptr) {
        // 먼 미래 road의 장애물을 보고 현재 road에서 미리 횟이동할
        // 때는 '같은 방향의 옆 차선'만으로는 부족하다. 중간
        // connector의 lane 번호가 바뀌기 때문에, 완료 차선이 장애물
        // road의 정확한 회피 차선으로 종방향 연결되는지 검증한다.
        if (!findForwardLaneRouteToLane(
                map, completedTarget, *requiredForwardLane, forwardRoute)) {
            traceDecision(trace, "target_lane", "keep_route_rejected",
                          "reason=shift_lane_does_not_reach_exact_obstacle_avoidance_lane");
            return false;
        }
        reconnectLane = *requiredForwardLane;
        reconnectLaneValid = true;
        awaitExactReconnect = true;
        for (std::size_t routeIndex = 1;
             routeIndex < forwardRoute.size(); ++routeIndex) {
            const auto segment = map.laneSegments.find(forwardRoute[routeIndex]);
            if (segment == map.laneSegments.end()) return false;
            for (const Waypoint& point : segment->second.points) {
                appendUnique(output, point);
            }
        }
    } else {
        LaneKey requestedRoadLane;
        reconnectLaneValid = findNextGlobalRoadLane(
            requestedRoadLane, reconnectGlobalIndex, globalPath, map,
            nearestGlobalIndex, world.egoRoadId);
        if (reconnectLaneValid && findForwardLaneRoute(
                map, completedTarget, requestedRoadLane.roadId,
                forwardRoute, reachedLane)) {
            reconnectLane = reachedLane;
            awaitExactReconnect = true;
            for (std::size_t routeIndex = 1;
                 routeIndex < forwardRoute.size(); ++routeIndex) {
                const auto segment =
                    map.laneSegments.find(forwardRoute[routeIndex]);
                if (segment == map.laneSegments.end()) return false;
                for (const Waypoint& point : segment->second.points) {
                    appendUnique(output, point);
                }
            }
        } else {
            reconnectLaneValid = false;
            traceDecision(trace, "target_lane", "keep_current_road_only",
                          "reason=obstacle_avoidance_precedes_next_road_alignment");
        }
    }
    updateCandidateGeometry(output, world.egoPathS);
    shiftCompleteRouteS = output[shiftCompletionIndex].s;
    return output.size() >= 3;
}


// 회피 기동은 "Ego가 지금 있는 도로"가 아니라 "장애물이 있는 도로/차선"을
// 기준으로 세운다. 장애물 앞 도로가 교차로 연결로처럼 편도 1차선이어도,
// 장애물이 놓인 도로에 옆 차선이 있으면 미리 계획해 두었다가 그 도로에
// 들어서는 순간 횡이동을 시작할 수 있다.
bool buildReturnCandidatePath(std::vector<Waypoint>& output,
                        std::uint8_t& turnSignal,
                        int laneHops,
                        double& returnStartRouteS,
                        double& rejoinRouteS,
                        int targetObstacleLaneId,
                        bool targetOnLeft,
                        double lateralOffset,
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
    if (!targetLaneHasSafeGap(world, obstacle.roadId, obstacle.sectionIndex,
                              targetObstacleLaneId, config, trace)) {
        return false;
    }
    const LaneKey obstacleTargetKey{obstacle.roadId, obstacle.sectionIndex,
                                    targetObstacleLaneId};

    // Ego가 현재 편도 1차선 연결로에 있을 수 있다. 이때 지금 위치에서 인접
    // 차선을 요구하면, 바로 앞 다차선 도로에 있는 장애물을 영원히 회피하지
    // 못한다. 참조경로를 따라 실제 인접 차선이 처음 생기며 그 차선이 장애물
    // 옆 목표 차선까지 연결되는 지점을 찾는다.
    double targetLaneStartS = std::numeric_limits<double>::infinity();
    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& point = globalPath.waypoints[index];
        if (point.s > obstacle.pathS) break;
        const auto roadFound = map.roads.find(point.roadId);
        if (roadFound == map.roads.end() || roadFound->second.sections.empty()) {
            continue;
        }
        const int pointSectionIndex =
            sectionIndexAt(roadFound->second, point.roadS);
        const LaneSection& pointSection =
            roadFound->second.sections[pointSectionIndex];
        std::optional<int> avoidanceLane;
        if (point.roadId == obstacle.roadId &&
            pointSectionIndex == obstacle.sectionIndex) {
            const auto fixed = pointSection.lanes.find(targetObstacleLaneId);
            if (fixed != pointSection.lanes.end() &&
                fixed->second.type == "driving") {
                avoidanceLane = targetObstacleLaneId;
            }
        } else {
            avoidanceLane = drivingLaneOnSide(
                roadFound->second, pointSection, point.laneId, point.roadS,
                point.yaw, targetOnLeft, laneHops);
        }
        if (!avoidanceLane) continue;
        targetLaneStartS = point.s;
        break;
    }
    if (!std::isfinite(targetLaneStartS)) {
        traceDecision(trace, "target_lane", "topology_route_rejected",
                      "reason=no_connected_adjacent_lane_before_obstacle");
        return false;
    }

    // 횡이동은 목표 차선이 생기는 지점부터 시작하고, 장애물에 닿기 전에 끝낸다.
    // 횡이동은 장애물 직전에서 시작한다. Ego 위치에서 곧바로 시작하면
    // 장애물이 45 m 앞일 때 45 m 내내 옆 차선에 걸친 경로가 되고, 그 긴
    // 구간에서 lane section 경계나 경로 자체의 차선 전환을 만나 후보 경로가
    // 끊긴다. 실제 주행 관점에서도 미리 45 m 전부터 나갈 이유가 없다.
    const double transitionDistance =
        config.transitionLength * std::sqrt(static_cast<double>(laneHops));
    const double earliestShiftStartS =
        world.egoPathS + std::max(0.0, std::abs(world.egoSpeed)) *
                                 config.prepareTime;
    const double shiftStartS = std::max(
        {earliestShiftStartS, targetLaneStartS,
         obstacle.pathS - transitionDistance});
    const double availableShift = obstacle.pathS - shiftStartS;
    // 끼어들어 멈춘 차량 뒤에서 이미 저속/정지한 경우에도 고정 18 m를
    // 요구하면 회피 후보가 영구히 생기지 않는다. 최소 형상 길이는 지키되
    // 실제 남은 공간에 맞춰 outward 전환만 줄이고, 생성 후 곡률/OBB 검사를
    // 통과한 경로만 선택한다. 장애물 뒤 복귀는 원래 전환 길이를 유지한다.
    constexpr double kMinimumShortShiftLength = 6.0;
    if (availableShift + 0.5 < kMinimumShortShiftLength) {
        std::ostringstream detail;
        detail << "available=" << availableShift
               << " minimum=" << kMinimumShortShiftLength
               << " target_lane_start_s=" << targetLaneStartS
               << " obstacle_path_s=" << obstacle.pathS;
        traceDecision(trace, "target_lane", "short_shift_rejected",
                      detail.str());
        return false;
    }
    const double shiftLength = std::min(transitionDistance, availableShift);

    double blockedUntilS = obstacle.pathS;
    const double corridorHalfWidth = 4.5 * (laneHops + 0.5);
    constexpr double kObstacleClusterLookahead = 80.0;
    for (const WorldObjectState& object : world.objects) {
        if (object.laneValid && object.roadId == obstacle.roadId &&
            object.sectionIndex == obstacle.sectionIndex &&
            object.laneId == targetObstacleLaneId)
            continue;
        if (!object.pathValid || !object.isAhead ||
            std::abs(object.raw.speed) >= 0.3 ||
            object.pathS < obstacle.pathS - 2.0 ||
            object.pathS > obstacle.pathS + kObstacleClusterLookahead ||
            std::abs(object.pathD) > corridorHalfWidth) continue;
        blockedUntilS = std::max(blockedUntilS, object.pathS);
    }
    const double leaveTargetS = blockedUntilS + config.obstacleClearance;
    returnStartRouteS = leaveTargetS;
    rejoinRouteS = leaveTargetS + transitionDistance;
    // Keep enough global-route tail after the return curve. Otherwise a
    // laterally lagging vehicle chases the final local point from the side and
    // commands maximum steering instead of looking farther down the road.
    constexpr double kPostRejoinTrackingLength = 30.0;
    const double pathEndS = rejoinRouteS + kPostRejoinTrackingLength;
    output.clear();

    const Waypoint* rejoinPoint = nullptr;
    double rejoinBestDelta = std::numeric_limits<double>::infinity();
    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const Waypoint& point = globalPath.waypoints[index];
        const double delta = std::abs(point.s - rejoinRouteS);
        if (delta < rejoinBestDelta) {
            rejoinBestDelta = delta;
            rejoinPoint = &point;
        }
        if (point.s > rejoinRouteS && delta > rejoinBestDelta) break;
    }
    if (rejoinPoint == nullptr) return false;
    const auto rejoinRoadFound = map.roads.find(rejoinPoint->roadId);
    if (rejoinRoadFound == map.roads.end()) return false;
    const int rejoinSectionIndex =
        sectionIndexAt(rejoinRoadFound->second, rejoinPoint->roadS);
    const LaneSection& rejoinSection =
        rejoinRoadFound->second.sections[rejoinSectionIndex];
    std::optional<int> rejoinTargetLane;
    if (rejoinPoint->roadId == obstacle.roadId &&
        (rejoinSectionIndex == obstacle.sectionIndex ||
         rejoinPoint->laneId == targetObstacleLaneId)) {
        const auto fixed = rejoinSection.lanes.find(targetObstacleLaneId);
        if (fixed != rejoinSection.lanes.end() &&
            fixed->second.type == "driving") {
            rejoinTargetLane = targetObstacleLaneId;
        }
    } else {
        rejoinTargetLane = drivingLaneOnSide(
            rejoinRoadFound->second, rejoinSection, rejoinPoint->laneId,
            rejoinPoint->roadS, rejoinPoint->yaw, targetOnLeft, laneHops);
    }
    const LaneKey rejoinKey{rejoinPoint->roadId, rejoinSectionIndex,
                            rejoinPoint->laneId};
    // 회피 차선이 합류하면서 전역경로 차선 자체가 되면, 재합류 지점에는 더
    // 이상 '옆 차선'이 존재하지 않는다. 횡방향 edge 없이 실제 successor로
    // 이어진 경우에만 같은 차선으로 간주한다.
    if (!rejoinTargetLane &&
        hasForwardLaneConnection(map, obstacleTargetKey, rejoinKey)) {
        rejoinTargetLane = rejoinPoint->laneId;
    }
    if (!rejoinTargetLane) {
        std::ostringstream detail;
        detail << "avoidance=" << obstacleTargetKey.roadId << '/'
               << obstacleTargetKey.sectionIndex << '/'
               << obstacleTargetKey.laneId << " rejoin="
               << rejoinKey.roadId << '/' << rejoinKey.sectionIndex << '/'
               << rejoinKey.laneId;
        traceDecision(trace, "target_lane", "rejoin_lane_rejected",
                      detail.str());
        return false;
    }

    double egoWidth = std::hypot(
        world.egoObb.corners[0].x - world.egoObb.corners[1].x,
        world.egoObb.corners[0].y - world.egoObb.corners[1].y);
    const double otherEgoDimension = std::hypot(
        world.egoObb.corners[1].x - world.egoObb.corners[2].x,
        world.egoObb.corners[1].y - world.egoObb.corners[2].y);
    egoWidth = std::min(egoWidth, otherEgoDimension);

    double previousOutwardProgress = 0.0;
    double previousReturnProgress = 0.0;
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
                                     transitionDistance);
        }

        // 아직 옆으로 나가지 않는 구간은 기준 경로를 그대로 쓴다. 이 구간이
        // 편도 1차선(연결로)이어도 문제되지 않는다.
        if (blend <= 1e-6) {
            output.push_back(original);
        } else {
            // 횡이동 구간은 웨이포인트마다 자기 road/section에서 목표 차선을
            // 찾는다. 여러 lane section 경계나
            // 경로 자체의 차선 전환을 만나도 기동이 끊기지 않아야 한다.
            if (pointRoadPtr == nullptr || pointSectionPtr == nullptr) break;
            const Road& pointRoad = *pointRoadPtr;
            const LaneSection& pointSection = *pointSectionPtr;

            // 장애물이 있는 동일 road/section에서는 목표 차선을 고정한다.
            // 장애물 뒤에서 전역경로가 그 목표 차선으로 합류해도 다시 한 칸
            // 바깥 차선을 선택하면 안 된다. 다른 section/road에서만 물리적으로
            // 같은 방향의 인접 차선을 다시 매핑한다.
            std::optional<int> pointTargetLane;
            const int pointSectionIndex =
                sectionIndexAt(pointRoad, original.roadS);
            if (original.roadId == obstacle.roadId &&
                (pointSectionIndex == obstacle.sectionIndex ||
                 original.laneId == targetObstacleLaneId)) {
                const auto fixed =
                    pointSection.lanes.find(targetObstacleLaneId);
                if (fixed != pointSection.lanes.end() &&
                    fixed->second.type == "driving") {
                    pointTargetLane = targetObstacleLaneId;
                }
            } else {
                pointTargetLane = drivingLaneOnSide(
                    pointRoad, pointSection, original.laneId, original.roadS,
                    original.yaw, targetOnLeft, laneHops);
                const LaneKey pointKey{original.roadId, pointSectionIndex,
                                       original.laneId};
                if (!pointTargetLane && hasForwardLaneConnection(
                        map, obstacleTargetKey, pointKey)) {
                    pointTargetLane = original.laneId;
                }
            }
            if (!pointTargetLane) break;
            const int pointTargetLaneId = *pointTargetLane;
            const auto pointTargetDefinition =
                pointSection.lanes.find(pointTargetLaneId);
            if (pointTargetDefinition == pointSection.lanes.end() ||
                pointTargetDefinition->second.type != "driving") break;
            const bool outwardTransition =
                original.s > shiftStartS &&
                original.s <= shiftStartS + shiftLength;
            const bool returnTransition =
                original.s >= leaveTargetS && original.s < rejoinRouteS;
            bool roadMarkAllowed = true;
            if (outwardTransition &&
                original.laneId != pointTargetLaneId) {
                roadMarkAllowed = crossedLaneBoundariesAreAllowed(
                    pointSection, original.laneId, pointTargetLaneId,
                    original.roadS, previousOutwardProgress, blend);
                previousOutwardProgress =
                    std::max(previousOutwardProgress, blend);
            }
            if (returnTransition &&
                pointTargetLaneId != original.laneId) {
                const double returnProgress = 1.0 - blend;
                roadMarkAllowed = roadMarkAllowed &&
                    crossedLaneBoundariesAreAllowed(
                        pointSection, pointTargetLaneId, original.laneId,
                        original.roadS, previousReturnProgress,
                        returnProgress);
                previousReturnProgress =
                    std::max(previousReturnProgress, returnProgress);
            }
            if (!roadMarkAllowed) break;

            const double targetLaneWidth =
                laneWidthAt(pointSection, pointTargetLaneId, original.roadS);
            const double maximumOffsetInsideLane =
                0.5 * targetLaneWidth - 0.5 * egoWidth -
                config.minimumLaneEdgeClearance;
            if (maximumOffsetInsideLane < -1e-6) break;
            // step 간격이 0.2 m여도 차선 끝선 안쪽의 실제 한계가
            // 0.61 m일 수 있다. 0.8 m 후보를 통째로 버리지 말고 한계에
            // clamp해 이 좁은 가능 구간도 검사한다.
            const double pointLateralOffset = std::clamp(
                lateralOffset, 0.0, std::max(0.0, maximumOffsetInsideLane));

            const RoadPose reference = roadToWorld(pointRoad, original.roadS);
            const double sourceOffset = laneOffset(
                pointRoad, pointSection, original.laneId, original.roadS);
            const double targetOffset = laneOffset(
                pointRoad, pointSection, pointTargetLaneId, original.roadS);
            const double relativeLaneOffset = targetOffset - sourceOffset;
            const double sideSign = targetOnLeft ? 1.0 : -1.0;
            Waypoint point = original;
            point.x = original.x + blend *
                (-relativeLaneOffset * std::sin(reference.yaw) -
                 sideSign * pointLateralOffset *
                 std::sin(original.yaw));
            point.y = original.y + blend *
                (relativeLaneOffset * std::cos(reference.yaw) +
                 sideSign * pointLateralOffset *
                 std::cos(original.yaw));
            if (blend > 0.5) point.laneId = pointTargetLaneId;
            output.push_back(point);
        }

    }
    if (output.size() < 3 || output.back().s < rejoinRouteS) {
        std::ostringstream detail;
        detail << "points=" << output.size()
               << " path_end_s="
               << (output.empty() ? -1.0 : output.back().s)
               << " required_rejoin_s=" << rejoinRouteS;
        traceDecision(trace, "target_lane", "return_path_rejected",
                      detail.str());
        return false;
    }

    // lane section 경계에서 lane id가 바뀔 때 두 centerline 표본이 몇 m
    // 어긋나는 맵이 있다. 종방향 successor로 확인된 동일 회피 차선의
    // 경계에서만 가장 큰 단절을 5차 곡선으로 연결한다.
    for (int repair = 0; repair < 6; ++repair) {
        std::size_t largestGapIndex = 0;
        double largestGap = 0.0;
        for (std::size_t index = 0; index + 1 < output.size(); ++index) {
            const double gap = std::hypot(
                output[index + 1].x - output[index].x,
                output[index + 1].y - output[index].y);
            if (gap > largestGap) {
                largestGap = gap;
                largestGapIndex = index;
            }
        }
        if (largestGap <= 2.0) break;
        std::vector<Waypoint> prefix(
            output.begin(), output.begin() + largestGapIndex + 1);
        const std::vector<Waypoint> continuation(
            output.begin() + largestGapIndex + 1, output.end());
        if (!appendQuinticReconnect(prefix, continuation,
                                    config.transitionLength,
                                    map.sampleInterval, trace)) {
            return false;
        }
        output = std::move(prefix);
    }

    for (std::size_t index = 0; index + 1 < output.size(); ++index) {
        output[index].yaw = std::atan2(output[index + 1].y - output[index].y,
                                      output[index + 1].x - output[index].x);
    }
    output.back().yaw = output[output.size() - 2].yaw;

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

    // 충돌이 없더라도 차량이 추종할 수 없는 직각/대간격 경로는
    // 선택하지 않는다. 이 검사가 실패하면 호출자가 다음 차선 후보나
    // 안전한 복귀 경로를 계속 탐색한다.
    std::string geometryReason;
    if (!validateGeneratedPath(points, &geometryReason)) {
        traceDecision(trace, "candidate_geometry", "candidate_rejected",
                      "reason=" + geometryReason);
        return false;
    }

    // 일반 주행의 3초 예측만으로는 긴 회피 경로의 복귀 구간을 놓칠 수 있다.
    // 차선변경 최대속도로 경로 끝까지 가는 시간을 계산해 전체 기동을 검사한다.
    WorldState predictionWorld = world;
    predictionWorld.egoSpeed = std::max(
        {1.0, std::abs(world.egoSpeed),
         laneChangeConfig.maximumLaneChangeSpeed});
    const double remainingLength =
        std::max(0.0, points.back().s - world.egoPathS);
    constexpr double kMaximumManeuverLength = 250.0;
    if (remainingLength > kMaximumManeuverLength) {
        std::ostringstream detail;
        detail << "remaining_length=" << remainingLength
               << " maximum=" << kMaximumManeuverLength
               << " result=reject reason=excessive_global_reconnect_detour";
        traceDecision(trace, "candidate_obb", "full_path_checked",
                      detail.str());
        return false;
    }
    const double fullManeuverHorizon =
        remainingLength / predictionWorld.egoSpeed +
        config.collisionPredictionStep;
    // 정적 장애물은 움직이지 않으므로 등속 예측 오차가 누적되지 않는다.
    // 여기까지 12초로 자르면 70 m 앞 장애물을 피하는 경로가 그보다 가까운
    // 옆 차선 장애물을 검사하기 전에 승인될 수 있다. 후보로 잘라 둔 기동
    // 구간 전체를 검사해, 장애물이 빽빽한 곳에서는 실제로 둘 사이를 통과하는
    // 종방향 샘플만 선택한다. 동적 객체 horizon은 아래에서 기존 3초를 유지한다.
    const double predictionHorizon =
        std::max(config.collisionPredictionHorizon, fullManeuverHorizon);

    PathSafetyResult safety;
    resetPathSafety(safety);
    checkCurrentCollisions(safety, world.egoObb, world.objects);
    if (!safety.collision) {
        // 정적 장애물은 기동 전체에서 배제해야 하지만, 동적 객체의 등속
        // 예측을 먼 미래까지 늘리면 실제로는 지나갈 차량 때문에 안전한
        // 후보를 버리게 된다. 동적 객체는 설정 horizon까지만 검사한다.
        WorldState dynamicWorld = predictionWorld;
        dynamicWorld.objects.erase(
            std::remove_if(dynamicWorld.objects.begin(),
                           dynamicWorld.objects.end(),
                           [](const WorldObjectState& object) {
                               return std::abs(object.raw.speed) < 0.3;
                           }),
            dynamicWorld.objects.end());
        checkPredictedCollisions(safety, dynamicWorld, candidate,
                                 config.collisionPredictionHorizon,
                                 config.collisionPredictionStep);
    }
    if (!safety.collision && !safety.predictedCollision) {
        WorldState staticWorld = predictionWorld;
        staticWorld.objects.erase(
            std::remove_if(staticWorld.objects.begin(),
                           staticWorld.objects.end(),
                           [](const WorldObjectState& object) {
                               return std::abs(object.raw.speed) >= 0.3;
                           }),
            staticWorld.objects.end());
        checkPredictedCollisions(safety, staticWorld, candidate,
                                 predictionHorizon,
                                 config.collisionPredictionStep, true);
    }
    {
        std::ostringstream detail;
        detail << "points=" << points.size()
               << " remaining_length=" << remainingLength
               << " assumed_ego_speed=" << predictionWorld.egoSpeed
               << " dynamic_horizon=" << config.collisionPredictionHorizon
               << " static_horizon=" << predictionHorizon
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

std::vector<Waypoint> maneuverPathPrefix(
    const std::vector<Waypoint>& points, double endRouteS) {
    if (points.size() < 3) return points;
    const auto end = std::upper_bound(
        points.begin(), points.end(), endRouteS,
        [](double value, const Waypoint& point) { return value < point.s; });
    if (end == points.end()) return points;
    const auto keepEnd = std::min(points.end(), end + 1);
    if (keepEnd < points.begin() + 3) return points;
    return {points.begin(), keepEnd};
}

double obstacleIntrusionIntoTargetLane(
    const WorldObjectState& obstacle, const Road& road,
    const LaneSection& section, int targetLaneId) {
    const RoadPose reference = roadToWorld(road, obstacle.roadS);
    const double sourceOffset =
        laneOffset(road, section, obstacle.laneId, obstacle.roadS);
    const double targetOffset =
        laneOffset(road, section, targetLaneId, obstacle.roadS);
    const double sourceX = reference.x - sourceOffset * std::sin(reference.yaw);
    const double sourceY = reference.y + sourceOffset * std::cos(reference.yaw);
    const double targetX = reference.x - targetOffset * std::sin(reference.yaw);
    const double targetY = reference.y + targetOffset * std::cos(reference.yaw);
    const double centerDistance = std::hypot(targetX - sourceX,
                                             targetY - sourceY);
    if (centerDistance < 1e-6) return std::numeric_limits<double>::infinity();
    const double directionX = (targetX - sourceX) / centerDistance;
    const double directionY = (targetY - sourceY) / centerDistance;
    double maximumProjection = -std::numeric_limits<double>::infinity();
    for (const Point2D& corner : obstacle.obb.corners) {
        maximumProjection = std::max(
            maximumProjection,
            (corner.x - sourceX) * directionX +
            (corner.y - sourceY) * directionY);
    }
    const double sourceHalfWidth =
        0.5 * laneWidthAt(section, obstacle.laneId, obstacle.roadS);
    return std::max(0.0, maximumProjection - sourceHalfWidth);
}

WorldObjectState matchBlockingObstacleToRoute(
    const WorldObjectState& detected,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    DecisionTraceData* trace) {
    WorldObjectState matched = detected;
    if (!detected.pathValid || !detected.isOnEgoPath ||
        globalPath.waypoints.empty()) {
        return matched;
    }

    const std::size_t beginIndex =
        std::min(nearestGlobalIndex, globalPath.waypoints.size() - 1);
    const auto begin = globalPath.waypoints.begin() + beginIndex;
    auto upper = std::lower_bound(
        begin, globalPath.waypoints.end(), detected.pathS,
        [](const Waypoint& point, double routeS) { return point.s < routeS; });
    auto best = upper;
    if (best == globalPath.waypoints.end()) {
        best = globalPath.waypoints.end() - 1;
    }
    if (best != begin) {
        const auto previous = best - 1;
        if (std::abs(previous->s - detected.pathS) <
            std::abs(best->s - detected.pathS)) {
            best = previous;
        }
    }

    // pathS/pathD는 이미 이 객체가 Ego 참조경로 위에 있음을 검증한 결과다.
    // 교차로나 중첩 도로에서는 독립적인 worldToLane 최근접 검색이 다른 도로를
    // 고를 수 있으므로, 회피 토폴로지는 참조경로 waypoint의 차선을 사용한다.
    constexpr double kMaximumRouteSampleDelta = 2.0;
    if (std::abs(best->s - detected.pathS) > kMaximumRouteSampleDelta) {
        return matched;
    }

    // Packaged route.csv files contain road_id/lane_id/road_s, but no lane
    // section column.  A freshly loaded Waypoint therefore has the C++ default
    // sectionIndex (0), which is not map data.  Using it here happened to work
    // on roads whose obstacle was in section 0, but remapped e.g. O04's real
    // road 2626/section 7 obstacle into section 0.  Always derive the section
    // from OpenDRIVE and road_s, and only trust the rematch when that driving
    // lane actually exists in the derived section.
    const auto routeRoad = map.roads.find(best->roadId);
    if (routeRoad == map.roads.end() || routeRoad->second.sections.empty() ||
        best->roadS < -1e-6 ||
        best->roadS > routeRoad->second.length + 1e-6) {
        traceDecision(trace, "obstacle_scan", "route_lane_rematch_rejected",
                      "reason=route_waypoint_has_no_valid_map_road");
        return matched;
    }
    const int routeSectionIndex =
        sectionIndexAt(routeRoad->second, best->roadS);
    if (routeSectionIndex < 0 ||
        routeSectionIndex >=
            static_cast<int>(routeRoad->second.sections.size())) {
        traceDecision(trace, "obstacle_scan", "route_lane_rematch_rejected",
                      "reason=route_waypoint_has_no_valid_map_section");
        return matched;
    }
    const auto routeLane =
        routeRoad->second.sections[routeSectionIndex].lanes.find(best->laneId);
    if (routeLane ==
            routeRoad->second.sections[routeSectionIndex].lanes.end() ||
        routeLane->second.type != "driving") {
        traceDecision(trace, "obstacle_scan", "route_lane_rematch_rejected",
                      "reason=route_waypoint_lane_is_not_driving");
        return matched;
    }

    if (!detected.laneValid || detected.roadId != best->roadId ||
        detected.sectionIndex != routeSectionIndex ||
        detected.laneId != best->laneId) {
        std::ostringstream detail;
        detail << "object=" << detected.raw.id
               << " raw=" << detected.roadId << '/'
               << detected.sectionIndex << '/' << detected.laneId
               << " route=" << best->roadId << '/'
               << routeSectionIndex << '/' << best->laneId
               << " path_s=" << detected.pathS
               << " path_d=" << detected.pathD;
        traceDecision(trace, "obstacle_scan", "route_lane_rematched",
                      detail.str());
    }
    matched.laneValid = true;
    matched.roadId = best->roadId;
    matched.sectionIndex = routeSectionIndex;
    matched.laneId = best->laneId;
    matched.roadS = best->roadS;
    return matched;
}

}  // namespace

bool buildRoadIdTrackingPath(
    PathPlannerData& output,
    const PathPlannerConfig& plannerConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    double horizon,
    DecisionTraceData* trace) {
    output = {};
    output.config = plannerConfig;
    if (!world.egoLaneValid || !world.egoPathValid) return false;

    // Synthetic checkpoint connectors deliberately use road/lane id 0 because
    // they bridge positions that the OpenDRIVE lane graph cannot connect.  A
    // road-center tracking path would otherwise ignore that connector and keep
    // following the current lane straight through the turn.  Select the CSV
    // global path while such a connector is inside the local planning horizon.
    if (!globalPath.waypoints.empty()) {
        const std::size_t begin = std::min(
            nearestGlobalIndex, globalPath.waypoints.size() - 1);
        const double endS = globalPath.waypoints[begin].s +
                            std::max(20.0, horizon);
        for (std::size_t index = begin;
             index < globalPath.waypoints.size() &&
             globalPath.waypoints[index].s <= endS;
             ++index) {
            if (globalPath.waypoints[index].roadId == 0 ||
                globalPath.waypoints[index].laneId == 0) {
                traceDecision(trace, "road_corridor",
                              "global_path_selected",
                              "reason=synthetic_checkpoint_connector");
                return false;
            }
        }
    }

    const LaneKey start{world.egoRoadId, world.egoSectionIndex,
                        world.egoLaneId};
    const auto startSegment = map.laneSegments.find(start);
    if (startSegment == map.laneSegments.end() ||
        startSegment->second.points.size() < 2) return false;

    std::vector<int> roadCorridor{world.egoRoadId};
    for (std::size_t index = nearestGlobalIndex;
         index < globalPath.waypoints.size(); ++index) {
        const int roadId = globalPath.waypoints[index].roadId;
        if (roadId != roadCorridor.back()) roadCorridor.push_back(roadId);
    }
    const bool hasNextRoad = roadCorridor.size() > 1;
    const int nextRoadId = hasNextRoad ? roadCorridor[1] : -1;

    std::vector<LaneKey> laneRoute{start};
    LaneKey corridorLane = start;
    bool connectsToNextRoad = !hasNextRoad;
    for (std::size_t roadIndex = 1;
         roadIndex < roadCorridor.size(); ++roadIndex) {
        std::vector<LaneKey> leg;
        LaneKey reachedLane;
        if (!findForwardLaneRoute(map, corridorLane,
                                  roadCorridor[roadIndex], leg,
                                  reachedLane)) {
            if (roadIndex == 1) connectsToNextRoad = false;
            break;
        }
        if (roadIndex == 1) connectsToNextRoad = true;
        for (std::size_t legIndex = 1; legIndex < leg.size(); ++legIndex) {
            laneRoute.push_back(leg[legIndex]);
        }
        corridorLane = reachedLane;
        double estimated = 0.0;
        for (const LaneKey& key : laneRoute) {
            const auto segment = map.laneSegments.find(key);
            if (segment != map.laneSegments.end()) {
                estimated += segment->second.length;
            }
        }
        if (estimated >= horizon + 20.0) break;
    }
    if (hasNextRoad && !connectsToNextRoad) {
        // 다음 road로 못 가는 차선이어도 도로 마지막 정렬 구간
        // 전까지는 이 차선을 자유롭게 유지한다.
        for (std::size_t guard = 0;
             guard < map.roads.at(world.egoRoadId).sections.size() + 2;
             ++guard) {
            const LaneKey current = laneRoute.back();
            const auto edges = map.laneGraph.find(current);
            if (edges == map.laneGraph.end()) break;
            const auto next = std::find_if(
                edges->second.begin(), edges->second.end(),
                [&](const LaneKey& key) {
                    return key.roadId == world.egoRoadId &&
                           !(key.sectionIndex == current.sectionIndex &&
                             key.laneId != current.laneId) &&
                           std::find(laneRoute.begin(), laneRoute.end(), key) ==
                               laneRoute.end();
                });
            if (next == edges->second.end()) break;
            laneRoute.push_back(*next);
        }
    }

    std::vector<Waypoint> points;
    const auto& firstPoints = startSegment->second.points;
    std::size_t firstIndex = 0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < firstPoints.size(); ++index) {
        const double distance = std::hypot(
            firstPoints[index].x - world.ego.x,
            firstPoints[index].y - world.ego.y);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            firstIndex = index;
        }
    }
    for (std::size_t index = firstIndex; index < firstPoints.size(); ++index) {
        appendUnique(points, firstPoints[index]);
    }
    for (std::size_t routeIndex = 1;
         routeIndex < laneRoute.size(); ++routeIndex) {
        const auto segment = map.laneSegments.find(laneRoute[routeIndex]);
        if (segment == map.laneSegments.end()) return false;
        for (const Waypoint& point : segment->second.points) {
            appendUnique(points, point);
        }
    }
    if (points.size() < 2) return false;
    updateCandidateGeometry(points, world.egoPathS);

    // 제어와 충돌예측에 필요한 구간만 남겨 매 프레임 새로
    // 만든다. s는 구역/정지선을 위한 기존 route_s에 이어서 붙인다.
    const double endS = world.egoPathS + std::max(20.0, horizon);
    const auto end = std::upper_bound(
        points.begin(), points.end(), endS,
        [](double value, const Waypoint& point) { return value < point.s; });
    if (end != points.end() && end > points.begin() + 1) {
        points.erase(end, points.end());
    }
    output.waypoints = std::move(points);

    std::ostringstream detail;
    detail << "road=" << world.egoRoadId
           << " lane=" << world.egoLaneId
           << " next_road=" << nextRoadId
           << " connects=" << connectsToNextRoad
           << " points=" << output.waypoints.size();
    traceDecision(trace, "road_corridor", "tracking_path_built",
                  detail.str());
    return output.waypoints.size() >= 2;
}

bool tryPrepareRoadConnectionLaneChange(
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
    LaneKey routeLane;
    std::size_t nextRoadIndex{};
    if (!findNextGlobalRoadLane(routeLane, nextRoadIndex, globalPath, map,
                                nearestGlobalIndex, world.egoRoadId)) {
        return false;
    }

    const LaneKey current{world.egoRoadId, world.egoSectionIndex,
                          world.egoLaneId};
    std::vector<LaneKey> currentRoute;
    LaneKey currentReached;
    if (findForwardLaneRoute(map, current, routeLane.roadId,
                             currentRoute, currentReached)) {
        return false;
    }

    const auto roadFound = map.roads.find(world.egoRoadId);
    if (roadFound == map.roads.end() || world.egoSectionIndex < 0 ||
        world.egoSectionIndex >=
            static_cast<int>(roadFound->second.sections.size())) return false;
    const Road& road = roadFound->second;
    const double remaining = world.egoLaneId < 0
        ? road.length - world.egoRoadS : world.egoRoadS;
    if (remaining > config.roadConnectionPrepareDistance) return false;

    const LaneSection& section = road.sections[world.egoSectionIndex];
    std::vector<int> candidates;
    for (const auto& [laneId, lane] : section.lanes) {
        if (laneId == world.egoLaneId || laneId == 0 ||
            lane.type != "driving" ||
            (laneId > 0) != (world.egoLaneId > 0)) continue;
        std::vector<LaneKey> route;
        LaneKey reached;
        if (findForwardLaneRoute(
                map, {world.egoRoadId, world.egoSectionIndex, laneId},
                routeLane.roadId, route, reached)) {
            candidates.push_back(laneId);
        }
    }
    std::stable_sort(candidates.begin(), candidates.end(),
        [&](int left, int right) {
            return std::abs(left - world.egoLaneId) <
                   std::abs(right - world.egoLaneId);
        });

    for (int targetLaneId : candidates) {
        std::vector<Waypoint> candidate;
        LaneKey reconnectLane;
        std::size_t reconnectIndex{};
        bool reconnectValid{};
        bool awaitReconnect{};
        double shiftCompleteRouteS{};
        const bool targetOnLeft =
            turnSignalTowardLane(world, map, targetLaneId) == 1U;
        const int laneHops = std::abs(targetLaneId - world.egoLaneId);
        const double connectionTransitionLength =
            config.transitionLength *
            std::sqrt(static_cast<double>(std::max(1, laneHops)));
        if (!buildKeepTargetPath(candidate, reconnectLane, reconnectIndex,
                                 reconnectValid, awaitReconnect,
                                 shiftCompleteRouteS,
                                 connectionTransitionLength, -1.0,
                                 targetOnLeft,
                                 laneHops, nullptr,
                                 config, world, map, globalPath,
                                 nearestGlobalIndex, trace) ||
            !reconnectValid || !awaitReconnect ||
            !candidatePathIsSafe(candidate, safetyConfig, config, world,
                                 trace)) {
            continue;
        }
        const std::uint8_t signal = turnSignalTowardLane(
            world, map, targetLaneId);
        if (signal == 0) continue;
        state = {};
        state.phase = LaneChangePhase::Prepare;
        state.routeMode = LaneChangeRouteMode::KeepTargetLane;
        state.roadConnectionManeuver = true;
        state.obstacleId = -1;
        state.sourceRoadId = world.egoRoadId;
        state.sourceLaneId = world.egoLaneId;
        state.sourceSectionIndex = world.egoSectionIndex;
        state.targetLaneId = targetLaneId;
        state.reconnectLane = reconnectLane;
        state.reconnectLaneValid = true;
        state.reconnectGlobalIndex = reconnectIndex;
        state.awaitExactReconnect = true;
        state.turnSignal = signal;
        state.outwardTurnSignal = signal;
        state.startRouteS = world.egoPathS;
        // 신호/횡단보도 정지선이 다음 road 진입 전에 있어도, 정지선보다
        // 앞에서 끝나는 횡방향 정렬은 허용한다. 후보 경로의 끝은 다음
        // road까지 이어질 수 있으므로 실행 차단 판단에는 실제 차선 이동이
        // 끝나는 route-s를 사용한다.
        state.rejoinRouteS = shiftCompleteRouteS;
        state.path = std::move(candidate);
        traceDecision(trace, "road_corridor", "connection_lane_selected",
                      "alignment_started_only_near_road_end");
        return true;
    }
    traceDecision(trace, "road_corridor", "connection_lane_unavailable",
                  "will_retry_before_road_boundary");
    return false;
}

bool tryPrepareObstacleLaneChange(
    LaneChangeData& state,
    const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safetyConfig,
    const WorldState& world,
    const PathGeneratorData& map,
    const PathPlannerData& globalPath,
    std::size_t nearestGlobalIndex,
    double* pendingBlockerDistance,
    const std::vector<int>* confirmedStationaryObjectIds,
    DecisionTraceData* trace) {
    if (pendingBlockerDistance != nullptr) *pendingBlockerDistance = -1.0;
    if (state.phase != LaneChangePhase::None || !world.egoLaneValid ||
        !world.egoPathValid) return false;
    const WorldObjectState* detectedObstacle = findBlockingObstacle(
        world, config, map, confirmedStationaryObjectIds, trace);
    if (detectedObstacle == nullptr) return false;
    const WorldObjectState routeMatchedObstacle = detectedObstacle->laneValid
        ? *detectedObstacle
        : matchBlockingObstacleToRoute(
              *detectedObstacle, map, globalPath, nearestGlobalIndex, trace);
    if (pendingBlockerDistance != nullptr) {
        *pendingBlockerDistance = routeMatchedObstacle.relativeS;
    }
    const WorldObjectState* obstacle = &routeMatchedObstacle;

    // 후보 차선은 Ego가 아니라 "장애물이 놓인 차선"의 양옆이다. 장애물 앞
    // 구간이 편도 1차선이어도, 장애물이 있는 도로에 옆 차선이 있으면 회피할 수
    // 있어야 한다.
    const int obstacleLaneId =
        obstacle->laneValid ? obstacle->laneId : world.egoLaneId;
    struct LaneCandidate {
        int laneId{};
        bool targetOnLeft{};
        int laneHops{};
        double forwardClearance{std::numeric_limits<double>::infinity()};
    };
    std::vector<LaneCandidate> laneCandidates;

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

    const int maximumLaneHops = static_cast<int>(obstacleSection.lanes.size());
    const RoadPose obstacleReference = roadToWorld(obstacleRoad, obstacle->roadS);
    const double obstacleDrivingYaw = obstacleLaneId < 0
        ? obstacleReference.yaw
        : normalizeAngle(obstacleReference.yaw + 3.14159265358979323846);
    for (int laneHops = 1; laneHops <= maximumLaneHops; ++laneHops) {
        for (const bool targetOnLeft : {true, false}) {
            const auto lane = drivingLaneOnSide(
                obstacleRoad, obstacleSection, obstacleLaneId,
                obstacle->roadS, obstacleDrivingYaw, targetOnLeft,
                laneHops);
            if (lane) {
                const LaneKey targetLane{
                    obstacle->roadId, obstacle->sectionIndex, *lane};
                // 같은 section/lane id만 비교하면 바로 다음 section에서
                // 번호가 바뀐 장애물을 놓치고 막힌 차선을 빈 차선으로
                // 평가한다. 횡방향 edge를 제외한 lane successor 그래프를
                // 따라가며 실제 전방 연결 위의 첫 정적 장애물을 찾는다.
                // 두 칸 이상 옮길 때는 최종 차선뿐 아니라 반드시 통과하는
                // 중간 차선도 모두 회피 회랑이다. 각 차선의 전방 여유 중
                // 최솟값을 후보 점수로 써, 중간 차선 장애물을 건너뛰지 않는다.
                double clearance = std::numeric_limits<double>::infinity();
                int crossedLaneId = obstacleLaneId;
                bool completeCorridor = true;
                for (int hop = 0; hop < laneHops; ++hop) {
                    const auto crossedLane = adjacentDrivingLaneOnSide(
                        obstacleRoad, obstacleSection, crossedLaneId,
                        obstacle->roadS, obstacleDrivingYaw, targetOnLeft);
                    if (!crossedLane) {
                        completeCorridor = false;
                        break;
                    }
                    crossedLaneId = *crossedLane;
                    const LaneKey crossedKey{
                        obstacle->roadId, obstacle->sectionIndex,
                        crossedLaneId};
                    clearance = std::min(
                        clearance,
                        forwardStaticClearance(
                            world, map, crossedKey, world.egoPathS,
                            static_cast<int>(obstacle->raw.id)));
                }
                if (!completeCorridor || crossedLaneId != targetLane.laneId) {
                    continue;
                }
                laneCandidates.push_back(
                    {*lane, targetOnLeft, laneHops, clearance});
            }
        }
    }

    // 장애물 회피 단계에서는 전역경로 차선을 보지 않는다.
    // 빈 차선이 하나라도 있으면 빈 차선 중 최소 횡이동을
    // 선택한다. 접근 가능한 차선이 모두 막혀 있으면
    // 횡이동 횟수보다 다음 장애물까지의 전방 여유를
    // 우선해 가장 멀리 진행할 수 있는 차선으로 피한다.
    // 다음 장애물은 안착 즉시 새 차선에서 다시 계획한다.
    std::stable_sort(laneCandidates.begin(), laneCandidates.end(),
        [](const LaneCandidate& lhs, const LaneCandidate& rhs) {
            const bool lhsClear = std::isinf(lhs.forwardClearance);
            const bool rhsClear = std::isinf(rhs.forwardClearance);
            if (lhsClear != rhsClear) return lhsClear;
            if (!lhsClear &&
                std::abs(lhs.forwardClearance - rhs.forwardClearance) > 1e-6) {
                return lhs.forwardClearance > rhs.forwardClearance;
            }
            return lhs.laneHops < rhs.laneHops;
        });

    for (const LaneCandidate& candidate : laneCandidates) {
        std::ostringstream detail;
        detail << "lane=" << candidate.laneId
               << " side=" << (candidate.targetOnLeft ? "left" : "right")
               << " hops=" << candidate.laneHops
               << " topology_clearance=" << candidate.forwardClearance;
        traceDecision(trace, "lane_change", "target_candidate_ranked",
                      detail.str());
    }

    LaneChangeData bestProgressCandidate;
    bool hasBestProgressCandidate = false;
    double bestPostShiftClearance = -1.0;
    double bestForwardClearance = -1.0;

    for (const LaneCandidate& laneCandidate : laneCandidates) {
        const int targetObstacleLaneId = laneCandidate.laneId;
        const int laneHops = laneCandidate.laneHops;
        const std::optional<bool> targetOnLeft = laneCandidate.targetOnLeft;
        const std::optional<int> targetEgoLane = drivingLaneOnSide(
            egoRoad, egoSection, world.egoLaneId, world.egoRoadS,
            world.ego.heading, *targetOnLeft, laneHops);
        const bool targetMayAppearAhead =
            obstacle->roadId != world.egoRoadId ||
            obstacle->sectionIndex != world.egoSectionIndex;
        {
            std::ostringstream detail;
            detail << "obstacle=" << obstacle->raw.id
                   << " obstacle_source_lane=" << obstacleLaneId
                   << " obstacle_target_lane=" << targetObstacleLaneId
                   << " ego_source_lane=" << world.egoLaneId
                   << " ego_target_lane="
                   << (targetEgoLane ? *targetEgoLane : 0)
                   << " side="
                   << (targetOnLeft
                           ? (*targetOnLeft ? "left" : "right")
                           : "invalid");
            traceDecision(trace, "lane_change", "target_candidate_started",
                          detail.str());
        }
        if (!targetOnLeft) {
            traceDecision(trace, "lane_change", "target_candidate_rejected",
                          "reason=physical_side_has_no_adjacent_lane_at_obstacle");
            continue;
        }
        const bool canBuildKeepTarget =
            targetEgoLane.has_value() || targetMayAppearAhead;
        if (!canBuildKeepTarget) {
            traceDecision(
                trace, "lane_change", "target_candidate_rejected",
                "reason=keep_target_has_no_adjacent_lane_in_current_section; checking_return_candidate");
        }
        const int targetEgoLaneId =
            targetEgoLane.value_or(targetObstacleLaneId);
        const LaneKey obstacleTargetKey{
            obstacle->roadId, obstacle->sectionIndex,
            targetObstacleLaneId};
        // 앞쪽 lane section에서 새 차선이 생기는 경우에는 현재 section에
        // target lane이 아직 없다. 물리적 회피 방향은 이미 장애물
        // section에서 확정됐으므로 그 방향으로 지시등을 미리 켠다.
        const std::uint8_t signal = targetEgoLane
            ? turnSignalTowardLane(world, map, *targetEgoLane)
            : (targetMayAppearAhead ? (*targetOnLeft ? 1U : 2U) : 0U);
        struct LongitudinalVariant {
            double transitionLength{};
            double desiredCompletionDistance{-1.0};
            std::string name;
        };
        const double mediumLength = std::min(config.transitionLength, 15.0);
        const double compactLength = std::min(config.transitionLength, 12.0);
        const double tightLength = std::min(config.transitionLength, 6.0);
        std::vector<LongitudinalVariant> variants;
        // 고정된 3개 완료점만 검사하면 지그재그 장애물 사이의 좁은
        // 빈 구간을 건너뛴다. 차선변경을 끝낼 수 있는 범위를 3 m
        // 간격으로 훑어서 장애물 사이에 실제로 들어갈 수 있는 경로를
        // 찾는다. 먼저 완만한 15 m 횡이동을, 필요할 때만 12 m
        // 횡이동을 검사한다.
        const auto appendVariants = [&](double transitionLength,
                                        const char* family) {
            const std::size_t familyBegin = variants.size();
            // 완료점을 장애물 6 m 앞에서 강제로 끊으면, 두 차선의 장애물이
            // 엇갈린 슬라럼에서 답이 사라진다. 목표 차선의 앞 장애물을 지난
            // 뒤 현재 차선 장애물 직전에 안착하는 대각 경로가 필요할 수 있다.
            // 장애물 뒤쪽까지도 격자로 생성하고 실제 차량/장애물 OBB가 전체
            // 기동에서 겹치지 않는 후보만 아래 안전검사로 남긴다.
            const double lastCompletion = std::max(
                transitionLength,
                obstacle->relativeS + config.obstacleClearance);
            constexpr double kCompletionStep = 3.0;
            int sampleIndex = 0;
            for (double completion = transitionLength;
                 completion <= lastCompletion + 1e-6;
                 completion += kCompletionStep, ++sampleIndex) {
                variants.push_back({
                    transitionLength, completion,
                    std::string(family) + "_" + std::to_string(sampleIndex)});
            }
            if (variants.size() == familyBegin ||
                lastCompletion - variants.back().desiredCompletionDistance >
                    0.75) {
                variants.push_back({
                    transitionLength, lastCompletion,
                    std::string(family) + "_last"});
            }
        };
        appendVariants(mediumLength, "normal");
        if (compactLength + 1e-6 < mediumLength) {
            appendVariants(compactLength, "compact");
        }
        // 연속 장애물 사이가 좁을 때만 마지막 수단으로 검사한다.
        // 속도는 maximumLaneChangeSpeed로 제한되고, 실제 OBB 충돌 및
        // 생성 경로 곡률 검사를 모두 통과해야 선택된다.
        if (tightLength + 1e-6 < compactLength) {
            appendVariants(tightLength, "tight");
        }
        std::vector<Waypoint> selectedCandidate;
        LaneKey selectedReconnectLane;
        std::size_t selectedReconnectIndex{};
        bool selectedReconnectLaneValid{};
        double selectedShiftCompleteRouteS{};
        std::string selectedVariant;
        for (const LongitudinalVariant& variant : variants) {
            if (!canBuildKeepTarget || signal == 0) break;
            std::vector<Waypoint> candidate;
            LaneKey reconnectLane;
            std::size_t reconnectIndex{};
            bool reconnectLaneValid{};
            bool awaitExactReconnect{};
            double shiftCompleteRouteS{};
            // 같은 road여도 lane section이 다르면 중간에서 차선이 늘거나
            // 줄며 lane id가 바뀌다. 완성 차선이 장애물 위치의 정확한
            // 회피 차선으로 종방향 연결되는지까지 검증한다.
            const LaneKey* requiredForwardLane = targetMayAppearAhead
                ? &obstacleTargetKey
                : nullptr;
            if (!buildKeepTargetPath(
                    candidate, reconnectLane, reconnectIndex,
                    reconnectLaneValid, awaitExactReconnect,
                    shiftCompleteRouteS, variant.transitionLength,
                    variant.desiredCompletionDistance,
                    *targetOnLeft, laneHops,
                    requiredForwardLane,
                    config, world, map, globalPath, nearestGlobalIndex,
                    trace)) {
                continue;
            }
            // 현재 회피와 안착 접선까지만 이 후보에서 검증한다. 다음
            // 지그재그 장애물까지 미리 포함하면 협소한 빈 공간으로 들어가는
            // 현재 회피마저 모두 폐기된다. 대신 기동 상태가 실제로 끝날 때까지
            // maximumLaneChangeSpeed를 유지해 안착 즉시 다음 회피를 수행할 거리를 남긴다.
            constexpr double kPostShiftTrackingTail = 2.0;
            const double safetyPrefixEnd = obstacle->roadId == world.egoRoadId
                ? shiftCompleteRouteS + kPostShiftTrackingTail
                : std::max(shiftCompleteRouteS + kPostShiftTrackingTail,
                           obstacle->pathS + kPostShiftTrackingTail);
            candidate = maneuverPathPrefix(
                candidate, safetyPrefixEnd);
            {
                std::ostringstream detail;
                detail << "variant=" << variant.name
                       << " transition_length=" << variant.transitionLength
                       << " completion_distance="
                       << (shiftCompleteRouteS - world.egoPathS);
                traceDecision(trace, "lane_change",
                              "longitudinal_variant_checked", detail.str());
            }
            if (!candidatePathIsSafe(candidate, safetyConfig, config, world,
                                     trace)) {
                continue;
            }
            selectedCandidate = std::move(candidate);
            selectedReconnectLane = reconnectLane;
            selectedReconnectIndex = reconnectIndex;
            selectedReconnectLaneValid = reconnectLaneValid;
            selectedShiftCompleteRouteS = shiftCompleteRouteS;
            selectedVariant = variant.name;
            break;
        }
        if (!selectedVariant.empty()) {
            LaneChangeData prepared;
            prepared.phase = LaneChangePhase::Prepare;
            prepared.routeMode = LaneChangeRouteMode::KeepTargetLane;
            prepared.obstacleId = static_cast<int>(obstacle->raw.id);
            prepared.sourceRoadId = world.egoRoadId;
            prepared.sourceLaneId = world.egoLaneId;
            prepared.sourceSectionIndex = world.egoSectionIndex;
            prepared.targetLaneId = targetEgoLaneId;
            // 장애물 회피는 현재 road의 목표 차선에 안착하면 끝낸다.
            // 다음 road의 exact lane에 도달할 때까지 상태를 유지하면
            // 그 사이 등장하는 두 번째·세 번째 장애물을 재계획할 수 없다.
            // 다음 road 연결 차선 정렬은 roadConnectionManeuver가 도로
            // 마지막 구간에서 따로 처리한다.
            prepared.reconnectLane = selectedReconnectLane;
            prepared.reconnectLaneValid = selectedReconnectLaneValid;
            prepared.reconnectGlobalIndex = selectedReconnectIndex;
            prepared.awaitExactReconnect = false;
            prepared.roadConnectionManeuver = false;
            prepared.turnSignal = signal;
            prepared.outwardTurnSignal = signal;
            prepared.startRouteS = world.egoPathS;
            prepared.rejoinRouteS = selectedShiftCompleteRouteS;

            // 첫 횡이동만 충돌하지 않는다고 즉시 확정하지 않는다. 안착
            // 차선의 종방향 successor를 따라 다음 정적 장애물을 찾고,
            // 다음 12 m급 회피를 준비할 거리가 남는 후보를 우선한다.
            // 모든 차선이 이 여유보다 짧으면 후보를 폐기하지 않고 아래에서
            // 가장 긴 여유를 가진 안전한 1단계 후보를 선택한다.
            const LaneKey completedLane = laneKeyAtPathS(
                selectedCandidate, selectedShiftCompleteRouteS);
            prepared.shiftCompletedLane = completedLane;
            prepared.shiftCompletedLaneValid = true;
            const WorldObjectState* followup =
                nearestStaticObjectOnForwardLane(
                    world, map, completedLane,
                    selectedShiftCompleteRouteS,
                    static_cast<int>(obstacle->raw.id));
            const double postShiftClearance = followup == nullptr
                ? std::numeric_limits<double>::infinity()
                : followup->pathS - selectedShiftCompleteRouteS;
            const double followupReserve =
                std::min(config.transitionLength, 12.0) +
                config.maximumLaneChangeSpeed * config.prepareTime;
            if (followup != nullptr) {
                prepared.followupObstacleId =
                    static_cast<int>(followup->raw.id);
                prepared.followupObstacleClearance = postShiftClearance;
            }
            prepared.path = std::move(selectedCandidate);
            std::ostringstream detail;
            detail << "mode=keep_target_lane phase=prepare variant="
                   << selectedVariant
                   << " post_shift_clearance=" << postShiftClearance
                   << " followup_reserve=" << followupReserve
                   << " followup_object=" << prepared.followupObstacleId;
            if (!std::isfinite(postShiftClearance) ||
                postShiftClearance + 1e-6 >= followupReserve) {
                state = std::move(prepared);
                traceDecision(trace, "lane_change", "candidate_selected",
                              detail.str());
                return true;
            }

            detail << " result=held_as_best_progress_fallback";
            traceDecision(trace, "lane_change", "candidate_sequence_checked",
                          detail.str());
            const bool fartherAfterShift =
                !hasBestProgressCandidate ||
                postShiftClearance > bestPostShiftClearance + 1e-6 ||
                (std::abs(postShiftClearance - bestPostShiftClearance) <= 1e-6 &&
                 laneCandidate.forwardClearance > bestForwardClearance);
            if (fartherAfterShift) {
                prepared.bestProgressFallback = true;
                bestProgressCandidate = std::move(prepared);
                hasBestProgressCandidate = true;
                bestPostShiftClearance = postShiftClearance;
                bestForwardClearance = laneCandidate.forwardClearance;
            }
        }

        traceDecision(trace, "lane_change", "return_candidate_started",
                      "keep-target route unavailable or unsafe; checking out-and-back route");

        const double intrusion = obstacleIntrusionIntoTargetLane(
            *obstacle, obstacleRoad, obstacleSection,
            targetObstacleLaneId);
        std::uint8_t returnCandidateSignal{};
        double returnStartS{};
        double rejoinS{};
        bool selected = false;
        double selectedOffset = 0.0;
        std::vector<Waypoint> candidate;
        for (double lateralOffset = 0.0;
             lateralOffset <= config.maximumLateralOffset + 1e-9;
             lateralOffset += config.lateralOffsetStep) {
            if (lateralOffset > 1e-9 &&
                intrusion > config.maximumObstacleLaneIntrusion) {
                std::ostringstream detail;
                detail << "intrusion=" << intrusion
                       << " maximum="
                       << config.maximumObstacleLaneIntrusion;
                traceDecision(trace, "lateral_offset",
                              "offset_search_rejected", detail.str());
                break;
            }
            candidate.clear();
            if (!buildReturnCandidatePath(
                    candidate, returnCandidateSignal, laneHops, returnStartS, rejoinS,
                    targetObstacleLaneId, *targetOnLeft,
                    lateralOffset, *obstacle, config, world, map, globalPath,
                    nearestGlobalIndex, trace)) {
                if (lateralOffset <= 1e-9) {
                    traceDecision(trace, "lane_change",
                                  "return_candidate_rejected",
                                  "reason=topology_geometry_gap_or_road_mark_check_failed");
                } else {
                    std::ostringstream detail;
                    detail << "offset=" << lateralOffset
                           << " reason=lane_edge_or_geometry_limit";
                    traceDecision(trace, "lateral_offset",
                                  "candidate_rejected", detail.str());
                }
                continue;
            }
            if (candidatePathIsSafe(candidate, safetyConfig, config, world,
                                    trace)) {
                selected = true;
                selectedOffset = lateralOffset;
                break;
            }
            std::ostringstream detail;
            detail << "offset=" << lateralOffset
                   << " intrusion=" << intrusion
                   << " result=obb_collision";
            traceDecision(trace, "lateral_offset", "candidate_rejected",
                          detail.str());
        }
        if (!selected) {
            traceDecision(trace, "lane_change", "return_candidate_rejected",
                          "reason=no_safe_center_or_lateral_offset_candidate");
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
        {
            std::ostringstream detail;
            detail << "mode=return_to_source phase=prepare requested_lateral_offset="
                   << selectedOffset << " obstacle_intrusion=" << intrusion;
            traceDecision(trace, "lane_change", "candidate_selected",
                          detail.str());
        }
        return true;
    }
    if (hasBestProgressCandidate) {
        state = std::move(bestProgressCandidate);
        std::ostringstream detail;
        detail << "mode=keep_target_lane phase=prepare"
               << " post_shift_clearance=" << bestPostShiftClearance
               << " result=best_safe_progress_when_all_lanes_blocked";
        traceDecision(trace, "lane_change", "candidate_selected",
                      detail.str());
        return true;
    }
    traceDecision(trace, "lane_change", "no_safe_candidate",
                  "no geometrically trackable collision-free lateral progress exists");
    return false;
}

void advanceLaneChangePhase(LaneChangeData& state,
                            const LaneChangeConfig& config,
                            const WorldState& world,
                            double dt,
                            bool executionBlocked,
                            bool obstacleStillRequiresManeuver,
                            bool candidatePathSafe,
                            DecisionTraceData* trace) {
    if (state.phase == LaneChangePhase::None) return;
    if (state.phase == LaneChangePhase::Prepare) {
        // Prepare 중에는 아직 횡이동을 시작하지 않았다. 이때 원인이던 객체가
        // 재출발하면 보관된 경로를 실행하지 않고 취소한다. Execute 진입 후에는
        // 판단을 뒤집지 않고 기동을 완주해 좌우 와리가리를 막는다.
        if (!state.roadConnectionManeuver &&
            !obstacleStillRequiresManeuver) {
            traceDecision(trace, "lane_change", "phase_transition",
                          "prepare->none reason=blocking_object_moved_or_disappeared");
            resetLaneChange(state);
            return;
        }
        if (executionBlocked) {
            if (state.roadConnectionManeuver) {
                std::ostringstream detail;
                detail << "execution_blocked=" << executionBlocked
                       << " candidate_path_safe=" << candidatePathSafe;
                traceDecision(trace, "lane_change", "prepare_cancelled",
                              detail.str());
                traceDecision(trace, "lane_change", "phase_transition",
                              "prepare->none reason=traffic_stop_active_replan_after_release");
                resetLaneChange(state);
                return;
            }
            // 정적 장애물이 정지선 가까이에 있으면 신호/보행자 정지는
            // 종방향 제어가 그대로 수행하되, 충돌을 피하기 위한 횡이동은
            // 정지점까지 계속한다. 여기서 경로를 취소하면 매 프레임 같은
            // 후보를 재생성하다 장애물 정면에 멈추게 된다.
            traceDecision(trace, "lane_change",
                          "traffic_stop_with_avoidance_retained",
                          "longitudinal stop remains active while lateral avoidance proceeds");
        }
        if (!candidatePathSafe) {
            traceDecision(trace, "lane_change", "phase_transition",
                          "prepare->none reason=candidate_became_unsafe");
            resetLaneChange(state);
            return;
        }
        if (state.roadConnectionManeuver &&
            std::abs(world.egoSpeed) >
                config.maximumLaneChangeSpeed + 0.25) {
            // 여러 차선을 건너는 도로 연결 정렬은 감속이 실제로
            // 끝난 뒤에만 실행한다. 지시등은 이 동안 계속 유지된다.
            state.phaseElapsed = 0.0;
            traceDecision(trace, "road_corridor", "prepare_speed_hold",
                          "execute_delayed_until_vehicle_reaches_lane_change_speed");
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
            // road 연결 정렬은 정확한 다음 road/lane에 도착해야 끝나고,
            // 장애물 회피는 생성 경로의 횡이동 완료점에 안착하면 끝난다.
            // lane ID는 lane section 경계를 지나며 2 -> 3처럼 바뀔 수
            // 있으므로 시작 section의 targetLaneId와 직접 비교하면 정상
            // 안착 후에도 기동이 영원히 끝나지 않는다. 완료점 접선에 대한
            // 횡오차로 실제 안착 여부를 판정한다.
            double completionLateralError =
                std::numeric_limits<double>::infinity();
            if (!state.path.empty()) {
                const auto completion = std::min_element(
                    state.path.begin(), state.path.end(),
                    [&](const Waypoint& lhs, const Waypoint& rhs) {
                        return std::abs(lhs.s - state.rejoinRouteS) <
                               std::abs(rhs.s - state.rejoinRouteS);
                    });
                const double dx = world.ego.x - completion->x;
                const double dy = world.ego.y - completion->y;
                completionLateralError = std::abs(
                    -std::sin(completion->yaw) * dx +
                     std::cos(completion->yaw) * dy);
            }
            constexpr double kMaximumCompletionLateralError = 0.75;
            const bool reconnected =
                state.awaitExactReconnect
                    ? (state.reconnectLaneValid && world.egoLaneValid &&
                       world.egoRoadId == state.reconnectLane.roadId &&
                       world.egoSectionIndex == state.reconnectLane.sectionIndex &&
                       world.egoLaneId == state.reconnectLane.laneId)
                    : (world.egoPathS >= state.rejoinRouteS - 1.0 &&
                       completionLateralError <=
                           kMaximumCompletionLateralError);
            if (reconnected) {
                if (!state.roadConnectionManeuver) {
                    traceDecision(trace, "lane_change", "phase_transition",
                                  "execute->none obstacle_shift_complete_replan_enabled");
                    resetLaneChange(state);
                    return;
                }
                state.phase = LaneChangePhase::Settle;
                state.phaseElapsed = 0.0;
                state.turnSignal = 0;
                traceDecision(trace, "lane_change", "phase_transition",
                              state.awaitExactReconnect
                                  ? "execute->settle mode=keep_target_lane exact_reconnect_lane_reached"
                                  : "execute->settle mode=keep_target_lane route_goal_reached");
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
        // 예정된 복귀 s를 통과했더라도 차가 아직 전역경로에서
        // 크게 떨어져 있으면 지역경로 추종을 계속한다. M05에서는
        // 약 2.8 m 떨어진 상태에서 넘겨 버려 조향이 반대로 튀었다.
        constexpr double kMaximumRejoinLateralError = 0.75;
        if (world.egoPathS >= state.rejoinRouteS - 1.0 &&
            std::abs(world.egoPathD) <= kMaximumRejoinLateralError) {
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

bool hasLaneChangeExecutionBlocker(const StopManagerData& stopManager) {
    for (const auto& request : stopManager.requests) {
        if (!request.active) continue;
        return true;
    }
    return false;
}

bool hasLaneChangeExecutionBlocker(const StopManagerData& stopManager,
                                   double maneuverEndRouteS) {
    constexpr double kCompletionBeforeStopMargin = 2.0;
    for (const auto& request : stopManager.requests) {
        if (!request.active) continue;
        // 먼 정지선이 있다는 이유만으로 그 앞의 장애물 회피까지 막지 않는다.
        // 회피를 정지선 전에 끝낼 수 있을 때는 기동을 먼저 완주하고,
        // 정지선과 횡단보도 정지는 완성된 회피 차선에서 그대로 수행한다.
        if (maneuverEndRouteS + kCompletionBeforeStopMargin <
            request.stopRouteS) {
            continue;
        }
        return true;
    }
    return false;
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
                             double egoSpeed) {
    PathPlannerData temporary;
    temporary.config = plannerConfig;
    temporary.waypoints = state.path;
    temporary.nearestIndex = state.nearestIndex;
    PathPlan plan = runPurePursuit(temporary, gt, egoSpeed);
    state.nearestIndex = temporary.nearestIndex;
    // 차선변경 경로의 끝은 중간 hand-off 지점이지 최종 목적지가
    // 아니다. 연결되지 않는 회피 차선 끝에서도 0 km/h를 내지 않는다.
    plan.goalReached = false;
    return plan;
}
