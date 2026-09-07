#include "planning/planning_process.hpp"

#include "data/gt/gt_processor.hpp"
#include "logger/decision_trace.hpp"
#include "planning/behavior/behavior_planner.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/local/local_path_planner.hpp"
#include "planning/local/lane_change_planner.hpp"
#include "planning/local/corridor_lattice.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace {

bool longitudinallyAdjacent(const PathGeneratorData& map,
                            const LaneKey& first,
                            const LaneKey& second) {
    if (first == second) return true;
    const auto hasForwardEdge = [&](const LaneKey& from,
                                    const LaneKey& to) {
        // 같은 section의 옆 차선 edge는 실제 section/road 전환이 아니다.
        if (from.roadId == to.roadId &&
            from.sectionIndex == to.sectionIndex &&
            from.laneId != to.laneId) {
            return false;
        }
        const auto found = map.laneGraph.find(from);
        return found != map.laneGraph.end() &&
            std::find(found->second.begin(), found->second.end(), to) !=
                found->second.end();
    };
    // 경계 샘플에서 맵매칭과 경로 투영의 전환 타이밍이 한 프레임
    // 어긋나도 버티도록 정방향과 역방향 직접 successor를 모두 인정한다.
    return hasForwardEdge(first, second) || hasForwardEdge(second, first);
}

bool trackingPathUsable(const PathPlannerData& planner,
                        const WorldState& sensedWorld,
                        const PathGeneratorData& map,
                        double& lateralDistance,
                        double& remainingDistance,
                        bool requireForwardHorizon = true) {
    lateralDistance = std::numeric_limits<double>::infinity();
    remainingDistance = 0.0;
    if (planner.waypoints.size() < 2 ||
        !std::isfinite(sensedWorld.ego.x) ||
        !std::isfinite(sensedWorld.ego.y) ||
        !std::isfinite(sensedWorld.ego.heading)) {
        return false;
    }

    WorldState projected = sensedWorld;
    if (!projectWorldToActivePath(
            projected, planner.waypoints,
            sensedWorld.frontBumperOffset)) {
        return false;
    }
    lateralDistance = std::abs(projected.egoPathD);
    // Lane map-match가 유효하고 아래 topology 검사까지 통과한 경로는
    // 충돌 회피/외력으로 차폭 한 칸가량 밀린 경우에도 복귀 가능한 현재
    // lane 경로다. 4.5 m 고정 컷으로 폐기하면 EBS 해제 뒤 정상 경로가
    // 매 프레임 생성돼도 planValid=0으로 영구 정지한다.
    const double maximumLateralDistance = std::max(
        8.0, 2.0 * sensedWorld.egoLaneWidth + 1.0);
    if (lateralDistance > maximumLateralDistance) return false;

    const auto nearest = std::lower_bound(
        planner.waypoints.begin(), planner.waypoints.end(),
        projected.egoPathS,
        [](const Waypoint& point, double pathS) {
            return point.s < pathS;
        });
    const Waypoint& pathPoint = nearest == planner.waypoints.end()
        ? planner.waypoints.back() : *nearest;
    if (std::abs(normalizeAngle(
            sensedWorld.ego.heading - pathPoint.yaw)) > 1.35) {
        return false;
    }

    if (sensedWorld.egoLaneValid && pathPoint.roadId != 0 &&
        pathPoint.laneId != 0) {
        const LaneKey sensedLane{
            sensedWorld.egoRoadId, sensedWorld.egoSectionIndex,
            sensedWorld.egoLaneId};
        const LaneKey pathLane{
            pathPoint.roadId, pathPoint.sectionIndex, pathPoint.laneId};
        if (!longitudinallyAdjacent(map, sensedLane, pathLane) &&
            !corridorHasLongitudinalConnection(map, sensedLane, pathLane)) {
            return false;
        }
    }

    remainingDistance = planner.waypoints.back().s - projected.egoPathS;
    // 새 경로도 종점 직전의 짧은 tail이면 채택하지 않는다. gateway/section
    // 전환 경로가 미리 준비돼야만 교체되고, 그 전까지는 기존 정상 경로를
    // 유지한다. 이렇게 해야 짧은 끝점을 쫓는 급조향과 local-path 공백을
    // 동시에 막을 수 있다.
    const double requiredAhead =
        std::max(2.0, std::abs(sensedWorld.egoSpeed) * 0.35);
    if (!requireForwardHorizon) {
        // section 끝에서 successor/Gateway 경로가 아직 준비되지
        // 않았다고 현재 차선의 짧은 tail까지 폐기하면 계획
        // 자체가 무효화되고 다음 경로를 만들 입력도 사라진다.
        // 차량 위치가 아직 경로 종점을 지나지 않았다면
        // 연속성 fallback으로 사용한다.
        return remainingDistance >= -0.25;
    }
    return remainingDistance >= requiredAhead;
}

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
        if (!line.routeRelevant) continue;
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

// 처음부터 멈춰 있는 물체는 고정 장애물로 보고 즉시 회피한다.
// 반면 한 번이라도 움직이던 차량은 끼어들기/급정거에 대비해 먼저
// 추종·긴급제동으로 반응하고, 짧은 시간 계속 정지했을 때만 회피
// 대상으로 전환한다.
std::vector<int> updateConfirmedStationaryObjects(
    PlanningProcessData& data, const WorldState& world, double dt) {
    std::vector<StationaryObjectState> current;
    std::vector<int> confirmed;
    current.reserve(world.objects.size());

    for (const auto& object : world.objects) {
        const auto previous = std::find_if(
            data.stationaryObjects.begin(), data.stationaryObjects.end(),
            [&](const StationaryObjectState& state) {
                return state.id == object.raw.id;
            });
        const bool wasTracked = previous != data.stationaryObjects.end();
        const double elapsed = std::max(0.0, dt);
        const double displacementSpeed =
            wasTracked && previous->hasPosition && elapsed > 1e-3
                ? std::hypot(object.raw.x - previous->lastX,
                             object.raw.y - previous->lastY) / elapsed
                : 0.0;
        const bool moving = std::abs(object.raw.speed) >= 0.5 ||
                            displacementSpeed >= 0.5;
        const bool stationary = std::abs(object.raw.speed) < 0.3 &&
                                (!wasTracked || displacementSpeed < 0.3);
        const bool hasEverMoved = moving ||
            (wasTracked && previous->hasEverMoved);
        const double stationaryDuration = stationary
            ? (wasTracked ? previous->stationaryDuration : 0.0) + elapsed
            : 0.0;

        current.push_back({object.raw.id, stationaryDuration,
                           static_cast<double>(object.raw.x),
                           static_cast<double>(object.raw.y), true,
                           hasEverMoved});
        if (stationary &&
            (!hasEverMoved || stationaryDuration >=
                                  data.laneChangeConfig
                                      .stoppedDynamicConfirmationTime)) {
            confirmed.push_back(static_cast<int>(object.raw.id));
        }
    }
    data.stationaryObjects = std::move(current);
    return confirmed;
}

bool crosswalkHasRiskObject(const Crosswalk& crosswalk,
                            const WorldState& world,
                            double predictionHorizon) {
    return std::any_of(
        world.objects.begin(), world.objects.end(),
        [&](const WorldObjectState& object) {
            // GT packet에 semantic type은 없지만 크기로 승용차급
            // 객체는 구분할 수 있다. 횡단보도 안에 멈춘 차량을
            // CrosswalkBlocked로 다시 정지 요청하면, 바로 그 차량을
            // 피하려고 만든 안전한 차선변경 경로가 prepare에서
            // 취소된다. 멈춘 대형 객체는 정적장애물 회피가
            // 담당한다. 보행자·자전거 크기 객체와 움직이는
            // 차량은 기존 횡단보도 정지/예측 제동을 그대로 유지한다.
            constexpr double kVulnerableRoadUserMaximumSize = 2.5;
            const double maximumSize = std::max(
                static_cast<double>(object.raw.length),
                static_cast<double>(object.raw.width));
            const bool stationaryVehicleSizedObject =
                std::abs(object.raw.speed) < 0.3 &&
                maximumSize > kVulnerableRoadUserMaximumSize;
            if (stationaryVehicleSizedObject) return false;
            // 중심점이 횡단보도 밖으로 나가도 보행자 체적의 뒤쪽이
            // 남아 있을 수 있다. polygon과 전체 OBB가 한 부분이라도
            // 겹치는 동안은 점유로 판정한다.
            if (intersects(crosswalk.polygon, object.obb)) return true;
            if (predictionHorizon <= 0.0 ||
                std::abs(object.raw.speed) < 0.3) return false;
            // horizon 끝점 하나만 보면 그 전에 횡단보도를 통과한 빠른 객체를
            // 놓친다. 0.25초 간격으로 궤적 전체를 검사한다.
            constexpr double kPredictionStep = 0.25;
            for (double time = kPredictionStep;
                 time <= predictionHorizon + 1e-9;
                 time += kPredictionStep) {
                const double predictedX =
                    object.raw.x + object.raw.speed *
                                       std::cos(object.raw.heading) * time;
                const double predictedY =
                    object.raw.y + object.raw.speed *
                                       std::sin(object.raw.heading) * time;
                const Obb2D predictedObb = makeObb2D(
                    predictedX, predictedY, object.raw.heading,
                    object.raw.length, object.raw.width);
                if (intersects(crosswalk.polygon, predictedObb)) {
                    return true;
                }
            }
            return false;
        });
}

CrosswalkOccupancyState* findCrosswalkState(
    PlanningProcessData& data, int crosswalkId) {
    const auto found = std::find_if(
        data.crosswalkStates.begin(), data.crosswalkStates.end(),
        [&](const CrosswalkOccupancyState& state) {
            return state.id == crosswalkId;
        });
    return found == data.crosswalkStates.end() ? nullptr : &*found;
}

void updateCrosswalkStates(PlanningProcessData& data,
                           const WorldState& world,
                           const StaticZoneData& zones,
                           double dt,
                           DecisionTraceData* trace) {
    data.crosswalkStates.erase(
        std::remove_if(
            data.crosswalkStates.begin(), data.crosswalkStates.end(),
            [&](const CrosswalkOccupancyState& state) {
                const auto crosswalk = std::find_if(
                    zones.crosswalks.begin(), zones.crosswalks.end(),
                    [&](const Crosswalk& candidate) {
                        return candidate.id == state.id;
                    });
                return crosswalk == zones.crosswalks.end() ||
                       !crosswalk->routeRelevant ||
                       (world.egoPathValid &&
                        crosswalk->exitRouteS < world.egoPathS);
            }),
        data.crosswalkStates.end());

    for (const auto& crosswalk : zones.crosswalks) {
        if (!crosswalk.routeRelevant ||
            (world.egoPathValid &&
             crosswalk.exitRouteS < world.egoPathS)) {
            continue;
        }
        const bool occupied = crosswalkHasRiskObject(
            crosswalk, world, data.config.crosswalkPedestrianPrediction);
        CrosswalkOccupancyState* state =
            findCrosswalkState(data, crosswalk.id);
        if (occupied) {
            if (state == nullptr) {
                data.crosswalkStates.push_back(
                    {crosswalk.id, true,
                     data.config.crosswalkClearanceHoldTime});
                traceDecision(trace, "crosswalk", "occupancy_started",
                              "full_object_obb_overlaps_or_is_predicted_to_overlap");
            } else {
                state->occupied = true;
                state->clearanceHoldRemaining =
                    data.config.crosswalkClearanceHoldTime;
            }
            continue;
        }
        if (state == nullptr) continue;
        if (state->occupied) {
            state->occupied = false;
            state->clearanceHoldRemaining =
                data.config.crosswalkClearanceHoldTime;
            std::ostringstream detail;
            detail << "crosswalk=" << crosswalk.id
                   << " hold_seconds="
                   << data.config.crosswalkClearanceHoldTime;
            traceDecision(trace, "crosswalk", "clearance_hold_started",
                          detail.str());
        } else {
            state->clearanceHoldRemaining = std::max(
                0.0, state->clearanceHoldRemaining - std::max(0.0, dt));
        }
    }
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

double routeHeadingAtS(const std::vector<Waypoint>& route, double routeS) {
    if (route.empty()) return 0.0;
    const auto found = std::lower_bound(
        route.begin(), route.end(), routeS,
        [](const Waypoint& waypoint, double value) {
            return waypoint.s < value;
        });
    if (found == route.end()) return route.back().yaw;
    return found->yaw;
}

// GT에는 semantic object type이 없으므로 보행자/자전거라는 이름에 의존하지
// 않는다. 대신 취약 도로사용자 크기의 OBB가 경로 쪽으로 횡이동하는 물리적
// 충돌 상황을 잡는다. 지도 횡단보도 매칭이 없거나 객체가 막 움직이기 시작한
// 프레임에도 jerk 제한을 거치지 않고 제동할 수 있는 마지막 안전망이다.
bool crossingObjectRequiresEmergency(
    const WorldState& world,
    const std::vector<Waypoint>& route,
    const PlanningBehaviorConfig& config,
    int& riskObjectId) {
    const double egoSpeed = std::max(0.0, world.egoSpeed);
    const double stoppingEnvelope =
        egoSpeed * config.emergencyReactionTime +
        egoSpeed * egoSpeed /
            (2.0 * config.emergencyBrakingDeceleration) +
        config.minimumFollowDistance;
    for (const auto& object : world.objects) {
        const double length = static_cast<double>(object.raw.length);
        const double width = static_cast<double>(object.raw.width);
        if (!object.pathValid || !object.isAhead ||
            object.relativeS > stoppingEnvelope ||
            std::abs(object.relativeD) >
                0.5 * std::max(3.0, world.egoLaneWidth) + 5.0 ||
            std::abs(object.raw.speed) < 0.3 ||
            length <= 0.0 || width <= 0.0 ||
            // 보행자뿐 아니라 자전거 크기도 포함한다. 승용차급 객체의
            // 끼어들기는 별도 OBB 예측과 선행차 제동 로직이 담당한다.
            std::max(length, width) > 2.5) {
            continue;
        }
        const double routeHeading =
            routeHeadingAtS(route, object.pathS);
        const double lateralVelocity =
            static_cast<double>(object.raw.speed) *
            std::sin(static_cast<double>(object.raw.heading) - routeHeading);
        const bool movingTowardPath =
            object.relativeD * lateralVelocity < -0.05;
        if (!movingTowardPath || std::abs(lateralVelocity) < 0.25) continue;
        riskObjectId = static_cast<int>(object.raw.id);
        return true;
    }
    return false;
}

// 선행차가 급제동하면 등속 OBB 충돌 TTC가 갱신되기 전에 상대 정지거리가
// 위험해질 수 있다. 반응시간 동안 진행할 거리까지 빼고 필요한 감속도를
// 계산해, 가용 비상제동의 75%를 넘으면 즉시 최종 safety brake를 요청한다.
bool leadObjectRequiresEmergency(const WorldState& world,
                                 const LocalPathData& selectedPath,
                                 const PlanningBehaviorConfig& config,
                                 int& riskObjectId,
                                 bool includeStationary = false) {
    if (!selectedPath.valid || selectedPath.points.empty()) return false;
    const double egoSpeed = std::max(0.0, world.egoSpeed);
    for (const auto& object : world.objects) {
        if (!includeStationary && std::abs(object.raw.speed) < 0.3) {
            continue;
        }

        const Waypoint* nearest = nullptr;
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (const Waypoint& point : selectedPath.points) {
            const double distance = std::hypot(
                point.x - static_cast<double>(object.raw.x),
                point.y - static_cast<double>(object.raw.y));
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = &point;
            }
        }
        const double pathHalfWidth =
            0.5 * std::max(3.0, world.egoLaneWidth) +
            0.5 * std::max(0.0F, object.raw.width);
        const double relativeS = nearest != nullptr
            ? nearest->s - world.egoPathS
            : -std::numeric_limits<double>::infinity();
        if (nearest == nullptr || relativeS <= 0.0 ||
            nearestDistance > pathHalfWidth) {
            continue;
        }
        const double leadSpeed = std::max(
            0.0, static_cast<double>(object.raw.speed) *
                     std::cos(static_cast<double>(object.raw.heading) -
                              static_cast<double>(world.ego.heading)));
        const double speedEnergy =
            egoSpeed * egoSpeed - leadSpeed * leadSpeed;
        if (speedEnergy <= 0.0) continue;
        const double usableDistance =
            relativeS - config.minimumFollowDistance -
            egoSpeed * config.emergencyReactionTime;
        if (usableDistance <= 0.0 ||
            speedEnergy / (2.0 * usableDistance) >=
                0.75 * config.emergencyBrakingDeceleration) {
            riskObjectId = static_cast<int>(object.raw.id);
            return true;
        }
    }
    return false;
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
                        double dt,
                        DecisionTraceData* trace) {
    data.stopManager.requests.clear();
    const StopLine* line =
        findActiveStopLine(world, zones, data.config.stopLineBehindWindow,
                           data.config.signalStopSearchDistance);
    const int signalState = static_cast<int>(world.trafficLight.state);
    if (signalState != 2) data.stopManager.yellow = {};
    if (signalState != 6 || line == nullptr) data.stopManager.hold = {};

    if (!data.config.ignoreTrafficLights && line != nullptr) {
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
        if (!crosswalk.routeRelevant) continue;
        // 앞범퍼가 진입한 뒤에도 후륜축이 횡단보도를 완전히 빠져나갈 때까지
        // 점유 검사를 유지한다. 진입점을 넘었다는 이유로 보행자 정지를
        // 해제하면 횡단 중인 보행자 앞으로 다시 출발할 수 있다.
        if (crosswalk.exitRouteS < world.egoPathS) continue;
        const CrosswalkOccupancyState* state =
            findCrosswalkState(data, crosswalk.id);
        const bool stopForOccupancy =
            state != nullptr &&
            (state->occupied || state->clearanceHoldRemaining > 1e-6);
        if (stopForOccupancy) {
            // 횡단보도 위에 1 km/h 이하로 3초 이상 서 있으면 감점이다.
            // 횡단보도 입구를 목표로 삼으면 제동 오차만큼 밀려 횡단보도
            // 위에 서게 되므로, 그 앞 정지선이 있으면 정지선을 목표로 한다.
            // 여유는 정지 프로파일의 stop_safety_margin이 이미 적용한다.
            double target = crosswalk.entryRouteS;
            for (const auto& candidate : zones.stopLines) {
                if (!candidate.routeRelevant) continue;
                if (crosswalk.roadId >= 0 && candidate.roadId >= 0 &&
                    candidate.roadId != crosswalk.roadId) continue;
                if (!candidate.laneIds.empty() && world.egoLaneValid &&
                    std::find(candidate.laneIds.begin(),
                              candidate.laneIds.end(), world.egoLaneId) ==
                        candidate.laneIds.end()) continue;
                if (candidate.routeS > crosswalk.entryRouteS + 1e-3) continue;
                if (crosswalk.entryRouteS - candidate.routeS >
                    data.config.crosswalkStopLineSearch) continue;
                target = std::min(target, candidate.routeS);
            }
            addStopRequest(data.stopManager, StopReason::CrosswalkBlocked,
                           std::max(target, world.frontBumperPathS),
                           crosswalk.id);
            std::ostringstream detail;
            detail << "crosswalk=" << crosswalk.id
                   << " road=" << crosswalk.roadId
                   << " entry_route_s=" << crosswalk.entryRouteS
                   << " stop_route_s=" << target
                   << " front_bumper_path_s=" << world.frontBumperPathS
                   << " occupied=" << state->occupied
                   << " clearance_hold_remaining="
                   << state->clearanceHoldRemaining;
            traceDecision(trace, "crosswalk_stop", "stop_request_created",
                          detail.str());
        }
    }

    // 정지 목표가 횡단보도 안이면, 아직 진입하지 않은 경우에 한해 앞범퍼
    // 목표를 횡단보도 입구로 당긴다. 이미 진입했다면 안전 정지를 우선한다.
    for (auto& request : data.stopManager.requests) {
        for (const auto& crosswalk : zones.crosswalks) {
            if (!crosswalk.routeRelevant) continue;
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

bool evaluateLaneChangeCandidate(PathSafetyResult& safety,
                                 const LaneChangeData& laneChange,
                                 const WorldState& world,
                                 const PlanningBehaviorConfig& config,
                                 double minimumPredictionSpeed) {
    LocalPathData candidate;
    makeLaneChangeLocalPath(candidate, laneChange);
    resetPathSafety(safety);
    if (!candidate.valid) return false;

    // KeepTarget 경로에는 현재 횡이동 뒤의 successor/connector tail도
    // 포함된다. 그 tail 끝의 다음 장애물이 안전한 첫 회피까지 매 프레임
    // 취소하지 못하도록, 안전 재검사는 현재 횡이동 완료점 뒤 2 m까지만
    // 수행한다. committed path는 자르지 않으며 다음 장애물은 안착 직전의
    // 원자적 재계획이 담당한다.
    if (laneChange.routeMode == LaneChangeRouteMode::KeepTargetLane &&
        candidate.points.size() >= 3) {
        const auto end = std::upper_bound(
            candidate.points.begin(), candidate.points.end(),
            laneChange.rejoinRouteS + 2.0,
            [](double value, const Waypoint& point) {
                return value < point.s;
            });
        if (end != candidate.points.end()) {
            const auto keepEnd = std::min(candidate.points.end(), end + 1);
            if (keepEnd >= candidate.points.begin() + 3) {
                candidate.points.erase(keepEnd, candidate.points.end());
            }
        }
    }
    WorldState predictionWorld = world;
    predictionWorld.egoSpeed = std::max(
        {1.0, std::abs(world.egoSpeed), minimumPredictionSpeed});
    const double remainingLength =
        std::max(0.0, candidate.points.back().s - world.egoPathS);
    const double fullManeuverHorizon =
        remainingLength / predictionWorld.egoSpeed +
        config.collisionPredictionStep;
    checkCurrentCollisions(safety, world.egoObb, world.objects);
    if (safety.collision) return false;

    // 등속 예측을 수십 초까지 늘리면 움직이는 차량이 먼 미래에도 같은
    // 속도·방향을 유지한다고 가정해 안전한 회피를 과도하게 막는다. 동적
    // 객체는 설정된 근거리 horizon만, 실제로 경로를 막는 정적 객체는 남은
    // 기동 전체를 검사한다.
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
    if (safety.predictedCollision) return false;

    WorldState staticWorld = predictionWorld;
    staticWorld.objects.erase(
        std::remove_if(staticWorld.objects.begin(),
                       staticWorld.objects.end(),
                       [](const WorldObjectState& object) {
                           return std::abs(object.raw.speed) >= 0.3;
                       }),
        staticWorld.objects.end());
    checkPredictedCollisions(safety, staticWorld, candidate,
                             std::max(config.collisionPredictionHorizon,
                                      fullManeuverHorizon),
                             config.collisionPredictionStep, true);
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
    } else if (world.egoPathValid &&
               !data.pathPlanner.waypoints.empty()) {
        // 도로 안 자유 차선 주행에서는 최종 checkpoint와 같은 종방향
        // 위치에 도착해도 옆 차선 중심이라 좌표 거리가 차선 폭만큼
        // 남는다. 마지막 route_s에 도달했으면 목적지를 통과한 것으로
        // 판정해, 끝점을 지나친 뒤 recovery 정지에 빠지지 않게 한다.
        crossed = world.egoPathS >=
            data.pathPlanner.waypoints.back().s -
                data.pathPlanner.config.goalTolerance;
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

// 실제로 추종할 경로가 확정된 뒤 그 경로의 곡률속도를 기본 제한으로 삼는다.
// 회피 중 Ego는 원래 전역경로에서 수 m 떨어지므로, 전역경로의 route-recovery
// 속도를 여기 사용하면 안전한 목표 차선에 안착한 뒤에도 약 7 km/h로 묶인다.
bool updateSpeedConstraints(PlanningProcessData& data,
                            const WorldState& world,
                            const StaticZoneData& zones,
                            double selectedPathSpeed,
                            bool& schoolZoneActive) {
    data.frame.constraints.speedLimit = findZoneSpeedLimit(
        zones, {world.ego.x, world.ego.y},
        world.egoPathValid ? world.egoPathS : -1.0,
        selectedPathSpeed);
    schoolZoneActive = isSchoolZoneActive(
        zones, {world.ego.x, world.ego.y},
        world.egoPathValid ? world.egoPathS : -1.0);

    if (world.egoPathValid) {
        for (const auto& zone : zones.speedZones) {
            if (!zone.routeRelevant || !zone.schoolZone ||
                !zone.hasRouteRange ||
                zone.routeSEnd < world.frontBumperPathS) {
                continue;
            }
            const double distanceToEntry = std::max(
                0.0, zone.routeSBegin - world.frontBumperPathS);
            const double approachLimit = std::sqrt(
                data.pathPlanner.config.schoolZoneSpeed *
                    data.pathPlanner.config.schoolZoneSpeed +
                2.0 * data.config.comfortableDeceleration *
                    distanceToEntry);
            data.frame.constraints.speedLimit = std::min(
                data.frame.constraints.speedLimit, approachLimit);
        }
    }
    if (schoolZoneActive) {
        data.frame.constraints.speedLimit = std::min(
            data.frame.constraints.speedLimit,
            data.pathPlanner.config.schoolZoneSpeed);
    }

    bool crosswalkControlActive = false;
    for (const auto& crosswalk : zones.crosswalks) {
        if (!crosswalk.routeRelevant) continue;
        if (crosswalk.exitRouteS < world.egoPathS) {
            continue;
        }
        const CrosswalkOccupancyState* state =
            findCrosswalkState(data, crosswalk.id);
        const bool risk = state != nullptr && state->occupied;
        const bool previouslyActive = state != nullptr;
        const bool nearby = crosswalkHasNearbyObject(
            crosswalk, world, data.config.crosswalkNearbyObjectDistance);
        if (!risk && !nearby && !previouslyActive) {
            continue;
        }
        const double distance = crosswalk.entryRouteS - world.frontBumperPathS;
        if ((risk || nearby) &&
            distance > data.config.crosswalkApproachDistance) {
            continue;
        }
        const double limit = (risk || nearby)
                                 ? data.config.crosswalkApproachSpeed
                                 : data.config.crosswalkClearanceSpeed;
        crosswalkControlActive = true;
        data.frame.constraints.speedLimit =
            data.frame.constraints.speedLimit > 0.0
                ? std::min(data.frame.constraints.speedLimit, limit)
                : limit;
    }
    return crosswalkControlActive;
}

}  // namespace

bool initializePlanning(PlanningProcessData& data,
                        const PathPlannerConfig& plannerConfig,
                        const PlanningBehaviorConfig& behaviorConfig,
                        const LaneChangeConfig& laneChangeConfig,
                        const std::string& referencePathCsv) {
    data = {};
    data.pathPlanner.config = plannerConfig;
    data.roadLanePlanner.config = plannerConfig;
    data.config = behaviorConfig;
    data.laneChangeConfig = laneChangeConfig;
    return loadWaypointsCsv(data.pathPlanner.waypoints, referencePathCsv);
}

void runPlanningProcess(PlanningProcessData& data,
                        const GtFrame& gt,
                        const WorldState& sensedWorld,
                        const PathGeneratorData& map,
                        double dt,
                        DecisionTraceData* trace) {
    resetPlanningFrame(data);
    updateMissionCorridorProgress(data.missionCorridor, sensedWorld);

    // sensedWorld의 path 좌표는 traffic-rule annotation용 guide route다.
    // checkpoint mission의 주행/장애물 판단은 아래에서 실제 추종할
    // lane-center 또는 보관된 maneuver path로 다시 투영한 복사본을 쓴다.
    WorldState world = sensedWorld;
    const bool missionActive =
        activeMissionLeg(data.missionCorridor) != nullptr;

    PathPlan globalTracking;
    std::size_t nearestGlobalIndex = data.pathPlanner.nearestIndex;
    if (!missionActive) {
        globalTracking = runPurePursuit(
            data.pathPlanner, gt, sensedWorld.egoSpeed);
        nearestGlobalIndex = globalTracking.nearestIndex;
    }
    // 새 맵매칭 section으로 경로를 만들다가 경계 한 프레임에
    // 실패해도 이전 정상 경로를 먼저 지우지 않는다. 임시 객체에 성공한
    // 경로만 만든 뒤, Ego 위치·헤딩·lane successor·잔여거리 검사를
    // 통과했을 때 원자적으로 교체한다.
    PathPlannerData roadLaneCandidate;
    const bool roadLaneCandidateBuilt = buildRoadIdTrackingPath(
        roadLaneCandidate, data.pathPlanner.config, world, map,
        data.pathPlanner, nearestGlobalIndex, data.config.localPathHorizon,
        trace, &data.missionCorridor);
    double candidateLateralDistance{};
    double candidateRemainingDistance{};
    const bool roadLaneCandidateUsable = roadLaneCandidateBuilt &&
        trackingPathUsable(roadLaneCandidate, sensedWorld, map,
                           candidateLateralDistance,
                           candidateRemainingDistance);
    bool retainedRoadLanePath = false;
    bool shortTailContinuityPath = false;
    if (roadLaneCandidateUsable) {
        data.roadLanePlanner = std::move(roadLaneCandidate);
    } else {
        double retainedLateralDistance{};
        double retainedRemainingDistance{};
        retainedRoadLanePath = trackingPathUsable(
            data.roadLanePlanner, sensedWorld, map,
            retainedLateralDistance, retainedRemainingDistance);
        if (retainedRoadLanePath) {
            std::ostringstream detail;
            detail << "reason="
                   << (roadLaneCandidateBuilt
                           ? "new_path_failed_pose_or_topology_validation"
                           : "new_path_build_failed")
                   << " lateral_distance=" << retainedLateralDistance
                   << " remaining_distance=" << retainedRemainingDistance
                   << " sensed_road=" << sensedWorld.egoRoadId
                   << " sensed_section=" << sensedWorld.egoSectionIndex
                   << " sensed_lane=" << sensedWorld.egoLaneId;
            traceDecision(trace, "road_corridor",
                          "previous_tracking_path_retained", detail.str());
        } else {
            std::ostringstream detail;
            detail << "candidate_built=" << roadLaneCandidateBuilt
                   << " candidate_lateral=" << candidateLateralDistance
                   << " candidate_remaining=" << candidateRemainingDistance
                   << " retained_lateral=" << retainedLateralDistance
                   << " retained_remaining=" << retainedRemainingDistance
                   << " sensed_input_valid=" << sensedWorld.inputValid
                   << " sensed_lane_valid=" << sensedWorld.egoLaneValid
                   << " sensed_road=" << sensedWorld.egoRoadId
                   << " sensed_section=" << sensedWorld.egoSectionIndex
                   << " sensed_lane=" << sensedWorld.egoLaneId;
            traceDecision(trace, "road_corridor",
                          "tracking_path_unavailable", detail.str());
        }
    }
    if (!roadLaneCandidateUsable && !retainedRoadLanePath &&
        roadLaneCandidateBuilt) {
        double shortTailLateralDistance{};
        double shortTailRemainingDistance{};
        shortTailContinuityPath = trackingPathUsable(
            roadLaneCandidate, sensedWorld, map,
            shortTailLateralDistance, shortTailRemainingDistance, false);
        if (shortTailContinuityPath) {
            data.roadLanePlanner = std::move(roadLaneCandidate);
            std::ostringstream detail;
            detail << "remaining_distance=" << shortTailRemainingDistance
                   << " lateral_distance=" << shortTailLateralDistance
                   << " reason=no_full_horizon_replacement_available";
            traceDecision(trace, "road_corridor",
                          "short_tail_continuity_path_retained",
                          detail.str());
        }
    }
    const bool roadLanePathValid = roadLaneCandidateUsable ||
                                   retainedRoadLanePath ||
                                   shortTailContinuityPath;
    PathPlan tracking;
    if (roadLanePathValid) {
        tracking = runPurePursuit(
            data.roadLanePlanner, gt, sensedWorld.egoSpeed);
    } else if (!missionActive) {
        tracking = globalTracking;
    }

    if (missionActive) {
        const std::vector<Waypoint>* activePath = nullptr;
        if (data.laneChange.phase != LaneChangePhase::None &&
            data.laneChange.path.size() >= 2) {
            activePath = &data.laneChange.path;
        } else if (roadLanePathValid) {
            activePath = &data.roadLanePlanner.waypoints;
        }
        const bool activePathValid = activePath != nullptr &&
            projectWorldToActivePath(
                world, *activePath, sensedWorld.frontBumperOffset);
        // guide route를 잃은 것은 정상일 수 있지만, 지도 lane이나 실제
        // 추종경로까지 잃으면 안전한 조향/충돌예측이 불가능하다.
        world.inputValid = sensedWorld.inputValid && activePathValid;
        if (!world.inputValid) {
            std::ostringstream detail;
            detail << "sensed_input_valid=" << sensedWorld.inputValid
                   << " active_path_present=" << (activePath != nullptr)
                   << " active_path_projected=" << activePathValid
                   << " road_lane_path_valid=" << roadLanePathValid
                   << " lane_change_phase="
                   << static_cast<int>(data.laneChange.phase);
            traceDecision(trace, "road_corridor",
                          "active_path_input_invalid", detail.str());
        }
    }
    if (roadLanePathValid && tracking.goalReached) {
        // 현재 차선 지역경로의 끝은 도로 연결/재계획 지점이다.
        // 전역경로의 최종 목적지가 아니므로 정지하지 않는다.
        tracking.goalReached = false;
        tracking.targetSpeed = chooseTargetSpeed(
            tracking.maxCurvature, data.pathPlanner.config);
    }
    // checkpoint mission에서는 이후 회피 생성/객체 rematch에도 dense
    // guide route를 전달하지 않는다. 현재 corridor lane chain을 유일한
    // 주행 기준경로로 사용한다.
    PathPlannerData committedManeuverPlanner;
    const bool committedManeuverActive =
        missionActive &&
        data.laneChange.phase != LaneChangePhase::None &&
        data.laneChange.path.size() >= 2;
    if (committedManeuverActive) {
        committedManeuverPlanner.config = data.pathPlanner.config;
        committedManeuverPlanner.waypoints = data.laneChange.path;
        committedManeuverPlanner.nearestIndex =
            data.laneChange.nearestIndex;
    }
    // 이미 확정한 기동 중에는 객체 재투영과 후속 회피 생성도 같은
    // committed path를 기준으로 수행한다. 일반 차선 경로를 다시
    // 참조하면 객체의 전후/차선 판정이 바뀌어 후보가 프레임마다
    // 흔들릴 수 있다.
    const PathPlannerData& maneuverReference = committedManeuverActive
        ? committedManeuverPlanner
        : (missionActive && roadLanePathValid
               ? data.roadLanePlanner : data.pathPlanner);
    const std::size_t nearestManeuverIndex = committedManeuverActive
        ? data.laneChange.nearestIndex
        : (missionActive && roadLanePathValid
               ? tracking.nearestIndex : nearestGlobalIndex);
    data.frame.constraints.mapMatchValid = world.inputValid;
    bool schoolZoneActive = false;
    bool crosswalkControlActive = false;
    const std::vector<int> confirmedStationaryObjectIds =
        updateConfirmedStationaryObjects(data, world, dt);

    // 횡단보도 점유는 여러 속도/정지 판단에서 공유한다. 한 프레임에
    // 한 번만 갱신해 점유 해제 타이머가 중복으로 감소하지 않게 한다.
    updateCrosswalkStates(data, sensedWorld, map.staticZones, dt, trace);

    // 신호/횡단보도 정지 여부를 먼저 판정한다. 빨간불 전에 만든
    // 회피 경로를 정지한 동안 보관했다가 녹색불에서 중간부터
    // 추종하면 정지선에서 바로 크게 조향한다.
    updateStopRequests(data, sensedWorld, map.staticZones, dt, trace);

    double pendingBlockerDistance = -1.0;
    // Gateway 정렬은 장애물 회피보다 우선할 수 없다. 이미
    // committed Gateway path를 추종 중이더라도 차선 중심에서
    // 확인된 정적 장애물의 안전한 회피 후보가 생기면 기존
    // 경로를 원자적으로 교체한다. 횟이동 중간에서 갈아 끼우면
    // 급조향이 될 수 있으므로 안착된 시점에서만 허용한다.
    if (data.laneChange.phase != LaneChangePhase::None &&
        data.laneChange.roadConnectionManeuver &&
        world.egoLaneCenterDistance <= 0.75) {
        LaneChangeData obstacleReplacement;
        double replacementBlockerDistance = -1.0;
        const bool replacementReady = tryPrepareObstacleLaneChange(
            obstacleReplacement, data.laneChangeConfig, data.config,
            world, map, maneuverReference, nearestManeuverIndex,
            &replacementBlockerDistance,
            &confirmedStationaryObjectIds, trace,
            &data.missionCorridor);
        if (replacementReady) {
            traceDecision(trace, "road_corridor",
                          "gateway_preempted_by_static_obstacle",
                          "safe_obstacle_path_committed");
            data.laneChange = std::move(obstacleReplacement);
            pendingBlockerDistance = replacementBlockerDistance;
        }
    }
    // Gateway route에 서로 다른 section의 횡이동이 연속하면
    // 이전 단계의 settle 종료를 기다리는 동안 다음 점선 구간을
    // 지나칠 수 있다. 이전 기동은 Gateway 정렬뿐 아니라 장애물
    // 회피일 수도 있다. 기동의 종류와 무관하게 현재 목표 차선
    // 중심에 안착했으면 다음 Gateway 단계를 즉시 계획해
    // committed path를 원자적으로 이어 간다. 이 조건은 road/lane
    // ID를 특정하지 않고 모든 corridor leg에 동일하게 적용된다.
    if (data.laneChange.phase != LaneChangePhase::None &&
        data.laneChange.shiftCompletedLaneValid &&
        world.egoLaneValid &&
        world.egoRoadId == data.laneChange.shiftCompletedLane.roadId &&
        world.egoSectionIndex ==
            data.laneChange.shiftCompletedLane.sectionIndex &&
        world.egoLaneId == data.laneChange.shiftCompletedLane.laneId &&
        world.egoLaneCenterDistance <= 0.75) {
        LaneChangeData chainedGateway;
        const bool gatewayReady = tryPrepareRoadConnectionLaneChange(
                chainedGateway, data.laneChangeConfig, data.config,
                world, map, maneuverReference, nearestManeuverIndex,
                trace, &data.missionCorridor);
        // Gateway 후보 자체가 전체 기동과 settling tail의 장애물
        // 여유를 통과해야 한다. 따라서 회피 기동 도중이라도 목표
        // 차선 중심에 안착했고 안전한 Gateway 단계가 있으면 방향과
        // 무관하게 바로 연결한다. 막힌 차선으로의 복귀는 후보 안전
        // 검사에서 탈락하므로 별도의 방향 휴리스틱을 두지 않는다.
        if (gatewayReady) {
            traceDecision(trace, "road_corridor",
                          "gateway_stage_chained",
                          "next_lateral_stage_committed_before_settle");
            data.laneChange = std::move(chainedGateway);
        }
    }
    // 보관한 회피 경로가 실행 중 새 정적 장애물에 막혀도 기존 경로부터
    // 비우지 않는다. 빈 상태로 만든 뒤 후보 탐색에 실패하면 프레임마다
    // 좌우 후보가 뒤집히고 결국 추종할 지역경로까지 사라진다. 현재 차가
    // 차로 중심에 안착한 경우에만 별도 상태에서 새 후보를 만들고, 안전한
    // 후보 생성이 성공했을 때 원자적으로 교체한다.
    if (data.laneChange.phase == LaneChangePhase::Execute &&
        !data.laneChange.roadConnectionManeuver) {
        PathSafetyResult activeSafety;
        const bool activePathSafe = evaluateLaneChangeCandidate(
            activeSafety, data.laneChange, world, data.config,
            data.laneChangeConfig.maximumLaneChangeSpeed);
        const bool staticRisk = activeSafety.riskObjectId >= 0 &&
            std::any_of(world.objects.begin(), world.objects.end(),
                        [&](const WorldObjectState& object) {
                            return static_cast<int>(object.raw.id) ==
                                       activeSafety.riskObjectId &&
                                   std::abs(object.raw.speed) < 0.3;
                        });
        // 후보 생성 시 안착 차선의 종방향 연결에서 확인한 다음 장애물은
        // 현재 경로 종점 뒤에 있어 activePathSafe만으로는 잡히지 않는다.
        // 목표 차선 중심에 먼저 안착한 순간 다음 회피를 준비해, 첫 기동의
        // rejoinRouteS를 모두 지난 뒤에야 재계획하던 한 프레임/한 기동
        // 지연을 없앤다. 2~3단계를 포함한 연속 장애물도 각 안전한 차선
        // 중심 구간에서 경로를 원자적으로 계속 이어 간다.
        const double followupDistance =
            data.laneChange.followupObstacleClearance >= 0.0
                ? std::max(0.0, data.laneChange.rejoinRouteS - world.egoPathS) +
                      data.laneChange.followupObstacleClearance
                : std::numeric_limits<double>::infinity();
        const bool anticipatedFollowupReady =
            data.laneChange.followupObstacleId >= 0 &&
            data.laneChange.shiftCompletedLaneValid &&
            world.egoLaneValid &&
            world.egoRoadId == data.laneChange.shiftCompletedLane.roadId &&
            world.egoSectionIndex ==
                data.laneChange.shiftCompletedLane.sectionIndex &&
            world.egoLaneId == data.laneChange.shiftCompletedLane.laneId &&
            world.egoLaneCenterDistance <= 0.75 &&
            followupDistance <=
                data.laneChangeConfig.obstacleTriggerMaximumDistance &&
            std::any_of(world.objects.begin(), world.objects.end(),
                        [&](const WorldObjectState& object) {
                            return static_cast<int>(object.raw.id) ==
                                       data.laneChange.followupObstacleId &&
                                   std::abs(object.raw.speed) < 0.3 &&
                                   object.isAhead &&
                                   object.relativeS >=
                                       data.laneChangeConfig
                                           .obstacleTriggerMinimumDistance &&
                                   object.relativeS <=
                                       data.laneChangeConfig
                                           .obstacleTriggerMaximumDistance;
                        });
        constexpr double kMinimumStaticReplanTtc = 1.8;
        const bool unsafeActivePathReplan =
            !activePathSafe && !activeSafety.collision && staticRisk &&
            activeSafety.predictedCollision &&
            activeSafety.predictedCollisionTTC >= kMinimumStaticReplanTtc &&
            world.egoLaneCenterDistance <= 0.75;
        if (anticipatedFollowupReady || unsafeActivePathReplan) {
            std::ostringstream detail;
            detail << "reason="
                   << (anticipatedFollowupReady
                           ? "anticipated_followup"
                           : "active_path_became_unsafe")
                   << " followup_object="
                   << data.laneChange.followupObstacleId
                   << " risk_object=" << activeSafety.riskObjectId
                   << " followup_distance=" << followupDistance
                   << " predicted_ttc="
                   << activeSafety.predictedCollisionTTC;
            LaneChangeData replacement;
            double replacementBlockerDistance = -1.0;
            const bool replacementReady = tryPrepareObstacleLaneChange(
                replacement, data.laneChangeConfig, data.config, world, map,
                maneuverReference, nearestManeuverIndex,
                &replacementBlockerDistance,
                &confirmedStationaryObjectIds, trace,
                &data.missionCorridor);
            detail << " replacement_ready=" << replacementReady;
            traceDecision(trace, "lane_change",
                          replacementReady
                              ? "active_path_replaced"
                              : "active_path_retained",
                          detail.str());
            if (replacementReady) {
                data.laneChange = std::move(replacement);
                pendingBlockerDistance = replacementBlockerDistance;
            }
        }
    }
    // 먼 신호 정지 요청보다 현재 도로의 장애물 회피를 먼저 계획한다.
    // 아래에서 경로 종점이 정지선을 침범할 때만 실행을 막는다.
    tryPrepareObstacleLaneChange(
        data.laneChange, data.laneChangeConfig, data.config, world, map,
        maneuverReference, nearestManeuverIndex, &pendingBlockerDistance,
        &confirmedStationaryObjectIds, trace, &data.missionCorridor);
    // 아직 확정 기동이 없을 때는 눈앞 장애물 후보를 먼저 확정한다.
    // 장애물 기동 중 안전하게 차선 중심에 안착하면 위의 chaining이
    // gateway 정렬을 동시에 이어 갈 수 있다. 다만 장애물을 찾았는데
    // 안전 후보가 없는 프레임에는 gateway가 그 실패를 덮지 않는다.
    if (data.laneChange.phase == LaneChangePhase::None &&
        pendingBlockerDistance < 0.0) {
        tryPrepareRoadConnectionLaneChange(
            data.laneChange, data.laneChangeConfig, data.config, world,
            map, maneuverReference, nearestManeuverIndex, trace,
            &data.missionCorridor);
    }
    // gateway target/path는 한 번 선택하면 정지나 감속 중에도 고정한다.
    // Pure Pursuit가 실행 시점의 최근접점을 다시 찾으므로 prepare 동안
    // 프레임마다 경로를 재생성할 필요가 없고, 재생성은 오히려 분기 선택을
    // 흔들거나 순간적인 실선 판정으로 정상 경로를 잃게 만든다.
    bool laneChangeExecutionBlocked =
        data.laneChange.phase == LaneChangePhase::None
            ? hasLaneChangeExecutionBlocker(data.stopManager)
            : hasLaneChangeExecutionBlocker(
                  data.stopManager, data.laneChange.rejoinRouteS);

    PathSafetyResult maneuverSpeedSafety;
    const bool candidateSafeAtManeuverSpeed =
        data.laneChange.phase == LaneChangePhase::None ||
        evaluateLaneChangeCandidate(
            maneuverSpeedSafety, data.laneChange, world, data.config,
            data.laneChangeConfig.maximumLaneChangeSpeed);
    if (data.laneChange.phase != LaneChangePhase::None && trace != nullptr &&
        trace->enabled) {
        std::ostringstream detail;
        detail << "phase=" << static_cast<int>(data.laneChange.phase)
               << " route_mode=" << static_cast<int>(data.laneChange.routeMode)
               << " safe=" << candidateSafeAtManeuverSpeed
               << " collision=" << maneuverSpeedSafety.collision
               << " predicted_collision=" << maneuverSpeedSafety.predictedCollision
               << " predicted_ttc=" << maneuverSpeedSafety.predictedCollisionTTC
               << " risk_object=" << maneuverSpeedSafety.riskObjectId;
        traceDecision(trace, "lane_change", "maneuver_speed_safety_checked",
                      detail.str());
    }

    const bool maneuverObstacleStillConfirmed =
        data.laneChange.phase == LaneChangePhase::None ||
        data.laneChange.roadConnectionManeuver ||
        std::any_of(world.objects.begin(), world.objects.end(),
                    [&](const WorldObjectState& object) {
                        return static_cast<int>(object.raw.id) ==
                                   data.laneChange.obstacleId &&
                               std::abs(object.raw.speed) < 0.3;
                    });
    advanceLaneChangePhase(data.laneChange, data.laneChangeConfig, world, dt,
                           laneChangeExecutionBlocked,
                           maneuverObstacleStillConfirmed,
                           candidateSafeAtManeuverSpeed,
                           trace, roadLanePathValid);

    // 기동에 진입한 뒤에는 정적 물체의 종방향 판정을 전역경로
    // isOnEgoPath로 다시 하지 않는다. 복귀 구간이 멀리서 잠시 막혀도
    // 현재 회피 차선은 안전할 수 있다. 실제 선택 경로의 정적 충돌은
    // 아래 OBB 예측이 계속 검사한다.
    const bool safelyAvoidingObstacle =
        data.laneChange.phase != LaneChangePhase::None &&
        !data.laneChange.roadConnectionManeuver;
    selectStopDecision(data.frame.stop, data.stopManager,
                       sensedWorld.frontBumperPathS);

    // LaneChangeData::path가 선택된 순간부터 기동과 settle이
    // 끝날 때까지 유일한 committed trajectory다. Prepare에서
    // 일반 차선 경로로 되돌아갔다 Execute에서 다시 곡선으로
    // 바꾸거나, Settle에서 짧은 lane tail로 되돌아가면 급조향과
    // 경로 공백이 생긴다. 후보 경로에는 prepare 시간 동안
    // 직진할 prefix가 이미 포함되어 있으므로 전 상태에서
    // 같은 경로를 지속 추종한다.
    if (data.laneChange.phase != LaneChangePhase::None) {
        tracking = trackLaneChangePath(
            data.laneChange, data.pathPlanner.config, gt, world.egoSpeed);
        makeLaneChangeLocalPath(data.frame.localPath, data.laneChange);
    } else {
        const PathPlannerData& selectedPlanner = roadLanePathValid
            ? data.roadLanePlanner : data.pathPlanner;
        generateLocalPath(data.frame.localPath, selectedPlanner,
                          tracking.nearestIndex,
                          data.config.localPathHorizon);
    }
    constexpr double kSharpCurveReleaseHoldTime = 4.0;
    if (tracking.valid &&
        tracking.maxCurvature >= data.pathPlanner.config.sharpCurveThreshold) {
        data.sharpCurveSpeedHoldRemaining = kSharpCurveReleaseHoldTime;
    } else {
        data.sharpCurveSpeedHoldRemaining = std::max(
            0.0, data.sharpCurveSpeedHoldRemaining - std::max(0.0, dt));
    }
    if (tracking.valid && data.sharpCurveSpeedHoldRemaining > 0.0) {
        tracking.targetSpeed = std::min(
            tracking.targetSpeed, data.pathPlanner.config.sharpCurveSpeed);
    }
    // 전역경로 또는 회피 지역경로 중 이번 프레임에 실제 선택된 경로를 기준으로
    // 곡률/구역 제한속도를 계산한다.
    crosswalkControlActive = updateSpeedConstraints(
        data, sensedWorld, map.staticZones, tracking.targetSpeed,
        schoolZoneActive);
    if (!world.inputValid || world.respawnDetected) {
        data.behaviorState.health = SystemHealth::Recovery;
    }
    data.behaviorState.staticObstacleAvoidanceActive =
        safelyAvoidingObstacle;
    data.behaviorState.safelyAvoidedObjectId = safelyAvoidingObstacle
        ? data.laneChange.obstacleId
        : -1;
    updateBehavior(data.behaviorState, data.frame.behavior,
                   data.frame.constraints, data.frame.stop, world,
                   tracking.targetSpeed, data.config, dt);
    data.behaviorState.laneChange = data.laneChange.phase;
    data.frame.behavior.laneChange = data.laneChange.phase;
    data.frame.behavior.turnSignal = data.laneChange.turnSignal;

    // Gateway 횡이동 edge가 다음 section에 있으면 현재 section에서는
    // 아직 차선변경 후보를 만들 수 없다. 이 때 일반 주행속도를
    // 유지하면 lateral edge가 실제로 열린 순간에는 이미 너무
    // 빨라 정지선 앞에서 경로가 뒤로 밀린다. 전역경로가 아니라
    // 미리 계산된 mission lane graph의 첫 lateral edge까지의 실제
    // local-path 잔여거리를 사용해, edge 진입 전에 횡이동 속도로
    // 감속을 완료한다.
    if (missionActive && roadLanePathValid &&
        data.laneChange.phase == LaneChangePhase::None &&
        world.egoLaneValid) {
        const LaneKey currentLane{
            world.egoRoadId, world.egoSectionIndex, world.egoLaneId};
        std::vector<LaneKey> routeToGateway;
        if (findMissionLaneRoute(
                map, data.missionCorridor, currentLane, routeToGateway) &&
            std::adjacent_find(
                routeToGateway.begin(), routeToGateway.end(),
                [](const LaneKey& from, const LaneKey& to) {
                    return from.roadId == to.roadId &&
                           from.sectionIndex == to.sectionIndex &&
                           from.laneId != to.laneId;
                }) != routeToGateway.end()) {
            const double distanceToGatewayAction = std::max(
                0.0, data.roadLanePlanner.waypoints.back().s -
                         world.egoPathS);
            if (distanceToGatewayAction <=
                data.laneChangeConfig.roadConnectionPrepareDistance) {
                const double before = data.frame.behavior.targetSpeed;
                data.frame.behavior.targetSpeed = std::min(
                    before, data.laneChangeConfig.maximumLaneChangeSpeed);
                if (before > data.frame.behavior.targetSpeed + 1e-6) {
                    std::ostringstream detail;
                    detail << "distance_to_lateral_edge="
                           << distanceToGatewayAction
                           << " speed_cap="
                           << data.laneChangeConfig.maximumLaneChangeSpeed;
                    traceDecision(trace, "road_corridor",
                                  "gateway_approach_speed_limited",
                                  detail.str());
                }
            }
        }
    }

    // 확인된 정지 장애물이 있는데도 아직 안전한 차선변경 후보를 못 찾았으면
    // (신호 대기, 도로표시 제약, 옆 차선 간격 부족 등으로 이번 프레임엔 회피가
    // 안 되는 경우) 일반 Follow 감속만으로는 장애물에 도달했을 때 옆으로
    // 옮길 거리(전환거리+통과여유)가 남아있는다는 보장이 없다. 회피가 실제로
    // 가능해지는 순간 곧바로 완주할 수 있도록, 장애물까지 그 거리가 남을
    // 때 회피 기동 속도(maximumLaneChangeSpeed)에 도달하게 미리 감속한다.
    if (data.laneChange.phase == LaneChangePhase::None &&
        pendingBlockerDistance >= 0.0) {
        const double reserveDistance = data.laneChangeConfig.transitionLength +
                                       data.laneChangeConfig.obstacleClearance;
        const double distanceToSlow =
            std::max(0.0, pendingBlockerDistance - reserveDistance);
        const double maneuverSpeed = data.laneChangeConfig.maximumLaneChangeSpeed;
        const double approachCap = std::sqrt(
            maneuverSpeed * maneuverSpeed +
            2.0 * data.config.comfortableDeceleration * distanceToSlow);
        data.frame.behavior.targetSpeed =
            std::min(data.frame.behavior.targetSpeed, approachCap);
    }

    // transitionLength는 곡선 길이일 뿐, 실제 안착 route_s가 아니다.
    // 지연 합류·연결 road에서는 안착점이 더 멀 수 있으므로 기동 상태가
    // 실제로 끝날 때까지 속도 상한을 유지한다.
    const bool lateralShiftActive =
        data.laneChange.phase != LaneChangePhase::None;
    if (lateralShiftActive) {
        data.frame.behavior.targetSpeed = std::min(
            data.frame.behavior.targetSpeed,
            data.laneChangeConfig.maximumLaneChangeSpeed);
    }
    if (data.laneChange.phase == LaneChangePhase::Execute) {
        if (data.laneChange.waitingForReturn) {
            data.frame.behavior.targetSpeed = std::min(
                data.frame.behavior.targetSpeed,
                data.laneChangeConfig.returnWaitingSpeed);
        }
    }

    // 미래 충돌검사가 선택된 지역경로를 사용한다.
    resetPathSafety(data.frame.pathSafety);
    checkCurrentCollisions(data.frame.pathSafety, world.egoObb,
                           world.objects);
    updateMinimumTTC(data.frame.pathSafety, world.objects,
                     safelyAvoidingObstacle,
                     safelyAvoidingObstacle
                         ? data.laneChange.obstacleId
                         : -1);
    checkPredictedCollisions(data.frame.pathSafety, world,
                             data.frame.localPath,
                             data.config.collisionPredictionHorizon,
                             data.config.collisionPredictionStep);
    // 비상제동은 급정거·끼어들기 같은 동적 위험과 현재 실제
    // 충돌에만 적용한다. 앞에 멈춰 있는 정적 장애물의 미래
    // 충돌은 속도 0 명령으로 해결할 대상이 아니라 차선회피 대상이다.
    WorldState dynamicEmergencyWorld = world;
    dynamicEmergencyWorld.objects.erase(
        std::remove_if(dynamicEmergencyWorld.objects.begin(),
                       dynamicEmergencyWorld.objects.end(),
                       [](const WorldObjectState& object) {
                           return std::abs(object.raw.speed) < 0.3;
                       }),
        dynamicEmergencyWorld.objects.end());
    PathSafetyResult dynamicEmergencySafety;
    resetPathSafety(dynamicEmergencySafety);
    checkCurrentCollisions(dynamicEmergencySafety, world.egoObb,
                           dynamicEmergencyWorld.objects);
    updateMinimumTTC(dynamicEmergencySafety,
                     dynamicEmergencyWorld.objects, false);
    checkPredictedCollisions(dynamicEmergencySafety, dynamicEmergencyWorld,
                             data.frame.localPath,
                             data.config.collisionPredictionHorizon,
                             data.config.collisionPredictionStep);
    // 복귀 경로가 멀리서 막혀 있을 때는 검증된 저속 복귀를 하거나
    // 위의 대기 지점 속도로 감속한다. 충돌 예측이 있다는
    // 사실만으로 즉시 -4 m/s^2를 명령하지 않고, 현재 충돌이거나
    // TTC가 임계 이내로 들어온 실제 긴급 상황에서만 비상제동한다.
    const double speedAwareEmergencyTTC = std::max(
        data.config.emergencyTTC,
        data.config.emergencyReactionTime +
            data.config.collisionPredictionStep +
            std::max(0.0, world.egoSpeed) /
                data.config.emergencyBrakingDeceleration);
    int physicalRiskObjectId = -1;
    const bool crossingEmergency = crossingObjectRequiresEmergency(
        world, data.frame.localPath.points, data.config,
        physicalRiskObjectId);
    // 급제동 선행차는 차선변경 중에도 검사하되, 원래 전역경로의
    // isOnEgoPath가 아니라 이번 프레임에 실제 추종하는 지역경로를 쓴다.
    // 따라서 이미 옆으로 피한 정적 장애물은 비상제동을 만들지 않는다.
    // 정적 장애물은 정상적으로는 회피 대상이므로 미래 충돌만으로 멈추지
    // 않는다. 다만 blocker를 확인했는데 기하적으로 추종 가능한 후보가
    // 하나도 없는 상태에서는 충돌할 때까지 진행하지 않고, 실제 남은
    // 제동거리 안으로 들어왔을 때만 비상제동 안전망을 다시 허용한다.
    const bool staticAvoidanceUnavailable =
        data.laneChange.phase == LaneChangePhase::None &&
        pendingBlockerDistance >= 0.0;
    const bool leadEmergency = leadObjectRequiresEmergency(
        world, data.frame.localPath, data.config, physicalRiskObjectId,
        staticAvoidanceUnavailable);
    const bool rawEmergency = data.frame.pathSafety.collision ||
        requiresEmergencyBrake(
            dynamicEmergencySafety, speedAwareEmergencyTTC) ||
        crossingEmergency || leadEmergency;
    if (rawEmergency) {
        data.emergencyBrakeHoldRemaining =
            data.config.emergencyMinimumHoldTime;
    } else {
        data.emergencyBrakeHoldRemaining = std::max(
            0.0, data.emergencyBrakeHoldRemaining - std::max(0.0, dt));
    }
    const bool emergency = rawEmergency ||
        data.emergencyBrakeHoldRemaining > 0.0;
    if (trace != nullptr && trace->enabled) {
        std::ostringstream detail;
        detail << "ego_road=" << world.egoRoadId
               << " ego_section=" << world.egoSectionIndex
               << " ego_lane=" << world.egoLaneId
               << " ego_path_s=" << world.egoPathS
               << " ego_path_d=" << world.egoPathD
               << " ego_speed=" << world.egoSpeed
               << " school_zone=" << schoolZoneActive
               << " crosswalk_control=" << crosswalkControlActive
               << " lane_change_phase="
               << static_cast<int>(data.laneChange.phase)
               << " target_lane=" << data.laneChange.targetLaneId
               << " maneuver_path_safe="
               << candidateSafeAtManeuverSpeed
               << " stop_valid=" << data.frame.stop.valid
               << " stop_distance=" << data.frame.stop.distanceToStop
               << " min_ttc=" << data.frame.pathSafety.minimumTTC
               << " collision=" << data.frame.pathSafety.collision
               << " predicted_collision="
               << data.frame.pathSafety.predictedCollision
               << " predicted_ttc="
               << data.frame.pathSafety.predictedCollisionTTC
               << " risk_object=" << data.frame.pathSafety.riskObjectId
               << " emergency=" << emergency
               << " raw_emergency=" << rawEmergency
               << " emergency_hold_remaining="
               << data.emergencyBrakeHoldRemaining
               << " emergency_ttc_threshold=" << speedAwareEmergencyTTC
               << " physical_risk_object=" << physicalRiskObjectId
               << " target_speed=" << data.frame.behavior.targetSpeed;
        traceDecision(trace, "planning", "frame_result", detail.str());
    }
    data.frame.behavior.emergencyBrake = emergency;
    if (emergency) data.frame.behavior.targetSpeed = 0.0;

    tracking.targetSpeed = data.frame.behavior.targetSpeed;
    const bool missionComplete = data.missionCorridor.valid &&
        activeMissionLeg(data.missionCorridor) == nullptr;
    tracking.goalReached = missionComplete || crossedFinishLine(
        data, sensedWorld, map.staticZones,
        !data.missionCorridor.valid && globalTracking.goalReached);
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
    data.result.currentSpeedLimit = data.frame.constraints.speedLimit;
    data.result.schoolZoneActive = schoolZoneActive;
    data.result.stopDistanceValid = data.frame.stop.valid;
    data.result.stopDistance = data.frame.stop.distanceToStop;
    data.result.crosswalkActive = crosswalkControlActive;
    data.result.laneChangeActive =
        data.laneChange.phase != LaneChangePhase::None;
}
