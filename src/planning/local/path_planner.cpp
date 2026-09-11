#include "planning/local/path_planner.hpp"

#include "utils/csv_utils.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>


// reference_path.csv의 x/y/yaw/s/road/lane 값을 Waypoint 구조체로 읽는다.
bool loadWaypointsCsv(std::vector<Waypoint>& waypoints, const std::string& csvPath) {
    std::ifstream input(csvPath);
    if (!input) return false;

    waypoints.clear();
    std::string line;
    std::getline(input, line);  // header

    while (std::getline(input, line)) {
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() < 7) return false;

        Waypoint point;
        double roadId = 0.0;
        double laneId = 0.0;
        if (!parseDouble(fields[0], point.x) ||
            !parseDouble(fields[1], point.y) ||
            !parseDouble(fields[2], point.yaw) ||
            !parseDouble(fields[3], point.s) ||
            !parseDouble(fields[4], roadId) ||
            !parseDouble(fields[5], laneId) ||
            !parseDouble(fields[6], point.roadS)) {
            return false;
        }
        point.roadId = static_cast<int>(roadId);
        point.laneId = static_cast<int>(laneId);
        waypoints.push_back(point);
    }
    return !waypoints.empty();
}

// Ego와 가장 가까운 전역경로 점의 index를 찾는다.
std::size_t findNearestWaypoint(const std::vector<Waypoint>& waypoints,
                           const GtFrame& gt,
                           std::size_t previousIndex) {
    if (waypoints.empty()) return 0;

    previousIndex = std::min(previousIndex, waypoints.size() - 1);
    constexpr double kMaximumBackwardProgress = 2.0;
    constexpr double kMaximumForwardProgress = 35.0;
    constexpr double kHeadingGate = 1.35;
    const double previousS = waypoints[previousIndex].s;
    std::size_t begin = previousIndex;
    while (begin > 0 &&
           waypoints[begin - 1].s >= previousS - kMaximumBackwardProgress) {
        --begin;
    }
    std::size_t end = previousIndex + 1;
    while (end < waypoints.size() &&
           waypoints[end].s <= previousS + kMaximumForwardProgress) {
        ++end;
    }
    std::size_t nearest = begin;
    double bestScore = std::numeric_limits<double>::infinity();
    bool foundHeadingCompatible = false;

    for (std::size_t index = begin; index < end; ++index) {
        const double distance = distanceSquared(
            gt.ego.x, gt.ego.y, waypoints[index].x, waypoints[index].y);
        const double headingError = std::abs(normalizeAngle(
            gt.ego.heading - waypoints[index].yaw));
        const bool headingCompatible = headingError <= kHeadingGate;
        if (foundHeadingCompatible && !headingCompatible) continue;
        const double progress = waypoints[index].s - previousS;
        const double score = distance + 2.0 * headingError * headingError +
                             0.002 * progress * progress;
        if ((headingCompatible && !foundHeadingCompatible) ||
            score < bestScore) {
            foundHeadingCompatible = headingCompatible;
            bestScore = score;
            nearest = index;
        }
    }
    return nearest;
}

// 최근접점에서 경로 누적거리로 lookahead 이상 앞선 점을 찾는다.
std::size_t findTargetWaypoint(const std::vector<Waypoint>& waypoints,
                          std::size_t nearestIndex,
                          double lookahead) {
    if (waypoints.empty()) return 0;
    const double targetS = waypoints[nearestIndex].s + lookahead;
    std::size_t index = nearestIndex;
    while (index + 1 < waypoints.size() && waypoints[index].s < targetS) ++index;
    return index;
}

// 앞쪽 경로의 yaw 변화량 / 이동거리로 최대 곡률을 계산한다.
double findMaxCurvature(const std::vector<Waypoint>& waypoints,
                        std::size_t nearestIndex,
                        double checkDistance) {
    if (waypoints.size() < 2) return 0.0;
    const double endS = waypoints[nearestIndex].s + checkDistance;
    double maximum = 0.0;

    for (std::size_t index = nearestIndex;
         index + 1 < waypoints.size() && waypoints[index].s <= endS;
         ++index) {
        const double ds = waypoints[index + 1].s - waypoints[index].s;
        if (ds <= 1e-6) continue;
        maximum = std::max(maximum,
                      std::abs(normalizeAngle(waypoints[index + 1].yaw -
                                         waypoints[index].yaw)) / ds);
    }
    return maximum;
}

// 곡선에서는 짧은 lookahead, 직선에서는 긴 lookahead를 고른 뒤,
// 현재 속도에 비례해 한 번 더 줄인다(하한 minLookahead).
double chooseLookahead(double maxCurvature,
                       const PathPlannerConfig& config,
                       double egoSpeed) {
    const double base = maxCurvature >= config.curveThreshold
                            ? config.curveLookahead
                            : config.straightLookahead;
    const double speedBased =
        config.lookaheadSpeedGain * std::max(0.0, egoSpeed);
    return std::clamp(speedBased, config.minLookahead, base);
}

// Ego heading과 target을 바라보는 방향의 각도 차이를 -pi~pi로 만든다.
double computeHeadingError(const GtFrame& gt, const Waypoint& target) {
    const double dx = target.x - gt.ego.x;
    const double dy = target.y - gt.ego.y;
    return normalizeAngle(std::atan2(dy, dx) - gt.ego.heading);
}

// Pure Pursuit bicycle model을 사용해 앞바퀴 조향각을 계산한다.
double computePurePursuitSteering(double headingError,
                                  double targetDistance,
                                  double wheelbase) {
    if (targetDistance <= 1e-6) return 0.0;
    return std::atan2(2.0 * wheelbase * std::sin(headingError), targetDistance);
}

// 앞쪽에 곡선이 있으면 감속한 목표속도를 선택한다.
double chooseTargetSpeed(double maxCurvature,
                         const PathPlannerConfig& config) {
    double nominalSpeed = config.straightSpeed;
    if (maxCurvature >= config.sharpCurveThreshold) {
        nominalSpeed = config.sharpCurveSpeed;
    } else if (maxCurvature >= config.curveThreshold) {
        nominalSpeed = config.curveSpeed;
    }
    // Discrete "curve/sharp curve" speeds alone do not reflect whether the
    // steering/tyre model can physically negotiate the detected radius.  Cap
    // every curved path by lateral acceleration so the same rule also covers
    // tight VTD junction connectors and newly generated avoidance paths.
    if (maxCurvature >= config.curveThreshold) {
        const double physicalSpeed = std::sqrt(
            config.maximumLateralAcceleration /
            std::max(maxCurvature, 1e-6));
        nominalSpeed = std::min(nominalSpeed, physicalSpeed);
    }
    return nominalSpeed;
}

// 최근접점, 곡률, lookahead target, 조향각, 목표속도를 순서대로 계산한다.
PathPlan runPurePursuit(PathPlannerData& planner, const GtFrame& gt,
                        double egoSpeed) {
    PathPlan plan;
    if (planner.waypoints.empty()) return plan;

    plan.nearestIndex = findNearestWaypoint(
        planner.waypoints, gt, planner.nearestIndex);
    plan.maxCurvature = findMaxCurvature(
        planner.waypoints, plan.nearestIndex,
        planner.config.curvatureCheckDistance);
    plan.lookahead = chooseLookahead(plan.maxCurvature, planner.config,
                                     egoSpeed);
    const Waypoint& nearest = planner.waypoints[plan.nearestIndex];
    const double routeOffset = std::hypot(nearest.x - gt.ego.x,
                                          nearest.y - gt.ego.y);
    const bool recoveringToRoute = routeOffset > 1.0;
    if (recoveringToRoute) {
        const double recoveryLookahead = std::clamp(
            4.0 + 2.0 * routeOffset, 6.0, 14.0);
        plan.lookahead = std::max(plan.lookahead, recoveryLookahead);
    }
    plan.targetIndex = findTargetWaypoint(
        planner.waypoints, plan.nearestIndex, plan.lookahead);

    const Waypoint& target = planner.waypoints[plan.targetIndex];
    const double dx = target.x - gt.ego.x;
    const double dy = target.y - gt.ego.y;
    const double targetDistance = std::hypot(dx, dy);
    const double headingError = computeHeadingError(gt, target);

    // A consumed local prefix is not a forward steering target. Projection
    // clamps to its last sample, so without this check PP keeps chasing a
    // point beside/behind the rear axle and can demand nearly pi/2 steering.
    const bool terminalTarget = plan.targetIndex + 1 == planner.waypoints.size();
    if (terminalTarget && targetDistance <= planner.config.goalTolerance) {
        plan.goalReached = true;
        plan.valid = true;
        planner.nearestIndex = plan.nearestIndex;
        planner.latestPlan = plan;
        return plan;
    }
    if (terminalTarget && dx * std::cos(gt.ego.heading) +
                              dy * std::sin(gt.ego.heading) <= 0.) {
        planner.latestPlan = plan;
        return plan;
    }

    plan.steering = computePurePursuitSteering(
        headingError, targetDistance, planner.config.wheelbase);
    plan.targetSpeed = chooseTargetSpeed(plan.maxCurvature, planner.config);
    // A sharp bend may just have moved behind Ego while the vehicle is still
    // carrying substantial steering to settle onto the outgoing lane.  Using
    // forward path curvature alone releases the speed cap too early and can
    // accelerate the car while it is still turning.  The steering demand is
    // another estimate of instantaneous curvature, so keep the same physical
    // lateral-acceleration cap until the demanded steering has unwound.
    const double steeringCurvature = std::abs(
        std::tan(plan.steering) / planner.config.wheelbase);
    if (steeringCurvature >= planner.config.curveThreshold) {
        const double steeringLimitedSpeed = std::sqrt(
            planner.config.maximumLateralAcceleration /
            std::max(steeringCurvature, 1e-6));
        plan.targetSpeed = std::min(plan.targetSpeed, steeringLimitedSpeed);
    }
    if (recoveringToRoute) {
        const double recoverySpeed = std::clamp(3.0 - 0.3 * routeOffset,
                                                1.5, 2.5);
        plan.targetSpeed = std::min(plan.targetSpeed, recoverySpeed);
    }
    plan.goalReached = plan.targetIndex + 1 == planner.waypoints.size() &&
                       targetDistance <= planner.config.goalTolerance;
    if (plan.goalReached) plan.targetSpeed = 0.0;
    plan.valid = true;

    planner.nearestIndex = plan.nearestIndex;
    planner.latestPlan = plan;
    return plan;
}
