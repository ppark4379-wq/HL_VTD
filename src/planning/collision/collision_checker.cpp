#include "planning/collision/collision_checker.hpp"

#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// 반대 방향으로 진행하는 객체로 판단하는 heading 차이 (약 120도).
constexpr double kOncomingHeadingDifference = 2.094;

// 전역경로 회랑(corridor) 기준 검사 범위.
// 횡방향은 현재 차선과 인접 차선을 덮어야 한다. 이 지도의 차선 폭은
// 3.0 / 3.2 / 4.0 m이므로 인접 차선 중심이 최대 4.0 m 떨어진다.
constexpr double kPredictionAheadRange = 60.0;
constexpr double kPredictionBehindRange = -5.0;
constexpr double kPredictionLateralRange = 5.0;
constexpr double kOncomingLateralRange = 2.0;

// 회랑 밖이라도 물리적으로 가까우면 무조건 검사하는 반경.
// 교차로에서 측면 도로로 접근하는 차량은 전역경로 횡거리가 매우 크므로
// 회랑 조건만으로는 걸리지 않는다.
constexpr double kRadiusPerSpeed = 5.0;
constexpr double kRadiusBase = 20.0;
constexpr double kRadiusMinimum = 40.0;
constexpr double kRadiusMaximum = 100.0;

struct EgoShape {
    double length{};
    double width{};
    double centerOffset{};  // 후륜축(Ego 좌표)에서 OBB 중심까지의 종방향 거리
};

double distanceBetween(const Point2D& first, const Point2D& second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

// 현재 프레임의 Ego OBB에서 차량 치수와 중심 오프셋을 역산한다.
// World process가 만든 OBB를 그대로 쓰므로 vehicle.yaml을 다시 읽지 않는다.
EgoShape makeEgoShape(const WorldState& world) {
    EgoShape shape;
    shape.width = distanceBetween(world.egoObb.corners[0],
                                  world.egoObb.corners[1]);
    shape.length = distanceBetween(world.egoObb.corners[1],
                                   world.egoObb.corners[2]);
    double centerX = 0.0;
    double centerY = 0.0;
    for (const auto& corner : world.egoObb.corners) {
        centerX += 0.25 * corner.x;
        centerY += 0.25 * corner.y;
    }
    const double offsetX = centerX - static_cast<double>(world.ego.x);
    const double offsetY = centerY - static_cast<double>(world.ego.y);
    shape.centerOffset = offsetX * std::cos(world.ego.heading) +
                         offsetY * std::sin(world.ego.heading);
    return shape;
}

// 속도가 빠를수록 넓게 본다. 저속 40 m ~ 고속 100 m.
double predictionRadius(double egoSpeed) {
    return std::clamp(kRadiusPerSpeed * std::abs(egoSpeed) + kRadiusBase,
                      kRadiusMinimum, kRadiusMaximum);
}

double distanceToEgo(const WorldObjectState& object, const WorldState& world) {
    return std::hypot(
        static_cast<double>(object.raw.x) - static_cast<double>(world.ego.x),
        static_cast<double>(object.raw.y) - static_cast<double>(world.ego.y));
}

bool insideRouteCorridor(const WorldObjectState& object, double lateralRange) {
    return object.pathValid &&
           object.relativeS >= kPredictionBehindRange &&
           object.relativeS <= kPredictionAheadRange &&
           std::abs(object.relativeD) <= lateralRange;
}

// 미래 충돌검사를 적용할 객체를 고른다.
//
// 두 가지 기준을 OR로 묶는다.
//   1) 전역경로 회랑 : 현재 차선 + 인접 차선. 앞차·컷인 대응
//   2) 근접 반경     : 회랑 밖이라도 가까우면 검사. 교차로 측면 차량 대응
//
// 단, 마주 오는 객체는 예외다. 객체를 등속 직진으로 예측하기 때문에
// 곡선에서 반대 차선 차량이 우리 차선으로 밀려 들어와 가짜 충돌이 난다.
// 실제로 우리 차선 폭 안에 있을 때만(역주행·중앙선 침범) 검사한다.
bool isPredictionCandidate(const WorldObjectState& object,
                           const WorldState& world,
                           double radius) {
    const double headingDifference = std::abs(normalizeAngle(
        static_cast<double>(object.raw.heading) -
        static_cast<double>(world.ego.heading)));

    if (headingDifference > kOncomingHeadingDifference) {
        return insideRouteCorridor(object, kOncomingLateralRange);
    }
    if (insideRouteCorridor(object, kPredictionLateralRange)) return true;
    return distanceToEgo(object, world) <= radius;
}

}  // namespace

double calculateTTC(double longitudinalGap, double closingSpeed) {
    if (longitudinalGap <= 0.0) return 0.0;
    if (closingSpeed <= 1e-6) return kInfinity;
    return longitudinalGap / closingSpeed;
}

bool routeToWorld(Pose2D& output,
                  const std::vector<Waypoint>& points,
                  double targetS) {
    if (points.size() < 2) return false;
    if (targetS <= points.front().s) {
        output = {points.front().x, points.front().y, points.front().yaw};
        return true;
    }
    if (targetS >= points.back().s) return false;

    for (std::size_t index = 0; index + 1 < points.size(); ++index) {
        const Waypoint& begin = points[index];
        const Waypoint& end = points[index + 1];
        if (targetS > end.s) continue;
        const double span = end.s - begin.s;
        const double ratio = span > 1e-9 ? (targetS - begin.s) / span : 0.0;
        output.x = begin.x + ratio * (end.x - begin.x);
        output.y = begin.y + ratio * (end.y - begin.y);
        output.heading =
            begin.yaw + ratio * normalizeAngle(end.yaw - begin.yaw);
        return true;
    }
    return false;
}

void resetPathSafety(PathSafetyResult& output) {
    output = {};
    output.valid = true;
    output.minimumTTC = kInfinity;
    output.predictedCollisionTTC = kInfinity;
    output.riskObjectId = -1;
}

void checkCurrentCollisions(PathSafetyResult& output,
                            const Obb2D& egoObb,
                            const std::vector<WorldObjectState>& objects) {
    for (const auto& object : objects) {
        if (!intersects(egoObb, object.obb)) continue;
        output.collision = true;
        output.minimumTTC = 0.0;
        output.riskObjectId = static_cast<int>(object.raw.id);
        return;
    }
}

void updateMinimumTTC(PathSafetyResult& output,
                      const std::vector<WorldObjectState>& objects,
                      bool ignoreStaticObjects,
                      int ignoredObjectId) {
    for (const auto& object : objects) {
        if (!object.isAhead || !object.isOnEgoPath) continue;
        if (ignoreStaticObjects && std::abs(object.raw.speed) < 0.3) continue;
        if (static_cast<int>(object.raw.id) == ignoredObjectId) continue;
        if (object.ttc >= output.minimumTTC) continue;
        output.minimumTTC = object.ttc;
        output.riskObjectId = static_cast<int>(object.raw.id);
    }
}

void checkPredictedCollisions(PathSafetyResult& output,
                              const WorldState& world,
                              const LocalPathData& localPath,
                              double horizon,
                              double step) {
    if (output.collision) return;
    if (!localPath.valid || !world.egoPathValid) return;
    if (step <= 0.0 || horizon <= 0.0) return;

    const EgoShape ego = makeEgoShape(world);
    const double egoSpeed = std::max(0.0, world.egoSpeed);
    const double radius = predictionRadius(world.egoSpeed);

    for (double time = step; time <= horizon + 1e-9; time += step) {
        Pose2D egoPose;
        if (!routeToWorld(egoPose, localPath.points,
                          world.egoPathS + egoSpeed * time)) {
            break;  // 지역경로 끝까지 예측했으면 더 볼 필요가 없다.
        }
        const Obb2D predictedEgo = makeObb2D(
            egoPose.x + ego.centerOffset * std::cos(egoPose.heading),
            egoPose.y + ego.centerOffset * std::sin(egoPose.heading),
            egoPose.heading, ego.length, ego.width);

        for (const auto& object : world.objects) {
            if (!isPredictionCandidate(object, world, radius)) continue;
            const double objectHeading =
                static_cast<double>(object.raw.heading);
            const double travel =
                static_cast<double>(object.raw.speed) * time;
            const Obb2D predictedObject = makeObb2D(
                object.raw.x + travel * std::cos(objectHeading),
                object.raw.y + travel * std::sin(objectHeading),
                objectHeading, object.raw.length, object.raw.width);
            if (!intersects(predictedEgo, predictedObject)) continue;

            output.predictedCollision = true;
            output.predictedCollisionTTC = time;
            if (time < output.minimumTTC) {
                output.minimumTTC = time;
                output.riskObjectId = static_cast<int>(object.raw.id);
            }
            return;  // 가장 이른 충돌 시점만 필요하다.
        }
    }
}
