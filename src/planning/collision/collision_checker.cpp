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
constexpr double kOncomingClosestApproachMargin = 0.5;

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

// 맞은편 차선 차량을 전부 직선 등속 예측에 넣으면 곡선에서 가짜 충돌이
// 생긴다. 다만 상대속도 기준 최근접 거리가 두 차폭보다 작다면 단순 맞은편
// 통행이 아니라 실제로 Ego 쪽으로 수렴하는 상황이다. 이 경우에는 넓은
// 회랑 밖이어도 아래 OBB 미래 검사가 최종 판단하도록 후보에 포함한다.
bool oncomingTrajectoryConverges(const WorldObjectState& object,
                                 const WorldState& world,
                                 double egoWidth,
                                 double horizon) {
    const double relativeX =
        static_cast<double>(object.raw.x) - static_cast<double>(world.ego.x);
    const double relativeY =
        static_cast<double>(object.raw.y) - static_cast<double>(world.ego.y);
    const double relativeVelocityX =
        objectVelocityX(object) -
        world.egoSpeed * std::cos(static_cast<double>(world.ego.heading));
    const double relativeVelocityY =
        objectVelocityY(object) -
        world.egoSpeed * std::sin(static_cast<double>(world.ego.heading));
    const double relativeSpeedSquared =
        relativeVelocityX * relativeVelocityX +
        relativeVelocityY * relativeVelocityY;
    if (relativeSpeedSquared <= 1e-6) return false;

    const double closestTime = std::clamp(
        -(relativeX * relativeVelocityX +
          relativeY * relativeVelocityY) /
            relativeSpeedSquared,
        0.0, horizon);
    if (closestTime <= 1e-6) return false;
    const double closestX = relativeX + relativeVelocityX * closestTime;
    const double closestY = relativeY + relativeVelocityY * closestTime;
    const double collisionWidth =
        0.5 * (egoWidth + static_cast<double>(object.raw.width)) +
        kOncomingClosestApproachMargin;
    return std::hypot(closestX, closestY) <= collisionWidth;
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
                           double radius,
                           double egoWidth,
                           double horizon) {
    const double headingDifference = std::abs(normalizeAngle(
        objectMotionHeading(object) -
        static_cast<double>(world.ego.heading)));

    if (headingDifference > kOncomingHeadingDifference) {
        if (insideRouteCorridor(object, kOncomingLateralRange)) return true;
        return distanceToEgo(object, world) <= radius &&
               oncomingTrajectoryConverges(
                   object, world, egoWidth, horizon);
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
                            const std::vector<WorldObjectState>& objects,
                            int excludeObjectId) {
    for (const auto& object : objects) {
        if (static_cast<int>(object.raw.id) == excludeObjectId) continue;
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
                              double step,
                              bool checkAllObjects) {
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
            // 일반 주행은 현재 route corridor/근접 반경으로 계산량과 오탐을
            // 제한한다. 반면 이미 만든 차선변경 경로를 검증할 때는 그 경로가
            // 여러 차선을 지나므로, 원래 Ego 차선 기준 corridor 밖의 정적
            // 장애물도 빠짐없이 검사해야 한다.
            if (!checkAllObjects &&
                !isPredictionCandidate(
                    object, world, radius, ego.width, horizon)) continue;
            const double objectHeading =
                static_cast<double>(object.raw.heading);
            // Constant-velocity prediction is exact only for the current
            // sample.  Give moving actors a small time-growing envelope so a
            // candidate that merely grazes today's extrapolated box is not
            // committed.  A stopped vehicle whose body crosses the Ego path
            // can start suddenly (M14 object 12); reserve additional side
            // room for that case while leaving cones and parallel parked
            // vehicles at their ordinary dimensions.
            const double motionSpeed = objectMotionSpeed(object);
            const double headingDifference = std::abs(normalizeAngle(
                objectHeading - egoPose.heading));
            const bool vehicleSized = object.raw.length >= 2.5F &&
                object.raw.length <= 7.5F && object.raw.width >= 1.4F &&
                object.raw.width <= 2.8F;
            const bool crossingOrientation =
                headingDifference >= 0.6108652382 &&
                headingDifference <= 2.5307274154;
            const double uncertaintyMargin = motionSpeed >= 0.3
                ? std::min(0.45, 0.15 + 0.06 * time)
                : (vehicleSized && crossingOrientation ? 1.50 : 0.0);
            const Obb2D predictedObject = makeObb2D(
                object.raw.x + objectVelocityX(object) * time,
                object.raw.y + objectVelocityY(object) * time,
                objectHeading,
                object.raw.length + 2.0 * uncertaintyMargin,
                object.raw.width + 2.0 * uncertaintyMargin);
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

bool requiresEmergencyBrake(const PathSafetyResult& safety,
                            double emergencyTTC) {
    if (safety.collision) return true;
    if (emergencyTTC < 0.0 || !std::isfinite(emergencyTTC)) return true;
    if (safety.predictedCollision &&
        safety.predictedCollisionTTC <= emergencyTTC) {
        return true;
    }
    return safety.minimumTTC <= emergencyTTC;
}
