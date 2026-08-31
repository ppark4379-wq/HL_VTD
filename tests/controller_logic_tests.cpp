#include "data/map/static_zones.hpp"
#include "network/send/control_sender.hpp"
#include "planning/behavior/behavior_planner.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/lane_change_planner.hpp"
#include "utils/geometry.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

static_assert(CONTROL_PACKET_SIZE == 9,
              "VTD control command must be exactly 9 bytes");

void testTrafficControllerParsing() {
    XmlElement root;
    XmlElement controller;
    controller.attributes["id"] = "11";
    controller.attributes["name"] = "ctrl011";
    for (const char* signal : {"54", "55", "58"}) {
        XmlElement control;
        control.attributes["signalId"] = signal;
        controller.children.emplace_back("control", std::move(control));
    }
    root.children.emplace_back("controller", std::move(controller));
    PathGeneratorData map;
    parseTrafficControllers(map, root);
    assert(map.trafficControllers.at(11).signalIds.size() == 3);
    assert(map.signalToController.at(54) == 11);
    assert(map.signalToController.at(55) == 11);
    assert(map.signalToController.at(58) == 11);
}

Polynomial constantWidth(double width) {
    Polynomial value;
    value.a = width;
    return value;
}

PathGeneratorData makeStraightTwoLaneMap() {
    PathGeneratorData map;
    Road road;
    road.id = 1;
    road.length = 100.0;
    Geometry geometry;
    geometry.type = GeometryType::Line;
    geometry.length = road.length;
    road.geometries.push_back(geometry);
    LaneSection section;
    LaneDefinition source;
    source.id = -1;
    source.type = "driving";
    source.widths.push_back(constantWidth(3.5));
    source.roadMarks.push_back({0.0, "both", "standard"});
    LaneDefinition target = source;
    target.id = -2;
    section.lanes.emplace(source.id, source);
    section.lanes.emplace(target.id, target);
    road.sections.push_back(section);
    map.roads.emplace(road.id, road);
    map.laneGraph[{1, 0, -2}].push_back({1, 0, -1});
    map.laneGraph[{1, 0, -1}].push_back({1, 0, -2});
    for (int laneId : {-1, -2}) {
        LaneSegment segment;
        segment.key = {1, 0, laneId};
        segment.startS = 0.0;
        segment.endS = 100.0;
        for (int index = 0; index <= 100; ++index) {
            Waypoint point;
            point.x = static_cast<double>(index);
            point.y = laneId == -1 ? -1.75 : -5.25;
            point.yaw = 0.0;
            point.roadS = static_cast<double>(index);
            point.roadId = 1;
            point.laneId = laneId;
            point.sectionIndex = 0;
            segment.points.push_back(point);
        }
        map.laneSegments.emplace(segment.key, std::move(segment));
    }
    return map;
}

PathPlannerData makeStraightGlobalPath() {
    PathPlannerData planner;
    for (int index = 0; index <= 90; ++index) {
        Waypoint point;
        point.x = static_cast<double>(index);
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = static_cast<double>(index);
        point.roadS = point.s;
        point.roadId = 1;
        point.laneId = -1;
        planner.waypoints.push_back(point);
    }
    return planner;
}

PathPlannerData makeGlobalPathWithNextRoad(PathGeneratorData& map) {
    PathPlannerData planner = makeStraightGlobalPath();
    Road nextRoad;
    nextRoad.id = 2;
    nextRoad.length = 50.0;
    Geometry geometry;
    geometry.type = GeometryType::Line;
    geometry.x = 100.0;
    geometry.y = -3.5;
    geometry.length = 50.0;
    nextRoad.geometries.push_back(geometry);
    LaneSection section;
    LaneDefinition lane;
    lane.id = -1;
    lane.type = "driving";
    lane.widths.push_back(constantWidth(3.5));
    section.lanes.emplace(-1, lane);
    nextRoad.sections.push_back(section);
    map.roads.emplace(2, nextRoad);

    LaneSegment segment;
    segment.key = {2, 0, -1};
    segment.startS = 0.0;
    segment.endS = 50.0;
    for (int index = 0; index <= 50; ++index) {
        Waypoint point;
        point.x = 100.0 + index;
        point.y = -5.25;
        point.yaw = 0.0;
        point.roadS = static_cast<double>(index);
        point.roadId = 2;
        point.laneId = -1;
        point.sectionIndex = 0;
        segment.points.push_back(point);
        point.s = 100.0 + index;
        planner.waypoints.push_back(point);
    }
    map.laneSegments.emplace(segment.key, std::move(segment));
    map.laneGraph[{1, 0, -2}].push_back({2, 0, -1});
    map.laneGraph[{1, 0, -1}].push_back({2, 0, -1});
    return planner;
}

WorldState makeWorldWithObstacle() {
    WorldState world;
    world.ego.x = 10.0F;
    world.ego.y = -1.75F;
    world.ego.heading = 0.0F;
    world.egoSpeed = 5.0;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 1;
    world.egoLaneId = -1;
    world.egoSectionIndex = 0;
    world.egoRoadS = 10.0;
    world.egoPathS = 10.0;
    world.frontBumperPathS = 13.8;
    world.egoObb = makeObb2D(11.4, -1.75, 0.0, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 7;
    obstacle.raw.x = 35.0F;
    obstacle.raw.y = -1.75F;
    obstacle.raw.length = 4.0F;
    obstacle.raw.width = 2.0F;
    obstacle.obb = makeObb2D(35.0, -1.75, 0.0, 4.0, 2.0);
    obstacle.laneValid = true;
    obstacle.roadId = 1;
    obstacle.laneId = -1;
    obstacle.sectionIndex = 0;
    obstacle.pathValid = true;
    obstacle.pathS = 35.0;
    obstacle.relativeS = 25.0;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);
    return world;
}

void testStaticZoneRouteAlignment() {
    StaticZoneData zones;
    StopLine line;
    line.begin = {20.0, -2.0};
    line.end = {20.0, 2.0};
    zones.stopLines.push_back(line);
    Crosswalk crosswalk;
    crosswalk.polygon = {{30.0, -2.0}, {34.0, -2.0},
                         {34.0, 2.0}, {30.0, 2.0}};
    zones.crosswalks.push_back(crosswalk);
    const auto path = makeStraightGlobalPath().waypoints;
    assert(alignStaticZonesToRoute(zones, path));
    assert(std::abs(zones.stopLines.front().routeS - 20.0) < 1e-6);
    assert(std::abs(zones.crosswalks.front().entryRouteS - 30.0) < 1e-6);
    assert(std::abs(zones.crosswalks.front().exitRouteS - 34.0) < 1e-6);
}

void testPredictedCollisionIsNotCurrentCollision() {
    WorldState world = makeWorldWithObstacle();
    world.objects.front().raw.x = 20.0F;
    world.objects.front().pathS = 20.0;
    world.objects.front().relativeS = 10.0;
    world.objects.front().obb = makeObb2D(20.0, -1.75, 0.0, 4.0, 2.0);
    LocalPathData path;
    path.valid = true;
    path.points = makeStraightGlobalPath().waypoints;
    PathSafetyResult safety;
    resetPathSafety(safety);
    checkCurrentCollisions(safety, world.egoObb, world.objects);
    checkPredictedCollisions(safety, world, path, 3.0, 0.1);
    assert(!safety.collision);
    assert(safety.predictedCollision);
    assert(safety.predictedCollisionTTC > 0.0);
}

void testObstacleLaneChange() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    PathPlannerData globalPath = makeStraightGlobalPath();
    WorldState world = makeWorldWithObstacle();
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    safetyConfig.collisionPredictionHorizon = 5.0;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 10));
    assert(laneChange.phase == LaneChangePhase::Prepare);
    assert(laneChange.targetLaneId == -2);
    assert(laneChange.turnSignal != 0);
    const auto nearObstacle = std::min_element(
        laneChange.path.begin(), laneChange.path.end(),
        [](const Waypoint& left, const Waypoint& right) {
            return std::abs(left.s - 35.0) < std::abs(right.s - 35.0);
        });
    assert(nearObstacle != laneChange.path.end());
    assert(nearObstacle->y < -4.0);
    advanceLaneChangePhase(laneChange, config, world,
                           config.prepareTime, false, true);
    assert(laneChange.phase == LaneChangePhase::Execute);
    advanceLaneChangePhase(laneChange, config, world, 0.1, false, false);
    assert(laneChange.waitingForReturn);

    WorldObjectState targetVehicle;
    targetVehicle.raw.id = 8;
    targetVehicle.raw.x = 20.0F;
    targetVehicle.raw.y = -5.25F;
    targetVehicle.raw.length = 4.0F;
    targetVehicle.raw.width = 2.0F;
    targetVehicle.obb = makeObb2D(20.0, -5.25, 0.0, 4.0, 2.0);
    targetVehicle.laneValid = true;
    targetVehicle.roadId = 1;
    targetVehicle.laneId = -2;
    targetVehicle.sectionIndex = 0;
    targetVehicle.pathValid = true;
    targetVehicle.pathS = 20.0;
    WorldState blockedWorld = makeWorldWithObstacle();
    blockedWorld.objects.push_back(targetVehicle);
    LaneChangeData blocked;
    assert(!tryPrepareObstacleLaneChange(
        blocked, config, safetyConfig, blockedWorld, map, globalPath, 10));

    // 3초 밖이지만 회피 경로의 목표 차선 위에 있는 객체도 진입 전에 걸러야 한다.
    targetVehicle.raw.x = 45.0F;
    targetVehicle.pathS = 45.0;
    targetVehicle.obb = makeObb2D(45.0, -5.25, 0.0, 4.0, 2.0);
    WorldState farBlockedWorld = makeWorldWithObstacle();
    farBlockedWorld.objects.push_back(targetVehicle);
    LaneChangeData farBlocked;
    assert(!tryPrepareObstacleLaneChange(
        farBlocked, config, safetyConfig, farBlockedWorld,
        map, globalPath, 10));
}

void testKeepTargetLaneWhenItRejoinsNextRoad() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    PathPlannerData globalPath = makeGlobalPathWithNextRoad(map);
    WorldState world = makeWorldWithObstacle();
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    safetyConfig.collisionPredictionHorizon = 5.0;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 10));
    assert(laneChange.routeMode == LaneChangeRouteMode::KeepTargetLane);
    const LaneKey expectedReconnect{2, 0, -1};
    assert(laneChange.reconnectLane == expectedReconnect);

    advanceLaneChangePhase(laneChange, config, world,
                           config.prepareTime, false, true);
    world.egoRoadId = 2;
    world.egoSectionIndex = 0;
    world.egoLaneId = -1;
    advanceLaneChangePhase(laneChange, config, world, 0.1, false, true);
    assert(laneChange.phase == LaneChangePhase::Settle);
}

void testNearbyAdjacentVehicleCapsSpeedBeforeCutIn() {
    PlanningBehaviorConfig config;
    config.followTimeGap = 1.8;
    config.minimumFollowDistance = 5.0;
    config.nearbyObjectCautionSpeed = 20.0 / 3.6;
    config.nearbyObjectLateralDistance = 5.5;

    WorldState world;
    world.egoSpeed = 50.0 / 3.6;
    WorldObjectState adjacent;
    adjacent.raw.id = 2;
    adjacent.raw.speed = 40.0F / 3.6F;
    adjacent.raw.heading = 0.0F;
    adjacent.pathValid = true;
    adjacent.isAhead = true;
    adjacent.isOnEgoPath = false;
    adjacent.relativeS = 25.0;
    adjacent.relativeD = 3.5;
    world.objects.push_back(adjacent);

    BehaviorState state;
    BehaviorDecision output;
    ConstraintSet constraints;
    constraints.speedLimit = 50.0 / 3.6;
    StopDecision stop;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Follow);
    assert(std::abs(output.targetSpeed - 20.0 / 3.6) < 1e-9);

    // 같은 객체여도 동적 주의거리(약 30 m) 밖이면 순항한다.
    world.objects.front().relativeS = 35.0;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Cruise);
    assert(std::abs(output.targetSpeed - 50.0 / 3.6) < 1e-9);

    // 경로 옆의 콘·표지물은 끼어들기 차량이 아니므로 정지하지 않는다.
    world.objects.front().relativeS = 10.0;
    world.objects.front().raw.speed = 0.0F;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Cruise);
    assert(std::abs(output.targetSpeed - 50.0 / 3.6) < 1e-9);
}

void testSharpCurveHasSeparateSpeed() {
    PathPlannerConfig config;
    config.curveThreshold = 0.02;
    config.sharpCurveThreshold = 0.20;
    config.straightSpeed = 50.0 / 3.6;
    config.curveSpeed = 30.0 / 3.6;
    config.sharpCurveSpeed = 20.0 / 3.6;
    assert(std::abs(chooseTargetSpeed(0.01, config) -
                    config.straightSpeed) < 1e-9);
    assert(std::abs(chooseTargetSpeed(0.055, config) -
                    config.curveSpeed) < 1e-9);
    assert(std::abs(chooseTargetSpeed(0.22, config) -
                    config.sharpCurveSpeed) < 1e-9);
}

}  // namespace

int main() {
    testTrafficControllerParsing();
    testStaticZoneRouteAlignment();
    testPredictedCollisionIsNotCurrentCollision();
    testObstacleLaneChange();
    testKeepTargetLaneWhenItRejoinsNextRoad();
    testNearbyAdjacentVehicleCapsSpeedBeforeCutIn();
    testSharpCurveHasSeparateSpeed();
    std::cout << "controller_logic_tests: PASS\n";
    return 0;
}
