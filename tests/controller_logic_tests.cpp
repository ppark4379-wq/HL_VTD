#include "data/map/static_zones.hpp"
#include "network/send/control_sender.hpp"
#include "planning/behavior/behavior_planner.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/lane_change_planner.hpp"
#include "planning/local/path_planner.hpp"
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

void testLaneChangeAcrossRenumberedLaneSection() {
    PathGeneratorData map;
    Road road;
    road.id = 418;
    road.length = 100.0;
    Geometry geometry;
    geometry.type = GeometryType::Line;
    geometry.length = road.length;
    road.geometries.push_back(geometry);
    road.laneOffsets.push_back({0.0, 0.0, 0.0, 0.0, 0.0});
    road.laneOffsets.push_back({50.0, 3.5, 0.0, 0.0, 0.0});

    LaneSection before;
    before.s = 0.0;
    for (int laneId : {-1, -2}) {
        LaneDefinition lane;
        lane.id = laneId;
        lane.type = "driving";
        lane.widths.push_back(constantWidth(3.5));
        lane.roadMarks.push_back({0.0, "both", "standard"});
        before.lanes.emplace(laneId, lane);
    }
    before.lanes.at(-1).successor = -2;
    before.lanes.at(-2).successor = -3;

    LaneSection after;
    after.s = 50.0;
    for (int laneId : {-1, -2, -3}) {
        LaneDefinition lane;
        lane.id = laneId;
        lane.type = "driving";
        lane.widths.push_back(constantWidth(3.5));
        lane.roadMarks.push_back({50.0, "both", "standard"});
        after.lanes.emplace(laneId, lane);
    }
    after.lanes.at(-2).predecessor = -1;
    after.lanes.at(-3).predecessor = -2;
    road.sections.push_back(before);
    road.sections.push_back(after);
    map.roads.emplace(road.id, road);
    map.laneGraph[{418, 0, -1}].push_back({418, 1, -2});
    map.laneGraph[{418, 0, -2}].push_back({418, 1, -3});
    map.laneGraph[{418, 1, -2}].push_back({418, 1, -3});
    map.laneGraph[{418, 1, -3}].push_back({418, 1, -2});

    PathPlannerData globalPath;
    for (int index = 0; index <= 100; ++index) {
        Waypoint point;
        point.x = static_cast<double>(index);
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = static_cast<double>(index);
        point.roadS = point.s;
        point.roadId = 418;
        point.sectionIndex = index < 50 ? 0 : 1;
        point.laneId = index < 50 ? -1 : -2;
        globalPath.waypoints.push_back(point);
    }

    WorldState world;
    world.ego.x = 35.0F;
    world.ego.y = -1.75F;
    world.ego.heading = 0.0F;
    world.egoSpeed = 5.0;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 418;
    world.egoLaneId = -1;
    world.egoSectionIndex = 0;
    world.egoRoadS = 35.0;
    world.egoPathS = 35.0;
    world.frontBumperPathS = 38.8;
    world.egoObb = makeObb2D(36.4, -1.75, 0.0, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 4;
    obstacle.raw.x = 60.0F;
    obstacle.raw.y = -1.75F;
    obstacle.raw.length = 4.0F;
    obstacle.raw.width = 2.0F;
    obstacle.obb = makeObb2D(60.0, -1.75, 0.0, 4.0, 2.0);
    obstacle.laneValid = true;
    obstacle.roadId = 418;
    obstacle.laneId = -2;
    obstacle.sectionIndex = 1;
    obstacle.roadS = 60.0;
    obstacle.pathValid = true;
    obstacle.pathS = 60.0;
    obstacle.relativeS = 25.0;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    safetyConfig.collisionPredictionHorizon = 5.0;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 35));
    assert(laneChange.routeMode == LaneChangeRouteMode::ReturnToSource);
    assert(laneChange.targetLaneId == -2);
    const auto nearObstacle = std::min_element(
        laneChange.path.begin(), laneChange.path.end(),
        [](const Waypoint& left, const Waypoint& right) {
            return std::abs(left.s - 60.0) < std::abs(right.s - 60.0);
        });
    assert(nearObstacle != laneChange.path.end());
    assert(nearObstacle->sectionIndex == 1);
    assert(nearObstacle->laneId == -3);
    assert(nearObstacle->y < -4.0);
}

void testLimitedLateralOffsetAvoidance() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    PathPlannerData globalPath = makeStraightGlobalPath();
    WorldState world = makeWorldWithObstacle();
    world.objects.front().raw.width = 5.8F;
    world.objects.front().obb = makeObb2D(35.0, -1.75, 0.0, 4.0, 5.8);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    config.maximumObstacleLaneIntrusion = 1.25;
    config.lateralOffsetStep = 0.2;
    config.maximumLateralOffset = 0.8;
    config.minimumLaneEdgeClearance = 0.0;
    PlanningBehaviorConfig safetyConfig;
    safetyConfig.collisionPredictionHorizon = 5.0;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 10));
    const auto nearObstacle = std::min_element(
        laneChange.path.begin(), laneChange.path.end(),
        [](const Waypoint& left, const Waypoint& right) {
            return std::abs(left.s - 35.0) < std::abs(right.s - 35.0);
        });
    assert(nearObstacle != laneChange.path.end());
    assert(nearObstacle->y < -5.55);

    WorldState impossible = makeWorldWithObstacle();
    impossible.objects.front().raw.width = 7.0F;
    impossible.objects.front().obb =
        makeObb2D(35.0, -1.75, 0.0, 4.0, 7.0);
    LaneChangeData rejected;
    assert(!tryPrepareObstacleLaneChange(
        rejected, config, safetyConfig, impossible, map, globalPath, 10));
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

void testSchoolZoneReferencePointConfig() {
    MapRuntimeConfig config;
    assert(loadMapRuntimeConfig(config, "config/maps/living_lab.yaml"));
    assert(config.schoolZoneReferencePointsPath ==
           "config/maps/living_lab_school_zone_points.yaml");
    assert(config.zones.schoolZoneReferencePoints.size() == 46);
    assert(std::abs(config.zones.schoolZoneReferenceSpeedLimit -
                    30.0 / 3.6) < 1e-9);
    const auto& first = config.zones.schoolZoneReferencePoints.front();
    assert(first.id == "SZP_01");
    assert(std::abs(first.position.x - 359.584) < 1e-9);
    assert(std::abs(first.position.y + 71.227) < 1e-9);
    assert(first.trackId == 420);
    assert(first.laneId == -1);
    assert(std::abs(first.trackS - 0.084) < 1e-9);
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

    // 급정거 선행차 뒤에 Ego가 이미 가까이 정지했어도, 정지 지속시간을
    // 충족했다면 짧은 횡이동 경로를 OBB 검사한 뒤 회피할 수 있다.
    WorldState stoppedBehind = makeWorldWithObstacle();
    stoppedBehind.egoSpeed = 0.0;
    stoppedBehind.objects.front().raw.x = 21.0F;
    stoppedBehind.objects.front().pathS = 21.0;
    stoppedBehind.objects.front().relativeS = 11.0;
    stoppedBehind.objects.front().obb =
        makeObb2D(21.0, -1.75, 0.0, 4.0, 2.0);
    LaneChangeData tooCloseNormally;
    assert(!tryPrepareObstacleLaneChange(
        tooCloseNormally, config, safetyConfig, stoppedBehind,
        map, globalPath, 10));
    const std::vector<int> persistentStoppedIds{7};
    LaneChangeData stoppedLeadAvoidance;
    assert(tryPrepareObstacleLaneChange(
        stoppedLeadAvoidance, config, safetyConfig, stoppedBehind,
        map, globalPath, 10, &persistentStoppedIds));

    // 교차로나 중첩 도로에서 객체의 독립 최근접 차선 매칭이 다른 road를
    // 선택해도, Ego 참조경로 위 장애물은 경로 차선으로 회피해야 한다.
    WorldState mismatchedLane = makeWorldWithObstacle();
    mismatchedLane.objects.front().roadId = 72;
    mismatchedLane.objects.front().sectionIndex = 0;
    mismatchedLane.objects.front().laneId = -3;
    mismatchedLane.objects.front().roadS = 44.0;
    LaneChangeData rematchedAvoidance;
    assert(tryPrepareObstacleLaneChange(
        rematchedAvoidance, config, safetyConfig, mismatchedLane,
        map, globalPath, 10));
    assert(rematchedAvoidance.targetLaneId == -2);
}

void testLaneChangeWhenAdjacentLaneStartsAhead() {
    PathGeneratorData map = makeStraightTwoLaneMap();

    Road approach;
    approach.id = 550;
    approach.length = 20.0;
    Geometry geometry;
    geometry.type = GeometryType::Line;
    geometry.x = -20.0;
    geometry.length = 20.0;
    approach.geometries.push_back(geometry);
    LaneSection approachSection;
    LaneDefinition approachLane;
    approachLane.id = -1;
    approachLane.type = "driving";
    approachLane.widths.push_back(constantWidth(3.5));
    approachSection.lanes.emplace(-1, approachLane);
    approach.sections.push_back(approachSection);
    map.roads.emplace(550, approach);
    map.laneGraph[{550, 0, -1}].push_back({1, 0, -1});

    const auto addTwoLaneRoad = [&](int roadId, double x, double length) {
        Road road;
        road.id = roadId;
        road.length = length;
        Geometry roadGeometry;
        roadGeometry.type = GeometryType::Line;
        roadGeometry.x = x;
        roadGeometry.length = length;
        road.geometries.push_back(roadGeometry);
        LaneSection section;
        for (int laneId : {-1, -2}) {
            LaneDefinition lane;
            lane.id = laneId;
            lane.type = "driving";
            lane.widths.push_back(constantWidth(3.5));
            lane.roadMarks.push_back({0.0, "both", "standard"});
            section.lanes.emplace(laneId, lane);
        }
        road.sections.push_back(section);
        map.roads.emplace(roadId, std::move(road));
    };
    addTwoLaneRoad(2, 20.0, 5.0);
    addTwoLaneRoad(3, 25.0, 75.0);
    for (int laneId : {-1, -2}) {
        map.laneGraph[{1, 0, laneId}].push_back({2, 0, laneId});
        map.laneGraph[{2, 0, laneId}].push_back({3, 0, laneId});
    }

    PathPlannerData globalPath;
    for (int index = 0; index <= 20; ++index) {
        Waypoint point;
        point.x = -20.0 + index;
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = static_cast<double>(index);
        point.roadS = static_cast<double>(index);
        point.roadId = 550;
        point.laneId = -1;
        point.sectionIndex = 0;
        globalPath.waypoints.push_back(point);
    }
    for (int index = 1; index <= 20; ++index) {
        Waypoint point;
        point.x = static_cast<double>(index);
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = 20.0 + index;
        point.roadS = static_cast<double>(index);
        point.roadId = 1;
        point.laneId = -1;
        point.sectionIndex = 0;
        globalPath.waypoints.push_back(point);
    }
    for (int index = 1; index <= 5; ++index) {
        Waypoint point;
        point.x = 20.0 + index;
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = 40.0 + index;
        point.roadS = static_cast<double>(index);
        point.roadId = 2;
        point.laneId = -1;
        point.sectionIndex = 0;
        globalPath.waypoints.push_back(point);
    }
    for (int index = 1; index <= 60; ++index) {
        Waypoint point;
        point.x = 25.0 + index;
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = 45.0 + index;
        point.roadS = static_cast<double>(index);
        point.roadId = 3;
        point.laneId = -1;
        point.sectionIndex = 0;
        globalPath.waypoints.push_back(point);
    }

    WorldState world;
    world.ego.x = -15.0F;
    world.ego.y = -1.75F;
    world.ego.heading = 0.0F;
    world.egoSpeed = 5.0;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 550;
    world.egoLaneId = -1;
    world.egoSectionIndex = 0;
    world.egoRoadS = 5.0;
    world.egoPathS = 5.0;
    world.frontBumperPathS = 8.8;
    world.egoObb = makeObb2D(-13.6, -1.75, 0.0, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 10;
    obstacle.raw.x = 10.0F;
    obstacle.raw.y = -1.75F;
    obstacle.raw.length = 4.0F;
    obstacle.raw.width = 2.0F;
    obstacle.obb = makeObb2D(10.0, -1.75, 0.0, 4.0, 2.0);
    obstacle.laneValid = true;
    obstacle.roadId = 1;
    obstacle.laneId = -1;
    obstacle.sectionIndex = 0;
    obstacle.roadS = 10.0;
    obstacle.pathValid = true;
    obstacle.pathS = 30.0;
    obstacle.pathD = 0.0;
    obstacle.relativeS = 25.0;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    safetyConfig.collisionPredictionHorizon = 5.0;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 5));
    assert(laneChange.phase == LaneChangePhase::Prepare);
    assert(laneChange.routeMode == LaneChangeRouteMode::ReturnToSource);
}

void testO01ObstacleRoadLaneMerge() {
    PathGeneratorData map;
    assert(loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr"));
    PathPlannerData globalPath;
    assert(loadWaypointsCsv(
        globalPath.waypoints,
        "scenario/obstacle/O01_front_obstacle/route.csv"));

    WorldState world;
    world.ego.x = 13.719388F;
    world.ego.y = 34.4014091F;
    world.ego.heading = 0.229751557F;
    world.egoSpeed = 0.0123493145;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 550;
    world.egoLaneId = -1;
    world.egoSectionIndex = 0;
    world.egoRoadS = 22.93;
    world.egoPathS = 62.1160052;
    world.frontBumperPathS = 65.9160052;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 10;
    obstacle.raw.x = 22.9734726F;
    obstacle.raw.y = 36.4065552F;
    obstacle.raw.heading = 0.0F;
    obstacle.raw.speed = 0.0F;
    obstacle.raw.length = 0.63F;
    obstacle.raw.width = 0.58F;
    obstacle.obb = makeObb2D(
        obstacle.raw.x, obstacle.raw.y, obstacle.raw.heading,
        obstacle.raw.length, obstacle.raw.width);
    obstacle.laneValid = true;
    obstacle.roadId = 72;
    obstacle.laneId = -3;
    obstacle.sectionIndex = 0;
    obstacle.roadS = 6.42135802;
    obstacle.pathValid = true;
    obstacle.pathS = 71.60266294;
    obstacle.pathD = -0.04;
    obstacle.relativeS = obstacle.pathS - world.egoPathS;
    obstacle.relativeD = -0.04;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    const std::vector<int> persistentStoppedIds{10};
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 133,
        &persistentStoppedIds));
    assert(laneChange.phase == LaneChangePhase::Prepare);
    assert(laneChange.routeMode == LaneChangeRouteMode::ReturnToSource);
    assert(laneChange.targetLaneId == -2);
}

void testO04RouteSectionComesFromMapRoadS() {
    PathGeneratorData map;
    assert(loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr"));
    PathPlannerData globalPath;
    assert(loadWaypointsCsv(
        globalPath.waypoints,
        "scenario/obstacle/O04_lane_obstacle/route.csv"));

    // Exact O04 state from the first avoidance-trigger frame.  route.csv has
    // no section_index column: its waypoint's zero-initialized section must
    // never overwrite the GT/map match (road 2626, section 7).
    WorldState world;
    world.ego.x = 1109.31445F;
    world.ego.y = 33.0897446F;
    world.ego.heading = 0.657545447F;
    world.egoSpeed = 7.84728318;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 2626;
    world.egoLaneId = 2;
    world.egoSectionIndex = 8;
    world.egoRoadS = 249.63;
    world.egoPathS = 16.4731802;
    world.frontBumperPathS = world.egoPathS + 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 2;
    obstacle.raw.x = 1144.37573F;
    obstacle.raw.y = 60.792263F;
    obstacle.raw.heading = 0.502082288F;
    obstacle.raw.speed = 0.0F;
    obstacle.raw.length = 0.15F;
    obstacle.raw.width = 1.28999996F;
    obstacle.obb = makeObb2D(
        obstacle.raw.x, obstacle.raw.y, obstacle.raw.heading,
        obstacle.raw.length, obstacle.raw.width);
    obstacle.laneValid = true;
    obstacle.roadId = 2626;
    obstacle.laneId = 2;
    obstacle.sectionIndex = 7;
    obstacle.roadS = 204.98;
    obstacle.pathValid = true;
    obstacle.pathS = 61.1588;
    obstacle.pathD = 0.167337;
    obstacle.relativeS = obstacle.pathS - world.egoPathS;
    obstacle.relativeD = 0.1595;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 33));
    assert(laneChange.phase == LaneChangePhase::Prepare);
    assert(laneChange.obstacleId == 2);
}

void testO02RouteSectionComesFromMapRoadS() {
    PathGeneratorData map;
    assert(loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr"));
    PathPlannerData globalPath;
    assert(loadWaypointsCsv(
        globalPath.waypoints,
        "scenario/obstacle/O02_obstacle_ver2/route.csv"));

    WorldState world;
    world.ego.x = 344.229279F;
    world.ego.y = -110.288513F;
    world.ego.heading = 5.29004002F;
    world.egoSpeed = 0.0;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 418;
    world.egoLaneId = 3;
    world.egoSectionIndex = 2;
    world.egoRoadS = 107.0;
    world.egoPathS = 0.0;
    world.frontBumperPathS = 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 5;
    obstacle.raw.x = 358.513824F;
    obstacle.raw.y = -133.437737F;
    obstacle.raw.heading = 0.0F;
    obstacle.raw.speed = 0.0F;
    obstacle.raw.length = 0.6F;
    obstacle.raw.width = 0.6F;
    obstacle.obb = makeObb2D(
        obstacle.raw.x, obstacle.raw.y, obstacle.raw.heading,
        obstacle.raw.length, obstacle.raw.width);
    obstacle.laneValid = true;
    obstacle.roadId = 418;
    obstacle.laneId = 4;
    obstacle.sectionIndex = 1;
    obstacle.roadS = 81.1;
    obstacle.pathValid = true;
    obstacle.pathS = 25.5688;
    obstacle.pathD = -0.0717227;
    obstacle.relativeS = 25.5688;
    obstacle.relativeD = -1.70554;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 0));
    assert(laneChange.phase == LaneChangePhase::Prepare);
    assert(laneChange.obstacleId == 5);
}

void testLaneChangeExecutionBlockingStops() {
    StopManagerData stops;
    stops.requests.push_back(
        {StopReason::Obstacle, true, 20.0, 0.0, 5});
    stops.requests.push_back(
        {StopReason::Obstacle, true, 20.1, 0.0, 6});
    stops.requests.push_back(
        {StopReason::Obstacle, true, 20.2, 0.0, 7});

    // The candidate path was checked against all objects, including every
    // member of a multi-object static obstacle such as O02.
    assert(!hasLaneChangeExecutionBlocker(stops, true));
    assert(hasLaneChangeExecutionBlocker(stops, false));

    StopDecision decision;
    selectStopDecision(decision, stops, 0.0);
    assert(decision.valid);
    assert(decision.reason == StopReason::Obstacle);
    selectStopDecision(decision, stops, 0.0, true);
    assert(!decision.valid);

    // A safe obstacle-avoidance path never overrides traffic-rule stops.
    stops.requests.push_back(
        {StopReason::RedLight, true, 30.0, 0.0, 11});
    assert(hasLaneChangeExecutionBlocker(stops, true));
    selectStopDecision(decision, stops, 0.0, true);
    assert(decision.valid);
    assert(decision.reason == StopReason::RedLight);
    stops.requests.back().active = false;
    stops.requests.push_back(
        {StopReason::CrosswalkBlocked, true, 31.0, 0.0, 12});
    assert(hasLaneChangeExecutionBlocker(stops, true));
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

void testSafeAvoidanceIgnoresStaticSiblingObstacles() {
    PlanningBehaviorConfig config;
    WorldState world;
    world.egoSpeed = 5.0;
    WorldObjectState staticSibling;
    staticSibling.raw.id = 6;
    staticSibling.raw.speed = 0.0F;
    staticSibling.isAhead = true;
    staticSibling.isOnEgoPath = true;
    staticSibling.relativeS = 10.0;
    staticSibling.ttc = 1.0;
    world.objects.push_back(staticSibling);

    BehaviorState state;
    state.staticObstacleAvoidanceActive = true;
    BehaviorDecision behavior;
    ConstraintSet constraints;
    constraints.speedLimit = 12.5;
    StopDecision stop;
    updateBehavior(state, behavior, constraints, stop, world,
                   12.5, config, 0.05);
    assert(behavior.longitudinal == LonMode::Cruise);
    assert(std::abs(behavior.targetSpeed - 12.5) < 1e-9);

    PathSafetyResult safety;
    resetPathSafety(safety);
    updateMinimumTTC(safety, world.objects, true);
    assert(std::isinf(safety.minimumTTC));

    // 움직이는 차량은 회피 중에도 추종/TTC 대상에서 제외하지 않는다.
    world.objects.front().raw.speed = 1.0F;
    state.safelyAvoidedObjectId = 6;
    updateBehavior(state, behavior, constraints, stop, world,
                   12.5, config, 0.05);
    assert(behavior.longitudinal == LonMode::Cruise);
    resetPathSafety(safety);
    updateMinimumTTC(safety, world.objects, true, 6);
    assert(std::isinf(safety.minimumTTC));

    // 같은 ID가 움직여도 후보 경로가 안전할 때는 기동을 유지하지만,
    // OBB 재검사에서 위험해지면 예외가 풀려 Follow/TTC가 즉시 복귀한다.
    state.staticObstacleAvoidanceActive = false;
    state.safelyAvoidedObjectId = -1;
    updateBehavior(state, behavior, constraints, stop, world,
                   12.5, config, 0.05);
    assert(behavior.longitudinal == LonMode::Follow);
    resetPathSafety(safety);
    updateMinimumTTC(safety, world.objects, false);
    assert(std::abs(safety.minimumTTC - 1.0) < 1e-9);
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
    testSchoolZoneReferencePointConfig();
    testPredictedCollisionIsNotCurrentCollision();
    testObstacleLaneChange();
    testLaneChangeWhenAdjacentLaneStartsAhead();
    testO01ObstacleRoadLaneMerge();
    testO02RouteSectionComesFromMapRoadS();
    testO04RouteSectionComesFromMapRoadS();
    testLaneChangeExecutionBlockingStops();
    testLaneChangeAcrossRenumberedLaneSection();
    testLimitedLateralOffsetAvoidance();
    testKeepTargetLaneWhenItRejoinsNextRoad();
    testNearbyAdjacentVehicleCapsSpeedBeforeCutIn();
    testSafeAvoidanceIgnoresStaticSiblingObstacles();
    testSharpCurveHasSeparateSpeed();
    std::cout << "controller_logic_tests: PASS\n";
    return 0;
}
