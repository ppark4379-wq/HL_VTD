#include "data/map/static_zones.hpp"
#include "data/map/map_matcher.hpp"
#include "data/gt/gt_processor.hpp"
#include "control/control_process.hpp"
#include "network/receive/rdb_clock.hpp"
#include "network/send/control_sender.hpp"
#include "planning/behavior/behavior_planner.hpp"
#include "planning/collision/collision_checker.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/global/mission_corridor.hpp"
#include "planning/planning_process.hpp"
#include "planning/local/lane_change_planner.hpp"
#include "planning/local/path_planner.hpp"
#include "utils/geometry.hpp"
#include "utils/math_utils.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

namespace {

static_assert(CONTROL_PACKET_SIZE == 9,
              "VTD control command must be exactly 9 bytes");

void testRecoveryDoesNotCommandAStop() {
    BehaviorState state;
    state.health = SystemHealth::Recovery;
    BehaviorDecision output;
    ConstraintSet constraints;
    constraints.speedLimit = 45.0 / 3.6;
    StopDecision stop;
    WorldState world;
    world.egoSpeed = 3.0;
    PlanningBehaviorConfig config;
    updateBehavior(state, output, constraints, stop, world,
                   30.0 / 3.6, config, 0.05);
    if (output.health == SystemHealth::Recovery ||
        output.targetSpeed <= 0.0 || output.longitudinal == LonMode::Stop) {
        std::cerr << "Recovery still commanded a non-emergency stop\n";
        std::abort();
    }
}

void testEmergencyBrakingLatchesEntrySteering() {
    ControlProcessData control;
    control.controller.config.maxSteering = 0.48;
    control.controller.config.maxSteeringRate = 10.0;
    WorldState world;
    world.inputValid = true;
    PlanningResult planning;
    planning.valid = true;
    planning.targetSpeed = 5.0;
    planning.trackingPlan.valid = true;
    planning.trackingPlan.steering = 0.08;
    runControlProcess(control, world, planning, 0.05);
    const float beforeEmergency = control.latestCommand.steering;

    planning.emergencyBrake = true;
    planning.targetSpeed = 0.0;
    planning.trackingPlan.steering = -0.48;
    runControlProcess(control, world, planning, 0.05);
    if (std::abs(control.latestCommand.steering - beforeEmergency) > 1e-6 ||
        !control.safety.emergencyBrakeApplied) {
        std::cerr << "EBS changed steering after brake entry\n";
        std::abort();
    }

    planning.trackingPlan.steering = 0.48;
    runControlProcess(control, world, planning, 0.05);
    if (std::abs(control.latestCommand.steering - beforeEmergency) > 1e-6) {
        std::cerr << "EBS steering latch changed while braking\n";
        std::abort();
    }
}

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

template <typename T>
void writeNative(std::vector<std::uint8_t>& bytes,
                 std::size_t offset,
                 const T& value) {
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

std::vector<std::uint8_t> makeRdbClockMessage(std::uint32_t frameNo,
                                              double simulationTime,
                                              double egoX,
                                              double egoY) {
    constexpr std::size_t headerSize = 24;
    constexpr std::size_t entryHeaderSize = 16;
    constexpr std::size_t objectSize = 112;
    std::vector<std::uint8_t> message(
        headerSize + entryHeaderSize + objectSize, 0);
    writeNative(message, 0, std::uint16_t{35712});
    writeNative(message, 2, std::uint16_t{0x011A});
    writeNative(message, 4, std::uint32_t{headerSize});
    writeNative(message, 8,
                std::uint32_t{entryHeaderSize + objectSize});
    writeNative(message, 12, frameNo);
    writeNative(message, 16, simulationTime);

    writeNative(message, headerSize, std::uint32_t{entryHeaderSize});
    writeNative(message, headerSize + 4, std::uint32_t{objectSize});
    writeNative(message, headerSize + 8, std::uint32_t{objectSize});
    writeNative(message, headerSize + 12, std::uint16_t{9});
    const std::size_t objectOffset = headerSize + entryHeaderSize;
    std::memcpy(message.data() + objectOffset + 8, "Ego", 4);
    writeNative(message, objectOffset + 64, egoX);
    writeNative(message, objectOffset + 72, egoY);
    return message;
}

void testRdbSimulationTimeMatchesGtInFrameOrder() {
    RdbClockData clock;
    const auto first = makeRdbClockMessage(20, 1.0, 10.0, 20.0);
    const auto second = makeRdbClockMessage(21, 1.05, 10.0, 20.0);
    clock.streamBuffer.insert(clock.streamBuffer.end(),
                              first.begin(), first.end());
    clock.streamBuffer.insert(clock.streamBuffer.end(),
                              second.begin(), second.end());
    extractRdbClockSamples(clock);
    assert(clock.samples.size() == 2);

    EgoState ego;
    ego.x = 10.0F;
    ego.y = 20.0F;
    const auto firstMatch = matchRdbSimulationTime(clock, ego);
    const auto secondMatch = matchRdbSimulationTime(clock, ego);
    assert(firstMatch.has_value() &&
           std::abs(*firstMatch - 1.0) < 1e-12);
    assert(secondMatch.has_value() &&
           std::abs(*secondMatch - 1.05) < 1e-12);
    assert(clock.samples.empty());

    // 새 RDB 샘플 없이 같은 GT가 반복되면 같은 simTime을 재사용한다.
    const auto duplicateMatch = matchRdbSimulationTime(clock, ego);
    assert(duplicateMatch.has_value() &&
           std::abs(*duplicateMatch - 1.05) < 1e-12);
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

PathGeneratorData makeStraightThreeLaneMap() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    Road& road = map.roads.at(1);
    LaneDefinition lane = road.sections.front().lanes.at(-2);
    lane.id = -3;
    road.sections.front().lanes.emplace(-3, lane);
    map.laneGraph[{1, 0, -2}].push_back({1, 0, -3});
    map.laneGraph[{1, 0, -3}].push_back({1, 0, -2});

    LaneSegment segment;
    segment.key = {1, 0, -3};
    segment.startS = 0.0;
    segment.endS = 100.0;
    for (int index = 0; index <= 100; ++index) {
        Waypoint point;
        point.x = static_cast<double>(index);
        point.y = -8.75;
        point.yaw = 0.0;
        point.roadS = static_cast<double>(index);
        point.roadId = 1;
        point.laneId = -3;
        point.sectionIndex = 0;
        segment.points.push_back(point);
    }
    map.laneSegments.emplace(segment.key, std::move(segment));
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
    StopLine farLine;
    farLine.id = 9998;
    farLine.begin = {20.0, 100.0};
    farLine.end = {24.0, 100.0};
    zones.stopLines.push_back(farLine);
    Crosswalk farCrosswalk;
    farCrosswalk.id = 9999;
    farCrosswalk.polygon = {{30.0, 100.0}, {34.0, 100.0},
                            {34.0, 104.0}, {30.0, 104.0}};
    zones.crosswalks.push_back(farCrosswalk);
    const auto path = makeStraightGlobalPath().waypoints;
    if (!alignStaticZonesToRoute(zones, path)) {
        std::cerr << "failed to align static zones\n";
        std::abort();
    }
    if (zones.stopLines.size() != 2 || zones.crosswalks.size() != 2 ||
        !zones.stopLines.front().routeRelevant ||
        zones.stopLines.back().routeRelevant ||
        !zones.crosswalks.front().routeRelevant ||
        zones.crosswalks.back().routeRelevant) {
        std::cerr << "route alignment deleted or activated wrong static data\n";
        std::abort();
    }
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

void testScenarioRoutesKeepTheirOwnCrosswalksAndSchoolZone() {
    struct Case {
        const char* routePath;
        int expectedCrosswalkId;
        bool expectsSchoolZone;
    };
    for (const Case testCase : {
             Case{"scenario/mixed/M03/route.csv", 6289, false},
             Case{"scenario/mixed/M04/route.csv", 1090, true},
             // M09는 교차로 sibling connector 1784를 타지만
             // 물리적으로는 road 1781의 횟단보도 1692를 관통한다.
             Case{"scenario/mixed/M09/route.csv", 1692, true}}) {
        MapRuntimeConfig config;
        std::vector<Waypoint> route;
        if (!loadMapRuntimeConfig(config, "config/maps/living_lab.yaml") ||
            !loadWaypointsCsv(route, testCase.routePath)) {
            std::cerr << "failed to align static zones for "
                      << testCase.routePath << '\n';
            std::abort();
        }
        const std::size_t loadedStopLineCount =
            config.zones.stopLines.size();
        const std::size_t loadedCrosswalkCount =
            config.zones.crosswalks.size();
        if (!alignStaticZonesToRoute(config.zones, route) ||
            config.zones.stopLines.size() != loadedStopLineCount ||
            config.zones.crosswalks.size() != loadedCrosswalkCount) {
            std::cerr << "route alignment removed loaded static data for "
                      << testCase.routePath << '\n';
            std::abort();
        }
        const bool hasExpectedCrosswalk = std::any_of(
            config.zones.crosswalks.begin(), config.zones.crosswalks.end(),
            [&](const Crosswalk& crosswalk) {
                return crosswalk.id == testCase.expectedCrosswalkId &&
                       crosswalk.routeRelevant;
            });
        const bool hasSchoolZone = std::any_of(
            config.zones.speedZones.begin(), config.zones.speedZones.end(),
            [](const SpeedZone& zone) {
                return zone.schoolZone && zone.routeRelevant;
            });
        if (!hasExpectedCrosswalk ||
            hasSchoolZone != testCase.expectsSchoolZone) {
            std::cerr << "route-specific static zone regression for "
                      << testCase.routePath << '\n';
            std::abort();
        }
    }
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

void testConvergingOncomingVehicleIsPredictedOutsideNarrowCorridor() {
    WorldState world;
    world.inputValid = true;
    world.ego.x = 332.961761F;
    world.ego.y = -70.869301F;
    world.ego.heading = 5.508448F;
    world.egoSpeed = 7.650285;
    world.egoPathValid = true;
    world.egoPathS = 0.0;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    WorldObjectState object;
    object.raw.id = 12;
    object.raw.x = 357.352661F;
    object.raw.y = -100.225708F;
    object.raw.heading = 2.150659F;
    object.raw.speed = 18.055555F;
    object.raw.length = 4.394F;
    object.raw.width = 1.808F;
    object.pathValid = true;
    object.isAhead = true;
    object.relativeS = 37.965;
    object.relativeD = -3.916;  // 기존 2 m 맞은편 회랑 밖
    object.obb = makeObb2D(
        object.raw.x, object.raw.y, object.raw.heading,
        object.raw.length, object.raw.width);
    world.objects.push_back(object);

    LocalPathData path;
    path.valid = true;
    for (int index = 0; index <= 50; ++index) {
        const double distance = static_cast<double>(index);
        Waypoint point;
        point.x = world.ego.x + distance * std::cos(world.ego.heading);
        point.y = world.ego.y + distance * std::sin(world.ego.heading);
        point.yaw = world.ego.heading;
        point.s = distance;
        path.points.push_back(point);
    }

    PathSafetyResult safety;
    resetPathSafety(safety);
    checkPredictedCollisions(safety, world, path, 3.0, 0.1);
    if (!safety.predictedCollision || safety.riskObjectId != 12) {
        std::cerr << "converging oncoming vehicle was filtered out\n";
        std::abort();
    }

    // 같은 속도/방향이어도 정상 맞은편 차선 간격을 유지하면 예측 후보가
    // 되지 않아 곡선도로 오제동 방지 필터를 보존한다.
    world.ego.x = 0.0F;
    world.ego.y = 0.0F;
    world.ego.heading = 0.0F;
    world.egoSpeed = 8.0;
    world.egoObb = makeObb2D(0.0, 0.0, 0.0, 4.8, 1.9);
    world.objects.front().raw.x = 30.0F;
    world.objects.front().raw.y = 3.5F;
    world.objects.front().raw.heading = static_cast<float>(M_PI);
    world.objects.front().raw.speed = 15.0F;
    world.objects.front().relativeD = 3.5;
    world.objects.front().obb = makeObb2D(
        30.0, 3.5, M_PI, 4.394, 1.808);
    path.points.clear();
    for (int index = 0; index <= 50; ++index) {
        Waypoint point;
        point.x = static_cast<double>(index);
        point.y = 0.0;
        point.yaw = 0.0;
        point.s = static_cast<double>(index);
        path.points.push_back(point);
    }
    resetPathSafety(safety);
    checkPredictedCollisions(safety, world, path, 3.0, 0.1);
    if (safety.predictedCollision) {
        std::cerr << "separated oncoming lane caused false collision\n";
        std::abort();
    }
}

void testBlockedReturnUsesWaitingSpeedUntilCollisionIsUrgent() {
    PathSafetyResult safety;
    resetPathSafety(safety);
    safety.predictedCollision = true;
    safety.predictedCollisionTTC = 3.0;
    safety.minimumTTC = 3.0;

    // 복귀 경로 멀리의 점유는 복귀 보류/감속 사유이지만,
    // 즉시 비상제동 사유는 아니다.
    assert(!requiresEmergencyBrake(safety, 1.5));

    safety.predictedCollisionTTC = 1.4;
    safety.minimumTTC = 1.4;
    assert(requiresEmergencyBrake(safety, 1.5));

    safety.predictedCollision = false;
    safety.predictedCollisionTTC =
        std::numeric_limits<double>::infinity();
    safety.minimumTTC = std::numeric_limits<double>::infinity();
    safety.collision = true;
    assert(requiresEmergencyBrake(safety, 1.5));
}

void testMovingReturnConflictCanClearAtWaitingSpeed() {
    WorldState world = makeWorldWithObstacle();
    WorldObjectState& object = world.objects.front();
    object.raw.x = 20.0F;
    object.raw.speed = 4.0F;
    object.raw.heading = 0.0F;
    object.pathS = 20.0;
    object.relativeS = 10.0;
    object.relativeD = 0.0;
    object.isAhead = true;
    object.isOnEgoPath = true;
    object.obb = makeObb2D(20.0, -1.75, 0.0, 4.0, 2.0);

    LocalPathData path;
    path.valid = true;
    path.points = makeStraightGlobalPath().waypoints;

    PathSafetyResult fastSafety;
    resetPathSafety(fastSafety);
    world.egoSpeed = 6.0;
    checkPredictedCollisions(fastSafety, world, path, 5.0, 0.1);
    assert(fastSafety.predictedCollision);

    // 같은 경로도 2 m/s로 감속하면 4 m/s 앞차가 먼저
    // 멀어진다. 이 경우는 멈추지 말고 느린 복귀가 안전하다.
    PathSafetyResult waitingSafety;
    resetPathSafety(waitingSafety);
    world.egoSpeed = 2.0;
    checkPredictedCollisions(waitingSafety, world, path, 5.0, 0.1);
    assert(!waitingSafety.predictedCollision);
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
    assert(laneChange.routeMode == LaneChangeRouteMode::KeepTargetLane);
    assert(!laneChange.reconnectLaneValid);
    assert(laneChange.path.back().laneId == -2);
    assert(laneChange.turnSignal != 0);
    const auto nearObstacle = std::min_element(
        laneChange.path.begin(), laneChange.path.end(),
        [](const Waypoint& left, const Waypoint& right) {
            return std::abs(left.s - 35.0) < std::abs(right.s - 35.0);
        });
    assert(nearObstacle != laneChange.path.end());
    assert(nearObstacle->y < -4.0);
    advanceLaneChangePhase(laneChange, config, world,
                           config.prepareTime, false, true, true);
    assert(laneChange.phase == LaneChangePhase::Execute);
    advanceLaneChangePhase(laneChange, config, world, 0.1,
                           false, true, false);
    assert(!laneChange.waitingForReturn);

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

    // 목표 차선의 더 먼 장애물은 첫 회피를 폐기시키지 않는다.
    // 현재 장애물보다 여유가 크면 먼저 안착한 뒤 새 차선에서
    // 후속 장애물을 다시 계획한다.
    targetVehicle.raw.x = 45.0F;
    targetVehicle.pathS = 45.0;
    targetVehicle.obb = makeObb2D(45.0, -5.25, 0.0, 4.0, 2.0);
    WorldState farBlockedWorld = makeWorldWithObstacle();
    farBlockedWorld.objects.push_back(targetVehicle);
    LaneChangeData farBlocked;
    if (!tryPrepareObstacleLaneChange(
            farBlocked, config, safetyConfig, farBlockedWorld,
            map, globalPath, 10) ||
        farBlocked.routeMode != LaneChangeRouteMode::KeepTargetLane ||
        farBlocked.rejoinRouteS >= targetVehicle.pathS) {
        std::cerr << "far target obstacle incorrectly rejected first avoidance\n";
        std::abort();
    }

    // 정지 이력을 기다리지 않고 현재 프레임에 정적인 객체면 즉시
    // 짧은 횡이동 경로를 OBB 검사한 뒤 회피한다.
    WorldState stoppedBehind = makeWorldWithObstacle();
    stoppedBehind.egoSpeed = 0.0;
    stoppedBehind.objects.front().raw.x = 29.0F;
    stoppedBehind.objects.front().pathS = 29.0;
    stoppedBehind.objects.front().relativeS = 19.0;
    stoppedBehind.objects.front().obb =
        makeObb2D(29.0, -1.75, 0.0, 4.0, 2.0);
    LaneChangeData immediateAvoidance;
    assert(tryPrepareObstacleLaneChange(
        immediateAvoidance, config, safetyConfig, stoppedBehind,
        map, globalPath, 10));

    // 이전에 움직이던 끼어들기 차량은 정지 확인 전에는 회피
    // 기동을 시작하지 않고, 확인된 ID가 되면 같은 경로를 생성한다.
    const std::vector<int> noConfirmedObjects;
    LaneChangeData unconfirmedStoppedVehicle;
    assert(!tryPrepareObstacleLaneChange(
        unconfirmedStoppedVehicle, config, safetyConfig, stoppedBehind,
        map, globalPath, 10, nullptr, &noConfirmedObjects));
    const std::vector<int> confirmedObjects{7};
    LaneChangeData confirmedStoppedVehicle;
    assert(tryPrepareObstacleLaneChange(
        confirmedStoppedVehicle, config, safetyConfig, stoppedBehind,
        map, globalPath, 10, nullptr, &confirmedObjects));

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

void testBlockedAdjacentLanesPreferFartherObstacle() {
    PathGeneratorData map = makeStraightThreeLaneMap();
    PathPlannerData globalPath = makeStraightGlobalPath();
    WorldState world = makeWorldWithObstacle();

    const auto addLaneBlocker = [&](int id, int laneId, double x,
                                    double y) {
        WorldObjectState object;
        object.raw.id = id;
        object.raw.x = static_cast<float>(x);
        object.raw.y = static_cast<float>(y);
        object.raw.length = 4.0F;
        object.raw.width = 2.0F;
        object.obb = makeObb2D(x, y, 0.0, 4.0, 2.0);
        object.laneValid = true;
        object.roadId = 1;
        object.laneId = laneId;
        object.sectionIndex = 0;
        object.pathValid = true;
        object.pathS = x;
        object.relativeS = x - world.egoPathS;
        object.isAhead = true;
        world.objects.push_back(object);
    };
    // 접근 가능한 두 회피 차선이 모두 막혀 있지만
    // 2회 횡이동인 -3 차선의 장애물이 더 멀리 있다.
    // 단순히 가까운 차선(-2)을 고르지 않고 전방 여유가
    // 큰 차선을 선택해 후속 회피 공간을 확보해야 한다.
    addLaneBlocker(8, -2, 42.0, -5.25);
    addLaneBlocker(9, -3, 55.0, -8.75);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath, 10) ||
        laneChange.targetLaneId != -3 ||
        laneChange.routeMode != LaneChangeRouteMode::KeepTargetLane) {
        std::cerr << "all-blocked lane ranking did not choose most clearance\n";
        std::abort();
    }
}

void testSolidLineForbidsObstacleLaneChange() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    Road& road = map.roads.at(1);
    road.sections[0].lanes.at(-1).roadMarks = {
        {0.0, "none", "standard"}};

    const PathPlannerData globalPath = makeStraightGlobalPath();
    const WorldState world = makeWorldWithObstacle();
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    assert(!tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 10));
    assert(laneChange.phase == LaneChangePhase::None);

    // 시작점이 점선이어도 실제 경계를 넘는 위치가 실선이면 금지한다.
    road.sections[0].lanes.at(-1).roadMarks = {
        {0.0, "both", "standard"},
        {15.0, "none", "standard"}};
    LaneChangeData crossingOnSolid;
    assert(!tryPrepareObstacleLaneChange(
        crossingOnSolid, config, safetyConfig, world, map, globalPath, 10));
}

void testAvoidanceLaneMustConnectToNextRoad() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    const PathPlannerData globalPath = makeGlobalPathWithNextRoad(map);
    map.laneGraph[{1, 0, -2}].clear();

    const WorldState world = makeWorldWithObstacle();
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath, 10)) {
        std::cerr << "obstacle avoidance was rejected only because target lane "
                     "does not connect to the next road\n";
        std::abort();
    }
    if (laneChange.routeMode != LaneChangeRouteMode::KeepTargetLane ||
        laneChange.reconnectLaneValid) {
        std::cerr << "obstacle avoidance did not keep the free current-road lane\n";
        std::abort();
    }
}

void testRoadIdTrackingKeepsCurrentLaneInsideRoad() {
    const PathGeneratorData map = makeStraightTwoLaneMap();
    const PathPlannerData globalPath = makeStraightGlobalPath();
    WorldState world = makeWorldWithObstacle();
    world.objects.clear();
    world.ego.y = -5.25F;
    world.egoLaneId = -2;
    world.egoPathD = -3.5;

    PathPlannerData selected;
    if (!buildRoadIdTrackingPath(
            selected, globalPath.config, world, map, globalPath, 10, 80.0)) {
        std::cerr << "road-ID tracking path was not generated\n";
        std::abort();
    }
    for (const Waypoint& point : selected.waypoints) {
        if (point.roadId == 1 && std::abs(point.y + 5.25) > 1e-6) {
            std::cerr << "road-ID tracking pulled the ego back to global lane\n";
            std::abort();
        }
    }
}

void testStaticObstacleFollowNeverCommandsZero() {
    WorldState world = makeWorldWithObstacle();
    world.egoSpeed = 5.0;
    world.objects.front().relativeS = 5.5;
    world.objects.front().raw.speed = 0.0F;
    BehaviorState state;
    BehaviorDecision behavior;
    ConstraintSet constraints;
    constraints.speedLimit = 45.0 / 3.6;
    StopDecision noLegalStop;
    PlanningBehaviorConfig config;
    config.minimumObjectResponseSpeed = 15.0 / 3.6;
    updateBehavior(state, behavior, constraints, noLegalStop, world,
                   45.0 / 3.6, config, 0.05);
    if (behavior.targetSpeed + 1e-9 < config.minimumObjectResponseSpeed) {
        std::cerr << "static obstacle ACC commanded below 15 km/h\n";
        std::abort();
    }

    // 서행 차량이 느려도 급정지/끼어들기 비상제동은 외부
    // safety 단계에서 처리하고, 일반 ACC 목표는 15 km/h를 지킨다.
    world.objects.front().raw.speed = 2.0F;
    updateBehavior(state, behavior, constraints, noLegalStop, world,
                   45.0 / 3.6, config, 0.05);
    if (behavior.targetSpeed + 1e-9 < config.minimumObjectResponseSpeed) {
        std::cerr << "dynamic object ACC commanded below 15 km/h\n";
        std::abort();
    }

    // 적신호/횡단보도 정지는 정적 장애물 최저속도의 예외다.
    StopDecision redLight;
    redLight.valid = true;
    redLight.reason = StopReason::RedLight;
    redLight.distanceToStop = 0.0;
    updateBehavior(state, behavior, constraints, redLight, world,
                   45.0 / 3.6, config, 0.05);
    if (behavior.targetSpeed != 0.0) {
        std::cerr << "legal stop was incorrectly overridden by static floor\n";
        std::abort();
    }
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
    obstacle.raw.x = 19.0F;
    obstacle.raw.y = -1.75F;
    obstacle.raw.length = 4.0F;
    obstacle.raw.width = 2.0F;
    obstacle.obb = makeObb2D(19.0, -1.75, 0.0, 4.0, 2.0);
    obstacle.laneValid = true;
    obstacle.roadId = 1;
    obstacle.laneId = -1;
    obstacle.sectionIndex = 0;
    obstacle.roadS = 19.0;
    obstacle.pathValid = true;
    obstacle.pathS = 39.0;
    obstacle.pathD = 0.0;
    obstacle.relativeS = 34.0;
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
    obstacle.raw.x = 35.957691F;
    obstacle.raw.y = 40.611736F;
    obstacle.raw.heading = 0.509310F;
    obstacle.raw.speed = 0.0F;
    obstacle.raw.length = 0.63F;
    obstacle.raw.width = 0.58F;
    obstacle.obb = makeObb2D(obstacle.raw.x, obstacle.raw.y,
                             obstacle.raw.heading,
                             obstacle.raw.length, obstacle.raw.width);
    obstacle.laneValid = true;
    obstacle.roadId = 72;
    obstacle.laneId = -3;
    obstacle.sectionIndex = 0;
    obstacle.roadS = 19.26407407;
    obstacle.pathValid = true;
    obstacle.pathS = 85.29596487;
    obstacle.pathD = -0.04;
    obstacle.relativeS = obstacle.pathS - world.egoPathS;
    obstacle.relativeD = -0.04;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    assert(tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 133));
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
    // 다음 도로의 연결 차선으로 복귀하려면 실선을 넘어야 하는 배치다.
    // section rematch가 맞더라도 불법 후보를 만들면 안 된다.
    assert(!tryPrepareObstacleLaneChange(
        laneChange, config, safetyConfig, world, map, globalPath, 0));
    assert(laneChange.phase == LaneChangePhase::None);
}

void testLaneChangeExecutionBlockedByTrafficStops() {
    StopManagerData stops;
    StopDecision decision;
    selectStopDecision(decision, stops, 0.0);
    assert(!decision.valid);

    // 장애물 정지 요청은 만들지 않지만 교통법규 정지는 그대로 지킨다.
    stops.requests.push_back(
        {StopReason::RedLight, true, 30.0, 0.0, 11});
    assert(hasLaneChangeExecutionBlocker(stops));
    assert(!hasLaneChangeExecutionBlocker(stops, 20.0));
    assert(hasLaneChangeExecutionBlocker(stops, 29.0));
    selectStopDecision(decision, stops, 0.0);
    assert(decision.valid);
    assert(decision.reason == StopReason::RedLight);
    stops.requests.back().active = false;
    stops.requests.push_back(
        {StopReason::CrosswalkBlocked, true, 31.0, 0.0, 12});
    assert(hasLaneChangeExecutionBlocker(stops));

    // 정지선을 침범하는 경로는 녹색불까지 보관하지 않는다.
    // 정지 요청이 풀린 현재 위치에서 다시 생성해야 한다.
    LaneChangeData prepared;
    prepared.phase = LaneChangePhase::Prepare;
    prepared.obstacleId = 12;
    prepared.path = makeStraightGlobalPath().waypoints;
    LaneChangeConfig laneChangeConfig;
    WorldState world;
    advanceLaneChangePhase(prepared, laneChangeConfig, world, 0.1,
                           true, true, true);
    assert(prepared.phase == LaneChangePhase::None);
    assert(prepared.path.empty());

    // gateway는 정지 중 버렸다가 프레임마다 다시 고르면 분기 선택이
    // 흔들린다. 같은 경로를 보관하고 정지가 풀린 뒤 실행해야 한다.
    prepared.phase = LaneChangePhase::Prepare;
    prepared.roadConnectionManeuver = true;
    prepared.path = makeStraightGlobalPath().waypoints;
    advanceLaneChangePhase(prepared, laneChangeConfig, world, 0.1,
                           true, true, true);
    assert(prepared.phase == LaneChangePhase::Prepare);
    assert(!prepared.path.empty());
}

void testActivePathProjectionDoesNotRequireGuideRoute() {
    WorldState world;
    world.ego.x = 50.0F;
    world.ego.y = 20.0F;
    world.ego.heading = 0.0F;
    world.egoSpeed = 5.0;
    world.egoLaneWidth = 3.5;
    world.egoLaneValid = true;
    world.inputValid = true;
    world.egoPathValid = false;  // guide route 투영은 의도적으로 실패

    WorldObjectState object;
    object.raw.id = 7;
    object.raw.x = 60.0F;
    object.raw.y = 20.0F;
    object.raw.heading = 0.0F;
    object.raw.length = 4.0F;
    object.raw.width = 2.0F;
    object.obb = makeObjectObb(object.raw);
    world.objects.push_back(object);

    std::vector<Waypoint> activePath;
    for (int x = 40; x <= 90; ++x) {
        Waypoint point;
        point.x = static_cast<double>(x);
        point.y = 20.0;
        point.yaw = 0.0;
        point.s = 100.0 + static_cast<double>(x - 40);
        activePath.push_back(point);
    }
    if (!projectWorldToActivePath(world, activePath, 3.8) ||
        !world.egoPathValid || std::abs(world.egoPathS - 110.0) > 0.1 ||
        !world.objects.front().isAhead ||
        !world.objects.front().isOnEgoPath ||
        std::abs(world.objects.front().relativeS - 10.0) > 0.1) {
        std::cerr << "active path still depended on guide route projection\n";
        std::abort();
    }
}

void testPreviousLocalPathBridgesOneFrameSectionRebuildFailure() {
    PlanningProcessData planning;
    planning.pathPlanner.config = {};
    planning.roadLanePlanner.config = planning.pathPlanner.config;
    planning.config = {};
    planning.laneChangeConfig = {};

    // 직전 프레임에 정상적으로 생성되어 아직 30 m 이상 남은
    // local path. 현재 프레임의 map에서는 의도적으로 새 경로를
    // 만들 수 없게 해 section 경계의 한 프레임 생성 실패를 재현한다.
    for (int x = 0; x <= 50; ++x) {
        Waypoint point;
        point.x = static_cast<double>(x);
        point.y = -1.75;
        point.yaw = 0.0;
        point.s = static_cast<double>(x);
        point.roadS = static_cast<double>(x);
        point.roadId = 10;
        point.sectionIndex = x < 20 ? 0 : 1;
        point.laneId = -1;
        planning.roadLanePlanner.waypoints.push_back(point);
    }

    planning.missionCorridor.valid = true;
    planning.missionCorridor.minimumTargetDistance =
        std::numeric_limits<double>::infinity();
    for (int index = 0; index < 3; ++index) {
        MissionCheckpointData checkpoint;
        checkpoint.checkpoint.sequence = index + 1;
        checkpoint.checkpoint.x = 1000.0 + index * 100.0;
        checkpoint.checkpoint.y = 1000.0;
        checkpoint.roadIds.push_back(100 + index);
        planning.missionCorridor.checkpoints.push_back(std::move(checkpoint));
    }
    MissionLegData leg;
    leg.fromCheckpoint = 0;
    leg.gatewayCheckpoint = 1;
    leg.toCheckpoint = 2;
    leg.valid = true;
    planning.missionCorridor.legs.push_back(leg);

    GtFrame gt;
    gt.ego.x = 20.2F;
    gt.ego.y = -1.75F;
    gt.ego.heading = 0.0F;
    WorldState world;
    world.ego = gt.ego;
    world.egoSpeed = 5.0;
    world.inputValid = true;
    world.egoLaneValid = true;
    world.egoRoadId = 10;
    world.egoSectionIndex = 1;
    world.egoLaneId = -1;
    world.egoLaneWidth = 3.5;
    world.frontBumperOffset = 3.8;
    world.egoObb = makeObb2D(20.2, -1.75, 0.0, 4.8, 1.9);

    PathGeneratorData temporarilyUnavailableMap;
    runPlanningProcess(planning, gt, world, temporarilyUnavailableMap, 0.05);
    if (!planning.result.valid ||
        planning.result.health == SystemHealth::Recovery ||
        planning.result.targetSpeed <= 0.0 ||
        planning.roadLanePlanner.waypoints.size() != 51) {
        std::cerr << "one-frame section rebuild failure discarded the valid "
                     "previous local path\n";
        std::abort();
    }

    // 반대로 정상 경로에서 크게 이탈한 경우는 이전 경로를 붙잡지
    // 않되, Recovery 상태 자체가 정지를 추가로 유지해서도 안 된다.
    gt.ego.y = 20.0F;
    world.ego = gt.ego;
    world.egoObb = makeObb2D(20.2, 20.0, 0.0, 4.8, 1.9);
    runPlanningProcess(planning, gt, world, temporarilyUnavailableMap, 0.05);
    if (planning.result.valid ||
        planning.result.health == SystemHealth::Recovery) {
        std::cerr << "large local-path departure retained a stale path or "
                     "latched Recovery\n";
        std::abort();
    }
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
                           config.prepareTime, false, true, true);
    world.egoRoadId = 2;
    world.egoSectionIndex = 0;
    world.egoLaneId = -1;
    advanceLaneChangePhase(laneChange, config, world, 0.1,
                           false, true, true);
    assert(laneChange.phase == LaneChangePhase::Settle);
}

void testKeepTargetReconnectSmoothsOffsetRoute() {
    PathGeneratorData map = makeStraightTwoLaneMap();
    PathPlannerData globalPath = makeGlobalPathWithNextRoad(map);

    // M05에서 나타난 경계 오차를 단순화한 회귀 테스트.
    // map lane centerline과 checkpoint로 만든 전역경로가 2.8 m
    // 엇갈리더라도 한 점에서 직각으로 붙이지 않아야 한다.
    for (Waypoint& point : globalPath.waypoints) {
        if (point.roadId == 2) point.y += 2.8;
    }

    WorldState world = makeWorldWithObstacle();
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    safetyConfig.collisionPredictionHorizon = 5.0;
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath, 10)) {
        std::cerr << "offset keep-target reconnect was not generated\n";
        std::abort();
    }
    if (laneChange.routeMode != LaneChangeRouteMode::KeepTargetLane) {
        std::cerr << "offset reconnect unexpectedly fell back to source lane\n";
        std::abort();
    }
    std::string reason;
    if (!validateGeneratedPath(laneChange.path, &reason)) {
        std::cerr << "offset reconnect retained invalid geometry: "
                  << reason << '\n';
        std::abort();
    }
}

void testM05AvoidanceReconnectHasTrackableGeometry() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::cerr << "failed to load LivingLAB map for M05 regression\n";
        std::abort();
    }
    PathPlannerData globalPath;
    if (!loadWaypointsCsv(globalPath.waypoints,
                          "scenario/mixed/M05/route.csv")) {
        std::cerr << "failed to load M05 route for reconnect regression\n";
        std::abort();
    }

    // m05_test에서 첫 회피가 생성된 frame 1198 상태.
    WorldState world;
    world.ego.x = 1079.87061F;
    world.ego.y = 729.56842F;
    world.ego.heading = 3.65332031F;
    world.egoSpeed = 11.3100725;
    world.egoLaneValid = true;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoRoadId = 2815;
    world.egoLaneId = 3;
    world.egoSectionIndex = 0;
    world.egoRoadS = 100.85;
    world.egoPathS = 387.794657;
    world.frontBumperPathS = world.egoPathS + 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    WorldObjectState obstacle;
    obstacle.raw.id = 2;
    obstacle.raw.x = 1026.5824F;
    obstacle.raw.y = 702.125244F;
    obstacle.raw.heading = 3.62375998F;
    obstacle.raw.speed = 0.0F;
    obstacle.raw.length = 4.67399979F;
    obstacle.raw.width = 1.77999997F;
    obstacle.obb = makeObb2D(
        obstacle.raw.x, obstacle.raw.y, obstacle.raw.heading,
        obstacle.raw.length, obstacle.raw.width);
    obstacle.laneValid = true;
    obstacle.roadId = 2815;
    obstacle.laneId = 4;
    obstacle.sectionIndex = 0;
    obstacle.roadS = 40.67146029;
    obstacle.pathValid = true;
    obstacle.pathS = 447.78821081;
    obstacle.pathD = -2.16670011;
    obstacle.relativeS = 59.90047;
    obstacle.relativeD = -2.16670011;
    obstacle.isAhead = true;
    obstacle.isOnEgoPath = true;
    world.objects.push_back(obstacle);

    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath, 779)) {
        std::cerr << "M05 avoidance candidate was not generated\n";
        std::abort();
    }
    std::string reason;
    if (!validateGeneratedPath(laneChange.path, &reason)) {
        std::cerr << "M05 avoidance retained invalid reconnect geometry: "
                  << reason << '\n';
        std::abort();
    }
    if (laneChange.path.back().s < laneChange.rejoinRouteS + 25.0) {
        std::cerr << "M05 return path has too little post-rejoin lookahead\n";
        std::abort();
    }

    laneChange.phase = LaneChangePhase::Execute;
    world.egoPathS = laneChange.rejoinRouteS;
    world.egoPathD = 2.8;
    advanceLaneChangePhase(laneChange, config, world, 0.1,
                           false, true, true);
    if (laneChange.phase != LaneChangePhase::Execute) {
        std::cerr << "M05 return completed before lateral convergence\n";
        std::abort();
    }
    world.egoPathD = 0.5;
    advanceLaneChangePhase(laneChange, config, world, 0.1,
                           false, true, true);
    if (laneChange.phase != LaneChangePhase::Settle) {
        std::cerr << "M05 return did not complete after lateral convergence\n";
        std::abort();
    }
}

void testM08StaticBlockerChoosesFreeLaneBeforeRoadConnection() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::cerr << "failed to load LivingLAB map for M08 regression\n";
        std::abort();
    }
    PathPlannerData globalPath;
    if (!loadWaypointsCsv(globalPath.waypoints,
                          "scenario/mixed/M08/route.csv")) {
        std::cerr << "failed to load M08 route for obstacle regression\n";
        std::abort();
    }

    // m08_fix frame 198: road 429/lane 1에서 앞의 ID 6을 인식한
    // 첫 프레임. 기존 코드는 다음 road 604로 연결되지 않는다는
    // 이유로 빈 회피 차선을 600프레임 동안 거부했다.
    WorldState world;
    world.ego.x = 269.025238F;
    world.ego.y = 69.5931091F;
    world.ego.heading = 5.04341364F;
    world.egoSpeed = 12.4377402;
    const LaneMatch egoMatch = worldToLane(
        map, world.ego.x, world.ego.y, world.ego.heading);
    world.egoLaneValid = egoMatch.valid;
    world.egoRoadId = egoMatch.roadId;
    world.egoSectionIndex = egoMatch.sectionIndex;
    world.egoLaneId = egoMatch.laneId;
    world.egoRoadS = egoMatch.roadS;
    world.egoLaneWidth = egoMatch.laneWidth;
    world.egoPathValid = true;
    world.egoPathS = 81.9163157;
    world.egoPathD = -0.00101259093;
    world.frontBumperPathS = world.egoPathS + 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    const auto addStaticObject = [&](int id, double x, double y,
                                     double pathS, double pathD,
                                     bool onEgoPath) {
        WorldObjectState object;
        object.raw.id = id;
        object.raw.x = static_cast<float>(x);
        object.raw.y = static_cast<float>(y);
        object.raw.heading = 5.06248283F;
        object.raw.speed = 0.0F;
        object.raw.length = 4.39400005F;
        object.raw.width = 1.80799997F;
        object.obb = makeObb2D(x, y, object.raw.heading,
                               object.raw.length, object.raw.width);
        const LaneMatch match = worldToLane(
            map, x, y, object.raw.heading);
        object.laneValid = match.valid;
        object.roadId = match.roadId;
        object.sectionIndex = match.sectionIndex;
        object.laneId = match.laneId;
        object.roadS = match.roadS;
        object.pathValid = true;
        object.pathS = pathS;
        object.pathD = pathD;
        object.relativeS = pathS - world.egoPathS;
        object.relativeD = pathD - world.egoPathD;
        object.isAhead = true;
        object.isOnEgoPath = onEgoPath;
        world.objects.push_back(object);
    };
    addStaticObject(6, 289.461914, 13.6035557,
                    world.egoPathS + 59.6056, 0.233827, true);
    addStaticObject(5, 290.549413, 1.191794,
                    world.egoPathS + 71.596, -3.07362, false);

    const auto nearest = std::lower_bound(
        globalPath.waypoints.begin(), globalPath.waypoints.end(),
        world.egoPathS,
        [](const Waypoint& point, double s) { return point.s < s; });
    const std::size_t nearestIndex = static_cast<std::size_t>(
        std::distance(globalPath.waypoints.begin(), nearest));
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath,
            nearestIndex)) {
        std::cerr << "M08 static blocker still did not produce an avoidance path\n";
        std::abort();
    }
    if (laneChange.routeMode != LaneChangeRouteMode::KeepTargetLane ||
        laneChange.roadConnectionManeuver) {
        std::cerr << "M08 obstacle avoidance was still subordinated to road alignment\n";
        std::abort();
    }
    if (laneChange.rejoinRouteS <=
        world.egoPathS + config.transitionLength + 10.0) {
        std::cerr << "M08 delayed lane shift still completes on the old fixed-distance timer\n";
        std::abort();
    }
}

void testM08DenseSecondBlockerUsesSafeLongitudinalSample() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::cerr << "failed to load LivingLAB map for dense M08 regression\n";
        std::abort();
    }
    PathPlannerData globalPath;
    if (!loadWaypointsCsv(globalPath.waypoints,
                          "scenario/mixed/M08/route.csv")) {
        std::cerr << "failed to load M08 route for dense obstacle regression\n";
        std::abort();
    }

    // m08_lattice3_final frame 352: lane 4 앞 ID 7을 피할 때
    // 왼쪽 lane 3의 ID 5가 11 m 앞에 있다. 고정된 완료점 하나를
    // 강제하지 않고, 종방향 샘플 중 충돌 없는 인접 차선 경로가
    // 선택되는지 검증한다. 어느 쪽이 비었는지는 후보 전체의 실제
    // 기하 충돌 검사 결과에 맡긴다.
    WorldState world;
    world.ego.x = 283.160278F;
    world.ego.y = 10.3804932F;
    world.ego.heading = 5.08701897F;
    world.egoSpeed = 6.65093027;
    const LaneMatch egoMatch = worldToLane(
        map, world.ego.x, world.ego.y, world.ego.heading);
    world.egoLaneValid = egoMatch.valid;
    world.egoRoadId = egoMatch.roadId;
    world.egoSectionIndex = egoMatch.sectionIndex;
    world.egoLaneId = egoMatch.laneId;
    world.egoRoadS = egoMatch.roadS;
    world.egoLaneWidth = egoMatch.laneWidth;
    world.egoLaneCenterDistance = egoMatch.distance;
    world.egoPathValid = true;
    world.egoPathS = 142.384479;
    world.egoPathD = -6.79130112;
    world.frontBumperPathS = world.egoPathS + 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    const auto addObject = [&](int id, double x, double y,
                               double relativeS, double relativeD,
                               bool onPath) {
        WorldObjectState object;
        object.raw.id = id;
        object.raw.x = static_cast<float>(x);
        object.raw.y = static_cast<float>(y);
        object.raw.heading = 5.06248283F;
        object.raw.speed = 0.0F;
        object.raw.length = 4.39400005F;
        object.raw.width = 1.80799997F;
        object.obb = makeObb2D(x, y, object.raw.heading,
                               object.raw.length, object.raw.width);
        const LaneMatch match = worldToLane(
            map, x, y, object.raw.heading);
        object.laneValid = match.valid;
        object.roadId = match.roadId;
        object.sectionIndex = match.sectionIndex;
        object.laneId = match.laneId;
        object.roadS = match.roadS;
        object.pathValid = true;
        object.pathS = world.egoPathS + relativeS;
        object.pathD = world.egoPathD + relativeD;
        object.relativeS = relativeS;
        object.relativeD = relativeD;
        object.isAhead = relativeS > 0.0;
        object.isOnEgoPath = onPath;
        world.objects.push_back(object);
    };
    addObject(5, 290.549413, 1.191794, 11.1279, 3.71768, false);
    addObject(7, 291.787359, -10.614313, 22.3623, 0.516563, true);
    addObject(8, 289.579656, -13.649157, 24.2968, -2.69769, false);
    addObject(4, 293.738928, -15.577894, 27.6344, 0.359875, true);

    const auto nearest = std::lower_bound(
        globalPath.waypoints.begin(), globalPath.waypoints.end(),
        world.egoPathS,
        [](const Waypoint& point, double s) { return point.s < s; });
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    const std::vector<int> confirmed{4, 5, 7, 8};
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath,
            static_cast<std::size_t>(std::distance(
                globalPath.waypoints.begin(), nearest)),
            nullptr, &confirmed)) {
        std::cerr << "M08 dense blocker did not produce a lattice candidate\n";
        std::abort();
    }
    if (laneChange.targetLaneId == world.egoLaneId || laneChange.path.empty() ||
        laneChange.rejoinRouteS - world.egoPathS < 12.0) {
        std::cerr << "M08 did not select a collision-free lattice sample"
                  << " target_lane=" << laneChange.targetLaneId
                  << " completion_distance="
                  << (laneChange.rejoinRouteS - world.egoPathS)
                  << " path_points=" << laneChange.path.size() << '\n';
        std::abort();
    }
}

void testM09DenseFutureRoadChecksEntireStaticManeuver() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::cerr << "failed to load LivingLAB map for dense M09 regression\n";
        std::abort();
    }
    PathPlannerData globalPath;
    if (!loadWaypointsCsv(globalPath.waypoints,
                          "scenario/mixed/M09/route.csv")) {
        std::cerr << "failed to load M09 route for dense obstacle regression\n";
        std::abort();
    }

    // m09_full_fix frame 3950. 현재 차선의 object 11보다 옆 차선의
    // object 16이 3.4 m 먼저 있다. 과거의 12초 정적 예측 horizon은
    // object 16 직전에서 끝나 잘못된 조기 차선변경을 승인했다.
    WorldState world;
    world.ego.x = 874.124084F;
    world.ego.y = -838.382935F;
    world.ego.heading = -2.44506216F;
    world.egoSpeed = 2.80346419;
    const LaneMatch egoMatch = worldToLane(
        map, world.ego.x, world.ego.y, world.ego.heading);
    world.egoLaneValid = egoMatch.valid;
    world.egoRoadId = egoMatch.roadId;
    world.egoSectionIndex = egoMatch.sectionIndex;
    world.egoLaneId = egoMatch.laneId;
    world.egoRoadS = egoMatch.roadS;
    world.egoLaneWidth = egoMatch.laneWidth;
    world.egoLaneCenterDistance = egoMatch.distance;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoPathS = 556.372871;
    world.egoPathD = 3.10229334;
    world.frontBumperPathS = world.egoPathS + 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    const auto addStaticObject = [&](int id, double x, double y,
                                     double heading, double length,
                                     double width, double relativeS,
                                     double relativeD, bool onPath) {
        WorldObjectState object;
        object.raw.id = id;
        object.raw.x = static_cast<float>(x);
        object.raw.y = static_cast<float>(y);
        object.raw.heading = static_cast<float>(heading);
        object.raw.speed = 0.0F;
        object.raw.length = static_cast<float>(length);
        object.raw.width = static_cast<float>(width);
        object.obb = makeObb2D(x, y, heading, length, width);
        const LaneMatch match = worldToLane(map, x, y, heading);
        object.laneValid = match.valid;
        object.roadId = match.roadId;
        object.sectionIndex = match.sectionIndex;
        object.laneId = match.laneId;
        object.roadS = match.roadS;
        object.pathValid = true;
        object.pathS = world.egoPathS + relativeS;
        object.pathD = world.egoPathD + relativeD;
        object.relativeS = relativeS;
        object.relativeD = relativeD;
        object.isAhead = true;
        object.isOnEgoPath = onPath;
        world.objects.push_back(object);
    };
    addStaticObject(2, 852.3479, -851.416016, 3.80569911,
                    4.39400005, 1.80799997, 25.1811, -3.15886, false);
    addStaticObject(16, 821.072998, -875.616638, 0.669031203,
                    0.900000036, 0.900000036, 64.7251, -3.5534, false);
    addStaticObject(11, 815.6571655, -884.3120728, 3.80738265,
                    4.39400005, 1.80799997, 68.1411, 0.0465695, true);

    const auto nearest = std::lower_bound(
        globalPath.waypoints.begin(), globalPath.waypoints.end(),
        world.egoPathS,
        [](const Waypoint& point, double s) { return point.s < s; });
    LaneChangeData laneChange;
    LaneChangeConfig config;
    PlanningBehaviorConfig safetyConfig;
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath,
            static_cast<std::size_t>(std::distance(
                globalPath.waypoints.begin(), nearest)))) {
        std::cerr << "M09 dense future-road blockers produced no avoidance path\n";
        std::abort();
    }
    // 정상 후보는 가까운 옆 차선 장애물보다 먼저 안착하는 조기 경로가
    // 아니어야 한다. 실제 OBB 검사를 통과한 늦은 종방향 샘플이어야 한다.
    if (laneChange.rejoinRouteS < world.egoPathS + 50.0) {
        std::cerr << "M09 selected a lane shift before checking the closer target-lane blocker"
                  << " completion_distance="
                  << (laneChange.rejoinRouteS - world.egoPathS) << '\n';
        std::abort();
    }
}

void testM10StaggeredBlockersUseForwardLaneTopology() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::cerr << "failed to load LivingLAB map for M10 regression\n";
        std::abort();
    }
    PathPlannerData globalPath;
    if (!loadWaypointsCsv(globalPath.waypoints,
                          "scenario/mixed/M10/route.csv")) {
        std::cerr << "failed to load M10 route for obstacle regression\n";
        std::abort();
    }

    // m10_avoidance_diagnosis frame 134. lane -2의 자전거를 피하려는
    // 순간 다음 section의 lane -1에는 정차 차량이 24.5 m 앞에 있다.
    // 과거 로직은 같은 section만 확인해 lane -1을 빈 차선으로 오인했고,
    // 그곳에 안착한 뒤 남은 9.9 m에서는 다음 회피를 만들지 못했다.
    WorldState world;
    world.ego.x = 1215.07166F;
    world.ego.y = -737.82135F;
    world.ego.heading = 3.82659125F;
    world.egoSpeed = 4.76115181;
    const LaneMatch egoMatch = worldToLane(
        map, world.ego.x, world.ego.y, world.ego.heading);
    world.egoLaneValid = egoMatch.valid;
    world.egoRoadId = egoMatch.roadId;
    world.egoSectionIndex = egoMatch.sectionIndex;
    world.egoLaneId = egoMatch.laneId;
    world.egoRoadS = egoMatch.roadS;
    world.egoLaneWidth = egoMatch.laneWidth;
    world.egoLaneCenterDistance = egoMatch.distance;
    world.egoPathValid = true;
    world.inputValid = true;
    world.egoPathS = 19.0111198;
    world.egoPathD = 1.20839201;
    world.frontBumperPathS = world.egoPathS + 3.8;
    world.egoObb = makeObb2D(
        world.ego.x, world.ego.y, world.ego.heading, 4.8, 1.9);

    const auto addStaticObject = [&](int id, double x, double y,
                                     double heading, double length,
                                     double width, double pathS,
                                     bool onPath) {
        WorldObjectState object;
        object.raw.id = id;
        object.raw.x = static_cast<float>(x);
        object.raw.y = static_cast<float>(y);
        object.raw.heading = static_cast<float>(heading);
        object.raw.speed = 0.0F;
        object.raw.length = static_cast<float>(length);
        object.raw.width = static_cast<float>(width);
        object.obb = makeObb2D(x, y, heading, length, width);
        // 자전거는 도로를 비스듬히 향해 있어 객체 heading 대신 Ego의
        // 진행방향으로 물리 차선을 매칭한다.
        const LaneMatch match = worldToLane(
            map, x, y, world.ego.heading);
        object.laneValid = match.valid;
        object.roadId = match.roadId;
        object.sectionIndex = match.sectionIndex;
        object.laneId = match.laneId;
        object.roadS = match.roadS;
        object.pathValid = true;
        object.pathS = pathS;
        object.pathD = 0.0;
        object.relativeS = pathS - world.egoPathS;
        object.relativeD = 0.0;
        object.isAhead = true;
        object.isOnEgoPath = onPath;
        world.objects.push_back(object);
    };
    addStaticObject(14, 1205.616333, -745.827087, 5.34381294,
                    2.22000003, 0.200000003, 32.2013, true);
    addStaticObject(3, 1198.65283, -755.245544, 3.82093191,
                    4.39400005, 1.80799997, 43.4922, false);

    const auto nearest = std::lower_bound(
        globalPath.waypoints.begin(), globalPath.waypoints.end(),
        world.egoPathS,
        [](const Waypoint& point, double s) { return point.s < s; });
    LaneChangeData laneChange;
    LaneChangeConfig config;
    config.transitionLength = 15.0;
    config.maximumLaneChangeSpeed = 15.0 / 3.6;
    PlanningBehaviorConfig safetyConfig;
    const std::vector<int> confirmed{3, 14};
    if (!tryPrepareObstacleLaneChange(
            laneChange, config, safetyConfig, world, map, globalPath,
            static_cast<std::size_t>(std::distance(
                globalPath.waypoints.begin(), nearest)),
            nullptr, &confirmed)) {
        std::cerr << "M10 staggered blockers produced no avoidance path\n";
        std::abort();
    }
    if (laneChange.targetLaneId == -1) {
        std::cerr << "M10 still selected the next-section occupied lane -1"
                  << " followup=" << laneChange.followupObstacleId
                  << " clearance=" << laneChange.followupObstacleClearance
                  << '\n';
        std::abort();
    }
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

void testLeadBrakingUsesRelativeStoppingDistance() {
    PlanningBehaviorConfig config;
    config.comfortableDeceleration = 1.5;
    config.followTimeGap = 1.8;
    config.minimumFollowDistance = 5.0;
    config.followGapGain = 0.35;

    WorldState world;
    world.egoSpeed = 10.0;
    world.ego.heading = 0.0F;
    WorldObjectState lead;
    lead.raw.id = 4;
    lead.raw.speed = 4.0F;
    lead.raw.heading = 0.0F;
    lead.isAhead = true;
    lead.isOnEgoPath = true;
    // 시간간격 문턱(23 m) 바로 밖이다. 예전 로직은 여기서 12.5 m/s로
    // 계속 가속했지만 상대 정지거리까지 보면 이미 감속해야 한다.
    lead.relativeS = 24.0;
    world.objects.push_back(lead);

    BehaviorState state;
    BehaviorDecision output;
    ConstraintSet constraints;
    constraints.speedLimit = 12.5;
    StopDecision stop;
    updateBehavior(state, output, constraints, stop, world,
                   12.5, config, 0.05);
    if (output.longitudinal != LonMode::Follow ||
        output.targetSpeed >= world.egoSpeed) {
        std::cerr << "lead braking did not trigger braking-aware follow\n";
        std::abort();
    }
}

void testCloseMovingLeadMayCommandStop() {
    PlanningBehaviorConfig config;

    WorldState world;
    world.egoSpeed = 20.0 / 3.6;
    world.ego.heading = 0.0F;
    WorldObjectState lead;
    lead.raw.id = 9;
    lead.raw.speed = 0.5F;
    lead.raw.heading = 0.0F;
    lead.isAhead = true;
    lead.isOnEgoPath = true;
    lead.relativeS = 1.0;
    world.objects.push_back(lead);

    BehaviorState state;
    BehaviorDecision output;
    ConstraintSet constraints;
    constraints.speedLimit = 20.0 / 3.6;
    StopDecision stop;
    updateBehavior(state, output, constraints, stop, world,
                   20.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Follow);
    assert(std::abs(output.targetSpeed) < 1e-9);

    // 신호·횡단보도 등 명시적인 정지 요청은 최소 진행속도로 덮지 않는다.
    stop.valid = true;
    stop.reason = StopReason::RedLight;
    stop.distanceToStop = 0.0;
    updateBehavior(state, output, constraints, stop, world,
                   20.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Stop);
    assert(std::abs(output.targetSpeed) < 1e-9);
}

void testPreparedLaneChangeCancelsWhenBlockerMoves() {
    LaneChangeData laneChange;
    laneChange.phase = LaneChangePhase::Prepare;
    laneChange.obstacleId = 17;
    LaneChangeConfig config;
    WorldState world;
    advanceLaneChangePhase(laneChange, config, world, 0.1,
                           false, false, true);
    if (laneChange.phase != LaneChangePhase::None ||
        laneChange.obstacleId != -1) {
        std::cerr << "prepared lane change survived a cleared blocker\n";
        std::abort();
    }
}

void testStationaryAdjacentVehicleUsesShortRangeCaution() {
    PlanningBehaviorConfig config;
    config.stationaryAdjacentVehicleCautionSpeed = 20.0 / 3.6;
    config.stationaryAdjacentVehicleMaximumDistance = 25.0;
    config.nearbyObjectLateralDistance = 5.5;

    WorldState world;
    world.egoSpeed = 40.0 / 3.6;
    world.ego.heading = 0.0F;
    world.egoLaneValid = true;
    world.egoRoadId = 10;
    world.egoSectionIndex = 2;
    world.egoLaneId = 3;

    WorldObjectState parkedVehicle;
    parkedVehicle.raw.id = 7;
    parkedVehicle.raw.speed = 0.0F;
    parkedVehicle.raw.heading = 0.0F;
    parkedVehicle.raw.length = 4.85F;
    parkedVehicle.raw.width = 1.89F;
    parkedVehicle.laneValid = true;
    parkedVehicle.roadId = 10;
    parkedVehicle.sectionIndex = 2;
    parkedVehicle.laneId = 2;
    parkedVehicle.pathValid = true;
    parkedVehicle.isAhead = true;
    parkedVehicle.isOnEgoPath = false;
    parkedVehicle.relativeS = 20.0;
    parkedVehicle.relativeD = 3.5;
    world.objects.push_back(parkedVehicle);

    BehaviorState state;
    BehaviorDecision output;
    ConstraintSet constraints;
    constraints.speedLimit = 50.0 / 3.6;
    StopDecision stop;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Follow);
    assert(std::abs(output.targetSpeed - 20.0 / 3.6) < 1e-9);

    // 25 m 밖의 차량까지 미리 감속하지 않는다.
    world.objects.front().relativeS = 26.0;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Cruise);

    // 같은 위치라도 콘 크기의 정적 물체에는 기존처럼 순항한다.
    world.objects.front().relativeS = 20.0;
    world.objects.front().raw.length = 0.6F;
    world.objects.front().raw.width = 0.6F;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Cruise);

    // 두 차선 떨어진 정차 차량 역시 감속 대상이 아니다.
    world.objects.front().raw.length = 4.85F;
    world.objects.front().raw.width = 1.89F;
    world.objects.front().laneId = 1;
    updateBehavior(state, output, constraints, stop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Cruise);
}

void testCloserVehicleConstraintOverridesFarStopPlan() {
    PlanningBehaviorConfig config;
    config.followTimeGap = 1.8;
    config.minimumFollowDistance = 5.0;
    config.nearbyObjectCautionSpeed = 20.0 / 3.6;
    config.nearbyObjectLateralDistance = 5.5;

    WorldState world;
    world.egoSpeed = 50.0 / 3.6;
    WorldObjectState cuttingIn;
    cuttingIn.raw.id = 6;
    cuttingIn.raw.speed = 2.0F;
    cuttingIn.raw.heading = 0.0F;
    cuttingIn.pathValid = true;
    cuttingIn.isAhead = true;
    cuttingIn.relativeS = 10.0;
    cuttingIn.relativeD = 3.5;
    world.objects.push_back(cuttingIn);

    ConstraintSet constraints;
    constraints.speedLimit = 50.0 / 3.6;
    StopDecision farSignalStop;
    farSignalStop.valid = true;
    farSignalStop.reason = StopReason::RedLight;
    farSignalStop.distanceToStop = 76.0;

    BehaviorState state;
    BehaviorDecision output;
    updateBehavior(state, output, constraints, farSignalStop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Follow);
    assert(std::abs(output.targetSpeed - 2.0) < 1e-6);

    // 정지점이 더 가까우면 신호 정지 제약이 그대로 우선한다.
    farSignalStop.distanceToStop = 1.0;
    updateBehavior(state, output, constraints, farSignalStop, world,
                   50.0 / 3.6, config, 0.05);
    assert(output.longitudinal == LonMode::Stop);
    assert(std::abs(output.targetSpeed) < 1e-9);
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

void testNearestWaypointCannotJumpToOverlappingFutureBranch() {
    std::vector<Waypoint> path;
    for (int index = 0; index <= 20; ++index) {
        Waypoint point;
        point.x = static_cast<double>(index);
        point.y = 0.0;
        point.yaw = 0.0;
        point.s = static_cast<double>(index);
        path.push_back(point);
    }
    // 훨씬 뒤의 route branch가 같은 월드 위치를 반대방향으로 지난다.
    for (int index = 0; index <= 20; ++index) {
        Waypoint point;
        point.x = static_cast<double>(20 - index);
        point.y = 0.0;
        point.yaw = 3.14159265358979323846;
        point.s = 60.0 + index;
        path.push_back(point);
    }
    GtFrame gt;
    gt.ego.x = 5.0F;
    gt.ego.y = 0.0F;
    gt.ego.heading = 0.0F;
    const std::size_t nearest = findNearestWaypoint(path, gt, 4);
    if (nearest != 5) {
        std::cerr << "nearest waypoint jumped to another route branch: "
                  << nearest << '\n';
        std::abort();
    }
}

void testLaneGraphDoesNotInventMissingSectionLink() {
    PathGeneratorData map;
    map.sampleInterval = 0.5;
    Road road;
    road.id = 77;
    road.length = 40.0;
    Geometry geometry;
    geometry.type = GeometryType::Line;
    geometry.length = road.length;
    road.geometries.push_back(geometry);
    for (double sectionS : {0.0, 20.0}) {
        LaneSection section;
        section.s = sectionS;
        LaneDefinition lane;
        lane.id = -1;
        lane.type = "driving";
        lane.widths.push_back(constantWidth(3.5));
        // successor/predecessor를 의도적으로 두지 않는다.
        section.lanes.emplace(-1, lane);
        road.sections.push_back(std::move(section));
    }
    map.roads.emplace(road.id, std::move(road));
    sampleLaneCenters(map);
    buildLaneGraph(map);
    const LaneKey start{77, 0, -1};
    const LaneKey goal{77, 1, -1};
    if (!connectLanes(map, start, goal).empty()) {
        std::cerr << "lane graph invented a missing laneSection link\n";
        std::abort();
    }
}

void testMixedCheckpointRoutesHaveValidGeometry() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::cerr << "failed to load LivingLAB map for route regression\n";
        std::abort();
    }
    for (const char* checkpoints : {
             "scenario/mixed/M03/checkpoints.csv",
             "scenario/mixed/M04/checkpoints.csv",
             "scenario/mixed/M09/checkpoints.csv"}) {
        if (!loadCheckpointsCsv(map, checkpoints)) std::abort();
        const std::vector<Waypoint> route = makePath(map);
        std::string reason;
        if (route.empty() || !validateGeneratedPath(route, &reason)) {
            std::cerr << "invalid mixed route generated from " << checkpoints
                      << ": " << reason << '\n';
            std::abort();
        }
    }
}

void testMixedCheckpointMissionsUseEntryExitCorridors() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::abort();
    }
    for (const char* scenario : {
             "M05", "M06", "M07", "M08",
             "M09", "M10", "M11", "M12", "M13", "M14"}) {
        MissionCorridorData mission;
        const std::string checkpointPath =
            std::string("scenario/mixed/") + scenario + "/checkpoints.csv";
        if (!initializeMissionCorridor(mission, map, checkpointPath)) {
            std::cerr << scenario
                      << " checkpoint road corridor was disconnected\n";
            std::abort();
        }
        const std::size_t expectedLegCount = mission.checkpoints.size() / 2;
        if (mission.legs.size() != expectedLegCount) {
            std::cerr << scenario
                      << " mission corridor has wrong checkpoint/leg count\n";
            std::abort();
        }
        for (std::size_t index = 0; index < mission.legs.size(); ++index) {
            mission.activeLegIndex = index;
            const MissionLegData& leg = mission.legs[index];
            const std::size_t expectedFrom = 2 * index;
            const std::size_t expectedTo = std::min(
                expectedFrom + 2, mission.checkpoints.size() - 1);
            if (leg.fromCheckpoint != expectedFrom ||
                leg.gatewayCheckpoint != expectedFrom + 1 ||
                leg.toCheckpoint != expectedTo ||
                !leg.valid || leg.allowedLanes.empty() ||
                leg.targetLanes.empty() || leg.preferredRoute.empty() ||
                leg.allowedLanes.count(leg.preferredRoute.front()) == 0 ||
                leg.targetLanes.count(leg.preferredRoute.back()) == 0) {
                std::cerr << "invalid " << scenario
                          << " mission corridor leg " << index << '\n';
                std::abort();
            }
            // Corridor 범위는 roadMark와 무관하다. 한 section/direction이
            // 들어왔으면 그 안의 모든 driving lane이 포함되고, 실제 실선
            // 검사는 lane-change trajectory 생성 시점에만 수행한다.
            for (const LaneKey& key : leg.allowedLanes) {
                const Road& road = map.roads.at(key.roadId);
                const LaneSection& section =
                    road.sections.at(key.sectionIndex);
                for (const auto& [laneId, lane] : section.lanes) {
                    if (laneId == 0 || lane.type != "driving" ||
                        (laneId > 0) != (key.laneId > 0)) {
                        continue;
                    }
                    if (!leg.allowedLanes.count(
                            {key.roadId, key.sectionIndex, laneId})) {
                        std::cerr << scenario
                                  << " corridor was narrowed by lane marking\n";
                        std::abort();
                    }
                }
            }
        }
        if (std::string(scenario) == "M09") {
            // 교차로 진입 좌표 근처의 잘못된 직진 road가 checkpoint
            // 통과로 오판되면 안 된다. exit road를 실제로 맵매칭한
            // 후에만 다음 자유 회랑으로 넘어간다.
            mission.activeLegIndex = 0;
            mission.minimumTargetDistance =
                std::numeric_limits<double>::infinity();
            mission.targetRoadStableFrames = 0;
            mission.targetRoadObserved = false;
            const MissionLegData& first = mission.legs.front();
            const Checkpoint& exit =
                mission.checkpoints[first.toCheckpoint].checkpoint;
            WorldState world;
            world.ego.x = exit.x;
            world.ego.y = exit.y;
            world.egoLaneValid = true;
            world.egoRoadId =
                mission.checkpoints[first.gatewayCheckpoint].roadIds.front();
            updateMissionCorridorProgress(mission, world);
            if (mission.activeLegIndex != 0) {
                std::cerr << "M09 advanced at an unobserved exit road\n";
                std::abort();
            }
            world.egoRoadId =
                mission.checkpoints[first.toCheckpoint].roadIds.front();
            updateMissionCorridorProgress(mission, world);
            if (mission.activeLegIndex != 1) {
                std::cerr << "M09 did not advance after observing exit road\n";
                std::abort();
            }
        }
    }
}

void testM11DisplacedEgoStillGetsContinuousLocalPath() {
    PathGeneratorData map;
    if (!loadMap(map, "src/data/map/HL_FMA_VTD_LivingLab.xodr", 0.5)) {
        std::abort();
    }
    PathPlannerData guide;
    if (!loadWaypointsCsv(guide.waypoints,
                          "scenario/mixed/M11/route.csv")) {
        std::abort();
    }
    MissionCorridorData mission;
    if (!initializeMissionCorridor(
            mission, map, "scenario/mixed/M11/checkpoints.csv")) {
        std::abort();
    }
    mission.activeLegIndex = 2;

    const std::vector<std::pair<double, double>> samples{
        {1455.30725, 587.564514}, {1455.55481, 587.562439},
        {1455.77686, 587.560364}, {1456.63, 587.551}};
    MapMatchState matchState;
    for (const auto& [x, y] : samples) {
        matchWorldPosition(matchState, map, x, y, -0.0091, std::nullopt);
    }
    const LaneMatch& match = matchState.lastMatch;
    WorldState world;
    world.ego.x = 1456.63F;
    world.ego.y = 587.551F;
    world.ego.heading = -0.0091F;
    world.egoLaneValid = match.valid;
    world.egoRoadId = match.roadId;
    world.egoSectionIndex = match.sectionIndex;
    world.egoLaneId = match.laneId;
    world.egoRoadS = match.roadS;
    world.egoLaneWidth = match.laneWidth;
    world.egoLaneCenterDistance = match.distance;
    GtFrame routeFrame;
    routeFrame.ego = world.ego;
    const WorldState routeProjection = makeWorldState(
        routeFrame, map, guide.waypoints);
    world.egoPathValid = routeProjection.egoPathValid;
    world.egoPathS = routeProjection.egoPathS;
    world.egoPathD = routeProjection.egoPathD;
    world.inputValid = world.egoLaneValid;
    world.egoSpeed = 0.0;
    world.frontBumperOffset = 3.8;

    PathPlannerData local;
    PathPlannerConfig plannerConfig;
    if (!buildRoadIdTrackingPath(
            local, plannerConfig, world, map, guide, 0, 80.0,
            nullptr, &mission) || local.waypoints.size() < 2) {
        std::cerr << "M11 displaced Ego did not get a local lane path: road="
                  << world.egoRoadId << " section=" << world.egoSectionIndex
                  << " lane=" << world.egoLaneId << " road_s="
                  << world.egoRoadS << '\n';
        std::abort();
    }
    WorldState projected = world;
    if (!projectWorldToActivePath(
            projected, local.waypoints, world.frontBumperOffset)) {
        std::cerr << "M11 displaced Ego local path could not be projected\n";
        std::abort();
    }
    if (std::abs(projected.egoPathD) > 8.0) {
        std::cerr << "M11 displaced Ego selected an unrelated local path\n";
        std::abort();
    }
    const auto point = std::lower_bound(
        local.waypoints.begin(), local.waypoints.end(), projected.egoPathS,
        [](const Waypoint& waypoint, double pathS) {
            return waypoint.s < pathS;
        });
    const Waypoint& nearest = point == local.waypoints.end()
        ? local.waypoints.back() : *point;
    const double remaining = local.waypoints.back().s - projected.egoPathS;
    const double headingError = std::abs(normalizeAngle(
        static_cast<double>(world.ego.heading) - nearest.yaw));
    if (remaining < 2.0 || headingError > 1.35) {
        std::cerr << "M11 displaced local path unusable: d="
                  << projected.egoPathD << " remaining=" << remaining
                  << " heading_error=" << headingError << " path_lane="
                  << nearest.roadId << '/' << nearest.sectionIndex << '/'
                  << nearest.laneId << '\n';
        std::abort();
    }
}

}  // namespace

int main() {
    testRecoveryDoesNotCommandAStop();
    testEmergencyBrakingLatchesEntrySteering();
    testTrafficControllerParsing();
    testRdbSimulationTimeMatchesGtInFrameOrder();
    testStaticZoneRouteAlignment();
    testSchoolZoneReferencePointConfig();
    testScenarioRoutesKeepTheirOwnCrosswalksAndSchoolZone();
    testPredictedCollisionIsNotCurrentCollision();
    testConvergingOncomingVehicleIsPredictedOutsideNarrowCorridor();
    testCloseMovingLeadMayCommandStop();
    testBlockedReturnUsesWaitingSpeedUntilCollisionIsUrgent();
    testMovingReturnConflictCanClearAtWaitingSpeed();
    testObstacleLaneChange();
    testBlockedAdjacentLanesPreferFartherObstacle();
    testSolidLineForbidsObstacleLaneChange();
    testAvoidanceLaneMustConnectToNextRoad();
    testRoadIdTrackingKeepsCurrentLaneInsideRoad();
    testStaticObstacleFollowNeverCommandsZero();
    testLaneChangeWhenAdjacentLaneStartsAhead();
    testO01ObstacleRoadLaneMerge();
    testO02RouteSectionComesFromMapRoadS();
    testO04RouteSectionComesFromMapRoadS();
    testLaneChangeExecutionBlockedByTrafficStops();
    testActivePathProjectionDoesNotRequireGuideRoute();
    testPreviousLocalPathBridgesOneFrameSectionRebuildFailure();
    testLaneChangeAcrossRenumberedLaneSection();
    testLimitedLateralOffsetAvoidance();
    testKeepTargetLaneWhenItRejoinsNextRoad();
    testKeepTargetReconnectSmoothsOffsetRoute();
    testM05AvoidanceReconnectHasTrackableGeometry();
    testM08StaticBlockerChoosesFreeLaneBeforeRoadConnection();
    testM08DenseSecondBlockerUsesSafeLongitudinalSample();
    testM09DenseFutureRoadChecksEntireStaticManeuver();
    testM10StaggeredBlockersUseForwardLaneTopology();
    testNearbyAdjacentVehicleCapsSpeedBeforeCutIn();
    testLeadBrakingUsesRelativeStoppingDistance();
    testPreparedLaneChangeCancelsWhenBlockerMoves();
    testStationaryAdjacentVehicleUsesShortRangeCaution();
    testCloserVehicleConstraintOverridesFarStopPlan();
    testSafeAvoidanceIgnoresStaticSiblingObstacles();
    testSharpCurveHasSeparateSpeed();
    testNearestWaypointCannotJumpToOverlappingFutureBranch();
    testLaneGraphDoesNotInventMissingSectionLink();
    testMixedCheckpointRoutesHaveValidGeometry();
    testMixedCheckpointMissionsUseEntryExitCorridors();
    testM11DisplacedEgoStillGetsContinuousLocalPath();
    std::cout << "controller_logic_tests: PASS\n";
    return 0;
}
