#include "planning/local/corridor_lattice.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "logger/decision_trace.hpp"
#include "utils/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;

void require(bool okay, const char* message) {
    if (!okay) throw std::runtime_error(message);
}

struct Scene {
    LaneChangeConfig config;
    PlanningBehaviorConfig safety;
    PathGeneratorData map;
    MissionCorridorData mission;
    WorldState world;
    std::vector<Waypoint> reference;
    DecisionTraceData trace;
    int laneCount;
    double maximumForegroundMs{};

    explicit Scene(int lanes = 3, bool solid = false, bool oneWayEscape = false)
        : laneCount(lanes) {
        PrecomputedLaneNetwork network;
        for (int roadId = 1; roadId <= 2; ++roadId) {
            Road road;
            road.id = roadId;
            road.length = 100.;
            Geometry geometry;
            geometry.x = (roadId - 1) * 100.;
            geometry.length = 100.;
            road.geometries.push_back(geometry);
            road.elevations.push_back({0., 0., 0., 0., 0.});
            LaneSection section;
            for (int laneId = -1; laneId >= -lanes; --laneId) {
                LaneDefinition lane;
                lane.id = laneId;
                lane.type = "driving";
                lane.widths.push_back({0., 3.5, 0., 0., 0.});
                lane.roadMarks.push_back({0., solid ? "none" : oneWayEscape ? "decrease" : "both", "white",
                                         solid ? "solid" : "broken"});
                section.lanes[laneId] = lane;
                PrecomputedLane raw;
                raw.key = {roadId, 0, laneId};
                raw.topologyStatus = "connected";
                LaneSegment segment;
                segment.key = raw.key;
                for (int index = 0; index <= 200; ++index) {
                    PrecomputedLanePoint p;
                    p.id = static_cast<std::int64_t>(network.points.size());
                    p.pointIndex = index;
                    p.width = 3.5;
                    p.usable = true;
                    p.pose.x = geometry.x + index * .5;
                    p.pose.y = (laneId + .5) * 3.5;
                    p.pose.s = p.pose.roadS = index * .5;
                    p.pose.roadId = roadId;
                    p.pose.laneId = laneId;
                    if (index < 200) p.nextPoint = p.id + 1;
                    raw.pointIds.push_back(p.id);
                    segment.points.push_back(p.pose);
                    network.points.push_back(p);
                }
                network.lanes[raw.key] = raw;
                map.laneSegments[raw.key] = segment;
            }
            road.sections.push_back(section);
            map.roads[roadId] = road;
        }
        for (auto& point : network.points) {
            for (int side : {1, -1}) {
                const auto adjacent = network.lanes.find(
                    {point.pose.roadId, 0, point.pose.laneId + side});
                if (adjacent == network.lanes.end()) continue;
                const auto id = adjacent->second.pointIds[point.pointIndex];
                if (side > 0) {
                    point.leftPoint = id;
                    point.leftChangeAllowed = !solid && !oneWayEscape;
                } else {
                    point.rightPoint = id;
                    point.rightChangeAllowed = !solid;
                }
            }
        }
        for (int laneId = -1; laneId >= -lanes; --laneId) {
            PrecomputedLaneConnection connection;
            connection.from = {1, 0, laneId};
            connection.to = {2, 0, laneId};
            connection.status = "continuous";
            connection.fromPoint = network.lanes[connection.from].pointIds.back();
            connection.toPoint = network.lanes[connection.to].pointIds.front();
            network.connections.push_back(connection);
            map.laneGraph[connection.from].push_back(connection.to);
        }
        std::string error;
        require(initializeCorridorRuntime(map, std::move(network), config, &error),
                error.c_str());
        mission.valid = mission.initialized = mission.topologyAuthority = true;
        mission.checkpoints.resize(2);
        mission.checkpoints[0].checkpoint.x = 10.;
        mission.checkpoints[1].checkpoint.x = 180.;
        MissionLegData leg;
        leg.valid = leg.topologyAuthority = leg.finalLeg = true;
        leg.gatewayCheckpoint = leg.toCheckpoint = 1;
        for (int roadId = 1; roadId <= 2; ++roadId) {
            for (int laneId = -1; laneId >= -lanes; --laneId) {
                leg.allowedLanes.insert({roadId, 0, laneId});
                if (roadId == 2 && (!oneWayEscape || laneId == -1)) {
                    leg.targetLanes.insert({roadId, 0, laneId});
                    leg.targetRoadS[{roadId, 0, laneId}] = 80.;
                }
            }
        }
        mission.legs.push_back(leg);
        world.ego.x = 10.;
        world.ego.y = -1.75;
        world.egoSpeed = 3.;
        world.inputValid = world.egoLaneValid = world.egoPathValid = true;
        world.egoRoadId = 1;
        world.egoLaneId = -1;
        world.egoRoadS = world.egoPathS = 10.;
        world.egoLaneWidth = 3.5;
        world.egoObb = makeObb2D(11.3725, -1.75, 0., 4.855, 1.88);
        for (double x = 0.; x <= 180.; x += .5) {
            Waypoint p;
            p.x = p.s = x;
            p.y = -1.75;
            p.roadId = x < 100. ? 1 : 2;
            p.roadS = x < 100. ? x : x - 100.;
            p.laneId = -1;
            reference.push_back(p);
        }
        trace.enabled = true;
        // Activate the production mission masks with an empty scene; unlike
        // buildCorridorTrackingPath this never launches the global worker.
        LaneChangeData empty;
        require(!call(empty), "empty scene unexpectedly activated local fallback");
        auto& runtime = *map.corridorRuntime;
        require(runtime.activeLeg == mission.activeLegIndex,
                "local API did not initialize mission permission masks");
        runtime.globalRouteSearchFailed = true;
        runtime.trackingPathApproved = false;
    }

    void addObject(int id, double x, double y, double length = 4.4, double width = 2.) {
        WorldObjectState object;
        object.raw.id = id;
        object.raw.x = x;
        object.raw.y = y;
        object.raw.length = length;
        object.raw.width = width;
        object.raw.height = 1.8;
        object.obb = makeObb2D(x, y, 0., length, width);
        object.relativeS = x - world.ego.x;
        object.relativeD = y - world.ego.y;
        object.isAhead = object.relativeS > 0.;
        object.isOnEgoPath = std::abs(object.relativeD) < 1.5;
        object.pathValid = true;
        world.objects.push_back(object);
    }

    bool call(LaneChangeData& replacement) {
        const auto before = Clock::now();
        const bool ready = prepareLocalCorridorFallback(
            replacement, config, safety, world, map, mission, reference, &trace);
        const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - before).count();
        maximumForegroundMs = std::max(maximumForegroundMs, ms);
        require(ms < 100., "foreground local poll performed a blocking search (>100ms)");
        return ready;
    }

    bool await(LaneChangeData& replacement) {
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        bool observedPending = false;
        do {
            if (call(replacement)) return true;
            const auto& runtime = *map.corridorRuntime;
            if (observedPending && !runtime.localFallbackPending) return false;
            observedPending = observedPending || runtime.localFallbackPending;
            if (!runtime.localFallbackEngaged) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        } while (Clock::now() < deadline);
        throw std::runtime_error("local worker did not complete a bounded fixture in 5s");
    }

    void requireSafe(const LaneChangeData& result) const {
        require(result.localCorridorAvoidance && result.phase != LaneChangePhase::None,
                "accepted trajectory lacks local-worker ownership");
        require(result.path.size() > 2 && result.validatedObstacleClearance >= .049,
                "accepted trajectory lacks a usable certified clearance");
        require(std::hypot(result.path.front().x - world.ego.x,
                           result.path.front().y - world.ego.y) < .3,
                "local path does not attach to the measured pose");
        const auto body = vehicleFootprintFromObb(
            world.egoObb, {world.ego.x, world.ego.y, world.ego.heading});
        const auto inspect = [&](double x, double y, double yaw) {
            const auto inflated = vehicleFootprintAt(
                body, {x, y, yaw}, result.validatedObstacleClearance);
            for (const auto& object : world.objects)
                require(!intersects(inflated, object.obb),
                        "accepted local path intersects a current-world obstacle");
            const auto physical = vehicleFootprintAt(body, {x, y, yaw});
            for (const auto& corner : physical.corners)
                require(corner.y >= -laneCount * 3.5 - 1e-6 && corner.y <= 1e-6,
                        "local swept vehicle body left the allowed road surface");
        };
        for (std::size_t i = 0; i < result.path.size(); ++i) {
            const auto& p = result.path[i];
            require(mission.legs[0].allowedLanes.count({p.roadId, p.sectionIndex, p.laneId}) != 0,
                    "local path uses a lane outside the mission corridor");
            inspect(p.x, p.y, p.yaw);
            if (!i) continue;
            const auto& previous = result.path[i - 1];
            const auto distance = std::hypot(p.x - previous.x, p.y - previous.y);
            const auto yawDelta = std::atan2(std::sin(p.yaw - previous.yaw),
                                              std::cos(p.yaw - previous.yaw));
            const int subdivisions = std::max(1, int(std::ceil(distance / .2)));
            for (int part = 1; part < subdivisions; ++part) {
                const double t = double(part) / subdivisions;
                inspect(previous.x + t * (p.x - previous.x),
                        previous.y + t * (p.y - previous.y), previous.yaw + t * yawDelta);
            }
        }
        require(corridorTrackingPathIsSafe(result.path, world,
                    result.validatedObstacleClearance, nullptr, &safety, &map),
                "local worker approval disagrees with the current-world suffix guard");
    }
};

void noActivationOnClearOrDistantRoad() {
    Scene scene;
    LaneChangeData replacement;
    require(!scene.call(replacement) && !scene.map.corridorRuntime->localFallbackEngaged,
            "clear road unnecessarily engaged local recovery");
    scene.addObject(1, 100., -1.75);
    require(!scene.call(replacement) && !scene.map.corridorRuntime->localFallbackEngaged,
            "distant obstacle prematurely engaged local recovery");
    require(replacement.phase == LaneChangePhase::None && replacement.path.empty(),
            "inactive local search mutated the selected maneuver");
}

void earlyBlockedRouteProducesSafeBypass() {
    Scene scene;
    scene.addObject(1, 38., -1.75);
    require(corridorStationaryFreeDistance(scene.reference, scene.world, .3, &scene.map) > 15.,
            "early-activation fixture accidentally starts inside the old mandatory range");
    auto& runtime = *scene.map.corridorRuntime;
    const auto targets = scene.mission.legs[0].targetLanes;
    const auto gatewayPolicy = runtime.gatewayNext;
    const auto allowed = runtime.allowed;
    LaneChangeData result;
    require(scene.await(result), "blocked global route did not trigger a safe local bypass before 15m");
    scene.requireSafe(result);
    require(result.path.back().x > 44., "local path stopped short of passing its first blocker");
    require(runtime.committedGlobalPath.empty() && runtime.globalRouteSearchFailed &&
                runtime.gatewayNext == gatewayPolicy && runtime.allowed == allowed &&
                scene.mission.legs[0].targetLanes == targets && scene.mission.activeLegIndex == 0,
            "local recovery altered global planning policy or advanced the mission");
    std::cout << "early local bypass foreground maximum " << scene.maximumForegroundMs << "ms\n";
}

void approvalFlagCannotHideActualBlocker() {
    Scene scene;
    scene.addObject(1, 38., -1.75);
    auto& runtime = *scene.map.corridorRuntime;
    runtime.globalRouteSearchFailed = false;
    runtime.globalRouteReplanRequested = false;
    runtime.trackingPathApproved = true;
    LaneChangeData result;
    require(scene.await(result), "stale global approval suppressed local avoidance");
    scene.requireSafe(result);
    require(runtime.trackingPathApproved && !runtime.globalRouteSearchFailed,
            "local fallback changed global approval state");
}

void safeReferenceIgnoresStaleBlockedGlobal() {
    Scene scene;
    scene.addObject(1, 38., -5.25);
    auto& runtime = *scene.map.corridorRuntime;
    runtime.committedGlobalPath = scene.reference;
    for (auto& p : runtime.committedGlobalPath) { p.y = -5.25; p.laneId = -2; }
    LaneChangeData result;
    require(!scene.call(result) && !runtime.localFallbackEngaged,
            "unused blocked global route displaced safe selected reference");
    runtime.localFallbackIssued = true;
    scene.reference.resize(45); // short, safe reference after an old local manoeuvre
    require(!scene.call(result) && !runtime.localFallbackEngaged,
            "completed local manoeuvre reactivated on a short safe reference");
}

void denseTightPassageDoesNotForceEarlyRecenter() {
    Scene scene(2);
    // A 2.20m aperture between two parked bodies fits Ego + .05m, but not
    // the ordinary .30m comfort envelope. It straddles the broken lane line.
    scene.addObject(1, 38., -1.35, 4.4, 2.1);
    scene.addObject(2, 38., -5.65, 4.4, 2.1);
    // The following middle obstruction requires another lateral choice;
    // returning to a lane centre at the first object's tail is not a plan.
    scene.addObject(3, 58., -3.5, 4.4, 1.2);
    // Independently certify that the requested fixture is physically
    // solvable. This witness is never supplied to the search: it moves to
    // the shared broken boundary, holds parallel through the first row,
    // then moves to lane -2 before the middle obstruction.
    LaneChangeData witness;
    witness.phase = LaneChangePhase::Execute;
    witness.localCorridorAvoidance = true;
    witness.validatedObstacleClearance = .05;
    for (double x = 10.; x <= 70. + 1e-6; x += .2) {
        const double start = x <= 44. ? 10. : 44.;
        const double length = x <= 44. ? 20. : 9.;
        const double base = x <= 44. ? -1.75 : -3.5;
        const double t = std::clamp((x - start) / length, 0., 1.);
        const double blend = t * t * t * (10. + t * (-15. + 6. * t));
        const double derivative = 30. * t * t * (1. - t) * (1. - t) / length;
        Waypoint p;
        p.x = p.roadS = x;
        p.y = base - 1.75 * blend;
        p.yaw = std::atan2(-1.75 * derivative, 1.);
        p.roadId = 1;
        p.laneId = p.y < -3.5 ? -2 : -1;
        p.s = witness.path.empty() ? 10. : witness.path.back().s +
            std::hypot(p.x - witness.path.back().x, p.y - witness.path.back().y);
        witness.path.push_back(p);
    }
    scene.requireSafe(witness);
    LaneChangeData result;
    const bool ready = scene.await(result);
    if (!ready) {
        for (const auto& event : scene.trace.frameEvents)
            if (event.find("corridor_local_fallback") != std::string::npos)
                std::cerr << event << '\n';
    }
    require(ready, "dense two-stage narrow passage produced no local trajectory");
    scene.requireSafe(result);
    require(result.validatedObstacleClearance < .1,
            "tight-only physical opening was not certified with its actual clearance");
    require(scene.config.latticeObstacleClearance == .30 &&
                scene.map.corridorRuntime->laneConfig.latticeObstacleClearance == .30,
            "local tight-gap recovery relaxed the ordinary global clearance");
    require(result.path.back().x > 64., "dense local path does not clear the complete obstacle group");
    bool crossedOpening = false, changedAfterOpening = false;
    for (const auto& p : result.path) {
        if (std::abs(p.x - 38.) < 1.) {
            crossedOpening = true;
            require(p.y < -3.35 && p.y > -3.65,
                    "narrow opening did not use the safe between-lane centre strip");
        }
        if (std::abs(p.x - 58.) < 1. && std::abs(p.y + 3.5) > 1.5)
            changedAfterOpening = true;
    }
    require(crossedOpening && changedAfterOpening,
            "dense path lacks a safe second lateral stage after the narrow opening");
    double maxCurvature = 0.;
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        const auto& p = result.path[i];
        const auto& previous = result.path[i - 1];
        const double distance = std::hypot(p.x - previous.x, p.y - previous.y);
        if (distance > 1e-5)
            maxCurvature = std::max(maxCurvature,
                std::abs(std::atan2(std::sin(p.yaw - previous.yaw),
                                   std::cos(p.yaw - previous.yaw))) / distance);
    }
    require(maxCurvature < .185,
            "assembled local path exceeds the physical curvature bound at a join");
    std::cout << "dense route points=" << result.path.size()
              << " end_x=" << result.path.back().x
              << " margin=" << result.validatedObstacleClearance
              << " max_curvature=" << maxCurvature
              << " foreground_ms=" << scene.maximumForegroundMs << '\n';
    for (const auto& event : scene.trace.frameEvents)
        if (event.find("corridor_local_fallback: worker_completed") != std::string::npos)
            std::cout << event << '\n';
}

void changedWorldNeverAdoptsStaleResult() {
    Scene scene;
    scene.addObject(1, 38., -1.75);
    LaneChangeData replacement;
    require(!scene.call(replacement) && scene.map.corridorRuntime->localFallbackPending,
            "new local search did not dispatch asynchronously");
    // New information arrives after dispatch. The full-width wall is still
    // outside the measured body, but closes every forward candidate.
    scene.addObject(99, 18., -5.25, 1., 10.5);
    require(!scene.await(replacement), "stale obstacle-free worker result was adopted after a new wall");
    require(replacement.path.empty() && replacement.phase == LaneChangePhase::None,
            "rejected stale worker result leaked into the selected maneuver");
}

void solidAndUnreachableGatewayCannotBecomeEscapeGoals() {
    Scene scene(2, true);
    scene.addObject(1, 38., -1.75);
    LaneChangeData replacement;
    const bool selected = scene.await(replacement);
    if (selected) {
        scene.requireSafe(replacement);
        for (const auto& p : replacement.path)
            require(p.y > -2.5 && p.laneId == -1,
                    "local recovery crossed a solid boundary to escape a blocked lane");
        require(replacement.path.back().x < 32.,
                "impossible solid-separated bypass was declared complete");
    }
    // Physically paved is not synonymous with gateway-reachable. A legal
    // one-way lateral move can enter lane -2, but lane-change permissions
    // prohibit returning to the only required exit lane, -1. Both worker
    // and foreground derive this from the same map/mission, not a manually
    // edited foreground mask that the worker snapshot would never see.
    Scene deadEnd(2, false, true);
    const auto& runtime = *deadEnd.map.corridorRuntime;
    const auto id = runtime.network.lanes.at({1, 0, -2}).pointIds[100];
    require(!std::isfinite(runtime.gatewayCost[id]),
            "one-way lateral fixture did not create a genuinely unreachable escape lane");
    deadEnd.addObject(1, 38., -1.75);
    LaneChangeData unavailable;
    if (deadEnd.await(unavailable)) {
        deadEnd.requireSafe(unavailable);
        const auto& end = unavailable.path.back();
        require(end.laneId == -1,
                "local search terminated in a lane with no route to the required gateway");
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        const std::string selected = argc > 1 ? argv[1] : "all";
        if (selected == "all" || selected == "inactive") noActivationOnClearOrDistantRoad();
        if (selected == "all" || selected == "early") earlyBlockedRouteProducesSafeBypass();
        if (selected == "all" || selected == "activation") {
            approvalFlagCannotHideActualBlocker();
            safeReferenceIgnoresStaleBlockedGlobal();
        }
        if (selected == "all" || selected == "dense") denseTightPassageDoesNotForceEarlyRecenter();
        if (selected == "all" || selected == "stale") changedWorldNeverAdoptsStaleResult();
        if (selected == "all" || selected == "legal") solidAndUnreachableGatewayCannotBecomeEscapeGoals();
        std::cout << "local corridor fallback " << selected << " regressions PASS\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
