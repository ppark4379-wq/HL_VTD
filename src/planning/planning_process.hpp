#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "planning/planning_structs.hpp"

#include <string>

struct DecisionTraceData;
// Front-bumper distance; returns an upper bound, never an acceleration request.
double schoolZoneApproachSpeedLimit(const PathPlannerConfig& planner,
    const PlanningBehaviorConfig& behavior, double distanceToEntry, double egoSpeed);
// Finite only for a stationary entrance queue after all complete safe
// gateway search failed. Mid-approach obstacles retain ordinary avoidance.
double soleGatewayQueueDistance(const WorldState& world,
    const PathGeneratorData& map, const MissionCorridorData& mission);
bool queuedGatewayBypassIsComplete(const std::vector<Waypoint>& path,
    const MissionCorridorData& mission);

// Full-crosswalk yield is for vulnerable road users; ordinary vehicles retain
// following, collision prediction and emergency braking authority.
bool crosswalkHasRiskObject(const Crosswalk& crosswalk,
                           const WorldState& world, double predictionHorizon,
                           const std::vector<int>& staticPropIds = {});
std::vector<int> updateCrosswalkStationaryRelease(const Crosswalk& crossing,
    const WorldState& world, const StopDecision& previousStop,
    const std::vector<StationaryObjectState>& history,
    const PlanningBehaviorConfig& config, double dt,
    std::vector<CrosswalkObjectWaitState>& waits);
double settledStraightManeuverSpeedCap(const LaneChangeData& maneuver,
    const WorldState& world, const std::vector<Waypoint>& path,
    double cruiseSpeed, double clearance, const LaneChangeConfig& laneConfig,
    const PlanningBehaviorConfig& config);
std::uint8_t previewTrajectoryTurnSignal(const WorldState& world,
    const PathGeneratorData& map, const std::vector<Waypoint>& path);
void maintainTrajectoryTurnSignal(PlanningProcessData& data,
    const WorldState& world, std::uint8_t previewSignal, double dt);
double crosswalkStopRouteS(const Crosswalk& crossing, const WorldState& world,
    const StaticZoneData& zones, const std::vector<Waypoint>& route, double searchDistance);

// A still-received actor is not necessarily still blocking the original
// driving path. Only retire an unstarted, single-object detour when the
// selected successor is safe and already continuous with the measured pose.
bool canCancelClearedPreparedAvoidance(
    const LaneChangeData& maneuver, const WorldState& world,
    const std::vector<Waypoint>& successor, bool successorPathSafe);

// Geometry gate only: the caller separately certifies gateway topology and
// current static/dynamic safety before releasing a temporary local detour.
bool canResumeCommittedGatewayRoute(
    const LaneChangeData& maneuver, const WorldState& world,
    const std::vector<Waypoint>& successor, bool successorSafe);

// Join the actually attached tracking path to its certified gateway tail.
// Collision checking must use this continuous path, not require the unjoined
// reference in the adjacent lane to already lie underneath Ego.
bool buildGatewayHandoffContinuation(
    std::vector<Waypoint>& output, const std::vector<Waypoint>& successor,
    const std::vector<Waypoint>& gatewayReference);

// Releasing a temporary recovery is a geometric/safety decision, not a
// minimum travel-distance or elapsed-time quota.
bool canResumeOrdinaryPathAfterRecovery(
    const WorldState& world, const std::vector<Waypoint>& path, bool pathSafe);

bool initializePlanning(PlanningProcessData& data,
                        const PathPlannerConfig& plannerConfig,
                        const PlanningBehaviorConfig& behaviorConfig,
                        const LaneChangeConfig& laneChangeConfig,
                        const std::string& referencePathCsv);

void runPlanningProcess(PlanningProcessData& data,
                        const GtFrame& gt,
                        const WorldState& world,
                        const PathGeneratorData& map,
                        double dt,
                        DecisionTraceData* trace = nullptr);
