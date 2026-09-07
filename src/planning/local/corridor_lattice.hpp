#pragma once
#include "data/map/precomputed_lane_network.hpp"
#include "planning/planning_structs.hpp"
#include <memory>

struct DecisionTraceData;
struct CorridorEdge {
    std::int64_t to{-1};
    double distance{};
    int transition{-1};
    bool lateral{};
};
struct CorridorRuntime {
    PrecomputedLaneNetwork network;
    std::vector<std::vector<CorridorEdge>> forward, reverse;
    std::vector<double> gatewayCost;
    // Cost through the current gateway and the next mission leg.  This is a
    // tracking-only handoff route: it prevents a valid precomputed merge from
    // disappearing during the one or two frames in which mission progress is
    // switching legs.
    std::vector<double> handoffCost;
    // Minimum number of legal lateral stages remaining to a gateway. Unlike
    // metric cost, this does not confuse ordinary forward progress with an
    // actual lane-alignment stage.
    std::vector<int> gatewayLateralSteps;
    std::vector<bool> allowed;
    std::vector<bool> trackingAllowed;
    std::vector<bool> surfaceAllowed;
    std::vector<std::pair<double,double>> tangent;
    std::map<std::pair<int,int>,std::vector<std::int64_t>> spatial;
    std::size_t activeLeg{static_cast<std::size_t>(-1)};
    double lateralLength{};
    double previousX{},previousY{},previousYaw{},curvature{};
    bool poseValid{};
};

bool initializeCorridorRuntime(PathGeneratorData& map,const std::string& directory,
                              const LaneChangeConfig& config,std::string* error);
bool initializeCorridorRuntime(PathGeneratorData& map,PrecomputedLaneNetwork network,
                              const LaneChangeConfig& config,std::string* error);
bool buildCorridorTrackingPath(PathPlannerData& output,const PathPlannerConfig& config,
                              const WorldState& world,const PathGeneratorData& map,
                              const MissionCorridorData& mission,double horizon,
                              DecisionTraceData* trace);
// The ordinary XODR lane graph does not contain every validated, precomputed
// merge/split curve.  Tracking validation uses this query so those curves are
// not rejected as disconnected paths.
bool corridorHasLongitudinalConnection(const PathGeneratorData& map,
                                       const LaneKey& from,
                                       const LaneKey& to);
bool prepareCorridorLattice(LaneChangeData& state,const LaneChangeConfig& config,
                           const PlanningBehaviorConfig& safetyConfig,const WorldState& world,
                           const PathGeneratorData& map,const MissionCorridorData& mission,
                           const std::vector<int>* confirmed,double* blockerDistance,
                           bool gatewayOnly,DecisionTraceData* trace);
