#pragma once
#include "data/map/precomputed_lane_network.hpp"
#include "planning/planning_structs.hpp"
#include <memory>
#include <chrono>
#include <array>

struct DecisionTraceData;
struct CorridorGlobalRouteWorker;
struct CorridorLocalRouteWorker;
struct CorridorEdge {
    std::int64_t to{-1};
    double distance{};
    int transition{-1};
    bool lateral{};
};
struct CorridorRuntime {
    PrecomputedLaneNetwork network;
    std::vector<std::vector<CorridorEdge>> forward, reverse;
    // Additional medium/long legal lane-change edges for obstacle-aware A*.
    // Keep the established gateway/tracking policy graph unchanged.
    std::vector<std::vector<CorridorEdge>> globalLateral;
    std::vector<std::array<CorridorEdge,4>> globalInLaneShift;
    std::vector<bool> ordinaryRoadPoint;
    // Bounded, obstacle-aware overlap beyond the current exit. This is not
    // the next intersection and never advances actual mission progress.
    std::vector<bool> globalSearchAllowed,globalSearchGoal;
    bool globalExitPreviewActive{};
    std::vector<double> gatewayCost;
    // Deterministic shortest-path policy to the active gateway.  This is the
    // corridor-wide global route: it is computed once when the mission leg is
    // entered and does not change from frame to frame.  A lateral next edge is
    // materialized by the checked quintic planner; longitudinal/merge/split
    // edges are followed directly by the tracker.
    std::vector<std::int64_t> gatewayNext;
    std::vector<bool> gatewaySpine;
    // Obstacle-aware route materialized from the current corridor entry all
    // the way to the checkpoint gateway.  Unlike the old per-frame greedy
    // lane choice, this path sees every stationary obstacle before choosing
    // the first lateral move.  It is kept until the leg changes, the vehicle
    // leaves it after a dynamic avoidance, or a newly stationary object
    // physically blocks it.
    std::vector<Waypoint> committedGlobalPath;
    std::size_t committedGlobalPathLeg{static_cast<std::size_t>(-1)};
    std::size_t committedGlobalPathProgress{};
    bool committedGlobalPathReachesGateway{};
    bool committedGlobalPathHasExitPreview{};
    std::vector<Waypoint> pendingGatewayHandoffPath;
    // Geometry availability is not a collision approval. A blocked route
    // remains available as a local-planning reference, but must not replace
    // a safe avoidance trajectory in the controller.
    bool trackingPathApproved{};
    int trackingPathBlockingObject{-1};
    double trackingPathValidatedClearance{-1.};
    // Short physical C2 bridge from the measured post-manoeuvre pose back to
    // sampled XODR lanes.  It is cached because rebuilding the same 80 m
    // bridge in every 20 Hz frame is both unnecessary and too expensive.
    std::vector<Waypoint> recoveryTrackingPath;
    std::size_t recoveryTrackingPathLeg{static_cast<std::size_t>(-1)};
    std::size_t recoveryTrackingPathProgress{};
    // A certified tight-gap avoidance tail keeps its checked clearance;
    // recaching must not silently inflate it to the ordinary comfort margin.
    double recoveryTrackingPathClearance{-1.};
    // True when this bridge ends on a caller-supplied map-derived mission
    // reference, rather than merely on the nearest physical driving lane.
    bool recoveryTrackingPathReachesReference{};
    // A failed measured-pose -> mission-reference bridge is expensive and
    // its geometry does not change every control tick. Retry after meaningful
    // motion or a short simulation-time cooldown while physical lane tracking
    // keeps commands flowing.
    LaneKey referenceRecoveryFailureLane;
    bool referenceRecoveryFailureValid{};
    double referenceRecoveryFailureRoadS{};
    double referenceRecoveryRetryAfter{};
    // Negative cache only for the expensive measured-pose -> committed
    // global-route bridge. It never certifies a path or suppresses collision
    // checks. Changes to the route, pose or received scene invalidate it.
    struct ReconnectFailure {
        bool valid{};
        std::size_t leg{}, progress{}, pointCount{};
        std::uint64_t routeRevision{}, requestRevision{};
        LaneKey lane;
        double x{}, y{}, z{}, yaw{}, speed{}, curvature{}, horizon{}, failedAt{};
        double clearance{}, maneuverSpeed{};
        VehicleFootprint2D footprint;
        std::vector<std::array<double,20>> observations;
    } committedReconnectFailure;
    // A failed full-corridor search is retried when the physical lane or the
    // stationary-object set changes, not in every 20 Hz frame.  The current
    // path remains available to the local avoidance planner meanwhile.
    bool globalRouteSearchFailed{};
    bool globalRouteReplanRequested{};
    std::uint64_t globalRouteRequestRevision{};
    std::shared_ptr<CorridorGlobalRouteWorker> globalRouteWorker;
    // Independent bounded local job. These flags do not approve any path;
    // the caller must keep ordinary safety and traffic checks active.
    std::shared_ptr<CorridorLocalRouteWorker> localRouteWorker;
    bool localFallbackPending{};
    bool localFallbackEngaged{};
    bool localFallbackContinuationRequested{};
    std::size_t localFallbackLeg{static_cast<std::size_t>(-1)};
    std::uint64_t localFallbackEpoch{}, localFallbackSignature{};
    std::chrono::steady_clock::time_point localFallbackRetryAt;
    double localFallbackLastX{},localFallbackLastY{};
    bool localFallbackIssued{};
    LaneKey globalRouteFailureLane;
    std::size_t globalRouteFailureStaticCount{};
    std::uint64_t globalRouteFailureStaticSignature{};
    double globalRouteFailureRoadS{};
    double globalRouteRetryAfter{};
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
    std::uint64_t routeRevision{};
    double lateralLength{};
    LaneChangeConfig laneConfig;
    double previousX{},previousY{},previousYaw{},curvature{};
    bool poseValid{};
    // Live planning shares one budget across all searches in a control frame.
    // Offline callers retain exhaustive search unless they start a frame.
    bool budgetedSearch{}, searchStarted{}, obstacleSearchCalled{}, gatewaySearchCalled{};
    std::chrono::steady_clock::time_point searchDeadline;
    std::array<std::size_t,2> searchCursor{};
    std::size_t searchLeg{static_cast<std::size_t>(-1)};
    int obstacleSearchObject{-1};
    // Set only after a complete nominal-margin scan fails for the same
    // mandatory blocker. Compact road furniture starts in this mode because
    // its usable passage can be narrower than the ordinary comfort margin.
    bool obstacleTightClearanceMode{};
    // A generated merge/split has no ordinary lane points in its interior.
    // Latch the selected transition so nearest-lane rematching cannot replace
    // it with the visually adjacent straight branch halfway through.
    int committedTransition{-1};
    std::size_t committedTransitionLeg{static_cast<std::size_t>(-1)};
    std::size_t committedTransitionProgress{};
    bool committedTransitionEntered{};
};

void beginCorridorSearchFrame(CorridorRuntime& runtime);
// Keep the usable trajectory until a replacement from the measured pose is
// ready. Used after avoidance has reached its target lane.
void requestCorridorGlobalReplan(CorridorRuntime& runtime);

// Returns only a currently attached, fully checked local replacement. False
// never clears the active trajectory; Pending/Engaged report worker state.
bool prepareLocalCorridorFallback(
    LaneChangeData& replacement,const LaneChangeConfig& config,
    const PlanningBehaviorConfig& safety,const WorldState& world,
    const PathGeneratorData& map,const MissionCorridorData& mission,
    const std::vector<Waypoint>& reference,DecisionTraceData* trace);

// Revalidate the unconsumed, physically attached suffix using the measured
// body, all received stationary boxes and predicted moving actors. No
// semantic size filter is applied. Known other road decks are excluded using
// candidate-road elevation, never the current Ego altitude. This is required
// before releasing a completed avoidance onto another tracking path.
bool corridorTrackingPathIsSafe(
    const std::vector<Waypoint>& path,const WorldState& world,double margin,
    int* blockerId=nullptr,
    const PlanningBehaviorConfig* behaviorConfig=nullptr,
    const PathGeneratorData* map=nullptr);
bool corridorObstacleAtPathLevel(const PathGeneratorData& map,
                                const Waypoint& point,
                                const WorldObjectState& object);
double corridorStationaryFreeDistance(
    const std::vector<Waypoint>& path,const WorldState& world,double margin,
    const PathGeneratorData* map=nullptr,int* blockerId=nullptr);
// Keep a safe avoidance continuation while its gateway replacement is being
// rebuilt. A blocked/consumed/detached trajectory is never cached as safe.
bool retainCorridorAvoidanceTail(
    const PathGeneratorData& map,const MissionCorridorData& mission,
    const WorldState& world,const std::vector<Waypoint>& path,
    DecisionTraceData* trace=nullptr,double validatedClearance=-1.);

bool initializeCorridorRuntime(PathGeneratorData& map,const std::string& directory,
                              const LaneChangeConfig& config,std::string* error);
bool initializeCorridorRuntime(PathGeneratorData& map,PrecomputedLaneNetwork network,
                              const LaneChangeConfig& config,std::string* error);
bool buildCorridorTrackingPath(PathPlannerData& output,const PathPlannerConfig& config,
                              const WorldState& world,const PathGeneratorData& map,
                              const MissionCorridorData& mission,double horizon,
                              DecisionTraceData* trace);
bool buildReferenceRecoveryTrackingPath(
    PathPlannerData& output,const PathPlannerConfig& config,
    const std::vector<Waypoint>& reference,std::size_t referenceProgress,
    const WorldState& world,const PathGeneratorData& map,
    const MissionCorridorData& mission,double horizon,
    DecisionTraceData* trace);
// The ordinary XODR lane graph does not contain every validated, precomputed
// merge/split curve.  Tracking validation uses this query so those curves are
// not rejected as disconnected paths.
bool corridorHasLongitudinalConnection(const PathGeneratorData& map,
                                       const LaneKey& from,
                                       const LaneKey& to);
// A materialized corridor route may cross into the target lane one or two
// frames before VTD's lane matcher changes IDs.  Accept only lane-key changes
// that are actually present on that already validated committed route.
bool corridorCommittedRouteConnects(const PathGeneratorData& map,
                                    const LaneKey& from,
                                    const LaneKey& to);
bool prepareCorridorLattice(LaneChangeData& state,const LaneChangeConfig& config,
                           const PlanningBehaviorConfig& safetyConfig,const WorldState& world,
                           const PathGeneratorData& map,const MissionCorridorData& mission,
                           const std::vector<int>* confirmed,double* blockerDistance,
                           bool gatewayOnly,DecisionTraceData* trace,
                           const PathPlannerData* activeReference=nullptr,
                           bool activeReferenceIsMissionGuide=false,
                           const LaneAlignmentContext* alignment=nullptr);
