#pragma once

#include "data/map/map_structs.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <tuple>
#include <vector>

// Mirrors tools/map_precompute's version-2 CSV schema. This is a static map
// product; corridor/gateway reachability belongs to the runtime mission.
struct PrecomputedLanePoint {
    std::int64_t id{-1};
    Waypoint pose;                // s = lane-local travel distance, NOT global route_s
    std::size_t pointIndex{};
    double curvature{};           // signed curvature in travel direction [1/m]
    double width{};
    std::int64_t nextPoint{-1};    // only inside this lane; -1 at end/source break
    std::int64_t leftPoint{-1};    // adjacent same-road/section sample near same road_s
    std::int64_t rightPoint{-1};
    bool leftChangeAllowed{};     // roadMark permission at this station only
    bool rightChangeAllowed{};
    bool usable{};                // width/curvature screen, NOT vehicle collision approval
    std::int64_t replacedByTransition{-1}; // raw dead-end tail superseded by a maneuver
    std::vector<std::size_t> outgoingTransitions; // branching at an interior sample
};

struct PrecomputedLaneConnection {
    LaneKey from;
    LaneKey to;
    std::int64_t fromPoint{-1};
    std::int64_t toPoint{-1};
    std::string provenance;       // lane_section_*, road_*, junction_laneLink
    std::string contactPoint;
    int junctionId{-1};
    std::string connectionId;
    double gap{};
    double headingDelta{};
    double curvatureDelta{};
    std::string status;           // "continuous" or explicit diagnostic reason
};

struct PrecomputedLane {
    LaneKey key;
    std::vector<std::int64_t> pointIds; // travel order; increasing lane-local s
    std::vector<std::size_t> outgoingConnections; // preserve junction branching
    bool hasTopologicalEntry{true};
    bool hasTopologicalExit{true};
    std::string topologyStatus{"not_audited"}; // connected/no_entry/no_exit/no_entry_or_exit/excluded_by_review
};

struct PrecomputedLaneKeyLess {
    bool operator()(const LaneKey& a, const LaneKey& b) const {
        return std::tie(a.roadId, a.sectionIndex, a.laneId) <
               std::tie(b.roadId, b.sectionIndex, b.laneId);
    }
};

struct PrecomputedTransitionPoint {
    double roadS{};
    double s{};
    double x{};
    double y{};
    double yaw{};
    double curvature{};
};

struct PrecomputedLaneTransition {
    std::string kind;              // merge, split or same-road source-gap continuation
    LaneKey eventLane;
    std::int64_t fromPoint{-1};    // actual shared raw sample, may be inside a lane
    std::int64_t toPoint{-1};
    double minClearance{};         // sampled vehicle footprint to driving surface
    double maxCurvature{};
    std::string provenance;       // generated maneuver, not an XML laneLink
    std::vector<PrecomputedTransitionPoint> points;
    bool crossesSolid{false};
    bool violatesLaneChangePermission{false};
    bool requiresPlannerValidation{true};
    std::string roadMarkConflicts;
    bool vehicleFootprintValid{false}; // sampled screen; false includes not audited
    std::string geometryOnlyReasons;
};

struct PrecomputedLaneNetwork {
    std::vector<PrecomputedLanePoint> points; // point.id is the vector index
    std::map<LaneKey, PrecomputedLane, PrecomputedLaneKeyLess> lanes;
    std::vector<PrecomputedLaneConnection> connections;
    std::vector<PrecomputedLaneTransition> transitions;
};

// Transactional load: reject malformed/dangling references without changing out.
// This does not enable the network in the live planner or validate source hashes;
// the manifest records the XODR SHA256 for the future runtime integration.
bool loadPrecomputedLaneNetwork(const std::filesystem::path& directory,
                                PrecomputedLaneNetwork& out,
                                std::string* error = nullptr);
