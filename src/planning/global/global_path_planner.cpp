#include "planning/global/global_path_planner.hpp"

#include "planning/global/reference_path_generator.hpp"

bool buildGlobalPath(PathPlannerData& planner,
                     PathGeneratorData& map,
                     const std::string& checkpointCsv) {
    if (!loadCheckpointsCsv(map, checkpointCsv)) return false;
    planner.waypoints = makePath(map);
    planner.nearestIndex = 0;
    planner.latestPlan = {};
    return !planner.waypoints.empty();
}
