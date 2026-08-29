#include "planning/local/local_path_planner.hpp"

void generateLocalPath(LocalPathData& output,
                       const PathPlannerData& globalPath,
                       std::size_t nearestIndex,
                       double horizon) {
    output = {};
    if (globalPath.waypoints.empty() ||
        nearestIndex >= globalPath.waypoints.size()) {
        return;
    }

    output.beginIndex = nearestIndex;
    const double endS = globalPath.waypoints[nearestIndex].s + horizon;
    std::size_t endIndex = nearestIndex;
    while (endIndex + 1 < globalPath.waypoints.size() &&
           globalPath.waypoints[endIndex].s < endS) {
        ++endIndex;
    }
    output.endIndex = endIndex;
    output.points.assign(globalPath.waypoints.begin() + nearestIndex,
                         globalPath.waypoints.begin() + endIndex + 1);
    output.valid = output.points.size() >= 2;
}
