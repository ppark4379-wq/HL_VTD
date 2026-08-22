#include "planning/planning_process.hpp"

#include "planning/path_planner.hpp"

// 전역경로 CSV의 각 행을 vector<Waypoint>에 저장한다.
bool initializePlanning(PathPlannerData& planner,
                        const std::string& referencePathCsv) {
    planner.nearestIndex = 0;
    planner.latestPlan = {};
    return loadWaypointsCsv(planner.waypoints, referencePathCsv);
}

// 경로추종 플래닝의 최상위 호출 흐름.
PathPlan runPlanningProcess(PathPlannerData& planner,
                            const GtFrame& gt) {
    return runPurePursuit(planner, gt);
}
