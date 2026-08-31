#pragma once

#include "planning/planning_structs.hpp"

#include <vector>

// VTD Scenario XML의 <Path><Waypoint TrackId="..." s="..."/> 한 점.
// VTD LivingLab의 TrackId는 같은 XODR의 road id로 사용한다.
struct VtdPathPoint {
    int sequence{};
    int roadId{};
    double roadS{};
};

struct VtdRouteResult {
    std::vector<Waypoint> path;
    std::vector<Waypoint> checkpoints;
    std::vector<LaneMatch> matches;
};

// OpenDRIVE road/lane/s를 해당 driving lane 중심의 정확한 월드 좌표로 바꾼다.
Waypoint roadLaneToWaypoint(const PathGeneratorData& data,
                            int roadId, int laneId, double roadS);

// VTD road/s 목록과 시작 lane을 기존 legal lane graph에 직접 연결한다.
VtdRouteResult makePathFromVtdRoute(
    const PathGeneratorData& data,
    const std::vector<VtdPathPoint>& vtdPath,
    int startLane);
