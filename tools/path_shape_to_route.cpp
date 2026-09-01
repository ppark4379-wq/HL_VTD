#include "planning/global/reference_path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

std::vector<Waypoint> interpolatePathShape(PathGeneratorData& data,
                                           double interval) {
    std::vector<Waypoint> route;
    double cumulativeS = 0.0;

    for (std::size_t segmentIndex = 0;
         segmentIndex + 1 < data.checkpoints.size(); ++segmentIndex) {
        const Checkpoint& start = data.checkpoints[segmentIndex];
        const Checkpoint& end = data.checkpoints[segmentIndex + 1];
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-9) continue;

        const int steps = std::max(1, static_cast<int>(std::ceil(length / interval)));
        for (int step = segmentIndex == 0 ? 0 : 1; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / steps;
            Waypoint point;
            point.x = start.x + ratio * dx;
            point.y = start.y + ratio * dy;
            point.yaw = std::atan2(dy, dx);
            if (!route.empty()) {
                cumulativeS += std::hypot(point.x - route.back().x,
                                          point.y - route.back().y);
            }
            point.s = cumulativeS;

            const LaneMatch lane = worldToLane(data, point.x, point.y);
            if (lane.valid) {
                point.roadId = lane.roadId;
                point.laneId = lane.laneId;
                point.roadS = lane.roadS;
            }
            route.push_back(point);
        }
    }

    return route;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " MAP.xodr CHECKPOINTS.csv ROUTE.csv\n";
        return 1;
    }

    PathGeneratorData data;
    if (!loadMap(data, argv[1], 0.5) ||
        !loadCheckpointsCsv(data, argv[2])) {
        return 2;
    }

    const std::vector<Waypoint> route = interpolatePathShape(data, 0.5);
    if (route.size() < 2 || !savePath(route, argv[3])) return 3;

    std::cout << "[path-shape] generated " << route.size()
              << " route points, length=" << route.back().s << "m\n";
    return 0;
}
