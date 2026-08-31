#include "../src/planning/global/reference_path_generator.hpp"
#include <iostream>
int main(int argc, char** argv) {
    PathGeneratorData data;
    if (!loadMap(data, argv[1], 0.5)) return 1;
    if (!loadCheckpointsCsv(data, argv[2])) return 1;
    for (const auto& cp : data.checkpoints) {
        auto candidates = findContainingLanes(data, cp.x, cp.y);
        std::cout << "seq=" << cp.sequence << " (" << cp.x << "," << cp.y << ") candidates=" << candidates.size() << "\n";
        for (const auto& c : candidates) {
            std::cout << "   road=" << c.roadId << " lane=" << c.laneId
                 << " section=" << c.sectionIndex << " roadS=" << c.roadS
                 << " dist=" << c.distance << " width=" << c.laneWidth << "\n";
        }
    }
}
