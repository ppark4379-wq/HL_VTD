#include "data/map/precomputed_lane_network.hpp"
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: check_precomputed_network <output directory>\n";
        return 2;
    }
    PrecomputedLaneNetwork network;
    std::string error;
    if (!loadPrecomputedLaneNetwork(argv[1], network, &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    std::size_t continuous = 0;
    for (const auto& c : network.connections) continuous += c.status == "continuous";
    std::cout << "Loaded C++ PrecomputedLaneNetwork: " << network.lanes.size()
              << " lanes, " << network.points.size() << " points, "
              << network.connections.size() << " declared connections ("
              << continuous << " continuous), " << network.transitions.size()
              << " merge/split/continuation transitions with shared sample anchors.\n";
}
