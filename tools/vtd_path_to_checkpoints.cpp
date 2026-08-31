#include "planning/global/reference_path_generator.hpp"
#include "path_generation/vtd_path_generator.hpp"
#include "utils/csv_utils.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::vector<VtdPathPoint> loadVtdPath(const std::string& csvPath) {
    std::ifstream input(csvPath);
    if (!input) throw std::runtime_error("cannot open VTD path CSV: " + csvPath);
    std::string line;
    std::getline(input, line);
    std::vector<VtdPathPoint> points;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() < 3) {
            throw std::runtime_error("invalid VTD path CSV row: " + line);
        }
        double sequence = 0.0;
        double roadId = 0.0;
        double roadS = 0.0;
        if (!parseDouble(fields[0], sequence) ||
            !parseDouble(fields[1], roadId) ||
            !parseDouble(fields[2], roadS)) {
            throw std::runtime_error("non-numeric VTD path CSV row: " + line);
        }
        points.push_back({static_cast<int>(sequence),
                          static_cast<int>(roadId), roadS});
    }
    if (points.size() < 2) {
        throw std::runtime_error("VTD path CSV requires at least two rows");
    }
    return points;
}

void saveCheckpoints(const std::vector<Waypoint>& checkpoints,
                     const std::string& outputPath) {
    std::ofstream output(outputPath);
    if (!output) {
        throw std::runtime_error("cannot write checkpoints: " + outputPath);
    }
    output << "seq,x,y\n" << std::fixed << std::setprecision(8);
    for (std::size_t index = 0; index < checkpoints.size(); ++index) {
        output << index + 1 << ',' << checkpoints[index].x << ','
               << checkpoints[index].y << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::cerr << "Usage: " << argv[0]
                  << " MAP.xodr VTD_PATH.csv START_LANE CHECKPOINTS.csv "
                     "ROUTE.csv LANE_MAP.csv\n";
        return 1;
    }
    try {
        PathGeneratorData data;
        if (!loadMap(data, argv[1], 0.5)) return 2;
        const std::vector<VtdPathPoint> vtdPath = loadVtdPath(argv[2]);
        const int startLane = std::stoi(argv[3]);
        const VtdRouteResult result =
            makePathFromVtdRoute(data, vtdPath, startLane);
        if (result.path.empty() || result.checkpoints.size() != vtdPath.size()) {
            return 3;
        }
        saveCheckpoints(result.checkpoints, argv[4]);
        if (!savePath(result.path, argv[5]) || !saveLaneMap(data, argv[6])) {
            return 4;
        }
        std::cout << "[vtd-path] generated " << result.path.size()
                  << " route points\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[vtd-path] ERROR: " << error.what() << '\n';
        return 5;
    }
}
