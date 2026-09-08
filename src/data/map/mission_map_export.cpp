#include "planning/global/mission_corridor.hpp"
#include "planning/global/reference_path_generator.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

bool exportMissionMap(const PathGeneratorData& map, const std::string& directory) {
    namespace fs = std::filesystem;
    fs::create_directories(directory);
    const auto key = [](std::ostream& out, const LaneKey& k) {
        out << '[' << k.roadId << ',' << k.sectionIndex << ',' << k.laneId << ']';
    };
    std::ofstream geometry(fs::path(directory)/"lane_geometry.json");
    std::ofstream topology(fs::path(directory)/"lane_topology.json");
    std::ofstream graph(fs::path(directory)/"lane_graph.json");
    geometry << std::setprecision(17) << '[';
    bool comma = false;
    for (const auto& [k, segment] : map.laneSegments) {
        if (segment.points.empty()) continue;
        const auto& road = map.roads.at(k.roadId);
        const auto& section = road.sections.at(k.sectionIndex);
        const auto& lane = section.lanes.at(k.laneId);
        if (comma) geometry << ',';
        comma = true;
        geometry << "{\"key\":"; key(geometry, k);
        geometry << ",\"junction\":" << road.junction << ",\"points\":[";
        auto samples = segment.points;
        if (k.sectionIndex + 1 < static_cast<int>(road.sections.size())) {
            Waypoint endpoint = samples.back();
            endpoint.roadS = std::nextafter(segment.endS, segment.startS);
            const auto ref = roadToWorld(road, endpoint.roadS);
            const auto offset = laneOffset(road, section, k.laneId, endpoint.roadS);
            endpoint.x = ref.x-offset*std::sin(ref.yaw);
            endpoint.y = ref.y+offset*std::cos(ref.yaw);
            endpoint.yaw = ref.yaw+(k.laneId>0?std::acos(-1.):0.);
            if (k.laneId>0) samples.insert(samples.begin(), endpoint);
            else samples.push_back(endpoint);
        }
        bool pointComma = false;
        for (const auto& p : samples) {
            if (pointComma) geometry << ',';
            pointComma = true;
            const auto ref = roadToWorld(road, p.roadS);
            const auto offset = laneOffset(road, section, k.laneId, p.roadS);
            const auto width = evaluatePiecewise(lane.widths, p.roadS, 0.);
            geometry << '[' << p.x << ',' << p.y << ',' << p.yaw << ','
                     << p.roadS << ',' << width;
            for (const double edge : {offset-width*.5, offset+width*.5})
                geometry << ',' << ref.x-edge*std::sin(ref.yaw)
                         << ',' << ref.y+edge*std::cos(ref.yaw);
            geometry << ']';
        }
        geometry << "]}";
    }
    geometry << "]\n";
    topology << '['; graph << '[';
    bool topComma = false, graphComma = false;
    for (const auto& [from, next] : map.laneGraph) for (const auto& to : next) {
        if (graphComma) graph << ',';
        graphComma = true;
        graph << '['; key(graph, from); graph << ','; key(graph, to); graph << ']';
        if (from.roadId == to.roadId && from.sectionIndex == to.sectionIndex) continue;
        if (topComma) topology << ',';
        topComma = true;
        topology << '['; key(topology, from); topology << ',';
        key(topology, to); topology << ']';
    }
    topology << "]\n"; graph << "]\n";
    geometry.flush(); topology.flush(); graph.flush();
    return geometry.good() && topology.good() && graph.good();
}
