// Export actual XODR geometry and directed lane links without a simulator.
#include "planning/global/reference_path_generator.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace fs = std::filesystem;
void key(std::ostream& out, const LaneKey& k) {
    out << '[' << k.roadId << ',' << k.sectionIndex << ',' << k.laneId << ']';
}
int main(int argc, char** argv) {
    if(argc!=3 && argc!=4) { std::cerr << "Usage: export_mission_corridors REPO OUTPUT [--geometry-only]\n"; return 2; }
    if(argc==4 && std::string(argv[3])!="--geometry-only") return 2;
    const fs::path root=fs::absolute(argv[1]), dest=fs::absolute(argv[2]);
    fs::create_directories(dest);
    PathGeneratorData map;
    if(!loadMap(map,(root/"src/data/map/HL_FMA_VTD_LivingLab.xodr").string())) return 1;
    std::ofstream geom(dest/"lane_geometry.json");
    geom << std::setprecision(10) << '[';
    bool comma=false;
    for(const auto& [k,segment]:map.laneSegments) {
        if(segment.points.empty()) continue;
        if(comma) geom << ','; comma=true;
        const auto& road=map.roads.at(k.roadId);
        const auto& section=road.sections.at(k.sectionIndex);
        const auto& lane=section.lanes.at(k.laneId);
        geom << "{\"key\":"; key(geom,k);
        geom << ",\"junction\":" << road.junction << ",\"points\":[";
        auto samples=segment.points;
        // Runtime center samples intentionally omit non-final section endpoints.
        // A surface polygon must include the XODR one-sided boundary, otherwise
        // every laneSection has an artificial ~0.5m unpainted strip.
        if(k.sectionIndex+1<static_cast<int>(road.sections.size())) {
            Waypoint endpoint=segment.points.back();
            endpoint.roadS=std::nextafter(segment.endS,segment.startS);
            const auto ref=roadToWorld(road,endpoint.roadS);
            const auto t=laneOffset(road,section,k.laneId,endpoint.roadS);
            endpoint.x=ref.x-t*std::sin(ref.yaw);endpoint.y=ref.y+t*std::cos(ref.yaw);
            endpoint.yaw=ref.yaw+(k.laneId>0?std::acos(-1.):0.);
            if(k.laneId>0) samples.insert(samples.begin(),endpoint); else samples.push_back(endpoint);
        }
        bool pc=false;
        for(const auto& p:samples) {
            if(pc) geom << ','; pc=true;
            const auto ref=roadToWorld(road,p.roadS);
            const auto center=laneOffset(road,section,k.laneId,p.roadS);
            const auto width=evaluatePiecewise(lane.widths,p.roadS,0.);
            const auto edge=[&](double t) {
                geom << ref.x-t*std::sin(ref.yaw) << ',' << ref.y+t*std::cos(ref.yaw);
            };
            geom << '[' << p.x << ',' << p.y << ',' << p.yaw << ',' << p.roadS << ',' << width << ',';
            edge(center-width*.5); geom << ','; edge(center+width*.5); geom << ']';
        }
        geom << "]}";
    }
    geom << "]\n";
    // Keep longitudinal XODR topology separate from lateral planning edges.
    std::ofstream graph(dest/"lane_topology.json");
    graph << '['; bool gc=false;
    for(const auto& [from, next]:map.laneGraph) {
        for(const auto& to:next) {
            if(from.roadId==to.roadId && from.sectionIndex==to.sectionIndex) continue;
            if(gc) graph << ','; gc=true;
            graph << '['; key(graph,from); graph << ','; key(graph,to); graph << ']';
        }
    }
    graph << "]\n";
    std::ofstream fullGraph(dest/"lane_graph.json");
    fullGraph << '['; bool fc=false;
    for(const auto& [from,next]:map.laneGraph) for(const auto& to:next) {
        if(fc) fullGraph << ','; fc=true;
        fullGraph << '['; key(fullGraph,from); fullGraph << ','; key(fullGraph,to); fullGraph << ']';
    }
    fullGraph << "]\n";
    return 0;
}
