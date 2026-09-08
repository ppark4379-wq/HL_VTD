// Export actual XODR geometry and directed lane links without a simulator.
#include "planning/global/reference_path_generator.hpp"
#include "planning/global/mission_corridor.hpp"
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
int main(int argc, char** argv) {
    if(argc!=3 && argc!=4) { std::cerr << "Usage: export_mission_corridors REPO OUTPUT [--geometry-only]\n"; return 2; }
    if(argc==4 && std::string(argv[3])!="--geometry-only") return 2;
    const fs::path root=fs::absolute(argv[1]), dest=fs::absolute(argv[2]);
    fs::create_directories(dest);
    PathGeneratorData map;
    if(!loadMap(map,(root/"src/data/map/HL_FMA_VTD_LivingLab.xodr").string())) return 1;
    return exportMissionMap(map,dest.string()) ? 0 : 1;
}
