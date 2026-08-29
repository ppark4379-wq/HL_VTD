#include "../src/planning/global/reference_path_generator.hpp"

#include <iostream>


int main(int argc, char** argv) {
    if (argc != 3 && argc != 5) {
        std::cerr << "사용법: " << argv[0]
             << " MAP.xodr LANE_MAP.csv\n"
             << "또는: " << argv[0]
             << " MAP.xodr CHECKPOINTS.csv PATH.csv LANE_MAP.csv\n";
        return 1;
    }

    PathGeneratorData data;

    if (!loadMap(data, argv[1], 0.5)) return 1;

    // 인자가 2개이면 XODR 차선 중심점만 계산해 lane_map.csv로 저장한다.
    if (argc == 3) {
        if (!saveLaneMap(data, argv[2])) return 1;
        std::cout << "[test] " << data.laneSegments.size()
             << "개 driving lane 중심선 저장 완료\n";
        return 0;
    }

    // 인자가 4개이면 checkpoint 매칭과 전역경로 생성까지 수행한다.
    if (!loadCheckpointsCsv(data, argv[2])) return 1;

    const std::vector<Waypoint> path = makePath(data);
    if (path.empty()) {
        std::cerr << "[test] 경로 생성 실패\n";
        return 1;
    }

    if (!savePath(path, argv[3])) return 1;
    if (!saveLaneMap(data, argv[4])) return 1;

    std::cout << "[test] " << path.size() << "개 경로점 생성 완료\n";
    return 0;
}
