#pragma once

#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

// Checkpoint는 정확히 밟을 좌표/차선이 아니라 통과할 road 구역을 나타낸다.
// 같은 위치에 여러 connector road가 겹칠 수 있으므로, 진행방향과 맞는
// road 후보와 그 road에서 같은 방향으로 주행하는 모든 lane을 보관한다.
struct MissionCheckpointData {
    Checkpoint checkpoint;
    std::vector<int> roadIds;
    std::set<LaneKey> lanes;
};

// 두 교차로 진출 anchor 사이에서 사용할 수 있는 굵은 회랑.
// allowedLanes는 최단
// 연결과 지나치게 우회하지 않는 대안 연결에 포함되는 모든 차선이다.
// targetLanes는 다음 checkpoint 부근에서만 반드시 도달해야 하는 gateway다.
struct MissionLegData {
    std::size_t fromCheckpoint{};
    // 교차로 진입 checkpoint. 정확히 밟을 lane 목표가 아니라,
    // 이 경계 끝에서만 다음 exit road로 연결되는 lane을 선택한다.
    std::size_t gatewayCheckpoint{};
    std::size_t toCheckpoint{};
    bool valid{};
    double bestCost{};
    std::set<LaneKey> allowedLanes;
    std::set<LaneKey> targetLanes;
    std::vector<LaneKey> preferredRoute;
    // 출발 전에 corridor의 각 사용 가능 lane에서 gateway까지
    // 연결되는 전체 lane chain을 검증·고정한다. 런타임에는
    // 현재 lane의 캐시를 읽어 프레임별 분기 선택 흔들림을 막는다.
    std::map<LaneKey, std::vector<LaneKey>> routesToTargetByLane;
};

struct MissionCorridorData {
    bool initialized{};
    bool valid{};
    std::vector<MissionCheckpointData> checkpoints;
    std::vector<MissionLegData> legs;
    std::size_t activeLegIndex{};
    double minimumTargetDistance{};
    int targetRoadStableFrames{};
    bool targetRoadObserved{};
};

// CSV checkpoint를 road 구역으로 변환하고, 각 구간의 자유 주행 회랑과
// 마지막 gateway를 lane graph에서 계산한다.
bool initializeMissionCorridor(MissionCorridorData& output,
                               PathGeneratorData& map,
                               const std::string& checkpointCsv);

// Checkpoint는 반드시 좌표를 밟지 않아도 된다. 목표 구역 진입, 근접 통과,
// 또는 충분히 가까워졌다가 지나친 사실을 이용해 구간을 단조롭게 전진시킨다.
void updateMissionCorridorProgress(MissionCorridorData& mission,
                                   const WorldState& world);

const MissionLegData* activeMissionLeg(const MissionCorridorData& mission);

// 횡방향 차선변경 edge는 제외하고 실제 lane successor/junction 연결만 따라
// 현재 차선에서 활성 checkpoint gateway까지 가는 경로를 찾는다.
bool findMissionForwardRoute(const PathGeneratorData& map,
                             const MissionCorridorData& mission,
                             const LaneKey& start,
                             std::vector<LaneKey>& route,
                             LaneKey& reachedLane);

// 차선변경 edge까지 포함해 출발 전에 계산한 활성 mission의 전체 lane 경로.
// Local planner는 첫 lateral edge 전까지 종방향으로 추종하고, 같은 section의
// 연속 lateral edge는 하나의 다중-hop 기동으로 묶은 뒤 successor/connector까지
// committed path에 연결한다.
bool findMissionLaneRoute(const PathGeneratorData& map,
                          const MissionCorridorData& mission,
                          const LaneKey& start,
                          std::vector<LaneKey>& route);

// 실행마다 output 폴더에 굵은 자유 회랑, 얇은 gateway, checkpoint와 기존
// guide path를 함께 저장한다. SVG는 별도 그래픽 라이브러리가 필요 없다.
bool saveMissionCorridorArtifacts(const MissionCorridorData& mission,
                                  const PathGeneratorData& map,
                                  const std::vector<Waypoint>& guidePath,
                                  const std::string& svgPath,
                                  const std::string& csvPath);
