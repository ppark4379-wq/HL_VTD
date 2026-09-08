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
    // from -> gateway, gateway -> exit 각각의 최소 그래프 비용.
    // 교차로 진입/진출 쌍 중 한쪽만 XODR successor가 끊긴
    // 경우를 전체 거리에 묻히지 않고 판정할 수 있게 보관한다.
    double gatewayBestCost{};
    double exitBestCost{};
    double bestCost{};
    // The official sampled route is the immutable authority for which road
    // sections form this corridor and which exit is required. Its samples are
    // not necessarily lane-centre connected: an approximate competition path
    // may draw across two junction branches that XODR says are disconnected.
    // Legal lane selection therefore still comes from the restricted XODR
    // graph; the sampled guide is used only when that graph has a real gap.
    bool guideRouteAuthority{};
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
                               const std::string& checkpointCsv,
                               const std::vector<Waypoint>* guidePath = nullptr);

// Checkpoint는 반드시 좌표를 밟지 않아도 된다. 목표 구역 진입, 근접 통과,
// 또는 충분히 가까워졌다가 지나친 사실을 이용해 구간을 단조롭게 전진시킨다.
void updateMissionCorridorProgress(MissionCorridorData& mission,
                                   const WorldState& world);

const MissionLegData* activeMissionLeg(const MissionCorridorData& mission);

// XODR successor graph가 교차로 내부의 제공 경로 연결을 표현하지 못해,
// 가까운 두 seq 사이를 수백 m 이상 우회해야 하는지 판정한다. 이 구간은
// packaged map-derived guide geometry를 continuity source로 사용한다.
bool missionLegRequiresGuideContinuity(const MissionCorridorData& mission);

// 미션 전체는 정상이어도 현재 map-matched lane에서 선택한
// gateway/exit로 이어지는 제한 경로가 없으면 guide continuity가
// 필요하다. 분기 직전의 잘못된 차선에서 임의 직진 후보를
// gateway 후보로 오판하는 것을 막는다.
bool missionLaneRequiresGuideContinuity(const MissionCorridorData& mission,
                                        const LaneKey& currentLane);

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
