#pragma once

#include "data/gt/gt_structs.hpp"
#include "planning_structs.hpp"

#include <optional>
#include <string>
#include <vector>

// =============================================================================
// 외부에서 호출하는 경로 생성 함수
// =============================================================================

// 대회 CSV의 checkpoint를 읽어 data.checkpoints에 저장한다.
bool loadCheckpointsCsv(PathGeneratorData& data, const std::string& csvPath);

// XODR을 읽고 driving lane 중심선과 차선 연결정보를 준비한다.
bool loadMap(PathGeneratorData& data, const std::string& xodrPath,
             double sampleInterval = 0.5);

// VTD 월드 좌표 (x, y)를 가장 가까운 road/lane에 매칭한다.
LaneMatch matchWorldToLane(
    const PathGeneratorData& data,
    double x,
    double y,
    std::optional<double> heading = std::nullopt);

// 월드 좌표가 XODR 차선 폭 내부에 포함되는 road/lane을 찾는다.
std::vector<LaneMatch> findContainingLanes(const PathGeneratorData& data,
                                           double x, double y);

// 수신한 GT의 Ego 월드 위치와 heading을 현재 road/lane에 매칭한다.
LaneMatch matchEgoToLane(const PathGeneratorData& data, const GtFrame& gt);

// CSV의 모든 checkpoint를 각각 road/lane에 매칭한다.
std::vector<LaneMatch> matchCheckpointsToLanes(const PathGeneratorData& data);

// 매칭된 checkpoint들을 legal lane 연결로 이어 전역경로를 생성한다.
std::vector<Waypoint> makePath(const PathGeneratorData& data);

// 생성한 전역경로를 디버깅과 시각화용 CSV로 저장한다.
bool savePath(
    const std::vector<Waypoint>& path,
    const std::string& outputPath);

// 모든 driving lane 중심선을 지도 시각화용 CSV로 저장한다.
bool saveLaneMap(
    const PathGeneratorData& data,
    const std::string& outputPath);

// =============================================================================
// 경로 생성 내부 단계 함수
// loadMap()과 makePath()가 아래 함수들을 순서대로 호출한다.
// =============================================================================

// 구간별 다항식에서 road s에 해당하는 값을 계산한다.
double evaluatePiecewise(const std::vector<Polynomial>& polynomials,
                         double s, double fallback = 0.0);

// geometry 시작점에서 ds만큼 진행한 기준선의 월드 x/y/yaw를 계산한다.
RoadPose evaluateGeometry(const Geometry& geometry, double ds);

// XODR의 road 목록을 읽는다.
void parseRoads(PathGeneratorData& data, const XmlElement& root);

// 한 road의 predecessor와 successor를 읽는다.
void parseRoadLinks(const XmlElement& node, Road& road);

// 한 road의 line/arc/spiral/poly3 geometry를 읽는다.
void parseGeometries(const XmlElement& node, Road& road);

// 한 road의 laneSection, lane 폭 및 laneLink를 읽는다.
void parseLanes(const XmlElement& node, Road& road);

// junction의 incomingRoad, connectingRoad 및 laneLink를 읽는다.
void parseJunctions(PathGeneratorData& data, const XmlElement& root);

// XML의 다항식 속성 a, b, c, d를 읽는다.
Polynomial parsePolynomial(const XmlElement& node,
                           const std::string& sName);

// road의 특정 s에서 기준선의 월드 x/y/yaw를 계산한다.
RoadPose roadPose(const Road& road, double s);

// road 기준선에서 지정한 lane 중심까지의 횡방향 거리를 계산한다.
double laneOffset(const Road& road, const LaneSection& section,
                  int laneId, double roadS);

// 월드 좌표를 road 기준선에 투영해 정확한 roadS와 부호 있는 횡방향 거리 t를 구한다.
struct RoadProjection { double s{}; double t{}; double distance{}; };
RoadProjection projectPointToRoad(const Road& road, double x, double y,
                                  double sGuess, double window);

// s 위치에서 lane의 roadMark laneChange 값을 찾는다 (없으면 "none").
std::string laneChangeMarkAt(const LaneDefinition& lane, double s);

// fromLaneId에서 toLaneId로 차선변경이 roadMark상 허용되는지 판단한다.
bool laneChangeAllowed(const LaneSection& section, int fromLaneId,
                       int toLaneId, double s);

// 모든 driving lane 중심선을 일정 간격의 점으로 만든다.
void sampleLaneCenters(PathGeneratorData& data);

// lane 연결 하나를 그래프에 추가한다.
void addLaneEdge(PathGeneratorData& data, const LaneKey& from,
                 const LaneKey& to);

// successor와 junction laneLink로 legal lane 그래프를 만든다.
void buildLaneGraph(PathGeneratorData& data);

// 인접 checkpoint의 두 lane 사이 연결을 찾는다.
std::vector<LaneKey> connectLanes(const PathGeneratorData& data,
                                  const LaneKey& start,
                                  const LaneKey& goal);

// lane route를 따라 start에서 goal까지 주행하는 대략적인 거리를 계산한다.
double calculateRouteDistance(const PathGeneratorData& data,
                              const std::vector<LaneKey>& route,
                              const LaneMatch& start,
                              const LaneMatch& goal);

// 찾은 lane 연결의 중심선 점들을 최종 경로에 추가한다.
void appendRoute(const PathGeneratorData& data,
                 const std::vector<LaneKey>& route,
                 const LaneMatch& start, const LaneMatch& goal,
                 std::vector<Waypoint>& output);

// 두 경로 조각 사이의 빈 공간을 일정 간격의 점으로 채운다.
void appendInterpolatedGap(const Waypoint& start, const Waypoint& end,
                           double interval, std::vector<Waypoint>& output);

// 같은 road 구간의 인접 차선 사이를 부드러운 곡선으로 연결한다.
size_t appendSmoothLaneChange(const std::vector<Waypoint>& currentLane,
                              const std::vector<Waypoint>& nextLane,
                              size_t begin,
                              std::vector<Waypoint>& output);

// 이어 붙인 경로의 연속 중복점을 제거한다.
void removeDuplicates(std::vector<Waypoint>& path);

// 최종 경로 각 점의 yaw와 누적거리 s를 다시 계산한다.
void updateYawAndS(std::vector<Waypoint>& path);
