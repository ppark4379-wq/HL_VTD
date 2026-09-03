#pragma once

#include "utils/geometry_structs.hpp"

#include <string>
#include <vector>

struct Waypoint;

// 제한속도 구간. 두 가지 방식 중 하나로 지정한다.
//   polygon      : 도로 밖까지 포함하는 임의 영역 (주차장 등)
//   route_s 범위 : 전역경로 위 구간. 커브가 있어도 숫자 두 개면 끝나므로
//                  주행 구간(보호구역·제한속도 표지)에는 이쪽이 훨씬 간단하다.
// yaml에는 speed_limit_kph(km/h)로 적고, 로딩하면서 m/s로 바꿔 저장한다.
struct SpeedZone {
    std::string id;
    double speedLimit{};
    std::vector<Point2D> polygon;
    double routeSBegin{};
    double routeSEnd{};
    bool hasRouteRange{};
};

// VTD Scenario Editor에서 측정한 보호구역 기준점 원본이다. 이 점들은
// 진입/이탈 구간이 확정되기 전의 자료이므로 SpeedZone처럼 자동 적용하지 않는다.
// trackId/trackS는 VTD가 표시한 값이며 OpenDRIVE roadId/roadS와 다를 수 있다.
struct SchoolZoneReferencePoint {
    std::string id;
    Point2D position;
    int trackId{-1};
    int laneId{};
    double trackS{};
};

struct StopLine {
    int id{-1};
    int controllerId{-1};
    int roadId{-1};
    std::vector<int> laneIds;
    Point2D begin;
    Point2D end;
    double routeS{};
};

struct Crosswalk {
    int id{-1};
    std::vector<Point2D> polygon;
    double entryRouteS{};
    double exitRouteS{};
};

struct FinishLine {
    Point2D referencePoint;
    double routeS{};
    bool valid{};
};

struct StaticZoneData {
    std::vector<SpeedZone> speedZones;
    std::vector<SchoolZoneReferencePoint> schoolZoneReferencePoints;
    double schoolZoneReferenceSpeedLimit{};
    std::vector<StopLine> stopLines;
    std::vector<Crosswalk> crosswalks;
    std::vector<int> flashingRedControllerIds;
    FinishLine finish;
};

struct MapRuntimeConfig {
    std::string xodrPath;
    std::string staticFeaturesPath;
    std::string schoolZoneReferencePointsPath;
    StaticZoneData zones;
};

bool loadMapRuntimeConfig(MapRuntimeConfig& config,
                          const std::string& yamlPath);
bool isFlashingRedController(const StaticZoneData& zones,
                             int controllerId);
bool alignStaticZonesToRoute(StaticZoneData& zones,
                             const std::vector<Waypoint>& route);

// 현재 위치에 적용되는 제한속도. polygon 구간과 route_s 구간을 모두 본다.
// routeS가 유효하지 않으면 음수를 넘겨 route 구간 검사를 건너뛴다.
double findZoneSpeedLimit(const StaticZoneData& zones,
                          const Point2D& position,
                          double routeS,
                          double fallbackLimit);
bool isInsideCrosswalk(const StaticZoneData& zones,
                       int crosswalkId,
                       const Point2D& position);
