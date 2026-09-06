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
    // 보호구역 기준점에서 런타임에 만든 구간인지 구분한다. 법정 제한속도와
    // 별개로 제어 목표(speed.school_zone_target_kph)를 적용할 때 사용한다.
    bool schoolZone{};
    // alignStaticZonesToRoute는 로드된 시설을 삭제하지 않는다. 현재 경로에
    // 적용할 시설인지 여부만 이 플래그로 표시한다.
    bool routeRelevant{true};
    bool derivedFromReferencePoints{};
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
    bool routeRelevant{true};
};

struct Crosswalk {
    int id{-1};
    // 생성 파일은 지도 전체 횡단보도를 담는다. OpenDRIVE road id는
    // 참고 정보일 뿐이며, 접속도로 ID가 달라도 경로가 폴리곤을 통과하면
    // 현재 경로의 횡단보도로 활성화한다(-1은 수동 설정).
    int roadId{-1};
    std::vector<Point2D> polygon;
    double entryRouteS{};
    double exitRouteS{};
    bool routeRelevant{true};
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
bool isSchoolZoneActive(const StaticZoneData& zones,
                        const Point2D& position,
                        double routeS);

// 현재 위치에 적용되는 제한속도. polygon 구간과 route_s 구간을 모두 본다.
// routeS가 유효하지 않으면 음수를 넘겨 route 구간 검사를 건너뛴다.
double findZoneSpeedLimit(const StaticZoneData& zones,
                          const Point2D& position,
                          double routeS,
                          double fallbackLimit);
bool isInsideCrosswalk(const StaticZoneData& zones,
                       int crosswalkId,
                       const Point2D& position);
