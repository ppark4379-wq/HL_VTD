// XODR의 <object>/<signal>에서 정지선·횡단보도·신호등·노면 제한속도 표시를
// 뽑아 config/maps/living_lab.yaml을 생성하는 오프라인 도구.
//
// XODR은 이들 위치를 (roadId, s, t) 도로 기준 좌표로 저장한다.
// 실시간 제어에 필요한 것은 월드 x/y와 전역경로 누적거리 routeS이므로
// roadToWorld()로 정변환한 뒤 reference path에 투영한다.
//
// 사용법:
//   extract_static_zones [--xodr PATH] [--route CSV] [--out YAML]

#include "data/map/map_structs.hpp"
#include "planning/global/reference_path_generator.hpp"
#include "planning/local/path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

// 정지선과 횡단보도는 지도 전체를 generated YAML에 보존하고 실제 주행
// 경로에 맞추는 일은 alignStaticZonesToRoute()가 런타임에 한다. 오프라인
// 생성 시 한 경로로 잘라 버리면 다른 시나리오 route의 시설이 영구 누락된다.
constexpr double kSpeedMarkLateralLimit = 6.0;
constexpr double kSignalLateralLimit = 40.0;

struct RouteProjection {
    bool valid{};
    double routeS{};
    double lateral{};
};

struct WorldFeature {
    Point2D center;
    double heading{};
    double length{};
    double width{};
};

// (roadId, s, t) -> 월드 좌표. lane centerline 생성과 동일한 부호 규약을 쓴다.
bool roadObjectToWorld(WorldFeature& output, const PathGeneratorData& map,
             const RoadObject& object) {
    const auto found = map.roads.find(object.roadId);
    if (found == map.roads.end()) return false;
    const RoadPose pose = roadToWorld(found->second, object.s);
    output.center.x = pose.x - object.t * std::sin(pose.yaw);
    output.center.y = pose.y + object.t * std::cos(pose.yaw);
    output.heading = pose.yaw + object.hdg;
    output.length = object.length;
    output.width = object.width;
    return true;
}

bool roadSignalToWorld(Point2D& output, const PathGeneratorData& map,
                   const RoadSignal& signal) {
    const auto found = map.roads.find(signal.roadId);
    if (found == map.roads.end()) return false;
    const RoadPose pose = roadToWorld(found->second, signal.s);
    output.x = pose.x - signal.t * std::sin(pose.yaw);
    output.y = pose.y + signal.t * std::cos(pose.yaw);
    return true;
}

// 월드 좌표를 전역경로에 투영해 누적거리와 횡거리를 구한다.
RouteProjection worldToRoute(const std::vector<Waypoint>& route,
                               const Point2D& point) {
    RouteProjection result;
    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        const double dx = route[i + 1].x - route[i].x;
        const double dy = route[i + 1].y - route[i].y;
        const double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared < 1e-9) continue;
        double ratio = ((point.x - route[i].x) * dx +
                        (point.y - route[i].y) * dy) / lengthSquared;
        ratio = std::clamp(ratio, 0.0, 1.0);
        const double projectedX = route[i].x + ratio * dx;
        const double projectedY = route[i].y + ratio * dy;
        const double distance =
            std::hypot(point.x - projectedX, point.y - projectedY);
        if (distance >= best) continue;
        best = distance;
        result.valid = true;
        result.routeS = route[i].s + ratio * std::sqrt(lengthSquared);
        result.lateral = distance;
    }
    return result;
}

// 중심·heading·치수에서 사각형 네 꼭짓점을 만든다.
std::vector<Point2D> makeRectangle(const WorldFeature& feature) {
    const double halfLength = 0.5 * feature.length;
    const double halfWidth = 0.5 * feature.width;
    const double cosHeading = std::cos(feature.heading);
    const double sinHeading = std::sin(feature.heading);
    const double localX[4] = {halfLength, halfLength, -halfLength, -halfLength};
    const double localY[4] = {halfWidth, -halfWidth, -halfWidth, halfWidth};
    std::vector<Point2D> corners;
    corners.reserve(4);
    for (int i = 0; i < 4; ++i) {
        corners.push_back(
            {feature.center.x + localX[i] * cosHeading - localY[i] * sinHeading,
             feature.center.y + localX[i] * sinHeading + localY[i] * cosHeading});
    }
    return corners;
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

int sectionIndexAtRoadS(const Road& road, double s) {
    int index = 0;
    for (std::size_t i = 0; i < road.sections.size(); ++i) {
        if (road.sections[i].s <= s + 1e-9) index = static_cast<int>(i);
        else break;
    }
    return index;
}

// 노면표시가 그려진 차선을 그 표시의 (road, s, t)로 직접 구한다.
//
// 월드 좌표를 가장 가까운 차선 중심선에 매칭하면(worldToLane) 교차로처럼
// 여러 도로의 차선이 겹치는 곳에서 다른 road의 차선을 골라, road_id는 이
// 도로인데 lane_id는 남의 도로 것인 모순된 결과가 나온다. 실제로 road 418의
// t=+10.30 정지선이 lane -3으로 잘못 배정돼 신호 정지가 통째로 실패했다.
//
// t는 이미 이 도로 기준선 기준값이므로 같은 도로의 차선 중심 t와 비교하면
// 탐색도 필요 없고 모순도 생기지 않는다.
int laneIdAtRoadT(const Road& road, double s, double t) {
    if (road.sections.empty()) return 0;
    const LaneSection& section =
        road.sections[static_cast<std::size_t>(sectionIndexAtRoadS(road, s))];
    int bestLaneId = 0;
    double bestDelta = std::numeric_limits<double>::max();
    for (const auto& entry : section.lanes) {
        const int laneId = entry.first;
        if (laneId == 0 || entry.second.type != "driving") continue;
        const double center = laneOffset(road, section, laneId, s);
        const double delta = std::abs(center - t);
        if (delta >= bestDelta) continue;
        bestDelta = delta;
        bestLaneId = laneId;
    }
    return bestLaneId;
}

bool isStopLine(const RoadObject& object) {
    return contains(object.name, "StopLine");
}

// type="crosswalk"로 태깅된 것과, 이름이 횡단보도 노면표시인 것을 함께 본다.
// RM_532_* 는 VTD 화면에서 실제 횡단보도인지 반드시 눈으로 확인할 것.
bool isCrosswalk(const RoadObject& object) {
    return object.type == "crosswalk" || contains(object.name, "RM_532_");
}

double speedMarkingLimitKph(const RoadObject& object) {
    if (contains(object.name, "roadmark_speed_30")) return 30.0;
    if (contains(object.name, "RM_517_50")) return 50.0;
    if (contains(object.name, "RM_517_30")) return 30.0;
    return 0.0;
}

// 정지선과 같은 road에 있고 s가 가장 가까우며 좌우가 같은 쪽인 신호등을 고른다.
// 어디까지나 추정이므로 생성된 yaml은 반드시 눈으로 검증해야 한다.
int guessControllerId(const PathGeneratorData& map,
                      const RoadObject& stopLine) {
    int bestId = -1;
    double bestDistance = std::numeric_limits<double>::max();
    for (const auto& signal : map.roadSignals) {
        if (signal.roadId != stopLine.roadId) continue;
        if (!signal.dynamic) continue;
        if (stopLine.t * signal.t < 0.0) continue;  // 반대편 차선 신호 제외
        const double distance = std::abs(signal.s - stopLine.s);
        if (distance >= bestDistance) continue;
        bestDistance = distance;
        bestId = signal.id;
    }
    if (bestDistance > 80.0) return -1;
    const auto controller = map.signalToController.find(bestId);
    return controller == map.signalToController.end()
               ? -1
               : controller->second;
}

std::string polygonText(const std::vector<Point2D>& polygon) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        if (i > 0) stream << "; ";
        stream << polygon[i].x << ',' << polygon[i].y;
    }
    return stream.str();
}

std::string argumentValue(int argc, char* argv[], const std::string& flag,
                          const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (flag == argv[i]) return argv[i + 1];
    }
    return fallback;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string xodrPath = argumentValue(
        argc, argv, "--xodr", "src/data/map/HL_FMA_VTD_LivingLab.xodr");
    const std::string routePath = argumentValue(
        argc, argv, "--route", "output/reference_path.csv");
    const std::string outputPath = argumentValue(
        argc, argv, "--out", "config/maps/living_lab.generated.yaml");

    PathGeneratorData map;
    if (!loadMap(map, xodrPath)) {
        std::cerr << "[extract] cannot load XODR: " << xodrPath << '\n';
        return 1;
    }

    std::vector<Waypoint> route;
    if (!loadWaypointsCsv(route, routePath)) {
        std::cerr << "[extract] cannot load route CSV: " << routePath << '\n';
        return 1;
    }

    std::ofstream output(outputPath);
    if (!output) {
        std::cerr << "[extract] cannot write: " << outputPath << '\n';
        return 1;
    }
    output << std::fixed << std::setprecision(3);
    output << "# extract_static_zones가 XODR에서 자동 생성한 파일이다.\n"
           << "# 직접 수정하지 말고 도구를 다시 실행할 것.\n"
           << "# source xodr : " << xodrPath << '\n'
           << "# source route: " << routePath << "\n\n";
    output << "map:\n  xodr: " << xodrPath << "\n\n";
    output << "finish:\n  x: 0.0\n  y: 0.0\n  route_s: 0.0\n"
           << "  configured: false  # 운영측 종료 좌표를 받으면 채운다\n\n";

    std::size_t stopLineCount = 0;
    std::size_t crosswalkCount = 0;
    std::size_t speedMarkCount = 0;

    output << "stop_lines:\n";
    for (const auto& object : map.roadObjects) {
        if (!isStopLine(object)) continue;
        WorldFeature feature;
        if (!roadObjectToWorld(feature, map, object)) continue;
        const RouteProjection projection = worldToRoute(route, feature.center);
        if (!projection.valid) continue;
        const auto roadFound = map.roads.find(object.roadId);
        if (roadFound == map.roads.end()) continue;
        const int laneId =
            laneIdAtRoadT(roadFound->second, object.s, object.t);
        if (laneId == 0) continue;
        const int controllerId = guessControllerId(map, object);
        output << "  - id: " << object.id << '\n'
               << "    controller_id: " << controllerId << '\n'
               << "    road_id: " << object.roadId << '\n'
               << "    lane_ids: [" << laneId << "]\n"
               << "    begin: [" << feature.center.x << ", "
               << feature.center.y << "]\n"
               << "    end: [" << feature.center.x << ", "
               << feature.center.y << "]\n"
               << "    route_s: " << projection.routeS << '\n'
               << "    # route lateral " << projection.lateral << " m\n";
        ++stopLineCount;
    }

    output << "\ncrosswalks:\n";
    for (const auto& object : map.roadObjects) {
        if (!isCrosswalk(object)) continue;
        WorldFeature feature;
        if (!roadObjectToWorld(feature, map, object)) continue;
        const RouteProjection projection = worldToRoute(route, feature.center);
        if (!projection.valid) continue;
        const std::vector<Point2D> polygon = makeRectangle(feature);
        double entryS = std::numeric_limits<double>::max();
        double exitS = -std::numeric_limits<double>::max();
        for (const auto& corner : polygon) {
            const RouteProjection cornerProjection =
                worldToRoute(route, corner);
            if (!cornerProjection.valid) continue;
            entryS = std::min(entryS, cornerProjection.routeS);
            exitS = std::max(exitS, cornerProjection.routeS);
        }
        if (entryS > exitS) continue;
        output << "  - id: " << object.id << '\n'
               << "    road_id: " << object.roadId << '\n'
               << "    polygon: \"" << polygonText(polygon) << "\"\n"
               << "    entry_route_s: " << entryS << '\n'
               << "    exit_route_s: " << exitS << '\n'
               << "    # source: " << object.name << '\n';
        ++crosswalkCount;
    }

    // 노면 제한속도 표시는 "지점"이라 구간이 아니다.
    // 위치만 뽑아주고, 적용 구간은 사람이 확인해서 채운다.
    output << "\nspeed_zones: []\n\n"
           << "# 아래는 XODR에서 찾은 노면 제한속도 표시다.\n"
           << "# 표시는 지점이고 speed_zones는 구간이므로 자동 변환하지 않는다.\n"
           << "# VTD 화면에서 적용 구간을 확인한 뒤 speed_zones에 직접 옮길 것.\n";
    for (const auto& object : map.roadObjects) {
        const double limitKph = speedMarkingLimitKph(object);
        if (limitKph <= 0.0) continue;
        WorldFeature feature;
        if (!roadObjectToWorld(feature, map, object)) continue;
        const RouteProjection projection = worldToRoute(route, feature.center);
        if (!projection.valid ||
            projection.lateral > kSpeedMarkLateralLimit) {
            continue;
        }
        output << "#   route_s " << projection.routeS << "  " << limitKph
               << " km/h  road " << object.roadId << "  (" << object.name
               << ")\n";
        ++speedMarkCount;
    }

    output << "\ntraffic_lights: []\n\n"
           << "# GT trafficLight.id는 XODR controller id다. 아래는 controller와\n"
           << "# 그 controller가 묶는 개별 signal id 목록이다.\n";
    std::size_t signalCount = 0;
    std::set<int> nearbyControllerIds;
    for (const auto& signal : map.roadSignals) {
        Point2D position;
        if (!roadSignalToWorld(position, map, signal)) continue;
        const RouteProjection projection = worldToRoute(route, position);
        if (!projection.valid ||
            projection.lateral > kSignalLateralLimit) {
            continue;
        }
        output << "#   id " << signal.id << "  road " << signal.roadId
               << "  type " << signal.type << "  route_s " << projection.routeS
               << "  lateral " << projection.lateral << '\n';
        const auto controller = map.signalToController.find(signal.id);
        if (controller != map.signalToController.end()) {
            nearbyControllerIds.insert(controller->second);
        }
        ++signalCount;
    }
    for (int controllerId : nearbyControllerIds) {
        const auto found = map.trafficControllers.find(controllerId);
        if (found == map.trafficControllers.end()) continue;
        const TrafficController& controller = found->second;
        output << "#   controller " << controllerId << " signals [";
        for (std::size_t index = 0; index < controller.signalIds.size(); ++index) {
            if (index > 0) output << ", ";
            output << controller.signalIds[index];
        }
        output << "]\n";
    }

    std::cout << "[extract] stop_lines=" << stopLineCount
              << " crosswalks=" << crosswalkCount
              << " speed_marks=" << speedMarkCount
              << " signals_near_route=" << signalCount << '\n'
              << "[extract] wrote " << outputPath << '\n';
    return 0;
}
