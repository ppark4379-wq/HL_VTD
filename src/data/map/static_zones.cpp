#include "data/map/static_zones.hpp"
#include "data/map/map_structs.hpp"

#include "utils/csv_utils.hpp"
#include "utils/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace {

struct RouteProjection {
    bool valid{};
    double routeS{};
    double distance{};
};

std::string cleanValue(std::string text) {
    text = trim(text);
    if (text.size() >= 2 &&
        ((text.front() == '"' && text.back() == '"') ||
         (text.front() == '\'' && text.back() == '\''))) {
        text = text.substr(1, text.size() - 2);
    }
    return trim(text);
}

bool parseInteger(const std::string& text, int& output) {
    double value{};
    if (!parseDouble(cleanValue(text), value)) return false;
    output = static_cast<int>(value);
    return true;
}

bool parsePoint(std::string text, Point2D& output) {
    text = cleanValue(text);
    text.erase(std::remove(text.begin(), text.end(), '['), text.end());
    text.erase(std::remove(text.begin(), text.end(), ']'), text.end());
    const std::size_t comma = text.find(',');
    if (comma == std::string::npos) return false;
    return parseDouble(trim(text.substr(0, comma)), output.x) &&
           parseDouble(trim(text.substr(comma + 1)), output.y);
}

std::vector<int> parseIntegerList(std::string text) {
    text = cleanValue(text);
    text.erase(std::remove(text.begin(), text.end(), '['), text.end());
    text.erase(std::remove(text.begin(), text.end(), ']'), text.end());
    std::vector<int> result;
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        int value{};
        if (parseInteger(token, value)) result.push_back(value);
    }
    return result;
}

std::vector<Point2D> parsePolygon(std::string text) {
    text = cleanValue(text);
    std::vector<Point2D> result;
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ';')) {
        Point2D point;
        if (parsePoint(token, point)) result.push_back(point);
    }
    return result;
}

RouteProjection projectToRoute(const Point2D& point,
                               const std::vector<Waypoint>& route) {
    RouteProjection result;
    result.distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index + 1 < route.size(); ++index) {
        const double dx = route[index + 1].x - route[index].x;
        const double dy = route[index + 1].y - route[index].y;
        const double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared < 1e-9) continue;
        double ratio = ((point.x - route[index].x) * dx +
                        (point.y - route[index].y) * dy) / lengthSquared;
        ratio = std::clamp(ratio, 0.0, 1.0);
        const double projectedX = route[index].x + ratio * dx;
        const double projectedY = route[index].y + ratio * dy;
        const double distance =
            std::hypot(point.x - projectedX, point.y - projectedY);
        if (distance >= result.distance) continue;
        result.valid = true;
        result.distance = distance;
        result.routeS = route[index].s + ratio * std::sqrt(lengthSquared);
    }
    return result;
}

double crossProduct(const Point2D& origin,
                    const Point2D& first,
                    const Point2D& second) {
    return (first.x - origin.x) * (second.y - origin.y) -
           (first.y - origin.y) * (second.x - origin.x);
}

bool pointOnSegment(const Point2D& point,
                    const Point2D& begin,
                    const Point2D& end) {
    constexpr double kTolerance = 1e-6;
    return std::abs(crossProduct(begin, end, point)) <= kTolerance &&
           point.x >= std::min(begin.x, end.x) - kTolerance &&
           point.x <= std::max(begin.x, end.x) + kTolerance &&
           point.y >= std::min(begin.y, end.y) - kTolerance &&
           point.y <= std::max(begin.y, end.y) + kTolerance;
}

bool segmentsIntersect(const Point2D& firstBegin,
                       const Point2D& firstEnd,
                       const Point2D& secondBegin,
                       const Point2D& secondEnd) {
    const double firstSideBegin =
        crossProduct(firstBegin, firstEnd, secondBegin);
    const double firstSideEnd =
        crossProduct(firstBegin, firstEnd, secondEnd);
    const double secondSideBegin =
        crossProduct(secondBegin, secondEnd, firstBegin);
    const double secondSideEnd =
        crossProduct(secondBegin, secondEnd, firstEnd);
    if (((firstSideBegin > 0.0 && firstSideEnd < 0.0) ||
         (firstSideBegin < 0.0 && firstSideEnd > 0.0)) &&
        ((secondSideBegin > 0.0 && secondSideEnd < 0.0) ||
         (secondSideBegin < 0.0 && secondSideEnd > 0.0))) {
        return true;
    }
    return pointOnSegment(secondBegin, firstBegin, firstEnd) ||
           pointOnSegment(secondEnd, firstBegin, firstEnd) ||
           pointOnSegment(firstBegin, secondBegin, secondEnd) ||
           pointOnSegment(firstEnd, secondBegin, secondEnd);
}

bool routeIntersectsPolygon(const std::vector<Waypoint>& route,
                            const std::vector<Point2D>& polygon) {
    if (route.size() < 2 || polygon.size() < 3) return false;
    for (std::size_t routeIndex = 0;
         routeIndex + 1 < route.size(); ++routeIndex) {
        const Point2D begin{route[routeIndex].x, route[routeIndex].y};
        const Point2D end{route[routeIndex + 1].x,
                          route[routeIndex + 1].y};
        if (containsPoint(polygon, begin) || containsPoint(polygon, end)) {
            return true;
        }
        for (std::size_t polygonIndex = 0;
             polygonIndex < polygon.size(); ++polygonIndex) {
            if (segmentsIntersect(
                    begin, end, polygon[polygonIndex],
                    polygon[(polygonIndex + 1) % polygon.size()])) {
                return true;
            }
        }
    }
    return false;
}

void appendZones(StaticZoneData& destination,
                 const StaticZoneData& source) {
    destination.speedZones.insert(destination.speedZones.end(),
                                  source.speedZones.begin(),
                                  source.speedZones.end());
    destination.stopLines.insert(destination.stopLines.end(),
                                 source.stopLines.begin(),
                                 source.stopLines.end());
    destination.crosswalks.insert(destination.crosswalks.end(),
                                  source.crosswalks.begin(),
                                  source.crosswalks.end());
    destination.flashingRedControllerIds.insert(
        destination.flashingRedControllerIds.end(),
        source.flashingRedControllerIds.begin(),
        source.flashingRedControllerIds.end());
    if (source.finish.valid) destination.finish = source.finish;
}

bool loadSchoolZoneReferencePoints(StaticZoneData& zones,
                                   const std::string& yamlPath) {
    std::ifstream input(yamlPath);
    if (!input) return false;

    std::vector<SchoolZoneReferencePoint> points;
    std::unordered_set<std::string> ids;
    int declaredPointCount = -1;
    double speedLimitKph = 0.0;
    bool hasSpeedLimit = false;
    bool inPointSection = false;
    bool hasCurrentPoint = false;
    bool hasId = false;
    bool hasX = false;
    bool hasY = false;
    bool hasTrackId = false;
    bool hasLaneId = false;
    bool hasTrackS = false;

    auto finishPoint = [&]() {
        if (!hasCurrentPoint) return true;
        if (!hasId || !hasX || !hasY || !hasTrackId ||
            !hasLaneId || !hasTrackS) {
            return false;
        }
        return ids.insert(points.back().id).second;
    };

    std::string rawLine;
    while (std::getline(input, rawLine)) {
        const std::size_t comment = rawLine.find('#');
        std::string line = trim(comment == std::string::npos
                                    ? rawLine
                                    : rawLine.substr(0, comment));
        if (line.empty()) continue;
        if (line == "school_zone_reference_points:") {
            inPointSection = true;
            continue;
        }

        bool newPoint = false;
        if (inPointSection && line.rfind("- ", 0) == 0) {
            if (!finishPoint()) return false;
            points.emplace_back();
            hasCurrentPoint = true;
            hasId = hasX = hasY = hasTrackId = hasLaneId = hasTrackS = false;
            line = trim(line.substr(2));
            newPoint = true;
        }

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) return false;
        const std::string key = trim(line.substr(0, colon));
        const std::string value = cleanValue(line.substr(colon + 1));

        if (!inPointSection) {
            if (key == "speed_limit_kph") {
                hasSpeedLimit = parseDouble(value, speedLimitKph) &&
                                speedLimitKph > 0.0;
                if (!hasSpeedLimit) return false;
            } else if (key == "point_count") {
                if (!parseInteger(value, declaredPointCount) ||
                    declaredPointCount < 0) {
                    return false;
                }
            }
            continue;
        }

        if (!hasCurrentPoint) return false;
        auto& point = points.back();
        if (key == "id") {
            point.id = value;
            hasId = !point.id.empty();
        } else if (key == "x") {
            hasX = parseDouble(value, point.position.x);
        } else if (key == "y") {
            hasY = parseDouble(value, point.position.y);
        } else if (key == "track_id") {
            hasTrackId = parseInteger(value, point.trackId);
        } else if (key == "lane_id") {
            hasLaneId = parseInteger(value, point.laneId);
        } else if (key == "s") {
            hasTrackS = parseDouble(value, point.trackS);
        } else if (!newPoint) {
            return false;
        }
    }

    if (!finishPoint() || !hasSpeedLimit || declaredPointCount < 0 ||
        static_cast<std::size_t>(declaredPointCount) != points.size()) {
        return false;
    }
    zones.schoolZoneReferenceSpeedLimit = speedLimitKph / 3.6;
    zones.schoolZoneReferencePoints = std::move(points);
    return true;
}

}  // namespace

bool loadSingleMapRuntimeConfig(MapRuntimeConfig& config,
                                const std::string& yamlPath) {
    std::ifstream input(yamlPath);
    if (!input) return false;
    config = {};

    std::string section;
    int currentTrafficLightId = -1;
    std::string rawLine;
    while (std::getline(input, rawLine)) {
        const std::size_t comment = rawLine.find('#');
        std::string line = trim(comment == std::string::npos
                                    ? rawLine
                                    : rawLine.substr(0, comment));
        if (line.empty()) continue;
        if (line.back() == ':' && line.front() != '-') {
            section = trim(line.substr(0, line.size() - 1));
            continue;
        }
        if (line == "[]" || line.find(": []") != std::string::npos) {
            const std::size_t colon = line.find(':');
            if (colon != std::string::npos) section = trim(line.substr(0, colon));
            continue;
        }

        bool newItem = false;
        if (line.rfind("- ", 0) == 0) {
            newItem = true;
            line = trim(line.substr(2));
            if (section == "speed_zones") config.zones.speedZones.emplace_back();
            if (section == "stop_lines") config.zones.stopLines.emplace_back();
            if (section == "crosswalks") config.zones.crosswalks.emplace_back();
            if (section == "traffic_lights") currentTrafficLightId = -1;
        }

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) return false;
        const std::string key = trim(line.substr(0, colon));
        const std::string value = cleanValue(line.substr(colon + 1));

        if (section == "map" && key == "xodr") {
            config.xodrPath = value;
        } else if (section == "map" && key == "static_features") {
            config.staticFeaturesPath = value;
        } else if (section == "map" &&
                   key == "school_zone_reference_points") {
            config.schoolZoneReferencePointsPath = value;
        } else if (section == "finish") {
            if (key == "x") parseDouble(value, config.zones.finish.referencePoint.x);
            else if (key == "y") parseDouble(value, config.zones.finish.referencePoint.y);
            else if (key == "route_s") parseDouble(value, config.zones.finish.routeS);
            else if (key == "configured")
                config.zones.finish.valid = value == "true" || value == "1";
        } else if (section == "speed_zones" && !config.zones.speedZones.empty()) {
            auto& zone = config.zones.speedZones.back();
            if (key == "id") zone.id = value;
            else if (key == "speed_limit_kph") {
                // yaml은 km/h, 내부는 m/s.
                if (parseDouble(value, zone.speedLimit)) {
                    zone.speedLimit /= 3.6;
                }
            }
            else if (key == "polygon") zone.polygon = parsePolygon(value);
            else if (key == "route_s_begin") {
                zone.hasRouteRange = parseDouble(value, zone.routeSBegin);
            } else if (key == "route_s_end") {
                parseDouble(value, zone.routeSEnd);
            }
        } else if (section == "stop_lines" && !config.zones.stopLines.empty()) {
            auto& lineData = config.zones.stopLines.back();
            if (key == "id") parseInteger(value, lineData.id);
            else if (key == "controller_id")
                parseInteger(value, lineData.controllerId);
            else if (key == "signal_id") return false;
            else if (key == "road_id") parseInteger(value, lineData.roadId);
            else if (key == "lane_ids") lineData.laneIds = parseIntegerList(value);
            else if (key == "begin") parsePoint(value, lineData.begin);
            else if (key == "end") parsePoint(value, lineData.end);
            else if (key == "route_s") parseDouble(value, lineData.routeS);
        } else if (section == "crosswalks" && !config.zones.crosswalks.empty()) {
            auto& crosswalk = config.zones.crosswalks.back();
            if (key == "id") parseInteger(value, crosswalk.id);
            else if (key == "road_id") parseInteger(value, crosswalk.roadId);
            else if (key == "polygon") crosswalk.polygon = parsePolygon(value);
            else if (key == "entry_route_s")
                parseDouble(value, crosswalk.entryRouteS);
            else if (key == "exit_route_s")
                parseDouble(value, crosswalk.exitRouteS);
        } else if (section == "traffic_lights") {
            if (key == "controller_id")
                parseInteger(value, currentTrafficLightId);
            else if (key == "id")
                parseInteger(value, currentTrafficLightId);
            else if (key == "flashing_type" && value == "red" &&
                     currentTrafficLightId >= 0) {
                config.zones.flashingRedControllerIds.push_back(
                    currentTrafficLightId);
            }
        } else if (!newItem) {
            return false;
        }
    }
    return !config.xodrPath.empty();
}

bool loadMapRuntimeConfig(MapRuntimeConfig& config,
                          const std::string& yamlPath) {
    MapRuntimeConfig manual;
    if (!loadSingleMapRuntimeConfig(manual, yamlPath)) return false;
    if (manual.staticFeaturesPath.empty()) {
        if (!manual.schoolZoneReferencePointsPath.empty() &&
            !loadSchoolZoneReferencePoints(
                manual.zones, manual.schoolZoneReferencePointsPath)) {
            return false;
        }
        config = std::move(manual);
        return true;
    }
    MapRuntimeConfig generated;
    if (!loadSingleMapRuntimeConfig(generated,
                                    manual.staticFeaturesPath)) return false;
    generated.xodrPath = manual.xodrPath;
    generated.staticFeaturesPath = manual.staticFeaturesPath;
    generated.schoolZoneReferencePointsPath =
        manual.schoolZoneReferencePointsPath;
    appendZones(generated.zones, manual.zones);
    if (!generated.schoolZoneReferencePointsPath.empty() &&
        !loadSchoolZoneReferencePoints(
            generated.zones, generated.schoolZoneReferencePointsPath)) {
        return false;
    }
    config = std::move(generated);
    return true;
}

bool isFlashingRedController(const StaticZoneData& zones,
                             int controllerId) {
    return std::find(zones.flashingRedControllerIds.begin(),
                     zones.flashingRedControllerIds.end(),
                     controllerId) !=
           zones.flashingRedControllerIds.end();
}

bool alignStaticZonesToRoute(StaticZoneData& zones,
                             const std::vector<Waypoint>& route) {
    if (route.size() < 2) return false;

    // generated YAML은 지도 전체 정적 시설을 담는다. 원본 시설은 절대로
    // 지우지 않고 현재 경로에 적용되는지만 routeRelevant로 표시한다.
    // Junction에서는 서로 겹치는 connector의 road ID가 달라질 수 있으므로
    // road ID는 힌트로만 쓰고 실제 좌표 투영/교차를 함께 검사한다.
    std::unordered_set<int> routeRoadIds;
    for (const auto& waypoint : route) {
        if (waypoint.roadId >= 0) routeRoadIds.insert(waypoint.roadId);
    }

    for (auto& line : zones.stopLines) {
        const Point2D center{0.5 * (line.begin.x + line.end.x),
                             0.5 * (line.begin.y + line.end.y)};
        const RouteProjection projection = projectToRoute(center, route);
        line.routeRelevant = projection.valid && projection.distance <= 5.0;
        if (line.routeRelevant) line.routeS = projection.routeS;
    }

    for (auto& crosswalk : zones.crosswalks) {
        // Junction의 서로 겹치는 connecting road는 도로 ID가 달라도
        // 같은 횟단보도를 물리적으로 통과한다. 체크포인트에서 선택된
        // sibling road ID만 비교하면 실제 주행 경로 위 횟단보도가 삭제된다.
        // road ID가 일치하지 않을 때는 경로 중심선이 폴리곤을
        // 직접 관통하는 시설만 유지해 교차로의 다른 횟단보도 오검출은 막는다.
        const bool sameRoad = crosswalk.roadId < 0 ||
            routeRoadIds.find(crosswalk.roadId) != routeRoadIds.end();
        const bool physicallyIntersected =
            routeIntersectsPolygon(route, crosswalk.polygon);
        double entry = std::numeric_limits<double>::infinity();
        double exit = -std::numeric_limits<double>::infinity();
        double minimumDistance = std::numeric_limits<double>::infinity();
        for (const auto& corner : crosswalk.polygon) {
            const RouteProjection projection = projectToRoute(corner, route);
            if (!projection.valid) continue;
            entry = std::min(entry, projection.routeS);
            exit = std::max(exit, projection.routeS);
            minimumDistance = std::min(minimumDistance, projection.distance);
        }
        crosswalk.routeRelevant =
            std::isfinite(entry) && std::isfinite(exit) &&
            minimumDistance <= 15.0 &&
            (sameRoad || physicallyIntersected);
        if (crosswalk.routeRelevant) {
            crosswalk.entryRouteS = entry;
            crosswalk.exitRouteS = exit;
        }
    }

    // 측정 기준점은 구간의 시작/끝 및 중간 표본이다. 현재 경로 가까이에
    // 투영되는 점만 모아 route_s 순으로 정렬하고, 큰 공백에서 서로 다른
    // 보호구역으로 나눈다. XODR track id와 OpenDRIVE road id가 다른 지도라
    // 좌표 기반 투영을 사용한다.
    // 기준점으로 계산한 구간은 원본 지도 정보가 아닌 파생 캐시다. 재정렬할
    // 때도 벡터에서 제거하지 않고 비활성화한 뒤 같은 ID 슬롯을 갱신한다.
    for (auto& zone : zones.speedZones) {
        if (zone.derivedFromReferencePoints) zone.routeRelevant = false;
    }
    std::vector<double> schoolRoutePoints;
    for (const auto& point : zones.schoolZoneReferencePoints) {
        const RouteProjection projection =
            projectToRoute(point.position, route);
        if (projection.valid && projection.distance <= 8.0) {
            schoolRoutePoints.push_back(projection.routeS);
        }
    }
    std::sort(schoolRoutePoints.begin(), schoolRoutePoints.end());
    constexpr double kSchoolZoneMaximumSampleGap = 200.0;
    for (std::size_t begin = 0; begin < schoolRoutePoints.size();) {
        std::size_t end = begin;
        while (end + 1 < schoolRoutePoints.size() &&
               schoolRoutePoints[end + 1] - schoolRoutePoints[end] <=
                   kSchoolZoneMaximumSampleGap) {
            ++end;
        }
        SpeedZone zone;
        zone.id = "auto_school_zone_" + std::to_string(begin);
        zone.speedLimit = zones.schoolZoneReferenceSpeedLimit;
        zone.routeSBegin = schoolRoutePoints[begin];
        zone.routeSEnd = schoolRoutePoints[end];
        zone.hasRouteRange = true;
        zone.schoolZone = true;
        zone.routeRelevant = true;
        zone.derivedFromReferencePoints = true;
        // 한 점만 매칭된 경로에서는 그 점을 구간 경계라고 단정할 수 없다.
        // 오검출 방지를 위해 실제 길이를 가진 구간만 활성화한다.
        if (zone.routeSEnd - zone.routeSBegin >= 10.0) {
            const auto existing = std::find_if(
                zones.speedZones.begin(), zones.speedZones.end(),
                [&](const SpeedZone& candidate) {
                    return candidate.derivedFromReferencePoints &&
                           candidate.id == zone.id;
                });
            if (existing == zones.speedZones.end()) {
                zones.speedZones.push_back(std::move(zone));
            } else {
                *existing = std::move(zone);
            }
        }
        begin = end + 1;
    }
    if (zones.finish.valid) {
        const RouteProjection projection =
            projectToRoute(zones.finish.referencePoint, route);
        if (!projection.valid) return false;
        zones.finish.routeS = projection.routeS;
    }
    return true;
}

bool isSchoolZoneActive(const StaticZoneData& zones,
                        const Point2D& position,
                        double routeS) {
    for (const auto& zone : zones.speedZones) {
        if (!zone.routeRelevant || !zone.schoolZone) continue;
        const bool insidePolygon =
            !zone.polygon.empty() && containsPoint(zone.polygon, position);
        const bool insideRouteRange =
            zone.hasRouteRange && routeS >= 0.0 &&
            routeS >= zone.routeSBegin && routeS <= zone.routeSEnd;
        if (insidePolygon || insideRouteRange) return true;
    }
    return false;
}

double findZoneSpeedLimit(const StaticZoneData& zones,
                          const Point2D& position,
                          double routeS,
                          double fallbackLimit) {
    double limit = fallbackLimit;
    for (const auto& zone : zones.speedZones) {
        if (!zone.routeRelevant || zone.speedLimit <= 0.0) continue;
        const bool insidePolygon =
            !zone.polygon.empty() && containsPoint(zone.polygon, position);
        const bool insideRouteRange =
            zone.hasRouteRange && routeS >= 0.0 &&
            routeS >= zone.routeSBegin && routeS <= zone.routeSEnd;
        if (insidePolygon || insideRouteRange) {
            limit = std::min(limit, zone.speedLimit);
        }
    }
    return limit;
}

bool isInsideCrosswalk(const StaticZoneData& zones,
                       int crosswalkId,
                       const Point2D& position) {
    for (const auto& crosswalk : zones.crosswalks) {
        if (crosswalk.id == crosswalkId) {
            return containsPoint(crosswalk.polygon, position);
        }
    }
    return false;
}
