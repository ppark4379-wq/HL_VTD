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
    for (auto& line : zones.stopLines) {
        const Point2D center{0.5 * (line.begin.x + line.end.x),
                             0.5 * (line.begin.y + line.end.y)};
        const RouteProjection projection = projectToRoute(center, route);
        if (!projection.valid) return false;
        line.routeS = projection.routeS;
    }
    for (auto& crosswalk : zones.crosswalks) {
        double entry = std::numeric_limits<double>::infinity();
        double exit = -std::numeric_limits<double>::infinity();
        for (const auto& corner : crosswalk.polygon) {
            const RouteProjection projection = projectToRoute(corner, route);
            if (!projection.valid) continue;
            entry = std::min(entry, projection.routeS);
            exit = std::max(exit, projection.routeS);
        }
        if (!std::isfinite(entry) || !std::isfinite(exit)) return false;
        crosswalk.entryRouteS = entry;
        crosswalk.exitRouteS = exit;
    }
    if (zones.finish.valid) {
        const RouteProjection projection =
            projectToRoute(zones.finish.referencePoint, route);
        if (!projection.valid) return false;
        zones.finish.routeS = projection.routeS;
    }
    return true;
}

double findZoneSpeedLimit(const StaticZoneData& zones,
                          const Point2D& position,
                          double routeS,
                          double fallbackLimit) {
    double limit = fallbackLimit;
    for (const auto& zone : zones.speedZones) {
        if (zone.speedLimit <= 0.0) continue;
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
