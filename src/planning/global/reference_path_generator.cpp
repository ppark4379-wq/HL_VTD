#include "planning/global/reference_path_generator.hpp"

#include "data/map/map_loader.hpp"
#include "utils/csv_utils.hpp"
#include "utils/math_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>


// 두 lane 식별자가 완전히 같은지 비교한다.
bool operator==(const LaneKey& left, const LaneKey& right) {
    return left.roadId == right.roadId &&
           left.sectionIndex == right.sectionIndex &&
           left.laneId == right.laneId;
}

// LaneKey를 map의 key로 사용할 수 있도록 일정한 정렬 순서를 정한다.
bool operator<(const LaneKey& left, const LaneKey& right) {
    if (left.roadId != right.roadId) return left.roadId < right.roadId;
    if (left.sectionIndex != right.sectionIndex)
        return left.sectionIndex < right.sectionIndex;
    return left.laneId < right.laneId;
}

// CSV checkpoint + OpenDRIVE 기반 reference path 생성기.
//
// 담당 기능
//   1) CSV checkpoint 파싱
//   2) OpenDRIVE road/lane/geometry 파싱
//   3) driving lane centerline 샘플링
//   4) Ego/checkpoint의 nearest lane map matching
//   5) lane successor + junction laneLink 그래프 구성
//   6) seq로 지정된 인접 checkpoint 사이의 legal lane 연결 복원
//   7) 최종 (x, y, yaw, s) reference path 생성/CSV 저장
//
constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

// =============================================================================
// 3. OpenDRIVE 내부 자료형과 도로 geometry 계산
// =============================================================================

// road s에 적용되는 여러 구간별 다항식 중 현재 구간 값을 계산한다.
double evaluatePiecewise(const std::vector<Polynomial>& polynomials, double s,
                         double fallback) {
    if (polynomials.empty()) return fallback;
    if (s + kEpsilon < polynomials.front().s) return fallback;
    const Polynomial* selected = &polynomials.front();
    for (const auto& polynomial : polynomials) {
        if (polynomial.s <= s + kEpsilon) selected = &polynomial;
        else break;
    }
    const double ds = std::max(0.0, s - selected->s);
    return ((selected->d * ds + selected->c) * ds + selected->b) * ds +
           selected->a;
}

// geometry 시작점에서 ds만큼 진행한 road 기준선의 월드 x/y/yaw를 계산한다.
// geometry.x/y 자체가 VTD 월드 좌표이므로 반환값도 VTD 월드 좌표다.
RoadPose evaluateGeometry(const Geometry& geometry, double ds) {
    ds = std::clamp(ds, 0.0, geometry.length);
    if (geometry.type == GeometryType::Line) {
        RoadPose pose;
        pose.x = geometry.x + ds * std::cos(geometry.heading);
        pose.y = geometry.y + ds * std::sin(geometry.heading);
        pose.yaw = geometry.heading;
        return pose;
    }
    if (geometry.type == GeometryType::Arc) {
        if (std::abs(geometry.curvature) < kEpsilon) {
            RoadPose pose;
            pose.x = geometry.x + ds * std::cos(geometry.heading);
            pose.y = geometry.y + ds * std::sin(geometry.heading);
            pose.yaw = geometry.heading;
            return pose;
        }
        const double yaw = geometry.heading + geometry.curvature * ds;
        RoadPose pose;
        pose.x = geometry.x +
                 (std::sin(yaw) - std::sin(geometry.heading)) / geometry.curvature;
        pose.y = geometry.y -
                 (std::cos(yaw) - std::cos(geometry.heading)) / geometry.curvature;
        pose.yaw = yaw;
        return pose;
    }
    if (geometry.type == GeometryType::Poly3) {
        const double u = ds;
        const double v = ((geometry.d * u + geometry.c) * u + geometry.b) * u +
                         geometry.a;
        const double dv = geometry.b + 2.0 * geometry.c * u +
                          3.0 * geometry.d * u * u;
        const double cosH = std::cos(geometry.heading);
        const double sinH = std::sin(geometry.heading);
        RoadPose pose;
        pose.x = geometry.x + u * cosH - v * sinH;
        pose.y = geometry.y + u * sinH + v * cosH;
        pose.yaw = geometry.heading + std::atan(dv);
        return pose;
    }

    // Spiral(clothoid)은 곡률을 선형 보간하고 midpoint 적분한다.
    const int steps = std::max(1, static_cast<int>(std::ceil(ds / 0.05)));
    const double step = ds / steps;
    double x = geometry.x;
    double y = geometry.y;
    double yaw = geometry.heading;
    const double curvatureRate = geometry.length > kEpsilon
                                     ? (geometry.curvatureEnd -
                                        geometry.curvatureStart) /
                                           geometry.length
                                     : 0.0;
    for (int i = 0; i < steps; ++i) {
        const double localMid = (i + 0.5) * step;
        const double curvature = geometry.curvatureStart +
                                 curvatureRate * localMid;
        const double yawMid = yaw + 0.5 * curvature * step;
        x += step * std::cos(yawMid);
        y += step * std::sin(yawMid);
        yaw += curvature * step;
    }
    RoadPose pose;
    pose.x = x;
    pose.y = y;
    pose.yaw = yaw;
    return pose;
}

// Checkpoint를 seq 오름차순으로 정렬할 때 사용한다.
bool compareCheckpointSequence(const Checkpoint& lhs, const Checkpoint& rhs) {
    return lhs.sequence < rhs.sequence;
}

// 매칭에 실패한 checkpoint가 있는지 검사한다.
bool isInvalidMatch(const LaneMatch& match) {
    return !match.valid;
}

// geometry를 road s 오름차순으로 정렬할 때 사용한다.
bool compareGeometryS(const Geometry& lhs, const Geometry& rhs) {
    return lhs.s < rhs.s;
}

// 다항식을 적용 시작 s 오름차순으로 정렬할 때 사용한다.
bool comparePolynomialS(const Polynomial& lhs, const Polynomial& rhs) {
    return lhs.s < rhs.s;
}

// laneSection을 road s 오름차순으로 정렬할 때 사용한다.
bool compareLaneSectionS(const LaneSection& lhs, const LaneSection& rhs) {
    return lhs.s < rhs.s;
}

// LaneMatch 후보를 실제 좌표거리 오름차순으로 정렬할 때 사용한다.
bool compareLaneMatchDistance(const LaneMatch& lhs, const LaneMatch& rhs) {
    return lhs.distance < rhs.distance;
}

// CSV의 x,y 또는 seq,x,y 형식을 읽고 seq 순서로 정렬한다.
bool loadCheckpointsCsv(PathGeneratorData& data, const std::string& csvPath) {
        data.checkpoints.clear();
        std::ifstream input(csvPath);
        if (!input) {
            std::cerr << "[path] CSV open failed: " << csvPath << '\n';
            return false;
        }

        std::string line;
        int automaticSequence = 1;
        while (std::getline(input, line)) {
            line = trim(line);
            if (line.empty() || line.front() == '#') continue;
            const auto fields = splitCsvLine(line);
            if (fields.size() < 2) continue;

            Checkpoint point;
            double first = 0.0;
            double second = 0.0;
            double third = 0.0;
            if (fields.size() >= 3 && parseDouble(fields[0], first) &&
                parseDouble(fields[1], second) && parseDouble(fields[2], third)) {
                point.sequence = static_cast<int>(first);
                point.x = second;
                point.y = third;
            } else if (parseDouble(fields[0], first) &&
                       parseDouble(fields[1], second)) {
                point.sequence = automaticSequence;
                point.x = first;
                point.y = second;
            } else {
                continue;  // x,y 또는 seq,x,y 헤더 행
            }
            ++automaticSequence;
            data.checkpoints.push_back(point);
        }

        std::sort(data.checkpoints.begin(), data.checkpoints.end(),
             compareCheckpointSequence);
        std::cout << "[path] loaded " << data.checkpoints.size() << " checkpoints\n";
        return data.checkpoints.size() >= 2;
}

    // XODR을 읽고 road/lane 중심선 및 legal lane 연결을 준비한다.
bool loadMap(PathGeneratorData& data, const std::string& xodrPath,
             double sampleInterval) {
        data.roads.clear();
        data.roadObjects.clear();
        data.roadSignals.clear();
        data.trafficControllers.clear();
        data.signalToController.clear();
        data.junctionLinks.clear();
        data.laneSegments.clear();
        data.laneGraph.clear();
        data.sampleInterval = std::clamp(sampleInterval, 0.1, 2.0);

        try {
            const auto root = loadOpenDrive(xodrPath);
            parseRoads(data, root);
            parseTrafficControllers(data, root);
            parseJunctions(data, root);
            sampleLaneCenters(data);
            buildLaneGraph(data);
        } catch (const std::exception& error) {
            std::cerr << "[path] XODR load failed: " << error.what() << '\n';
            return false;
        }

        std::cout << "[path] loaded " << data.roads.size() << " roads, "
                  << data.laneSegments.size() << " driving lane segments\n";
        return !data.laneSegments.empty();
}

    // 임의의 월드 좌표를 가장 가까운 driving lane 중심선에 매칭한다.
LaneMatch worldToLane(const PathGeneratorData& data,
                           double x, double y,
                           std::optional<double> heading) {
        LaneMatch best;
        double bestScore = std::numeric_limits<double>::infinity();
        for (const auto& [key, segment] : data.laneSegments) {
            for (std::size_t index = 0; index < segment.points.size(); ++index) {
                const auto& point = segment.points[index];
                const double squaredDistance = distanceSquared(x, y, point.x, point.y);
                double score = squaredDistance;
                if (heading) {
                    const double error = std::abs(normalizeAngle(*heading - point.yaw));
                    // 반대 방향 lane 오매칭 억제. 약 90도부터 큰 penalty.
                    score += 4.0 * error * error;
                }
                if (score < bestScore) {
                    const Road& road = data.roads.at(key.roadId);
                    const LaneDefinition& lane =
                        road.sections[key.sectionIndex].lanes.at(key.laneId);
                    const double width =
                        evaluatePiecewise(lane.widths, point.roadS, 0.0);
                    bestScore = score;
                    best = {true, key.roadId, key.laneId, key.sectionIndex,
                            point.roadS, std::sqrt(squaredDistance), width, index};
                }
            }
        }
        if (best.valid &&
            best.distance > 0.5 * best.laneWidth + 0.5) {
            best.valid = false;
        }
        return best;
}

// 각 lane마다 가장 가까운 샘플점을 초기값으로 삼아 road 기준선에 정확히
// 투영한 뒤(roadS, 부호있는 t), lane 중심 offset과 실제 폭을 그 roadS에서
// 계산해 t가 lane 폭 내부에 있는지 판정한다. 0.5m 샘플 간격의 오차를
// 흡수하기 위한 임의 여유값(예전의 0.35m) 없이 해석적으로 포함 여부를 정한다.
std::vector<LaneMatch> findContainingLanes(const PathGeneratorData& data,
                                      double x, double y) {
    std::vector<LaneMatch> candidates;
    constexpr double kContainmentEpsilon = 0.05;

    for (const auto& [key, segment] : data.laneSegments) {
        std::size_t nearestIndex = 0;
        double nearestDistanceSquared = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < segment.points.size(); ++index) {
            const Waypoint& point = segment.points[index];
            const double candidateDistanceSquared =
                distanceSquared(x, y, point.x, point.y);
            if (candidateDistanceSquared < nearestDistanceSquared) {
                nearestDistanceSquared = candidateDistanceSquared;
                nearestIndex = index;
            }
        }
        if (segment.points.empty()) continue;

        const Road& road = data.roads.at(key.roadId);
        const LaneSection& section = road.sections[key.sectionIndex];
        const double sGuess = segment.points[nearestIndex].roadS;
        const RoadProjection projection =
            worldToRoad(road, x, y, sGuess, data.sampleInterval * 2.0);

        // t와 lane 중심 offset을 그대로 빼면, 투영이 checkpoint에서 먼
        // 엉뚱한 s로 잘못 수렴했을 때도 두 값이 우연히 비슷해 통과할 수
        // 있다. 실제 lane 중심의 월드 좌표를 복원해 유클리드 거리로
        // 다시 검증한다 (sampleLaneCenters와 동일한 식).
        const double center = laneOffset(road, section, key.laneId, projection.s);
        const double width =
            evaluatePiecewise(section.lanes.at(key.laneId).widths, projection.s, 0.0);
        const RoadPose reference = roadToWorld(road, projection.s);
        const double laneCenterX = reference.x - center * std::sin(reference.yaw);
        const double laneCenterY = reference.y + center * std::cos(reference.yaw);
        const double lateralDistance =
            std::sqrt(distanceSquared(x, y, laneCenterX, laneCenterY));

        if (lateralDistance <= width * 0.5 + kContainmentEpsilon) {
            candidates.push_back({true, key.roadId, key.laneId, key.sectionIndex,
                                  projection.s, lateralDistance, width,
                                  nearestIndex});
        }
    }

    std::sort(candidates.begin(), candidates.end(), compareLaneMatchDistance);
    return candidates;
}

// Ego GT의 heading까지 사용하여 반대 방향 차선 오매칭을 줄인다.
LaneMatch matchEgoToLane(const PathGeneratorData& data, const GtFrame& gt) {
    return worldToLane(data, gt.ego.x, gt.ego.y, gt.ego.heading);
}

    // 모든 checkpoint의 주변 lane 후보를 먼저 만든다.
    // 시작부터 끝까지 연결되는 후보 조합 중 총 주행거리가 가장 짧은 조합을
    // 동적 계획법으로 선택해, 가까워도 큰 우회가 필요한 lane은 제외한다.
std::vector<LaneMatch> matchCheckpointsToLanes(const PathGeneratorData& data) {
        std::vector<std::vector<LaneMatch>> candidateSets;
        for (const Checkpoint& checkpoint : data.checkpoints) {
            candidateSets.push_back(
                findContainingLanes(data, checkpoint.x, checkpoint.y));
            if (candidateSets.back().empty()) {
                const LaneMatch nearest =
                    worldToLane(data, checkpoint.x, checkpoint.y);
                std::cerr << "[match] checkpoint seq=" << checkpoint.sequence
                     << " is outside every driving lane, nearest road="
                     << nearest.roadId << " lane=" << nearest.laneId
                     << " distance=" << nearest.distance << "m\n";
                return {};
            }
        }

        const double infinity = std::numeric_limits<double>::infinity();
        std::vector<std::vector<double>> costs(candidateSets.size());
        std::vector<std::vector<int>> previous(candidateSets.size());
        costs[0].resize(candidateSets[0].size(), infinity);
        previous[0].resize(candidateSets[0].size(), -1);
        for (std::size_t candidateIndex = 0;
             candidateIndex < candidateSets[0].size(); ++candidateIndex) {
            costs[0][candidateIndex] =
                candidateSets[0][candidateIndex].distance * 10.0;
        }

        for (std::size_t checkpointIndex = 1;
             checkpointIndex < candidateSets.size(); ++checkpointIndex) {
            costs[checkpointIndex].resize(candidateSets[checkpointIndex].size(),
                                          infinity);
            previous[checkpointIndex].resize(
                candidateSets[checkpointIndex].size(), -1);

            for (std::size_t currentIndex = 0;
                 currentIndex < candidateSets[checkpointIndex].size();
                 ++currentIndex) {
                const LaneMatch& current =
                    candidateSets[checkpointIndex][currentIndex];
                const LaneKey currentKey{current.roadId, current.sectionIndex,
                                         current.laneId};

                for (std::size_t priorIndex = 0;
                     priorIndex < candidateSets[checkpointIndex - 1].size();
                     ++priorIndex) {
                    if (!std::isfinite(costs[checkpointIndex - 1][priorIndex]))
                        continue;
                    const LaneMatch& prior =
                        candidateSets[checkpointIndex - 1][priorIndex];
                    const LaneKey priorKey{prior.roadId, prior.sectionIndex,
                                           prior.laneId};
                    if (priorKey == currentKey &&
                        current.pointIndex < prior.pointIndex) {
                        continue;
                    }

                    const std::vector<LaneKey> route =
                        connectLanes(data, priorKey, currentKey);
                    if (route.empty()) continue;
                    const double routeDistance =
                        calculateRouteDistance(data, route, prior, current);
                    const double newCost =
                        costs[checkpointIndex - 1][priorIndex] +
                        routeDistance + current.distance * 10.0;
                    if (newCost < costs[checkpointIndex][currentIndex]) {
                        costs[checkpointIndex][currentIndex] = newCost;
                        previous[checkpointIndex][currentIndex] =
                            static_cast<int>(priorIndex);
                    }
                }
            }

            bool hasConnectedCandidate = false;
            for (double cost : costs[checkpointIndex]) {
                if (std::isfinite(cost)) {
                    hasConnectedCandidate = true;
                    break;
                }
            }
            if (!hasConnectedCandidate) {
                const Checkpoint& checkpoint =
                    data.checkpoints[checkpointIndex];
                const LaneMatch nearest = candidateSets[checkpointIndex].front();
                std::cerr << "[match] no connected lane candidate at seq="
                     << checkpoint.sequence << " (" << checkpoint.x << ", "
                     << checkpoint.y << "), nearest road=" << nearest.roadId
                     << " lane=" << nearest.laneId
                     << " distance=" << nearest.distance << "m\n";
                return {};
            }
        }

        const std::size_t lastCheckpoint = candidateSets.size() - 1;
        int selected = -1;
        double bestCost = infinity;
        for (std::size_t index = 0; index < costs[lastCheckpoint].size(); ++index) {
            if (costs[lastCheckpoint][index] < bestCost) {
                bestCost = costs[lastCheckpoint][index];
                selected = static_cast<int>(index);
            }
        }
        if (selected < 0) return {};

        std::vector<LaneMatch> matches(candidateSets.size());
        for (std::size_t reverseIndex = candidateSets.size(); reverseIndex > 0;
             --reverseIndex) {
            const std::size_t checkpointIndex = reverseIndex - 1;
            matches[checkpointIndex] =
                candidateSets[checkpointIndex][selected];
            selected = previous[checkpointIndex][selected];
        }

        for (std::size_t index = 0; index < matches.size(); ++index) {
            const Checkpoint& checkpoint = data.checkpoints[index];
            const LaneMatch& match = matches[index];
            std::cout << "[match] seq=" << checkpoint.sequence << " ("
                      << checkpoint.x << ", " << checkpoint.y << ") -> road="
                      << match.roadId << " lane=" << match.laneId
                      << " section=" << match.sectionIndex
                      << " road_s=" << match.roadS
                      << " distance=" << match.distance << "m\n";
        }
        return matches;
}

    // seq 순서 그대로 인접 checkpoint를 연결해 최종 기준경로를 만든다.
std::vector<Waypoint> makePath(const PathGeneratorData& data) {
        const auto matches = matchCheckpointsToLanes(data);
        if (matches.size() < 2 ||
            std::any_of(matches.begin(), matches.end(), isInvalidMatch)) {
            return {};
        }

        std::vector<Waypoint> result;
        for (std::size_t i = 0; i + 1 < matches.size(); ++i) {
            const LaneKey start{matches[i].roadId, matches[i].sectionIndex,
                                matches[i].laneId};
            const LaneKey goal{matches[i + 1].roadId, matches[i + 1].sectionIndex,
                               matches[i + 1].laneId};
            const auto route = connectLanes(data, start, goal);
            if (route.empty()) {
                std::cerr << "[path] no legal lane route between waypoint "
                          << data.checkpoints[i].sequence << " and "
                          << data.checkpoints[i + 1].sequence << '\n';
                return {};
            }
            appendRoute(data, route, matches[i], matches[i + 1], result);
        }
        removeDuplicates(result);
        updateYawAndS(result);
        return result;
}

    // 시각화용으로 모든 driving lane centerline을 CSV로 저장한다.
bool saveLaneMap(const PathGeneratorData& data, const std::string& outputPath) {
        std::ofstream output(outputPath);
        if (!output) return false;
        output << "road_id,lane_id,section_index,point_index,x,y\n";
        output << std::fixed << std::setprecision(8);
        for (const auto& [key, segment] : data.laneSegments) {
            for (std::size_t index = 0; index < segment.points.size(); ++index) {
                const auto& point = segment.points[index];
                output << key.roadId << ',' << key.laneId << ','
                       << key.sectionIndex << ',' << index << ','
                       << point.x << ',' << point.y << '\n';
            }
        }
        return true;
}

    // =========================================================================
    // 5. XODR 파싱 함수
    // =========================================================================
    // XODR의 모든 <road>를 읽어 도로 형상과 차선 정의를 저장한다.
void parseRoads(PathGeneratorData& data, const XmlElement& root) {
        for (const auto& entry : root.children) {
            if (entry.first != "road") continue;
            const auto& node = entry.second;
            Road road;
            road.id = readAttribute<int>(node, "id", 0);
            road.junction = readAttribute<int>(node, "junction", -1);
            road.length = readAttribute<double>(node, "length", 0.0);
            parseRoadLinks(node, road);
            parseGeometries(node, road);
            parseLanes(node, road);
            parseRoadObjects(data, node, road);
            parseRoadSignals(data, node, road);
            data.roads.emplace(road.id, std::move(road));
        }
    }

    // XODR object/signal 값이 물리적으로 말이 되는 범위인지 검사한다.
    // 이 지도에는 length=2.8e-76, width=9.8e+252 같은 깨진 항목이 실제로 있고,
    // 걸러내지 않으면 폭 10^252 m짜리 영역이 만들어져 지도 전체를 덮는다.
bool isSaneRoadFeature(double s, double t, double length, double width,
                       double roadLength) {
        if (!std::isfinite(s) || !std::isfinite(t)) return false;
        if (!std::isfinite(length) || !std::isfinite(width)) return false;
        if (s < -1.0 || s > roadLength + 1.0) return false;
        if (std::fabs(t) > 50.0) return false;
        if (length < 0.05 || length > 60.0) return false;
        if (width < 0.05 || width > 60.0) return false;
        return true;
    }

    // road의 <objects>를 읽는다. 정지선/횡단보도/노면 제한속도 표시가 여기 있다.
void parseRoadObjects(PathGeneratorData& data, const XmlElement& node,
                      const Road& road) {
        const auto* objects = findFirstChild(node, "objects");
        if (!objects) return;
        for (const auto& entry : objects->children) {
            if (entry.first != "object") continue;
            const auto& element = entry.second;
            RoadObject object;
            object.roadId = road.id;
            object.id = readAttribute<int>(element, "id", -1);
            object.name = readAttribute<std::string>(element, "name", "");
            object.type = readAttribute<std::string>(element, "type", "");
            object.subtype = readAttribute<std::string>(element, "subtype", "");
            object.s = readAttribute<double>(element, "s", 0.0);
            object.t = readAttribute<double>(element, "t", 0.0);
            object.hdg = readAttribute<double>(element, "hdg", 0.0);
            object.length = readAttribute<double>(element, "length", 0.0);
            object.width = readAttribute<double>(element, "width", 0.0);
            if (!isSaneRoadFeature(object.s, object.t, object.length,
                                   object.width, road.length)) {
                continue;
            }
            data.roadObjects.push_back(std::move(object));
        }
    }

    // road의 <signals>를 읽는다. 신호등 id와 적용 위치가 여기 있다.
void parseRoadSignals(PathGeneratorData& data, const XmlElement& node,
                      const Road& road) {
        const auto* signals = findFirstChild(node, "signals");
        if (!signals) return;
        for (const auto& entry : signals->children) {
            if (entry.first != "signal") continue;
            const auto& element = entry.second;
            RoadSignal signal;
            signal.roadId = road.id;
            signal.id = readAttribute<int>(element, "id", -1);
            signal.name = readAttribute<std::string>(element, "name", "");
            signal.type = readAttribute<std::string>(element, "type", "");
            signal.subtype = readAttribute<std::string>(element, "subtype", "");
            signal.s = readAttribute<double>(element, "s", 0.0);
            signal.t = readAttribute<double>(element, "t", 0.0);
            signal.dynamic =
                readAttribute<std::string>(element, "dynamic", "no") == "yes";
            signal.orientation =
                readAttribute<std::string>(element, "orientation", "");
            if (signal.id < 0 || !std::isfinite(signal.s) || !std::isfinite(signal.t)) {
                continue;
            }
            if (signal.s < -1.0 || signal.s > road.length + 1.0) continue;
            data.roadSignals.push_back(std::move(signal));
        }
    }

void parseTrafficControllers(PathGeneratorData& data,
                             const XmlElement& root) {
    for (const auto& entry : root.children) {
        if (entry.first != "controller") continue;
        TrafficController controller;
        controller.id = readAttribute<int>(entry.second, "id", -1);
        controller.name =
            readAttribute<std::string>(entry.second, "name", "");
        if (controller.id < 0) continue;
        for (const auto& child : entry.second.children) {
            if (child.first != "control") continue;
            const int signalId =
                readAttribute<int>(child.second, "signalId", -1);
            if (signalId < 0) continue;
            controller.signalIds.push_back(signalId);
            data.signalToController[signalId] = controller.id;
        }
        data.trafficControllers.emplace(controller.id,
                                        std::move(controller));
    }
}

    // road 단위 predecessor/successor 연결정보를 읽는다.
void parseRoadLinks(const XmlElement& node, Road& road) {
        const auto* links = findFirstChild(node, "link");
        if (!links) return;
        for (const auto& entry : links->children) {
            RoadLink* target = nullptr;
            if (entry.first == "predecessor") target = &road.predecessor;
            if (entry.first == "successor") target = &road.successor;
            if (!target) continue;
            target->valid = true;
            target->elementType = readAttribute<std::string>(entry.second, "elementType", "");
            target->elementId = readAttribute<int>(entry.second, "elementId", -1);
            target->contactPoint =
                readAttribute<std::string>(entry.second, "contactPoint", "");
        }
    }

    // planView의 line/arc/spiral/poly3 기준선을 읽는다.
void parseGeometries(const XmlElement& node, Road& road) {
        const auto* planView = findFirstChild(node, "planView");
        if (!planView) return;
        for (const auto& entry : planView->children) {
            if (entry.first != "geometry") continue;
            Geometry geometry;
            geometry.s = readAttribute<double>(entry.second, "s", 0.0);
            geometry.x = readAttribute<double>(entry.second, "x", 0.0);
            geometry.y = readAttribute<double>(entry.second, "y", 0.0);
            geometry.heading = readAttribute<double>(entry.second, "hdg", 0.0);
            geometry.length = readAttribute<double>(entry.second, "length", 0.0);
            for (const auto& shape : entry.second.children) {
                if (shape.first == "line") {
                    geometry.type = GeometryType::Line;
                } else if (shape.first == "arc") {
                    geometry.type = GeometryType::Arc;
                    geometry.curvature = readAttribute<double>(shape.second, "curvature", 0.0);
                } else if (shape.first == "spiral") {
                    geometry.type = GeometryType::Spiral;
                    geometry.curvatureStart =
                        readAttribute<double>(shape.second, "curvStart", 0.0);
                    geometry.curvatureEnd =
                        readAttribute<double>(shape.second, "curvEnd", 0.0);
                } else if (shape.first == "poly3") {
                    geometry.type = GeometryType::Poly3;
                    geometry.a = readAttribute<double>(shape.second, "a", 0.0);
                    geometry.b = readAttribute<double>(shape.second, "b", 0.0);
                    geometry.c = readAttribute<double>(shape.second, "c", 0.0);
                    geometry.d = readAttribute<double>(shape.second, "d", 0.0);
                }
            }
            road.geometries.push_back(geometry);
        }
        std::sort(road.geometries.begin(), road.geometries.end(), compareGeometryS);
    }

    // OpenDRIVE의 a+b*ds+c*ds^2+d*ds^3 다항식 속성을 읽는다.
Polynomial parsePolynomial(const XmlElement& node, const std::string& sName) {
        return {readAttribute<double>(node, sName, 0.0), readAttribute<double>(node, "a", 0.0),
                readAttribute<double>(node, "b", 0.0), readAttribute<double>(node, "c", 0.0),
                readAttribute<double>(node, "d", 0.0)};
    }

    // laneOffset, laneSection, lane width와 lane link를 읽는다.
void parseLanes(const XmlElement& node, Road& road) {
        const auto* lanes = findFirstChild(node, "lanes");
        if (!lanes) return;
        for (const auto& entry : lanes->children) {
            if (entry.first == "laneOffset") {
                road.laneOffsets.push_back(parsePolynomial(entry.second, "s"));
            } else if (entry.first == "laneSection") {
                LaneSection section;
                section.s = readAttribute<double>(entry.second, "s", 0.0);
                for (const auto& side : entry.second.children) {
                    if (side.first != "left" && side.first != "right" &&
                        side.first != "center") continue;
                    for (const auto& laneNode : side.second.children) {
                        if (laneNode.first != "lane") continue;
                        LaneDefinition lane;
                        lane.id = readAttribute<int>(laneNode.second, "id", 0);
                        lane.type = readAttribute<std::string>(laneNode.second, "type", "none");
                        for (const auto& item : laneNode.second.children) {
                            if (item.first == "width") {
                                auto width = parsePolynomial(item.second, "sOffset");
                                width.s += section.s;
                                lane.widths.push_back(width);
                            } else if (item.first == "roadMark") {
                                RoadMarkEntry mark;
                                mark.s = section.s +
                                    readAttribute<double>(item.second, "sOffset", 0.0);
                                mark.laneChange =
                                    readAttribute<std::string>(item.second, "laneChange", "none");
                                mark.color =
                                    readAttribute<std::string>(item.second, "color", "standard");
                                lane.roadMarks.push_back(mark);
                            } else if (item.first == "link") {
                                for (const auto& link : item.second.children) {
                                    if (link.first == "predecessor")
                                        lane.predecessor =
                                            readAttribute<int>(link.second, "id", 0);
                                    if (link.first == "successor")
                                        lane.successor =
                                            readAttribute<int>(link.second, "id", 0);
                                }
                            }
                        }
                        section.lanes.emplace(lane.id, std::move(lane));
                    }
                }
                road.sections.push_back(std::move(section));
            }
        }
        std::sort(road.laneOffsets.begin(), road.laneOffsets.end(),
             comparePolynomialS);
        std::sort(road.sections.begin(), road.sections.end(), compareLaneSectionS);
    }

    // junction의 incomingRoad -> connectingRoad 및 laneLink를 읽는다.
void parseJunctions(PathGeneratorData& data, const XmlElement& root) {
        for (const auto& entry : root.children) {
            if (entry.first != "junction") continue;
            for (const auto& connection : entry.second.children) {
                if (connection.first != "connection") continue;
                const int incoming = readAttribute<int>(connection.second, "incomingRoad", -1);
                const int connecting = readAttribute<int>(connection.second, "connectingRoad", -1);
                const std::string contact =
                    readAttribute<std::string>(connection.second, "contactPoint", "start");
                for (const auto& laneLink : connection.second.children) {
                    if (laneLink.first != "laneLink") continue;
                    data.junctionLinks.push_back(
                        {incoming, connecting, contact,
                         readAttribute<int>(laneLink.second, "from", 0),
                         readAttribute<int>(laneLink.second, "to", 0)});
                }
            }
        }
    }

    // 주어진 road s에서 도로 기준선의 VTD 월드 x/y/yaw를 계산한다.
    // =========================================================================
    // 6. Lane centerline 생성 함수
    // =========================================================================

RoadPose roadToWorld(const Road& road, double s) {
        if (road.geometries.empty()) return {};
        const Geometry* selected = &road.geometries.front();
        for (const auto& geometry : road.geometries) {
            if (geometry.s <= s + kEpsilon) selected = &geometry;
            else break;
        }
        return evaluateGeometry(*selected, s - selected->s);
    }

    // 도로 기준선으로부터 해당 lane 중심까지의 횡방향 거리 t를 계산한다.
double laneOffset(const Road& road, const LaneSection& section,
                  int laneId, double s) {
        double offset = evaluatePiecewise(road.laneOffsets, s, 0.0);
        if (laneId > 0) {
            for (int id = 1; id < laneId; ++id) {
                const auto found = section.lanes.find(id);
                if (found != section.lanes.end())
                    offset += evaluatePiecewise(found->second.widths, s, 0.0);
            }
            const auto found = section.lanes.find(laneId);
            if (found != section.lanes.end())
                offset += 0.5 * evaluatePiecewise(found->second.widths, s, 0.0);
        } else {
            for (int id = -1; id > laneId; --id) {
                const auto found = section.lanes.find(id);
                if (found != section.lanes.end())
                    offset -= evaluatePiecewise(found->second.widths, s, 0.0);
            }
            const auto found = section.lanes.find(laneId);
            if (found != section.lanes.end())
                offset -= 0.5 * evaluatePiecewise(found->second.widths, s, 0.0);
        }
        return offset;
    }

double laneWidthAt(const LaneSection& section, int laneId, double roadS) {
        const auto found = section.lanes.find(laneId);
        if (found == section.lanes.end()) return 0.0;
        return evaluatePiecewise(found->second.widths, roadS, 0.0);
    }

    // s 위치에서 적용되는 roadMark의 laneChange 값을 찾는다.
std::string laneChangeMarkAt(const LaneDefinition& lane, double s) {
    if (lane.roadMarks.empty()) return "none";
    const RoadMarkEntry* selected = &lane.roadMarks.front();
    for (const auto& mark : lane.roadMarks) {
        if (mark.s <= s + kEpsilon) selected = &mark;
        else break;
    }
    return selected->laneChange;
}

    // lane의 roadMark는 해당 lane의 외측 경계선을 정의한다.
    // 따라서 인접한 lane 사이 경계는 |id|가 더 작은(중심에 가까운)
    // lane의 외측 roadMark에서 읽어야 한다.
// 한 lane section 안에서도 roadMark는 중간에 바뀐다(앞 25 m 실선, 이후 점선 등).
// laneGraph는 "이 구간에서 차선변경이 가능한 조합인가"만 표현하면 되므로
// 구간 내 어느 지점에서든 허용되면 연결로 본다. 실제 지점에서의 허용 여부는
// 기동을 세울 때 laneChangeAllowed가 정확한 s로 다시 판정한다.
bool laneChangeEverAllowed(const LaneSection& section, int fromLaneId,
                           int toLaneId) {
    const int ownerId =
        std::abs(fromLaneId) < std::abs(toLaneId) ? fromLaneId : toLaneId;
    const auto owner = section.lanes.find(ownerId);
    if (owner == section.lanes.end()) return false;
    for (const auto& mark : owner->second.roadMarks) {
        if (mark.laneChange == "both") return true;
        if (mark.laneChange == "increase" && toLaneId > fromLaneId) return true;
        if (mark.laneChange == "decrease" && toLaneId < fromLaneId) return true;
    }
    return false;
}

bool laneChangeAllowed(const LaneSection& section, int fromLaneId,
                       int toLaneId, double s) {
    const int ownerId = std::abs(fromLaneId) < std::abs(toLaneId) ? fromLaneId : toLaneId;
    const auto owner = section.lanes.find(ownerId);
    if (owner == section.lanes.end()) return false;
    const std::string mark = laneChangeMarkAt(owner->second, s);
    if (mark == "both") return true;
    if (mark == "increase") return toLaneId > fromLaneId;
    if (mark == "decrease") return toLaneId < fromLaneId;
    return false;
}

    // 월드 좌표 (x, y)를 road 기준선에 투영해 정확한 roadS와 부호 있는
    // 횡방향 거리 t를 구한다. sGuess 주변 [-window, window] 구간에서
    // golden-section search로 기준선까지 거리를 최소화하는 s를 찾는다.
    // line/arc는 닫힌 형태로 역산 가능하지만, poly3/spiral까지 함께
    // 다루기 위해 evaluateGeometry를 그대로 재사용하는 수치해석 방식을 쓴다.
RoadProjection worldToRoad(const Road& road, double x, double y,
                                  double sGuess, double window) {
    double lo = std::clamp(sGuess - window, 0.0, road.length);
    double hi = std::clamp(sGuess + window, 0.0, road.length);
    if (hi <= lo) { lo = 0.0; hi = road.length; }

    const auto costAt = [&](double s) {
        const RoadPose pose = roadToWorld(road, s);
        return distanceSquared(x, y, pose.x, pose.y);
    };

    const double goldenRatio = (std::sqrt(5.0) - 1.0) / 2.0;
    double a = lo, b = hi;
    double c = b - goldenRatio * (b - a);
    double d = a + goldenRatio * (b - a);
    for (int i = 0; i < 40 && (b - a) > 1e-4; ++i) {
        if (costAt(c) < costAt(d)) b = d; else a = c;
        c = b - goldenRatio * (b - a);
        d = a + goldenRatio * (b - a);
    }

    const double sBest = std::clamp(0.5 * (a + b), 0.0, road.length);
    const RoadPose pose = roadToWorld(road, sBest);
    const double dx = x - pose.x;
    const double dy = y - pose.y;
    const double t = dx * (-std::sin(pose.yaw)) + dy * std::cos(pose.yaw);
    return {sBest, t, std::hypot(dx, dy)};
}

    // 모든 driving lane을 sampleInterval 간격의 중심선 점으로 변환한다.
void sampleLaneCenters(PathGeneratorData& data) {
        for (const auto& [roadId, road] : data.roads) {
            for (std::size_t sectionIndex = 0; sectionIndex < road.sections.size();
                 ++sectionIndex) {
                const auto& section = road.sections[sectionIndex];
                const double endS = sectionIndex + 1 < road.sections.size()
                                        ? road.sections[sectionIndex + 1].s
                                        : road.length;
                for (const auto& [laneId, lane] : section.lanes) {
                    if (laneId == 0 || lane.type != "driving" || lane.widths.empty())
                        continue;
                    LaneSegment segment;
                    segment.key = {roadId, static_cast<int>(sectionIndex), laneId};
                    segment.startS = section.s;
                    segment.endS = endS;
                    segment.length = std::max(0.0, endS - section.s);
                    const int count = std::max(
                        1, static_cast<int>(std::ceil(segment.length / data.sampleInterval)));
                    // laneSection 경계점은 다음 section에서만 생성한다.
                    // 같은 roadS에 서로 다른 laneOffset을 두 번 적용하는 것을 막는다.
                    const bool isLastSection =
                        sectionIndex + 1 == road.sections.size();
                    const int lastSample = isLastSection ? count : count - 1;
                    for (int i = 0; i <= lastSample; ++i) {
                        const double s = section.s + segment.length * i / count;
                        const auto reference = roadToWorld(road, s);
                        const double t = laneOffset(road, section, laneId, s);
                        Waypoint point;
                        point.x = reference.x - t * std::sin(reference.yaw);
                        point.y = reference.y + t * std::cos(reference.yaw);
                        point.yaw = laneId < 0 ? reference.yaw
                                               : normalizeAngle(reference.yaw + kPi);
                        point.roadS = s;
                        point.roadId = roadId;
                        point.laneId = laneId;
                        point.sectionIndex = static_cast<int>(sectionIndex);
                        segment.points.push_back(point);
                    }
                    // RHT OpenDRIVE: 양수 lane은 road s 반대 방향으로 주행한다.
                    if (laneId > 0) std::reverse(segment.points.begin(), segment.points.end());
                    data.laneSegments.emplace(segment.key, std::move(segment));
                }
            }
        }
    }

    // =========================================================================
    // 7. Legal lane 연결 및 checkpoint 구간 탐색 함수
    // =========================================================================

    // 실제로 존재하는 두 lane segment 사이에 유효한 방향 연결을 추가한다.
void addLaneEdge(PathGeneratorData& data, const LaneKey& from,
                 const LaneKey& to) {
        if (data.laneSegments.count(from) && data.laneSegments.count(to))
            data.laneGraph[from].push_back(to);
    }

    // lane successor와 junction laneLink를 이용해 합법적인 연결 그래프를 만든다.
void buildLaneGraph(PathGeneratorData& data) {
        // 같은 road의 인접 laneSection 연결.
        for (const auto& [roadId, road] : data.roads) {
            for (std::size_t i = 0; i < road.sections.size(); ++i) {
                for (const auto& [laneId, lane] : road.sections[i].lanes) {
                    if (lane.type != "driving") continue;
                    const LaneKey from{roadId, static_cast<int>(i), laneId};
                    if (laneId < 0 && i + 1 < road.sections.size()) {
                        addLaneEdge(data, from, {roadId, static_cast<int>(i + 1),
                                            lane.successor.value_or(laneId)});
                    } else if (laneId > 0 && i > 0) {
                        addLaneEdge(data, from, {roadId, static_cast<int>(i - 1),
                                            lane.predecessor.value_or(laneId)});
                    }
                }
            }
        }

        // 같은 road 안에서 다음 junction의 지정 차선으로 이동할 수 있도록
        // 같은 방향의 인접 driving lane 연결을 추가한다. roadMark의
        // laneChange가 실선(none)이면 연결하지 않는다.
        for (const auto& [roadId, road] : data.roads) {
            for (std::size_t sectionIndex = 0;
                 sectionIndex < road.sections.size(); ++sectionIndex) {
                const LaneSection& section = road.sections[sectionIndex];
                for (const auto& [laneId, lane] : section.lanes) {
                    if (lane.type != "driving" || laneId == 0) continue;

                    const int towardCenter = laneId < 0 ? laneId + 1 : laneId - 1;
                    const int awayFromCenter = laneId < 0 ? laneId - 1 : laneId + 1;
                    const int adjacentIds[] = {towardCenter, awayFromCenter};

                    for (int adjacentId : adjacentIds) {
                        const auto adjacent = section.lanes.find(adjacentId);
                        if (adjacent == section.lanes.end() ||
                            adjacent->second.type != "driving") {
                            continue;
                        }
                        if (!laneChangeEverAllowed(section, laneId,
                                                   adjacentId)) {
                            continue;
                        }
                        addLaneEdge(data,
                                    {roadId, static_cast<int>(sectionIndex), laneId},
                                    {roadId, static_cast<int>(sectionIndex), adjacentId});
                    }
                }
            }
        }

        // Junction incoming lane -> connectingRoad lane.
        for (const auto& link : data.junctionLinks) {
            const auto incomingRoad = data.roads.find(link.incomingRoad);
            const auto connectingRoad = data.roads.find(link.connectingRoad);
            if (incomingRoad == data.roads.end() || connectingRoad == data.roads.end()) continue;
            const int incomingSection = link.fromLane < 0
                                            ? static_cast<int>(incomingRoad->second.sections.size()) - 1
                                            : 0;
            const int connectingSection =
                link.contactPoint == "end"
                    ? static_cast<int>(connectingRoad->second.sections.size()) - 1
                    : 0;
            addLaneEdge(data, {link.incomingRoad, incomingSection, link.fromLane},
                         {link.connectingRoad, connectingSection, link.toLane});
        }

        // 일반 road successor/predecessor 연결. Junction 연결은 위 laneLink가 우선한다.
        for (const auto& [roadId, road] : data.roads) {
            for (const auto& [key, segment] : data.laneSegments) {
                if (key.roadId != roadId) continue;
                const bool atRoadEnd = key.laneId < 0 &&
                    key.sectionIndex == static_cast<int>(road.sections.size()) - 1;
                const bool atRoadStart = key.laneId > 0 && key.sectionIndex == 0;
                const RoadLink* link = atRoadEnd ? &road.successor
                                      : atRoadStart ? &road.predecessor
                                                    : nullptr;
                if (!link || !link->valid || link->elementType != "road") continue;
                const auto targetRoad = data.roads.find(link->elementId);
                if (targetRoad == data.roads.end() || targetRoad->second.sections.empty()) continue;
                const int targetSection = link->contactPoint == "end"
                                              ? static_cast<int>(targetRoad->second.sections.size()) - 1
                                              : 0;
                const auto& lane = road.sections[key.sectionIndex].lanes.at(key.laneId);
                const int targetLane = atRoadEnd
                                           ? lane.successor.value_or(key.laneId)
                                           : lane.predecessor.value_or(key.laneId);
                addLaneEdge(data, key, {link->elementId, targetSection, targetLane});
            }
        }
    }

    // 연속된 두 checkpoint 사이의 successor/junction 연결을 순서대로 추적한다.
    // 이것은 전체 경로를 새로 최적화하는 것이 아니라, CSV가 생략한 XODR 내부
    // connectingRoad와 laneSection을 펼치는 과정이다.
std::vector<LaneKey> connectLanes(const PathGeneratorData& data,
                             const LaneKey& start, const LaneKey& goal) {
        if (start == goal) return {start};

        // 거리 cost를 사용하지 않는 BFS이다. road 길이로 경로를 고르지 않는다.
        std::queue<LaneKey> queue;
        std::map<LaneKey, bool> visited;
        std::map<LaneKey, LaneKey> previous;
        visited[start] = true;
        queue.push(start);

        while (!queue.empty()) {
            const LaneKey current = queue.front();
            queue.pop();
            if (current == goal) break;

            const auto neighbors = data.laneGraph.find(current);
            if (neighbors == data.laneGraph.end()) continue;
            for (const auto& next : neighbors->second) {
                if (visited.count(next)) continue;
                visited[next] = true;
                previous[next] = current;
                queue.push(next);
            }
        }

        if (!visited.count(goal)) return {};
        std::vector<LaneKey> route;
        for (LaneKey current = goal;; current = previous.at(current)) {
            route.push_back(current);
            if (current == start) break;
        }
        std::reverse(route.begin(), route.end());
        return route;
    }

// route의 첫 lane은 현재 위치 이후만, 마지막 lane은 목표 위치 이전만 계산한다.
double calculateRouteDistance(const PathGeneratorData& data,
                              const std::vector<LaneKey>& route,
                              const LaneMatch& start,
                              const LaneMatch& goal) {
    if (route.empty()) return std::numeric_limits<double>::infinity();
    if (route.size() == 1) {
        if (goal.pointIndex < start.pointIndex)
            return std::numeric_limits<double>::infinity();
        return std::max(0.0, goal.roadS - start.roadS);
    }

    double distance = 0.0;
    for (std::size_t index = 0; index < route.size(); ++index) {
        const LaneSegment& segment = data.laneSegments.at(route[index]);
        if (index == 0) {
            distance += segment.length *
                        (1.0 - static_cast<double>(start.pointIndex) /
                                   std::max<std::size_t>(1, segment.points.size() - 1));
        } else if (index + 1 == route.size()) {
            distance += segment.length *
                        static_cast<double>(goal.pointIndex) /
                        std::max<std::size_t>(1, segment.points.size() - 1);
        } else {
            distance += segment.length;
        }
    }
    return distance;
}

// 차선 변경이나 road 경계에서 떨어진 두 점 사이를 직선 보간한다.
// legal laneLink 경계점은 XODR 계산이 정확하면 거의 붙어 있어야 하므로,
// kMaxLegalGap을 넘는 간격은 연결 자체가 잘못됐다는 신호로 보고 숨기지
// 않고 큰 소리로 경고한다. 그래도 경로는 계속 이어 붙여 디버깅 가능하게 한다.
void appendInterpolatedGap(const Waypoint& start, const Waypoint& end,
                           double interval, std::vector<Waypoint>& output) {
    constexpr double kMaxLegalGap = 2.0;
    const double gap = std::sqrt(distanceSquared(start.x, start.y, end.x, end.y));
    if (gap <= interval * 1.5) return;

    if (gap > kMaxLegalGap) {
        std::cerr << "[gap-warning] " << gap
             << "m gap between road=" << start.roadId << " lane=" << start.laneId
             << " (roadS=" << start.roadS << ") and road=" << end.roadId
             << " lane=" << end.laneId << " (roadS=" << end.roadS
             << ") exceeds " << kMaxLegalGap
             << "m -- likely a wrong lane connection, not just sampling error\n";
    }

    const int count = std::max(1, static_cast<int>(std::ceil(gap / interval)));
    for (int index = 1; index < count; ++index) {
        const double ratio = static_cast<double>(index) / count;
        Waypoint point = end;
        point.x = start.x + (end.x - start.x) * ratio;
        point.y = start.y + (end.y - start.y) * ratio;
        point.roadS = start.roadS + (end.roadS - start.roadS) * ratio;
        output.push_back(point);
    }
}

// 현재 차선을 따라 전진하면서 smoothstep 비율로 다음 차선 중심선에 합류한다.
// 반환값은 다음 차선에서 이어서 복사할 point index다.
std::size_t appendSmoothLaneChange(const std::vector<Waypoint>& currentLane,
                              const std::vector<Waypoint>& nextLane,
                              std::size_t begin,
                              std::vector<Waypoint>& output) {
    if (currentLane.empty() || nextLane.empty()) return 0;

    begin = std::min(begin, currentLane.size() - 1);
    const std::size_t remaining = currentLane.size() - begin;
    const std::size_t transitionStart = begin + remaining / 3;
    const std::size_t desiredSamples = 40;  // 0.5m sampling 기준 약 20m
    const std::size_t availableSamples =
        std::max<std::size_t>(2, currentLane.size() - transitionStart);
    const std::size_t transitionSamples =
        std::min(desiredSamples, availableSamples);
    const std::size_t transitionEnd =
        std::min(currentLane.size() - 1,
            transitionStart + transitionSamples - 1);

    if (!output.empty()) {
        appendInterpolatedGap(output.back(), currentLane[begin], 0.5, output);
    }
    output.insert(output.end(), currentLane.begin() + begin,
                  currentLane.begin() + transitionStart);

    std::size_t finalNextIndex = 0;
    const std::size_t denominator = std::max<std::size_t>(1, transitionEnd - transitionStart);
    for (std::size_t index = transitionStart; index <= transitionEnd; ++index) {
        const double roadRatio = currentLane.size() > 1
                                     ? static_cast<double>(index) /
                                           (currentLane.size() - 1)
                                     : 0.0;
        const std::size_t nextIndex = std::min(
            nextLane.size() - 1,
            static_cast<std::size_t>(roadRatio * (nextLane.size() - 1)));
        const double progress =
            static_cast<double>(index - transitionStart) / denominator;
        const double blend = progress * progress * (3.0 - 2.0 * progress);

        Waypoint point = currentLane[index];
        point.x = currentLane[index].x +
                  (nextLane[nextIndex].x - currentLane[index].x) * blend;
        point.y = currentLane[index].y +
                  (nextLane[nextIndex].y - currentLane[index].y) * blend;
        if (blend >= 0.5) {
            point.roadId = nextLane[nextIndex].roadId;
            point.laneId = nextLane[nextIndex].laneId;
            point.sectionIndex = nextLane[nextIndex].sectionIndex;
            point.roadS = nextLane[nextIndex].roadS;
        }
        output.push_back(point);
        finalNextIndex = nextIndex;
    }
    return std::min(finalNextIndex + 1, nextLane.size() - 1);
}

    // 첫/마지막 lane은 매칭 지점에서 자르고 중간 lane은 전체를 결과에 붙인다.
void appendRoute(const PathGeneratorData& data,
                 const std::vector<LaneKey>& route,
                 const LaneMatch& start, const LaneMatch& goal,
                 std::vector<Waypoint>& output) {
        std::size_t nextBegin = 0;
        for (std::size_t routeIndex = 0; routeIndex < route.size(); ++routeIndex) {
            const auto& points = data.laneSegments.at(route[routeIndex]).points;
            std::size_t begin = nextBegin;
            std::size_t end = points.size();
            nextBegin = 0;
            if (routeIndex == 0) begin = std::min(start.pointIndex, points.size() - 1);
            if (routeIndex + 1 == route.size())
                end = std::min(goal.pointIndex + 1, points.size());

            // 같은 road/section에서 여러 차선을 연속으로 옮기는 경우 중간
            // 차선마다 꺾지 않고 최초 차선에서 최종 차선까지 한 번만 이동한다.
            std::size_t finalLaneIndex = routeIndex;
            while (finalLaneIndex + 1 < route.size() &&
                   route[routeIndex].roadId ==
                       route[finalLaneIndex + 1].roadId &&
                   route[routeIndex].sectionIndex ==
                       route[finalLaneIndex + 1].sectionIndex) {
                ++finalLaneIndex;
            }
            if (finalLaneIndex > routeIndex) {
                const auto& finalLanePoints =
                    data.laneSegments.at(route[finalLaneIndex]).points;
                nextBegin = appendSmoothLaneChange(points, finalLanePoints,
                                                   begin, output);
                routeIndex = finalLaneIndex - 1;
                continue;
            }
            if (begin >= end) continue;
            if (!output.empty()) {
                appendInterpolatedGap(output.back(), points[begin],
                                      data.sampleInterval, output);
            }
            output.insert(output.end(), points.begin() + begin, points.begin() + end);
        }
    }

    // =========================================================================
    // 8. 최종 경로 후처리 함수
    // =========================================================================

    // lane 경계에서 중복 삽입된 거의 동일한 점을 제거한다.
void removeDuplicates(std::vector<Waypoint>& path) {
        if (path.empty()) return;
        std::vector<Waypoint> filtered;
        filtered.reserve(path.size());
        filtered.push_back(path.front());
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (distanceSquared(filtered.back().x, filtered.back().y, path[i].x,
                          path[i].y) > 0.01) {
                filtered.push_back(path[i]);
            }
        }
        path.swap(filtered);
    }

    // 최종 점 순서를 기준으로 yaw와 누적거리 s를 다시 계산한다.
void updateYawAndS(std::vector<Waypoint>& path) {
        if (path.empty()) return;
        path.front().s = 0.0;
        for (std::size_t i = 1; i < path.size(); ++i) {
            const double dx = path[i].x - path[i - 1].x;
            const double dy = path[i].y - path[i - 1].y;
            path[i].s = path[i - 1].s + std::hypot(dx, dy);
            if (std::hypot(dx, dy) > kEpsilon) path[i - 1].yaw = std::atan2(dy, dx);
        }
        if (path.size() > 1) path.back().yaw = path[path.size() - 2].yaw;
    }

// 생성된 전역경로를 디버깅/시각화용 CSV로 저장한다.
bool savePath(const std::vector<Waypoint>& path, const std::string& outputPath) {
    std::ofstream output(outputPath);
    if (!output) return false;
    output << "x,y,yaw,s,road_id,lane_id,road_s\n";
    output << std::fixed << std::setprecision(8);
    for (const auto& point : path) {
        output << point.x << ',' << point.y << ',' << point.yaw << ','
               << point.s << ',' << point.roadId << ',' << point.laneId
               << ',' << point.roadS << '\n';
    }
    return true;
}
