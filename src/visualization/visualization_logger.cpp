#include "visualization/visualization_logger.hpp"

#include "planning/global/reference_path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <tuple>

namespace {

std::string csvEscape(const std::string& value) {
    std::string escaped = "\"";
    for (char character : value) {
        if (character == '\"') escaped += '\"';
        escaped += character;
    }
    escaped += '\"';
    return escaped;
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream output;
    for (unsigned char character : value) {
        switch (character) {
            case '\\': output << "\\\\"; break;
            case '\"': output << "\\\""; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default:
                if (character < 0x20) {
                    output << "?";
                } else {
                    output << static_cast<char>(character);
                }
        }
    }
    return output.str();
}

const LaneSection* findSection(const Road& road, int sectionIndex) {
    if (sectionIndex < 0 ||
        static_cast<std::size_t>(sectionIndex) >= road.sections.size()) {
        return nullptr;
    }
    return &road.sections[static_cast<std::size_t>(sectionIndex)];
}

const LaneSection* findSectionAt(const Road& road, double roadS) {
    if (road.sections.empty()) return nullptr;
    const LaneSection* selected = &road.sections.front();
    for (const auto& section : road.sections) {
        if (section.s <= roadS + 1e-6) selected = &section;
        else break;
    }
    return selected;
}

const RoadMarkEntry* roadMarkAt(const LaneDefinition& lane, double roadS) {
    if (lane.roadMarks.empty()) return nullptr;
    const RoadMarkEntry* selected = &lane.roadMarks.front();
    for (const auto& mark : lane.roadMarks) {
        if (mark.s <= roadS + 1e-6) selected = &mark;
        else break;
    }
    return selected;
}

void writeStaticPoint(std::ofstream& output,
                      const std::string& layer,
                      const std::string& group,
                      int roadId,
                      int sectionIndex,
                      int laneId,
                      std::size_t pointIndex,
                      double x,
                      double y,
                      double z,
                      const std::string& style,
                      const std::string& label = {}) {
    output << csvEscape(layer) << ',' << csvEscape(group) << ','
           << roadId << ',' << sectionIndex << ',' << laneId << ','
           << pointIndex << ',' << std::fixed << std::setprecision(4)
           << x << ',' << y << ',' << z << ',' << csvEscape(style) << ','
           << csvEscape(label) << '\n';
}

void writeLaneCenters(std::ofstream& output, const PathGeneratorData& map) {
    const std::size_t stride = static_cast<std::size_t>(std::max(
        1.0, std::round(1.0 / std::max(0.05, map.sampleInterval))));
    for (const auto& [key, segment] : map.laneSegments) {
        const std::string group = std::to_string(key.roadId) + ":" +
            std::to_string(key.sectionIndex) + ":" +
            std::to_string(key.laneId);
        for (std::size_t index = 0; index < segment.points.size();
             index += stride) {
            const auto& point = segment.points[index];
            writeStaticPoint(output, "lane_center", group, key.roadId,
                             key.sectionIndex, key.laneId, index,
                             point.x, point.y, 0.03, "driving");
        }
        if (segment.points.size() > 1 &&
            (segment.points.size() - 1) % stride != 0) {
            const std::size_t index = segment.points.size() - 1;
            const auto& point = segment.points[index];
            writeStaticPoint(output, "lane_center", group, key.roadId,
                             key.sectionIndex, key.laneId, index,
                             point.x, point.y, 0.03, "driving");
        }
    }
}

void writeRoadCenters(std::ofstream& output, const PathGeneratorData& map) {
    const double interval = std::max(1.0, map.sampleInterval);
    for (const auto& [roadId, road] : map.roads) {
        std::string previousStyle;
        std::size_t run = 0;
        std::size_t index = 0;
        for (double s = 0.0; s <= road.length + 1e-6; s += interval) {
            const double clampedS = std::min(s, road.length);
            const RoadPose pose = roadToWorld(road, clampedS);
            const double offset = evaluatePiecewise(
                road.laneOffsets, clampedS, 0.0);
            std::string style = "reference";
            const LaneSection* section = findSectionAt(road, clampedS);
            if (section != nullptr) {
                const auto center = section->lanes.find(0);
                if (center != section->lanes.end()) {
                    const RoadMarkEntry* mark = roadMarkAt(
                        center->second, clampedS);
                    if (mark != nullptr) {
                        style = mark->type + "|" + mark->color + "|" +
                                mark->laneChange;
                    }
                }
            }
            if (!previousStyle.empty() && style != previousStyle) {
                ++run;
                index = 0;
            }
            previousStyle = style;
            const std::string group = std::to_string(roadId) + ":" +
                                      std::to_string(run);
            writeStaticPoint(output, "road_center", group, roadId, -1, 0,
                             index++, pose.x - offset * std::sin(pose.yaw),
                             pose.y + offset * std::cos(pose.yaw), 0.035,
                             style);
        }
    }
}

void writeLaneBoundaries(std::ofstream& output,
                         const PathGeneratorData& map) {
    const std::size_t stride = static_cast<std::size_t>(std::max(
        1.0, std::round(1.0 / std::max(0.05, map.sampleInterval))));
    for (const auto& [key, segment] : map.laneSegments) {
        const auto roadFound = map.roads.find(key.roadId);
        if (roadFound == map.roads.end()) continue;
        const Road& road = roadFound->second;
        const LaneSection* section = findSection(road, key.sectionIndex);
        if (section == nullptr) continue;
        const auto laneFound = section->lanes.find(key.laneId);
        if (laneFound == section->lanes.end()) continue;

        std::string previousStyle;
        std::size_t run = 0;
        std::size_t pointIndex = 0;
        for (std::size_t sourceIndex = 0;
             sourceIndex < segment.points.size(); sourceIndex += stride) {
            const auto& point = segment.points[sourceIndex];
            const RoadMarkEntry* mark = roadMarkAt(laneFound->second,
                                                   point.roadS);
            const std::string style = mark == nullptr
                ? "none|standard|none"
                : mark->type + "|" + mark->color + "|" + mark->laneChange;
            if (!previousStyle.empty() && style != previousStyle) {
                ++run;
                pointIndex = 0;
            }
            previousStyle = style;
            const RoadPose reference = roadToWorld(road, point.roadS);
            const double center = laneOffset(road, *section, key.laneId,
                                             point.roadS);
            const double width = laneWidthAt(*section, key.laneId,
                                             point.roadS);
            const double edge = center + (key.laneId > 0 ? 0.5 : -0.5) * width;
            const std::string group = std::to_string(key.roadId) + ":" +
                std::to_string(key.sectionIndex) + ":" +
                std::to_string(key.laneId) + ":" + std::to_string(run);
            writeStaticPoint(output, "lane_boundary", group, key.roadId,
                             key.sectionIndex, key.laneId, pointIndex++,
                             reference.x - edge * std::sin(reference.yaw),
                             reference.y + edge * std::cos(reference.yaw),
                             0.045, style);
        }
    }
}

void writeStaticFeatures(std::ofstream& output,
                         const PathGeneratorData& map) {
    for (const auto& line : map.staticZones.stopLines) {
        const std::string group = std::to_string(line.id);
        const std::string label = "stop_line " + std::to_string(line.id) +
            " controller=" + std::to_string(line.controllerId);
        writeStaticPoint(output, "stop_line", group, line.roadId, -1, 0, 0,
                         line.begin.x, line.begin.y, 0.08, "white", label);
        writeStaticPoint(output, "stop_line", group, line.roadId, -1, 0, 1,
                         line.end.x, line.end.y, 0.08, "white", label);
    }
    for (const auto& crosswalk : map.staticZones.crosswalks) {
        const std::string group = std::to_string(crosswalk.id);
        for (std::size_t index = 0; index < crosswalk.polygon.size(); ++index) {
            const auto& point = crosswalk.polygon[index];
            writeStaticPoint(output, "crosswalk", group, crosswalk.roadId,
                             -1, 0, index, point.x, point.y, 0.07, "white",
                             "crosswalk " + std::to_string(crosswalk.id));
        }
        if (!crosswalk.polygon.empty()) {
            const auto& point = crosswalk.polygon.front();
            writeStaticPoint(output, "crosswalk", group, crosswalk.roadId,
                             -1, 0, crosswalk.polygon.size(), point.x, point.y,
                             0.07, "white");
        }
    }
    for (const auto& signal : map.roadSignals) {
        const auto roadFound = map.roads.find(signal.roadId);
        if (roadFound == map.roads.end()) continue;
        const RoadPose pose = roadToWorld(roadFound->second, signal.s);
        const int controller = [&]() {
            const auto found = map.signalToController.find(signal.id);
            return found == map.signalToController.end() ? -1 : found->second;
        }();
        writeStaticPoint(output, "traffic_signal", std::to_string(signal.id),
                         signal.roadId, -1, 0, 0,
                         pose.x - signal.t * std::sin(pose.yaw),
                         pose.y + signal.t * std::cos(pose.yaw), 1.3,
                         "controller=" + std::to_string(controller),
                         "signal " + std::to_string(signal.id));
    }
}

void writeMission(std::ofstream& output,
                  const PathGeneratorData& map,
                  const MissionCorridorData& mission) {
    for (std::size_t index = 0; index < mission.checkpoints.size(); ++index) {
        const auto& checkpoint = mission.checkpoints[index].checkpoint;
        writeStaticPoint(output, "checkpoint", std::to_string(index), -1,
                         -1, 0, 0, checkpoint.x, checkpoint.y, 0.30, "red",
                         "CP" + std::to_string(checkpoint.sequence));
    }
    for (std::size_t legIndex = 0; legIndex < mission.legs.size(); ++legIndex) {
        const auto& leg = mission.legs[legIndex];
        for (const auto& key : leg.allowedLanes) {
            const auto found = map.laneSegments.find(key);
            if (found == map.laneSegments.end()) continue;
            const std::string group = std::to_string(legIndex) + ":" +
                std::to_string(key.roadId) + ":" +
                std::to_string(key.sectionIndex) + ":" +
                std::to_string(key.laneId);
            for (std::size_t pointIndex = 0;
                 pointIndex < found->second.points.size(); ++pointIndex) {
                const auto& point = found->second.points[pointIndex];
                writeStaticPoint(output, "corridor", group, key.roadId,
                                 key.sectionIndex, key.laneId, pointIndex,
                                 point.x, point.y, 0.015, "leg=" +
                                 std::to_string(legIndex));
            }
        }
        for (const auto& key : leg.targetLanes) {
            const auto found = map.laneSegments.find(key);
            if (found == map.laneSegments.end()) continue;
            const std::string group = std::to_string(legIndex) + ":" +
                std::to_string(key.roadId) + ":" +
                std::to_string(key.sectionIndex) + ":" +
                std::to_string(key.laneId);
            for (std::size_t pointIndex = 0;
                 pointIndex < found->second.points.size(); ++pointIndex) {
                const auto& point = found->second.points[pointIndex];
                writeStaticPoint(output, "gateway", group, key.roadId,
                                 key.sectionIndex, key.laneId, pointIndex,
                                 point.x, point.y, 0.025, "leg=" +
                                 std::to_string(legIndex));
            }
        }
    }
}

template <typename PointContainer>
void writePointArray(std::ostream& output, const PointContainer& points,
                     double z = 0.0, std::size_t stride = 1) {
    output << '[';
    bool first = true;
    for (std::size_t index = 0; index < points.size(); index += stride) {
        if (!first) output << ',';
        first = false;
        output << '[' << points[index].x << ',' << points[index].y << ','
               << z << ']';
    }
    if (points.size() > 1 && (points.size() - 1) % stride != 0) {
        const auto& point = points.back();
        if (!first) output << ',';
        output << '[' << point.x << ',' << point.y << ',' << z << ']';
    }
    output << ']';
}

const char* phaseName(LaneChangePhase phase) {
    switch (phase) {
        case LaneChangePhase::None: return "none";
        case LaneChangePhase::Prepare: return "prepare";
        case LaneChangePhase::Execute: return "execute";
        case LaneChangePhase::Settle: return "settle";
    }
    return "unknown";
}

const char* healthName(SystemHealth health) {
    return health == SystemHealth::Ok ? "ok" : "recovery";
}

std::string stopReasonText(StopReason reason) {
    if (reason == StopReason::None) return "none";
    const auto bits = static_cast<std::uint32_t>(reason);
    std::string result;
    const auto append = [&](const char* name) {
        if (!result.empty()) result += '+';
        result += name;
    };
    if ((bits & static_cast<std::uint32_t>(StopReason::RedLight)) != 0U) {
        append("red_light");
    }
    if ((bits & static_cast<std::uint32_t>(StopReason::FlashingRed)) != 0U) {
        append("flashing_red");
    }
    if ((bits & static_cast<std::uint32_t>(StopReason::Pedestrian)) != 0U) {
        append("pedestrian");
    }
    if ((bits & static_cast<std::uint32_t>(StopReason::CrosswalkBlocked)) != 0U) {
        append("crosswalk_blocked");
    }
    return result.empty() ? "unknown" : result;
}

}  // namespace

void initializeVisualizationLogger(
    VisualizationLoggerData& data,
    const std::filesystem::path& outputDirectory,
    const PathGeneratorData& map,
    const MissionCorridorData& mission) {
    data = {};
    data.outputDirectory = outputDirectory;
    data.staticMapPath = outputDirectory / "static_map.csv";
    data.latestFramePath = outputDirectory / "latest.json";
    std::filesystem::create_directories(outputDirectory);

    const std::filesystem::path temporary =
        data.staticMapPath.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
        throw std::runtime_error("cannot write RViz static map: " +
                                 data.staticMapPath.string());
    }
    output << "layer,group,road_id,section_index,lane_id,point_index,"
              "x,y,z,style,label\n";
    writeRoadCenters(output, map);
    writeLaneCenters(output, map);
    writeLaneBoundaries(output, map);
    writeStaticFeatures(output, map);
    if (mission.valid) writeMission(output, map, mission);
    output.close();
    if (!output) {
        throw std::runtime_error("cannot finish RViz static map: " +
                                 data.staticMapPath.string());
    }
    std::error_code error;
    std::filesystem::rename(temporary, data.staticMapPath, error);
    if (error) {
        std::filesystem::remove(data.staticMapPath, error);
        error.clear();
        std::filesystem::rename(temporary, data.staticMapPath, error);
    }
    if (error) {
        throw std::runtime_error("cannot publish RViz static map: " +
                                 data.staticMapPath.string());
    }
    data.enabled = true;
}

void writeVisualizationFrame(
    VisualizationLoggerData& data,
    std::size_t frameIndex,
    double elapsedTime,
    const WorldState& world,
    const PlanningProcessData& planning,
    const ControlProcessData& control,
    const DecisionTraceData* trace) {
    if (!data.enabled) return;
    const std::filesystem::path temporary =
        data.latestFramePath.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) return;
    output << std::fixed << std::setprecision(4);
    output << "{\"frame\":" << frameIndex
           << ",\"time_s\":" << elapsedTime
           << ",\"ego\":{\"x\":" << world.ego.x
           << ",\"y\":" << world.ego.y
           << ",\"z\":" << world.ego.z
           << ",\"heading\":" << world.ego.heading
           << ",\"speed\":" << world.egoSpeed
           << ",\"road_id\":" << world.egoRoadId
           << ",\"section_index\":" << world.egoSectionIndex
           << ",\"lane_id\":" << world.egoLaneId
           << ",\"obb\":";
    writePointArray(output, world.egoObb.corners, 0.20);
    output << "},\"objects\":[";
    for (std::size_t index = 0; index < world.objects.size(); ++index) {
        if (index != 0) output << ',';
        const auto& object = world.objects[index];
        output << "{\"id\":" << object.raw.id
               << ",\"speed\":" << object.raw.speed
               << ",\"road_id\":" << object.roadId
               << ",\"section_index\":" << object.sectionIndex
               << ",\"lane_id\":" << object.laneId
               << ",\"ahead\":" << (object.isAhead ? "true" : "false")
               << ",\"on_path\":" << (object.isOnEgoPath ? "true" : "false")
               << ",\"collision\":" << (object.isCollisionRisk ? "true" : "false")
               << ",\"path_s\":" << object.pathS
               << ",\"path_d\":" << object.pathD
               << ",\"ttc\":";
        if (std::isfinite(object.ttc)) output << object.ttc;
        else output << "null";
        output << ",\"obb\":";
        writePointArray(output, object.obb.corners, 0.16);
        output << '}';
    }
    output << "],\"candidate_lanes\":[";
    if (trace != nullptr) {
        for (std::size_t index = 0; index < trace->candidateLanes.size(); ++index) {
            if (index != 0) output << ',';
            const auto& lane = trace->candidateLanes[index];
            output << '[' << lane.roadId << ',' << lane.sectionIndex << ','
                   << lane.laneId << ']';
        }
    }
    output << "],\"candidate_paths\":[";
    if (trace != nullptr) {
        for (std::size_t index = 0; index < trace->candidatePaths.size(); ++index) {
            if (index != 0) output << ',';
            const auto& path = trace->candidatePaths[index];
            output << "{\"lane\":[" << path.lane.roadId << ','
                   << path.lane.sectionIndex << ',' << path.lane.laneId
                   << "],\"status\":\"" << jsonEscape(path.status)
                   << "\",\"reason\":\"" << jsonEscape(path.reason)
                   << "\",\"points\":";
            writePointArray(output, path.points, 0.35, 2);
            output << '}';
        }
    }
    output << "],\"selected_lane\":[";
    if (planning.laneChange.shiftCompletedLaneValid) {
        const auto& lane = planning.laneChange.shiftCompletedLane;
        output << lane.roadId << ',' << lane.sectionIndex << ',' << lane.laneId;
    } else {
        output << world.egoRoadId << ',' << world.egoSectionIndex << ','
               << (planning.laneChange.phase == LaneChangePhase::None
                       ? world.egoLaneId
                       : planning.laneChange.targetLaneId);
    }
    output << "],\"tracked_path\":";
    writePointArray(output, planning.result.localPath.points, 0.42, 2);
    output << ",\"maneuver_path\":";
    writePointArray(output, planning.laneChange.path, 0.38, 2);
    output << ",\"decision\":{\"target_speed\":"
           << planning.result.targetSpeed
           << ",\"steering\":" << control.safeCommand.steering
           << ",\"target_accel\":" << control.safeCommand.targetAccel
           << ",\"turn_signal\":"
           << static_cast<unsigned int>(control.safeCommand.turnSignal)
           << ",\"emergency_brake\":"
           << (planning.result.emergencyBrake ? "true" : "false")
           << ",\"plan_valid\":"
           << (planning.result.valid ? "true" : "false")
           << ",\"stop_reason\":"
           << static_cast<std::uint32_t>(planning.result.stopReason)
           << ",\"stop_reason_text\":\""
           << stopReasonText(planning.result.stopReason) << "\""
           << ",\"health\":\"" << healthName(planning.result.health)
           << "\",\"lane_change_phase\":\""
           << phaseName(planning.laneChange.phase)
           << "\",\"target_lane\":" << planning.laneChange.targetLaneId
           << ",\"road_connection\":"
           << (planning.laneChange.roadConnectionManeuver ? "true" : "false")
           << ",\"best_progress_fallback\":"
           << (planning.laneChange.bestProgressFallback ? "true" : "false")
           << ",\"active_leg\":" << planning.missionCorridor.activeLegIndex
           << "},\"traffic_light\":{\"id\":"
           << world.trafficLight.id << ",\"state\":"
           << static_cast<unsigned int>(world.trafficLight.state) << "},"
           << "\"events\":[";
    if (trace != nullptr) {
        for (std::size_t index = 0; index < trace->frameEvents.size(); ++index) {
            if (index != 0) output << ',';
            output << '\"' << jsonEscape(trace->frameEvents[index]) << '\"';
        }
    }
    output << "]}\n";
    output.close();
    if (!output) return;
    std::error_code error;
    std::filesystem::rename(temporary, data.latestFramePath, error);
    if (error) {
        std::filesystem::remove(data.latestFramePath, error);
        error.clear();
        std::filesystem::rename(temporary, data.latestFramePath, error);
    }
}
