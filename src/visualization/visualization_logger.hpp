#pragma once

#include "control/control_structs.hpp"
#include "data/gt/gt_structs.hpp"
#include "data/map/map_structs.hpp"
#include "logger/decision_trace.hpp"
#include "planning/planning_structs.hpp"

#include <cstddef>
#include <filesystem>

// ROS에 의존하지 않는 RViz 중계용 출력기다. 주행 프로세스는 정적 지도
// CSV와 최신 프레임 JSON만 쓰고, tools/visualization/rviz_bridge.py가 이를
// visualization_msgs/MarkerArray로 변환한다.
struct VisualizationLoggerData {
    bool enabled{};
    std::filesystem::path outputDirectory;
    std::filesystem::path staticMapPath;
    std::filesystem::path latestFramePath;
};

void initializeVisualizationLogger(
    VisualizationLoggerData& data,
    const std::filesystem::path& outputDirectory,
    const PathGeneratorData& map,
    const MissionCorridorData& mission);

void writeVisualizationFrame(
    VisualizationLoggerData& data,
    std::size_t frameIndex,
    double elapsedTime,
    const WorldState& world,
    const PlanningProcessData& planning,
    const ControlProcessData& control,
    const DecisionTraceData* trace);
