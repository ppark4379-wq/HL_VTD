#include "logger/logger.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

constexpr const char* kCsvHeader =
    "scenario_id,run_id,frame,elapsed_wall_time_s,"
    "ego_x,ego_y,ego_z,ego_heading,ego_pitch,ego_roll,ego_speed,"
    "object_count,nearest_object_id,nearest_object_x,nearest_object_y,"
    "nearest_object_speed,nearest_object_distance,"
    "traffic_light_id,traffic_light_state,"
    "plan_valid,goal_reached,nearest_path_index,target_path_index,"
    "lookahead,max_curvature,target_speed,planned_steering,"
    "command_steering,target_accel,turn_signal,"
    "ego_reference_x,ego_reference_y,rear_axle_x,rear_axle_y,"
    "front_bumper_x,front_bumper_y,"
    "current_road_id,current_lane_id,lane_width_m,"
    "lane_center_distance_m,path_s_m,path_d_m,"
    "nearest_object_heading,nearest_object_length,nearest_object_width,"
    "nearest_object_longitudinal_m,nearest_object_lateral_m,"
    "nearest_object_relative_speed_mps,nearest_object_ttc_s,"
    "evaluation_section_id,current_speed_limit_kmh,school_zone_active,"
    "stop_line_distance_m,crosswalk_active,lane_change_active,"
    "solid_line_crossing,centerline_intrusion_m,sidewalk_intrusion_m,"
    "pedestrian_crossing_complete,collision,respawn_count\n";

struct ObjectLogSummary {
    std::size_t count{};
    NearestObjectLog nearest;
};

std::string sanitizeCsvField(std::string value) {
    for (char& character : value) {
        if (character == ',' ||
            character == '\n' ||
            character == '\r') {
            character = '_';
        }
    }
    return value;
}

bool isPopulatedObject(const ObjectState& object) {
    constexpr float epsilon = 1e-6F;
    return object.id != 0 ||
           std::fabs(object.x) > epsilon ||
           std::fabs(object.y) > epsilon ||
           std::fabs(object.z) > epsilon ||
           std::fabs(object.heading) > epsilon ||
           std::fabs(object.speed) > epsilon ||
           std::fabs(object.length) > epsilon ||
           std::fabs(object.width) > epsilon ||
           std::fabs(object.height) > epsilon;
}

ObjectLogSummary summarizeObjects(const GtFrame& gt, double egoSpeed) {
    ObjectLogSummary summary;
    summary.nearest.distance = std::numeric_limits<double>::infinity();

    for (const ObjectState& object : gt.objects) {
        if (!isPopulatedObject(object)) {
            continue;
        }
        ++summary.count;

        const double distance = std::hypot(
            static_cast<double>(object.x - gt.ego.x),
            static_cast<double>(object.y - gt.ego.y));
        if (!summary.nearest.valid ||
            distance < summary.nearest.distance) {
            summary.nearest.valid = true;
            summary.nearest.id = object.id;
            summary.nearest.x = object.x;
            summary.nearest.y = object.y;
            summary.nearest.heading = object.heading;
            summary.nearest.speed = object.speed;
            summary.nearest.length = object.length;
            summary.nearest.width = object.width;
            summary.nearest.distance = distance;

            const double dx = static_cast<double>(object.x - gt.ego.x);
            const double dy = static_cast<double>(object.y - gt.ego.y);
            const double cosine = std::cos(gt.ego.heading);
            const double sine = std::sin(gt.ego.heading);
            summary.nearest.longitudinalDistance = dx * cosine + dy * sine;
            summary.nearest.lateralDistance = -dx * sine + dy * cosine;

            const double objectLongitudinalSpeed =
                static_cast<double>(object.speed) *
                std::cos(static_cast<double>(object.heading - gt.ego.heading));
            summary.nearest.relativeSpeed = egoSpeed - objectLongitudinalSpeed;
            summary.nearest.ttcValid =
                summary.nearest.longitudinalDistance > 0.0 &&
                summary.nearest.relativeSpeed > 0.1;
            if (summary.nearest.ttcValid) {
                summary.nearest.ttc =
                    summary.nearest.longitudinalDistance /
                    summary.nearest.relativeSpeed;
            }
        }
    }
    return summary;
}

LogRecord makeLogRecord(const LogFrameInput& input) {
    const ObjectLogSummary objects = summarizeObjects(
        input.gt, input.control.egoSpeed);

    LogRecord record;
    record.frameIndex = input.frameIndex;
    record.elapsedWallTimeS = input.elapsedWallTimeS;
    record.ego = input.gt.ego;
    record.egoSpeed = input.control.egoSpeed;
    record.egoLaneValid = input.world.egoLaneValid;
    record.egoRoadId = input.world.egoRoadId;
    record.egoLaneId = input.world.egoLaneId;
    record.egoLaneWidth = input.world.egoLaneWidth;
    record.egoLaneCenterDistance = input.world.egoLaneCenterDistance;
    record.egoPathValid = input.world.egoPathValid;
    record.egoPathS = input.world.egoPathS;
    record.egoPathD = input.world.egoPathD;
    record.objectCount = objects.count;
    record.nearestObject = objects.nearest;
    record.trafficLight = input.gt.trafficLight;
    record.plan = input.planning.trackingPlan;
    // trackingPlan.valid only says that Pure Pursuit produced a numerical
    // command.  The controller gates that command with PlanningResult::valid
    // (active-path projection included), so log the same validity that is
    // actually sent to the safety limiter.
    record.plan.valid = input.planning.valid;
    record.command = input.control.latestCommand;
    record.currentSpeedLimit = input.planning.currentSpeedLimit;
    record.schoolZoneActive = input.planning.schoolZoneActive;
    record.stopDistanceValid = input.planning.stopDistanceValid;
    record.stopDistance = input.planning.stopDistance;
    record.crosswalkActive = input.planning.crosswalkActive;
    record.laneChangeActive = input.planning.laneChangeActive;
    record.collision = input.planning.pathSafety.collision;
    return record;
}

void openCsvOutput(CsvLoggerData& logger) {
    logger.output.open(logger.outputPath);
    if (!logger.output) {
        throw std::runtime_error(
            "cannot open log file: " + logger.outputPath.string());
    }
}

void writeCsvHeader(CsvLoggerData& logger) {
    logger.output << kCsvHeader;
}

void configureCsvFormat(CsvLoggerData& logger) {
    logger.output << std::setprecision(9);
}

void writeNearestObject(std::ofstream& output,
                        const NearestObjectLog& object) {
    if (!object.valid) {
        output << ",,,,,";
        return;
    }

    output
        << object.id << ','
        << object.x << ','
        << object.y << ','
        << object.speed << ','
        << object.distance << ',';
}

void writeCsvRecord(CsvLoggerData& logger,
                    const LogRecord& record) {
    logger.output
        << logger.scenarioId << ','
        << logger.runId << ','
        << record.frameIndex << ','
        << record.elapsedWallTimeS << ','
        << record.ego.x << ','
        << record.ego.y << ','
        << record.ego.z << ','
        << record.ego.heading << ','
        << record.ego.pitch << ','
        << record.ego.roll << ','
        << record.egoSpeed << ','
        << record.objectCount << ',';

    writeNearestObject(logger.output, record.nearestObject);

    logger.output
        << record.trafficLight.id << ','
        << static_cast<unsigned int>(record.trafficLight.state) << ','
        << static_cast<int>(record.plan.valid) << ','
        << static_cast<int>(record.plan.goalReached) << ','
        << record.plan.nearestIndex << ','
        << record.plan.targetIndex << ','
        << record.plan.lookahead << ','
        << record.plan.maxCurvature << ','
        << record.plan.targetSpeed << ','
        << record.plan.steering << ','
        << record.command.steering << ','
        << record.command.targetAccel << ','
        << static_cast<unsigned int>(record.command.turnSignal) << ','
        // 운영측 안내문은 제어 차량 기준 좌표를 후륜축으로 정의한다.
        << record.ego.x << ','
        << record.ego.y << ','
        << record.ego.x << ','
        << record.ego.y << ',';

    // 현재 GT에는 Ego 차량 길이/앞 오버행이 없어 앞 범퍼는 계산하지 않는다.
    logger.output << ",,";

    if (record.egoLaneValid) {
        logger.output
            << record.egoRoadId << ','
            << record.egoLaneId << ','
            << record.egoLaneWidth << ','
            << record.egoLaneCenterDistance << ',';
    } else {
        logger.output << ",,,,";
    }

    if (record.egoPathValid) {
        logger.output << record.egoPathS << ',' << record.egoPathD << ',';
    } else {
        logger.output << ",,";
    }

    if (record.nearestObject.valid) {
        logger.output
            << record.nearestObject.heading << ','
            << record.nearestObject.length << ','
            << record.nearestObject.width << ','
            << record.nearestObject.longitudinalDistance << ','
            << record.nearestObject.lateralDistance << ','
            << record.nearestObject.relativeSpeed << ',';
        if (record.nearestObject.ttcValid) {
            logger.output << record.nearestObject.ttc;
        }
        logger.output << ',';
    } else {
        logger.output << ",,,,,,,";
    }

    // Planner가 실제 적용한 값은 공식 평가 열에도 남긴다. 도로 침범과
    // respawn처럼 현재 GT만으로 신뢰성 있게 계산할 수 없는 값만 비워 둔다.
    logger.output << ',';  // evaluation_section_id
    if (std::isfinite(record.currentSpeedLimit)) {
        logger.output << record.currentSpeedLimit * 3.6;
    }
    logger.output
        << ',' << static_cast<int>(record.schoolZoneActive) << ',';
    if (record.stopDistanceValid && std::isfinite(record.stopDistance)) {
        logger.output << record.stopDistance;
    }
    logger.output
        << ',' << static_cast<int>(record.crosswalkActive)
        << ',' << static_cast<int>(record.laneChangeActive)
        // solid_line_crossing, centerline_intrusion_m,
        // sidewalk_intrusion_m, pedestrian_crossing_complete
        << ",,,,," << static_cast<int>(record.collision)
        << ",\n";  // respawn_count
}

void flushCsvOutput(CsvLoggerData& logger) {
    // 실패 재현 중 프로그램이 중단되어도 최신 프레임을 최대한 보존한다.
    logger.output.flush();
    if (!logger.output) {
        throw std::runtime_error(
            "failed to write log file: " + logger.outputPath.string());
    }
}

}  // namespace

void initializeCsvLogger(CsvLoggerData& logger,
                         const CsvLoggerConfig& config) {
    logger.outputPath = config.outputPath;
    logger.scenarioId = sanitizeCsvField(config.scenarioId);
    logger.runId = sanitizeCsvField(config.runId);

    openCsvOutput(logger);
    writeCsvHeader(logger);
    configureCsvFormat(logger);
}

void runLoggingProcess(CsvLoggerData& logger,
                       const LogFrameInput& input) {
    const LogRecord record = makeLogRecord(input);
    writeCsvRecord(logger, record);
    flushCsvOutput(logger);
}
