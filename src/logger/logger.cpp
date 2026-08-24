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
    "command_steering,target_accel,turn_signal\n";

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

ObjectLogSummary summarizeObjects(const GtFrame& gt) {
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
            summary.nearest.speed = object.speed;
            summary.nearest.distance = distance;
        }
    }
    return summary;
}

LogRecord makeLogRecord(const LogFrameInput& input) {
    const ObjectLogSummary objects = summarizeObjects(input.gt);

    LogRecord record;
    record.frameIndex = input.frameIndex;
    record.elapsedWallTimeS = input.elapsedWallTimeS;
    record.ego = input.gt.ego;
    record.egoSpeed = input.control.egoSpeed;
    record.objectCount = objects.count;
    record.nearestObject = objects.nearest;
    record.trafficLight = input.gt.trafficLight;
    record.plan = input.plan;
    record.command = input.control.latestCommand;
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
        << static_cast<unsigned int>(record.command.turnSignal)
        << '\n';
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
