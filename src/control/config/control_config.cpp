#include "control/config/control_config.hpp"

#include "utils/csv_utils.hpp"

#include <fstream>
#include <map>


// YAML 주석을 제거한다.
std::string removeYamlComment(const std::string& line) {
    const std::size_t position = line.find('#');
    return trim(position == std::string::npos ? line : line.substr(0, position));
}

// section/key 형태의 간단한 숫자 YAML을 map으로 읽는다.
bool readYamlNumbers(const std::string& yamlPath, std::map<std::string, double>& values) {
    std::ifstream input(yamlPath);
    if (!input) return false;

    std::string section;
    std::string rawLine;
    while (std::getline(input, rawLine)) {
        const std::string line = removeYamlComment(rawLine);
        if (line.empty()) continue;

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) return false;
        const std::string key = trim(line.substr(0, colon));
        const std::string text = trim(line.substr(colon + 1));

        if (text.empty()) {
            section = key;
            continue;
        }

        double value = 0.0;
        if (section.empty() || !parseDouble(text, value)) return false;
        values[section + "." + key] = value;
    }
    return true;
}

// 필수 설정값 하나를 map에서 구조체 필드로 복사한다.
bool setConfigValue(const std::map<std::string, double>& values,
                    const std::string& key,
                    double& output) {
    const auto found = values.find(key);
    if (found == values.end()) return false;
    output = found->second;
    return true;
}

// km/h로 적힌 설정을 내부 단위인 m/s로 바꿔 읽는다.
// 속도만 km/h를 쓰고 거리·시간·가속도는 m, s, m/s^2 그대로 둔다.
// 사람이 채점 기준(km/h)과 직접 비교할 수 있는 값만 km/h로 표기한다.
bool setSpeedConfigValue(const std::map<std::string, double>& values,
                         const std::string& key,
                         double& output) {
    const auto found = values.find(key);
    if (found == values.end()) return false;
    output = found->second / 3.6;
    return true;
}

bool isDrivingConfigValid(const DrivingConfig& config) {
    return config.behavior.comfortableDeceleration > 0.0 &&
           config.behavior.stopHoldTime >= 1.0 &&
           config.behavior.followTimeGap > 0.0 &&
           config.behavior.minimumFollowDistance > 0.0 &&
           config.behavior.signalStopSearchDistance > 0.0 &&
           config.behavior.emergencyTTC > 0.0 &&
           config.behavior.collisionPredictionStep > 0.0 &&
           config.behavior.collisionPredictionHorizon >=
               config.behavior.collisionPredictionStep &&
           config.laneChange.obstacleTriggerMinimumDistance > 0.0 &&
           config.laneChange.obstacleTriggerMaximumDistance >
               config.laneChange.obstacleTriggerMinimumDistance &&
           config.laneChange.transitionLength > 0.0 &&
           config.laneChange.minimumTargetTTC > 0.0 &&
           config.laneChange.returnWaitingSpeed > 0.0 &&
           config.laneChange.returnWaitingSpeed <=
               config.laneChange.maximumLaneChangeSpeed &&
           config.controller.maxErrorDerivative > 0.0 &&
           config.controller.maxAccelerationRate > 0.0 &&
           config.runtime.nominalDt > 0.0 &&
           config.runtime.speedFilterAlpha > 0.0 &&
           config.runtime.speedFilterAlpha <= 1.0 &&
           config.runtime.maxValidPositionSpeed > 0.0 &&
           config.runtime.inputTimeout > 0.0;
}

// YAML의 모든 튜닝값을 실제 플래너/제어기 설정으로 로드한다.
bool loadDrivingConfig(DrivingConfig& config,
                       const std::string& yamlPath) {
    std::map<std::string, double> values;
    if (!readYamlNumbers(yamlPath, values)) return false;

    return
        setConfigValue(values, "pure_pursuit.wheelbase",
                       config.planner.wheelbase) &&
        setConfigValue(values, "pure_pursuit.straight_lookahead",
                       config.planner.straightLookahead) &&
        setConfigValue(values, "pure_pursuit.curve_lookahead",
                       config.planner.curveLookahead) &&
        setConfigValue(values, "pure_pursuit.curve_threshold",
                       config.planner.curveThreshold) &&
        setConfigValue(values, "pure_pursuit.curvature_check_distance",
                       config.planner.curvatureCheckDistance) &&
        setConfigValue(values, "pure_pursuit.goal_tolerance",
                       config.planner.goalTolerance) &&
        setSpeedConfigValue(values, "speed.straight_target_kph",
                       config.planner.straightSpeed) &&
        setSpeedConfigValue(values, "speed.curve_target_kph",
                       config.planner.curveSpeed) &&
        setSpeedConfigValue(values, "speed.school_zone_target_kph",
                       config.planner.schoolZoneSpeed) &&
        setConfigValue(values, "planning.comfortable_deceleration",
                       config.behavior.comfortableDeceleration) &&
        setConfigValue(values, "planning.stop_safety_margin",
                       config.behavior.stopSafetyMargin) &&
        setConfigValue(values, "planning.stop_hold_time",
                       config.behavior.stopHoldTime) &&
        setSpeedConfigValue(values, "planning.stopped_speed_threshold_kph",
                       config.behavior.stoppedSpeedThreshold) &&
        setConfigValue(values, "planning.follow_time_gap",
                       config.behavior.followTimeGap) &&
        setConfigValue(values, "planning.minimum_follow_distance",
                       config.behavior.minimumFollowDistance) &&
        setConfigValue(values, "planning.emergency_ttc",
                       config.behavior.emergencyTTC) &&
        setConfigValue(values, "planning.collision_prediction_horizon",
                       config.behavior.collisionPredictionHorizon) &&
        setConfigValue(values, "planning.collision_prediction_step",
                       config.behavior.collisionPredictionStep) &&
        setConfigValue(values, "planning.local_path_horizon",
                       config.behavior.localPathHorizon) &&
        setConfigValue(values, "planning.signal_stop_search_distance",
                       config.behavior.signalStopSearchDistance) &&
        setConfigValue(values, "lane_change.obstacle_trigger_minimum_distance",
                       config.laneChange.obstacleTriggerMinimumDistance) &&
        setConfigValue(values, "lane_change.obstacle_trigger_maximum_distance",
                       config.laneChange.obstacleTriggerMaximumDistance) &&
        setConfigValue(values, "lane_change.transition_length",
                       config.laneChange.transitionLength) &&
        setConfigValue(values, "lane_change.obstacle_clearance",
                       config.laneChange.obstacleClearance) &&
        setConfigValue(values, "lane_change.prepare_time",
                       config.laneChange.prepareTime) &&
        setConfigValue(values, "lane_change.settle_time",
                       config.laneChange.settleTime) &&
        setConfigValue(values, "lane_change.minimum_target_ahead_gap",
                       config.laneChange.minimumTargetAheadGap) &&
        setConfigValue(values, "lane_change.minimum_target_behind_gap",
                       config.laneChange.minimumTargetBehindGap) &&
        setConfigValue(values, "lane_change.minimum_target_ttc",
                       config.laneChange.minimumTargetTTC) &&
        setSpeedConfigValue(values, "lane_change.maximum_lane_change_speed_kph",
                       config.laneChange.maximumLaneChangeSpeed) &&
        setSpeedConfigValue(values, "lane_change.return_waiting_speed_kph",
                       config.laneChange.returnWaitingSpeed) &&
        setConfigValue(values, "speed_pd.kp",
                       config.controller.kp) &&
        setConfigValue(values, "speed_pd.kd",
                       config.controller.kd) &&
        setConfigValue(values, "speed_pd.max_error_derivative",
                       config.controller.maxErrorDerivative) &&
        setConfigValue(values, "limits.min_acceleration",
                       config.controller.minAcceleration) &&
        setConfigValue(values, "limits.max_acceleration",
                       config.controller.maxAcceleration) &&
        setConfigValue(values, "limits.max_acceleration_rate",
                       config.controller.maxAccelerationRate) &&
        setConfigValue(values, "limits.max_steering",
                       config.controller.maxSteering) &&
        setConfigValue(values, "runtime.nominal_dt",
                       config.runtime.nominalDt) &&
        setConfigValue(values, "runtime.speed_filter_alpha",
                       config.runtime.speedFilterAlpha) &&
        setSpeedConfigValue(values, "runtime.max_valid_position_speed_kph",
                       config.runtime.maxValidPositionSpeed) &&
        setConfigValue(values, "runtime.input_timeout",
                       config.runtime.inputTimeout) &&
        isDrivingConfigValid(config);
}
