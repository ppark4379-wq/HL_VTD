#include "control/control_config.hpp"

#include "utils/csv_utils.hpp"

#include <fstream>
#include <map>

using namespace std;

// YAML 주석을 제거한다.
string removeYamlComment(const string& line) {
    const size_t position = line.find('#');
    return trim(position == string::npos ? line : line.substr(0, position));
}

// section/key 형태의 간단한 숫자 YAML을 map으로 읽는다.
bool readYamlNumbers(const string& yamlPath, map<string, double>& values) {
    ifstream input(yamlPath);
    if (!input) return false;

    string section;
    string rawLine;
    while (getline(input, rawLine)) {
        const string line = removeYamlComment(rawLine);
        if (line.empty()) continue;

        const size_t colon = line.find(':');
        if (colon == string::npos) return false;
        const string key = trim(line.substr(0, colon));
        const string text = trim(line.substr(colon + 1));

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
bool setConfigValue(const map<string, double>& values,
                    const string& key,
                    double& output) {
    const auto found = values.find(key);
    if (found == values.end()) return false;
    output = found->second;
    return true;
}

// YAML의 모든 튜닝값을 실제 플래너/제어기 설정으로 로드한다.
bool loadDrivingConfig(DrivingConfig& config,
                       const string& yamlPath) {
    map<string, double> values;
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
        setConfigValue(values, "speed.straight_target",
                       config.planner.straightSpeed) &&
        setConfigValue(values, "speed.curve_target",
                       config.planner.curveSpeed) &&
        setConfigValue(values, "speed.school_zone_target",
                       config.planner.schoolZoneSpeed) &&
        setConfigValue(values, "speed_pd.kp",
                       config.controller.kp) &&
        setConfigValue(values, "speed_pd.kd",
                       config.controller.kd) &&
        setConfigValue(values, "limits.min_acceleration",
                       config.controller.minAcceleration) &&
        setConfigValue(values, "limits.max_acceleration",
                       config.controller.maxAcceleration) &&
        setConfigValue(values, "limits.max_steering",
                       config.controller.maxSteering) &&
        setConfigValue(values, "runtime.nominal_dt",
                       config.nominalDt);
}
