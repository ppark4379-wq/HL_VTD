#include "control/config/control_config.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

struct TemporaryConfig {
    std::filesystem::path directory;
    TemporaryConfig() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        for (unsigned attempt = 0; attempt < 100; ++attempt) {
            auto candidate = std::filesystem::temp_directory_path() /
                ("hl_vtd_local_fallback_config_" + std::to_string(stamp) +
                 "_" + std::to_string(attempt));
            if (std::filesystem::create_directory(candidate)) {
                directory = std::move(candidate);
                return;
            }
        }
        throw std::runtime_error("cannot create isolated config fixture directory");
    }
    ~TemporaryConfig() {
        if (!directory.empty()) {
            std::error_code ignored;
            std::filesystem::remove(directory / "control.yaml", ignored);
            std::filesystem::remove(directory, ignored);
        }
    }
    bool load(const std::string& text, DrivingConfig& config) const {
        const auto path = directory / "control.yaml";
        {
            std::ofstream output(path);
            output << text;
            require(output.good(), "cannot write isolated config fixture");
        }
        return loadDrivingConfig(config, path.string());
    }
};

std::string change(const std::string& source, const std::string& key,
                   const std::string& value) {
    const std::string needle = "  " + key + ":";
    const auto start = source.find(needle);
    require(start != std::string::npos, "fixture config key not found");
    const auto end = source.find('\n', start);
    auto text = source;
    text.replace(start, end - start, needle + " " + value);
    return text;
}

std::string withoutFallbackKeys(const std::string& source) {
    auto text = source;
    for (std::size_t start; (start = text.find("  local_fallback_")) != std::string::npos;) {
        const auto end = text.find('\n', start);
        text.erase(start, end == std::string::npos ? end : end + 1 - start);
    }
    return text;
}

void requireDefaults(const DrivingConfig& config) {
    const auto& lane = config.laneChange;
    require(lane.localFallbackActivationDistance == 35.0 &&
            lane.localFallbackTimeHeadway == 5.0 &&
            lane.localFallbackHorizon == 60.0 &&
            lane.localFallbackRetryInterval == 0.5 &&
            lane.localFallbackSearchBudgetMs == 80.0 &&
            std::abs(lane.localFallbackSpeed - 5.0 / 3.6) < 1e-12,
            "local fallback defaults or speed units disagree");
}
}  // namespace

int main() {
    try {
        std::ifstream input("config/control.yaml");
        require(input.good(), "run from the repository root");
        const std::string source((std::istreambuf_iterator<char>(input)), {});
        TemporaryConfig file;

        DrivingConfig current;
        require(file.load(source, current), "repository config rejected");
        requireDefaults(current);
        require(!LaneChangeData{}.localCorridorAvoidance,
                "ordinary maneuvers defaulted to local fallback mode");

        const auto legacy = withoutFallbackKeys(source);
        DrivingConfig oldConfig;
        require(file.load(legacy, oldConfig), "legacy config missing optional keys rejected");
        requireDefaults(oldConfig);
        auto partial = legacy;
        const auto planning = partial.find("planning:\n");
        require(planning != std::string::npos, "planning config section missing");
        partial.insert(planning + std::string("planning:\n").size(),
                       "  local_fallback_speed_kph: 3.6\n");
        DrivingConfig partialConfig;
        require(file.load(partial, partialConfig) &&
                std::abs(partialConfig.laneChange.localFallbackSpeed - 1.0) < 1e-12 &&
                partialConfig.laneChange.localFallbackActivationDistance == 35.0,
                "partial optional speed override changed units or other defaults");

        for (const std::string key : {
                 "local_fallback_activation_distance", "local_fallback_time_headway",
                 "local_fallback_horizon", "local_fallback_retry_interval",
                 "local_fallback_search_budget_ms", "local_fallback_speed_kph"}) {
            for (const std::string invalid : {"0", "-1", "nan", "inf", "-inf"}) {
                DrivingConfig config;
                require(!file.load(change(source, key, invalid), config),
                        "non-positive or non-finite fallback config accepted");
            }
        }
        DrivingConfig invalidHorizon, invalidBudget;
        require(!file.load(change(source, "local_fallback_horizon", "34"), invalidHorizon),
                "activation outside search horizon accepted");
        require(!file.load(change(source, "local_fallback_search_budget_ms", "501"), invalidBudget),
                "worker budget larger than retry period accepted");
        DrivingConfig boundary;
        auto valid = change(source, "local_fallback_activation_distance", "60");
        valid = change(valid, "local_fallback_retry_interval", "0.08");
        require(file.load(valid, boundary), "valid equal-boundary config rejected");

        std::cout << "local fallback configuration tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
