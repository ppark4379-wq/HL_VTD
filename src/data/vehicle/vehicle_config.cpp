#include "data/vehicle/vehicle_config.hpp"

#include "utils/csv_utils.hpp"

#include <fstream>
#include <map>

bool loadVehicleGeometry(VehicleGeometry& geometry,
                         const std::string& yamlPath) {
    std::ifstream input(yamlPath);
    if (!input) return false;

    std::map<std::string, double> values;
    std::string section;
    std::string rawLine;
    while (std::getline(input, rawLine)) {
        const std::size_t comment = rawLine.find('#');
        const std::string line = trim(comment == std::string::npos
                                          ? rawLine
                                          : rawLine.substr(0, comment));
        if (line.empty()) continue;
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) return false;
        const std::string key = trim(line.substr(0, colon));
        const std::string text = trim(line.substr(colon + 1));
        if (text.empty()) {
            section = key;
            continue;
        }
        double value{};
        if (!parseDouble(text, value)) return false;
        values[section + "." + key] = value;
    }

    const auto read = [&](const std::string& key, double& output) {
        const auto found = values.find(key);
        if (found == values.end()) return false;
        output = found->second;
        return true;
    };
    if (!read("geometry.wheelbase", geometry.wheelbase) ||
        !read("geometry.width", geometry.width) ||
        !read("geometry.length", geometry.length) ||
        !read("geometry.rear_axle_to_front_bumper",
              geometry.rearAxleToFrontBumper) ||
        !read("geometry.rear_axle_to_rear_bumper",
              geometry.rearAxleToRearBumper)) {
        return false;
    }
    return geometry.wheelbase > 0.0 && geometry.width > 0.0 &&
           geometry.length > 0.0 &&
           geometry.rearAxleToFrontBumper > 0.0 &&
           geometry.rearAxleToRearBumper > 0.0;
}
