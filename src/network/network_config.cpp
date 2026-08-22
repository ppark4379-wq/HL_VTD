#include "network/network_config.hpp"

#include <charconv>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include <unordered_set>

namespace hlvtd::network {
namespace {

std::string_view trim(std::string_view value) {
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
        value.remove_prefix(1);
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
        value.remove_suffix(1);
    }
    return value;
}

std::string_view removeComment(std::string_view line) {
    char quote = 0;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        const bool is_quote = ch == 39 || ch == 34;
        if (is_quote && (i == 0 || line[i - 1] != 92)) {
            quote = quote == 0 ? ch : (quote == ch ? 0 : quote);
        } else if (ch == 35 && quote == 0) {
            return line.substr(0, i);
        }
    }
    return line;
}

[[noreturn]] void fail(std::size_t line, const std::string& detail) {
    throw std::runtime_error("network.yaml line " + std::to_string(line) + ": " + detail);
}

std::string parseString(std::string_view value, std::size_t line_number) {
    value = trim(value);
    if (value.empty()) {
        fail(line_number, "value must not be empty");
    }
    if (value.front() == 39 || value.front() == 34) {
        if (value.size() < 2 || value.back() != value.front()) {
            fail(line_number, "unterminated quoted value");
        }
        value.remove_prefix(1);
        value.remove_suffix(1);
    }
    if (value.empty()) {
        fail(line_number, "value must not be empty");
    }
    return std::string(value);
}

std::uint16_t parsePort(std::string_view value, std::size_t line_number) {
    value = trim(value);
    unsigned int parsed = 0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
        parsed == 0 || parsed > 65535) {
        fail(line_number, "port must be an integer between 1 and 65535");
    }
    return static_cast<std::uint16_t>(parsed);
}

void markOnce(std::unordered_set<std::string>& fields,
              const std::string& full_key,
              std::size_t line_number) {
    if (!fields.insert(full_key).second) {
        fail(line_number, "duplicate key '" + full_key + "'");
    }
}

}  // namespace

NetworkConfig loadNetworkConfig(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open network config: " + path.string());
    }

    NetworkConfig config;
    std::string section;
    std::unordered_set<std::string> fields;
    std::string raw_line;
    std::size_t line_number = 0;

    while (std::getline(input, raw_line)) {
        ++line_number;
        const auto without_comment = removeComment(raw_line);
        const auto line = trim(without_comment);
        if (line.empty() || line == "---" || line == "...") {
            continue;
        }

        const auto indentation = without_comment.find_first_not_of(" \t");
        const bool top_level = indentation == 0;
        const auto colon = line.find(':');
        if (colon == std::string_view::npos) {
            fail(line_number, "expected 'key: value'");
        }

        const auto key = trim(line.substr(0, colon));
        const auto value = trim(line.substr(colon + 1));
        if (key.empty()) {
            fail(line_number, "key must not be empty");
        }

        if (top_level) {
            if (!value.empty() || (key != "vtd" && key != "controller")) {
                fail(line_number, "expected section 'vtd:' or 'controller:'");
            }
            section = std::string(key);
            continue;
        }

        if (section.empty()) {
            fail(line_number, "setting appears before a section");
        }

        const std::string full_key = section + "." + std::string(key);
        markOnce(fields, full_key, line_number);

        if (full_key == "vtd.host_ip") {
            config.vtd.host_ip = parseString(value, line_number);
        } else if (full_key == "vtd.tcp_port") {
            config.vtd.tcp_port = parsePort(value, line_number);
        } else if (full_key == "vtd.rtsp_port") {
            config.vtd.rtsp_port = parsePort(value, line_number);
        } else if (full_key == "controller.ip") {
            config.controller.ip = parseString(value, line_number);
        } else if (full_key == "controller.udp_port") {
            config.controller.udp_port = parsePort(value, line_number);
        } else {
            fail(line_number, "unknown key '" + full_key + "'");
        }
    }

    const char* required[] = {
        "vtd.host_ip", "vtd.tcp_port", "vtd.rtsp_port", "controller.ip", "controller.udp_port"};
    for (const char* key : required) {
        if (fields.find(key) == fields.end()) {
            throw std::runtime_error("network config is missing required key '" +
                                     std::string(key) + "'");
        }
    }

    return config;
}

}  // namespace hlvtd::network
