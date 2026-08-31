#include "csv_utils.hpp"


// 문자열 앞뒤의 공백과 줄바꿈을 제거한다.
std::string trim(const std::string& value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

// CSV 한 줄을 따옴표를 고려해 쉼표 기준 필드로 분리한다.
std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;

    for (char character : line) {
        if (character == '"') {
            quoted = !quoted;
        } else if (character == ',' && !quoted) {
            fields.push_back(trim(field));
            field.clear();
        } else {
            field.push_back(character);
        }
    }
    fields.push_back(trim(field));
    return fields;
}

// 문자열 전체를 double로 변환하고 성공 여부를 반환한다.
bool parseDouble(const std::string& text, double& value) {
    try {
        std::size_t used = 0;
        value = std::stod(text, &used);
        return used == text.size();
    } catch (...) {
        return false;
    }
}
