#pragma once

#include <string>
#include <vector>

std::string trim(const std::string& value);
std::vector<std::string> splitCsvLine(const std::string& line);
bool parseDouble(const std::string& text, double& value);
