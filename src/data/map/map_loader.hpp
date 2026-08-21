#pragma once

#include "../../planning/planning_structs.hpp"

#include <sstream>
#include <string>

// 기능 1: XODR XML 파일을 읽고 <OpenDRIVE> 지도 루트를 반환한다.
XmlElement loadOpenDrive(const std::string& xodrPath);

// XML 태그의 직계 하위 태그 중 이름이 같은 첫 태그를 찾는다.
const XmlElement* findFirstChild(const XmlElement& parent,
                                 const std::string& childName);

// 기능 2: XML 속성 문자열을 int/double 등 요청한 자료형으로 변환한다.
template <typename T>
T readAttribute(const XmlElement& element,
                const std::string& name,
                const T& fallback) {
    const auto found = element.attributes.find(name);
    if (found == element.attributes.end()) return fallback;

    std::istringstream input(found->second);
    T value{};
    return input >> value ? value : fallback;
}

// 기능 3: string 속성은 형 변환 없이 그대로 반환한다.
template <>
inline std::string readAttribute<std::string>(
    const XmlElement& element,
    const std::string& name,
    const std::string& fallback) {
    const auto found = element.attributes.find(name);
    return found == element.attributes.end() ? fallback : found->second;
}
