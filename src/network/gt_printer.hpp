#pragma once

#include "data/gt_structs.hpp"

#include <cstddef>

namespace hlvtd::network {

void printEgoState(const EgoState& ego);
void printObjectState(const ObjectState& object, std::size_t index);
void printObjectStates(const GtFrame& gt);
void printGtFrame(const GtFrame& gt);

}  // namespace hlvtd::network
