#pragma once

#include "data/gt/gt_structs.hpp"

#include <cstddef>

void printEgoState(const EgoState& ego);
void printObjectState(const ObjectState& object, std::size_t index);
void printObjectStates(const GtFrame& gt);
void printGtFrame(const GtFrame& gt);
void printWorldObjectState(const WorldObjectState& object);
void printWorldObjectStates(const WorldState& world);
