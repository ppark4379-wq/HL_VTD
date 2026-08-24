#include "network/receive/gt_printer.hpp"

#include <iostream>

void printEgoState(const EgoState& ego) {
    std::cout << "Ego x=" << ego.x
              << " y=" << ego.y
              << " heading=" << ego.heading
              << " pitch=" << ego.pitch
              << " roll=" << ego.roll
              << '\n';
}

void printObjectState(const ObjectState& object, std::size_t index) {
    std::cout << "Object[" << index << "]"
              << " id=" << object.id
              << " x=" << object.x
              << " y=" << object.y
              << " z=" << object.z
              << " heading=" << object.heading
              << " speed=" << object.speed
              << " length=" << object.length
              << " width=" << object.width
              << " height=" << object.height
              << '\n';
}

void printObjectStates(const GtFrame& gt) {
    for (std::size_t i = 0; i < gt.objects.size(); ++i) {
        const ObjectState& object = gt.objects[i];
        if (object.id != 0) {
            printObjectState(object, i);
        }
    }
}

void printGtFrame(const GtFrame& gt) {
    printEgoState(gt.ego);
    printObjectStates(gt);
}

void printWorldObjectState(const WorldObjectState& object) {
    std::cout << "[object] id=" << object.raw.id
              << " x=" << object.raw.x
              << " y=" << object.raw.y
              << " laneValid=" << object.laneValid
              << " roadId=" << object.roadId
              << " laneId=" << object.laneId
              << " pathValid=" << object.pathValid
              << " pathS=" << object.pathS
              << " pathD=" << object.pathD
              << '\n';
}

void printWorldObjectStates(const WorldState& world) {
    for (const WorldObjectState& object : world.objects) {
        printWorldObjectState(object);
    }
}
