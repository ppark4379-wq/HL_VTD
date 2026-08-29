#pragma once

#include <array>

struct Point2D {
    double x{};
    double y{};
};

struct Pose2D {
    double x{};
    double y{};
    double heading{};
};

struct Obb2D {
    std::array<Point2D, 4> corners{};
};
