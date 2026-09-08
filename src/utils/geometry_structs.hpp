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

// A vehicle body expressed relative to its longitudinal reference point
// (the Ego rear axle). The source OBB is built from vehicle.yaml by World.
struct VehicleFootprint2D {
    double length{};
    double width{};
    double centerOffset{};

    double frontExtent() const { return centerOffset + 0.5 * length; }
    double rearExtent() const { return 0.5 * length - centerOffset; }
};
