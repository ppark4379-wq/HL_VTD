#pragma once

#include "utils/geometry_structs.hpp"

#include <vector>

Obb2D makeObb2D(double centerX, double centerY, double heading,
                double length, double width);
VehicleFootprint2D vehicleFootprintFromObb(const Obb2D& body,
                                         const Pose2D& reference);
Obb2D vehicleFootprintAt(const VehicleFootprint2D& footprint,
                        const Pose2D& reference, double margin = 0.0);
bool intersects(const Obb2D& first, const Obb2D& second);
bool intersects(const std::vector<Point2D>& polygon, const Obb2D& obb);
bool containsPoint(const std::vector<Point2D>& polygon,
                   const Point2D& point);
