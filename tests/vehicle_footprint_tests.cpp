#include "utils/geometry.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

void close(double actual, double expected, const char* message) {
    require(std::abs(actual - expected) < 1e-9, message);
}

void sameBox(const Obb2D& actual, const Obb2D& expected) {
    for (std::size_t i = 0; i < actual.corners.size(); ++i) {
        close(actual.corners[i].x, expected.corners[i].x, "footprint corner x");
        close(actual.corners[i].y, expected.corners[i].y, "footprint corner y");
    }
}
} // namespace

int main() {
    try {
        // Refactoring must preserve every box for the existing vehicle,
        // including rear-axle offset, rotations and both configured margins.
        for (const double heading : {0.0, 1.2, -2.3}) {
            const Pose2D reference{900., -400., heading};
            const auto body = makeObb2D(
                reference.x + 1.3725 * std::cos(heading),
                reference.y + 1.3725 * std::sin(heading),
                heading, 4.855, 1.88);
            const auto footprint = vehicleFootprintFromObb(body, reference);
            close(footprint.length, 4.855, "nominal length");
            close(footprint.width, 1.88, "nominal width");
            close(footprint.frontExtent(), 3.8, "nominal front extent");
            close(footprint.rearExtent(), 1.055, "nominal rear extent");
            sameBox(vehicleFootprintAt(footprint, reference), body);
            for (const double targetYaw : {0.7, -1.8}) {
                const Pose2D target{150., 80., targetYaw};
                for (const double margin : {0., 0.05, 0.30}) {
                    const auto expected = makeObb2D(
                        target.x + 1.3725 * std::cos(targetYaw),
                        target.y + 1.3725 * std::sin(targetYaw), targetYaw,
                        4.855 + 2. * margin, 1.88 + 2. * margin);
                    sameBox(vehicleFootprintAt(footprint, target, margin), expected);
                }
            }
        }

        const Pose2D origin{};
        const auto normal = vehicleFootprintFromObb(
            makeObb2D(1.3725, 0., 0., 4.855, 1.88), origin);
        const auto larger = vehicleFootprintFromObb(
            makeObb2D(1.8, 0., 0., 7.2, 2.8), origin);
        close(larger.frontExtent(), 5.4, "configured front extent ignored");
        close(larger.rearExtent(), 1.8, "configured rear extent ignored");
        const auto beside = makeObb2D(2., 1.15, 0., 0.4, 0.2);
        require(!intersects(vehicleFootprintAt(normal, origin), beside),
                "normal vehicle unexpectedly clips side object");
        require(intersects(vehicleFootprintAt(larger, origin), beside),
                "wider vehicle footprint ignored");
        const auto ahead = makeObb2D(4.6, 0., 0., 0.4, 0.2);
        require(!intersects(vehicleFootprintAt(normal, origin), ahead),
                "normal front extent changed");
        require(intersects(vehicleFootprintAt(larger, origin), ahead),
                "longer front overhang ignored");
        const auto behind = makeObb2D(-1.5, 0., 0., 0.4, 0.2);
        require(!intersects(vehicleFootprintAt(normal, origin), behind),
                "normal rear extent changed");
        require(intersects(vehicleFootprintAt(larger, origin), behind),
                "longer rear overhang ignored");
        require(intersects(vehicleFootprintAt(normal, origin, 0.30), beside),
                "configured obstacle clearance lost");
        std::cout << "vehicle footprint nominal/configuration regressions PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
