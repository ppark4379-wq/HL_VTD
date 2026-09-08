#include "planning/global/reference_path_generator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void close(double actual, double expected) {
    require(std::abs(actual-expected) < 1e-9, "incorrect road-relative elevation");
}
XmlElement elevation(const char* s, const char* a, const char* b,
                     const char* c = "0", const char* d = "0") {
    return {"elevation", {{"s", s}, {"a", a}, {"b", b}, {"c", c}, {"d", d}}, {}};
}
XmlElement road(const char* id, std::vector<XmlElement> profile) {
    XmlElement result{"road", {{"id", id}, {"length", "100"}, {"junction", "-1"}}, {}};
    if (!profile.empty()) {
        XmlElement child{"elevationProfile", {}, {}};
        for (auto& record : profile) child.children.emplace_back("elevation", record);
        result.children.emplace_back("elevationProfile", std::move(child));
    }
    return result;
}
} // namespace

int main() {
    try {
        // Minimal parsed-XODR tree: reverse record order deliberately checks
        // sorting, cubic ds evaluation, and exact segment-boundary selection.
        XmlElement root{"OpenDRIVE", {}, {
            {"road", road("1", {elevation("50", "20", ".4", ".01", ".001"),
                                 elevation("0", "10", ".2")})},
            {"road", road("2", {})},
            {"road", road("3", {elevation("5", "30", "0")})},
            // Coincident x/y roads can be separate floors; their z is not
            // derived from the current Ego or the other road's elevation.
            {"road", road("4", {elevation("0", "40", "0")})},
            {"road", road("5", {elevation("0", "10", "0"),
                                 elevation("20", "bad", "0")})}
        }};
        PathGeneratorData map;
        parseRoads(map, root);
        require(map.roads.size() == 5, "elevation parsing changed road loading");
        require(map.roads.at(1).elevations.size() == 2, "elevation profile not parsed");
        double z = -123.;
        require(sampleRoadElevation(z, map, 1, 0.), "start elevation missing"); close(z, 10.);
        require(sampleRoadElevation(z, map, 1, 40.), "uphill elevation missing"); close(z, 18.);
        require(sampleRoadElevation(z, map, 1, 50.), "profile boundary missing"); close(z, 20.);
        require(sampleRoadElevation(z, map, 1, 75.), "cubic elevation missing"); close(z, 51.875);
        require(sampleRoadElevation(z, map, 4, 40.), "second floor missing"); close(z, 40.);

        z = 123.;
        require(!sampleRoadElevation(z, map, 2, 0.), "missing profile became world z=0");
        require(!sampleRoadElevation(z, map, 3, 4.), "profile extrapolated before first record");
        require(!sampleRoadElevation(z, map, 5, 25.), "corrupt profile silently extrapolated");
        require(!sampleRoadElevation(z, map, 99, 0.), "unknown road accepted");
        require(!sampleRoadElevation(z, map, 1, -1.), "negative road station accepted");
        require(!sampleRoadElevation(z, map, 1, 101.), "past-end road station accepted");
        require(!sampleRoadElevation(z, map, 1, std::numeric_limits<double>::quiet_NaN()),
                "non-finite station accepted");
        close(z, 123.);
        Geometry geometry;geometry.length=100.;
        map.roads.at(1).geometries.push_back(geometry);
        Waypoint point;point.roadId=1;point.x=40.;point.roadS=40.;
        require(roadPointSharesObjectLevel(map,point,40.,0.,18.),
                "same road uphill obstacle was excluded by a different ego elevation");
        require(!roadPointSharesObjectLevel(map,point,40.,0.,40.),
                "separate upper deck became a collision on the lower road");
        point.x=60.;point.roadS=60.;
        require(roadPointSharesObjectLevel(map,point,50.,0.,20.),
                "moving same-road actor compared to future ego elevation");
        point.roadId=2;
        require(roadPointSharesObjectLevel(map,point,40.,0.,40.),
                "unknown elevation silently discarded a received obstacle");
        std::cout << "road elevation parser/query tests PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
