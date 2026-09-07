#include "data/map/precomputed_lane_network.hpp"
#include "utils/csv_utils.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace {
std::int64_t integer(const std::string& text) {
    std::size_t used{};
    const auto value = std::stoll(text, &used);
    if (used != text.size()) throw std::runtime_error("invalid integer: " + text);
    return value;
}
double real(const std::string& text) {
    double value{};
    if (!parseDouble(text, value) || !std::isfinite(value))
        throw std::runtime_error("invalid finite number: " + text);
    return value;
}
bool boolean(const std::string& text) {
    if (text != "0" && text != "1") throw std::runtime_error("invalid boolean");
    return text == "1";
}
bool sameLane(const LaneKey& a, const LaneKey& b) {
    return a.roadId == b.roadId && a.sectionIndex == b.sectionIndex && a.laneId == b.laneId;
}
LaneKey keyOf(const PrecomputedLanePoint& p) {
    return {p.pose.roadId, p.pose.sectionIndex, p.pose.laneId};
}
std::ifstream openCsv(const std::filesystem::path& file, const std::string& header) {
    std::ifstream stream(file);
    std::string line;
    if (!stream || !std::getline(stream, line) || trim(line) != header)
        throw std::runtime_error("missing CSV or incompatible header: " + file.string());
    return stream;
}
}

bool loadPrecomputedLaneNetwork(const std::filesystem::path& directory,
                                PrecomputedLaneNetwork& out, std::string* error) {
    try {
        PrecomputedLaneNetwork result;
        auto points = openCsv(directory / "lane_points.csv",
            "id,road_id,section_index,lane_id,point_index,road_s,lane_s,x,y,heading,curvature,width,next_id,left_id,right_id,left_change_allowed,right_change_allowed,usable");
        std::string line;
        while (std::getline(points, line)) {
            const auto f = splitCsvLine(line);
            if (f.size() != 18) throw std::runtime_error("point CSV field count");
            PrecomputedLanePoint p;
            p.id = integer(f[0]);
            if (p.id != static_cast<std::int64_t>(result.points.size()))
                throw std::runtime_error("point IDs must be contiguous");
            p.pose.roadId = static_cast<int>(integer(f[1]));
            p.pose.sectionIndex = static_cast<int>(integer(f[2]));
            p.pose.laneId = static_cast<int>(integer(f[3]));
            const auto index = integer(f[4]);
            if (p.pose.sectionIndex < 0 || p.pose.laneId == 0 || index < 0)
                throw std::runtime_error("invalid lane key/index");
            p.pointIndex = static_cast<std::size_t>(index);
            p.pose.roadS = real(f[5]); p.pose.s = real(f[6]);
            p.pose.x = real(f[7]); p.pose.y = real(f[8]); p.pose.yaw = real(f[9]);
            p.curvature = real(f[10]); p.width = real(f[11]);
            p.nextPoint = integer(f[12]); p.leftPoint = integer(f[13]); p.rightPoint = integer(f[14]);
            p.leftChangeAllowed = boolean(f[15]); p.rightChangeAllowed = boolean(f[16]);
            p.usable = boolean(f[17]);
            const auto key = keyOf(p);
            auto& lane = result.lanes[key];
            lane.key = key;
            if (p.pointIndex != lane.pointIds.size()) throw std::runtime_error("lane point order");
            if (!lane.pointIds.empty()) {
                const auto& previous = result.points.at(static_cast<std::size_t>(lane.pointIds.back()));
                const double direction = key.laneId < 0 ? 1.0 : -1.0;
                if (p.pose.s < previous.pose.s || direction*(p.pose.roadS-previous.pose.roadS) < -1e-8)
                    throw std::runtime_error("lane point travel direction");
            }
            lane.pointIds.push_back(p.id);
            result.points.push_back(p);
        }
        if (result.points.empty()) throw std::runtime_error("empty point network");
        const auto validRef = [&](std::int64_t id) {
            return id >= -1 && id < static_cast<std::int64_t>(result.points.size());
        };
        for (const auto& p : result.points) {
            if (!validRef(p.nextPoint) || !validRef(p.leftPoint) || !validRef(p.rightPoint))
                throw std::runtime_error("dangling point reference");
            if (p.nextPoint >= 0) {
                const auto& next = result.points[static_cast<std::size_t>(p.nextPoint)];
                if (!sameLane(keyOf(p), keyOf(next)) || next.pointIndex != p.pointIndex + 1)
                    throw std::runtime_error("next point must stay in same lane and advance");
            }
            for (const auto ref : {p.leftPoint, p.rightPoint}) {
                if (ref < 0) continue;
                const auto& adjacent = result.points[static_cast<std::size_t>(ref)];
                if (p.pose.roadId != adjacent.pose.roadId || p.pose.sectionIndex != adjacent.pose.sectionIndex ||
                    std::abs(p.pose.laneId-adjacent.pose.laneId) != 1 ||
                    (p.pose.laneId > 0) != (adjacent.pose.laneId > 0))
                    throw std::runtime_error("invalid adjacent lane reference");
            }
            if ((p.leftChangeAllowed && p.leftPoint < 0) || (p.rightChangeAllowed && p.rightPoint < 0))
                throw std::runtime_error("lane change permission without adjacent point");
        }
        auto connections = openCsv(directory / "lane_connections.csv",
            "id,from_road,from_section,from_lane,to_road,to_section,to_lane,from_point,to_point,provenance,contact_point,junction_id,connection_id,gap_m,heading_delta_rad,curvature_delta,status");
        while (std::getline(connections, line)) {
            const auto f = splitCsvLine(line);
            if (f.size() != 17 || integer(f[0]) != static_cast<std::int64_t>(result.connections.size()))
                throw std::runtime_error("connection CSV field count/order");
            PrecomputedLaneConnection c;
            c.from = {static_cast<int>(integer(f[1])), static_cast<int>(integer(f[2])), static_cast<int>(integer(f[3]))};
            c.to = {static_cast<int>(integer(f[4])), static_cast<int>(integer(f[5])), static_cast<int>(integer(f[6]))};
            c.fromPoint = integer(f[7]); c.toPoint = integer(f[8]);
            c.provenance = f[9]; c.contactPoint = f[10]; c.junctionId = static_cast<int>(integer(f[11]));
            c.connectionId = f[12]; c.gap = real(f[13]); c.headingDelta = real(f[14]);
            c.curvatureDelta = real(f[15]); c.status = f[16];
            if (!validRef(c.fromPoint) || c.fromPoint < 0 || !validRef(c.toPoint))
                throw std::runtime_error("dangling connection point");
            const auto from = result.lanes.find(c.from);
            const auto to = result.lanes.find(c.to);
            if (from == result.lanes.end() || from->second.pointIds.back() != c.fromPoint ||
                (c.toPoint >= 0 && (to == result.lanes.end() || to->second.pointIds.front() != c.toPoint)))
                throw std::runtime_error("connection must join lane endpoints");
            if (c.status == "continuous" && (c.toPoint < 0 || c.gap > .05000001 ||
                c.headingDelta > .087266464 || c.curvatureDelta > .05000001 ||
                ((c.contactPoint == "start") != (c.to.laneId < 0))))
                throw std::runtime_error("invalid continuous connection");
            from->second.outgoingConnections.push_back(result.connections.size());
            result.connections.push_back(std::move(c));
        }
        // Version 1 exports have no transition files. Version 2 requires all
        // three tables; never silently load a partial maneuver network.
        if (std::filesystem::exists(directory / "lane_transitions.csv") ||
            std::filesystem::exists(directory / "transition_points.csv") ||
            std::filesystem::exists(directory / "replaced_points.csv")) {
            auto transitions = openCsv(directory / "lane_transitions.csv",
                "id,kind,event_road,event_section,event_lane,from_point,to_point,point_count,min_clearance_m,max_curvature,provenance");
            std::vector<std::size_t> expectedCounts;
            while (std::getline(transitions, line)) {
                const auto f = splitCsvLine(line);
                if (f.size() != 11 || integer(f[0]) != static_cast<std::int64_t>(result.transitions.size()))
                    throw std::runtime_error("transition field count/order");
                PrecomputedLaneTransition t;
                t.kind = f[1];
                t.eventLane = {static_cast<int>(integer(f[2])), static_cast<int>(integer(f[3])), static_cast<int>(integer(f[4]))};
                t.fromPoint = integer(f[5]); t.toPoint = integer(f[6]);
                const auto count = integer(f[7]);
                t.minClearance = real(f[8]); t.maxCurvature = real(f[9]); t.provenance = f[10];
                if ((t.kind != "merge" && t.kind != "split" && t.kind != "continuation") || count < 2 ||
                    !validRef(t.fromPoint) || !validRef(t.toPoint) || t.fromPoint < 0 || t.toPoint < 0)
                    throw std::runtime_error("invalid transition anchors/kind/count");
                const auto& a = result.points[static_cast<std::size_t>(t.fromPoint)];
                const auto& b = result.points[static_cast<std::size_t>(t.toPoint)];
                if (a.pose.roadId != b.pose.roadId || a.pose.roadId != t.eventLane.roadId ||
                    (a.pose.laneId > 0) != (b.pose.laneId > 0))
                    throw std::runtime_error("transition road/direction mismatch");
                result.points[static_cast<std::size_t>(t.fromPoint)].outgoingTransitions.push_back(result.transitions.size());
                expectedCounts.push_back(static_cast<std::size_t>(count));
                result.transitions.push_back(std::move(t));
            }
            auto samples = openCsv(directory / "transition_points.csv",
                "transition_id,point_index,road_s,path_s,x,y,heading,curvature");
            while (std::getline(samples, line)) {
                const auto f = splitCsvLine(line);
                if (f.size() != 8) throw std::runtime_error("transition point field count");
                const auto tid = integer(f[0]);
                if (tid < 0 || tid >= static_cast<std::int64_t>(result.transitions.size()))
                    throw std::runtime_error("dangling transition reference");
                auto& t = result.transitions[static_cast<std::size_t>(tid)];
                if (integer(f[1]) != static_cast<std::int64_t>(t.points.size()))
                    throw std::runtime_error("transition point order");
                t.points.push_back({real(f[2]), real(f[3]), real(f[4]), real(f[5]), real(f[6]), real(f[7])});
            }
            for (std::size_t i = 0; i < result.transitions.size(); ++i) {
                const auto& t = result.transitions[i];
                if (t.points.size() != expectedCounts[i]) throw std::runtime_error("transition point count mismatch");
                const auto matches = [&](const PrecomputedTransitionPoint& p, std::int64_t id) {
                    const auto& q = result.points[static_cast<std::size_t>(id)];
                    return std::hypot(p.x-q.pose.x,p.y-q.pose.y) < 1e-6 &&
                           std::abs(std::remainder(p.yaw-q.pose.yaw, 6.283185307179586)) < 1e-6 &&
                           std::abs(p.curvature-q.curvature) < 1e-6 && std::abs(p.roadS-q.pose.roadS) < 1e-6;
                };
                if (!matches(t.points.front(),t.fromPoint) || !matches(t.points.back(),t.toPoint))
                    throw std::runtime_error("transition endpoint mismatch");
                const double direction = result.points[static_cast<std::size_t>(t.fromPoint)].pose.laneId < 0 ? 1.0 : -1.0;
                for (std::size_t j=1; j<t.points.size(); ++j) {
                    const auto& a=t.points[j-1]; const auto& b=t.points[j];
                    if (b.s<a.s || direction*(b.roadS-a.roadS)<-1e-8 || std::hypot(b.x-a.x,b.y-a.y)>.510001)
                        throw std::runtime_error("transition sample discontinuity/direction");
                }
            }
            auto replaced = openCsv(directory / "replaced_points.csv", "point_id,transition_id");
            while (std::getline(replaced,line)) {
                const auto f=splitCsvLine(line);
                if (f.size()!=2) throw std::runtime_error("replaced point field count");
                const auto pid=integer(f[0]), tid=integer(f[1]);
                if (!validRef(pid) || pid<0 || tid<0 || tid>=static_cast<std::int64_t>(result.transitions.size()))
                    throw std::runtime_error("dangling replaced point");
                result.points[static_cast<std::size_t>(pid)].replacedByTransition=tid;
            }
            for (const auto& t:result.transitions) {
                if (result.points[static_cast<std::size_t>(t.fromPoint)].replacedByTransition>=0 ||
                    result.points[static_cast<std::size_t>(t.toPoint)].replacedByTransition>=0)
                    throw std::runtime_error("transition anchor hidden by another transition");
            }
        }
        if (std::filesystem::exists(directory / "transition_checks.csv")) {
            auto checks=openCsv(directory / "transition_checks.csv",
                "transition_id,crosses_solid,violates_lane_change_permission,requires_planner_validation,conflicts,vehicle_footprint_valid,geometry_only_reasons");
            std::size_t count=0;
            while (std::getline(checks,line)) {
                const auto f=splitCsvLine(line);
                if (f.size()!=7 || integer(f[0])!=static_cast<std::int64_t>(count) || count>=result.transitions.size())
                    throw std::runtime_error("transition checks reference/order");
                const auto solid=integer(f[1]),permission=integer(f[2]),required=integer(f[3]),footprint=integer(f[5]);
                if ((solid!=0 && solid!=1) || (permission!=0 && permission!=1) || required!=1 || (footprint!=0 && footprint!=1))
                    throw std::runtime_error("invalid transition planning flags");
                auto& t=result.transitions[count++];
                t.crossesSolid=solid;t.violatesLaneChangePermission=permission;t.requiresPlannerValidation=required;
                t.roadMarkConflicts=f[4];
                t.vehicleFootprintValid=footprint;t.geometryOnlyReasons=f[6];
            }
            if (count!=result.transitions.size()) throw std::runtime_error("incomplete transition checks");
        }
        if (std::filesystem::exists(directory / "lane_access.csv")) {
            auto access = openCsv(directory / "lane_access.csv",
                                 "road_id,section_index,lane_id,has_entry,has_exit,status");
            std::size_t count=0;
            while (std::getline(access,line)) {
                const auto f=splitCsvLine(line);
                if (f.size()!=6) throw std::runtime_error("lane access field count");
                const LaneKey key{static_cast<int>(integer(f[0])),static_cast<int>(integer(f[1])),static_cast<int>(integer(f[2]))};
                auto lane=result.lanes.find(key);
                if (lane==result.lanes.end() || lane->second.topologyStatus!="not_audited")
                    throw std::runtime_error("dangling/duplicate lane access reference");
                const auto entry=integer(f[3]),exit=integer(f[4]);
                if ((entry!=0 && entry!=1) || (exit!=0 && exit!=1))
                    throw std::runtime_error("invalid lane access flag");
                const std::string expected=entry ? (exit ? "connected":"no_exit") : (exit ? "no_entry":"no_entry_or_exit");
                if (f[5]!=expected && f[5]!="excluded_by_review") throw std::runtime_error("lane access status mismatch");
                lane->second.hasTopologicalEntry=entry;
                lane->second.hasTopologicalExit=exit;
                lane->second.topologyStatus=f[5];
                ++count;
            }
            if (count!=result.lanes.size()) throw std::runtime_error("incomplete lane access audit");
        }
        out = std::move(result);
        if (error) error->clear();
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}
