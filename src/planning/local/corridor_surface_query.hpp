#pragma once

#include "planning/local/corridor_lattice.hpp"
#include <cmath>
#include <map>
#include <vector>

namespace corridor_detail {

// Scratch cache for ONE immutable surface sweep/search. Never retain it across
// mission activation, permission changes or frames. The candidate order and
// exact containment/tie-breaking predicates match containing() in the planner.
class SurfaceQuery {
public:
    SurfaceQuery(const CorridorRuntime& runtime, bool corridorOnly = true,
                 const std::vector<bool>* surfaceOverride = nullptr)
        : runtime_(runtime), corridorOnly_(corridorOnly), surface_(surfaceOverride) {}
    SurfaceQuery(const SurfaceQuery&) = delete;
    SurfaceQuery& operator=(const SurfaceQuery&) = delete;

    std::int64_t containing(double x, double y, double c, double s) {
        const std::pair<int,int> key{int(std::floor(x/10.)), int(std::floor(y/10.))};
        // Most consecutive footprint probes share a grid cell.
        if (!last_ || key != lastKey_) {
            auto [entry, inserted] = cells_.try_emplace(key);
            if (inserted) {
                for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) {
                    const auto cell=runtime_.spatial.find({key.first+dx,key.second+dy});
                    if (cell==runtime_.spatial.end()) continue;
                    for (const auto id:cell->second) {
                        const auto& p=runtime_.network.points[id];
                        if (corridorOnly_) {
                            if (!(surface_ ? (*surface_)[id] : runtime_.surfaceAllowed[id])) continue;
                        } else {
                            const auto lane=runtime_.network.lanes.find(
                                {p.pose.roadId,p.pose.sectionIndex,p.pose.laneId});
                            if (p.replacedByTransition>=0 || lane==runtime_.network.lanes.end() ||
                                lane->second.topologyStatus=="excluded_by_review") continue;
                        }
                        const double radius=p.width/2.+.6;
                        const auto [pc,ps]=runtime_.tangent[id];
                        entry->second.push_back({id,p.pose.x,p.pose.y,radius*radius,
                                                 pc,ps,p.width/2.+.03});
                    }
                }
            }
            lastKey_=key;last_=&entry->second;
        }
        std::int64_t result=-1;
        double best=std::numeric_limits<double>::infinity();
        for (const auto& p:*last_) {
            const double dx=x-p.x,dy=y-p.y;
            if (dx*dx+dy*dy>p.radiusSquared) continue;
            if (c*p.c+s*p.s<.36235775) continue;
            const double a=dx*p.c+dy*p.s;
            const double b=-dx*p.s+dy*p.c;
            if (std::abs(a)>.55 || std::abs(b)>p.halfWidth) continue;
            const double score=std::abs(b)+.1*std::abs(a);
            if (score<best) {best=score;result=p.id;}
        }
        return result;
    }

private:
    struct Candidate { std::int64_t id; double x,y,radiusSquared,c,s,halfWidth; };
    const CorridorRuntime& runtime_;
    bool corridorOnly_;
    const std::vector<bool>* surface_;
    std::map<std::pair<int,int>,std::vector<Candidate>> cells_;
    std::pair<int,int> lastKey_{};
    const std::vector<Candidate>* last_{};
};

} // namespace corridor_detail
