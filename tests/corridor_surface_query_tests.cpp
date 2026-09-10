#include "planning/local/corridor_surface_query.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

// Frozen pre-optimization query: include the original traversal and strict
// score comparison so equal-score lane IDs must also remain identical.
std::int64_t reference(const CorridorRuntime& r,double x,double y,double yaw,
                       bool corridorOnly,const std::vector<bool>* overrideMask) {
    const int gx=int(std::floor(x/10.)),gy=int(std::floor(y/10.));
    std::int64_t result=-1;double best=std::numeric_limits<double>::infinity();
    const double c=std::cos(yaw),s=std::sin(yaw);
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
        const auto cell=r.spatial.find({gx+dx,gy+dy});if(cell==r.spatial.end()) continue;
        for(auto id:cell->second) {
            const auto& p=r.network.points[id];
            if(corridorOnly) {
                if(!(overrideMask?(*overrideMask)[id]:r.surfaceAllowed[id])) continue;
            } else {
                const auto lane=r.network.lanes.find({p.pose.roadId,p.pose.sectionIndex,p.pose.laneId});
                if(p.replacedByTransition>=0 || lane==r.network.lanes.end() ||
                   lane->second.topologyStatus=="excluded_by_review") continue;
            }
            const double px=x-p.pose.x,py=y-p.pose.y,radius=p.width/2.+.6;
            if(px*px+py*py>radius*radius) continue;
            const auto [pc,ps]=r.tangent[id];
            if(c*pc+s*ps<.36235775) continue;
            const double a=px*pc+py*ps,b=-px*ps+py*pc;
            if(std::abs(a)>.55 || std::abs(b)>p.width/2.+.03) continue;
            const double score=std::abs(b)+.1*std::abs(a);
            if(score<best) {best=score;result=id;}
        }
    }
    return result;
}

int main() {
    try {
        CorridorRuntime r;
        // Include negative grid coordinates, overlapping IDs, excluded lanes,
        // source knots, varied widths and nonzero map tangents.
        for(int lane=0;lane<6;++lane) {
            const LaneKey key{lane+1,0,-1};
            PrecomputedLane raw;raw.key=key;
            raw.topologyStatus=lane==4?"excluded_by_review":"connected";
            r.network.lanes[key]=raw;
            for(int i=0;i<241;++i) {
                PrecomputedLanePoint p;p.id=r.network.points.size();p.width=3.+.1*lane;
                p.pose.x=-60.+i*.5;p.pose.y=lane==5?0.:lane*3.2;
                p.pose.yaw=lane==2?.1:0.;p.pose.roadId=key.roadId;p.pose.laneId=-1;
                if(i%47==0) p.replacedByTransition=0;
                r.network.points.push_back(p);
                r.tangent.push_back({std::cos(p.pose.yaw),std::sin(p.pose.yaw)});
                r.surfaceAllowed.push_back(lane!=3 && lane!=4);
                r.spatial[{int(std::floor(p.pose.x/10.)),int(std::floor(p.pose.y/10.))}].push_back(p.id);
            }
        }
        std::vector<std::array<double,3>> probes;
        // Sequential footprint-like probes exercise both cache reuse and
        // crossing cell boundaries; random probes exercise less local access.
        for(int i=0;i<800;++i) for(double lateral:{-1.53,0.,1.53})
            probes.push_back({-60.+i*.15,lateral,0.});
        std::mt19937 random(20260910);
        std::uniform_real_distribution<double> x(-75.,75.),y(-5.,20.),yaw(-3.15,3.15);
        for(int i=0;i<18000;++i) probes.push_back({x(random),y(random),yaw(random)});
        for(double px:{-10.,-.55,0.,.55,10.}) for(double py:{-1.53,0.,1.53,3.2})
            for(double heading:{0.,1.2,-1.2,3.141592653589793}) probes.push_back({px,py,heading});
        auto extended=r.surfaceAllowed;
        for(std::size_t i=0;i<extended.size();++i) if(i%3==0) extended[i]=true;
        std::size_t checks=0;
        for(int mode=0;mode<3;++mode) {
            const bool corridorOnly=mode!=2;
            const auto* mask=mode==1?&extended:nullptr;
            corridor_detail::SurfaceQuery query(r,corridorOnly,mask);
            for(const auto& p:probes) {
                const auto expected=reference(r,p[0],p[1],p[2],corridorOnly,mask);
                if(query.containing(p[0],p[1],std::cos(p[2]),std::sin(p[2]))!=expected)
                    throw std::runtime_error("Surface query changed containment or lane tie-breaking");
                ++checks;
            }
        }
        // The production cache is scratch-only: a fresh sweep must see changes.
        r.surfaceAllowed.assign(r.surfaceAllowed.size(),false);
        corridor_detail::SurfaceQuery empty(r);
        if(empty.containing(0.,0.,1.,0.)!=-1) throw std::runtime_error("Stale permissions");
        r.surfaceAllowed=extended;
        using Clock=std::chrono::steady_clock;
        volatile std::int64_t checksum=0;
        const auto before=Clock::now();
        for(int repeat=0;repeat<10;++repeat) for(std::size_t i=0;i<2400;++i) {
            const auto& p=probes[i];checksum+=reference(r,p[0],p[1],p[2],true,nullptr);
        }
        const auto middle=Clock::now();
        for(int repeat=0;repeat<10;++repeat) {
            corridor_detail::SurfaceQuery query(r);
            for(std::size_t i=0;i<2400;++i) {
                const auto& p=probes[i];checksum+=query.containing(p[0],p[1],std::cos(p[2]),std::sin(p[2]));
            }
        }
        const auto after=Clock::now();
        std::cout<<"equivalent_queries="<<checks<<" reference_ms="
                 <<std::chrono::duration<double,std::milli>(middle-before).count()
                 <<" cached_ms="<<std::chrono::duration<double,std::milli>(after-middle).count()
                 <<" checksum="<<checksum<<'\n';
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
