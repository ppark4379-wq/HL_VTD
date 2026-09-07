"""Bridge source position breaks on explicitly connected same-road tracks.

Original samples and laneLinks remain unchanged. Generated C2 candidates are
separate edges with actual shared anchors, just like merge/split candidates.
Never infer connections between roads, opposite directions or missing links.
"""
from collections import Counter
import math

from lane_transitions import LaneTrack, LaneTransition, RoadGuard
from reviewed_repairs import LinkedRoadGuard, world_candidate


def advance_split_anchors(network,incoming):
    """Begin local splitting at the taper, without moving downstream joins.

    Refine only after all shared anchors are known. Keeping the old destination
    and replacement span prevents an earlier split from breaking a later merge.
    Source-boundary-parallel guides and manually reviewed cases are unchanged.
    """
    reviewed={c.get('transition_id') for c in network.reviewed_cases}
    changed=0
    for row in network.transition_diagnostics:
        if row['status']!='generated' or row['kind']!='split':continue
        t=network.transitions[row['transition_id']]
        if t.id in reviewed or t.shape_method!='local_quintic':continue
        a,b=network.points[t.from_point],network.points[t.to_point]
        direction=-1 if a.key.lane>0 else 1
        road=network.roads[a.key.road]
        through=LaneTrack(network,a.key,incoming)
        branch=LaneTrack(network,b.key,incoming)
        guard=LinkedRoadGuard(network,road,direction,branch)
        for advance in (4.,2.,0.):
            start=through.point(row['road_s']-direction*advance)
            if start is None or start.id in network.replaced_points:continue
            if direction*(a.road_s-start.road_s)<1.:continue
            result,_=world_candidate(network,road,start,b,guard)
            if result is None:continue
            points,margin=result
            t.from_point=start.id;t.points=points;t.min_clearance=margin
            t.max_curvature=max(abs(p.curvature) for p in points)
            t.roadmark_conflicts=tuple(sorted(guard.mark_conflicts))
            t.shape_method='advanced_split_C2'
            t.provenance='XODR_taper_event+earlier_shared_start+world_C2+footprint'
            row.update(from_point=start.id,start_road_s=start.road_s,
                       shape_method=t.shape_method,min_clearance=margin,max_curvature=t.max_curvature,
                       roadmark_conflicts=t.roadmark_conflicts)
            changed+=1;break
    print('[split refinement]',changed,'earlier shared anchors',flush=True)


def repair_source_gaps(network, incoming):
    network.gap_diagnostics=[]
    events=[]
    for lane in network.lanes.values():
        for a,b in zip(lane.points,lane.points[1:]):
            if a.next_id<0 and math.hypot(a.x-b.x,a.y-b.y)>.05:
                events.append((a,b,'within_lane_source_break'))
    for edge in network.connections:
        if (edge.status=='position_gap' and edge.source.road==edge.target.road
                and edge.source.lane*edge.target.lane>0 and edge.target in network.lanes):
            events.append((network.lanes[edge.source].points[-1],
                           network.lanes[edge.target].points[0],'explicit_laneLink_position_gap'))
    for a,b,cause in events:
        row=dict(lane=a.key.text(),to_lane=b.key.text(),from_point=a.id,to_point=b.id,
                 x=a.x,y=a.y,road_s=a.road_s,cause=cause,
                 source_gap_m=math.hypot(a.x-b.x,a.y-b.y))
        network.gap_diagnostics.append(row)
        if a.key in network.review_excluded_lanes or b.key in network.review_excluded_lanes:
            row['status']='excluded_by_review';continue
        if a.id in network.replaced_points or b.id in network.replaced_points:
            row['status']='superseded_by_transition';continue
        track=LaneTrack(network,a.key,incoming,allow_source_gaps=True)
        if b.key not in track.keys:
            row['status']='ambiguous_declared_track';continue
        direction=-1 if a.key.lane>0 else 1
        road=network.roads[a.key.road]
        full_guard=LinkedRoadGuard(network,road,direction,track)
        center_guard=RoadGuard(road,direction,0.,rear=0.,front=0.,defer_marks=True)
        anchors={i for t in network.transitions for i in (t.from_point,t.to_point)}
        candidates=[]
        for before in (.5,1.,2.,3.,5.,8.,12.,18.):
            for after in (.5,1.,2.,3.,5.,8.,12.,18.):
                start=track.point(a.road_s-direction*before)
                end=track.point(b.road_s+direction*after)
                if start is None or end is None:continue
                if start.id in network.replaced_points or end.id in network.replaced_points:continue
                replaced=[p.id for p in track.points if direction*(p.road_s-start.road_s)>1e-7
                          and direction*(end.road_s-p.road_s)>1e-7]
                if any(i in anchors or i in network.replaced_points for i in replaced):continue
                candidates.append((before+after,start,end,replaced))
        reasons=Counter();chosen=None
        for guard,footprint_valid in ((full_guard,True),(center_guard,False)):
            for _,start,end,replaced in sorted(candidates,key=lambda c:c[0]):
                result,reason=world_candidate(network,road,start,end,guard,min_length=.25)
                if result is None:
                    reasons[reason]+=1;continue
                points,margin=result
                chosen=LaneTransition(len(network.transitions),'continuation',a.key,start.id,end.id,
                                      points,replaced,margin if footprint_valid else 0.,
                                      max(abs(p.curvature) for p in points),
                                      cause+'+source_track+world_C2+planning_required',
                                      'source_gap_world_C2')
                chosen.roadmark_conflicts=tuple(sorted(guard.mark_conflicts))
                chosen.vehicle_footprint_valid=footprint_valid
                chosen.geometry_only_reasons=() if footprint_valid else tuple(sorted(reasons))
                break
            if chosen:break
        if chosen:
            network.transitions.append(chosen)
            network.replaced_points.update({i:chosen.id for i in chosen.replaced_point_ids})
            row.update(status='generated',transition_id=chosen.id,
                       anchor_from=chosen.from_point,anchor_to=chosen.to_point,
                       vehicle_footprint_valid=chosen.vehicle_footprint_valid)
        else:row['status']='unresolved'
        row['rejections']=dict(reasons)
    print('[gap repair]',dict(Counter(r['status'] for r in network.gap_diagnostics)),flush=True)
