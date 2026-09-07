"""Whole-export direction and shared-anchor checks, separate from source issues."""
import math
from collections import Counter
from xodr_network import angle


def audit(network):
    failures=[];max_position=max_heading=max_curvature=0.
    for key,lane in network.lanes.items():
        direction=-1 if key.lane>0 else 1
        for a,b in zip(lane.points,lane.points[1:]):
            if direction*(b.road_s-a.road_s)<-1e-8:
                failures.append(dict(kind='raw_travel_direction',lane=key.text(),point=a.id))
    for t in network.transitions:
        a,b=network.points[t.from_point],network.points[t.to_point]
        direction=-1 if a.key.lane>0 else 1
        if a.key.road!=b.key.road or a.key.lane*b.key.lane<=0:
            failures.append(dict(kind='transition_lane_direction',transition=t.id))
        for p,q in ((t.points[0],a),(t.points[-1],b)):
            pos=math.hypot(p.x-q.x,p.y-q.y);hd=abs(angle(p.heading-q.heading));kd=abs(p.curvature-q.curvature)
            max_position=max(max_position,pos);max_heading=max(max_heading,hd);max_curvature=max(max_curvature,kd)
            if pos>1e-6 or hd>1e-6 or kd>1e-6 or q.id in network.replaced_points:
                failures.append(dict(kind='shared_anchor',transition=t.id,point=q.id))
        for p,q in zip(t.points,t.points[1:]):
            ds=math.hypot(p.x-q.x,p.y-q.y)
            if ds>.51 or direction*(q.road_s-p.road_s)<-1e-8:
                failures.append(dict(kind='transition_sample_direction_or_gap',transition=t.id))
    for pid,tid in network.replaced_points.items():
        if tid in network.excluded_transitions and network.points[pid].key not in network.excluded_lanes:
            failures.append(dict(kind='hidden_replacement_leaves_gap',point=pid,transition=tid))
    return dict(lanes_checked=len(network.lanes),transitions_checked=len(network.transitions),
                source_gap_repairs=network.gap_diagnostics,
                visible_candidate_lanes=len(network.lanes)-len(network.excluded_lanes),
                topology_unconnected_but_visible=len(network.topology_unconnected_lanes-network.excluded_lanes),
                geometry_only_transitions=[t.id for t in network.transitions if not t.vehicle_footprint_valid],
                failures=failures,max_anchor_position_error_m=max_position,
                max_anchor_heading_error_rad=max_heading,max_anchor_curvature_error=max_curvature,
                lifecycle_status=dict(Counter(r['status'] for r in network.transition_diagnostics)),
                remaining_unresolved=[r for r in network.transition_diagnostics if r['status']=='unresolved'],
                held=[r for r in network.transition_diagnostics if r['status']=='held'],
                original_connection_status=dict(Counter(e.status for e in network.connections)),
                direction_conflicts=[r for r in network.issues if r['kind']=='pavement_lane_direction_conflict'],
                scope='All raw lane travel order and generated shared anchors sampled; original XODR link geometry discontinuities remain separately reported. Not live-controller safety approval.')
