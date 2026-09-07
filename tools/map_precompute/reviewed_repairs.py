"""Explicit, source-hash-bound map review and guarded Cartesian C2 repairs.

Never alter XODR or invent junction connectivity. The world-space curve bridges
source-frame derivative discontinuities, but still has to pass road/paint and
sampled vehicle-footprint checks. Exclusions are separate from original data.
"""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from xodr_network import LaneKey,angle,number
from lane_transitions import RoadGuard,TransitionPoint


class LinkedRoadGuard(RoadGuard):
    """A road's XML endpoint isn't a wall when an explicit laneLink continues.

    Only footprint slices outside this road may use declared predecessor or
    successor roads of the branch track. No nearest-road or new turn is added.
    """
    def __init__(self,network,road,direction,branch):
        super().__init__(road,direction,network.vehicle_width,defer_marks=True)
        self.linked=[]
        for edge in network.connections:
            other=edge.target if edge.source in branch.keys else edge.source if edge.target in branch.keys else None
            if other is None or other.road==road.id or other not in network.lanes or edge.gap>.05:continue
            target=network.roads[other.road]
            self.linked.append(RoadGuard(target,-1 if other.lane>0 else 1,network.vehicle_width,rear=0,front=0,defer_marks=True))

    def check_outside_slice(self,x,y,heading,lateral_direction):
        for guard in self.linked:
            road=guard.road
            for seed in (0.,road.length):
                s=seed
                for _ in range(6):
                    rx,ry,h,k,_=road.ref(min(road.length,max(0,s)))
                    along=(x-rx)*math.cos(h)+(y-ry)*math.sin(h)
                    t=-(x-rx)*math.sin(h)+(y-ry)*math.cos(h)
                    if not 0<=s<=road.length:break
                    s+=along/max(.2,1-k*t)
                if not 0<=s<=road.length:continue
                rx,ry,h,_,_=road.ref(s)
                if abs((x-rx)*math.cos(h)+(y-ry)*math.sin(h))>.01:continue
                p=TransitionPoint(s,0.,x,y,heading,0.)
                guard.mark_conflicts.clear()
                valid,reason,margin=guard.check(p,lateral_direction)
                if valid:
                    self.mark_conflicts.update(guard.mark_conflicts)
                    return True,'declared_linked_surface',margin
        return False,'footprint_outside_declared_surfaces',0.


def world_candidate(network,road,start,end,guard,scale=1.,min_length=3.):
    from lane_transitions import TransitionPoint,quintic,profile_jet
    guard.mark_conflicts.clear()
    length=math.hypot(end.x-start.x,end.y-start.y)*scale
    if length<min_length or min(start.width,end.width)<network.vehicle_width:
        return None,'insufficient_anchor_space'
    direction=1 if end.road_s>start.road_s else -1
    if (start.key.road!=end.key.road or start.key.road!=road.id or start.key.lane*end.key.lane<=0 or
            direction!=(-1 if start.key.lane>0 else 1)):
        return None,'wrong_travel_direction'
    jets=[]
    for p in (start,end):
        c,s=math.cos(p.heading),math.sin(p.heading)
        jets.append(([p.x,length*c,-length**2*p.curvature*s],
                     [p.y,length*s,length**2*p.curvature*c]))
    cx=quintic(jets[0][0],jets[1][0],1.)
    cy=quintic(jets[0][1],jets[1][1],1.)
    lateral=np.sign(road.offset_jet(end.key.section,end.key.lane,end.road_s)[0]-
                    road.offset_jet(start.key.section,start.key.lane,start.road_s)[0])
    points=[];distance=0.;clearance=math.inf
    for u in np.linspace(0,1,max(2,math.ceil(length/.18))+1):
        x,dx,ddx=profile_jet(cx,u,0,1);y,dy,ddy=profile_jet(cy,u,0,1)
        speed=math.hypot(dx,dy)
        if speed<.1:return None,'curve_cusp'
        heading=math.atan2(dy,dx);k=(dx*ddy-dy*ddx)/speed**3
        if abs(k)>network.max_curvature:return None,'curvature_limit'
        station=start.road_s+u*(end.road_s-start.road_s)
        if 0<u<1:
            for _ in range(5):
                if not 0<=station<=road.length:return None,'footprint_outside_road'
                rx,ry,h,rk,_=road.ref(station)
                along=(x-rx)*math.cos(h)+(y-ry)*math.sin(h)
                t=-(x-rx)*math.sin(h)+(y-ry)*math.cos(h)
                station+=along/max(.2,1-rk*t)
        if points:
            prev=points[-1];ds=math.hypot(x-prev.x,y-prev.y)
            if ds>.51 or direction*(station-prev.road_s)<-1e-8:return None,'nonmonotone_projection'
            if abs(angle(heading-prev.heading))>network.max_curvature*ds+.002:return None,'curve_heading_jump'
            distance+=ds
        p=TransitionPoint(float(station),distance,float(x),float(y),heading,float(k))
        valid,reason,margin=guard.check(p,lateral)
        if not valid:return None,reason
        clearance=min(clearance,margin);points.append(p)
    for p,q in ((points[0],start),(points[-1],end)):
        if math.hypot(p.x-q.x,p.y-q.y)>1e-6 or abs(angle(p.heading-q.heading))>1e-6 or abs(p.curvature-q.curvature)>1e-6:
            return None,'endpoint_mismatch'
    return (points,clearance),'accepted'


def apply_reviewed_repairs(network,incoming):
    from lane_transitions import LaneTrack,LaneTransition,RoadGuard,make_candidate,taper_geometry
    network.review_excluded_lanes=set();network.reviewed_cases=[]
    config=json.loads(Path(__file__).with_name('reviewed_events.json').read_text())
    if hashlib.sha256(network.source.read_bytes()).hexdigest()!=config['source_sha256']:
        return
    for spec in config['events']:
        key=LaneKey(*map(int,spec['lane'].split(':')));kind=spec['kind']
        row=next((r for r in network.transition_diagnostics if r['lane']==key.text() and r['kind']==kind),None)
        if row is None:
            raise ValueError(f'reviewed event no longer exists: {spec}')
        case=dict(spec,road_s=row['road_s'],x=row['x'],y=row['y'])
        network.reviewed_cases.append(case)
        if spec['action']=='hold':
            row['status']='held';case['status']='held';continue
        branch=LaneTrack(network,key,incoming,allow_source_gaps=True)
        if spec['action']=='exclude':
            network.review_excluded_lanes.update(branch.keys)
            row['status']='excluded_by_review';row['review_note']=spec['note']
            case.update(status='excluded_by_review',excluded_lanes=[k.text() for k in sorted(branch.keys)])
            continue
        if row['status']=='generated':
            t=network.transitions[row['transition_id']]
            case.update(status='connected',transition_id=t.id,from_point=t.from_point,to_point=t.to_point,
                        start_road_s=t.points[0].road_s,end_road_s=t.points[-1].road_s,
                        roadmark_conflicts=t.roadmark_conflicts)
            continue
        road=network.roads[key.road];event=network.lanes[key].points[-1 if kind=='merge' else 0]
        direction=-1 if key.lane>0 else 1
        neighbours=[LaneKey(key.road,key.section,lid) for lid in (key.lane-1,key.lane+1)
                    if lid*key.lane>0 and LaneKey(key.road,key.section,lid) in network.lanes]
        guard=LinkedRoadGuard(network,road,direction,branch)
        candidates=[]
        for neighbour in neighbours:
            through=LaneTrack(network,neighbour,incoming,allow_source_gaps=True)
            # Search source samples on both sides of the actual lane lifecycle
            # event. Asymmetric approach/departure windows fit short tapers.
            for shift in (-12.,-8.,-6.,-4.,-2.,0.,2.,4.,6.,8.,12.,16.,20.):
                for length in (10.,12.,15.,18.,22.,26.,30.,36.,44.,55.,65.):
                    if kind=='split':
                        start=through.point(event.road_s+direction*shift)
                        end=branch.point(event.road_s+direction*(shift+length))
                    else:
                        start=branch.point(event.road_s-direction*(shift+length))
                        end=through.point(event.road_s-direction*shift)
                    if start is None or end is None:continue
                    if min(start.width,end.width)<network.vehicle_width:continue
                    if kind=='merge' and direction*(start.road_s-event.road_s)>=0:continue
                    if kind=='split' and direction*(end.road_s-event.road_s)<=0:continue
                    if spec['number']==1:
                        # User requested a branch anchor after the source
                        # crosswalk, in this positive lane's -s travel direction.
                        crossings=[o for o in road.node.findall('objects/object') if o.get('name','').startswith('RM_532_')
                                   and abs(number(o,'s')-event.road_s)<15]
                        if any(direction*(start.road_s-number(o,'s'))<number(o,'length')/2+.2 for o in crossings):continue
                    # Prefer an event-local anchor and a moderate curve length.
                    score=abs(shift)+.15*abs(length-18)
                    candidates.append((score,start,end))
        reasons=Counter();chosen=None
        anchors={i for t in network.transitions for i in (t.from_point,t.to_point)}
        tried=set()
        for _,start,end in sorted(candidates,key=lambda v:v[0]):
            if (start.id,end.id) in tried:continue
            tried.add((start.id,end.id))
            if start.id in network.replaced_points or end.id in network.replaced_points:
                reasons['overlapping_transition']+=1;continue
            if kind=='merge':
                replaced=[p.id for p in branch.points if direction*(p.road_s-start.road_s)>1e-7 and direction*(p.road_s-event.road_s)<=1e-7]
            else:
                replaced=[p.id for p in branch.points if direction*(p.road_s-event.road_s)>=-1e-7 and direction*(p.road_s-end.road_s)<-1e-7]
            if any(p in anchors for p in replaced):
                reasons['overlapping_transition']+=1;continue
            for scale in (1.,.85,1.15):
                result,reason=world_candidate(network,road,start,end,guard,scale)
                if result is None:reasons[reason]+=1;continue
                points,margin=result
                chosen=LaneTransition(len(network.transitions),kind,key,start.id,end.id,points,replaced,margin,
                                      max(abs(p.curvature) for p in points),
                                      'user_review+explicit_XODR_track+world_C2+footprint+roadMark',
                                      'reviewed_world_C2')
                break
            if chosen:break
        # Short/narrow tapers need actual endpoint samples, including both
        # one-sided section-boundary poses, not a rounded fixed-distance grid.
        if chosen is None and spec['number'] in (1,13):
            fine=[]
            for neighbour in neighbours:
                through=LaneTrack(network,neighbour,incoming,allow_source_gaps=True)
                starts=[p for p in through.points if abs(p.road_s-event.road_s)<30 and p.width>=network.vehicle_width]
                ends=[p for p in branch.points if p.width>=network.vehicle_width and abs(p.road_s-event.road_s)<40]
                for start in starts[::2]:
                    if spec['number']==1 and any(direction*(start.road_s-number(o,'s'))<number(o,'length')/2+.2 for o in crossings):continue
                    for end in ends:
                        length=direction*(end.road_s-start.road_s)
                        if not 6<=length<=35:continue
                        fine.append((abs(length-18),start,end,neighbour))
            for _,start,end,neighbour in sorted(fine,key=lambda x:x[0]):
                if start.id in network.replaced_points or end.id in network.replaced_points:continue
                replaced=[p.id for p in branch.points if direction*(p.road_s-event.road_s)>=-1e-7 and direction*(p.road_s-end.road_s)<-1e-7]
                if any(p in anchors for p in replaced):continue
                for clearance_offset in (0.,.15,.30):
                    guide=taper_geometry(branch,event,kind,neighbour,clearance_offset)
                    if guide is not None and spec['number']==1:
                        guide=(min(start.road_s,end.road_s),max(start.road_s,end.road_s),guide[2])
                    result,reason=make_candidate(network,road,start,end,guard,guide)
                    if result is None:
                        result,reason=world_candidate(network,road,start,end,guard)
                    if result is None:reasons[reason]+=1;continue
                    points,margin=result
                    chosen=LaneTransition(len(network.transitions),kind,key,start.id,end.id,points,replaced,margin,
                                          max(abs(p.curvature) for p in points),
                                          'user_review+XODR_boundary_guide+shared_samples+footprint+roadMark',
                                          'reviewed_fine_anchor_C2')
                    break
                if chosen:break
        if chosen is None and spec['number']==13:
            # The user requests a geometric candidate network, with vehicle
            # feasibility delegated to planning. Keep a centre-in-road C2
            # candidate when the complete vehicle footprint cannot fit this
            # short road. Record that failure, never advertise it as approved.
            center_guard=RoadGuard(road,direction,0.,rear=0.,front=0.,defer_marks=True)
            for _,start,end,neighbour in sorted(fine,key=lambda x:x[0]):
                if start.id in network.replaced_points or end.id in network.replaced_points:continue
                replaced=[p.id for p in branch.points if direction*(p.road_s-event.road_s)>=-1e-7 and direction*(p.road_s-end.road_s)<-1e-7]
                if any(p in anchors for p in replaced):continue
                result,reason=world_candidate(network,road,start,end,center_guard)
                if result is None:continue
                points,_=result
                chosen=LaneTransition(len(network.transitions),kind,key,start.id,end.id,points,replaced,0.,
                                      max(abs(p.curvature) for p in points),
                                      'user_review+XODR_center_in_surface+world_C2+planning_required',
                                      'geometry_candidate_C2')
                chosen.vehicle_footprint_valid=False
                chosen.geometry_only_reasons=tuple(sorted(reasons))
                guard=center_guard
                break
        if chosen:
            chosen.roadmark_conflicts=tuple(sorted(guard.mark_conflicts))
            network.transitions.append(chosen);network.replaced_points.update({i:chosen.id for i in chosen.replaced_point_ids})
            row.update(status='generated',transition_id=chosen.id,from_point=chosen.from_point,to_point=chosen.to_point,
                       shape_method=chosen.shape_method,min_clearance=chosen.min_clearance,max_curvature=chosen.max_curvature,
                       start_road_s=chosen.points[0].road_s,end_road_s=chosen.points[-1].road_s)
            case.update(status='connected',transition_id=chosen.id,from_point=chosen.from_point,to_point=chosen.to_point,
                        start_road_s=chosen.points[0].road_s,end_road_s=chosen.points[-1].road_s)
            row['roadmark_conflicts']=chosen.roadmark_conflicts
            case['roadmark_conflicts']=chosen.roadmark_conflicts
            row['vehicle_footprint_valid']=chosen.vehicle_footprint_valid
            case['vehicle_footprint_valid']=chosen.vehicle_footprint_valid
            case['geometry_only_reasons']=chosen.geometry_only_reasons
        else:case['status']='unresolved'
        row['review_rejections']=dict(reasons);case['rejections']=dict(reasons)
        print(f"[review {spec['number']:02d}] {key.text()} {case['status']} {dict(reasons)}",flush=True)
