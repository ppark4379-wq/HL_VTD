"""XODR-constrained maneuvers for lanes born/dropped inside a road.

Raw lane centers stay immutable. Maneuver endpoints reference existing samples;
intermediate points follow a quintic lateral profile in the source road frame.
These are generated paths, not additional links claimed to exist in the XML.
"""
from collections import Counter, defaultdict
from dataclasses import dataclass, field
import bisect
import math

import numpy as np

from xodr_network import LaneKey, angle, number


@dataclass
class TransitionPoint:
    road_s: float
    s: float
    x: float
    y: float
    heading: float
    curvature: float


@dataclass
class LaneTransition:
    id: int
    kind: str
    event_lane: LaneKey
    from_point: int
    to_point: int
    points: list[TransitionPoint] = field(default_factory=list)
    replaced_point_ids: list[int] = field(default_factory=list)
    min_clearance: float = 0.
    max_curvature: float = 0.
    provenance: str = 'xodr_lane_lifecycle+adjacency+roadMark+quintic'
    shape_method: str = 'local_quintic'
    taper_range: tuple | None = None
    roadmark_conflicts: tuple = ()
    vehicle_footprint_valid: bool = True
    geometry_only_reasons: tuple = ()


class LaneTrack:
    """Follow explicit same-road longitudinal links through lane renumbering."""
    def __init__(self, network, key, incoming, allow_source_gaps=False):
        self.network = network
        keys = {key}
        for reverse in (False, True):
            current = key
            while True:
                edges = incoming.get(current, []) if reverse else [network.connections[i] for i in network.lanes[current].outgoing]
                adjacent = []
                for e in edges:
                    target = e.source if reverse else e.target
                    if (target in network.lanes and target.road == key.road and
                            target.lane*key.lane > 0 and (allow_source_gaps or e.gap <= .05) and
                            e.status not in ('wrong_contact_direction','missing_driving_target')):
                        adjacent.append(target)
                if len(adjacent) != 1 or adjacent[0] in keys:
                    break
                current = adjacent[0]
                keys.add(current)
        self.keys = keys
        self.by_section = {k.section: k for k in keys}
        self.points = sorted((p for k in keys for p in network.lanes[k].points), key=lambda p: p.road_s)
        self.stations = [p.road_s for p in self.points]

    def point(self, station):
        if station < self.stations[0]-1e-6 or station > self.stations[-1]+1e-6:
            return None
        i = min(bisect.bisect_left(self.stations, station),len(self.points)-1)
        if i and abs(self.stations[i-1]-station)<abs(self.stations[i]-station):
            i -= 1
        return self.points[i]

    def key_at(self, station):
        road = self.network.roads[self.points[0].key.road]
        si = min(max(bisect.bisect_right(road.starts, station)-1, 0), len(road.starts)-1)
        return self.by_section.get(si)


def taper_geometry(branch, event, kind, neighbour, clearance_offset=0.):
    """Locate the real zero-width taper, not a fixed look-ahead lane change.

    The guide stays half a developed lane width away from the boundary opposite
    the through lane. It therefore follows the actual widening/merging boundary
    parallel to it, rather than following the vanishing lane's boundary-centre.
    """
    if event.width > .20:
        return None
    road = branch.network.roads[event.key.road]
    travel = -1 if event.key.lane > 0 else 1
    away = travel if kind == 'split' else -travel
    rows = sorted((p for p in branch.points if 0 <= away*(p.road_s-event.road_s) <= 80),
                  key=lambda p: away*(p.road_s-event.road_s))
    if not rows:
        return None
    nominal = max(p.width for p in rows)
    if nominal < branch.network.vehicle_width:
        return None
    developed = next((p for p in rows if p.width >= .98*nominal), None)
    if developed is None or abs(developed.road_s-event.road_s) < 2:
        return None
    lo, hi = sorted((event.road_s, developed.road_s))
    outer = abs(event.key.lane) > abs(neighbour.lane)
    side = 1 if event.key.lane > 0 else -1
    def guide(station):
        key = branch.key_at(station)
        if key is None:
            raise ValueError('guide outside declared lane track')
        jet = road.offset_jet(key.section, key.lane, station, outer=True).copy()
        if outer:
            jet[0] -= side*(nominal/2+clearance_offset)
        else:
            jet -= side*road.widths[key.section][key.lane].jet(station)
            jet[0] += side*(nominal/2+clearance_offset)
        return jet
    return lo, hi, guide


def quintic(start_jet, end_jet, delta_s):
    c = np.zeros(6)
    c[:3] = (start_jet[0], start_jet[1]*delta_s, start_jet[2]*delta_s**2/2)
    c[3:] = np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],
                           [end_jet[0]-sum(c[:3]), end_jet[1]*delta_s-c[1]-2*c[2],
                            end_jet[2]*delta_s**2-2*c[2]])
    return c


def profile_jet(coefficients, station, start, delta):
    u = (station-start)/delta
    c = coefficients
    return np.array([sum(c[i]*u**i for i in range(6)),
                     sum(i*c[i]*u**(i-1) for i in range(1,6))/delta,
                     sum(i*(i-1)*c[i]*u**(i-2) for i in range(2,6))/delta**2])


def offset_pose(road, station, jet, direction):
    x,y,h,k,dk = road.ref(station)
    t,dt,ddt = jet
    a,b = 1-k*t,dt
    aa,bb = -dk*t-2*k*dt,k*a+ddt
    heading = angle(h+math.atan2(b,a)+(math.pi if direction<0 else 0))
    curvature = direction*(a*bb-b*aa)/max(math.hypot(a,b)**3,1e-15)
    return x-t*math.sin(h),y+t*math.cos(h),heading,curvature


class RoadGuard:
    """Sampled rectangular footprint and road-mark clearance in the road frame.

Projection uses the reference tangent and Newton refinement, not the car's
world x axis. Cross sections are spaced <= 0.5m over the full vehicle length.
This is an offline geometric screen; moving objects are deliberately absent.
"""
    def __init__(self, road, direction, vehicle_width, rear=1.055, front=3.800, defer_marks=False):
        self.road = road
        self.direction = direction
        self.half = vehicle_width/2
        self.longitudinals = np.linspace(-rear,front,math.ceil((front+rear)/.5)+1)
        self.defer_marks=defer_marks
        self.mark_conflicts=set()

    def bounds(self, s):
        if s<0 or s>self.road.length:
            return [],[]
        si = min(max(bisect.bisect_right(self.road.starts,s)-1,0),len(self.road.starts)-1)
        intervals,marks = [],[]
        for lid,node in self.road.lanes[si].items():
            if lid == 0 or (-1 if lid>0 else 1) != self.direction:
                continue
            t = self.road.offset_jet(si,lid,s,outer=True)[0]
            width = self.road.widths[si][lid].jet(s)[0]
            inner = t-(1 if lid>0 else -1)*width
            if node.get('type')=='driving' and width>1e-6:
                intervals.append((min(inner,t),max(inner,t)))
            active = [m for m in node.findall('roadMark')
                      if self.road.starts[si]+number(m,'sOffset')<=s+1e-9]
            if active and active[-1].get('type','none')!='none':
                marks.append((float(t),active[-1]))
        union=[]
        for a,b in sorted(intervals):
            if union and a<=union[-1][1]+1e-5:
                union[-1]=(union[-1][0],max(union[-1][1],b))
            else:
                union.append((a,b))
        return union,marks

    def check(self, p, lateral_direction):
        minimum = math.inf
        c,s = math.cos(p.heading),math.sin(p.heading)
        for u in self.longitudinals:
            # The centre and two sides at this longitudinal station cover the
            # rectangle, including lane-surface holes and painted boundaries.
            x,y = p.x+u*c,p.y+u*s
            station = p.road_s+self.direction*u
            outside=False
            for _ in range(3):
                if not 0<=station<=self.road.length:
                    outside=True
                    break
                rx,ry,h,k,_ = self.road.ref(station)
                along = (x-rx)*math.cos(h)+(y-ry)*math.sin(h)
                t = -(x-rx)*math.sin(h)+(y-ry)*math.cos(h)
                station += along/max(.2,1-k*t)
            if outside or not 0<=station<=self.road.length:
                valid,reason,margin=self.check_outside_slice(x,y,p.heading,lateral_direction)
                if not valid:return False,reason,0.
                minimum=min(minimum,margin)
                continue
            rx,ry,h,_,_ = self.road.ref(station)
            # Project both sides onto this cross section. Longitudinal sweep is
            # sampled densely and retains 5cm clearance from the surface edge.
            center_t = -(x-rx)*math.sin(h)+(y-ry)*math.cos(h)
            half = self.half*abs(math.cos(p.heading-h))
            low,high = center_t-half,center_t+half
            intervals,marks = self.bounds(station)
            containing = [(a,b) for a,b in intervals if low>=a+.05 and high<=b-.05]
            if not containing:
                return False,'footprint_outside_driving_surface',0.
            minimum=min(minimum,max(min(low-a,b-high) for a,b in containing))
            for boundary,mark in marks:
                if not low-.01<boundary<high+.01:
                    continue
                kind = mark.get('type','none')
                if kind!='broken':
                    self.mark_conflicts.add('solid_boundary')
                    if not self.defer_marks:return False,'solid_boundary',0.
                    continue
                permission = mark.get('laneChange','none')
                if not (permission=='both' or permission=='increase' and lateral_direction>0 or
                        permission=='decrease' and lateral_direction<0):
                    self.mark_conflicts.add('roadmark_lane_change_forbidden')
                    if not self.defer_marks:return False,'roadmark_lane_change_forbidden',0.
        return True,'accepted',minimum

    def check_outside_slice(self,x,y,heading,lateral_direction):
        return False,'footprint_outside_road',0.


def make_candidate(network, road, start, end, guard, taper=None):
    guard.mark_conflicts.clear()
    delta=end.road_s-start.road_s
    if abs(delta)<5 or start.width<network.vehicle_width or end.width<network.vehicle_width:
        return None,'insufficient_anchor_space'
    direction=1 if delta>0 else -1
    sj=road.offset_jet(start.key.section,start.key.lane,start.road_s)
    ej=road.offset_jet(end.key.section,end.key.lane,end.road_s)
    coefficients=quintic(sj,ej,delta)
    evaluate = lambda s: profile_jet(coefficients, s, start.road_s, delta)
    guide_knots = []
    if taper is not None:
        lo, hi, guide = taper
        # Keep the central 60% exactly boundary-parallel. Two quintics round
        # the approach/departure corners while matching both endpoint jets.
        g0, g1 = lo+.2*(hi-lo), hi-.2*(hi-lo)
        if direction < 0:
            g0, g1 = g1, g0
        if direction*(g0-start.road_s) <= 0 or direction*(end.road_s-g1) <= 0:
            return None, 'anchors_do_not_cover_taper'
        c0 = quintic(sj, guide(g0), g0-start.road_s)
        c1 = quintic(guide(g1), ej, end.road_s-g1)
        def evaluate(s):
            if direction*(s-g0) < 0:
                return profile_jet(c0,s,start.road_s,g0-start.road_s)
            if direction*(s-g1) > 0:
                return profile_jet(c1,s,g1,end.road_s-g1)
            return guide(s)
        guide_knots = [g0,g1]
    stations=list(np.linspace(start.road_s,end.road_s,math.ceil(abs(delta)/.25)+1))
    stations += guide_knots
    for si in range(len(road.sections)):
        if road.end(si)<min(start.road_s,end.road_s) or road.starts[si]>max(start.road_s,end.road_s):
            continue
        stations += [s for s in road.stations(si,2.) if min(start.road_s,end.road_s)<s<max(start.road_s,end.road_s)]
    stations=sorted(set(stations),reverse=direction<0)
    path=[]
    distance=0.
    clearance=math.inf
    for station in stations:
        jet=evaluate(station)
        x,y,h,k=offset_pose(road,station,jet,direction)
        if abs(k)>network.max_curvature:
            return None,'curvature_limit'
        if path:
            prior=path[-1]
            ds=math.hypot(x-prior.x,y-prior.y)
            if ds>.51 or abs(angle(h-prior.heading))>network.max_curvature*ds+.002:
                return None,'source_geometry_discontinuity'
            distance+=ds
        point=TransitionPoint(station,distance,x,y,h,k)
        path.append(point)
    # Numerical endpoints coincide with actual sample IDs (no display-only link).
    for p,q in ((path[0],start),(path[-1],end)):
        if math.hypot(p.x-q.x,p.y-q.y)>1e-6 or abs(angle(p.heading-q.heading))>1e-6 or abs(p.curvature-q.curvature)>1e-6:
            return None,'endpoint_mismatch'
    # Check every exported sample, including both anchors. make_candidate rejects
    # successive samples farther than 0.51m apart (normally about 0.25m).
    lateral_direction=np.sign(ej[0]-sj[0])
    for p in path:
        valid,reason,margin=guard.check(p,lateral_direction)
        if not valid:
            return None,reason
        clearance=min(clearance,margin)
    return (path,clearance),'accepted'


def build_lane_transitions(network):
    network.transitions=[]
    network.transition_diagnostics=[]
    network.replaced_points={}
    incoming=defaultdict(list)
    for e in network.connections:
        if e.target in network.lanes:
            incoming[e.target].append(e)
    track_cache={}
    def track(key):
        if key not in track_cache:
            track_cache[key]=LaneTrack(network,key,incoming)
        return track_cache[key]
    for key,lane in network.lanes.items():
        road=network.roads[key.road]
        # Junction connector roads can themselves widen from one usable lane
        # into two (or taper back to one).  OpenDRIVE often expresses that as
        # a zero-width lane birth/drop inside a junction road, without an
        # incoming laneLink for the newborn lane.  Skipping every junction
        # road therefore removed exactly the precomputed branch needed to
        # enter that lane.  Treat lifecycle geometry identically everywhere;
        # the same footprint, curvature and road-mark guards below decide
        # whether a transition is usable.
        direction=-1 if key.lane>0 else 1
        for kind,event in (('merge',lane.points[-1]),('split',lane.points[0])):
            # A connector can declare a lane at its road boundary with zero
            # width and then widen it inside the road (or taper it to zero at
            # the far boundary).  It has no incoming/outgoing laneLink by
            # construction, but it is still a real XODR lifecycle event.
            # Keep ordinary full-width map boundaries terminal; admit only a
            # narrow endpoint whose width polynomial supplies the taper.
            interior=.01<event.road_s<road.length-.01
            endpoint_taper=event.width<=.20 and (
                kind=='split' and event.road_s<=.01 or
                kind=='merge' and event.road_s>=road.length-.01)
            if not interior and not endpoint_taper:
                continue
            edges=[network.connections[i] for i in lane.outgoing] if kind=='merge' else incoming.get(key,[])
            if edges:
                continue  # explicit longitudinal links are audited separately
            # Only actual lane births/drops: a missing link between two ordinary
            # equal-width sections is not evidence of a lateral merge.
            neighbour_si=key.section+(direction if kind=='merge' else -direction)
            if 0<=neighbour_si<len(road.sections):
                boundary_t=road.offset_jet(key.section,key.lane,event.road_s)[0]
                same_coverage=False
                for lid,node in road.lanes[neighbour_si].items():
                    if lid*key.lane<=0 or node.get('type')!='driving':
                        continue
                    t=road.offset_jet(neighbour_si,lid,event.road_s)[0]
                    if abs(t-boundary_t)<.10 and event.width>network.vehicle_width:
                        same_coverage=True
                if same_coverage:
                    continue
            neighbours=[LaneKey(key.road,key.section,lid) for lid in (key.lane-1,key.lane+1)
                        if lid*key.lane>0 and LaneKey(key.road,key.section,lid) in network.lanes]
            reasons=Counter()
            chosen=None
            branch=track(key)
            guard=RoadGuard(road,direction,network.vehicle_width,defer_marks=True)
            for neighbour in neighbours:
                through=track(neighbour)
                taper = taper_geometry(branch,event,kind,neighbour)
                candidates = []
                if taper:
                    lo,hi,_ = taper
                    first,last = (lo,hi) if direction>0 else (hi,lo)
                    for clearance_offset in (0.,.15,.30,.45):
                        guide = taper_geometry(branch,event,kind,neighbour,clearance_offset)
                        for pad in (4.,6.,8.,10.,12.,2.):
                            start_track,end_track = (branch,through) if kind=='merge' else (through,branch)
                            candidates.append((start_track.point(first-direction*pad),
                                               end_track.point(last+direction*pad),guide))
                # Abrupt lane type changes have no geometric taper. Search
                # locally, shortest feasible first; do not merge 25m early by
                # accepting the first hard-coded 20m candidate.
                for shift in ((4.,5.,6.,8.) if kind=='merge' else (.5,2.,4.,6.)):
                    for length in (15.,18.,20.,25.,30.,40.,12.,10.):
                        if kind=='merge':
                            start=branch.point(event.road_s-direction*(shift+length))
                            end=through.point(event.road_s-direction*shift)
                        else:
                            start=through.point(event.road_s+direction*shift)
                            end=branch.point(event.road_s+direction*(shift+length))
                        candidates.append((start,end,None))
                for start,end,candidate_taper in candidates:
                    if start is None or end is None:
                        reasons['insufficient_track_length']+=1
                        continue
                    if start.id in network.replaced_points or end.id in network.replaced_points:
                        reasons['overlapping_transition']+=1
                        continue
                    result,reason=make_candidate(network,road,start,end,guard,candidate_taper)
                    if result is None:
                        reasons[reason]+=1
                        continue
                    points,margin=result
                    if kind=='merge':
                        replaced=[p.id for p in branch.points if direction*(p.road_s-start.road_s)>1e-7
                                  and direction*(p.road_s-event.road_s)<=1e-7]
                    else:
                        replaced=[p.id for p in branch.points if direction*(p.road_s-event.road_s)>=-1e-7
                                  and direction*(p.road_s-end.road_s)<-1e-7]
                    # Don't remove anchors used by previously accepted maneuvers.
                    anchors={i for t in network.transitions for i in (t.from_point,t.to_point)}
                    if any(p in anchors for p in replaced):
                        reasons['overlapping_transition']+=1
                        continue
                    chosen=LaneTransition(len(network.transitions),kind,key,start.id,end.id,points,
                                          replaced,margin,max(abs(p.curvature) for p in points))
                    chosen.roadmark_conflicts=tuple(sorted(guard.mark_conflicts))
                    if candidate_taper:
                        chosen.shape_method='boundary_parallel_taper'
                        chosen.taper_range=candidate_taper[:2]
                        chosen.provenance='xodr_lane_lifecycle+boundary_parallel+roadMark+C2_blends'
                    break
                if chosen:
                    break
            record=dict(kind=kind,lane=key.text(),road_s=event.road_s,x=event.x,y=event.y,
                        status='generated' if chosen else 'unresolved',rejections=dict(reasons))
            if chosen:
                network.transitions.append(chosen)
                network.replaced_points.update({i:chosen.id for i in chosen.replaced_point_ids})
                record.update(transition_id=chosen.id,from_point=chosen.from_point,to_point=chosen.to_point,
                              min_clearance=chosen.min_clearance,max_curvature=chosen.max_curvature,
                              shape_method=chosen.shape_method,taper_range=chosen.taper_range,
                              roadmark_conflicts=chosen.roadmark_conflicts,
                              start_road_s=chosen.points[0].road_s,end_road_s=chosen.points[-1].road_s)
            network.transition_diagnostics.append(record)
    from reviewed_repairs import apply_reviewed_repairs
    apply_reviewed_repairs(network,incoming)
    from continuity_repairs import repair_source_gaps, advance_split_anchors
    advance_split_anchors(network,incoming)
    repair_source_gaps(network,incoming)
    audit_lane_access(network)
    audit_pavement_directions(network)


def audit_lane_access(network):
    """Directed *topological* entry/exit audit, not a geometric safety claim.

    Preserve explicit laneLinks even when their geometry has diagnostics. A
    lane with no way into/out of its road's network remains a visible geometric
    candidate, not an approved runtime route. Only explicit review exclusions
    hide geometry; reachability must not erase entire approach lane sections.
    """
    forward,reverse = defaultdict(set),defaultdict(set)
    entries,exits = set(),set()
    for edge in network.connections:
        if edge.source not in network.lanes or edge.target not in network.lanes:
            continue
        if edge.status in ('wrong_contact_direction','missing_driving_target'):
            continue
        if edge.source.road != edge.target.road:
            exits.add(edge.source)
            entries.add(edge.target)
        else:
            forward[edge.source].add(edge.target)
            reverse[edge.target].add(edge.source)
    for t in network.transitions:
        a,b = network.points[t.from_point].key,network.points[t.to_point].key
        forward[a].add(b)
        reverse[b].add(a)
    for key,lane in network.lanes.items():
        road = network.roads[key.road]
        for entry,p in ((True,lane.points[0]),(False,lane.points[-1])):
            endpoint = 'predecessor' if p.road_s < .01 else 'successor' if p.road_s > road.length-.01 else None
            # A real open map boundary is a valid terminal. A junction-linked
            # boundary without this lane's laneLink is not an implied terminal.
            if endpoint and road.node.find('link/'+endpoint) is None:
                (entries if entry else exits).add(key)
    def reachable(seeds,edges):
        seen=set(seeds);todo=list(seeds)
        while todo:
            for target in edges[todo.pop()]:
                if target not in seen:
                    seen.add(target);todo.append(target)
        return seen
    has_entry,has_exit = reachable(entries,forward),reachable(exits,reverse)
    network.lane_access={}
    network.excluded_lanes=set()
    network.topology_unconnected_lanes=set()
    for key,lane in network.lanes.items():
        incoming,outgoing = key in has_entry,key in has_exit
        status = 'connected' if incoming and outgoing else 'no_entry_or_exit' if not incoming and not outgoing else 'no_entry' if not incoming else 'no_exit'
        network.lane_access[key]=dict(has_entry=incoming,has_exit=outgoing,status=status)
        if status!='connected':
            network.topology_unconnected_lanes.add(key)
    network.excluded_lanes.update(network.review_excluded_lanes)
    for key in network.review_excluded_lanes:
        network.lane_access[key].update(status='excluded_by_review')
    network.excluded_transitions={t.id for t in network.transitions
                                  if network.points[t.from_point].key in network.excluded_lanes
                                  or network.points[t.to_point].key in network.excluded_lanes}


def audit_pavement_directions(network):
    """Compare source arrow-model forward headings with lane travel tangents.

    This detects conflicting source semantics; it must NOT reverse a lane or
    fabricate junction links based on an external texture/model alone.
    """
    for rid,road in network.roads.items():
        for obj in road.node.findall('objects/object'):
            name=obj.get('name','').upper()
            if not name.startswith(('RM_537_','RM_538_','RM_539_')):
                continue
            s,t=number(obj,'s'),number(obj,'t')
            if not 0<=s<=road.length:
                continue
            si=min(bisect.bisect_right(road.starts,s)-1,len(road.starts)-1)
            for lid,node in road.lanes[si].items():
                if not lid or node.get('type')!='driving':
                    continue
                center=road.offset_jet(si,lid,s)[0]
                width=road.widths[si][lid].jet(s)[0]
                if abs(t-center)>width/2 or width<.1:
                    continue
                yaw=road.lane_pose(si,lid,s)[2]
                arrow_yaw=road.ref(s)[2]+number(obj,'hdg')
                delta=abs(angle(arrow_yaw-yaw))
                if delta>math.pi/2:
                    x,y=road.at_offset(s,t)
                    network.issues.append(dict(kind='pavement_lane_direction_conflict',lane=LaneKey(rid,si,lid).text(),
                                               x=float(x),y=float(y),object_id=obj.get('id'),
                                               detail=f'{name} object {obj.get("id")}: source arrow-model forward heading differs from lane travel by {math.degrees(delta):.1f}deg; do not reverse lane or invent junction links.'))
