"""Offline, source-preserving OpenDRIVE lane sampling (no scenario input).

Raw centers are evaluated from planView + laneOffset + lane width. Additional
merge/split maneuvers are explicitly tagged and constrained by this source map.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import bisect
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.integrate import solve_ivp


def number(node, name, default=0.0):
    value = float(node.get(name, default))
    if not math.isfinite(value):
        raise ValueError(f"non-finite {node.tag}.{name}")
    return value


def angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


class Piecewise:
    def __init__(self, nodes=(), origin=0.0, key="sOffset"):
        self.rows = []
        for node in nodes:
            row = [origin + number(node, key)] + [number(node, n) for n in "abcd"]
            # VTD repeats effectively equal starts with round-off; XML's last
            # declaration wins. Same convention as the existing C++ parser.
            old = next((i for i, r in enumerate(self.rows)
                        if abs(r[0] - row[0]) <= 1e-6), None)
            if old is None:
                self.rows.append(row)
            else:
                row[0] = min(row[0], self.rows[old][0])
                self.rows[old] = row
        self.rows.sort()

    def jet(self, s):
        rows = [r for r in self.rows if r[0] <= s + 1e-9]
        if not rows:
            return np.zeros(3)
        start, a, b, c, d = rows[-1]
        u = max(0.0, s - start)
        return np.array([a + u * (b + u * (c + u * d)),
                         b + 2 * c * u + 3 * d * u * u, 2 * c + 6 * d * u])


class Geometry:
    def __init__(self, node):
        self.s, self.x, self.y, self.h, self.length = (
            number(node, n) for n in ("s", "x", "y", "hdg", "length"))
        self.shape = next(iter(node))
        self.kind = self.shape.tag
        self.solution = None
        if self.length <= 0:
            raise ValueError("non-positive planView geometry length")
        if self.kind == "spiral":
            self.k0 = number(self.shape, "curvStart")
            self.dk = (number(self.shape, "curvEnd") - self.k0) / self.length
            self.solution = solve_ivp(
                lambda s, _: [math.cos(self.h + self.k0*s + self.dk*s*s/2),
                              math.sin(self.h + self.k0*s + self.dk*s*s/2)],
                (0, self.length), [0., 0.], rtol=2e-11, atol=1e-11,
                dense_output=True, max_step=5.0)
        elif self.kind == "poly3":
            self.a, self.b, self.c, self.d = (number(self.shape, n) for n in "abcd")
            # u != road arc length s. Invert ds/du = sqrt(1 + v'(u)^2).
            self.solution = solve_ivp(
                lambda _, u: [1 / math.hypot(1, self.b + 2*self.c*u[0] + 3*self.d*u[0]**2)],
                (0, self.length), [0.], rtol=2e-11, atol=1e-11,
                dense_output=True, max_step=5.0)
        elif self.kind not in ("line", "arc"):
            raise ValueError(f"unsupported planView {self.kind}; no approximate fallback")
        if self.solution is not None and not self.solution.success:
            raise ValueError(self.solution.message)

    def pose(self, ds):
        s = min(self.length, max(0., ds))
        if self.kind == "line":
            return np.array([self.x+s*math.cos(self.h), self.y+s*math.sin(self.h), self.h, 0., 0.])
        if self.kind == "arc":
            k = number(self.shape, "curvature")
            if abs(k) < 1e-12:
                return np.array([self.x+s*math.cos(self.h), self.y+s*math.sin(self.h), self.h, 0., 0.])
            h = self.h+k*s
            return np.array([self.x+(math.sin(h)-math.sin(self.h))/k,
                             self.y-(math.cos(h)-math.cos(self.h))/k, h, k, 0.])
        if self.kind == "spiral":
            x, y = self.solution.sol(s)
            return np.array([self.x+x, self.y+y, self.h+self.k0*s+self.dk*s*s/2,
                             self.k0+self.dk*s, self.dk])
        u = float(self.solution.sol(s)[0])
        v = self.a + u*(self.b+u*(self.c+u*self.d))
        dv, ddv = self.b+2*self.c*u+3*self.d*u*u, 2*self.c+6*self.d*u
        k = ddv / (1+dv*dv)**1.5
        dk = 6*self.d/(1+dv*dv)**2 - 3*dv*ddv*ddv/(1+dv*dv)**3
        return np.array([self.x+u*math.cos(self.h)-v*math.sin(self.h),
                         self.y+u*math.sin(self.h)+v*math.cos(self.h),
                         self.h+math.atan(dv), k, dk])


@dataclass(frozen=True, order=True)
class LaneKey:
    road: int
    section: int
    lane: int

    def text(self):
        return f"{self.road}:{self.section}:{self.lane}"


@dataclass
class LanePoint:
    id: int
    key: LaneKey
    index: int
    road_s: float
    lane_s: float
    x: float
    y: float
    heading: float
    curvature: float
    width: float
    next_id: int = -1
    left_id: int = -1
    right_id: int = -1
    left_change: bool = False
    right_change: bool = False
    usable: bool = True


@dataclass
class LaneConnection:
    source: LaneKey
    target: LaneKey
    provenance: str
    contact: str
    junction: int = -1
    connection: str = ""
    gap: float = 0.
    heading_delta: float = 0.
    curvature_delta: float = 0.
    status: str = "pending"


@dataclass
class SampledLane:
    key: LaneKey
    points: list[LanePoint] = field(default_factory=list)
    outgoing: list[int] = field(default_factory=list)


class Road:
    def __init__(self, node):
        self.node = node
        self.id = int(node.get("id"))
        self.length = number(node, "length")
        self.junction = int(node.get("junction", -1))
        if node.get("rule", "RHT") != "RHT":
            raise ValueError("this exporter requires explicit RHT support")
        unsupported = node.findall(".//lane[@direction]") + node.findall(".//lane[@type='bidirectional']")
        if unsupported:
            raise ValueError("lane direction override/bidirectional lanes require explicit support")
        if node.findall(".//border") or node.findall(".//superelevation") or node.findall(".//lateralProfile/shape"):
            raise ValueError("border-defined lanes / banked surface not supported by this planar exporter")
        self.geometries = [Geometry(g) for g in node.findall("planView/geometry")]
        if not self.geometries:
            raise ValueError(f"road {self.id} missing planView")
        self.offset = Piecewise(node.findall("lanes/laneOffset"), key="s")
        self.sections = node.findall("lanes/laneSection")
        self.starts = [number(s, "s") for s in self.sections]
        self.lanes = [{int(l.get("id")): l for l in s.findall("*/lane")} for s in self.sections]
        self.widths = [{lid: Piecewise(l.findall("width"), self.starts[i])
                        for lid, l in lanes.items()} for i, lanes in enumerate(self.lanes)]

    def end(self, section):
        return self.starts[section+1] if section+1 < len(self.starts) else self.length

    def ref(self, s):
        g = self.geometries[0]
        for candidate in self.geometries:
            if candidate.s > s + 1e-9:
                break
            g = candidate
        return g.pose(s-g.s)

    def offset_jet(self, section, lane, s, outer=False):
        t = self.offset.jet(s)
        if lane == 0:
            return t
        sign = 1 if lane > 0 else -1
        for lid in range(sign, lane, sign):
            if lid in self.widths[section]:
                t = t + sign*self.widths[section][lid].jet(s)
        return t + sign*(1. if outer else .5)*self.widths[section][lane].jet(s)

    def at_offset(self, s, t):
        x, y, h, _, _ = self.ref(s)
        return np.array([x-t*math.sin(h), y+t*math.cos(h)])

    def lane_pose(self, section, lane, s):
        x, y, h, k, dk = self.ref(s)
        t, dt, ddt = self.offset_jet(section, lane, s)
        a, b = 1-k*t, dt
        aa, bb = -dk*t-2*k*dt, k*a+ddt
        speed = math.hypot(a, b)
        direction = -1 if lane > 0 else 1
        yaw = angle(h+math.atan2(b, a)+(math.pi if direction < 0 else 0))
        curvature = direction*(a*bb-b*aa)/max(speed**3, 1e-15)
        return x-t*math.sin(h), y+t*math.cos(h), yaw, curvature

    def stations(self, section, interval):
        start, end = self.starts[section], self.end(section)
        knots = [start, end] + [g.s for g in self.geometries]
        knots += [r[0] for r in self.offset.rows]
        for w in self.widths[section].values():
            knots += [r[0] for r in w.rows]
        knots += [start+number(m, "sOffset") for l in self.lanes[section].values() for m in l.findall("roadMark")]
        stations = list(np.linspace(start, end, max(1, math.ceil((end-start)/interval))+1))
        for s in knots:
            if start <= s <= end:
                stations.append(s)
                if start+1e-5 < s < end:
                    stations.append(s-1e-5)  # expose, don't smooth, source discontinuities
        return sorted(set(stations))


class LaneNetwork:
    def __init__(self, source: Path, interval=.5, vehicle_width=1.88,
                 wheelbase=2.944, max_steering=.48):
        if not .05 <= interval <= 2.0:
            raise ValueError("sample interval must be between 0.05 and 2 metres")
        self.source = source
        self.root = ET.parse(source).getroot()
        if self.root.tag != "OpenDRIVE":
            raise ValueError("expected OpenDRIVE")
        self.interval = interval
        self.vehicle_width = vehicle_width
        self.max_curvature = math.tan(max_steering)/wheelbase
        self.roads = {int(r.get("id")): Road(r) for r in self.root.findall("road")}
        self.lanes: dict[LaneKey, SampledLane] = {}
        self.points: list[LanePoint] = []
        self.connections: list[LaneConnection] = []
        self.issues = []
        self.sample()
        self.connect()
        self.adjacency()
        from lane_transitions import build_lane_transitions
        build_lane_transitions(self)

    def issue(self, kind, key, x, y, detail):
        self.issues.append(dict(kind=kind, lane=key.text(), x=float(x), y=float(y), detail=detail))

    def sample(self):
        for rid, road in sorted(self.roads.items()):
            for section, lanes in enumerate(road.lanes):
                stations = road.stations(section, self.interval)
                for lid, lane_node in sorted(lanes.items()):
                    if lid == 0 or lane_node.get("type") != "driving":
                        continue
                    if not road.widths[section][lid].rows:
                        raise ValueError(f"driving lane {rid}/{section}/{lid} lacks width")
                    key = LaneKey(rid, section, lid)
                    lane = SampledLane(key)
                    self.lanes[key] = lane
                    # Refine exact XODR evaluations where offset curvature makes
                    # world-space spacing exceed the requested sample interval.
                    dense = [stations[0]]
                    def refine(a, b, depth=0):
                        pa, pb = road.lane_pose(section, lid, a), road.lane_pose(section, lid, b)
                        if b-a > 1e-4 and depth < 14 and math.hypot(pa[0]-pb[0], pa[1]-pb[1]) > self.interval*1.001:
                            m = (a+b)/2
                            refine(a, m, depth+1)
                            refine(m, b, depth+1)
                        else:
                            dense.append(b)
                    for a, b in zip(stations, stations[1:]):
                        refine(a, b)
                    if lid > 0:
                        dense.reverse()
                    lane_s = 0.
                    for s in dense:
                        x, y, yaw, k = road.lane_pose(section, lid, s)
                        width = float(road.widths[section][lid].jet(s)[0])
                        point = LanePoint(len(self.points), key, len(lane.points), float(s), lane_s,
                                          float(x), float(y), yaw, float(k), width,
                                          usable=width >= self.vehicle_width and abs(k) <= self.max_curvature)
                        if lane.points:
                            prior = lane.points[-1]
                            distance = math.hypot(x-prior.x, y-prior.y)
                            lane_s += distance
                            point.lane_s = lane_s
                            # A width/geometry jump is not a valid next-point edge.
                            jumped = (distance > self.interval*1.01 or
                                      (abs(s-prior.road_s) < 1e-4 and distance > .02))
                            heading_jump = (abs(s-prior.road_s) < 1e-4 and
                                            abs(angle(yaw-prior.heading)) > math.radians(5))
                            if jumped or heading_jump:
                                self.issue("source_position_jump" if jumped else "source_heading_jump", key, x, y,
                                           f"road_s={s:.6f} gap={distance:.4f}m heading={math.degrees(angle(yaw-prior.heading)):.2f}deg")
                            else:
                                prior.next_id = point.id
                        lane.points.append(point)
                        self.points.append(point)
                    narrow = min(lane.points, key=lambda p: p.width)
                    curved = max(lane.points, key=lambda p: abs(p.curvature))
                    if narrow.width < self.vehicle_width:
                        self.issue("narrow_or_tapering_lane", key, narrow.x, narrow.y,
                                   f"min width={narrow.width:.3f}m; vehicle width={self.vehicle_width:.3f}m")
                    if abs(curved.curvature) > self.max_curvature:
                        self.issue("curvature_exceeds_vehicle", key, curved.x, curved.y,
                                   f"curvature={curved.curvature:.4f}/m limit={self.max_curvature:.4f}/m")

    def add_connection(self, source, target, provenance, contact, junction=-1, connection=""):
        if source not in self.lanes:
            return
        edge = LaneConnection(source, target, provenance, contact, junction, connection)
        if any(e.source == source and e.target == target and e.provenance == provenance
               for e in self.connections):
            return
        self.connections.append(edge)
        self.lanes[source].outgoing.append(len(self.connections)-1)
        a = self.lanes[source].points[-1]
        if target not in self.lanes:
            edge.status = "missing_driving_target"
        elif (contact == "start") != (target.lane < 0):
            edge.status = "wrong_contact_direction"
        else:
            b = self.lanes[target].points[0]
            edge.gap = math.hypot(a.x-b.x, a.y-b.y)
            edge.heading_delta = abs(angle(a.heading-b.heading))
            edge.curvature_delta = abs(a.curvature-b.curvature)
            if edge.gap > .05:
                edge.status = "position_gap"
            elif edge.heading_delta > math.radians(5):
                edge.status = "heading_jump"
            elif a.width < 1e-3 or b.width < 1e-3:
                edge.status = "zero_width_endpoint"
            elif edge.curvature_delta > .05:
                edge.status = "curvature_jump"
            else:
                edge.status = "continuous"
        if edge.status != "continuous":
            self.issue("connection_"+edge.status, source, a.x, a.y,
                       f"to={target.text()} {provenance}; gap={edge.gap:.4f}m "
                       f"heading={math.degrees(edge.heading_delta):.2f}deg")

    def connect(self):
        for key, lane in self.lanes.items():
            road = self.roads[key.road]
            node = road.lanes[key.section][key.lane]
            name = "successor" if key.lane < 0 else "predecessor"
            links = node.findall("link/"+name)
            next_section = key.section + (1 if key.lane < 0 else -1)
            if 0 <= next_section < len(road.sections):
                for link in links:
                    self.add_connection(key, LaneKey(key.road, next_section, int(link.get("id"))),
                                        "lane_section_"+name, "start" if key.lane < 0 else "end")
            else:
                link = road.node.find("link/"+name)
                if link is not None and link.get("elementType") == "road":
                    rid = int(link.get("elementId"))
                    contact = link.get("contactPoint", "")
                    if contact not in ("start", "end"):
                        raise ValueError(f"missing road contactPoint: {key}")
                    sec = (len(self.roads[rid].sections)-1 if contact == "end" and rid in self.roads else 0)
                    for lane_link in links:
                        self.add_connection(key, LaneKey(rid, sec, int(lane_link.get("id"))),
                                            "road_"+name, contact)
        for junction in self.root.findall("junction"):
            jid = int(junction.get("id"))
            for connection in junction.findall("connection"):
                if connection.get("connectingRoad") is None:
                    raise ValueError("unsupported direct/virtual junction")
                incoming, target = int(connection.get("incomingRoad")), int(connection.get("connectingRoad"))
                if incoming not in self.roads or target not in self.roads:
                    raise ValueError("junction references missing road")
                contact = connection.get("contactPoint", "")
                if contact not in ("start", "end"):
                    raise ValueError("junction missing contactPoint")
                for link in connection.findall("laneLink"):
                    lid, tid = int(link.get("from")), int(link.get("to"))
                    source_section = len(self.roads[incoming].sections)-1 if lid < 0 else 0
                    target_section = len(self.roads[target].sections)-1 if contact == "end" else 0
                    source = LaneKey(incoming, source_section, lid)
                    # The incoming lane must actually exit toward this junction.
                    road_link = self.roads[incoming].node.find("link/"+("successor" if lid < 0 else "predecessor"))
                    if road_link is None or road_link.get("elementType") != "junction" or int(road_link.get("elementId")) != jid:
                        if source in self.lanes:
                            p = self.lanes[source].points[-1]
                            self.issue("junction_direction_mismatch", source, p.x, p.y, f"junction={jid}")
                        continue
                    self.add_connection(source, LaneKey(target, target_section, tid),
                                        "junction_laneLink", contact, jid, connection.get("id", ""))
        for key, lane in self.lanes.items():
            if not lane.outgoing:
                p = lane.points[-1]
                self.issue("no_declared_successor", key, p.x, p.y,
                           "XODR lane ends here; no nearest-lane connection invented")

    def adjacency(self):
        ascending = {k: v.points if k.lane < 0 else list(reversed(v.points)) for k, v in self.lanes.items()}
        station_arrays = {k: [p.road_s for p in points] for k, points in ascending.items()}
        for key, lane in self.lanes.items():
            road = self.roads[key.road]
            for p in lane.points:
                # In RHT, toward the reference line is driver's left on either side.
                for side, tid in (("left", key.lane+(1 if key.lane < 0 else -1)),
                                  ("right", key.lane+(-1 if key.lane < 0 else 1))):
                    target = self.lanes.get(LaneKey(key.road, key.section, tid))
                    if tid == 0 or target is None or tid*key.lane <= 0:
                        continue
                    # Binary search in road-s, not Euclidean nearest across roads.
                    ps = ascending[target.key]
                    stations = station_arrays[target.key]
                    i = min(bisect.bisect_left(stations, p.road_s), len(ps)-1)
                    if i and abs(ps[i-1].road_s-p.road_s) < abs(ps[i].road_s-p.road_s):
                        i -= 1
                    setattr(p, side+"_id", ps[i].id)
                    owner = key.lane if abs(key.lane) < abs(tid) else tid
                    marks = road.lanes[key.section][owner].findall("roadMark")
                    valid = [m for m in marks if road.starts[key.section]+number(m,"sOffset") <= p.road_s+1e-9]
                    if not valid:
                        # An absent marking is open pavement, not a solid
                        # boundary.  Junction taper lanes frequently omit the
                        # mark while their width grows from zero.
                        setattr(p, side+"_change", True)
                        continue
                    mark = valid[-1]
                    permission = mark.get("laneChange", "none")
                    # Conservative metadata only. Full maneuver still needs online validation.
                    allowed = mark.get("type") == "none" or mark.get("type") == "broken" and (
                        permission == "both" or permission == "increase" and tid > key.lane or
                        permission == "decrease" and tid < key.lane)
                    setattr(p, side+"_change", bool(allowed))
