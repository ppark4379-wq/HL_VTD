"""Validate captured school boundaries and expand only unique lane-link chains."""
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def load_school_zone_data(path,corrections_path=None):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8-sig'))
    applied = []
    if not corrections_path:
        return data,applied
    corrections=yaml.safe_load(Path(corrections_path).read_text(encoding='utf-8-sig'))
    if corrections.get('map') != data.get('map'):
        raise ValueError('school corrections map does not match source')
    by_id={zone['id']:zone for zone in data['school_zones']}
    for fix in corrections['corrections']:
        if fix['id'] not in by_id or not fix.get('evidence'):
            raise ValueError('school correction requires a known zone and evidence')
        zone=by_id[fix['id']]
        for key,expected in fix['expected'].items():
            actual=zone.get(key)
            if expected is None:
                if actual is not None:
                    raise ValueError('school correction source has changed')
            elif actual is None or any(actual.get(name)!=value for name,value in expected.items()):
                raise ValueError('school correction source has changed')
        for key in ('point_1','point_2'):
            if key in fix:
                zone[key]=copy.deepcopy(fix[key])
        zone['direct_s_range']=None
        applied.append(dict(id=fix['id'],evidence=fix['evidence'],reason=fix['reason']))
    return data,applied


def build_adjacency(sections,direct_next):
    adjacency = {}
    for road_id,road_sections in sections.items():
        for index,section in enumerate(road_sections):
            for lane in section.findall('left/lane')+section.findall('right/lane'):
                lane_id=int(lane.get('id'))
                for direction in (-1,1):
                    state=(road_id,index,lane_id,direction)
                    for target in direct_next(*state):
                        adjacency.setdefault(state,set()).add(target)
                        target_road,target_index,target_lane,target_direction=target
                        adjacency.setdefault(
                            (target_road,target_index,target_lane,-target_direction),set()
                        ).add((road_id,index,lane_id,-direction))
    return adjacency


def append_link_target(sections,out,target,target_lane,contact):
    if target not in sections or contact not in {'start','end'}:
        return
    index=0 if contact=='start' else len(sections[target])-1
    side='left' if target_lane>0 else 'right'
    if sections[target][index].find(f"{side}/lane[@id='{target_lane}']") is not None:
        out.append((target,index,target_lane,1 if contact=='start' else -1))


class SchoolZones:
    def __init__(self, path, road_map, corrections_path=None):
        self.map = road_map
        data,self.corrections = load_school_zone_data(path,corrections_path)
        if data.get('coordinate_frame') != 'vtd_world' or data.get('map') != 'HL_FMA_VTD_LivingLab':
            raise ValueError('unsupported school-zone coordinate frame or map')
        self.limit = float(data['speed_limit_kph'])
        if not math.isfinite(self.limit) or self.limit <= 0:
            raise ValueError('school speed limit must be positive and finite')
        self.sections = {
            road_id: road.findall('lanes/laneSection')
            for road_id,road in road_map.roads.items()
        }
        self.junctions = getattr(road_map, 'junctions', {})
        self.adjacency = build_adjacency(self.sections,self._direct_next)
        self.ranges = []
        self.pending = []
        self.accepted = []
        self.max_s_error = 0.0
        for zone in data['school_zones']:
            self._add_zone(zone)

    @staticmethod
    def _path_budget(first,last):
        """Maximum lane-link path length accepted for one captured zone.

        The same bound also gives a conservative world-space uncertainty
        envelope when both boundary observations are valid but their lane-link
        path is ambiguous: every point on any admissible path must have a sum
        of Euclidean distances to the two boundaries no greater than this
        budget.
        """
        return math.hypot(first['x']-last['x'],first['y']-last['y'])*1.5+20

    def _validate_boundary(self,point):
        road_id=int(point['track_id'])
        lane_id=int(point['lane_id'])
        station=float(point['s'])
        if (
            road_id not in self.map.roads
            or not 0 <= station <= float(self.map.roads[road_id].get('length'))
        ):
            raise ValueError('road or longitudinal position outside map')
        lane = self.map.lanes_at(road_id,station)['lanes'].get(lane_id)
        if lane is None:
            raise ValueError('lane not present at boundary')
        projected = self.map.project(
            road_id,
            (float(point['x']),float(point['y'])),
            station,
        )
        if projected is None or abs(projected[0]-station) > .25:
            raise ValueError('world coordinate and road s disagree')
        lower,upper=sorted((lane['inner'],lane['outer']))
        if not lower-.25 <= projected[1] <= upper+.25:
            raise ValueError('world coordinate outside specified lane')
        self.max_s_error=max(self.max_s_error,abs(projected[0]-station))

    def _resolve_zone_ranges(self,zone):
        first,last=zone['point_1'],zone.get('point_2')
        if not last:
            raise ValueError('missing second boundary')
        self._validate_boundary(first)
        self._validate_boundary(last)
        direct=zone.get('direct_s_range')
        if direct:
            if first['track_id']!=last['track_id'] or first['lane_id']!=last['lane_id']:
                raise ValueError('direct range joins different roads or lanes')
            lower,upper=sorted((float(first['s']),float(last['s'])))
            if (
                (int(direct['track_id']),int(direct['lane_id']))
                != (first['track_id'],first['lane_id'])
                or abs(float(direct['s_min'])-lower)>1e-6
                or abs(float(direct['s_max'])-upper)>1e-6
            ):
                raise ValueError('direct range differs from boundary points')
            ranges=[(first['track_id'],first['lane_id'],lower,upper)]
            method='direct'
        else:
            paths=self._paths(first,last)
            if len(paths)!=1:
                raise ValueError('no unique continuous lane-link path; boundary pairing needs review')
            ranges=paths[0]
            method='lane_links'
        if sum(upper-lower for _,_,lower,upper in ranges)<.1:
            raise ValueError('boundaries do not define a longitudinal zone')
        return ranges,method

    def _add_zone(self,zone):
        identifier=zone['id']
        try:
            ranges,method=self._resolve_zone_ranges(zone)
            self.ranges.extend(
                dict(id=identifier,road=int(road_id),lane=int(lane_id),s_min=lower,s_max=upper)
                for road_id,lane_id,lower,upper in ranges
            )
            self.accepted.append(dict(id=identifier,method=method,segments=len(ranges)))
        except (ValueError,KeyError,TypeError) as error:
            pending=dict(id=identifier,reason=str(error),uncertainty_scope='global')
            first,last=zone.get('point_1'),zone.get('point_2')
            try:
                if first is None or last is None:
                    raise ValueError('both boundaries are required for bounded uncertainty')
                # Boundary validation is intentionally independent from path
                # resolution.  A bad/missing observation remains global
                # unknown; two trusted observations allow the ambiguity to be
                # bounded without guessing which lane-link path is correct.
                self._validate_boundary(first)
                self._validate_boundary(last)
                budget=self._path_budget(first,last)
                coordinates=[
                    {'x':float(first['x']),'y':float(first['y'])},
                    {'x':float(last['x']),'y':float(last['y'])},
                ]
                if not all(
                    math.isfinite(value)
                    for point in coordinates for value in point.values()
                ) or not math.isfinite(budget):
                    raise ValueError('non-finite boundary uncertainty')
                pending.update(
                    uncertainty_scope='bounded_path_budget',
                    boundary_points=coordinates,
                    path_budget_m=budget,
                )
            except (ValueError,KeyError,TypeError):
                # Do not weaken an unverified boundary into a guessed local
                # region.  It must continue to make all negative samples
                # unavailable until the source is corrected.
                pass
            self.pending.append(pending)

    def _index(self,rid,s):
        return max(i for i,sec in enumerate(self.sections[rid]) if float(sec.get('s'))<=s)

    def _next(self,rid,index,lid,direction):
        return sorted(self.adjacency.get((rid,index,lid,direction),()))

    def _direct_next(self,rid,index,lid,direction):
        sections=self.sections[rid]
        sec=sections[index]
        lane=sec.find(f"{'left' if lid>0 else 'right'}/lane[@id='{lid}']")
        if lane is None:
            return []
        end='successor' if direction>0 else 'predecessor'
        lane_link=lane.find('link/'+end)
        next_index=index+direction
        if 0<=next_index<len(sections):
            return [(rid,next_index,int(lane_link.get('id')),direction)] if lane_link is not None else []
        link=self.map.roads[rid].find('link/'+end)
        if link is None:
            return []
        out=[]
        if link.get('elementType')=='road' and lane_link is not None:
            append_link_target(self.sections,out,int(link.get('elementId')),int(lane_link.get('id')),link.get('contactPoint'))
        elif link.get('elementType')=='junction':
            junction=self.junctions.get(int(link.get('elementId')))
            if junction is not None:
                for connection in junction.findall('connection'):
                    if int(connection.get('incomingRoad'))==rid:
                        for ll in connection.findall('laneLink'):
                            if int(ll.get('from'))==lid:
                                append_link_target(self.sections,out,int(connection.get('connectingRoad')),int(ll.get('to')),connection.get('contactPoint'))
        return out

    def _paths(self,first,last):
        rid,lid,s=int(first['track_id']),int(first['lane_id']),float(first['s'])
        target=(int(last['track_id']),self._index(int(last['track_id']),float(last['s'])),int(last['lane_id']))
        # A boundary pair must describe a local continuous strip, not a detour.
        budget=self._path_budget(first,last)
        stack=[((rid,self._index(rid,s),lid,direction),s,[],0,set()) for direction in (-1,1)]
        found=[]
        visits=0
        while stack and visits<5000:
            state,start,path,length,seen=stack.pop(); visits+=1
            if state in seen or len(path)>24:
                continue
            r,i,l,d=state
            sec=self.sections[r]
            end=float(sec[i+1].get('s')) if d>0 and i+1<len(sec) else float(self.map.roads[r].get('length')) if d>0 else float(sec[i].get('s'))
            if (r,i,l)==target and d*(float(last['s'])-start)>=0:
                end=float(last['s'])
                candidate=path+[(r,l,min(start,end),max(start,end))]
                if length+abs(end-start)<=budget and candidate not in found:
                    found.append(candidate)
                if len(found)>1:
                    break
                continue
            length+=abs(end-start)
            if length>budget:
                continue
            path=path+[(r,l,min(start,end),max(start,end))]
            for nxt in self._next(*state):
                nr,ni,nl,nd=nxt
                ss=self.sections[nr]
                ns=float(ss[ni].get('s')) if nd>0 else float(ss[ni+1].get('s')) if ni+1<len(ss) else float(self.map.roads[nr].get('length'))
                stack.append((nxt,ns,path,length,seen|{state}))
        return found if not stack or len(found)>1 else []

    def measure(self,road,rear):
        if road is None or rear is None or road['road'] not in self.map.roads:
            return None,[]
        projected=self.map.project(road['road'],rear,road['s'])
        if projected is None:
            return None,[]
        s,t=projected
        lanes=self.map.lanes_at(road['road'],s)
        if not lanes:
            return None,[]
        rear_lanes=[lid for lid,lane in lanes['lanes'].items() if min(lane['inner'],lane['outer'])<=t<=max(lane['inner'],lane['outer'])]
        ids=sorted({r['id'] for r in self.ranges if r['road']==road['road'] and r['lane'] in rear_lanes and r['s_min']<=s<=r['s_max']})
        if ids:
            return True,ids
        # A pending zone with unverified boundaries is still global unknown.
        # For two verified boundaries, any path admitted by ``_paths`` lies in
        # this prolate-ellipse superset.  Outside it, a negative membership is
        # therefore provable even though the exact lane-link chain is pending.
        for pending in self.pending:
            if pending.get('uncertainty_scope') != 'bounded_path_budget':
                return None,[]
            first,last=pending['boundary_points']
            distance=(
                math.hypot(rear[0]-first['x'],rear[1]-first['y'])
                + math.hypot(rear[0]-last['x'],rear[1]-last['y'])
            )
            if distance <= pending['path_budget_m']+1e-6:
                return None,[]
        return False,[]

    def report(self):
        return dict(accepted=self.accepted,pending=self.pending,corrections=self.corrections,
                    resolved_ranges=self.ranges,
                    max_boundary_s_error_m=self.max_s_error,complete=not self.pending)
