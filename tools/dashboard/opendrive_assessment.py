"""Local lane boundary measurements from the currently selected OpenDRIVE map.

Reference curves are tessellated at <=0.20 m, lazily per road. No scenario IDs.
Unsupported geometry and unconnected road ends are reported as unavailable.
"""
from __future__ import annotations
import bisect
import math
import xml.etree.ElementTree as ET
from assessment_geometry import obb
from assessment_map_geometry import finite_strip_intrusion, finite_patches_cover


def number(node,key,default=0):
    return float(node.get(key,default))


def polynomial(node, ds, suffix=''):
    return sum(number(node,k+suffix)*ds**i for i,k in enumerate('abcd'))


def active(nodes,s,key):
    eligible=[n for n in nodes if number(n,key)<=s+1e-9]
    return max(eligible,key=lambda n:number(n,key)) if eligible else None


def poly3_derivative(node, u):
    slope = number(node,'b')+2*number(node,'c')*u+3*number(node,'d')*u*u
    return 1/math.sqrt(1+slope**2)


class OpenDriveMap:
    def __init__(self,path):
        root=ET.parse(path).getroot()
        if root.tag!='OpenDRIVE':
            raise ValueError('map must contain OpenDRIVE root')
        self.roads={int(r.get('id')):r for r in root.findall('road')}
        self.junctions={int(j.get('id')):j for j in root.findall('junction')}
        self.samples={}
        self.sample_stations={}

    def reference(self,road_id):
        if road_id not in self.roads:
            return None
        if road_id in self.samples:
            return self.samples[road_id]
        road=self.roads[road_id]
        points=[]
        for geo in road.findall('planView/geometry'):
            s0,x0,y0,h,length=(number(geo,k) for k in ('s','x','y','hdg','length'))
            kind=next(iter(geo))
            n=max(1,math.ceil(length/.2))
            c,si=math.cos(h),math.sin(h)
            u=v=0.0
            for i in range(n+1):
                ds=length*i/n
                if kind.tag=='line':
                    u,v=ds,0
                elif kind.tag=='arc':
                    curvature=number(kind,'curvature')
                    u,v=(math.sin(curvature*ds)/curvature,(1-math.cos(curvature*ds))/curvature) if abs(curvature)>1e-12 else (ds,0)
                elif kind.tag=='poly3':
                    if i:
                        step=length/n
                        # Integrate du/ds to preserve OpenDRIVE arc-length semantics.
                        k1=poly3_derivative(kind,u); k2=poly3_derivative(kind,u+step*k1/2)
                        k3=poly3_derivative(kind,u+step*k2/2); k4=poly3_derivative(kind,u+step*k3)
                        u+=step*(k1+2*k2+2*k3+k4)/6
                    v=polynomial(kind,u)
                elif kind.tag=='paramPoly3':
                    p=ds/length if kind.get('pRange')=='normalized' and length else ds
                    u,v=polynomial(kind,p,'U'),polynomial(kind,p,'V')
                elif kind.tag=='spiral':
                    if i:
                        mid=length*(i-.5)/n
                        angle=number(kind,'curvStart')*mid+(number(kind,'curvEnd')-number(kind,'curvStart'))*mid*mid/(2*length)
                        u+=math.cos(angle)*length/n
                        v+=math.sin(angle)*length/n
                else:
                    self.samples[road_id]=None
                    return None
                points.append((s0+ds,x0+c*u-si*v,y0+si*u+c*v))
        self.samples[road_id]=points
        self.sample_stations[road_id]=[point[0] for point in points]
        return points

    def project(self,road_id,point,s_hint):
        points=self.reference(road_id)
        if not points:
            return None
        ss=self.sample_stations[road_id]
        start=max(0,bisect.bisect_left(ss,s_hint-12)-1)
        end=min(len(points)-1,bisect.bisect_right(ss,s_hint+12))
        best=None
        for i in range(start,end):
            s0,x,y=points[i]; s1,bx,by=points[i+1]
            dx,dy=bx-x,by-y
            length2=dx*dx+dy*dy
            if length2<1e-12: continue
            alpha=((point[0]-x)*dx+(point[1]-y)*dy)/length2
            alpha=max(0,min(1,alpha))
            px,py=x+alpha*dx,y+alpha*dy
            distance=(point[0]-px)**2+(point[1]-py)**2
            t=((point[0]-px)*(-dy)+(point[1]-py)*dx)/math.sqrt(length2)
            if best is None or distance<best[0]: best=(distance,s0+alpha*(s1-s0),t)
        if best is not None:
            # Select the closest point first. Discarding an out-of-road end
            # segment before selection incorrectly snapped an external corner
            # onto an earlier interior segment and manufactured valid coverage.
            for at_start in (True, False):
                endpoint = points[0] if at_start else points[-1]
                if abs(best[1] - endpoint[0]) > 1e-8:
                    continue
                pairs = zip(points, points[1:]) if at_start else zip(reversed(points[:-1]), reversed(points[1:]))
                for first, second in pairs:
                    dx, dy = second[1] - first[1], second[2] - first[2]
                    if dx * dx + dy * dy <= 1e-12:
                        continue
                    along = (point[0] - endpoint[1]) * dx + (point[1] - endpoint[2]) * dy
                    if (at_start and along < -1e-10) or (not at_start and along > 1e-10):
                        return None
                    break
        return None if best is None else best[1:]

    def reference_pose(self, road_id, s):
        """Return an interpolated world reference-line pose at road ``s``."""
        points = self.reference(road_id)
        if not points:
            return None
        if s <= points[0][0]:
            index = 0
            alpha = 0.0
        elif s >= points[-1][0]:
            index = len(points) - 2
            alpha = 1.0
        else:
            ss = self.sample_stations[road_id]
            index = max(0, min(len(points) - 2, bisect.bisect_right(ss, s) - 1))
            span = points[index + 1][0] - points[index][0]
            alpha = 0.0 if span <= 1e-12 else (s - points[index][0]) / span
        _, x0, y0 = points[index]
        _, x1, y1 = points[index + 1]
        return (
            x0 + alpha * (x1 - x0),
            y0 + alpha * (y1 - y0),
            math.atan2(y1 - y0, x1 - x0),
        )

    def lane_center_world(self, road_id, s, lane_id):
        """Return the selected OpenDRIVE lane centre in world coordinates."""
        pose = self.reference_pose(road_id, s)
        data = self.lanes_at(road_id, s) if road_id in self.roads else None
        lane = data and data['lanes'].get(lane_id)
        if pose is None or lane is None:
            return None
        x, y, heading = pose
        lateral = (lane['inner'] + lane['outer']) / 2
        return (
            x - math.sin(heading) * lateral,
            y + math.cos(heading) * lateral,
        )

    def lane_change_direction(self, road_id, s, old_lane, new_lane, ego_heading):
        """Resolve LEFT/RIGHT in the vehicle's travel frame, with audit evidence.

        RDB ``leftLaneId``/``rightLaneId`` and the OpenDRIVE lane-id sign are
        road-reference concepts.  They reverse when a vehicle travels against
        the OpenDRIVE reference direction.  Comparing the two physical lane
        centres against Ego's world heading avoids that inversion.
        """
        old_center = self.lane_center_world(road_id, s, old_lane)
        new_center = self.lane_center_world(road_id, s, new_lane)
        if old_center is None or new_center is None:
            return None, None
        dx = new_center[0] - old_center[0]
        dy = new_center[1] - old_center[1]
        signed_lateral = math.cos(ego_heading) * dy - math.sin(ego_heading) * dx
        evidence = {
            'source': 'xodr_lane_centres_in_ego_travel_frame',
            'old_lane_center': list(old_center),
            'new_lane_center': list(new_center),
            'signed_lateral_m': signed_lateral,
        }
        if abs(signed_lateral) < 0.25:
            return None, evidence
        return ('LEFT' if signed_lateral > 0 else 'RIGHT'), evidence

    def _section_index(self, road_id, s):
        road = self.roads.get(road_id)
        if road is None or not math.isfinite(s) or not 0 <= s <= number(road, 'length') + 1e-8:
            return None
        sections = road.findall('lanes/laneSection')
        indices = [i for i, section in enumerate(sections) if number(section, 's') <= s + 1e-9]
        return indices[-1] if indices else None

    def _lane_node(self, road_id, section_index, lane_id):
        if section_index is None or not lane_id:
            return None
        sections = self.roads[road_id].findall('lanes/laneSection')
        side = 'left' if lane_id > 0 else 'right'
        return sections[section_index].find(f'{side}/lane[@id="{lane_id}"]')

    def _follow_sections(self, road_id, start_section, end_section, lane_id):
        """Return the uniquely linked lane at the requested laneSection."""
        chain = []
        step = 1 if end_section > start_section else -1
        for index in range(start_section, end_section, step):
            lane = self._lane_node(road_id, index, lane_id)
            tag = 'successor' if step > 0 else 'predecessor'
            links = [] if lane is None else lane.findall('link/' + tag)
            if len(links) != 1:
                return None, chain, 'missing_or_ambiguous_lane_section_link'
            target_id = int(links[0].get('id'))
            target = self._lane_node(road_id, index + step, target_id)
            if target is None or target.get('type') != 'driving':
                return None, chain, 'linked_driving_lane_missing'
            inverse = target.findall('link/' + ('predecessor' if step > 0 else 'successor'))
            if inverse and (len(inverse) != 1 or int(inverse[0].get('id')) != lane_id):
                return None, chain, 'conflicting_lane_section_link'
            chain.append(dict(road_id=road_id, from_section=index, to_section=index + step,
                              from_lane=lane_id, to_lane=target_id, link=tag))
            lane_id = target_id
        return lane_id, chain, None

    def classify_lane_transition(self, before_road, after_road):
        """Classify lane identity before evaluating an indicator or solid line.

        ``mapped_old_lane`` is expressed in the *new* road/laneSection. Lane
        numbers are not identities across sections. Across roads, only a unique
        explicit road/junction lane link proves continuity; an ambiguous route
        choice is reported as unknown instead of inventing a lane change.
        """
        result = dict(classification='unknown', mapped_old_lane=None,
                      reason='road_position_unavailable', source='xodr_lane_links',
                      from_road_id=None, to_road_id=None, from_section=None,
                      to_section=None, lane_link_chain=[])
        try:
            old_road, new_road = int(before_road['road']), int(after_road['road'])
            old_lane, new_lane = int(before_road['lane']), int(after_road['lane'])
            old_s, new_s = float(before_road['s']), float(after_road['s'])
        except (TypeError, ValueError, KeyError):
            return result
        old_section = self._section_index(old_road, old_s)
        new_section = self._section_index(new_road, new_s)
        result.update(from_road_id=old_road, to_road_id=new_road,
                      from_section=old_section, to_section=new_section)
        if old_section is None or new_section is None:
            result['reason'] = 'road_or_lane_section_unavailable'
            return result
        old_node = self._lane_node(old_road, old_section, old_lane)
        new_node = self._lane_node(new_road, new_section, new_lane)
        if any(node is None or node.get('type') != 'driving' for node in (old_node, new_node)):
            result['reason'] = 'driving_lane_unavailable'
            return result
        if old_road == new_road:
            mapped, chain, error = self._follow_sections(old_road, old_section, new_section, old_lane)
            result.update(mapped_old_lane=mapped, lane_link_chain=chain)
            if error:
                result['reason'] = error
                return result
            result.update(classification='continuity' if mapped == new_lane else 'actual',
                          reason='same_lane' if mapped == new_lane and not chain else
                          'lane_section_link_continuity' if mapped == new_lane else 'different_driving_lane')
            return result

        candidates = []
        old_sections = self.roads[old_road].findall('lanes/laneSection')
        new_sections = self.roads[new_road].findall('lanes/laneSection')
        for end in self.roads[old_road].findall('link/*'):
            if end.tag not in {'predecessor', 'successor'}:
                continue
            exit_section = 0 if end.tag == 'predecessor' else len(old_sections) - 1
            exit_lane, prefix, error = self._follow_sections(old_road, old_section, exit_section, old_lane)
            if error:
                continue
            links = []
            if end.get('elementType') == 'road' and int(end.get('elementId')) == new_road:
                lane = self._lane_node(old_road, exit_section, exit_lane)
                for link in lane.findall('link/' + end.tag):
                    links.append((int(link.get('id')), end.get('contactPoint'), 'road_lane_link'))
            elif end.get('elementType') == 'junction':
                junction = self.junctions.get(int(end.get('elementId')))
                for connection in [] if junction is None else junction.findall('connection'):
                    if (int(connection.get('incomingRoad')) != old_road or
                            int(connection.get('connectingRoad')) != new_road):
                        continue
                    for link in connection.findall('laneLink'):
                        if int(link.get('from')) == exit_lane:
                            links.append((int(link.get('to')), connection.get('contactPoint'), 'junction_lane_link'))
            for target, contact, source in links:
                if contact not in {'start', 'end'}:
                    continue
                entry_section = 0 if contact == 'start' else len(new_sections) - 1
                entry = self._lane_node(new_road, entry_section, target)
                if entry is None or entry.get('type') != 'driving':
                    continue
                inverse = entry.findall('link/' + ('predecessor' if contact == 'start' else 'successor'))
                if inverse and (len(inverse) != 1 or int(inverse[0].get('id')) != exit_lane):
                    continue
                mapped, suffix, error = self._follow_sections(new_road, entry_section, new_section, target)
                if error:
                    continue
                cross = dict(from_road_id=old_road, to_road_id=new_road,
                             from_section=exit_section, to_section=entry_section,
                             from_lane=exit_lane, to_lane=target, link=source,
                             contact_point=contact)
                candidates.append((mapped, prefix + [cross] + suffix))
        if len(candidates) != 1:
            if not candidates:
                replacement = self._overlapping_connector_identity(
                    before_road, after_road, old_section, new_section)
                if replacement is not None:
                    result.update(replacement)
                    return result
            result['reason'] = 'road_lane_link_missing' if not candidates else 'ambiguous_road_lane_link'
            return result
        mapped, chain = candidates[0]
        result.update(mapped_old_lane=mapped, lane_link_chain=chain)
        if mapped == new_lane:
            result.update(classification='continuity', reason='road_lane_link_continuity')
        else:
            result['reason'] = 'observed_lane_differs_from_road_link'
        return result

    def _overlapping_connector_identity(self, before, after, old_section, new_section):
        """Recognise a GT road-match replacement, not every common-origin fork.

        A shared incoming lane alone is insufficient: both observed world
        positions and their short sweep must already lie inside the new lane.
        Disjoint branches and sweeps crossing a boundary remain unknown.
        """
        old_id, new_id = int(before['road']), int(after['road'])
        old_junction = int(self.roads[old_id].get('junction', '-1'))
        if old_junction < 0 or old_junction != int(self.roads[new_id].get('junction', '-1')):
            return None
        junction = self.junctions.get(old_junction)
        if junction is None:
            return None
        origins = []
        for rid, section, lane_id in ((old_id, old_section, int(before['lane'])),
                                     (new_id, new_section, int(after['lane']))):
            entries = set()
            for connection in junction.findall('connection'):
                if int(connection.get('connectingRoad')) != rid:
                    continue
                contact = connection.get('contactPoint')
                if contact not in {'start', 'end'}:
                    continue
                entry_section = 0 if contact == 'start' else len(self.roads[rid].findall('lanes/laneSection')) - 1
                entry_lane, _, error = self._follow_sections(rid, section, entry_section, lane_id)
                if error:
                    continue
                entries.update((int(connection.get('incomingRoad')), int(link.get('from')))
                               for link in connection.findall('laneLink')
                               if int(link.get('to')) == entry_lane)
            origins.append(entries)
        shared = origins[0] & origins[1]
        if len(shared) != 1:
            return None
        try:
            stations = [float(before['s']), float(after['s'])]
            offsets = [float(before['t']), float(after['t'])]
            if not all(math.isfinite(value) for value in stations + offsets):
                return None
            points = [self._lateral_world(rid, station, offset)
                      for rid, station, offset in zip((old_id, new_id), stations, offsets)]
        except (TypeError, ValueError, KeyError):
            return None
        travel = math.dist(*points)
        if travel > 2.0:
            return None
        # No extrapolation and a 5 cm interior tolerance avoid declaring a
        # boundary-touching or discretisation-sensitive crossing a continuity.
        margins = []
        count = max(1, math.ceil(travel / .2))
        for i in range(count + 1):
            point = tuple(a + (b - a) * i / count for a, b in zip(*points))
            projected = self.project(new_id, point, stations[1])
            if projected is None:
                return None
            ps, pt = projected
            data = self.lanes_at(new_id, ps)
            lane = data and data['lanes'].get(int(after['lane']))
            if not lane or lane['type'] != 'driving':
                return None
            lo, hi = sorted((lane['inner'], lane['outer']))
            margins.append(min(pt - lo, hi - pt))
        if min(margins) < .05:
            return None
        # Roads in the same junction must also describe the same elevation;
        # a topology error must not equate vertically separated surfaces.
        heights = [self.elevation(rid, station) for rid, station in zip((old_id, new_id), stations)]
        if all(height is not None for height in heights) and abs(heights[0] - heights[1]) > .25:
            return None
        return dict(classification='continuity', mapped_old_lane=int(after['lane']),
                    reason='overlapping_connector_gt_match_replacement',
                    source='xodr_shared_incoming_lane_and_world_sweep',
                    lane_link_chain=[dict(junction_id=old_junction, incoming_road=next(iter(shared))[0],
                                          incoming_lane=next(iter(shared))[1],
                                          from_road=old_id, to_road=new_id,
                                          world_points=points, sweep_length_m=travel,
                                          minimum_new_lane_interior_margin_m=min(margins))])

    def elevation(self, road_id, s):
        road = self.roads.get(road_id)
        node = None if road is None else active(road.findall('elevationProfile/elevation'), s, 's')
        return None if node is None else polynomial(node, s - number(node, 's'))

    def _connected_surface_candidates(self, road_pos, corners):
        """Only explicit topological continuations, never nearby parallel roads."""
        rid = int(road_pos['road'])
        road = self.roads[rid]
        identifiers = set()
        for link in road.findall('link/*'):
            if link.get('elementType') == 'road':
                identifiers.add(int(link.get('elementId')))
            elif link.get('elementType') == 'junction':
                junction = self.junctions.get(int(link.get('elementId')))
                if junction is not None:
                    identifiers.update(int(c.get('connectingRoad')) for c in junction.findall('connection')
                                       if int(c.get('incomingRoad')) == rid)
        # A road may declare the link only in the inverse direction.
        for other_id, other in self.roads.items():
            if any(link.get('elementType') == 'road' and int(link.get('elementId')) == rid
                   for link in other.findall('link/*')):
                identifiers.add(other_id)
        center = tuple(sum(point[i] for point in corners) / len(corners) for i in (0, 1))
        result = []
        for other_id in sorted(identifiers - {rid}):
            points = self.reference(other_id)
            if not points:
                continue
            nearest = min(points, key=lambda p: math.hypot(p[1] - center[0], p[2] - center[1]))
            if math.hypot(nearest[1] - center[0], nearest[2] - center[1]) > 30:
                continue
            station = nearest[0]
            data = self.lanes_at(other_id, station)
            for lane_id, lane in (data or {}).get('lanes', {}).items():
                if lane['type'] != 'driving':
                    continue
                other_pos = dict(road=other_id, lane=lane_id, s=station)
                forward = self.classify_lane_transition(road_pos, other_pos)
                reverse = self.classify_lane_transition(other_pos, road_pos)
                if forward['classification'] == 'continuity' or reverse['classification'] == 'continuity':
                    result.append((other_id, station, 1 if lane_id > 0 else -1))
        return list(dict.fromkeys(result))

    def _measure_connected(self, ego, road_pos, expected_side, footprint_samples, corners, original):
        """Join finite linked patches when a footprint straddles a road end.

        Coverage requires *every* footprint edge sample to have a topologically
        connected finite road surface. Missing/contradictory definitions remain
        unavailable; this does not extend a road infinitely or fill a junction.
        """
        rid, station = road_pos['road'], road_pos['s']
        candidates = [(rid, station, expected_side)] + self._connected_surface_candidates(road_pos, corners)
        assigned = {}
        for point in footprint_samples:
            choices = []
            for candidate, hint, side in candidates:
                projected = self.project(candidate, point, hint)
                if projected is None:
                    continue
                ps, pt = projected
                data = self.lanes_at(candidate, ps)
                if data is None:
                    continue
                height = self.elevation(candidate, ps)
                reference_height = self.elevation(rid, station)
                # Comparing linked map surfaces avoids treating RDB z offsets
                # (different model reference points) as ground elevations.
                if height is not None and reference_height is not None and abs(height - reference_height) > 1.0:
                    continue
                edges = [v for lane in data['lanes'].values() for v in (lane['inner'], lane['outer'])]
                if candidate != rid and (not edges or not min(edges) - 1e-6 <= pt <= max(edges) + 1e-6):
                    continue
                choices.append((candidate, side, ps, pt, data))
                if candidate == rid:
                    break
            if len(choices) != 1:
                original['boundary_connected_candidate_count'] = len(candidates) - 1
                return original
            candidate, side, ps, pt, data = choices[0]
            assigned.setdefault((candidate, side), []).append((ps, pt, data))
        center_depths, sidewalk_depths, center_valid, sidewalk_valid = [], [], True, True
        coverage_patches = []
        for (candidate, side), values in assigned.items():
            start, end = min(x[0] for x in values), max(x[0] for x in values)
            # Include the full finite overlap: edge samples on either side of
            # a seam need a strip reaching the actual endpoint, not a gap.
            projections = [self.project(candidate, point, hint) for point in corners
                           for cid, hint, _ in candidates if cid == candidate]
            projected_stations = [x[0] for x in projections if x is not None]
            if any(x is None for x in projections):
                length = number(self.roads[candidate], 'length')
                if start < 6:
                    start = 0.0
                if length - end < 6:
                    end = length
            if projected_stations:
                start, end = min(start, *projected_stations), max(end, *projected_stations)
            cuts = self._boundary_cuts(candidate, start, end)
            definitions = [x[2] for x in values]
            for first, second in zip(cuts, cuts[1:]):
                if second - first < 1e-9:
                    continue
                middle = self.lanes_at(candidate, (first + second) / 2)
                before = self.lanes_at(candidate, first + 1e-9)
                after = self.lanes_at(candidate, second - 1e-9)
                if any(x is None for x in (middle, before, after)):
                    return original
                definitions.append(middle)
                limits = [[value for lane in data['lanes'].values() for value in (lane['inner'], lane['outer'])]
                          for data in (before, after)]
                if all(limits):
                    a, b = (self._lateral_world(candidate, first, min(limits[0])),
                            self._lateral_world(candidate, second, min(limits[1])))
                    c, d = (self._lateral_world(candidate, second, max(limits[1])),
                            self._lateral_world(candidate, first, max(limits[0])))
                    coverage_patches.extend(([a, b, c], [a, c, d]))
                for lane_id, lane in middle['lanes'].items():
                    if lane['type'] == 'sidewalk' and lane_id in before['lanes'] and lane_id in after['lanes']:
                        left, right = before['lanes'][lane_id], after['lanes'][lane_id]
                        sidewalk_depths.append(finite_strip_intrusion(
                            corners, self._lateral_world(candidate, first, left['inner']),
                            self._lateral_world(candidate, second, right['inner']),
                            self._lateral_world(candidate, first, left['outer']),
                            self._lateral_world(candidate, second, right['outer'])))
            center_valid &= side in (-1, 1) and all(self._has_center_divider(data) for data in definitions)
            sidewalk_valid &= all(any(lane['type'] == 'sidewalk' for lane in data['lanes'].values())
                                  for data in definitions)
            if side in (-1, 1):
                center_depths.extend(max(0., -side * (pt - data['offset'])) for _, pt, data in values)
        if not finite_patches_cover(corners, coverage_patches):
            original['centerline_measurement_reason'] = 'gap_in_connected_finite_road_surfaces'
            original['sidewalk_measurement_reason'] = 'gap_in_connected_finite_road_surfaces'
            return original
        original.update(boundary_source='xodr_finite_linked_road_patches',
                        boundary_patch_road_ids=sorted({key[0] for key in assigned}))
        if center_valid and center_depths:
            original.update(centerline_intrusion_m=max(center_depths), centerline_measurement_status='AVAILABLE',
                            centerline_measurement_reason='explicit_connected_center_dividers')
        else:
            original['centerline_measurement_reason'] = 'connected_center_divider_definition_partial'
        if sidewalk_valid:
            original.update(sidewalk_intrusion_m=max(sidewalk_depths, default=0.),
                            sidewalk_measurement_status='AVAILABLE',
                            sidewalk_measurement_reason='explicit_connected_finite_sidewalk_polygons')
        else:
            original['sidewalk_measurement_reason'] = 'connected_sidewalk_definition_partial'
        return original

    def lanes_at(self,road_id,s,section_index=None):
        road=self.roads.get(road_id)
        if road is None or not 0 <= s <= number(road, 'length') + 1e-8:
            return None
        sections=road.findall('lanes/laneSection')
        sec=active(sections,s,'s') if section_index is None else sections[section_index]
        if sec is None: return None
        ds=s-number(sec,'s')
        offset_node=active(road.findall('lanes/laneOffset'),s,'s')
        offset=polynomial(offset_node,s-number(offset_node,'s')) if offset_node is not None else 0
        lanes={}
        for side,sign in (('left',1),('right',-1)):
            edge=offset
            for lane in sorted(sec.findall(side+'/lane'),key=lambda l:abs(int(l.get('id')))):
                width_node=active(lane.findall('width'),ds,'sOffset')
                if width_node is None: return None
                width=max(0,polynomial(width_node,ds-number(width_node,'sOffset')))
                outer=edge+sign*width
                mark=active(lane.findall('roadMark'),ds,'sOffset')
                lanes[int(lane.get('id'))]=dict(inner=edge,outer=outer,type=lane.get('type'),
                                               mark=mark.get('type','none') if mark is not None else 'none',
                                               mark_color=mark.get('color','standard') if mark is not None else None)
                edge=outer
        center=sec.find('center/lane')
        mark=active(center.findall('roadMark'),ds,'sOffset') if center is not None else None
        return dict(lanes=lanes,offset=offset,center_mark=mark.get('type','none') if mark is not None else 'none',
                    center_mark_color=mark.get('color','standard') if mark is not None else None)

    def _boundary_cuts(self, road_id, start, end):
        """Sample within the road and split at every feature definition change."""
        road = self.roads[road_id]
        cuts = {start, end}
        cuts.update(s for s in self.sample_stations[road_id] if start < s < end)
        for section in road.findall('lanes/laneSection'):
            base = number(section, 's')
            if start < base < end:
                cuts.add(base)
            for element in section.findall('.//width') + section.findall('.//roadMark'):
                station = base + number(element, 'sOffset')
                if start < station < end:
                    cuts.add(station)
        for element in road.findall('lanes/laneOffset'):
            station = number(element, 's')
            if start < station < end:
                cuts.add(station)
        return sorted(cuts)

    def _lateral_world(self, road_id, s, lateral):
        x, y, heading = self.reference_pose(road_id, s)
        return x - math.sin(heading) * lateral, y + math.cos(heading) * lateral

    @staticmethod
    def _has_center_divider(data):
        mark = data['center_mark']
        if mark == 'none':
            return False
        two_sided = (any(i < 0 and lane['type'] == 'driving' for i, lane in data['lanes'].items())
                     and any(i > 0 and lane['type'] == 'driving' for i, lane in data['lanes'].items()))
        # A yellow solid divider is explicit even when an individual junction
        # connector describes only the lanes on one side of that divider.
        explicit_yellow = data['center_mark_color'] == 'yellow' and mark in {'solid', 'solid solid'}
        return two_sided or explicit_yellow

    def measure(self,ego,road_pos,expected_side):
        """Measure only explicit, finite map boundaries, including junctions.

        A missing sidewalk or central divider is unknown, not a zero-depth
        observation. Junction connector reference lines are not automatically
        promoted into central dividers. Sidewalk depth is limited to the actual
        width-defined polygon and is never an unbounded half-plane.
        """
        result = dict(boundary_source='xodr_explicit_road_boundaries',
                      centerline_measurement_status='UNAVAILABLE',
                      sidewalk_measurement_status='UNAVAILABLE',
                      centerline_measurement_reason='road_position_unavailable',
                      sidewalk_measurement_reason='road_position_unavailable')
        def unavailable(reason):
            result['centerline_measurement_reason'] = reason
            result['sidewalk_measurement_reason'] = reason
            return result
        if not road_pos:
            return result
        rid, s = road_pos.get('road'), road_pos.get('s')
        if rid not in self.roads:
            return unavailable('road_not_in_xodr')
        if not isinstance(s, (int, float)) or self._section_index(rid, s) is None:
            return unavailable('road_station_outside_defined_domain')
        if not self.reference(rid):
            return unavailable('unsupported_reference_geometry')
        corners = obb(ego)
        # Sampling the footprint edges also detects the maximum on a curved
        # reference, which can lie between the four OBB corners.
        footprint_samples = []
        for a, b in zip(corners, corners[1:] + corners[:1]):
            count = max(1, math.ceil(math.dist(a, b) / .5))
            footprint_samples.extend((a[0] + (b[0] - a[0]) * i / count,
                                      a[1] + (b[1] - a[1]) * i / count) for i in range(count))
        projections = [self.project(rid, point, s) for point in footprint_samples]
        if any(point is None for point in projections):
            return self._measure_connected(ego, road_pos, expected_side, footprint_samples, corners,
                                           unavailable('footprint_outside_defined_road_geometry'))
        samples = [(ps, pt, self.lanes_at(rid, ps)) for ps, pt in projections]
        if any(data is None for _, _, data in samples):
            return unavailable('lane_width_definition_unavailable')
        start, end = min(ps for ps, _, _ in samples), max(ps for ps, _, _ in samples)
        cuts = self._boundary_cuts(rid, start, end)
        strips = []
        for first, second in zip(cuts, cuts[1:]):
            if second - first < 1e-9:
                continue
            midpoint = (first + second) / 2
            section = self._section_index(rid, midpoint)
            middle = self.lanes_at(rid, midpoint, section)
            # One-sided evaluation keeps a width/section discontinuity from
            # replacing the preceding finite patch by the next patch's width.
            eps = min(1e-8, (second - first) / 4)
            before = self.lanes_at(rid, first + eps, section)
            after = self.lanes_at(rid, second - eps, section)
            if any(data is None for data in (middle, before, after)):
                return unavailable('lane_width_definition_unavailable')
            strips.append((first, second, middle, before, after))
        definitions = [data for _, _, data in samples] + [patch[2] for patch in strips]
        if expected_side not in (-1, 1):
            result['centerline_measurement_reason'] = 'travel_side_unavailable'
        elif all(self._has_center_divider(data) for data in definitions):
            result.update(centerline_intrusion_m=max(0.0, max(-expected_side * (pt - data['offset'])
                                                           for _, pt, data in samples)),
                          centerline_measurement_status='AVAILABLE',
                          centerline_measurement_reason='explicit_center_divider')
        else:
            result['centerline_measurement_reason'] = (
                'center_divider_definition_partial' if any(self._has_center_divider(data) for data in definitions)
                else 'center_divider_not_defined')
        has_sidewalk = lambda data: any(lane['type'] == 'sidewalk' for lane in data['lanes'].values())
        if not all(has_sidewalk(data) for data in definitions):
            result['sidewalk_measurement_reason'] = (
                'sidewalk_definition_partial' if any(has_sidewalk(data) for data in definitions)
                else 'sidewalk_not_defined')
        else:
            depth = 0.0
            for first, second, middle, before, after in strips:
                for lane_id, lane in middle['lanes'].items():
                    if lane['type'] != 'sidewalk':
                        continue
                    left, right = before['lanes'][lane_id], after['lanes'][lane_id]
                    depth = max(depth, finite_strip_intrusion(
                        corners,
                        self._lateral_world(rid, first, left['inner']),
                        self._lateral_world(rid, second, right['inner']),
                        self._lateral_world(rid, first, left['outer']),
                        self._lateral_world(rid, second, right['outer']),
                    ))
            result.update(sidewalk_intrusion_m=depth,
                          sidewalk_measurement_status='AVAILABLE',
                          sidewalk_measurement_reason='explicit_finite_sidewalk_polygon')
        depths = []
        for _, pt, data in samples:
            lane = data['lanes'].get(road_pos.get('lane'))
            if lane:
                lo, hi = sorted((lane['inner'], lane['outer']))
                depths.append(max(0.0, lo - pt, pt - hi))
        if len(depths) == len(samples):
            result['lane_intrusion_m'] = max(depths)
        return result

    def boundary_type(self,road_id,s,old_lane,new_lane):
        if road_id not in self.roads: return None
        if old_lane*new_lane<=0 or abs(old_lane-new_lane)!=1: return None
        data=self.lanes_at(road_id,s)
        if data is None: return None
        lane=data['lanes'].get(min((old_lane,new_lane),key=abs))
        return lane['mark'] if lane else None

    def boundary_contact(self,before,after,road_id,s,old_lane,new_lane):
        """OBB sweep across the shared boundary, not the reference point alone."""
        if road_id not in self.roads or old_lane*new_lane<=0 or abs(old_lane-new_lane)!=1:
            return None
        inner_lane=min((old_lane,new_lane),key=abs)
        distances=[]
        for point in obb(before)+obb(after):
            projected=self.project(road_id,point,s)
            if projected is None: return None
            ps,pt=projected
            data=self.lanes_at(road_id,ps)
            lane=data and data['lanes'].get(inner_lane)
            if lane is None: return None
            distances.append(pt-lane['outer'])
        return min(distances)<=0<=max(distances)
