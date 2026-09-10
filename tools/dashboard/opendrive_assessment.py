"""Local lane boundary measurements from the currently selected OpenDRIVE map.

Reference curves are tessellated at <=0.20 m, lazily per road. No scenario IDs.
Unsupported geometry and corners beyond a road end are reported as unavailable.
"""
from __future__ import annotations
import bisect
import math
import xml.etree.ElementTree as ET
from assessment_geometry import obb


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

    def reference(self,road_id):
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
        return points

    def project(self,road_id,point,s_hint):
        points=self.reference(road_id)
        if not points:
            return None
        ss=[p[0] for p in points]
        start=max(0,bisect.bisect_left(ss,s_hint-12)-1)
        end=min(len(points)-1,bisect.bisect_right(ss,s_hint+12))
        best=None
        for i in range(start,end):
            s0,x,y=points[i]; s1,bx,by=points[i+1]
            dx,dy=bx-x,by-y
            length2=dx*dx+dy*dy
            if length2<1e-12: continue
            alpha=((point[0]-x)*dx+(point[1]-y)*dy)/length2
            if (i==0 and alpha<0) or (i==len(points)-2 and alpha>1): continue
            alpha=max(0,min(1,alpha))
            px,py=x+alpha*dx,y+alpha*dy
            distance=(point[0]-px)**2+(point[1]-py)**2
            t=((point[0]-px)*(-dy)+(point[1]-py)*dx)/math.sqrt(length2)
            if best is None or distance<best[0]: best=(distance,s0+alpha*(s1-s0),t)
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
            ss = [point[0] for point in points]
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

    def lanes_at(self,road_id,s):
        road=self.roads[road_id]
        sec=active(road.findall('lanes/laneSection'),s,'s')
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
                                               mark=mark.get('type','none') if mark is not None else 'none')
                edge=outer
        center=sec.find('center/lane')
        mark=active(center.findall('roadMark'),ds,'sOffset') if center is not None else None
        return dict(lanes=lanes,offset=offset,center_mark=mark.get('type','none') if mark is not None else 'none')

    def measure(self,ego,road_pos,expected_side):
        rid=road_pos['road']; s=road_pos['s']
        if rid not in self.roads or self.roads[rid].get('junction','-1')!='-1': return {}
        center=sidewalk=lane_depth=0.0
        all_center=True
        for point in obb(ego):
            projected=self.project(rid,point,s)
            if projected is None: return {}
            ps,pt=projected
            data=self.lanes_at(rid,ps)
            if data is None: return {}
            if data['center_mark']=='none' or not any(i<0 and l['type']=='driving' for i,l in data['lanes'].items()) or not any(i>0 and l['type']=='driving' for i,l in data['lanes'].items()):
                all_center=False
            center=max(center,-expected_side*(pt-data['offset']))
            lane=data['lanes'].get(road_pos['lane'])
            if lane:
                lo,hi=sorted((lane['inner'],lane['outer']))
                lane_depth=max(lane_depth,lo-pt,pt-hi)
            for lid,lane in data['lanes'].items():
                if lane['type']=='sidewalk':
                    sidewalk=max(sidewalk,(pt-lane['inner'])*(1 if lid>0 else -1))
        result=dict(sidewalk_intrusion_m=max(0,sidewalk),lane_intrusion_m=max(0,lane_depth))
        if all_center: result['centerline_intrusion_m']=max(0,center)
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
