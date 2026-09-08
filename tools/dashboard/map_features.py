"""Extract finite stop lines and zebra polygons, not crosswalk warning diamonds.

Finish candidates are proposals only; choosing the final stop line is a scenario
configuration decision, not something the map's object name can prove.
"""
import bisect
import math
from assessment_geometry import obb


def road_pose(map_data, rid, s):
    points = map_data.reference(rid)
    if not points:
        raise ValueError(f'unsupported road geometry: {rid}')
    i = min(len(points)-2, max(0, bisect.bisect_right([p[0] for p in points], s)-1))
    a, b = points[i:i+2]
    if b[0] <= a[0]:
        raise ValueError(f'degenerate road sample: {rid}')
    ratio = (s-a[0])/(b[0]-a[0])
    return a[1]+ratio*(b[1]-a[1]), a[2]+ratio*(b[2]-a[2]), math.atan2(b[2]-a[2],b[1]-a[1])


def extract_features(map_data):
    stop_lines, crosswalks = [], []
    for rid, road in map_data.roads.items():
        for node in road.findall('objects/object'):
            name = node.get('name', '')
            stop = 'StopLine' in name
            zebra = node.get('type') == 'crosswalk' or name.startswith('RM_532_')
            if not (stop or zebra):
                continue
            s, t, length, width, hdg = (float(node.get(k,0)) for k in ('s','t','length','width','hdg'))
            if length <= 0 or width <= 0:
                continue
            x,y,h = road_pose(map_data,rid,s)
            x,y = x-math.sin(h)*t, y+math.cos(h)*t
            heading = h+hdg
            base = dict(id=f'{rid}:{node.get("id")}',object_id=node.get('id'),road_id=rid,s=s,t=t,source=name)
            if zebra:
                base['polygon'] = obb(dict(x=x,y=y,h=heading,length=length,width=width))
                crosswalks.append(base)
            else:
                # Width runs across the lane, length is marking thickness.
                dx,dy = -math.sin(heading)*width/2, math.cos(heading)*width/2
                base.update(begin=[x+dx,y+dy],end=[x-dx,y-dy],direction=1)
                lanes=map_data.lanes_at(rid,s)
                base['lane_ids']=[lid for lid,lane in (lanes or {}).get('lanes',{}).items()
                                  if lane['type']=='driving' and min(lane['inner'],lane['outer']) <= t <= max(lane['inner'],lane['outer'])]
                stop_lines.append(base)
    return stop_lines,crosswalks


def finish_candidates(stop_lines, route):
    if not route:
        return []
    goal=route.points[-1]
    result=[]
    for line in stop_lines:
        if line['road_id'] != goal['road'] or goal['lane'] not in line['lane_ids']:
            continue
        x=(line['begin'][0]+line['end'][0])/2
        y=(line['begin'][1]+line['end'][1])/2
        distance=math.hypot(x-goal['x'],y-goal['y'])
        if distance <= 30:
            result.append({**line,'distance_from_route_end_m':distance})
    return sorted(result,key=lambda item:item['distance_from_route_end_m'])


def finish_from_object(stop_lines, selection):
    """Explicit object selection; merge only collinear markings of same direction."""
    chosen=next((line for line in stop_lines if line['road_id']==selection['road_id'] and
                 line['object_id']==str(selection['object_id'])),None)
    if not chosen:
        raise ValueError('finish_stop_line object was not found in selected XODR')
    result=dict(chosen)
    if selection.get('whole_carriageway',False):
        a,b=chosen['begin'],chosen['end']
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        dx,dy=dx/length,dy/length
        points=[]
        for line in stop_lines:
            if line['road_id']!=chosen['road_id'] or line['t']*chosen['t']<=0 or abs(line['s']-chosen['s'])>.1:
                continue
            for x,y in (line['begin'],line['end']):
                if abs((x-a[0])*(-dy)+(y-a[1])*dx) <= .1:
                    points.append((x-a[0])*dx+(y-a[1])*dy)
        if points:
            result['begin']=[a[0]+dx*min(points),a[1]+dy*min(points)]
            result['end']=[a[0]+dx*max(points),a[1]+dy*max(points)]
            result.pop('lane_ids',None)
    result['direction']=selection.get('direction',chosen['direction'])
    return result
