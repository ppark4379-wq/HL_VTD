"""Geometry in world metres. All scenario boundaries are data, never object IDs in code."""
from __future__ import annotations
import math


def local_point(obj, x=0.0, y=0.0):
    c, s = math.cos(obj['h']), math.sin(obj['h'])
    return obj['x'] + c*x - s*y, obj['y'] + s*x + c*y


def obb(obj):
    return [local_point(obj, obj.get('off_x', 0) + x*obj['length']/2,
                        obj.get('off_y', 0) + y*obj['width']/2)
            for x, y in ((-1,-1), (1,-1), (1,1), (-1,1))]


def overlaps(a, b):
    for poly in (a, b):
        for i, p in enumerate(poly):
            q = poly[(i+1) % len(poly)]
            axis = (p[1]-q[1], q[0]-p[0])
            pa = [x*axis[0]+y*axis[1] for x,y in a]
            pb = [x*axis[0]+y*axis[1] for x,y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def contains(polygon, point):
    x,y = point
    inside = False
    for i,(ax,ay) in enumerate(polygon):
        bx,by = polygon[(i+1) % len(polygon)]
        cross = (bx-ax)*(y-ay)-(by-ay)*(x-ax)
        if abs(cross) < 1e-8 and min(ax,bx)-1e-8 <= x <= max(ax,bx)+1e-8 and min(ay,by)-1e-8 <= y <= max(ay,by)+1e-8:
            return True
        if (ay>y) != (by>y) and x < (bx-ax)*(y-ay)/(by-ay)+ax:
            inside = not inside
    return inside


def signed_distance(line, point):
    a,b = line['begin'], line['end']
    dx,dy = b[0]-a[0],b[1]-a[1]
    length = math.hypot(dx,dy)
    if length <= 0:
        raise ValueError('boundary endpoints must differ')
    return ((point[0]-a[0])*(-dy)+(point[1]-a[1])*dx)/length


def crosses(line, before, after):
    """Directed finite line crossing; normal points into the reached side."""
    d0,d1 = signed_distance(line,before), signed_distance(line,after)
    direction = line.get('direction',1)
    if not d0*direction < 0 <= d1*direction:
        return False
    alpha = -d0/(d1-d0)
    point = (before[0]+alpha*(after[0]-before[0]), before[1]+alpha*(after[1]-before[1]))
    a,b = line['begin'],line['end']
    dx,dy = b[0]-a[0],b[1]-a[1]
    along = ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)
    return 0 <= along <= 1


def intrusion(line, corners):
    a,b = line['begin'],line['end']
    dx,dy = b[0]-a[0],b[1]-a[1]
    length2 = dx*dx+dy*dy
    along = [((x-a[0])*dx+(y-a[1])*dy)/length2 for x,y in corners]
    if max(along) < 0 or min(along) > 1:
        return 0.0
    return max(0.0, max(signed_distance(line,p)*line.get('forbidden_side',1) for p in corners))


def relevant(feature, road):
    if 'road_id' in feature and (not road or int(feature['road_id']) != road['road']):
        return False
    if feature.get('lane_ids') and (not road or road['lane'] not in feature['lane_ids']):
        return False
    return True


def _points_ok(points):
    return all(
        isinstance(point, (list, tuple))
        and len(point) == 2
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in point
        )
        for point in points
    )


def validate_metadata(meta):
    if not isinstance(meta,dict):
        raise ValueError('metadata must be a mapping')
    if meta.get('rear_axle_offset_m') is not None and (not isinstance(meta['rear_axle_offset_m'],(int,float)) or not math.isfinite(meta['rear_axle_offset_m'])):
        raise ValueError('rear_axle_offset_m must be finite')
    for key in ('sections','school_zones','crosswalks','stop_lines','centerlines','sidewalk_boundaries','lane_boundaries'):
        features = meta.get(key, [])
        if not isinstance(features,list):
            raise ValueError(f'metadata.{key} must be a list')
        ids = set()
        for feature in features:
            if not isinstance(feature,dict):
                raise ValueError(f'{key}: each feature must be a mapping')
            ident = str(feature.get('id',''))
            if not ident or ident in ids:
                raise ValueError(f'{key}: IDs must be present and unique')
            ids.add(ident)
            if key in ('sections','school_zones','crosswalks'):
                points = feature.get('polygon',[])
                if len(points)<3:
                    raise ValueError(f'{key}.{ident}: polygon needs >=3 vertices')
            else:
                points = [feature.get('begin',[]),feature.get('end',[])]
                if points[0] == points[1]:
                    raise ValueError(f'{key}.{ident}: endpoints must differ')
            if not _points_ok(points):
                raise ValueError(f'{key}.{ident}: finite [x,y] coordinates required')
            for name in ('direction','forbidden_side'):
                if feature.get(name,1) not in (-1,1):
                    raise ValueError(f'{key}.{ident}: {name} must be -1 or 1')
            if key in ('sections','school_zones','crosswalks'):
                area = sum(p[0]*points[(i+1)%len(points)][1]-p[1]*points[(i+1)%len(points)][0] for i,p in enumerate(points))
                if abs(area)<1e-8:
                    raise ValueError(f'{key}.{ident}: polygon has zero area')
                if key=='crosswalks':
                    turns = []
                    for i,a in enumerate(points):
                        b,c=points[(i+1)%len(points)],points[(i+2)%len(points)]
                        turn=(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
                        if abs(turn)>1e-8: turns.append(turn>0)
                    if len(set(turns))>1:
                        raise ValueError(f'{key}.{ident}: use convex crosswalk polygons (split concave shapes)')
            if key=='stop_lines':
                if not isinstance(feature.get('signal_id'),int):
                    raise ValueError(f'{key}.{ident}: integer signal_id required')
                masks=feature.get('state_masks',{})
                if not isinstance(masks,dict) or any(v not in {'RED','GREEN','YELLOW','FLASHING_RED'} for v in masks.values()):
                    raise ValueError(f'{key}.{ident}: invalid state_masks')
    finish = meta.get('finish_line')
    if finish:
        if not isinstance(finish,dict) or not _points_ok([finish.get('begin'),finish.get('end')]) or finish.get('direction',1) not in (-1,1):
            raise ValueError('finish_line requires finite begin/end and direction -1 or 1')
        signed_distance(finish, finish['begin'])
