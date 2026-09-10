"""Compare saved lane bands with the source map polygons."""
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QPainterPath, QPolygonF

ROOT = Path(__file__).resolve().parents[2]

def path(xy):
    p = QPainterPath()
    p.addPolygon(QPolygonF([QPointF(float(x), float(y)) for x, y in xy]))
    p.closeSubpath()
    return p

def area(p):
    total = 0.
    for poly in p.toFillPolygons():
        a = np.array([[v.x(), v.y()] for v in poly])
        total += abs(np.sum(a[:, 0]*np.roll(a[:, 1], -1)-a[:, 1]*np.roll(a[:, 0], -1)))/2
    return total

def main():
    base = ROOT/'output/콜리더_게이트웨이'
    ns = '{http://www.w3.org/2000/svg}'
    shapes = {}
    for el in ET.parse(base/'road_map.svg').getroot().find(ns+'g'):
        title = el.find(ns+'title')
        if title is None: continue
        m = re.fullmatch(r'road (\d+) section (\d+) lane (-?\d+) (\w+)', title.text)
        if m:
            shapes[tuple(map(int,m.groups()[:3]))] = (m[4],path([list(map(float,p.split(','))) for p in el.get('points').split()]))
    geo = {tuple(g['key']): np.array(g['points']) for g in json.loads((base/'lane_geometry.json').read_text())}
    sidewalks=[]
    for kind, shape in shapes.values():
        if kind=='sidewalk': sidewalks.append(shape)
    from render_mission_corridors import draw,load_background,png
    from generate_random_corridors import create_app
    app=create_app()
    reports=[]
    for name in ['A70','A74','A119','A120']:
        mission=json.loads((base/'자동생성_20개'/name/'complete_network.json').read_text())
        keys={tuple(k) for leg in mission['legs'] for k in leg['allowed']}
        print(name)
        report = dict(scenario=name, original_sidewalk_overlap_m2=0., rendered_sidewalk_overlap_m2=0.)
        for k in sorted(keys):
            p=geo[k]; xy=np.concatenate([p[:,5:7],p[::-1,7:9]])*[1,-1]
            a=path(xy)
            if k not in shapes: print(k,'missing');continue
            diff=area(a.subtracted(shapes[k][1]))
            if diff>.05: print(k,shapes[k][0],'outside own driving polygon',round(diff,3))
            overlap=sum(area(a.intersected(b)) for b in sidewalks if a.boundingRect().intersects(b.boundingRect()))
            report['original_sidewalk_overlap_m2'] += overlap
            if overlap>.05: print(k,'source sidewalk overlap',round(overlap,3))

        svg_root=ET.parse(base/'자동생성_20개'/name/'overview.svg').getroot()
        for group in svg_root.findall(ns+'g'):
            if group.get('id') not in ('corridors','gateways','road_boundaries'): continue
            for el in group.iter():
                if el.tag != ns+'path': continue
                shape=QPainterPath()
                for segment in el.get('d','').split('M')[1:]:
                    values=list(map(float,re.findall(r'-?\d+(?:\.\d+)?',segment)))
                    shape.addPath(path(np.array(values).reshape(-1,2)))
                report['rendered_sidewalk_overlap_m2'] += sum(area(shape.intersected(b)) for b in sidewalks if shape.boundingRect().intersects(b.boundingRect()))
        assert report['rendered_sidewalk_overlap_m2'] < .005, report
        reports.append(report)
        for road in [3005,3027,3033,3183]:
            arrays=[p for k,p in geo.items() if k in keys and k[0]==road]
            if not arrays:continue
            xy=np.concatenate([p[:,5:7] for p in arrays]+[p[:,7:9] for p in arrays])
            lo,hi=xy.min(axis=0)-8,xy.max(axis=0)+8
            svg=draw(mission,{k:dict(points=p) for k,p in geo.items()},load_background(base/'road_map.svg'),list(range(len(mission['legs']))),bounds=(lo[0],-hi[1],hi[0]-lo[0],hi[1]-lo[1]))
            out=ROOT/'output/verification/surface_alignment'
            out.mkdir(exist_ok=True)
            (out/f'{name}_R{road}_after.svg').write_text(svg,encoding='utf-8')
            png(svg,out/f'{name}_R{road}_after.png',1200)

    (ROOT/'output/verification/surface_alignment/audit.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
    print(json.dumps(reports,indent=2))

if __name__=='__main__': main()
