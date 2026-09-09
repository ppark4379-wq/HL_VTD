#!/usr/bin/env python3
"""Shared deterministic SVG renderer for complete XODR corridor networks."""
import html
import math
from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
NS = 'http://www.w3.org/2000/svg'
ET.register_namespace('', NS)


def keytext(key):
    return ':'.join(map(str, key))


def pts(a):
    return ' '.join(f'{x:.3f},{-y:.3f}' for x, y in a)


def polygon(a, **attributes):
    attrs = ' '.join(f'{k.replace("_", "-")}="{v}"' for k, v in attributes.items())
    return f'<polygon points="{pts(a)}" {attrs}/>'


def line(a, color, width=.3, dash=''):
    return f'<polyline points="{pts(a)}" fill="none" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'


def text(x, y, label, size, color='white', anchor='middle'):
    return (f'<text x="{x:.3f}" y="{-y:.3f}" fill="{color}" font-family="sans-serif" '
            f'font-size="{size}" text-anchor="{anchor}" stroke="#17212a" stroke-width="{size*.23}" '
            f'paint-order="stroke fill">{html.escape(str(label))}</text>')


def load_background(source=None):
    root = ET.parse(source or ROOT/'output/콜리더_게이트웨이/road_map.svg').getroot()
    shapes = []
    for group in root.findall(f'{{{NS}}}g'):
        role = group.get('id')
        if role not in ('surface', 'markings', 'objects', 'object_labels'):
            continue
        for element in group:
            raw = element.get('points')
            if raw:
                a = np.array([[float(v) for v in pair.split(',')] for pair in raw.split()])
                bb = (a[:, 0].min(), a[:, 1].min(), a[:, 0].max(), a[:, 1].max())
            elif element.get('x') and element.get('y'):
                x, y = float(element.get('x')), float(element.get('y'))
                bb = (x-10, y-10, x+10, y+10)
            else:
                continue
            shapes.append((role, bb, ET.tostring(element, encoding='unicode')))
    return shapes


def near_slice(points, cp, half_length=2.):
    i = int(np.argmin(np.sum((points[:, :2]-[cp['x'], cp['y']])**2, axis=1)))
    station = points[i, 3]
    selected = points[np.abs(points[:, 3]-station) <= half_length]
    return selected if len(selected) >= 2 else points[max(0, i-1):i+2]


def band(points):
    return np.concatenate([points[:, 5:7], points[::-1, 7:9]])


def draw(mission, geometry, background, leg_indices, bounds=None, closeup=False):
    legs = [mission['legs'][i] for i in leg_indices]
    allowed = {tuple(k) for leg in legs for k in leg['allowed']}
    allowed-={tuple(k) for k in mission.get('surface_excluded_lanes',[])}
    if mission.get('surface_clips'):
        geometry=dict(geometry)
        for clip in mission['surface_clips']:
            k=tuple(clip['key']);p=geometry[k]['points']
            selected=p[(p[:,3]>=clip.get('road_s_min',-math.inf)-1e-7)&(p[:,3]<=clip.get('road_s_max',math.inf)+1e-7)]
            if len(selected)==0:allowed.discard(k)
            else:geometry[k]=dict(geometry[k],points=selected)
    checkpoints = mission['checkpoints']
    seqs = {v for leg in legs for v in [leg['from_seq'],leg['entry_seq'],leg['exit_seq']]}
    if len(leg_indices) == len(mission['legs']): seqs = {cp['seq'] for cp in checkpoints}
    selected_cp = [cp for cp in checkpoints if cp['seq'] in seqs]
    cp_by_seq = {cp['seq']: cp for cp in checkpoints}
    arrays = [geometry[k]['points'] for k in sorted(allowed) if k in geometry]
    if bounds is None:
        xy = np.concatenate([a[:, :2] for a in arrays]+[np.array([[c['x'], c['y']] for c in selected_cp])])
        low, high = xy.min(axis=0), xy.max(axis=0)
        pad = max(22., float(max(high-low))*.07)
        bounds = (low[0]-pad, -(high[1]+pad), high[0]-low[0]+2*pad, high[1]-low[1]+2*pad)
    x, y, w, h = bounds
    unit = max(w, h)/1400.
    def visible(bb):
        return bb[0] <= x+w and bb[2] >= x and bb[1] <= y+h and bb[3] >= y
    surface = ''.join(s for role, bb, s in background if role == 'surface' and visible(bb))
    marks = ''.join(s for role, bb, s in background if role != 'surface' and visible(bb))
    corridor, boundaries, labels, arrows, gateways, warnings = [], [], [], [], [], []
    by_road = defaultdict(list)
    for k in sorted(allowed):
        if k not in geometry: continue
        p = geometry[k]['points']
        by_road[k[0]].append(p)
        title = f'road:section:lane {keytext(k)} | allowed surface (not collision approval)'
        corridor.append(f'<g><title>{title}</title>'+polygon(band(p), fill='#fff000', fill_opacity='.68')+'</g>')
        for end in (p[0], p[-1]):
            boundaries.append(line([end[5:7], end[7:9]], '#123d51', max(.06,unit*.2)))
        length = sum(np.linalg.norm(p[i,:2]-p[i-1,:2]) for i in range(1,len(p)))
        if length > 10:
            for i in range(len(p)//2, len(p), max(len(p), 1)):
                a=p[i]; d=np.array([math.cos(a[2]),math.sin(a[2])]); n=np.array([-d[1],d[0]])
                scale=max(.8,unit*3)
                arrows.append(line([a[:2]-scale*d+scale*.45*n,a[:2],a[:2]-scale*d-scale*.45*n],'#645800',max(.18,unit*.7)))
    occupied = []
    for road, arrays_for_road in sorted(by_road.items()):
        p=max(arrays_for_road,key=len); a=p[len(p)//2]
        # Road labels are displaced with a leader, so even tiny connector IDs remain readable.
        size = max(1.25, unit*10)
        normal=np.array([-math.sin(a[2]),math.cos(a[2])])
        pos=a[:2]+normal*max(5.,unit*15)
        for attempt in range(16):
            if all(np.linalg.norm(pos-v)>size*3.2 for v in occupied): break
            pos=a[:2]+normal*(max(5.,unit*15)+size*(attempt+1)*2)
        occupied.append(pos)
        labels.append(line([a[:2],pos],'#bbf0ff',max(.1,unit*.4)))
        labels.append(text(*pos,f'R{road}',size,'#d4f7ff'))
    for leg in legs:
        n=leg['index']+1
        if leg.get('topology'):
            if mission.get('mode')=='whole_mission_xodr_precomputed':
                if leg.get('declared_gateway_index') is None:continue
                n=leg['declared_gateway_index']
            for k in leg.get('surface_lanes',leg['connection_lanes']):
                p=geometry[tuple(k)]['points']
                gateways.append(f'<g><title>G{n}: XODR connecting lane {keytext(k)}</title>'+polygon(
                    band(p),fill='#ffe000',fill_opacity='.6',stroke='#ff8500',stroke_width=max(.18,unit*.7))+'</g>')
            for direction in ('entry','exit'):
                for a,b in leg.get('surface_'+direction+'_edges',leg[direction+'_edges']):
                    # Actual directed lane interface, not the nearest CP station.
                    p=geometry[tuple(a)]['points'][-1] if direction=='entry' else geometry[tuple(b)]['points'][0]
                    gateways.append(line([p[5:7],p[7:9]],'#ff8200' if direction=='entry' else 'white',max(.45,unit*2)))
            if leg['connection_lanes']:
                p=geometry[tuple(leg['connection_lanes'][0])]['points']
                a=p[len(p)//2]
                gateways.append(text(a[0],a[1]+unit*15,f'G{n}',max(1.5,unit*13),'#ffb348'))
            continue
        for name, candidates, cp, is_entry in (
            (f'CP match IN {n}',leg['entry_candidates'],cp_by_seq[leg['entry_seq']],True),
            (f'Runtime target {n}',leg['targets'],cp_by_seq[leg['exit_seq']],False)):
            if is_entry and leg['entry_seq']==leg['exit_seq']: continue
            for k in candidates:
                k=tuple(k)
                if k not in geometry: continue
                p=near_slice(geometry[k]['points'],cp,max(1.3,unit*2))
                if len(p)<2: continue
                # These transverse bands are a display convention, not stored gate polygons.
                gateways.append(f'<g><title>{name} {keytext(k)}; '+
                                ('checkpoint lane candidate' if is_entry else 'runtime targetLanes')+'</title>'+polygon(
                                    band(p), fill='#fff500', stroke='#ff9200' if is_entry else '#f8ffff',
                                    stroke_width=max(.3,unit*1.4), stroke_dasharray='' if is_entry else f'{unit*3} {unit*2}')+'</g>')
        cached={tuple(r['start']) for r in leg['cached_routes']}
        for k0 in leg['allowed']:
            k=tuple(k0)
            if k not in cached and k in geometry:
                warnings.append(line(geometry[k]['points'][:, :2], '#ff5575', max(.25,unit*1.0),f'{unit*4} {unit*3}'))
    cp_layer=[]
    for j,cp in enumerate(selected_cp):
        px,py=cp['x'],cp['y']; r=max(1.0,unit*6)
        cp_layer.append(f'<circle cx="{px}" cy="{-py}" r="{r}" fill="#ffe600" stroke="#fff" stroke-width="{unit*1.6}"/>')
        cp_layer.append(line([[px-r*.5,py],[px+r*.5,py]],'#171f24',unit*1.2))
        cp_layer.append(line([[px,py-r*.5],[px,py+r*.5]],'#171f24',unit*1.2))
        seq=cp['seq']; last=max(c['seq'] for c in checkpoints)
        role='START' if seq==1 else ('FINISH' if seq==last else ('IN' if seq%2==0 else 'OUT'))
        size=max(1.5,unit*13); dx=(1 if j%2==0 else -1)*unit*34; dy=(1 if j%3==0 else -1)*unit*30
        tx,ty=px+dx,py+dy
        width_guess=len(f'P{seq} {role}')*size*.65
        tx=max(x+unit*10+(width_guess if dx<0 else 0),min(x+w-unit*10-(width_guess if dx>0 else 0),tx))
        ty=max(-y-h+unit*20,min(-y-unit*45,ty))
        cp_layer.append(line([[px,py],[tx,ty]],'#fff',unit*.8))
        cp_layer.append(text(tx,ty,f'P{seq} {role}',size,'#fff77c','start' if dx>0 else 'end'))
    ego_layer=[]
    display_corridors=mission.get('corridor_groups',mission.get('corridor_components',[]))
    for component in display_corridors:
        available=[geometry[tuple(k)]['points'] for k in component['lanes'] if tuple(k) in allowed]
        if not available: continue
        p=max(available,key=len); a=p[len(p)//3]
        if x<=a[0]<=x+w and y<=-a[1]<=y+h:
            ego_layer.append(text(a[0],a[1],f'C{component.get("index",component.get("id"))}',max(1.5,unit*13),'#fff'))
    ego=mission.get('ego')
    if ego:
        ex,ey=ego['x'],ego['y']
        if x<=ex<=x+w and y<=-ey<=y+h:
            er=unit*5
            ego_layer.append(polygon([[ex,ey+er],[ex+er,ey],[ex,ey-er],[ex-er,ey]],fill='#fa63eb',stroke='white',stroke_width=unit))
            ego_layer.append(text(ex+unit*10,ey+unit*12,'EGO start',unit*11,'#fa96ef','start'))
    title=mission['scenario']+(' | stored data only (no active scenario)' if mission['scenario']=='M15' else '')
    title+=' | '+('COMPLETE XODR network (offline)' if mission.get('mode')=='whole_mission_xodr_precomputed' else ('TOPOLOGY proposal (not runtime)' if mission.get('mode') else 'CURRENT runtime sets'))
    title+=' | '+('overview' if len(legs)>1 else f'#{legs[0]["index"]+1}: P{legs[0]["entry_seq"]} -> P{legs[0]["exit_seq"]}')
    layers=(f'<g id="road_surface">{surface}</g><g id="corridors">{"".join(corridor)}</g>'
            f'<g id="road_marks">{marks}</g><g id="road_boundaries">{"".join(boundaries)}</g>'
            f'<g id="arrows">{"".join(arrows)}</g><g id="gateways">{"".join(gateways)}</g>'
            f'<g id="unreachable" visibility="hidden">{"".join(warnings)}</g>'
            f'<g id="road_ids">{"".join(labels)}</g><g id="checkpoints">{"".join(cp_layer+ego_layer)}</g>')
    return (f'<svg xmlns="{NS}" viewBox="{x} {y} {w} {h}" width="1600" height="{int(1600*h/w)}">'
            f'<title>{html.escape(title)}</title><rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#087521"/>'
            +layers+text(x+unit*20,-y-unit*25,title,unit*16,'#fff','start')+'</svg>')


def png(svg, destination, maximum=1800):
    from PyQt5.QtCore import QByteArray, QRectF
    from PyQt5.QtGui import QImage, QPainter
    from PyQt5.QtSvg import QSvgRenderer
    renderer=QSvgRenderer(QByteArray(svg.encode()))
    if not renderer.isValid(): raise ValueError(f'Invalid SVG: {destination}')
    box=renderer.viewBoxF(); scale=maximum/max(box.width(),box.height())
    image=QImage(max(1,int(box.width()*scale)),max(1,int(box.height()*scale)),QImage.Format_ARGB32)
    image.fill(0xff087521); painter=QPainter(image)
    renderer.render(painter,QRectF(0,0,image.width(),image.height())); painter.end()
    if not image.save(str(destination)): raise OSError(destination)


STYLE='body{background:#152129;color:#e6eef3;font:15px system-ui;margin:24px}a{color:#aee7ff}p{line-height:1.65}img{max-width:100%;max-height:800px}table{border-collapse:collapse}td,th{border:1px solid #556;padding:8px}.warn{color:#ffbf77}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:16px}article{background:#24323c;padding:16px}object{width:100%;height:78vh;background:#087521}button,select{padding:8px;margin:5px}'


if __name__=='__main__':
    # Keep the familiar entry point, but never regenerate the old partial view.
    from generate_complete_corridors import main
    main()
