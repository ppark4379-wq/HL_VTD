#!/usr/bin/env python3
"""Visualize unresolved events from existing exports; never regenerate paths."""
import argparse
import csv
import html
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.collections import LineCollection, PolyCollection

NS='http://www.w3.org/2000/svg'
ET.register_namespace('',NS)
REASONS={
    'insufficient_track_length':'연결 가능한 차선 길이 부족',
    'insufficient_anchor_space':'시작·종료점 폭/공간 부족',
    'footprint_outside_driving_surface':'차체가 driving 영역 밖으로 나감',
    'footprint_outside_road':'차체가 해당 road 범위를 벗어남',
    'source_geometry_discontinuity':'원본 형상 불연속',
    'solid_boundary':'실선 침범',
    'curvature_limit':'곡률 상한 초과',
    'overlapping_transition':'다른 연결 경로의 앵커와 충돌',
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[2]/'output/precomputed_lane_network')
    args=parser.parse_args();base=args.output;dest=base/'unresolved';dest.mkdir(exist_ok=True)
    report=json.loads((base/'validation.json').read_text())
    events=[dict(number=i+1,**r) for i,r in enumerate(r for r in report['transitions'] if r['status']=='unresolved')]
    source=ET.parse(base/'lane_network.svg').getroot()
    view=list(map(float,source.get('viewBox').split()))
    shapes=[]
    for group in source.findall(f'{{{NS}}}g'):
        for el in group:
            kind=el.tag.rsplit('}',1)[-1]
            if kind not in ('polygon','polyline'):
                continue
            xy=np.array([[float(v) for v in pair.split(',')] for pair in el.get('points').split()])
            shapes.append((kind,xy,el,(*xy.min(axis=0),*xy.max(axis=0))))
    wanted={e['lane'] for e in events};raw={k:[] for k in wanted}
    with (base/'lane_points.csv').open() as f:
        for p in csv.DictReader(f):
            k=f"{p['road_id']}:{p['section_index']}:{p['lane_id']}"
            if k in raw:raw[k].append((float(p['x']),-float(p['y'])))
    def selected(bounds):
        x,y,w,h=bounds
        return [s for s in shapes if s[3][0]<=x+w and s[3][2]>=x and s[3][1]<=y+h and s[3][3]>=y]
    def marker(e,overview=False):
        x,y=e['x'],-e['y'];n=e['number']
        if overview:
            near=[r for r in events if math.hypot(r['x']-e['x'],r['y']-e['y'])<30]
            theta=-math.pi/3+near.index(e)*2*math.pi/max(len(near),1)
            dx,dy=35*math.cos(theta),35*math.sin(theta);radius,font=11,13
        else:dx,dy=12,-12;radius,font=2,2.4
        return (f'<circle cx="{x}" cy="{y}" r="{3 if overview else 1.3}" fill="none" stroke="#ff525d" stroke-width="{1 if overview else .25}"/>'
                f'<path d="M{x},{y} L{x+dx},{y+dy}" stroke="#ff525d" stroke-width="{.9 if overview else .16}"/>'
                f'<circle cx="{x+dx}" cy="{y+dy}" r="{radius}" fill="#c72335" stroke="white" stroke-width="{.6 if overview else .12}"/>'
                f'<text x="{x+dx}" y="{y+dy}" dy=".35em" fill="white" font-size="{font}" font-family="sans-serif" text-anchor="middle">{n:02d}</text>')
    def target_svg(e):
        points=' '.join(f'{x},{y}' for x,y in raw[e['lane']])
        return f'<polyline points="{points}" fill="none" stroke="#ff9b63" stroke-width=".26" stroke-dasharray=".8 .45"><title>Unresolved raw lane {e["lane"]}; NOT a valid connection</title></polyline>'
    # Numbered whole-map vector: preserve the existing map and all path vertices.
    overlay=''.join(marker(e,True) for e in events)
    svg_text=(base/'lane_network.svg').read_text()
    (dest/'overview.svg').write_text(svg_text.replace('</svg>',f'<g id="unresolved_events">{overlay}</g></svg>'))
    def draw(ax,rows,bounds,e):
        x,y,w,h=bounds;ax.set(xlim=(x,x+w),ylim=(y+h,y),aspect='equal');ax.set_facecolor('#182126')
        polygons=[s for s in rows if s[0]=='polygon'];lines=[s for s in rows if s[0]=='polyline']
        if polygons:ax.add_collection(PolyCollection([s[1] for s in polygons],facecolors=[s[2].get('fill') for s in polygons],edgecolors='none'))
        if lines:ax.add_collection(LineCollection([s[1] for s in lines],colors=[s[2].get('stroke') for s in lines],linewidths=[max(.25,float(s[2].get('stroke-width','.2'))*4) for s in lines]))
        if raw[e['lane']]:
            a=np.array(raw[e['lane']]);ax.plot(a[:,0],a[:,1],color='#ff9b63',linewidth=1.1,linestyle='--')
        ax.plot(e['x'],-e['y'],'o',markersize=12,markerfacecolor='none',markeredgecolor='#ff525d',markeredgewidth=1.8)
        ax.set_xticks([]);ax.set_yticks([])
        ax.set_title(f"{e['number']:02d} | {e['lane']} | {e['kind']}",color='white',fontsize=11,pad=7)
    fig=Figure(figsize=(16,4*math.ceil(len(events)/4)),dpi=120,facecolor='#10191f');FigureCanvasAgg(fig)
    cards=[]
    for i,e in enumerate(events):
        bounds=[e['x']-30,-e['y']-30,60,60];rows=selected(bounds)
        stem=f"{e['number']:02d}_{e['kind']}_{e['lane'].replace(':','_')}"
        body=''.join(ET.tostring(s[2],encoding='unicode') for s in rows)
        detail=(f'<svg xmlns="{NS}" viewBox="{" ".join(map(str,bounds))}" width="900" height="900">'
                f'<rect x="{bounds[0]}" y="{bounds[1]}" width="60" height="60" fill="#182126"/>'
                +body+target_svg(e)+marker(e)+'</svg>')
        (dest/(stem+'.svg')).write_text(detail)
        draw(fig.add_subplot(math.ceil(len(events)/4),4,i+1),rows,bounds,e)
        reason=' · '.join(f'{REASONS.get(k,k)} ({v}회)' for k,v in e['rejections'].items())
        cards.append(f'<article id="p{e["number"]}"><h2>{e["number"]:02d} — {e["lane"]} / {"합류" if e["kind"]=="merge" else "분기"}</h2>'
                     f'<a href="{stem}.svg"><img loading="lazy" src="{stem}.svg" alt="미해결 {e["number"]}"></a>'
                     f'<p>s={e["road_s"]:.3f}m · x={e["x"]:.3f}, y={e["y"]:.3f}</p><p>{html.escape(reason)}</p></article>')
        e.update(svg=stem+'.svg',reason_ko=reason)
    fig.tight_layout(pad=1.6);fig.savefig(dest/'contact_sheet.png',facecolor=fig.get_facecolor());fig.clear()
    page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>미해결 {len(events)}곳</title>
<style>body{{background:#10191f;color:#e6edf3;font:15px system-ui;margin:24px}}a{{color:#8bd6ff}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:18px}}article{{background:#1e2b33;padding:14px;scroll-margin-top:16px}}img{{width:100%;display:block}}h2{{font-size:18px}}p{{line-height:1.6}}nav{{line-height:2}}</style>
<h1>미해결 합류·분기 {len(events)}곳</h1><p>빨간 원: 생성 실패 이벤트의 실제 위치. 주황 점선: 해당 원본 차선(사용 가능 경로라는 뜻이 아님). 노란색: 현재 후보 경로. 각 확대 그림은 60m × 60m. 같은 위치의 합류/분기는 별도 번호입니다.</p>
<p>사유의 횟수는 탐색한 후보들의 탈락 횟수이며, 서로 다른 장애물이나 실패 지점 수가 아닙니다. 이 목록은 합류·분기 미해결만 포함하며 전체 지도 불연속 목록과는 다릅니다.</p>
<nav><a href="overview.svg">19곳 번호 표시 전체 SVG</a> · <a href="../lane_network.svg">전체 경로 SVG</a> · <a href="contact_sheet.png">19곳 한 장 보기 PNG</a> · <a href="../index.html">전체 검수 화면</a></nav>
<p>{' · '.join(f'<a href="#p{e["number"]}">{e["number"]:02d} {e["lane"]}</a>' for e in events)}</p><div class="grid">{''.join(cards)}</div></html>'''
    (dest/'index.html').write_text(page)
    (dest/'locations.json').write_text(json.dumps(events,indent=2,ensure_ascii=False)+'\n')
    print(f'Wrote {len(events)} detailed SVGs, numbered overview.svg, contact_sheet.png and {dest / "index.html"}')


if __name__=='__main__':main()
