#!/usr/bin/env python3
"""17 stable-number before/after images using exported geometry, no new paths."""
import csv
import html
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.collections import LineCollection,PolyCollection
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/precomputed_lane_network'
NS='http://www.w3.org/2000/svg'


def shapes(path):
    root=ET.parse(path).getroot();out=[]
    for e in root.iter():
        kind=e.tag.rsplit('}',1)[-1]
        if kind not in ('polygon','polyline'):continue
        xy=np.array([[float(v) for v in p.split(',')] for p in e.get('points').split()])
        out.append((kind,xy,e,(*xy.min(axis=0),*xy.max(axis=0))))
    return out


def draw(ax,rows,bounds,title):
    x,y,w,h=bounds
    rows=[s for s in rows if s[3][0]<=x+w and s[3][2]>=x and s[3][1]<=y+h and s[3][3]>=y]
    polys=[s for s in rows if s[0]=='polygon'];lines=[s for s in rows if s[0]=='polyline']
    if polys:ax.add_collection(PolyCollection([s[1] for s in polys],facecolors=[s[2].get('fill') for s in polys],edgecolors='none'))
    if lines:ax.add_collection(LineCollection([s[1] for s in lines],colors=[s[2].get('stroke') for s in lines],linewidths=[max(.35,float(s[2].get('stroke-width','.2'))*4) for s in lines]))
    ax.set(xlim=(x,x+w),ylim=(y+h,y),aspect='equal');ax.set_facecolor('#182126')
    ax.set_xticks([]);ax.set_yticks([]);ax.set_title(title,color='white',fontsize=12)
    return rows


def main():
    dest=BASE/'reviewed_17';dest.mkdir(exist_ok=True)
    report=json.loads((BASE/'validation.json').read_text())
    cases=[c for c in report['reviewed_cases'] if c['number'] not in (2,3)]
    assert len(cases)==17
    old={c['number']:c for c in json.loads((BASE/'unresolved/locations.json').read_text())}
    after=shapes(BASE/'lane_network.svg')
    anchors={int(c[k]) for c in cases if c['status']=='connected' for k in ('from_point','to_point')}
    lookup={}
    with (BASE/'lane_points.csv').open() as f:
        for p in csv.DictReader(f):
            if int(p['id']) in anchors:lookup[int(p['id'])]=(float(p['x']),-float(p['y']))
    transitions={c['transition_id']:[] for c in cases if c['status']=='connected'}
    with (BASE/'transition_points.csv').open() as f:
        for p in csv.DictReader(f):
            i=int(p['transition_id'])
            if i in transitions:transitions[i].append((float(p['x']),-float(p['y'])))
    summary=Figure(figsize=(16,20),dpi=130,facecolor='#10191f');FigureCanvasAgg(summary)
    cards=[]
    for i,c in enumerate(cases):
        n=c['number'];b=old[n];bounds=[b['x']-30,-b['y']-30,60,60]
        before=shapes(BASE/'unresolved'/b['svg'])
        status={'connected':'CONNECTED','excluded_by_review':'EXCLUDED','unresolved':'UNRESOLVED'}[c['status']]
        if c.get('vehicle_footprint_valid') is False:status='GEOMETRY ONLY'
        fig=Figure(figsize=(14,7.4),dpi=130,facecolor='#10191f');FigureCanvasAgg(fig)
        ax0=fig.add_subplot(121);ax1=fig.add_subplot(122)
        draw(ax0,before,bounds,f'{n:02d} BEFORE | {c["lane"]}')
        draw(ax1,after,bounds,f'{n:02d} AFTER | {c["kind"]} | {status}')
        ax0.plot(b['x'],-b['y'],'o',markerfacecolor='none',markeredgecolor='#ff525d',markersize=14,markeredgewidth=2)
        axs=summary.add_subplot(5,4,i+1);selected=draw(axs,after,bounds,f'{n:02d} | {c["lane"]} | {status}')
        overlays=''
        if c['status']=='connected':
            pts=np.array(transitions[c['transition_id']])
            for ax in (ax1,axs):
                ax.plot(pts[:,0],pts[:,1],color='#6effcf',linewidth=1.5)
                for k in ('from_point','to_point'):
                    x,y=lookup[c[k]];ax.plot(x,y,'o',color='#4dcfff',markersize=5)
            for k in ('from_point','to_point'):
                x,y=lookup[c[k]]
                overlays+=f'<circle cx="{x}" cy="{y}" r=".4" fill="#4dcfff"><title>shared sample {c[k]}</title></circle>'
        title_ko={'connected':'공유 점 연결 완료','excluded_by_review':'요청에 따라 후보 제외','unresolved':'연결 검사 미통과'}[c['status']]
        name=f'{n:02d}_{c["lane"].replace(":","_")}'
        fig.tight_layout(pad=1.2);fig.savefig(dest/(name+'.png'),facecolor=fig.get_facecolor());fig.clear()
        body=''.join(ET.tostring(s[2],encoding='unicode') for s in selected)
        (dest/(name+'.svg')).write_text(f'<svg xmlns="{NS}" width="900" height="900" viewBox="{" ".join(map(str,bounds))}"><rect x="{bounds[0]}" y="{bounds[1]}" width="60" height="60" fill="#182126"/>{body}{overlays}</svg>')
        details=f'공유 점 {c["from_point"]} → {c["to_point"]}; s={c["start_road_s"]:.3f} → {c["end_road_s"]:.3f}' if c['status']=='connected' else c['note']
        if c.get('vehicle_footprint_valid') is False:
            details+=' | 기하 후보만 연결: 차량 전체 통과 검사 미통과, 플래닝 검증 전 사용 금지.'
        if c.get('roadmark_conflicts'):
            details+=' | 실선/차선변경 제한: '+', '.join(c['roadmark_conflicts'])
        cards.append(f'<article id="p{n}"><h2>{n:02d}. {c["lane"]} — {title_ko}</h2><a href="{name}.png"><img loading="lazy" src="{name}.png"></a><p>{html.escape(details)}</p><a href="{name}.svg">수정 후 SVG 확대</a></article>')
    summary.tight_layout(pad=1.5);summary.savefig(dest/'contact_sheet.png',facecolor=summary.get_facecolor());summary.clear()
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>검토 17건 수정 전후</title><style>body{background:#10191f;color:#edf3f6;font:15px system-ui;margin:24px}a{color:#86dcff}article{margin:20px 0;padding:16px;background:#1d2930}img{width:100%;max-width:1700px}p{line-height:1.6}</style><h1>요청한 17건 수정 전후</h1><p>기존 번호 유지: 2·3번은 보류. 각 사진 왼쪽은 이전, 오른쪽은 현재. 청록 선은 새 연결, 파란 점은 실제 공유 점 ID. 제외한 원본은 삭제하지 않고 주행 후보에서만 제외했습니다.</p><p><a href="../lane_network.svg">전체 경로 SVG</a> · <a href="contact_sheet.png">17건 한 장 PNG</a> · <a href="../full_network_audit.json">전체 연결·방향 검사</a></p>'''+''.join(cards)+'</html>'
    (dest/'index.html').write_text(page)
    (dest/'results.json').write_text(json.dumps(cases,indent=2,ensure_ascii=False)+'\n')
    print(f'Wrote 17 before/after PNGs and SVGs: {dest}/index.html')


if __name__=='__main__':main()
