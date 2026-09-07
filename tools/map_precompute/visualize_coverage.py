#!/usr/bin/env python3
"""Inspect restored approach lanes and every same-road source position gap."""
import csv
import html
import json
from collections import defaultdict

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from visualize_reviewed import BASE,draw,shapes


def main():
    dest=BASE/'coverage';dest.mkdir(exist_ok=True)
    report=json.loads((BASE/'validation.json').read_text())
    after=shapes(BASE/'lane_network.svg')
    before=shapes(BASE/'unresolved/overview.svg')
    roads=defaultdict(list)
    with (BASE/'lane_points.csv').open() as f:
        for p in csv.DictReader(f):
            rid=int(p['road_id'])
            if rid in (429,1778,2004,2115):roads[rid].append((float(p['x']),-float(p['y'])))
    cards=[]
    for rid,points in sorted(roads.items()):
        xs,ys=zip(*points)
        bounds=(min(xs)-15,min(ys)-15,max(xs)-min(xs)+30,max(ys)-min(ys)+30)
        fig=Figure(figsize=(12,9),dpi=160,facecolor='#10191f');FigureCanvasAgg(fig)
        draw(fig.add_subplot(121),before,bounds,f'BEFORE | road {rid}')
        draw(fig.add_subplot(122),after,bounds,f'AFTER | road {rid}')
        name=f'road{rid}_before_after.png'
        fig.tight_layout();fig.savefig(dest/name,facecolor=fig.get_facecolor());fig.clear()
        cards.append(f'<article><h2>road {rid}: 접근부 / 합류·분기 전후</h2><a href="{name}"><img src="{name}"></a></article>')
    grouped=defaultdict(list)
    for r in report['source_gap_repairs']:
        grouped[(int(r['lane'].split(':')[0]),round(r['road_s'],2))].append(r)
    for i,((rid,station),rows) in enumerate(sorted(grouped.items()),1):
        x=sum(r['x'] for r in rows)/len(rows);y=-sum(r['y'] for r in rows)/len(rows)
        bounds=(x-22,y-22,44,44)
        fig=Figure(figsize=(12,6),dpi=150,facecolor='#10191f');FigureCanvasAgg(fig)
        draw(fig.add_subplot(121),before,bounds,f'BEFORE | road {rid} s={station:.2f}')
        draw(fig.add_subplot(122),after,bounds,f'AFTER | road {rid} s={station:.2f}')
        name=f'gap_{i:02d}_road{rid}.png'
        fig.tight_layout();fig.savefig(dest/name,facecolor=fig.get_facecolor());fig.clear()
        details=' / '.join(r['lane']+' '+r['status'] for r in rows)
        cards.append(f'<article><h2>원본 단절 {i}: road {rid}, s={station:.2f}</h2><a href="{name}"><img loading="lazy" src="{name}"></a><p>{html.escape(details)}</p></article>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>빈 차선 복원 및 연결 검수</title><style>body{background:#10191f;color:#edf3f6;font:15px system-ui;margin:24px}a{color:#86dcff}article{padding:16px;margin:20px 0;background:#1d2930}img{width:100%;max-width:1800px}</style><h1>빈 차선 복원 및 연결 검수</h1><p>왼쪽: 기존 검토 때의 전체 SVG. 오른쪽: 현재 전체 SVG의 실제 점 경로. 노란색은 기하 후보이며 실선·차량·장애물 제약은 플래닝에서 검증해야 합니다.</p><p><a href="../lane_network.svg">최신 전체 경로 SVG</a> · <a href="../reviewed_17/index.html">기존 17건 검토</a> · <a href="../full_network_audit.json">전체 검사 결과</a></p>'''+''.join(cards)+'</html>'
    (dest/'index.html').write_text(page,encoding='utf-8')
    print('Coverage gallery:',dest/'index.html')


if __name__=='__main__':main()
