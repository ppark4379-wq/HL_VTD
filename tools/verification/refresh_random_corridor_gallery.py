"""Rebuild the SVG review gallery and audit from the route folders still present."""
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from render_mission_corridors import STYLE, draw, png


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def refresh_gallery(dest, geometry, background, bounds, kept=()):
    from PyQt5.QtGui import QImage
    dest = Path(dest)
    folders = sorted((p for p in dest.iterdir() if p.is_dir() and p.name.startswith('A')
                      and p.name[1:].isdigit()), key=lambda p: int(p.name[1:]))
    rows, missions, cards = [], [], []
    for folder in folders:
        mission = json.loads((folder/'complete_network.json').read_text(encoding='utf-8'))
        row = json.loads((folder/'audit.json').read_text(encoding='utf-8'))
        with (folder/'checkpoints.csv').open(encoding='utf-8') as f:
            cps = [dict(seq=int(p['seq']), x=float(p['x']), y=float(p['y'])) for p in csv.DictReader(f)]
        if cps != mission['checkpoints'] or len(cps) != 2*len(mission['gateway_groups'])+2:
            raise ValueError(f'{folder.name}: inconsistent checkpoints')
        if not mission['initialized_ok'] or not all(mission['validation'][key] for key in (
                'checkpoint_rules_valid', 'ordered_lane_graph_connected',
                'gateway_interfaces_connected', 'surface_interfaces_connected')):
            raise ValueError(f'{folder.name}: corridor validation failed')
        for view in ('overview', 'whole_map'):
            ET.parse(folder/f'{view}.svg')
            image = QImage(str(folder/f'{view}.png'))
            if image.isNull() or not 2999 <= max(image.width(), image.height()) <= 3000:
                raise ValueError(f'{folder.name}: missing or invalid {view} PNG')
        rows.append(row); missions.append(mission)
        label = '보존' if folder.name in kept else '새 경로'
        cards.append(f'<article><h2>{folder.name} · {label} · 교차로 {row["gateways"]}쌍</h2>'
                     f'<a href="{folder.name}/overview.svg"><img src="{folder.name}/overview.svg" loading="lazy"></a>'
                     f'<p>좌 {row["left"]} / 우 {row["right"]} / 직진 {row["straight"]}</p>'
                     f'<a href="{folder.name}/overview.svg">SVG 원본</a> · '
                     f'<a href="{folder.name}/checkpoints.csv">좌표 CSV</a> · '
                     f'<a href="{folder.name}/index.html">검사</a></article>')
    if not missions:
        raise ValueError('No route folders to display')
    save_json(dest/'audit.json', rows)
    with (dest/'audit.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    names = [p.name for p in folders]
    allowed = {tuple(k) for m in missions for leg in m['legs'] for k in leg['allowed']}
    union = dict(scenario='Combined corridor coverage', mode='whole_mission_xodr_precomputed',
                 checkpoints=[missions[0]['checkpoints'][0], dict(missions[-1]['checkpoints'][-1], seq=2)],
                 legs=[dict(index=0, from_seq=1, entry_seq=1, exit_seq=2, topology=True,
                            allowed=sorted(allowed), connection_lanes=[], entry_edges=[], exit_edges=[])])
    svg = ET.fromstring(draw(union, geometry, background, [0], bounds=bounds))
    ns = '{http://www.w3.org/2000/svg}'
    for group in svg.findall(ns+'g'):
        if group.get('id') == 'checkpoints':
            svg.remove(group)
    for label in svg.findall(ns+'text') + svg.findall(ns+'title'):
        label.text = f'{len(names)} routes | combined corridor coverage (offline)'
    content = ET.tostring(svg, encoding='unicode')
    (dest/'coverage.svg').write_text(content, encoding='utf-8'); png(content, dest/'coverage.png', 3600)
    save_json(dest/'coverage.json', dict(missions=len(names), scenarios=names,
              road_ids=sorted({k[0] for k in allowed}),
              note='Union of complete corridor components before endpoint clipping; not exhaustive map coverage.'))
    kept = [name for name in names if name in kept]
    new = [name for name in names if name not in kept]
    initial = new[0] if new else names[0]
    options = ''.join(f'<option value="{n}"'+(' selected' if n == initial else '')+
                      f'>{n} · '+('보존' if n in kept else '새 경로')+'</option>' for n in names)
    viewer = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>게이트웨이 SVG 보기</title>
<style>body{margin:0;background:#152129;color:#fff;font:16px system-ui}header{position:sticky;top:0;background:#152129;padding:14px 24px;z-index:1;box-shadow:0 2px 8px #0008}select,button,a{font:inherit;margin-right:10px;padding:6px}a{color:#aee7ff}p{margin:8px 0}main{overflow:auto;padding:12px}img{display:block;width:100%;height:auto}h1{font-size:21px;margin:0 0 10px}</style>
<header><h1>보존 __KEPT__개 + 새 경로 __NEW__개 · SVG 검토</h1>
<button id="previous">← 이전</button><select id="route" aria-label="경로">__OPTIONS__</select><button id="next">다음 →</button>
<select id="view" aria-label="보기"><option value="overview">경로 전체</option><option value="whole_map">전체 맵</option></select>
<button id="minus">축소 −</button><button id="plus">확대 +</button><button id="fit">화면 맞춤</button>
<a id="direct" target="_blank">SVG 원본</a><a id="csv">좌표 CSV</a><a href="index.html">전체 목록</a>
<p>노란 면: 콜리더 · 주황 윤곽: 게이트웨이 · P: 출발 / 교차로 진입·진출 / 도착</p></header>
<main><img id="map" alt="콜리더 및 게이트웨이 SVG"></main>
<script>const route=document.getElementById('route'),view=document.getElementById('view'),map=document.getElementById('map');let scale=100;
function refresh(){const path=route.value+'/'+view.value+'.svg';map.src=path;document.getElementById('direct').href=path;document.getElementById('csv').href=route.value+'/checkpoints.csv';scale=100;map.style.width='100%';}
route.onchange=view.onchange=refresh;
function step(d){route.selectedIndex=(route.selectedIndex+d+route.options.length)%route.options.length;refresh();}
document.getElementById('previous').onclick=()=>step(-1);document.getElementById('next').onclick=()=>step(1);
document.getElementById('plus').onclick=()=>{scale=Math.min(800,scale*1.4);map.style.width=scale+'%';};
document.getElementById('minus').onclick=()=>{scale=Math.max(25,scale/1.4);map.style.width=scale+'%';};
document.getElementById('fit').onclick=()=>{scale=100;map.style.width='100%';};refresh();</script></html>'''
    viewer = viewer.replace('__KEPT__', str(len(kept))).replace('__NEW__', str(len(new))).replace('__OPTIONS__', options)
    (dest/'SVG_보기.html').write_text(viewer, encoding='utf-8')
    (dest/'index.html').write_text(f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>콜리더·게이트웨이 {len(names)}개</title><style>{STYLE}</style>
<h1>보존 {len(kept)}개 + 새 경로 {len(new)}개</h1><p>보존 경로: {', '.join(kept)}. 출발 → (교차로 진입·진출 쌍 n개) → 도착.</p>
<p><a href="SVG_보기.html">SVG 선택·확대 화면 열기</a> · <a href="coverage.svg">전체 생성 영역 SVG</a> · <a href="audit.csv">검사 CSV</a></p>
<p>규칙·순서·게이트웨이·면 연결 검사 통과. 중심선 단차는 각 경로 audit.json에 기록했습니다. 검토·보간 전 자료입니다.</p>
<div class="grid">{''.join(cards)}</div></html>''', encoding='utf-8')
    save_json(dest/'selection.json', dict(kept=kept, new=new, scenarios=names))
    save_json(dest/'verification.json', dict(passed=True, missions=len(names),
              checkpoints=sum(r['checkpoints'] for r in rows), gateways=sum(r['gateways'] for r in rows),
              centerline_review=[r['scenario'] for r in rows if r['centerline_steps']],
              checks=['checkpoint CSV matches generated JSON', 'checkpoint pairing', 'generator connectivity flags',
                      'surface continuity', 'SVG and PNG readability']))
    (dest/'README.md').write_text(f'''# 콜리더·게이트웨이 검토 자료 {len(names)}개

`SVG_보기.html`을 열어 경로를 선택하고 확대합니다. 각 경로 폴더의 `overview.svg`는 경로 전체, `whole_map.svg`는 전체 맵에서의 위치입니다.

보존 경로: {', '.join(kept)}. 새 경로: {', '.join(new)}.

`checkpoints.csv`에는 seq,x,y가 있으며 출발 + 진입·진출 n쌍 + 도착으로 구성됩니다. `complete_network.json`은 기존 생성기의 콜리더·게이트웨이 결과, `checkpoint_roles.json`은 각 점의 역할, `audit.json`은 경로별 검사입니다. 루트 audit.csv는 현재 남아 있는 모든 경로의 검사 목록입니다.

모든 경로는 규칙·그래프 연결·면 연결 검사를 통과했습니다. 중심선 단차가 있는 경로는 verification.json에 기록되며 보간 시 검토해야 합니다. 주행용 연속 궤적 검증은 별도입니다.

coverage.svg는 현재 경로들이 사용하는 콜리더 전체 영역의 합집합입니다(출발·도착점 절단 전). 모든 지도 차선의 완전 커버리지를 뜻하지 않습니다.

폴더 이름은 이전 링크 유지를 위해 그대로 두었으며 실제 경로 수는 {len(names)}개입니다. 배치별 생성 정보는 generation_batches.json에 보존합니다.
''', encoding='utf-8')
    return rows
