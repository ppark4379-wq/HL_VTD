#!/usr/bin/env python3
"""Generate whole-map views and ordered XODR mission artifacts without VTD."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import yaml

from whole_mission_corridors import Network
from render_mission_corridors import draw, png, load_background, STYLE

ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/콜리더_게이트웨이')
    parser.add_argument('--reuse-geometry',action='store_true')
    parser.add_argument('--first',type=int,default=5,help='첫 mixed 시나리오 번호 (기본: 5)')
    parser.add_argument('--last',type=int,default=17,help='끝 mixed 시나리오 번호 (기본: 17)')
    args=parser.parse_args();dest=args.output.resolve();dest.mkdir(parents=True,exist_ok=True)
    if args.first < 1 or args.last < args.first:
        parser.error('--first/--last 범위가 올바르지 않습니다')
    if not args.reuse_geometry:
        subprocess.run([str(ROOT/'build/export_mission_corridors'),str(ROOT),str(dest),'--geometry-only'],check=True)
    geometry={tuple(g['key']):dict(g,points=np.asarray(g['points'])) for g in json.loads((dest/'lane_geometry.json').read_text())}
    network=Network(geometry,json.loads((dest/'lane_topology.json').read_text()),json.loads((dest/'lane_graph.json').read_text()))
    os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
    from PyQt5.QtWidgets import QApplication
    app=QApplication([])
    background_path=dest/'road_map.svg'
    if not background_path.exists():
        shutil.copyfile(ROOT/'output/precomputed_lane_network/road_map.svg',background_path)
    background=load_background(background_path);summary=[];cards=[]
    legend='C1=P1→P2, C2=P3→P4 … 마지막 C=마지막 진출→종료. 노란 면: 그 사이 전체 동일 방향 도로 연결. G1=P2의 콜리더→P3의 콜리더 연결 차선(주황 윤곽), G2=P4→P5 … P는 도로 식별용 좌표이지 필수 통과점이 아닙니다. R은 road ID, 가는 남색 선은 road/section 경계입니다. 콜리더 내부의 작은 XODR junction 연결도 노란 면으로 포함합니다.'
    scenario_numbers=range(args.first,args.last+1)
    for n in scenario_numbers:
        name=f'M{n:02d}';package=ROOT/'scenario/mixed'/name;folder=dest/name
        with (package/'checkpoints.csv').open() as f:
            checkpoints=[dict(seq=int(r['seq']),x=float(r['x']),y=float(r['y'])) for r in csv.DictReader(f)]
        result=network.build(dict(scenario=name,checkpoints=checkpoints))
        if not result['initialized_ok']:
            raise RuntimeError(f'{name}: {result.get("failure", "gateway suffix disconnected")}')
        folder.mkdir(exist_ok=True)
        manifest=yaml.safe_load((package/'scenario.yaml').read_text())
        scene=ET.parse(package/manifest['scenario']['file']).getroot()
        for player in scene.iter('Player'):
            desc=player.find('Description');pos=player.find('.//Init/PosAbsolute')
            if desc is not None and desc.get('Name')=='Ego' and pos is not None:
                result['ego']=dict(x=float(pos.get('X')),y=float(pos.get('Y')))
        result['artifact_version']=2
        serialized=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
        (folder/'complete_network.json').write_text(serialized)
        (folder/'topology.json').write_text(serialized)
        (folder/'audit.json').write_text(serialized)
        svg=draw(result,geometry,background,list(range(len(result['legs']))))
        for alias in ('overview','topology'):
            (folder/f'{alias}.svg').write_text(svg);png(svg,folder/f'{alias}.png',3000)
        row=dict(scenario=name,checkpoints=len(checkpoints),corridors=len(result['corridor_groups']),
            gateways=len(result['gateway_groups']),xodr_connector_groups=sum(bool(g['entry_edges']) for g in result['legs']),
            ordered_connected=result['validation']['ordered_lane_graph_connected'],
            gate_connected=result['validation']['gateway_interfaces_connected'],
            geometry_gaps=len(result['geometry_gap_warnings']),
            centerline_steps=len(result['centerline_step_warnings']),
            pairing_valid=result['validation']['checkpoint_rules_valid'],
            unreachable_lane_sections=sum(len(c['not_reachable']) for c in result['corridor_reachability']))
        summary.append(row)
        warning=('규칙 형식 불일치: 체크포인트 수가 홀수입니다. 좌표 순서 연결 참고용이며 진입·진출 쌍의 정식 검증 통과가 아닙니다. ' if not row['pairing_valid'] else '')
        if name=='M15':warning+='사용자 확인: 실제 시나리오 없음. 저장 데이터만 표시. '
        if row['geometry_gaps']:warning+=f'기하 경계 오차 확인 필요 {row["geometry_gaps"]}개. '
        if row['centerline_steps']:warning+=f'콜리더 면 연결과 별개로 중심선 lane-link 위치 차이 {row["centerline_steps"]}개가 있음. 연속 주행 궤적으로 직접 사용 금지. '
        table=''.join(f'<tr><td>P{m["seq"]}</td><td>{",".join(map(str,network.components[m["component"]] and sorted({k[0] for k in network.components[m["component"]]})))}</td><td>{m["association"]}</td><td>{m["distance_m"]:.2f}m</td></tr>' for m in result['checkpoint_matches'])
        report=f'<p>순서 연결: {row["ordered_connected"]} · 게이트웨이 연결: {row["gate_connected"]} · 콜리더 {row["corridors"]} / 게이트웨이 {row["gateways"]} / 내부 연결 포함 XODR 접속 {row["xodr_connector_groups"]}</p><p class="warn">{warning}이는 지도 그래프·면 연결 검증이며 주행/차선변경 실행 가능성 검증은 아닙니다. 차량 런타임에는 아직 연결하지 않았습니다.</p>'
        page=f'<!doctype html><html lang="ko"><meta charset="utf-8"><title>{name} 전체 콜리더·게이트웨이</title><style>{STYLE}</style><h1>{name} 시작부터 종료까지 전체 연결</h1><p>{legend}</p>{report}<p><a href="../index.html">전체 목록</a> · <a href="overview.svg">전체 SVG 확대</a> · <a href="overview.png">전체 PNG</a> · <a href="complete_network.json">전체 연결 데이터·검사</a></p><a href="overview.svg"><img src="overview.png" style="max-height:none"></a><h2>체크포인트 → 도로 콜리더 매칭</h2><table><tr><th>CP</th><th>Road IDs</th><th>매칭 근거</th><th>일반도로 면까지 거리</th></tr>{table}</table><p>교차로 내부에 피팅된 좌표는 인접 일반도로까지 거리가 클 수 있습니다. 이 거리는 필수 통과점 오차가 아닙니다.</p></html>'
        (folder/'index.html').write_text(page)
        cards.append(f'<article><h2>{name}: C {row["corridors"]} / G {row["gateways"]}</h2><a href="{name}/index.html"><img src="{name}/overview.png" loading="lazy"></a>{report}<a href="{name}/overview.svg">전체 SVG</a> · <a href="{name}/overview.png">전체 PNG</a> · <a href="{name}/index.html">매칭·검사</a></article>')
        print(row,flush=True)
    (dest/'audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    with (dest/'audit.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summary[0]),lineterminator='\n');writer.writeheader();writer.writerows(summary)
    first_name=f'M{args.first:02d}';last_name=f'M{args.last:02d}'
    (dest/'index.html').write_text(f'<!doctype html><html lang="ko"><meta charset="utf-8"><title>전체 콜리더·게이트웨이 v2</title><style>{STYLE}</style><h1>{first_name}–{last_name} 전체 연결 — v2</h1><p>{legend}</p><p>원본 체크포인트 + XODR에서 생성. route.csv 및 기존 체크포인트 lane 매칭 미사용. 그림뿐 아니라 전체 순서 lane graph 연결을 검사합니다.</p><p><a href="README.md">원인·수정 로직·한계</a> · <a href="audit.csv">전체 검사 CSV</a> · <a href="audit.json">JSON</a></p><div class="grid">{"".join(cards)}</div></html>')
    (dest/'README.md').write_text((ROOT/'tools/verification/complete_corridors_README.md').read_text())
    sources=['src/data/map/HL_FMA_VTD_LivingLab.xodr','tools/verification/whole_mission_corridors.py','tools/verification/generate_complete_corridors.py','tools/verification/export_mission_corridors.cpp','tools/verification/render_mission_corridors.py','tools/verification/complete_corridors_README.md']
    sources += [f'scenario/mixed/M{i:02d}/checkpoints.csv' for i in scenario_numbers]
    hashes={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    hashes['bundled:road_map.svg']=hashlib.sha256(background_path.read_bytes()).hexdigest()
    (dest/'provenance.json').write_text(json.dumps(hashes,indent=2)+'\n')


if __name__=='__main__':main()
