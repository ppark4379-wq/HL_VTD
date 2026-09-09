#!/usr/bin/env python3
"""Pick reproducible map-wide checkpoints and use the existing corridor generator."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import numpy as np

from whole_mission_corridors import Network
from render_mission_corridors import draw, png, load_background, STYLE

ROOT = Path(__file__).resolve().parents[2]


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def create_app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtGui import QFontDatabase, QFont
    app = QApplication([])
    # Qt's Windows offscreen plugin does not enumerate system fonts itself.
    if os.name == 'nt':
        fonts = Path(os.environ.get('WINDIR', 'C:/Windows'))/'Fonts'
        for filename in ('arial.ttf', 'malgun.ttf'):
            if (fonts/filename).exists():
                QFontDatabase.addApplicationFont(str(fonts/filename))
        app.setFont(QFont('Arial'))
    if not QFontDatabase().families():
        raise RuntimeError('Qt has no fonts; load a system font before rendering checkpoint labels.')
    return app


def sample_point(network, key, fraction=None, offset=None):
    p = network.geometry[key]['points']
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]
    station = distance[-1] * fraction if fraction is not None else (
        min(offset, distance[-1] * .2) if offset >= 0 else max(distance[-1] + offset, distance[-1] * .8))
    return p[int(np.argmin(abs(distance - station)))]


def propose(network, rng, starts, count):
    """Walk actual longitudinal links, recording every normal-road transition."""
    key = rng.choice(starts)
    route = [key]
    owners = [network.owner[key]]
    gates = []
    last_road = key
    for _ in range(500):
        options = []
        for nxt in sorted(network.forward[key]):
            if nxt in route:
                continue
            owner = network.owner.get(nxt)
            if owner is not None and owner in owners and owner != owners[-1]:
                continue
            options.append(nxt)
        if not options:
            return None
        key = rng.choice(options)
        route.append(key)
        owner = network.owner.get(key)
        if owner is None:
            continue
        if owner != owners[-1]:
            gates.append((last_road, key))
            owners.append(owner)
        last_road = key
        if len(gates) == count and network.length[key] > 35:
            break
        if len(gates) > count:
            return None
    else:
        return None
    points = [sample_point(network, route[0], fraction=.2)]
    keys = [route[0]]
    for incoming, outgoing in gates:
        points.extend([sample_point(network, incoming, offset=-6), sample_point(network, outgoing, offset=6)])
        keys.extend([incoming, outgoing])
    points.append(sample_point(network, key, fraction=.8))
    keys.append(key)
    # Give each approach/departure pair enough separation to identify direction.
    if any(np.linalg.norm(points[i][:2] - points[i+1][:2]) < 8 for i in range(0, len(points), 2)):
        return None
    cps = [dict(seq=i+1, x=round(float(p[0]), 6), y=round(float(p[1]), 6)) for i, p in enumerate(points)]
    turns = []
    for incoming, outgoing in gates:
        a = network.geometry[incoming]['points'][-1, 2]
        b = network.geometry[outgoing]['points'][0, 2]
        angle = math.degrees(math.atan2(math.sin(b-a), math.cos(b-a)))
        turns.append('uturn' if abs(angle) > 150 else 'straight' if abs(angle) < 35 else 'left' if angle > 0 else 'right')
    return dict(checkpoints=cps, sampled_keys=keys, sampled_route=route, owners=owners,
                gates=gates, turns=turns, roads={k[0] for k in route},
                cells={(int(p[0]//250), int(p[1]//250)) for p in points})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, default=ROOT/'output/콜리더_게이트웨이')
    parser.add_argument('--output', type=Path, default=ROOT/'output/콜리더_게이트웨이/자동생성_20개')
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--seed', type=int, default=20260909)
    parser.add_argument('--start-id', type=int, default=1)
    parser.add_argument('--append', action='store_true')
    parser.add_argument('--exclude-signatures', type=Path)
    parser.add_argument('--kept', nargs='*', default=[])
    args = parser.parse_args()
    if args.count < 1 or args.start_id < 1:
        parser.error('--count and --start-id must be positive')
    dest = args.output.resolve()
    if dest.exists() and any(dest.iterdir()) and not args.append:
        parser.error('Output must be empty; choose a new directory to preserve reviewed checkpoints.')
    dest.mkdir(parents=True, exist_ok=True)
    for number in range(args.start_id, args.start_id + args.count):
        if (dest/f'A{number:02d}').exists():
            parser.error(f'A{number:02d} already exists; choose a new --start-id')
    batches_path = dest/'generation_batches.json'
    batches = json.loads(batches_path.read_text(encoding='utf-8')) if batches_path.exists() else []
    if not batches and (dest/'provenance.json').exists():
        batches.append(dict(kind='previous_batch', provenance=json.loads((dest/'provenance.json').read_text(encoding='utf-8'))))
    source = args.geometry.resolve()
    provenance = json.loads((source/'provenance.json').read_text(encoding='utf-8'))
    map_path = 'src/data/map/HL_FMA_VTD_LivingLab.xodr'
    if hashlib.sha256((ROOT/map_path).read_bytes()).hexdigest() != provenance[map_path]:
        raise RuntimeError('Bundled geometry map hash differs from the current XODR; re-export geometry first.')
    geo = {tuple(g['key']): dict(g, points=np.asarray(g['points']))
           for g in json.loads((source/'lane_geometry.json').read_text(encoding='utf-8'))}
    network = Network(geo, json.loads((source/'lane_topology.json').read_text(encoding='utf-8')),
                      json.loads((source/'lane_graph.json').read_text(encoding='utf-8')))
    rng = random.Random(args.seed)
    starts = sorted(k for k in network.owner if network.length[k] > 40)
    background = load_background(source/'road_map.svg')
    bounds = tuple(map(float, ET.parse(source/'road_map.svg').getroot().get('viewBox').split()))
    app = create_app()
    print(f'Loaded {len(geo)} lane sections, {len(network.components)} directed corridors.', flush=True)
    pool = []
    seen = set()
    if args.exclude_signatures:
        seen.update(tuple(tuple(k) for k in sig) for sig in json.loads(args.exclude_signatures.read_text(encoding='utf-8')))
    for _ in range(max(9000, args.count*600)):
        proposal = propose(network, rng, starts, rng.choice([2, 3, 4, 5, 6, 7, 8]))
        if proposal is None:
            continue
        signature = tuple(proposal['owners'])
        if signature not in seen:
            seen.add(signature)
            pool.append(proposal)
    print(f'Candidate pool: {len(pool)} connected checkpoint sequences.', flush=True)
    used_roads, used_cells, used_gates = set(), set(), set()
    for existing in sorted(dest.glob('A*/complete_network.json')):
        old = json.loads(existing.read_text(encoding='utf-8'))
        used_roads.update(k[0] for k in old['witness_route'])
        used_cells.update((int(p['x']//250), int(p['y']//250)) for p in old['checkpoints'])
    summary, cards, rejected, accepted = [], [], [], []
    for number in range(args.start_id, args.start_id + args.count):
        target_count = [2, 3, 4, 5, 6, 7, 8][(number-args.start_id) % 7]
        def score(p):
            return (len(p['gates']) == target_count, len(p['cells']-used_cells)*12 +
                    len(p['roads']-used_roads)*2 + len(set(p['gates'])-used_gates)*3)
        pool.sort(key=score, reverse=True)
        chosen = None
        for attempt in range(min(100, len(pool))):
            candidate = pool.pop(0)
            name = f'A{number:02d}'
            result = network.build(dict(scenario=name, checkpoints=candidate['checkpoints']))
            expected = [network.owner[k] for k in candidate['sampled_keys']]
            actual = [tuple(m['component']) for m in result.get('checkpoint_matches', [])]
            validation = result.get('validation', {})
            reason = None
            if not result['initialized_ok']:
                reason = 'generator_disconnected'
            elif actual != expected:
                reason = 'checkpoint_association_changed'
            elif not validation.get('checkpoint_rules_valid') or result['geometry_gap_warnings']:
                reason = 'pairing_or_surface_gap'
            elif len(result['legs']) != len(candidate['gates']):
                reason = 'unexpected_intermediate_gateway'
            elif any(len(g['connector_indices']) != 1 for g in result['gateway_groups']):
                reason = 'gateway_pair_not_single_transition'
            if reason:
                rejected.append(dict(slot=name, reason=reason, checkpoints=candidate['checkpoints']))
                print(f'{name} retry: {reason}', flush=True)
                continue
            chosen = candidate
            break
        if chosen is None:
            write_json(dest/'rejected_candidates.json', rejected)
            raise RuntimeError(f'Could not generate validated mission {number}')
        used_roads.update(chosen['roads']); used_cells.update(chosen['cells']); used_gates.update(chosen['gates'])
        folder = dest/name
        folder.mkdir()
        with (folder/'checkpoints.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=['seq', 'x', 'y'])
            writer.writeheader(); writer.writerows(chosen['checkpoints'])
        details = []
        for cp, key in zip(chosen['checkpoints'], chosen['sampled_keys']):
            seq = cp['seq']
            role = 'start' if seq == 1 else 'finish' if seq == len(chosen['checkpoints']) else 'entry' if seq % 2 == 0 else 'exit'
            details.append(dict(**cp, role=role, gateway=(seq//2 if role in ('entry','exit') else None), sampled_lane=key))
        write_json(folder/'checkpoint_roles.json', details)
        result['artifact_version'] = 3
        result['auto_generation'] = dict(seed=args.seed, turns=chosen['turns'], sampled_lane_route=chosen['sampled_route'])
        write_json(folder/'complete_network.json', result)
        indices = list(range(len(result['legs'])))
        for title, view_bounds in [('overview', None), ('whole_map', bounds)]:
            svg = draw(result, geo, background, indices, bounds=view_bounds)
            (folder/f'{title}.svg').write_text(svg, encoding='utf-8')
            png(svg, folder/f'{title}.png', 3000)
        row = dict(scenario=name, checkpoints=len(chosen['checkpoints']), gateways=len(result['gateway_groups']),
                   corridors=len(result['corridor_groups']), straight=chosen['turns'].count('straight'),
                   left=chosen['turns'].count('left'), right=chosen['turns'].count('right'),
                   uturn=chosen['turns'].count('uturn'),
                   pairing_valid=validation['checkpoint_rules_valid'],
                   ordered_connected=validation['ordered_lane_graph_connected'],
                   gateways_connected=validation['gateway_interfaces_connected'],
                   surface_gaps=len(result['geometry_gap_warnings']), centerline_steps=len(result['centerline_step_warnings']),
                   surface_extensions=sum(len(g.get('surface_extensions', [])) for g in result['legs']))
        summary.append(row); accepted.append(result)
        write_json(folder/'audit.json', row)
        cp_rows = ''.join(f'<tr><td>P{p["seq"]}</td><td>{p["role"]}</td><td>{p["x"]:.6f}</td><td>{p["y"]:.6f}</td></tr>' for p in details)
        page = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>{name} 자동 생성</title>
<style>{STYLE}</style><h1>{name} · 교차로 {row['gateways']}쌍 · 체크포인트 {row['checkpoints']}개</h1>
<p>출발 → (진입 → 진출) × {row['gateways']} → 도착. 노란 면은 콜리더, 주황 윤곽은 게이트웨이입니다.</p>
<p><a href="../index.html">전체 목록</a> · <a href="checkpoints.csv">좌표 CSV</a> · <a href="checkpoint_roles.json">좌표 역할</a> · <a href="complete_network.json">생성기 결과</a></p>
<p>규칙·순서 연결·게이트웨이 연결 통과. 면 단절 0개. 중심선 단차 {row['centerline_steps']}개(보간 검토 항목).</p>
<p><a href="overview.svg">경로 SVG 확대</a> · <a href="overview.png">경로 PNG</a> · <a href="whole_map.png">전체 맵 PNG</a> · <a href="whole_map.svg">전체 맵 SVG</a></p>
<a href="overview.svg"><img src="overview.png" style="max-height:none"></a>
<table><tr><th>점</th><th>역할</th><th>X</th><th>Y</th></tr>{cp_rows}</table></html>'''
        (folder/'index.html').write_text(page, encoding='utf-8')
        cards.append(f'<article><h2>{name} · 교차로 {row["gateways"]}쌍</h2><a href="{name}/index.html"><img src="{name}/overview.png" loading="lazy"></a><p>좌 {row["left"]} / 우 {row["right"]} / 직진 {row["straight"]} / 유턴 {row["uturn"]}</p><a href="{name}/index.html">검토</a> · <a href="{name}/checkpoints.csv">CSV</a> · <a href="{name}/whole_map.png">전체 맵</a></article>')
        print(row, flush=True)
    from refresh_random_corridor_gallery import refresh_gallery
    all_rows = refresh_gallery(dest, geo, background, bounds, kept=args.kept)
    write_json(dest/f'rejected_candidates_A{args.start_id:02d}_A{args.start_id+args.count-1:02d}.json', rejected)
    files = [source/f for f in ('lane_geometry.json','lane_topology.json','lane_graph.json','road_map.svg')]
    files += [ROOT/map_path, Path(__file__), Path(__file__).with_name('whole_mission_corridors.py'),
              Path(__file__).with_name('render_mission_corridors.py'), Path(__file__).with_name('refresh_random_corridor_gallery.py')]
    if args.exclude_signatures:
        files.append(args.exclude_signatures.resolve())
    provenance = dict(seed=args.seed, sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    batches.append(dict(first_id=args.start_id, count=args.count, seed=args.seed,
                        scenarios=[r['scenario'] for r in summary], kept=args.kept, provenance=provenance))
    write_json(batches_path, batches)
    write_json(dest/'provenance.json', dict(latest_batch=provenance, history_file='generation_batches.json'))
    print(f'Complete: {len(summary)} new, {len(all_rows)} total routes.', flush=True)


if __name__ == '__main__':
    main()
