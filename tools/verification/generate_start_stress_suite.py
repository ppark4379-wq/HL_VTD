"""Map-derived endpoint stress cases and explicit fault injection controls."""
import argparse
import copy
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import random
import numpy as np
from check_start_readiness import inspect_start, first_lanes
from generate_random_corridors import create_app, propose, sample_point, write_json
from rebuild_endpoint_review_routes import BASE, inspect_svg
from render_mission_corridors import draw, load_background
from verify_endpoint_corridors import first_road
from whole_mission_corridors import Network
from prepare_runtime_mission import runtime_legs


def checkpoint(p, seq=1):
    return dict(seq=seq, x=float(p[0]), y=float(p[1]), heading=float(p[2]))


def probe_observation(p, lane, length=4.855, width=1.880):
    """Synthetic geometric-centre OBB, not an actual RDB capture."""
    xy = np.array([p['x'], p['y']]); yaw = p['heading']
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    corners = np.array([[length/2, width/2], [length/2, -width/2], [-length/2, -width/2], [-length/2, width/2]])@rotation.T+xy
    return dict(frame=0, time_s=0., ego=dict(x=p['x'], y=p['y'], heading=yaw, speed=0.,
                road_id=lane[0], section_index=lane[1], lane_id=lane[2], obb=corners.tolist()),
                decision=dict(plan_valid=False, target_speed=0., emergency_brake=False, stop_reason=0))


def annotate_probe_svg(svg, observation):
    """Show the assumed body explicitly, without changing corridor polygons."""
    points = ' '.join(f'{x:.6f},{-y:.6f}' for x, y in observation['ego']['obb'])
    overlay = ('<g id="synthetic_body_probe"><title>Assumed geometric-centre body: 4.855 x 1.880 m; not measured VTD OBB</title>'
               f'<polygon points="{points}" fill="#f55cff" fill-opacity="0.12" stroke="#f55cff" stroke-width="0.16"/></g>')
    return svg.replace('</svg>', overlay+'</svg>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE/'시작점_대비_검증')
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Choose an empty output directory.')
    app = create_app()
    read = lambda name: json.loads((BASE/name).read_text(encoding='utf-8'))
    geo = {tuple(g['key']): dict(g, points=np.asarray(g['points'])) for g in read('lane_geometry.json')}
    net = Network(geo, read('lane_topology.json'), read('lane_graph.json'))
    bg = load_background(BASE/'road_map.svg')
    rng = random.Random(2026091019)
    cases, rejected = [], []
    args.output.mkdir(parents=True)

    def start_request(key, p):
        road = first_road(net, key, True) if key not in net.owner else key
        if road is None:
            return None
        for _ in range(25):
            raw = propose(net, rng, [road], 1)
            if raw:
                cps = raw['checkpoints']; cps[0] = checkpoint(p)
                q = geo[raw['sampled_keys'][-1]]['points']
                cps[-1]['heading'] = float(q[np.argmin(np.sum((q[:, :2]-[cps[-1]['x'], cps[-1]['y']])**2, axis=1)), 2])
                return dict(checkpoints=cps)
        return dict(checkpoints=[checkpoint(p), checkpoint(sample_point(net, road, fraction=.8), 2)])

    def add(label, family, request, expected_lane=None, expected_rejection=False, mutation=None):
        if request is None:
            rejected.append(dict(label=label, reason='no_directed_regular_road'))
            return
        name = f'S{len(cases)+1:02d}'; request = copy.deepcopy(request); request['scenario'] = name
        mission = net.build(request)
        clean = copy.deepcopy(mission)
        if mutation and mission['initialized_ok']:
            mutation(mission)
        expected_present = expected_lane is None or (mission['initialized_ok'] and tuple(expected_lane) in
            set(map(tuple, mission['corridor_groups'][-1 if family == 'junction_finish' else 0]['lanes'])))
        lane = tuple(mission['checkpoint_matches'][0]['nearest_lane']) if mission.get('checkpoint_matches') else tuple(expected_lane or (0, 0, 0))
        obs = probe_observation(request['checkpoints'][0], lane)
        assessment = inspect_start(mission, obs, net, bg) if mission['initialized_ok'] else dict(status='BLOCKED', findings=['MISSION_INITIALIZATION_FAILED'])
        detected = (not mission['initialized_ok'] or assessment['status'] == 'BLOCKED') if expected_rejection else (mission['initialized_ok'] and expected_present)
        try:
            projected = runtime_legs(net, mission)
            runtime_projection = dict(status='SCHEMA_OK', leg_count=len(projected),
                                      final_targets=projected[-1]['targets'])
        except ValueError as error:
            runtime_projection = dict(status='REJECTED', reason=str(error))
        detected = detected and (runtime_projection['status']=='REJECTED' if expected_rejection else runtime_projection['status']=='SCHEMA_OK')
        svg_check = None
        if clean['initialized_ok']:
            # Fault cases show the damaged data, so an omitted endpoint must
            # visibly disappear rather than be reconstructed by the renderer.
            svg = draw(mission, geo, bg, list(range(len(mission['legs']))))
            if not mutation:
                svg_check = inspect_svg(svg, mission, bg)
        else:
            view = dict(scenario=name, checkpoints=request['checkpoints'], mode='whole_mission_xodr_precomputed',
                        legs=[dict(index=0, from_seq=1, entry_seq=1, exit_seq=len(request['checkpoints']), allowed=[], topology=True)])
            svg = draw(view, geo, bg, [0])
        (args.output/f'{name}.svg').write_text(annotate_probe_svg(svg, obs), encoding='utf-8')
        case = dict(id=name, label=label, family=family, expected='reject' if expected_rejection else 'generate_with_endpoint',
                    expectation_met=bool(detected), endpoint_lane_present=bool(expected_present),
                    synthetic_pose_reference='geometric centre, length=4.855 m width=1.880 m; actual live check uses observed OBB',
                    initial_probe=assessment, svg_check=svg_check, mission=mission,
                    runtime_status='NOT_RUN', runtime_projection=runtime_projection, fault_injection=bool(mutation))
        cases.append(case)
        print(name, family, 'expectation='+str(detected), assessment['status'], assessment['findings'], flush=True)

    # Same map location as the reported issue; not the missing original M16.
    for fraction in (.5, .8, .95):
        k = (2515, 0, -1)
        if k in geo:
            add(f'겹친 시작 연결차선 · 진행 {fraction:.0%}', 'overlap_start', start_request(k, sample_point(net, k, fraction=fraction)), k)
    alias_base = copy.deepcopy(cases[-1]['mission'])
    for lateral, angle in ((.12, 0.), (-.12, 0.), (0., math.radians(5))):
        request = dict(checkpoints=copy.deepcopy(alias_base['checkpoints']))
        cp = request['checkpoints'][0]; yaw = cp['heading']
        cp['x'] -= lateral*math.sin(yaw); cp['y'] += lateral*math.cos(yaw); cp['heading'] += angle
        add(f'겹친 시작점 오차 · 횡 {lateral:+.2f}m / 방향 {math.degrees(angle):+.0f}°', 'pose_perturbation', request)

    for road in (2578, 1901, 3102, 1964):
        key = next(k for k in sorted(geo) if k[0] == road and k[2] == -1)
        add(f'교차로 중간 출발 · R{road}', 'junction_start', start_request(key, sample_point(net, key, fraction=.5)), key)
        source = first_road(net, key, False)
        if source:
            add(f'교차로 안 도착 · R{road}', 'junction_finish', dict(checkpoints=[
                checkpoint(sample_point(net, source, fraction=.3)), checkpoint(sample_point(net, key, fraction=.5), 2)]), key)

    boundaries = [(a, b) for a in sorted(net.owner) for b in sorted(net.forward[a])
                  if b in net.owner and a[:2] != b[:2] and net.length[a] > 12 and net.length[b] > 12]
    selected_roads = set()
    for a, b in boundaries:
        if a[0] in selected_roads:
            continue
        selected_roads.add(a[0])
        for offset in (-.25, 0., .25):
            q = geo[b]['points'][0].copy(); q[:2] += offset*np.array([math.cos(q[2]), math.sin(q[2])])
            add(f'도로·구간 경계 {offset:+.2f}m · R{a[0]}→R{b[0]}', 'section_boundary', start_request(b, q))
        if len(selected_roads) == 2:
            break

    # Width changes and same-road parallel connector interfaces.
    taper_keys = [k for k, g in geo.items() if g['junction'] >= 0 and np.min(g['points'][:, 4]) <= .01
                  and np.max(g['points'][:, 4]) > 2.5 and net.length[k] > 10]
    for key in sorted(taper_keys)[:4]:
        p = geo[key]['points']; index = int(np.argmin(abs(p[:, 4]-2.)))
        add(f'폭 변화 연결차선 · R{key[0]} L{key[2]}', 'width_change', start_request(key, p[index]))

    # A real map fork: select overlapping seeds leading to different roads.
    found = 0
    junction_keys = sorted(k for k in geo if k not in net.owner and net.length[k] > 15)
    rng.shuffle(junction_keys)
    for key in junction_keys[:180]:
        p = sample_point(net, key, fraction=.05); cp = checkpoint(p)
        options = net.candidates(cp, float(p[2]), 'start')
        exits = {net.owner[r] for k, m in options.items() if 'endpoint_lanes' in m
                 and abs(m['heading_error_rad']) < math.radians(20)
                 for r in [first_road(net, k, True)] if r is not None}
        if len(exits) < 2:
            continue
        add(f'겹친 위치·서로 다른 출구 · R{key[0]}', 'different_exit_overlap', start_request(key, p), key)
        found += 1
        if found == 2:
            break

    base = next(c['mission'] for c in cases if c['initial_probe']['status'] == 'STATIC_START_OK')
    def remove_start(m):
        k = tuple(m['checkpoint_matches'][0]['nearest_lane'])
        for group in m['corridor_groups']:
            group['lanes'] = [v for v in group['lanes'] if tuple(v) != k]
        for leg in m['legs']:
            leg['allowed'] = [v for v in leg['allowed'] if tuple(v) != k]
    add('시작 차선 누락을 의도적으로 주입', 'fault_missing_start_lane', dict(checkpoints=base['checkpoints']), expected_rejection=True, mutation=remove_start)
    request = dict(checkpoints=copy.deepcopy(base['checkpoints']))
    request['checkpoints'][0].update(x=-200., y=-1500.)
    add('지도 밖 시작점 · 일반도로로 몰래 이동하면 실패', 'invalid_off_map', request, expected_rejection=True)
    key = next(k for k in sorted(net.owner) if net.length[k] > 50)
    add('같은 차선에서 역방향 목표', 'invalid_reverse', dict(checkpoints=[checkpoint(sample_point(net, key, fraction=.8)),
        checkpoint(sample_point(net, key, fraction=.2), 2)]), expected_rejection=True)
    # Centre within the selected lane, OBB outside the allowed corridor.
    edge_base = next(c['mission'] for c in cases if c['family'] == 'junction_start' and c['initial_probe']['status'] == 'STATIC_START_OK')
    for side in (-1, 1):
        request = dict(checkpoints=copy.deepcopy(edge_base['checkpoints']))
        cp = request['checkpoints'][0]; yaw = cp['heading']; key = tuple(edge_base['checkpoint_matches'][0]['nearest_lane'])
        p = geo[key]['points']; q = p[np.argmin(np.sum((p[:, :2]-[cp['x'], cp['y']])**2, axis=1))]
        cp['x'] -= side*(q[4]/2-.05)*math.sin(yaw); cp['y'] += side*(q[4]/2-.05)*math.cos(yaw)
        add(f'차체 경계 걸침 · 횡방향 {side:+d}', 'body_edge_probe', request)

    suite = dict(kind='offline_endpoint_stress_and_fault_injection', cases=cases, omitted_candidates=rejected,
                 map_sha256=hashlib.sha256(Path('src/data/map/HL_FMA_VTD_LivingLab.xodr').read_bytes()).hexdigest(),
                 original_M16_available=(Path(__file__).resolve().parents[2]/'scenario/mixed/M16/M16.xml').exists(), live_VTD_run=False,
                 all_generation_expectations_met=all(c['expectation_met'] for c in cases))
    write_json(args.output/'suite.json', suite)
    with (args.output/'checkpoints.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['case', 'seq', 'x', 'y', 'heading']); writer.writeheader()
        for c in cases:
            writer.writerows(dict(case=c['id'], **p) for p in c['mission']['checkpoints'])
    options = ''.join(f'<option value="{c["id"]}">{c["id"]} · {html.escape(c["label"])}</option>' for c in cases)
    metadata = {c['id']: dict(label=c['label'], expected=c['expected'], expectation_met=c['expectation_met'],
                            probe=c['initial_probe'], runtime='미실행') for c in cases}
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>시작·종료 위험 조건 검증</title>
<style>body{background:#152129;color:#fff;font:16px system-ui;margin:0}header{position:sticky;top:0;background:#152129;padding:18px}select,button,a{font:inherit;padding:7px;margin-right:8px}a{color:#aee7ff}p{margin:8px 0}img{width:100%}pre{white-space:pre-wrap;color:#ffd18b}</style>
<header><h2>시작·종료 위험 조건 검증</h2><select id="pick">__OPTIONS__</select><button onclick="step(-1)">이전</button><button onclick="step(1)">다음</button><a id="direct" target="_blank">SVG 원본</a><a href="checkpoints.csv">전체 좌표</a><p>정상 경로와 거부해야 할 입력을 함께 검사합니다. 차체 검사는 가정한 크기의 정적 탐색이며 VTD 출발 검증은 미실행입니다.</p><p>자홍색 사각형: 가정한 차체. BLOCKED는 원인을 확인해야 하는 사례입니다.</p><pre id="info"></pre></header><img id="map">
<script>const cases=__DATA__;const pick=document.getElementById('pick');function refresh(){const c=cases[pick.value];document.getElementById('map').src=pick.value+'.svg';document.getElementById('direct').href=pick.value+'.svg';document.getElementById('info').textContent='기대: '+c.expected+' / 기대 결과 확인: '+c.expectation_met+' / 정적 차체 검사: '+c.probe.status+' / 실제 출발: '+c.runtime+'\\n'+c.probe.findings.join(', ');}function step(d){pick.selectedIndex=(pick.selectedIndex+d+pick.length)%pick.length;refresh();}pick.onchange=refresh;refresh();</script></html>'''
    page = page.replace('__OPTIONS__', options).replace('__DATA__', json.dumps(metadata, ensure_ascii=False).replace('<', '\\u003c'))
    (args.output/'index.html').write_text(page, encoding='utf-8')
    print('Cases:', len(cases), 'unexpected results:', [c['id'] for c in cases if not c['expectation_met']], flush=True)


if __name__ == '__main__':
    main()
