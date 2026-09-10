"""Rebuild a saved combined SVG batch without changing its checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import numpy as np
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QPainterPath, QPainterPathStroker
from corridor_surface_bounds import surface_segments
from generate_random_corridors import create_app, write_json
from inspect_surface_alignment import path, area
from render_mission_corridors import draw, load_background
from whole_mission_corridors import Network

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'output/콜리더_게이트웨이'
NS = '{http://www.w3.org/2000/svg}'


def svg_path(element):
    shape = QPainterPath()
    for segment in element.get('d', '').split('M')[1:]:
        values = list(map(float, re.findall(r'-?\d+(?:\.\d+)?', segment)))
        shape.addPath(path(np.array(values).reshape(-1, 2)))
    return shape


def inspect_svg(svg, result, background):
    root = ET.fromstring(svg)
    endpoints = [QPointF(p['x'], -p['y']) for p in (result['checkpoints'][0], result['checkpoints'][-1])]
    covered = [False, False]
    stroker = QPainterPathStroker(); stroker.setWidth(.02)
    overlap = 0.
    for group in root.findall(NS+'g'):
        if group.get('id') not in ('corridors', 'gateways', 'road_boundaries'):
            continue
        for element in group.iter(NS+'path'):
            shape = svg_path(element)
            if group.get('id') == 'corridors':
                for i, point in enumerate(endpoints):
                    if not covered[i] and shape.boundingRect().adjusted(-.01, -.01, .01, .01).contains(point):
                        covered[i] = shape.contains(point) or stroker.createStroke(shape).contains(point)
            overlap += sum(area(shape.intersected(b)) for b in background.sidewalks
                           if shape.boundingRect().intersects(b.boundingRect()))
    assert all(covered), (result['scenario'], 'endpoint missing from SVG', covered)
    assert overlap < .005, (result['scenario'], 'sidewalk overlap', overlap)
    return dict(svg_endpoints_covered=covered, svg_sidewalk_overlap_m2=overlap)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=BASE/'SVG_추가_시작종료_60개')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ids', nargs='*')
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error('Build and verify in a separate directory before replacing the review batch.')
    app = create_app()
    old = json.loads((args.source/'complete_networks.json').read_text(encoding='utf-8'))
    if args.ids:
        old = [m for m in old if m['scenario'] in args.ids]
    geo = {tuple(g['key']): dict(g, points=np.asarray(g['points']))
           for g in json.loads((BASE/'lane_geometry.json').read_text(encoding='utf-8'))}
    net = Network(geo, json.loads((BASE/'lane_topology.json').read_text(encoding='utf-8')),
                  json.loads((BASE/'lane_graph.json').read_text(encoding='utf-8')))
    background = load_background(BASE/'road_map.svg')
    args.output.mkdir(parents=True, exist_ok=True)
    results, reports = [], []
    for previous in old:
        name = previous['scenario']
        result = net.build(dict(scenario=name, checkpoints=previous['checkpoints']))
        assert result['initialized_ok'], (name, result.get('failure'))
        assert result['validation']['surface_interfaces_connected'], name
        assert result['checkpoints'] == previous['checkpoints'], name
        assert [list(k) for k in result['witness_route']] == previous['witness_route'], (name, 'witness changed')
        result['artifact_version'] = 5
        result['auto_generation'] = previous['auto_generation']
        pieces = surface_segments(result, geo, {k for g in result['legs'] for k in g['allowed']})
        assert all(tuple(k) in pieces for k in result['witness_route']), (name, 'witness surface missing')
        # Previously reported over-inclusions: check physical retained extents,
        # not just that a clipping metadata field was added.
        if name == 'A126':
            assert (2578, 0, -2) in pieces
            assert abs(pieces[(2578, 0, -2)][0][0, 3]-3.996777397) < .001
        if name == 'A128':
            assert abs(pieces[(1901, 0, -1)][0][0, 3]-38.13326335) < .001
            assert abs(pieces[(1964, 0, -3)][0][-1, 3]-5.972305087) < .001
        if name == 'A129':
            assert not any(k[0] == 3183 for k in pieces)
            assert (3174, 0, -1) in pieces and (2819, 0, -2) in pieces
        svg = draw(result, geo, background, list(range(len(result['legs']))))
        check = inspect_svg(svg, result, background)
        report = dict(scenario=name, checkpoints_unchanged=True, witness_unchanged=True,
                      surface_gaps=len(result['geometry_gap_warnings']),
                      excluded_lanes=result['surface_excluded_lanes'],
                      endpoint_width_extensions=sum(len(e.get('surface_extensions', [])) for e in result['endpoint_connections'].values()),
                      clipped_lanes=sum(any(r['road_s_min'] > geo[k]['points'][:, 3].min()+1e-7 or
                                            r['road_s_max'] < geo[k]['points'][:, 3].max()-1e-7
                                            for r in item['ranges'])
                                        for item in result['surface_ranges'] for k in [tuple(item['key'])]), **check)
        (args.output/f'{name}.svg').write_text(svg, encoding='utf-8')
        results.append(result); reports.append(report)
        print(json.dumps(report), flush=True)
    write_json(args.output/'complete_networks.json', results)
    write_json(args.output/'verification.json', dict(count=len(results), checks=reports,
               endpoint_coverage_tolerance_m=.01, sidewalk_overlap_limit_m2=.005,
               runtime_applied=False, vehicle_feasibility_validated=False))
    sources = [Path(__file__), Path(__file__).with_name('corridor_surface_bounds.py'),
               Path(__file__).with_name('whole_mission_corridors.py'), Path(__file__).with_name('render_mission_corridors.py')]
    write_json(args.output/'surface_bounds_provenance.json',
               {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})


if __name__ == '__main__':
    main()
