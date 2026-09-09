#!/usr/bin/env python3
"""Rebuild saved random corridors from their existing checkpoint CSV files."""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from generate_random_corridors import ROOT, create_app, write_json
from refresh_random_corridor_gallery import refresh_gallery
from render_mission_corridors import STYLE, draw, load_background, png
from whole_mission_corridors import Network


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--review', action='append', default=[], metavar='SCENARIO:ROAD',
                        help='Save before/after closeups of a reported gateway road.')
    args = parser.parse_args()
    source, dest = args.geometry.resolve(), args.output.resolve()
    provenance = json.loads((source/'provenance.json').read_text(encoding='utf-8'))
    map_path = 'src/data/map/HL_FMA_VTD_LivingLab.xodr'
    if digest(ROOT/map_path) != provenance[map_path]:
        raise RuntimeError('Geometry must be re-exported for the current XODR.')
    geometry = {tuple(g['key']): dict(g, points=np.asarray(g['points']))
                for g in json.loads((source/'lane_geometry.json').read_text(encoding='utf-8'))}
    network = Network(geometry, json.loads((source/'lane_topology.json').read_text()),
                      json.loads((source/'lane_graph.json').read_text()))
    background = load_background(source/'road_map.svg')
    bounds = tuple(map(float, ET.parse(source/'road_map.svg').getroot().get('viewBox').split()))
    app = create_app()
    review_roads = {}
    for spec in args.review:
        name, road = spec.split(':')
        review_roads.setdefault(name, []).append(int(road))
    review = dest/'gateway_surface_review'
    if review_roads:
        review.mkdir(exist_ok=True)
    folders = sorted(p for p in dest.glob('A*') if p.is_dir() and p.name[1:].isdigit())
    if not folders:
        raise ValueError('No saved routes to rebuild.')
    if set(review_roads)-{p.name for p in folders}:
        raise ValueError('Review scenario is not a saved route.')
    records, cards = [], []
    for folder in folders:
        old = json.loads((folder/'complete_network.json').read_text(encoding='utf-8'))
        before_hash = digest(folder/'checkpoints.csv')
        with (folder/'checkpoints.csv').open(encoding='utf-8-sig') as f:
            cps = [dict(seq=int(r['seq']), x=float(r['x']), y=float(r['y'])) for r in csv.DictReader(f)]
        result = dict(old, **network.build(dict(scenario=folder.name, checkpoints=cps)))
        checks = ('checkpoint_rules_valid', 'ordered_lane_graph_connected',
                  'gateway_interfaces_connected', 'surface_interfaces_connected')
        if not result['initialized_ok'] or not all(result['validation'][k] for k in checks):
            raise ValueError(f'{folder.name}: rebuild failed validation')
        # Compare serialized forms because old JSON keys are lists.
        if json.loads(json.dumps(result['witness_route'])) != old['witness_route']:
            raise ValueError(f'{folder.name}: directed route changed; inspect before replacing artifacts')
        result['artifact_version'] = 3
        indices = list(range(len(result['legs'])))
        for road in review_roads.get(folder.name, []):
            arrays = [g['points'] for k, g in geometry.items() if k[0] == road]
            if not arrays:
                raise ValueError(f'Unknown review road: {road}')
            xy = np.concatenate([p[:, 5:7] for p in arrays]+[p[:, 7:9] for p in arrays])
            low, high = xy.min(axis=0)-14, xy.max(axis=0)+14
            crop = (low[0], -high[1], high[0]-low[0], high[1]-low[1])
            stem = f'{folder.name}_R{road}'
            for label, data in [('before', old), ('after', result)]:
                target = review/f'{stem}_{label}.svg'
                if label == 'before' and target.exists():
                    continue
                svg = draw(data, geometry, background, indices, bounds=crop)
                target.write_text(svg, encoding='utf-8')
                png(svg, review/f'{stem}_{label}.png', 1600)
            cards.append(f'<article><h2>{html.escape(stem)}</h2><div class="pair">'+
                         ''.join(f'<div><p>{label}</p><a href="{stem}_{label}.svg">'
                                 f'<img src="{stem}_{label}.svg"></a></div>' for label in ('before', 'after'))+
                         '</div></article>')
        extensions = [dict(gateway=g['declared_gateway_index'], **e)
                      for g in result['legs'] for e in g.get('surface_extensions', [])]
        write_json(folder/'complete_network.json', result)
        for view, view_bounds in [('overview', None), ('whole_map', bounds)]:
            svg = draw(result, geometry, background, indices, bounds=view_bounds)
            (folder/f'{view}.svg').write_text(svg, encoding='utf-8')
            png(svg, folder/f'{view}.png', 3000)
        row = json.loads((folder/'audit.json').read_text(encoding='utf-8'))
        row.update(surface_gaps=len(result['geometry_gap_warnings']),
                   centerline_steps=len(result['centerline_step_warnings']),
                   surface_extensions=len(extensions))
        write_json(folder/'audit.json', row)
        if digest(folder/'checkpoints.csv') != before_hash:
            raise RuntimeError('Checkpoint CSV changed during rebuild')
        records.append(dict(scenario=folder.name, checkpoint_sha256=before_hash,
                            surface_extensions=extensions, validation=result['validation']))
        print(f'{folder.name}: {len(extensions)} taper surfaces; no surface gaps', flush=True)
    selection = dest/'selection.json'
    kept = json.loads(selection.read_text(encoding='utf-8')).get('kept', []) if selection.exists() else []
    refresh_gallery(dest, geometry, background, bounds, kept=kept)
    sources = [source/name for name in ('lane_geometry.json', 'lane_topology.json', 'lane_graph.json', 'road_map.svg')]
    sources += [Path(__file__), Path(__file__).with_name('whole_mission_corridors.py'),
                Path(__file__).with_name('render_mission_corridors.py')]
    write_json(dest/'surface_rebuild.json', dict(artifact_version=3, routes=records,
               sources={str(p.relative_to(ROOT)): digest(p) for p in sources},
               note='Original checkpoint files and directed routes preserved. Taper surface membership does not add routing edges.'))
    if cards:
        (review/'index.html').write_text(
            f'<!doctype html><meta charset="utf-8"><title>Gateway surface comparison</title>'
            f'<style>{STYLE}.pair{{display:grid;grid-template-columns:1fr 1fr;gap:16px}}'
            '.pair img{max-height:900px;width:100%}</style>'
            '<h1>Gateway surface: before / after</h1>'
            '<p>A02 R3145: adjacent R3024 leads to R3018, while this route exits onto R3142. '
            'That separate branch is not a missing taper.</p>'+''.join(cards), encoding='utf-8')
    print(f'Rebuilt {len(records)} saved routes.', flush=True)


if __name__ == '__main__':
    main()

