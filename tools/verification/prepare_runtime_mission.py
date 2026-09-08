#!/usr/bin/env python3
"""Headless startup adapter for the same XODR network used by the gallery.

No scenario name, route.csv, simulator, or graphics dependency. Checkpoints
identify road regions; only XODR lane connections define gateway targets.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from whole_mission_corridors import Network


def runtime_legs(network, result):
    validation = result.get('validation', {})
    if not result['initialized_ok'] or not all(validation.get(k) for k in (
            'checkpoint_rules_valid', 'ordered_lane_graph_connected',
            'gateway_interfaces_connected', 'surface_interfaces_connected')):
        raise ValueError(f'Invalid mission network: {result.get("failure", validation)}')
    matches = result['checkpoint_matches']
    legs = []
    for corridor in result['corridor_groups']:
        start = corridor['from_seq'] - 1
        entry = corridor['to_seq'] - 1
        final = entry == len(matches) - 1
        exit_ = entry if final else entry + 1
        allowed = {tuple(k) for k in corridor['lanes']}
        gate_lanes = set()
        targets = set()
        if not final:
            gateway = next(g for g in result['gateway_groups'] if g['from_seq'] == entry + 1)
            allowed.update(tuple(k) for k in gateway['lanes'])
            gates = [result['legs'][i] for i in gateway['connector_indices']]
            for gate in gates:
                gate_lanes.update(tuple(k) for k in gate['connection_lanes'])
            # The last actual XODR interface, not the CP's arbitrary lane.
            for gate in gates:
                if tuple(gate['target_component']) == tuple(matches[exit_]['component']):
                    targets.update(tuple(b) for a, b in gate['exit_edges'])
            if not targets:
                raise ValueError('Declared intersection has no outgoing XODR interface')
        else:
            match = matches[exit_]
            targets = {k for k in network.components[tuple(match['component'])]
                       if k[:2] == tuple(match['nearest_lane'])[:2]}
        allowed.update(targets)
        # Preserve a complete road-width handoff, including when an exit CP
        # is fitted well downstream of the physical gateway interface.
        if legs:
            allowed.update(tuple(t['lane']) for t in legs[-1]['targets'])
        goal_records = []
        for k in sorted(targets):
            points = np.asarray(network.geometry[k]['points'])
            if final:
                station = matches[exit_]['road_s']
            else:
                # A short distance INSIDE the exit lane prevents a tangent
                # touch or an overlapping incoming lane from advancing a leg.
                first, last = float(points[0, 3]), float(points[-1, 3])
                station = first + np.sign(last - first) * min(2., abs(last - first) / 2.)
            goal_records.append(dict(lane=k, road_s=float(station)))
        lo = result['checkpoint_spans'][start][0]
        hi = result['checkpoint_spans'][exit_ - 1][1]
        witness = result['witness_route'][lo:hi + 1]
        legs.append(dict(from_checkpoint=start, gateway_checkpoint=entry,
                         to_checkpoint=exit_, final=final, allowed=sorted(allowed),
                         gateway_lanes=sorted(gate_lanes), targets=goal_records,
                         witness_route=witness))
    return legs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--checkpoints', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--xodr', type=Path)
    parser.add_argument('--point-network', type=Path)
    args = parser.parse_args()
    if bool(args.xodr) != bool(args.point_network):
        parser.error('--xodr and --point-network must be provided together')
    source_sha = None
    if args.xodr:
        source_sha = hashlib.sha256(args.xodr.read_bytes()).hexdigest()
        manifest = json.loads((args.point_network / 'manifest.json').read_text())
        if source_sha != manifest['source_sha256']:
            raise ValueError('Precomputed lane points belong to a different XODR; regenerate them before driving')
    with args.checkpoints.open(encoding='utf-8-sig', newline='') as f:
        checkpoints = [dict(seq=int(r['seq']), x=float(r['x']), y=float(r['y']))
                       for r in csv.DictReader(f)]
    geometry = {tuple(g['key']): dict(g, points=np.asarray(g['points']))
                for g in json.loads((args.geometry / 'lane_geometry.json').read_text())}
    network = Network(geometry,
        json.loads((args.geometry / 'lane_topology.json').read_text()),
        json.loads((args.geometry / 'lane_graph.json').read_text()))
    result = network.build(dict(checkpoints=checkpoints))
    result['runtime_legs'] = runtime_legs(network, result)
    result['runtime_schema_version'] = 1
    result['checkpoint_sha256'] = hashlib.sha256(args.checkpoints.read_bytes()).hexdigest()
    result['xodr_sha256'] = source_sha
    # The C++ consumer must validate/load successfully before it sets its
    # own topologyAuthority flag. Creating this file alone is not execution.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix('.tmp')
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(args.output)
    print(f'[mission-precompute] corridors={len(result["runtime_legs"])} '
          f'gateways={len(result["gateway_groups"])} output={args.output}', flush=True)


if __name__ == '__main__':
    main()
