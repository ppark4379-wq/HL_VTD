#!/usr/bin/env python3
"""Headless startup adapter for the same XODR network used by the gallery.

Checkpoints identify road regions; only XODR lane connections define gateway
targets. An explicit M12 first-exit experiment may add clipped physical surface
spans, never routing edges or targets. No simulator or graphics dependency.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from whole_mission_corridors import Network


def same_exit_surface_spans(network, leg, candidate_roads, maximum_length=15.,
                            maximum_heading_error=np.deg2rad(20.)):
    """Verified terminal apron only; do not extend the leg's routing graph.

    The 15m trial envelope covers the observed ~12m upstream body/prefix
    rejection, with room for the full vehicle. Actual terminal arc length and
    direction bound each span; unrelated/crossing connector parts stay out.
    """
    if maximum_length <= 0. or not np.isfinite(maximum_length):
        raise ValueError('Invalid apron length')
    allowed = set(map(tuple, leg['allowed']))
    targets = {tuple(t['lane']) for t in leg['targets']}
    exit_sections = {(k[0], k[1], k[2] > 0) for k in targets}
    junctions = {network.geometry[tuple(k)]['junction'] for k in leg['gateway_lanes']}
    spans = []
    for outgoing in sorted(allowed):
        if (outgoing[0], outgoing[1], outgoing[2] > 0) not in exit_sections:
            continue
        outgoing_geometry = network.geometry[outgoing]
        if outgoing_geometry['junction'] >= 0:
            continue
        outgoing_start = np.asarray(outgoing_geometry['points'])[0]
        for source in sorted(network.reverse[outgoing]):
            if source[0] not in candidate_roads or source in allowed:
                continue
            geometry = network.geometry[source]
            if geometry['junction'] < 0 or geometry['junction'] not in junctions:
                continue
            points = np.asarray(geometry['points'])
            if len(points) < 2 or np.linalg.norm(points[-1, :2]-outgoing_start[:2]) > .1:
                continue
            heading = np.abs(np.arctan2(np.sin(points[:, 2]-outgoing_start[2]),
                                       np.cos(points[:, 2]-outgoing_start[2])))
            if heading[-1] > maximum_heading_error:
                continue
            end_station = start_station = float(points[-1, 3])
            distance = 0.
            for i in range(len(points)-2, -1, -1):
                if heading[i] > maximum_heading_error:
                    break
                step = float(np.linalg.norm(points[i+1, :2]-points[i, :2]))
                if distance+step > maximum_length:
                    fraction = (maximum_length-distance)/step if step else 0.
                    start_station = float(points[i+1, 3] + fraction*(points[i, 3]-points[i+1, 3]))
                    distance = maximum_length
                    break
                start_station = float(points[i, 3])
                distance += step
            if distance < 4.855:
                continue  # No useful full-body envelope; never fabricate one.
            spans.append(dict(lane=source, road_s_min=min(start_station, end_station),
                              road_s_max=max(start_station, end_station),
                              outgoing_lane=outgoing, arc_length_m=distance,
                              maximum_heading_error_rad=maximum_heading_error,
                              provenance='xodr_same_exit_terminal_surface_only'))
    return spans


def apply_surface_trial(network, result, legs, trial=None):
    if trial is None:
        return
    if trial != 'm12-g1-exit-apron':
        raise ValueError('Unknown surface-only trial')
    if not legs or set(map(tuple, legs[0]['gateway_lanes'])) != {(2653, 0, -1)} or \
            {tuple(t['lane']) for t in legs[0]['targets']} != {(2445, 3, 4)}:
        raise ValueError('M12 G1 surface trial does not match the requested gateway')
    first = next((g for g in result['legs'] if g.get('declared_gateway_index') == 1), None)
    if first is None or ((2625, 3, -4), (2653, 0, -1)) not in \
            {(tuple(a), tuple(b)) for a, b in first['entry_edges']}:
        raise ValueError('M12 G1 surface trial has a different incoming road')
    spans = same_exit_surface_spans(network, legs[0], {2659})
    if {tuple(s['lane']) for s in spans} != {(2659, 0, -1), (2659, 0, -2)}:
        raise ValueError('M12 G1 same-exit apron interfaces were not validated')
    legs[0]['surface_only_trial'] = trial
    legs[0]['surface_only_spans'] = spans
    result['surface_only_trial'] = dict(name=trial, runtime_leg=0,
                                       routing_graph_unchanged=True,
                                       gateway_targets_unchanged=True,
                                       spans=spans)


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
            if match.get('association') == 'junction_terminal':
                targets = {tuple(k) for k in result.get(
                    'terminal_alias_lanes', [match['nearest_lane']])}
            else:
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
                aliases = {tuple(item['nearest_lane']): item['road_s']
                           for item in result.get('terminal_alias_matches', [])}
                station = aliases.get(k, matches[exit_]['road_s'])
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
    parser.add_argument('--surface-trial', choices=['m12-g1-exit-apron'])
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
    apply_surface_trial(network, result, result['runtime_legs'], args.surface_trial)
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
