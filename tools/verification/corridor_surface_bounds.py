"""Directed, per-visit surface extents for an offline mission corridor."""
from collections import defaultdict
import numpy as np


def section(key):
    return key[0], key[1], key[2] > 0


def add_endpoint_surface(network, connection, target, at_start):
    """Fill touching parallel connector lanes that reach the selected road.

    Surface aliases are separate from lanes physically under the checkpoint.
    The checkpoint selects a branch; it does not restrict corridor width.
    """
    core = set(connection['lanes'])
    allowed = set(core)
    sections = {section(k) for k in core}
    candidates = {k for k, g in network.geometry.items() if g['junction'] >= 0
                  and section(k) in sections and k not in core}
    while candidates:
        added = set()
        for k in candidates:
            p = np.asarray(network.geometry[k]['points'])
            if p[:, 4].max() <= .01:
                continue
            for neighbor in allowed:
                if section(k) != section(neighbor) or abs(k[2]-neighbor[2]) != 1:
                    continue
                q = np.asarray(network.geometry[neighbor]['points'])
                q = q[np.argsort(q[:, 3])]
                if p[:, 3].min() < q[0, 3]-1e-6 or p[:, 3].max() > q[-1, 3]+1e-6:
                    continue
                gap = min(float(np.linalg.norm(p[:, a:a+2]-np.column_stack([
                    np.interp(p[:, 3], q[:, 3], q[:, b+j]) for j in (0, 1)]), axis=1).max())
                    for a in (5, 7) for b in (5, 7))
                if gap <= .1:
                    added.add(k)
                    break
        if not added:
            break
        allowed |= added
        candidates -= added
    alias_sections = {section(k) for k in connection['aliases']}
    seeds = {k for k in allowed if section(k) in alias_sections}
    graph = network.forward if at_start else network.reverse
    edges, seen, pending = set(), set(), list(seeds)
    while pending:
        k = pending.pop()
        if k in seen:
            continue
        seen.add(k)
        if k in target:
            continue
        for b in graph[k] & (allowed | target):
            edges.add((k, b)); pending.append(b)
    reverse = defaultdict(set)
    for a, b in edges:
        reverse[b].add(a)
    good, pending = set(), list(seen & target)
    while pending:
        k = pending.pop()
        if k in good:
            continue
        good.add(k); pending.extend(reverse[k]-good)
    connection['surface_lanes'] = sorted(core | (good & allowed))
    connection['surface_aliases'] = sorted(seeds & good)
    connection['surface_edges'] = sorted((a, b) if at_start else (b, a)
                                         for a, b in edges if a in good and b in good)
    connection['surface_extensions'] = sorted((good & allowed)-core)


def station(points, checkpoint):
    """Project onto this lane, including between samples and connector aliases."""
    p = np.asarray(points)
    xy = np.array([checkpoint['x'], checkpoint['y']])
    delta = np.diff(p[:, :2], axis=0)
    length2 = np.sum(delta*delta, axis=1)
    t = np.clip(np.sum((xy-p[:-1, :2])*delta, axis=1)/np.maximum(length2, 1e-20), 0., 1.)
    distances = np.sum((p[:-1, :2]+t[:, None]*delta-xy)**2, axis=1)
    i = int(np.argmin(distances))
    return float(p[i, 3]+t[i]*(p[i+1, 3]-p[i, 3]))


def slice_points(points, low, high):
    """Cut both boundaries at exact road-s values, preserving travel order."""
    p = np.asarray(points)
    ascending = p[-1, 3] >= p[0, 3]
    ordered = p if ascending else p[::-1]
    low, high = max(low, ordered[0, 3]), min(high, ordered[-1, 3])
    if high-low <= 1e-9:
        return np.empty((0, p.shape[1]))
    interior = ordered[(ordered[:, 3] > low+1e-9) & (ordered[:, 3] < high-1e-9)]
    ends = np.array([[np.interp(s, ordered[:, 3], ordered[:, j])
                      for j in range(p.shape[1])] for s in (low, high)])
    ends[:, 2] = np.interp([low, high], ordered[:, 3], np.unwrap(ordered[:, 2]))
    cut = np.vstack((ends[0], interior, ends[1]))
    return cut if ascending else cut[::-1]


def surface_segments(mission, geometry, allowed):
    """Canonical visible slices; old serialized missions remain renderable."""
    excluded = set(map(tuple, mission.get('surface_excluded_lanes', [])))
    ranges = {tuple(r['key']): r['ranges'] for r in mission.get('surface_ranges', [])}
    clips = {tuple(r['key']): r for r in mission.get('surface_clips', [])}
    result = {}
    for k in sorted(set(allowed)-excluded):
        if k not in geometry:
            continue
        p = np.asarray(geometry[k]['points'])
        limits = ranges.get(k, [clips.get(k, {})])
        chunks = [slice_points(p, r.get('road_s_min', -np.inf), r.get('road_s_max', np.inf)) for r in limits]
        chunks = [p for p in chunks if len(p) >= 2]
        if chunks:
            result[k] = chunks
    return result


def apply_surface_bounds(network, mission, selected, matches):
    """Keep road width only between actual ingress and egress sections.

    A normal-road component is an undirected topology grouping, so it may
    contain an upstream merge or an unused downstream fork. Its full extent
    is never sufficient evidence for including that branch in a surface.
    Contributions are unioned per visit: a later pass may legitimately use
    an area behind the initial spawn, or two disjoint parts of one lane.
    """
    contributions = defaultdict(list)
    endpoint = mission['endpoint_connections']
    gates = mission['legs'] if len(selected) > 1 else []

    def extent(k, start=None, finish=None):
        p = np.asarray(network.geometry[k]['points'])
        low, high = float(p[:, 3].min()), float(p[:, 3].max())
        increasing = p[-1, 3] >= p[0, 3]
        for cp, at_start in ((start, True), (finish, False)):
            if cp is None:
                continue
            s = station(p, cp)
            if increasing == at_start:
                low = max(low, s)
            else:
                high = min(high, s)
        return dict(road_s_min=low, road_s_max=high)

    mission['corridor_surface_visits'] = []
    for i, component in enumerate(selected):
        keys = network.components[component]
        forward, reverse = defaultdict(set), defaultdict(set)
        for a in keys:
            for b in network.forward[a] & keys:
                forward[section(a)].add(section(b))
                reverse[section(b)].add(section(a))
        if i:
            ingress = {tuple(b) for a, b in gates[i-1].get('surface_exit_edges', gates[i-1]['exit_edges'])}
        elif endpoint['start']['lanes']:
            ingress = {tuple(b) for a, b in endpoint['start'].get('surface_edges', endpoint['start']['edges']) if tuple(b) in keys}
        else:
            ingress = {tuple(matches[0]['nearest_lane'])}
        if i < len(selected)-1:
            egress = {tuple(a) for a, b in gates[i].get('surface_entry_edges', gates[i]['entry_edges'])}
        elif endpoint['finish']['lanes']:
            egress = {tuple(a) for a, b in endpoint['finish'].get('surface_edges', endpoint['finish']['edges']) if tuple(a) in keys}
        else:
            egress = {tuple(matches[-1]['nearest_lane'])}

        def reachable(seeds, graph):
            seen, pending = set(), [section(k) for k in seeds if k in keys]
            while pending:
                s = pending.pop()
                if s in seen:
                    continue
                seen.add(s)
                pending.extend(graph[s]-seen)
            return seen

        permitted = reachable(ingress, forward) & reachable(egress, reverse)
        retained = {k for k in keys if section(k) in permitted}
        visit = dict(corridor=i+1, ingress=sorted(ingress), egress=sorted(egress),
                     lanes=sorted(retained), excluded_lanes=sorted(keys-retained), ranges=[])
        for k in sorted(retained):
            start = mission['checkpoints'][0] if i == 0 and not endpoint['start']['lanes'] and section(k) == section(tuple(matches[0]['nearest_lane'])) else None
            finish = mission['checkpoints'][-1] if i == len(selected)-1 and not endpoint['finish']['lanes'] and section(k) == section(tuple(matches[-1]['nearest_lane'])) else None
            # Parallel lanes share the road reference station. Project once
            # on the matched lane to keep a single transverse road boundary.
            bounds = extent(k)
            for cp, match, at_start in ((start, matches[0], True), (finish, matches[-1], False)):
                if cp is None:
                    continue
                p = np.asarray(network.geometry[k]['points'])
                s = station(network.geometry[tuple(match['nearest_lane'])]['points'], cp)
                field = 'road_s_min' if (p[-1, 3] >= p[0, 3]) == at_start else 'road_s_max'
                bounds[field] = s
            contributions[k].append(bounds)
            visit['ranges'].append(dict(key=k, **bounds))
        mission['corridor_surface_visits'].append(visit)

    # A connector starting on an excluded merge branch is not a gateway
    # alternative. Recompute until its reduced interfaces also stabilize the
    # neighboring road visits; every iteration only removes candidate edges.
    changed = False
    for i, gate in enumerate(gates):
        source = set(mission['corridor_surface_visits'][i]['lanes'])
        target = set(mission['corridor_surface_visits'][i+1]['lanes'])
        permitted = source | target | set(gate['connection_lanes'])
        edges = [(a, b) for a, b in gate['connection_edges'] if a in permitted and b in permitted]
        forward, reverse = defaultdict(set), defaultdict(set)
        for a, b in edges:
            forward[a].add(b); reverse[b].add(a)

        def walk(seeds, graph):
            seen, pending = set(), list(seeds)
            while pending:
                k = pending.pop()
                if k in seen:
                    continue
                seen.add(k); pending.extend(graph[k]-seen)
            return seen

        good = walk(source, forward) & walk(target, reverse)
        old = (gate['connection_edges'], gate.get('surface_entry_edges'), gate.get('surface_exit_edges'))
        for field in ('connection_edges', 'entry_edges', 'exit_edges'):
            gate[field] = [(a, b) for a, b in gate[field] if a in good and b in good]
        gate['connection_lanes'] = [k for k in gate['connection_lanes'] if k in good]
        gate['status'] = 'connected' if gate['entry_edges'] and gate['exit_edges'] else 'UNRESOLVED'
        network.gateway_surface(gate, source=source, target=target)
        changed |= old != (gate['connection_edges'], gate['surface_entry_edges'], gate['surface_exit_edges'])
    if changed:
        return apply_surface_bounds(network, mission, selected, matches)

    for gate in gates:
        for k in gate.get('surface_lanes', gate['connection_lanes']):
            k = tuple(k)
            contributions[k].append(extent(k))
    for role in ('start', 'finish'):
        aliases = set(map(tuple, endpoint[role].get('surface_aliases', endpoint[role]['aliases'])))
        for k in map(tuple, endpoint[role].get('surface_lanes', endpoint[role]['lanes'])):
            cp = mission['checkpoints'][0 if role == 'start' else -1] if k in aliases else None
            bounds = extent(k)
            if cp is not None:
                reference = next((v for v in endpoint[role]['aliases'] if section(v) == section(k)), k)
                p = np.asarray(network.geometry[k]['points'])
                field = 'road_s_min' if (p[-1, 3] >= p[0, 3]) == (role == 'start') else 'road_s_max'
                bounds[field] = station(network.geometry[reference]['points'], cp)
            contributions[k].append(bounds)

    ranges = []
    for k, pieces in sorted(contributions.items()):
        merged = []
        for r in sorted(pieces, key=lambda r: r['road_s_min']):
            # A closed endpoint at a section seam still identifies a real
            # lane, even when its displayed slice has zero area. Retain that
            # topological membership; surface_segments draws only positive
            # length pieces, so no upstream road is painted back in.
            if r['road_s_max']-r['road_s_min'] < -1e-9:
                continue
            if merged and r['road_s_min'] <= merged[-1]['road_s_max']+1e-8:
                merged[-1]['road_s_max'] = max(merged[-1]['road_s_max'], r['road_s_max'])
            else:
                merged.append(dict(r))
        if merged:
            ranges.append(dict(key=k, ranges=merged))
    allowed = {r['key'] for r in ranges}
    original = {tuple(k) for gate in mission['legs'] for k in gate['allowed']}
    mission['surface_ranges'] = ranges
    mission['surface_excluded_lanes'] = sorted(original-allowed)
    mission['surface_clips'] = [dict(key=r['key'], **r['ranges'][0]) for r in ranges if len(r['ranges']) == 1]
    for gate in mission['legs']:
        gate['allowed'] = sorted(set(map(tuple, gate['allowed'])) & allowed)
    for group in mission['corridor_groups']+mission['gateway_groups']:
        group['lanes'] = sorted(set(map(tuple, group['lanes'])) & allowed)
