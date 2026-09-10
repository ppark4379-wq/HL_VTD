"""Exercise unseen junction endpoints against exported XODR topology (no VTD)."""
import argparse
import json
import random
from pathlib import Path
import numpy as np
from whole_mission_corridors import Network

ROOT = Path(__file__).resolve().parents[2]


def first_road(network, key, at_start):
    graph = network.forward if at_start else network.reverse
    seen = set()
    pending = [key]
    while pending:
        k = pending.pop(0)
        if k in seen:
            continue
        seen.add(k)
        if k in network.owner:
            return k
        pending.extend(sorted(graph[k]-seen))
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--seed', type=int, default=74120)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error('--samples must be positive')
    geometry = {tuple(g['key']):dict(g,points=np.asarray(g['points']))
                for g in json.loads((args.geometry/'lane_geometry.json').read_text())}
    network = Network(geometry,json.loads((args.geometry/'lane_topology.json').read_text()),
                      json.loads((args.geometry/'lane_graph.json').read_text()))
    keys = sorted(k for k,g in geometry.items() if g['junction'] >= 0 and len(g['points'])>5
                  and np.min(g['points'][:,4])>1.5)
    random.Random(args.seed).shuffle(keys)
    records = []
    for key in keys[:args.samples]:
        for at_start in (True,False):
            road = first_road(network,key,at_start)
            if road is None:
                records.append(dict(key=key,role='start' if at_start else 'finish',skipped='no adjacent regular road'))
                continue
            p,q = geometry[key]['points'],geometry[road]['points']
            for fraction in (.1,.5,.9):
                endpoint=p[round((len(p)-1)*fraction)]
                other=q[len(q)//2]
                points=[endpoint,other] if at_start else [other,endpoint]
                cps=[dict(seq=i+1,x=float(v[0]),y=float(v[1]),heading=float(v[2])) for i,v in enumerate(points)]
                result=network.build(dict(scenario='unseen_endpoint',checkpoints=cps))
                group=(result.get('corridor_groups') or [{}])[0 if at_start else -1]
                occupied=key in set(map(tuple,group.get('lanes',[])))
                route=result.get('witness_route',[])
                real_edges=all(any(c==b for c,_ in network.search_graph[a]) for a,b in zip(route,route[1:]))
                passed=result['initialized_ok'] and occupied and real_edges
                records.append(dict(key=key,role='start' if at_start else 'finish',fraction=fraction,
                                    passed=passed,occupied_lane_in_corridor=occupied,
                                    directed_edges_valid=real_edges,failure=result.get('failure'),checkpoints=cps))
                if not passed:
                    print(json.dumps(records[-1]),flush=True)
        print(f'checked {key}: {len(records)} probes',flush=True)
    failed=[r for r in records if r.get('passed') is False]
    summary=dict(seed=args.seed,requested_connectors=args.samples,
                 passed=sum(r.get('passed') is True for r in records),failed=len(failed),
                 skipped=sum('skipped' in r for r in records),records=records,
                 scope='Endpoint occupancy and directed lane graph only; not vehicle or VTD driving validation.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(f"endpoint probes: {summary['passed']} passed, {len(failed)} failed, {summary['skipped']} skipped")
    return 1 if failed else 0


if __name__=='__main__':
    raise SystemExit(main())
