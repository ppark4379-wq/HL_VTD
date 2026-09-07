#!/usr/bin/env python3
"""Sample every nonzero driving lane directly from XODR and export inspection artifacts."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

from xodr_network import LaneNetwork
from render_lane_network import Drawing

ROOT=Path(__file__).resolve().parents[2]


def write_csv(path, fields, rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(fields)
        writer.writerows(rows)


def export(network, output):
    fields=['id','road_id','section_index','lane_id','point_index','road_s','lane_s',
            'x','y','heading','curvature','width','next_id','left_id','right_id',
            'left_change_allowed','right_change_allowed','usable']
    write_csv(output/'lane_points.csv',fields,
              ([p.id,p.key.road,p.key.section,p.key.lane,p.index,
                *[f'{v:.9f}' for v in (p.road_s,p.lane_s,p.x,p.y,p.heading,p.curvature,p.width)],
                p.next_id,p.left_id,p.right_id,int(p.left_change),int(p.right_change),int(p.usable)]
               for p in network.points))
    write_csv(output/'lane_connections.csv',
              ['id','from_road','from_section','from_lane','to_road','to_section','to_lane',
               'from_point','to_point','provenance','contact_point','junction_id','connection_id',
               'gap_m','heading_delta_rad','curvature_delta','status'],
              ([i,e.source.road,e.source.section,e.source.lane,e.target.road,e.target.section,e.target.lane,
                network.lanes[e.source].points[-1].id,
                network.lanes[e.target].points[0].id if e.target in network.lanes else -1,
                e.provenance,e.contact,e.junction,e.connection,
                f'{e.gap:.9f}',f'{e.heading_delta:.9f}',f'{e.curvature_delta:.9f}',e.status]
               for i,e in enumerate(network.connections)))
    write_csv(output/'lane_transitions.csv',
              ['id','kind','event_road','event_section','event_lane','from_point','to_point',
               'point_count','min_clearance_m','max_curvature','provenance'],
              ([t.id,t.kind,t.event_lane.road,t.event_lane.section,t.event_lane.lane,
                t.from_point,t.to_point,len(t.points),f'{t.min_clearance:.9f}',
                f'{t.max_curvature:.9f}',t.provenance] for t in network.transitions))
    write_csv(output/'transition_points.csv',
              ['transition_id','point_index','road_s','path_s','x','y','heading','curvature'],
              ([t.id,i,*[f'{v:.9f}' for v in (p.road_s,p.s,p.x,p.y,p.heading,p.curvature)]]
               for t in network.transitions for i,p in enumerate(t.points)))
    write_csv(output/'replaced_points.csv',['point_id','transition_id'],sorted(network.replaced_points.items()))
    write_csv(output/'transition_checks.csv',
              ['transition_id','crosses_solid','violates_lane_change_permission','requires_planner_validation','conflicts','vehicle_footprint_valid','geometry_only_reasons'],
              ([t.id,int('solid_boundary' in t.roadmark_conflicts),
                int('roadmark_lane_change_forbidden' in t.roadmark_conflicts),1,';'.join(t.roadmark_conflicts),
                int(t.vehicle_footprint_valid),';'.join(t.geometry_only_reasons)] for t in network.transitions))
    write_csv(output/'lane_access.csv',['road_id','section_index','lane_id','has_entry','has_exit','status'],
              ([k.road,k.section,k.lane,int(v['has_entry']),int(v['has_exit']),v['status']]
               for k,v in sorted(network.lane_access.items())))
    lanes=[dict(road_id=k.road,section_index=k.section,lane_id=k.lane,
                first_point=v.points[0].id,point_count=len(v.points),length_m=v.points[-1].lane_s,
                outgoing_connections=v.outgoing) for k,v in network.lanes.items()]
    manifest=dict(schema_version=2,source=str(network.source.resolve()),
                  source_sha256=hashlib.sha256(network.source.read_bytes()).hexdigest(),
                  sample_interval_m=network.interval,sample_spacing_tolerance_ratio=.001,
                  coordinate_system='XODR inertial x/y; metres; yaw radians CCW',
                  driving_direction='RHT: negative lane follows increasing road_s; positive lane decreasing road_s',
                  files=dict(points='lane_points.csv',connections='lane_connections.csv',
                             transitions='lane_transitions.csv',transition_points='transition_points.csv',
                             replaced_points='replaced_points.csv',lane_access='lane_access.csv',transition_checks='transition_checks.csv'),lanes=lanes,
                  scope='All nonzero driving lane centers, explicit longitudinal XODR connections and constrained merge/split paths; no scenario input',
                  usable_definition='Point width >= vehicle width and abs(curvature) <= steering limit; NOT swept-footprint collision approval',
                  connection_definition='continuous: position gap <= 0.05m, heading difference <= 5deg, curvature difference <= 0.05/m and nonzero endpoint width',
                  lane_changes='Merge/split lifecycle maneuvers are precomputed; general obstacle-avoidance lane changes remain online',
                  transition_definition='XODR taper-boundary-parallel guide with C2 quintic endpoint blends; local quintic where no feasible taper guide exists; actual shared sample anchors and sampled footprint/roadMark checks',
                  topology_definition='Directed lane-level reachability from/to declared external road connections or open map boundaries; diagnostic only, NOT geometric continuity or online safety approval. Geometry is retained irrespective of topology status except explicit excluded_by_review.',
                  roadmark_policy='Geometric candidate generation defers solid-line/laneChange permission decisions to runtime planning; consult transition_checks.csv. No transition is online safety-approved.',
                  transitions=[dict(id=t.id,kind=t.kind,from_point=t.from_point,to_point=t.to_point,
                                    event_lane=t.event_lane.text(),point_count=len(t.points),
                                    shape_method=t.shape_method,taper_range=t.taper_range,
                                    topologically_connected=all(network.lane_access[network.points[p].key]['status']=='connected' for p in (t.from_point,t.to_point)),
                                    roadmark_conflicts=t.roadmark_conflicts,requires_planner_validation=True,
                                    vehicle_footprint_valid=t.vehicle_footprint_valid,geometry_only_reasons=t.geometry_only_reasons) for t in network.transitions],
                  vehicle_width_m=network.vehicle_width,max_curvature_per_m=network.max_curvature,
                  transition_guard=dict(rear_extent_m=1.055,front_extent_m=3.800,
                                        longitudinal_grid_max_m=.5,path_sample_max_m=.51,
                                        minimum_surface_clearance_m=.05))
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xodr',type=Path,default=ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
    parser.add_argument('--output',type=Path,default=ROOT/'output/precomputed_lane_network')
    parser.add_argument('--interval',type=float,default=.5)
    parser.add_argument('--png-pixels',type=int,default=8000)
    parser.add_argument('--vehicle-width',type=float,default=1.880)
    parser.add_argument('--wheelbase',type=float,default=2.944)
    parser.add_argument('--max-steering',type=float,default=.48)
    parser.add_argument('--no-png',action='store_true')
    parser.add_argument('--detail-roads',type=int,nargs='*',default=[72,211],
                        help='Road IDs for reproducible whole-road and maneuver close-ups')
    args=parser.parse_args()
    if not 1000 <= args.png_pixels <= 16000:
        parser.error('--png-pixels must be 1000..16000')
    print(f'[precompute] reading {args.xodr}',flush=True)
    network=LaneNetwork(args.xodr,args.interval,args.vehicle_width,args.wheelbase,args.max_steering)
    args.output.mkdir(parents=True,exist_ok=True)
    manifest=export(network,args.output)
    from network_audit import audit
    full_audit=audit(network)
    (args.output/'full_network_audit.json').write_text(json.dumps(full_audit,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    if full_audit['failures']:
        raise ValueError('whole-network direction/shared-anchor audit failed; see full_network_audit.json')
    print(f'[precompute] {len(network.lanes)} lanes, {len(network.points)} points, {len(network.connections)} links',flush=True)
    print('[precompute] drawing source road surfaces, markings and objects',flush=True)
    drawing=Drawing(network)
    report=dict(counts=dict(roads=len(network.roads),lanes=len(network.lanes),points=len(network.points),
                            connections=len(network.connections),transitions=len(network.transitions),
                            transition_points=sum(len(t.points) for t in network.transitions),
                            replaced_raw_points=len(network.replaced_points),
                            excluded_lanes=len(network.excluded_lanes),
                            topology_unconnected_lanes=len(network.topology_unconnected_lanes),
                            excluded_transitions=len(network.excluded_transitions)),
                source_sha256=manifest['source_sha256'],
                connection_status=dict(Counter(e.status for e in network.connections)),
                issue_counts=dict(Counter(i['kind'] for i in network.issues)),issues=network.issues,
                road_features=dict(drawing.feature_counts),rendering_defaults=dict(drawing.defaults),
                feature_issues=drawing.feature_issues,
                transition_status=dict(Counter(d['status'] for d in network.transition_diagnostics)),
                transitions=network.transition_diagnostics,
                reviewed_cases=network.reviewed_cases,
                source_gap_repairs=network.gap_diagnostics,
                transition_shape_methods=dict(Counter(t.shape_method for t in network.transitions)),
                lane_access_status=dict(Counter(v['status'] for v in network.lane_access.values())),
                excluded_lanes=[dict(lane=k.text(),**network.lane_access[k]) for k in sorted(network.excluded_lanes)],
                notes=[
                    'Raw XODR samples are preserved. Separate merge/split geometric candidates retain road-mark conflict metadata for runtime planning.',
                    'Lifecycle transitions use shared raw point IDs at both anchors; replaced dead-end tails are hidden in the driving overlay.',
                    'A generated transition is not an additional laneLink claimed to exist in XODR. Its provenance is explicit.',
                    'Position/heading breaks are kept visible and excluded from continuous next-point links.',
                    'Pavement FLT files are external; symbols use XODR pose/dimensions and explicit model-name mappings.',
                    'Without roadMark/type/line, broken-line dash lengths are a display default (3m/6m), not source geometry.',
                    'No scenario XML, obstacle positions, corridor or gateway selection was used.',
                    'transition_checks.csv records rectangular-footprint results and deferred roadMark restrictions; geometry-only candidates are explicitly flagged. No continuous swept-volume proof or online safety approval.'])
    (args.output/'validation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    (args.output/'road_map.svg').write_text(drawing.svg(overlay=False),encoding='utf-8')
    (args.output/'lane_network.svg').write_text(drawing.svg(),encoding='utf-8')
    (args.output/'index.html').write_text(drawing.inspection_html(report),encoding='utf-8')
    if not args.no_png:
        print('[precompute] rendering full-map PNGs',flush=True)
        drawing.png(args.output/'road_map.png',overlay=False,pixels=args.png_pixels)
        drawing.png(args.output/'lane_network.png',pixels=args.png_pixels)
        details=args.output/'details'
        details.mkdir(exist_ok=True)
        # Reproducible close-ups for source gaps, tapering lanes and junctions.
        selected=[]
        for kind in ('connection_position_gap','connection_heading_jump','source_position_jump',
                     'narrow_or_tapering_lane','connection_curvature_jump'):
            examples=[i for i in network.issues if i['kind']==kind]
            for issue in examples[:2]:
                if any((issue['x']-j['x'])**2+(issue['y']-j['y'])**2 < 40**2 for j in selected):
                    continue
                selected.append(issue)
        for i,issue in enumerate(selected):
            x,y=issue['x'],issue['y']
            bounds=[x-40,y-40,x+40,y+40]
            name=f"{i+1:02d}_{issue['kind']}_{issue['lane'].replace(':','_')}"
            drawing.png(details/(name+'.png'),bounds=bounds,pixels=1800,diagnostic=True)
        # Whole-road context includes junction approaches and excluded islands.
        for rid in args.detail_roads:
            rows=[p for p in network.points if p.key.road==rid]
            if not rows:
                continue
            xs=[p.x for p in rows];ys=[p.y for p in rows]
            x,y=(min(xs)+max(xs))/2,(min(ys)+max(ys))/2
            span=max(max(xs)-min(xs),max(ys)-min(ys),50)+30
            drawing.png(details/f'road{rid}_overview.png',
                        bounds=[x-span/2,y-span/2,x+span/2,y+span/2],pixels=1800)
        # User-reported merge/split examples, with raw tails removed.
        for transition in network.transitions:
            if transition.event_lane.road not in args.detail_roads:
                continue
            xs=[p.x for p in transition.points]; ys=[p.y for p in transition.points]
            x,y=(min(xs)+max(xs))/2,(min(ys)+max(ys))/2
            span=max(max(xs)-min(xs),max(ys)-min(ys),25)+25
            drawing.png(details/f'road{transition.event_lane.road}_{transition.kind}_{transition.event_lane.section}_{transition.event_lane.lane}.png',
                        bounds=[x-span/2,y-span/2,x+span/2,y+span/2],pixels=1800)
        (details/'index.json').write_text(json.dumps(selected,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    text=(f"# XODR 사전 차선 네트워크\n\n"
          f"- 원본: `{args.xodr}`\n- SHA256: `{manifest['source_sha256']}`\n"
          f"- {len(network.roads)} roads / {len(network.lanes)} nonzero driving lanes / {len(network.points):,} points\n"
          f"- 목표 샘플 간격: {network.interval}m(정상 연결의 간격 허용오차 0.1%; 원본 불연속은 별도 표시)\n"
          f"- 연결 검사: `{report['connection_status']}`\n\n"
          "`index.html`을 브라우저에서 열면 확대, 방향, 실제 점, 연결, 검토 지점을 확인할 수 있습니다.\n"
          "`road_map.png/svg`: 도로 원본 표시. `lane_network.png/svg`: 노란 중앙선 + 진행 방향.\n"
          "`lane_points.csv`: 점의 위치·접선·곡률·폭·다음 점·인접 점. `lane_connections.csv`: 원본 연결과 검사 상태.\n"
          "`manifest.json`: 차선별 점 범위와 연결 인덱스. `validation.json`: 형상 불연속과 렌더링 제한.\n\n"
          "점의 usable은 폭/곡률 검사이며 차량 전체의 안전 통과를 보증하지 않습니다.\n"
          f"XODR 기반 합류/분기 및 같은 도로 내 단절 보완 후보 {len(network.transitions)}개를 추가했습니다.\n"
          "`lane_transitions.csv`, `transition_points.csv`에 실제 연결 점을 저장하고 `replaced_points.csv`에 대체된 원본 꼬리 점을 기록합니다.\n"
          "노란 선은 연결 경로이며 HTML의 원본 중앙선 레이어로 원본과 비교할 수 있습니다. 미해결 연결은 validation.json에 남습니다.\n"
          f"합류/분기 형상: {report['transition_shape_methods']}. 폭 변화 구간은 경계선과 평행한 가이드 및 양 끝 C2 연결을 우선 사용합니다.\n"
          f"진입/진출 미연결 차선도 노란 기하 후보로 표시합니다. 명시적으로 제외 요청한 차선 조각 {len(network.excluded_lanes)}개만 숨깁니다. 위상 진단은 `lane_access.csv`에 보존합니다.\n"
          "실선 및 차선변경 제한은 후보를 지우지 않고 `transition_checks.csv`에 기록합니다. `vehicle_footprint_valid=0` 후보는 차량 전체 통과 검사를 통과하지 못했으며 플래닝 검증 전 사용하면 안 됩니다.\n"
          "`reviewed_17/index.html`: 기존 번호 기준 17건 전후 비교. 2·3번 보류, 14~17·19번은 요청에 따른 후보 제외입니다.\n"
          "노면 모델은 XODR 위치·방향·크기 기반 2D 기호이며 원본 FLT 텍스처가 아닙니다.\n"
          "제어기에 자동 적용하지 않았으며, 추후 Corridor/Gateway가 이 점과 연결을 선택할 수 있습니다.\n")
    (args.output/'README.md').write_text(text,encoding='utf-8')
    print('[precompute] connection audit:',report['connection_status'],flush=True)
    print(f'[precompute] open {args.output / "index.html"}',flush=True)


if __name__=='__main__':
    main()
