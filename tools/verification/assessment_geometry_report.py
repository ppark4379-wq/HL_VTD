#!/usr/bin/env python3
"""Read existing controller/evaluator artifacts and write a separate review report.

No controller, route, corridor or gateway generator is imported or executed.
The recorded lane polylines are displayed as polylines, never invented widths
or regenerated corridor surfaces. Files inside the input run are read-only.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import html
import json
import math
import os
from pathlib import Path


def jsonl(path):
    opener=gzip.open if str(path).endswith('.gz') else open
    with opener(path,'rt',encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def finite(value):
    try:
        value=float(value)
        return value if math.isfinite(value) else None
    except (ValueError,TypeError):
        return None


def xy(value):
    if isinstance(value,dict):
        point=[finite(value.get('x')),finite(value.get('y'))]
    elif isinstance(value,(list,tuple)) and len(value)>=2:
        point=[finite(value[0]),finite(value[1])]
    else:
        return None
    return point if None not in point else None


def csv_rows(path):
    if path is None or not path.is_file():
        return []
    with path.open(newline='',encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def sampled(points,spacing=1.0,limit=None):
    points=[p for raw in points if (p:=xy(raw)) is not None]
    if len(points)<2:
        return points
    out=[points[0]]
    for p in points[1:-1]:
        if math.dist(p,out[-1])>=spacing:
            out.append(p)
    out.append(points[-1])
    if limit and len(out)>limit:
        out=[out[round(i*(len(out)-1)/(limit-1))] for i in range(limit)]
    return [[round(x,4),round(y,4)] for x,y in out]


def segment_distance(point,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length=dx*dx+dy*dy
    t=max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length)) if length else 0
    return math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)


class PolylineIndex:
    def __init__(self,points,cell=10):
        self.points=points;self.cell=cell;self.cells=defaultdict(set)
        for i,(a,b) in enumerate(zip(points,points[1:])):
            for x in range(math.floor(min(a[0],b[0])/cell),math.floor(max(a[0],b[0])/cell)+1):
                for y in range(math.floor(min(a[1],b[1])/cell),math.floor(max(a[1],b[1])/cell)+1):
                    self.cells[x,y].add(i)

    def distance(self,p):
        x,y=math.floor(p[0]/self.cell),math.floor(p[1]/self.cell)
        indices=set().union(*(self.cells.get((x+dx,y+dy),set()) for dx in (-1,0,1) for dy in (-1,0,1)))
        if not indices:
            indices=range(max(0,len(self.points)-1))
        result=min((segment_distance(p,self.points[i],self.points[i+1]) for i in indices),default=None)
        # Outside the local neighborhood, a segment in a further grid cell
        # may be nearer. Keep review distances exact instead of trusting a
        # local-search approximation in an actual route departure.
        if result is not None and result>=self.cell:
            result=min((segment_distance(p,a,b) for a,b in zip(self.points,self.points[1:])),default=None)
        return result


def pick_indices(index,events,max_frames=240):
    if not index:
        return []
    if max_frames<2:
        raise ValueError('max_frames must be at least 2')
    chosen={0,len(index)-1}
    clocks=[r.get('receive_time_s') or 0 for r in index]
    for event in events:
        clock=finite(event.get('received_monotonic_ns'))
        if clock is None:
            continue
        for offset in (-.5,0,.5):
            target=clock/1e9+offset
            at=min(len(index)-1,bisect.bisect_left(clocks,target))
            if at and abs(clocks[at-1]-target)<abs(clocks[at]-target):at-=1
            chosen.add(at)
    # Event neighborhoods and endpoints have priority over uniform samples.
    if len(chosen)>max_frames:
        raise ValueError('Too many event neighborhoods for max_frames; increase the report budget')
    remaining=max_frames-len(chosen)
    candidates=[round(i*(len(index)-1)/max(1,remaining-1)) for i in range(remaining)]
    chosen.update(candidates)
    return sorted(chosen)


def source_info(path):
    record=dict(path=str(path),exists=path.is_file())
    if path.is_file():
        record['size']=path.stat().st_size
        digest=hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
        record['sha256']=digest.hexdigest()
    return record


def metadata_shapes(paths):
    shapes=[];errors=[]
    def walk(node,label,source):
        if isinstance(node,dict):
            identity=str(node.get('id',node.get('name',label)))
            for field in ('polygon','points','boundary_points','line'):
                points=node.get(field)
                if isinstance(points,list) and points and all(xy(p) is not None for p in points):
                    shapes.append(dict(source=str(source),label=identity,kind=field,points=sampled(points,0),metadata=node))
            if all(k in node for k in ('x','y')) and xy(node):
                shapes.append(dict(source=str(source),label=identity,kind='point',points=[xy(node)]))
            for key,value in node.items():
                if key not in ('polygon','points','boundary_points','line'):
                    walk(value,identity+'.'+key,source)
        elif isinstance(node,list):
            for i,value in enumerate(node):walk(value,f'{label}[{i}]',source)
    for path in paths:
        if not path.is_file():errors.append(str(path)+': missing');continue
        try:
            if path.suffix.lower() in ('.yaml','.yml'):
                import yaml
                value=yaml.safe_load(path.read_text(encoding='utf-8'))
            else:value=json.loads(path.read_text(encoding='utf-8'))
            walk(value,path.stem,path)
        except (ValueError,OSError) as error:errors.append(f'{path}: {error}')
    return shapes,errors


def generate_report(*,run_dir,report_dir,route_csv=None,checkpoints_csv=None,
                    assessment_jsonl=None,metadata_paths=None,event_times=None,max_frames=240,
                    history_records=None):
    run_dir=Path(run_dir).resolve();report_dir=Path(report_dir).resolve()
    if report_dir==run_dir or report_dir in run_dir.parents:
        raise ValueError('Report directory must be separate from the input run directory')
    assessment_jsonl=Path(assessment_jsonl) if assessment_jsonl else None
    records=list(jsonl(assessment_jsonl)) if assessment_jsonl and assessment_jsonl.is_file() else []
    session=next((r for r in records if r.get('kind')=='session'),{})
    result=next((r for r in reversed(records) if r.get('kind')=='result'),{})
    events=[r for r in records if r.get('kind') in ('event','deduction')]
    assessment=[r for r in records if r.get('kind')=='frame']
    route_csv=Path(route_csv) if route_csv else Path(session.get('rules',{}).get('route',{}).get('csv',''))
    checkpoints_csv=Path(checkpoints_csv) if checkpoints_csv else None
    route=[p for row in csv_rows(route_csv) if (p:=xy(row)) is not None]
    checkpoints=[dict(row,point=xy(row)) for row in csv_rows(checkpoints_csv) if xy(row)]
    history=run_dir/'rviz/frames.jsonl.gz';mission=run_dir/'mission_corridor.csv';static=run_dir/'rviz/static_map.csv'
    original_svg=run_dir/'mission_corridor.svg'
    missing=[]
    have_history = bool(history_records) if history_records is not None else history.is_file()
    read_history = lambda: iter(history_records) if history_records is not None else jsonl(history)
    for label,present in (('recorded_frame_history',have_history),('intended_route',bool(route)),
                          ('checkpoints',bool(checkpoints)),('assessment_session',bool(records))):
        if not present:missing.append(label)
    groups=defaultdict(list);group_info={}
    mission_rows=csv_rows(mission)
    for row in mission_rows:
        group=':'.join(row.get(k,'') for k in ('leg_index','role','road_id','section_index','lane_id'))
        if (point:=xy(row)) is not None:groups[group].append(point)
        group_info[group]=dict(role=row.get('role'),leg=row.get('leg_index'),lane=[row.get(k) for k in ('road_id','section_index','lane_id')])
    if not mission_rows:
        missing.append('mission_corridor_csv_using_static_fallback')
        for row in csv_rows(static):
            if row.get('layer') not in ('corridor','gateway','corridor_surface_only'):continue
            group=row.get('layer','')+':'+row.get('group','')
            if (point:=xy(row)) is not None:groups[group].append(point)
            group_info[group]=dict(role=row.get('layer'),leg=row.get('group','').split(':')[0],lane=[row.get(k) for k in ('road_id','section_index','lane_id')])
    if not groups:missing.append('recorded_corridor_and_gateway_geometry')
    geometry=[dict(id=k,**group_info[k],points=sampled(p,.75),original_points=len(p)) for k,p in groups.items()]
    allowed={}
    for group in geometry:
        allowed.setdefault(str(group['leg']),set()).add(tuple(map(str,group['lane'])))
    route_index=PolylineIndex(route)
    index=[];trajectory=[];deviations=[];step_distances=[];outside=[];last=None
    checkpoint_nearest=[dict(seq=c.get('seq',str(i+1)),point=c['point'],distance_m=None) for i,c in enumerate(checkpoints)]
    frame_gaps=[];gateway_observations={};max_path_step=0.0
    if have_history:
        for row in read_history():
            ego=row.get('ego',{});point=xy(ego)
            if point is None:continue
            record=dict(frame=row.get('frame'),time_s=row.get('time_s'),receive_time_s=finite(row.get('receive_time_s')),
                        point=point,speed=finite(ego.get('speed')),active_leg=row.get('decision',{}).get('active_leg'),
                        lane=[ego.get(k) for k in ('road_id','section_index','lane_id')])
            record['route_distance_m']=route_index.distance(point)
            if record['route_distance_m'] is not None:deviations.append(record['route_distance_m'])
            membership=allowed.get(str(record['active_leg']))
            record['corridor_lane_membership']=tuple(map(str,record['lane'])) in membership if membership else None
            if record['corridor_lane_membership'] is False:outside.append(dict(frame=record['frame'],lane=record['lane'],leg=record['active_leg']))
            if last:
                step_distances.append(math.dist(last['point'],point))
                if isinstance(record['frame'],int) and isinstance(last['frame'],int) and record['frame']>last['frame']+1:
                    frame_gaps.append(dict(after=last['frame'],before=record['frame'],missing=record['frame']-last['frame']-1))
            for near in checkpoint_nearest:
                distance=math.dist(point,near['point'])
                if near['distance_m'] is None or distance<near['distance_m']:
                    near.update(distance_m=distance,frame=record['frame'],receive_time_s=record['receive_time_s'])
            for group in geometry:
                if group['role']=='gateway' and tuple(map(str,group['lane']))==tuple(map(str,record['lane'])):
                    entry=gateway_observations.setdefault(group['id'],dict(leg=group['leg'],lane=group['lane'],first_frame=record['frame'],last_frame=record['frame'],observed_frames=0))
                    entry.update(last_frame=record['frame'],observed_frames=entry['observed_frames']+1)
            tracked=[p for raw in row.get('tracked_path',[]) if (p:=xy(raw)) is not None]
            max_path_step=max(max_path_step,max((math.dist(a,b) for a,b in zip(tracked,tracked[1:])),default=0))
            index.append(record);trajectory.append(point);last=record
    assessment_clocks=[finite(r.get('received_monotonic_ns')) or 0 for r in assessment]
    for seconds in event_times or []:
        nearest=min(assessment,key=lambda r:abs(float(r.get('sim_time_s',0))-seconds),default=None)
        if nearest:events.append(dict(event='selected_sim_time',sim_time_s=seconds,received_monotonic_ns=nearest.get('received_monotonic_ns')))
    selected=set(pick_indices(index,events,max_frames))
    selected_frames=[];valid_index=-1
    if have_history:
        for row in read_history():
            if xy(row.get('ego')) is None:continue
            valid_index+=1
            if valid_index not in selected:continue
            summary=index[valid_index]
            clock=summary.get('receive_time_s')
            assessment_match=None;clock_error=None
            if assessment and clock is not None:
                position=min(len(assessment)-1,bisect.bisect_left(assessment_clocks,clock*1e9))
                if position and abs(assessment_clocks[position-1]-clock*1e9)<abs(assessment_clocks[position]-clock*1e9):position-=1
                clock_error=(assessment_clocks[position]-clock*1e9)/1e9
                if abs(clock_error)<=.25:assessment_match=assessment[position]
            selected_frames.append(dict(summary,ego=row.get('ego'),objects=row.get('objects',[]),
                tracked_path=sampled(row.get('tracked_path',[]),.3,180),maneuver_path=sampled(row.get('maneuver_path',[]),.3,180),
                candidate_paths=[dict(lane=c.get('lane'),status=c.get('status'),reason=c.get('reason'),points=sampled(c.get('points',[]),.5,120)) for c in row.get('candidate_paths',[])[:12]],
                decision=row.get('decision'),global_route_metadata=row.get('global_route'),
                assessment=assessment_match,assessment_clock_error_s=clock_error))
    metadata_paths=[Path(p) for p in (metadata_paths or [])]
    overlays,metadata_errors=metadata_shapes(metadata_paths)
    points=route+trajectory+[c['point'] for c in checkpoints]
    if not points:points=[p for g in geometry for p in g['points']]
    bounds=[min((p[0] for p in points),default=0)-15,min((p[1] for p in points),default=0)-15,
            max((p[0] for p in points),default=100)+15,max((p[1] for p in points),default=100)+15]
    paths=[history,mission,static,original_svg]+([route_csv] if route_csv.is_file() else [])+([checkpoints_csv] if checkpoints_csv else [])+([assessment_jsonl] if assessment_jsonl else [])+metadata_paths
    sources=[source_info(p) for p in paths]
    report=dict(schema='hl_vtd_assessment_geometry_report/v1',title=run_dir.name,bounds=bounds,sources=sources,missing=missing,
        source_policy='Only existing artifacts were read. No controller or geometry generator was executed or modified.',
        live_prefix_snapshot=history_records is not None,
        sampling=dict(recorded_frames=len(index),displayed_frames=len(selected_frames),max_frames=max_frames,
            frames='Uniform sample plus exact recorded frames nearest each event and ±0.5 seconds by monotonic clock',
            geometry='Original polyline endpoints retained; static 0.75m spacing, tracked 0.3m/max180, candidates 0.5m/max120',
            trajectory='All recorded positions used for metrics; display spacing 0.3m/max6000'),
        summary=dict(route_deviation_max_m=max(deviations,default=None),trajectory_step_max_m=max(step_distances,default=None),
            tracked_path_step_max_m=max_path_step,recorded_frame_gap_count=len(frame_gaps),recorded_missing_frame_numbers=sum(g['missing'] for g in frame_gaps),
            outside_recorded_corridor_lane_frames=len(outside),outside_examples=outside[:30],
            geometry_groups=dict(Counter(g['role'] for g in geometry)),checkpoint_nearest=checkpoint_nearest,
            gateway_lane_observations=list(gateway_observations.values()),assessment_summary=result.get('summary')),
        limitations=['Corridor/gateway CSV records lane polylines, not boundary polygons. Lane membership is not a geometric containment verdict.',
            'Gateway observations mean Ego used a recorded gateway lane, not proof of crossing a finite gateway line.',
            'Dynamic full global-route coordinates were not recorded; only tracked_path, maneuver_path and global_route metadata are available.',
            'RViz history is sampled by its original recorder; displayed objects are the controller selected objects, not the full RDB object feed.',
            'Assessment linkage uses monotonic receipt clocks within 0.25s, not equal controller/RDB frame numbers.',
            'Distance from route is a geometric review measurement, not automatically a competition violation.'],
        unrecorded_geometry=['precise_corridor_boundary_polygons','finite_gateway_crossing_lines','full_dynamic_global_route_coordinates'],
        events=events,assessment_grid=result.get('official_grid',[]),metadata_errors=metadata_errors,
        original_mission_svg=os.path.relpath(original_svg,report_dir) if original_svg.is_file() else None)
    payload=dict(bounds=bounds,route=sampled(route,.5),trajectory=sampled(trajectory,.3,6000),checkpoints=checkpoints,geometry=geometry,metadata=overlays,frames=selected_frames,events=events)
    report_dir.mkdir(parents=True,exist_ok=True)
    outputs=dict(html=report_dir/'index.html',svg=report_dir/'overview.svg',json=report_dir/'report.json',frames_json=report_dir/'geometry.json',script=report_dir/'geometry.js')
    input_paths={p.resolve() for p in paths}
    if any(path.resolve() in input_paths for path in outputs.values()):
        raise ValueError('A report output would overwrite an input artifact')
    # Fixed report filenames are the only write targets. Sources are never rewritten.
    outputs['json'].write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    encoded=json.dumps(payload,ensure_ascii=False,separators=(',',':'),allow_nan=False)
    outputs['frames_json'].write_text(encoded+'\n',encoding='utf-8')
    outputs['script'].write_text('window.GEOMETRY_DATA='+encoded+';\n',encoding='utf-8')
    outputs['svg'].write_text(overview_svg(payload),encoding='utf-8')
    outputs['html'].write_text(report_html(report),encoding='utf-8')
    return {**{k:str(v) for k,v in outputs.items()},'sources':sources,'missing':missing,'summary':report['summary']}


def overview_svg(payload):
    x0,y0,x1,y1=payload['bounds'];width,height=x1-x0,y1-y0
    chunks=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0} {-y1} {width} {height}" width="1000" height="850"><rect x="{x0}" y="{-y1}" width="{width}" height="{height}" fill="#101921"/>']
    def line(points,color,stroke):
        if len(points)>1:chunks.append('<polyline fill="none" stroke="'+color+'" stroke-width="'+str(stroke)+'" points="'+' '.join(f'{x},{-y}' for x,y in points)+'"/>')
    for group in payload['geometry']:line(group['points'],'#25ced1' if group['role']=='gateway' else '#456370',2 if group['role']=='gateway' else .8)
    line(payload['route'],'#ffa938',1.1);line(payload['trajectory'],'#ff58ac',1.5)
    for i,c in enumerate(payload['checkpoints']):
        x,y=c['point'];chunks.append(f'<circle cx="{x}" cy="{-y}" r="2" fill="#fff"/><text x="{x+3}" y="{-y}" fill="#fff" font-size="5">CP{html.escape(str(c.get("seq",i+1)))}</text>')
    chunks.append('</svg>');return ''.join(chunks)


def report_html(report):
    title=html.escape(report['title'])
    rows=''.join(f'<tr><td>{html.escape(str(r.get("number")))}</td><td>{html.escape(str(r.get("status")))}</td><td>{html.escape(str(r.get("penalty")))}</td><td>{html.escape(str(r.get("detail")))}</td></tr>' for r in report['assessment_grid'])
    missing=html.escape(', '.join(report['missing']) or '필수 입력 파일 누락 없음')
    warnings=''.join('<li>'+html.escape(t)+'</li>' for t in report['limitations'])
    original='<a href="'+html.escape(report['original_mission_svg'],quote=True)+'">원본 mission_corridor.svg</a>' if report['original_mission_svg'] else '원본 mission SVG 없음'
    template='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>@@TITLE@@ · 평가 기하 검토</title>
<style>body{margin:0;background:#111a23;color:#e7edf3;font:14px system-ui}header,main{padding:16px;max-width:1500px;margin:auto}header{border-bottom:1px solid #334650}h1{font-size:21px;margin:0 0 8px}a{color:#64dcff}.layout{display:grid;grid-template-columns:minmax(400px,1fr) 340px;gap:14px}.map{background:#0b1119;border:1px solid #34444f;touch-action:none;width:100%;height:650px}aside{background:#17242f;padding:12px;overflow:auto;height:626px}button,select,input{background:#203744;color:#edf8ff;border:1px solid #4b6272;border-radius:4px;padding:6px}input[type=range]{width:55%}.tools{display:flex;gap:12px;flex-wrap:wrap;padding:12px 0}pre{white-space:pre-wrap;font-size:12px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #344650;padding:7px;text-align:left}.legend span{margin-right:15px}.muted{color:#9eb4c5}details{margin:15px 0}@media(max-width:850px){.layout{grid-template-columns:1fr}.map{height:500px}aside{height:auto}}</style>
<header><h1>@@TITLE@@ · 기록 기반 평가 검토</h1><div>원본 경로·corridor·gateway를 재생성하지 않고 저장된 출력만 표시합니다. @@ORIGINAL@@ · <a href="report.json">검증 JSON</a> · <a href="geometry.json">표시 데이터</a> · <a href="overview.svg">전체 SVG</a></div></header><main>
<div class="tools"><button id="home">전체 보기</button><label><input id="follow" type="checkbox" checked>Ego 확대</label><select id="event"><option value="">판정 사건 선택</option></select><select id="leg"><option value="all">전체 mission leg</option></select></div>
<div class="tools"><button id="back">◀</button><input id="slider" type="range" min="0" value="0"><button id="next">▶</button><span id="stamp"></span></div>
<div class="legend"><span style="color:#ffa938">━ 의도한 route</span><span style="color:#ff58ac">━ 실제 주행</span><span style="color:#466977">━ 기록 corridor</span><span style="color:#25ced1">━ 기록 gateway</span><span style="color:#94f078">━ 추종 경로</span><span style="color:#a685ff">━ maneuver</span></div>
<div class="layout"><canvas class="map" id="map"></canvas><aside><div id="info"></div><details><summary>선택 프레임 원문 지표</summary><pre id="detail"></pre></details></aside></div>
<p class="muted">휠: 확대/축소 · 드래그: 이동 · 표시 누락: @@MISSING@@</p>
<details open><summary>평가기 기록 결과 — 정확성이 확인됐다는 뜻은 아닙니다</summary><table><tr><th>항목</th><th>기록 상태</th><th>감점</th><th>근거</th></tr>@@ROWS@@</table></details>
<details><summary>기하·표본화·누락 요약</summary><pre>@@SUMMARY@@</pre><pre>@@SAMPLING@@</pre></details>
<details open><summary>해석 한계</summary><ul>@@WARNINGS@@</ul></details></main><script src="geometry.js"></script><script>
const D=window.GEOMETRY_DATA, C=document.getElementById('map'),ctx=C.getContext('2d'),slider=document.getElementById('slider'),by=id=>document.getElementById(id);let at=0,view=D.bounds.slice(),drag=null;
const fmt=(n,d=2)=>n!==null&&n!==undefined&&n!==''&&Number.isFinite(Number(n))?Number(n).toFixed(d):'미확인';const safe=s=>String(s??'미확인').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
slider.max=Math.max(0,D.frames.length-1);for(const e of D.events){let op=document.createElement('option');op.value=String(e.received_monotonic_ns||'');op.textContent=`${fmt(e.sim_time_s)}s · ${e.event||'감점 항목 '+e.item}`;by('event').append(op)}
for(const leg of [...new Set(D.geometry.map(g=>g.leg))].sort()){let op=document.createElement('option');op.value=leg;op.textContent='mission leg '+leg;by('leg').append(op)}
function transform(){let w=C.clientWidth,h=C.clientHeight;C.width=w*devicePixelRatio;C.height=h*devicePixelRatio;ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);let scale=Math.min(w/(view[2]-view[0]),h/(view[3]-view[1]));return {w,h,scale,x:(view[0]+view[2])/2,y:(view[1]+view[3])/2}}
function draw(){let t=transform();ctx.clearRect(0,0,t.w,t.h);const pt=p=>[t.w/2+(p[0]-t.x)*t.scale,t.h/2-(p[1]-t.y)*t.scale];function line(points,color,width=1,close=false){if(!points||points.length<2)return;ctx.beginPath();points.forEach((p,i)=>{let[x,y]=pt(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});if(close)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();if(close){ctx.fillStyle=color+'35';ctx.fill()}}
for(let g of D.geometry){if(by('leg').value!='all'&&g.leg!=by('leg').value)continue;line(g.points,g.role=='gateway'?'#25ced1':g.role=='preferred'?'#587c85':'#344f5f',g.role=='gateway'?3:1.5)}line(D.route,'#ffa938',2);line(D.trajectory,'#ff58ac',2);
for(let m of D.metadata)line(m.points,'#b1a6d5',1,m.kind=='polygon');for(let c of D.checkpoints){let[x,y]=pt(c.point);ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.font='13px system-ui';ctx.fillText('CP'+c.seq,x+7,y-7)}let f=D.frames[at];if(!f)return;
for(let p of f.candidate_paths)line(p.points,p.status=='rejected'?'#835954':'#3f6583',.8);line(f.tracked_path,'#94f078',3);line(f.maneuver_path,'#a685ff',2);
for(let o of f.objects){let polygon=o.obb;if(!polygon||polygon.length<3)continue;line(polygon,o.collision?'#ff685f':'#c0cfdb',2,true);let[x,y]=pt([o.x,o.y]);ctx.fillStyle='#fff';ctx.font='11px system-ui';ctx.fillText('#'+o.id,x+3,y-3)}line(f.ego.obb,'#ffe66e',3,true);let[x,y]=pt(f.point);ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fillStyle='#ffe66e';ctx.fill();
let a=f.assessment||{},info=`<h3>저장 프레임 ${safe(f.frame)}</h3><p>평가 시각 ${a.sim_time_s==null?'연결 없음':fmt(a.sim_time_s)+'s'}<br>기록 경과 ${fmt(f.time_s)}s · 속도 ${fmt((f.speed||0)*3.6)}km/h<br>위치 (${fmt(f.point[0],3)}, ${fmt(f.point[1],3)})</p><p>road / section / lane: ${safe(f.lane.join(' / '))}<br>mission leg: ${safe(f.active_leg)}<br>route 거리: ${fmt(f.route_distance_m,3)}m<br>기록 corridor lane 포함: ${f.corridor_lane_membership===null?'미확인':f.corridor_lane_membership?'포함':'외부'}<br>평가 clock 차이: ${fmt(f.assessment_clock_error_s,4)}s</p><p>판단: ${safe(f.decision?.stop_reason_text)}<br>계획 유효: ${safe(f.decision?.plan_valid)}<br>주변 객체: ${f.objects.length}개 (제어기 선택)</p>`;by('info').innerHTML=info;by('stamp').textContent=`${at+1}/${D.frames.length} · frame ${f.frame}`;by('detail').textContent=JSON.stringify({decision:f.decision,global_route_metadata:f.global_route_metadata,assessment:a},null,2)}
function select(i){at=Math.max(0,Math.min(D.frames.length-1,i));slider.value=at;let f=D.frames[at];if(f&&by('follow').checked)view=[f.point[0]-40,f.point[1]-40,f.point[0]+40,f.point[1]+40];draw()}slider.oninput=()=>select(Number(slider.value));by('back').onclick=()=>select(at-1);by('next').onclick=()=>select(at+1);by('home').onclick=()=>{by('follow').checked=false;view=D.bounds.slice();draw()};by('follow').onchange=()=>select(at);by('leg').onchange=draw;by('event').onchange=()=>{let target=Number(by('event').value)/1e9;if(target)select(D.frames.reduce((best,f,i)=>Math.abs((f.receive_time_s||0)-target)<Math.abs((D.frames[best].receive_time_s||0)-target)?i:best,0))};
C.onwheel=e=>{e.preventDefault();by('follow').checked=false;let factor=Math.exp(e.deltaY*.001),cx=(view[0]+view[2])/2,cy=(view[1]+view[3])/2,wx=(view[2]-view[0])*factor/2,wy=(view[3]-view[1])*factor/2;view=[cx-wx,cy-wy,cx+wx,cy+wy];draw()};C.onpointerdown=e=>{drag=[e.clientX,e.clientY,view.slice()];C.setPointerCapture(e.pointerId);by('follow').checked=false};C.onpointermove=e=>{if(!drag)return;let scale=Math.min(C.clientWidth/(drag[2][2]-drag[2][0]),C.clientHeight/(drag[2][3]-drag[2][1])),dx=(e.clientX-drag[0])/scale,dy=(e.clientY-drag[1])/scale;view=[drag[2][0]-dx,drag[2][1]+dy,drag[2][2]-dx,drag[2][3]+dy];draw()};C.onpointerup=()=>drag=null;window.onresize=draw;select(location.hash==='#latest'?D.frames.length-1:0);
</script></html>'''
    values={'TITLE':title,'ORIGINAL':original,'MISSING':missing,'ROWS':rows,'WARNINGS':warnings,
            'SUMMARY':html.escape(json.dumps(report['summary'],ensure_ascii=False,indent=2)),
            'SAMPLING':html.escape(json.dumps(report['sampling'],ensure_ascii=False,indent=2))}
    for key,value in values.items():template=template.replace('@@'+key+'@@',value)
    return template


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True);parser.add_argument('--report-dir',type=Path,required=True)
    parser.add_argument('--route-csv',type=Path);parser.add_argument('--checkpoints-csv',type=Path)
    parser.add_argument('--assessment-jsonl',type=Path);parser.add_argument('--metadata',type=Path,action='append',default=[])
    parser.add_argument('--event-time',type=float,action='append',default=[]);parser.add_argument('--max-frames',type=int,default=240)
    args=parser.parse_args()
    result=generate_report(run_dir=args.run_dir,report_dir=args.report_dir,route_csv=args.route_csv,
        checkpoints_csv=args.checkpoints_csv,assessment_jsonl=args.assessment_jsonl,
        metadata_paths=args.metadata,event_times=args.event_time,max_frames=args.max_frames)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
