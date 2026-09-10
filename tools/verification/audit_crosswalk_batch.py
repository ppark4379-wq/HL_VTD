#!/usr/bin/env python3
"""Evidence-based, explicitly provisional checks against the supplied 15 items.

Never substitutes a planner speed cap for a posted limit, or a planner
crosswalk-active flag for physical rear-axle occupancy. No official score.
"""
import argparse
import bisect
import collections
import csv
import gzip
import json
import math
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/dashboard'))
from assessment_geometry import contains, overlaps
from opendrive_assessment import OpenDriveMap

ITEMS=['제한속도 준수','보호구역 속도 준수','차로 유지','중앙선 침범',
       '보도 침범','실선 차로 변경','적색신호 정지','녹색신호 통과',
       '적색점멸 일시정지','보행자 대응','장애물 대응','횡단보도 정차 금지',
       '차로 변경 방향지시등','도로 이용자 충돌','리스폰']

def fnum(row,key,default=0.):
    try:return float(row.get(key,''))
    except (ValueError,TypeError):return default

class Episodes:
    def __init__(self):self.active={};self.events=[]
    def step(self,key,time,active,threshold=0.):
        if active:
            start,last,reported=self.active.get(key,(time,time,False))
            if time-last>.6:start=time;reported=False
            if time-start+1e-6>=threshold and not reported:
                self.events.append(dict(key=key,start=round(start,3),time=round(time,3)))
                reported=True
            self.active[key]=(start,time,reported)
        else:self.active.pop(key,None)

class Auditor:
    def __init__(self,run_id=None,retry_run_id=None):
        zones=yaml.safe_load((ROOT/'config/maps/living_lab.generated.yaml').read_text())
        self.crossings=[]
        for c in zones['crosswalks']:
            poly=[tuple(map(float,v.split(','))) for v in c['polygon'].split(';')]
            self.crossings.append((c['id'],poly,
                (sum(x for x,y in poly)/len(poly),sum(y for x,y in poly)/len(poly))))
        self.lines=zones['stop_lines']
        self.map=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        self.samples={}
        self.lamp_records=collections.defaultdict(list)
        for recorded_id in (run_id,retry_run_id):
            source=ROOT/'output/verification'/str(recorded_id)/'rdb_audit.jsonl.gz'
            if source.exists():
                try:
                    with gzip.open(source,'rt') as stream:
                        for line in stream:
                            d=json.loads(line)
                            if d.get('light') is not None:
                                self.lamp_records[(recorded_id,d['scenario'])].append((d['monotonic_s'],d['light']))
                except (EOFError,OSError,json.JSONDecodeError):pass

    def road_position(self,ego):
        rid=ego['road_id']
        if rid not in self.map.roads:return None
        if rid not in self.samples:
            points=self.map.reference(rid)
            self.samples[rid]=np.asarray(points) if points else np.empty((0,3))
        points=self.samples[rid]
        if not len(points):return None
        i=int(np.argmin((points[:,1]-ego['x'])**2+(points[:,2]-ego['y'])**2))
        projected=self.map.project(rid,(ego['x'],ego['y']),points[i,0])
        return None if projected is None else dict(road=rid,s=projected[0],lane=ego['lane_id'])

    def audit(self,scenario,run_id):
        path=ROOT/'output/verification'/scenario/run_id
        result=dict(scenario=scenario,status='NO_RUN',items={},metrics={},events={})
        result['items']={str(i):dict(name=name,status='UNAVAILABLE',count=None,
            note='유효한 주행 표본 없음') for i,name in enumerate(ITEMS,1)}
        result['source_run_id']=run_id
        console=ROOT/'output/verification'/run_id/f'{scenario}.console.log'
        if console.exists():
            failures=[line for line in console.read_text(errors='replace').splitlines()
                      if '[run_scenario] automation failed:' in line]
            if failures:result['termination_reason']=failures[-1]
        manifest=path/'evidence_manifest.json'
        if manifest.exists():
            evidence=json.loads(manifest.read_text())
            result['automation_status']=evidence.get('status')
            result['controller_sha256']=evidence.get('inputs',{}).get('controller_binary',{}).get('sha256')
        if not (path/'run.csv').exists():
            manifest=path/'evidence_manifest.json'
            if manifest.exists():result['status']=json.loads(manifest.read_text()).get('status','NO_RUN')
            return result
        rows=list(csv.DictReader((path/'run.csv').open()))
        if not rows:return result
        byframe={int(r['frame']):r for r in rows}
        result['status']='GOAL' if any(r['goal_reached']=='1' for r in rows) else 'PARTIAL'
        if (result['status']=='PARTIAL' and result.get('automation_status')=='AUTOMATION_FAILED'
            and 'scenario exceeded --run-timeout' not in result.get('termination_reason','')):
            result['status']='PARTIAL_AUTOMATION_FAILED'
        duration=fnum(rows[-1],'elapsed_wall_time_s')
        result['metrics'].update(duration_s=round(duration,2),
            control_hz=round((len(rows)-1)/max(.001,duration-fnum(rows[0],'elapsed_wall_time_s')),2),
            max_speed_kph=round(max(fnum(r,'ego_speed') for r in rows)*3.6,2),
            invalid_frames=sum(r['plan_valid']!='1' for r in rows),
            collision_flag_frames=sum(r['collision']=='1' for r in rows))
        timing=path/'timing.csv'
        if timing.exists():
            ts=list(csv.DictReader(timing.open()))
            sends=[fnum(r,'send_done_wall_s') for r in ts if fnum(r,'send_done_wall_s')>0]
            if len(sends)>1 and sends[-1]>sends[0]:
                result['metrics'].update(control_hz=round((len(sends)-1)/(sends[-1]-sends[0]),2),
                    send_gap_p95_ms=round(float(np.percentile(np.diff(sends)*1000,95)),2),
                    planning_p95_ms=round(float(np.percentile([fnum(r,'planning_ms') for r in ts],95)),2))
        speed=Episodes();school=Episodes();crossstop=Episodes();pedestrian=Episodes()
        contact=Episodes();center=Episodes();sidewalk=Episodes();lane_drift=Episodes();green=Episodes()
        changes=[];redcross=[];redstops={};jump=[]
        signal_history=collections.deque();previous=None;line_previous={};commands=[]
        measurements=0;geometry_checks=0;occupied_crossing_frames=0;red_observations=0
        green_observations=0;school_observations=0
        crossing_seen=set();first_leg=None;last_leg=None;last_measure=-1.
        source=path/'rviz/frames.jsonl.gz'
        if not source.exists():
            result['items']={str(i+1):dict(name=n,status='UNAVAILABLE',count=None,
                note='RViz 기하 증거 없음') for i,n in enumerate(ITEMS)}
            return result
        try:
            with gzip.open(source,'rt') as stream:
                for raw in stream:
                    d=json.loads(raw);ego=d['ego'];t=d['time_s'];row=byframe.get(d['frame'],{})
                    dec=d['decision'];v=abs(ego['speed'])*3.6;xy=ego['x'],ego['y']
                    commands.append((d['receive_time_s'],dec.get('turn_signal',0)))
                    signal_history.append((t,dec.get('turn_signal',0)))
                    while signal_history and signal_history[0][0]<t-3.:signal_history.popleft()
                    leg=dec.get('active_leg',0)
                    if first_leg is None:first_leg=leg
                    last_leg=leg
                    speed.step('normal_limit_50',t,v>51.,.2)
                    school_observations+=row.get('school_zone_active')=='1'
                    school.step('school_limit_30',t,row.get('school_zone_active')=='1' and v>31.,.2)
                    front=(xy[0]+3.808*math.cos(ego['heading']),xy[1]+3.808*math.sin(ego['heading']))
                    body=[p[:2] for p in ego['obb']]
                    nearby=[c for c in self.crossings if math.dist(xy,c[2])<45]
                    occupied_any=False
                    for cid,poly,_ in nearby:
                        inside=contains(poly,xy)
                        if inside:crossing_seen.add(cid)
                        crossstop.step(str(cid),t,inside and v<=1.,3.)
                        occupied=any(max(o['length'],o['width'])<=2.5 and
                            overlaps(poly,[p[:2] for p in o['obb']]) for o in d['objects'])
                        occupied_any|=occupied
                        occupied_crossing_frames+=int(occupied)
                        # Evidence of travelling ON the crossing while any small
                        # user remains anywhere on it. Approaching before it is OK.
                        pedestrian.step(str(cid),t,occupied and v>1. and
                                        (inside or contains(poly,front)),.2)
                    for o in d['objects']:
                        contact.step(str(o['id']),t,overlaps(body,[p[:2] for p in o['obb']]))
                    if previous:
                        old=previous['ego'];elapsed=t-previous['time_s']
                        if elapsed<=.6 and math.dist(xy,(old['x'],old['y']))>max(5.,ego['speed']*elapsed+3.):
                            jump.append(dict(time=t,metres=math.dist(xy,(old['x'],old['y']))))
                        if (ego['road_id']==old['road_id'] and
                            ego['section_index']==old['section_index'] and
                            ego['lane_id']!=old['lane_id'] and ego['lane_id']*old['lane_id']>0):
                            road=self.map.roads.get(ego['road_id'])
                            if road is not None and road.get('junction')=='-1':
                                direction=(ego['lane_id']-old['lane_id'])*(1 if ego['lane_id']<0 else -1)
                                expected=1 if direction>0 else 2
                                before=[ts for ts,s in signal_history if s==expected and ts<t-.025]
                                pos=self.road_position(ego)
                                mark=self.map.boundary_type(ego['road_id'],pos['s'],old['lane_id'],ego['lane_id']) if pos else None
                                changes.append(dict(time=round(t,3),old=old['lane_id'],new=ego['lane_id'],
                                    expected_signal=expected,pre_signal=bool(before),
                                    lead_s=round(t-min(before),3) if before else None,boundary=mark))
                    candidates=[]
                    for line in self.lines:
                        if line['road_id']!=ego['road_id'] or ego['lane_id'] not in line['lane_ids']:continue
                        if line['controller_id']!=d['traffic_light']['id']:continue
                        x,y=line['begin'];dx=x-front[0];dy=y-front[1]
                        along=dx*math.cos(ego['heading'])+dy*math.sin(ego['heading'])
                        lateral=-dx*math.sin(ego['heading'])+dy*math.cos(ego['heading'])
                        if -8<along<40 and abs(lateral)<2.5:candidates.append((line['id'],along))
                    red=d['traffic_light']['state']==1
                    for lid,distance in candidates:
                        prior=line_previous.get(lid)
                        if red:
                            red_observations+=1
                            if prior and prior[1]>0>=distance and t-prior[0]<.6:
                                redcross.append(dict(line=lid,time=round(t,3),distance=round(distance,3)))
                            # A transient stop during avoidance/following is
                            # not a completed stop-at-red position assessment.
                            if (v<=.55 and dec.get('target_speed',1.)<=.05 and
                                dec.get('stop_reason_text')=='red_light' and
                                not dec.get('emergency_brake')):
                                redstops.setdefault(str(lid),[]).append((t,distance))
                        line_previous[lid]=(t,distance)
                    justified=occupied_any or bool(dec.get('emergency_brake')) or any(
                        o.get('ahead') and math.dist(xy,(o['x'],o['y']))<30 for o in d['objects'])
                    green_near=d['traffic_light']['state']==3 and any(0<distance<30 for _,distance in candidates)
                    green_observations+=green_near
                    green.step('green_stop_review',t,green_near and v<=.55 and not justified,5.)
                    if t-last_measure>=.095:
                        last_measure=t;geometry_checks+=1
                        pos=self.road_position(ego)
                        obj=dict(x=ego['x'],y=ego['y'],h=ego['heading'],length=4.848,width=1.886,off_x=1.384)
                        measured=self.map.measure(obj,pos,1 if ego['lane_id']>0 else -1) if pos else None
                        if measured:
                            measurements+=1
                            center.step('centre',t,measured.get('centerline_intrusion_m',0)>.6,.6)
                            sidewalk.step('sidewalk',t,measured.get('sidewalk_intrusion_m',0)>.5,.3)
                            lane_drift.step('lane_keep_review',t,measured.get('lane_intrusion_m',0)>.1 and
                                dec.get('lane_change_phase')=='none' and not justified,.3)
                    previous=d
        except (EOFError,OSError,json.JSONDecodeError):
            result['snapshot_incomplete']=True
        lamp_mismatch=Episodes();lamp_samples=0;lamp_active_samples=0
        command_times=[c[0] for c in commands]
        for timestamp,actual in self.lamp_records[(run_id,scenario)]:
            current=bisect.bisect_right(command_times,timestamp)-1
            prior=bisect.bisect_right(command_times,timestamp-.3)-1
            if prior<0 or current<0 or timestamp-command_times[current]>.25:continue
            expected=commands[current][1]
            if any(commands[j][1]!=expected for j in range(prior,current+1)):continue
            lamp_samples+=1;lamp_active_samples+=expected!=0
            lamp_mismatch.step('lamp',timestamp,actual!={0:'OFF',1:'LEFT',2:'RIGHT'}.get(expected),.3)
        no_signal=[e for e in changes if not e['pre_signal']]
        solid=[e for e in changes if e['boundary'] and 'solid' in e['boundary']]
        bad_red_stop=[]
        for lid,values in redstops.items():
            if len(values)>1 and values[-1][0]-values[0][0]>=.5:
                closest=min(abs(v[1]) for v in values)
                if closest>=2:bad_red_stop.append(dict(line=lid,closest_m=round(closest,3)))
        result['metrics'].update(crosswalk_stop_episodes=len(crossstop.events),
            crossed_crosswalk_count=len(crossing_seen),pedestrian_conflict_episodes=len(pedestrian.events),
            occupied_crossing_frames=occupied_crossing_frames,lane_changes=len(changes),
            missing_pre_signal=len(no_signal),obb_contact_episodes=len(contact.events),
            red_line_crossings=len(redcross),red_observations=red_observations,
            geometry_samples=measurements,geometry_requested=geometry_checks,
            first_leg=first_leg,last_leg=last_leg)
        result['metrics'].update(lamp_samples=lamp_samples,lamp_active_samples=lamp_active_samples,
                                lamp_mismatch_episodes=len(lamp_mismatch.events))
        result['events']=dict(crosswalk_stop=crossstop.events,pedestrian_conflict=pedestrian.events,
            lane_changes=changes,obb_contacts=contact.events,red_crossings=redcross,
            red_stop_position_review=bad_red_stop,centre=center.events,sidewalk=sidewalk.events,
            green_stop_review=green.events,respawn_review=jump,lamp_mismatch=lamp_mismatch.events)
        def item(i,status,count=None,note=''):
            result['items'][str(i)]=dict(name=ITEMS[i-1],status=status,count=count,note=note)
        for i,ep,note in [(1,speed,'일반 제한 50km/h 가정'),(2,school,'보호구역 30km/h; 기존 측정 지도 기준'),
                          (3,lane_drift,'비회피 차체 경계 0.1m/0.3초, 임시 검토 기준'),
                          (4,center,'차체 0.6m/0.6초'),(5,sidewalk,'차체 0.5m/0.3초')]:
            item(i,'REVIEW' if ep.events else 'NO_EVENT',len(ep.events),note)
        if not school_observations:result['items']['2']['status']='NOT_OBSERVED'
        if not measurements:
            for i in (3,4,5):result['items'][str(i)]['status']='UNAVAILABLE'
        item(6,'REVIEW' if solid else 'NO_EVENT' if changes else 'NOT_OBSERVED',len(solid),'동일 도로·section 차선 변경만 판정')
        item(7,'REVIEW' if redcross or bad_red_stop else 'NO_EVENT' if red_observations else 'NOT_OBSERVED',
             len(redcross)+len(bad_red_stop),'지도 정지선·GT 적색 상태; 중복 선은 별도 검토')
        item(8,'REVIEW' if green.events else 'NO_EVENT' if green_observations else 'NOT_OBSERVED',len(green.events),'객체·비상제동 면제는 임시 근사')
        item(9,'UNAVAILABLE',None,'GT state=6만으로 점멸 색 판별 불가')
        item(10,'REVIEW' if pedestrian.events else 'NO_EVENT' if occupied_crossing_frames else 'NOT_OBSERVED',
             len(pedestrian.events),'전체 횡단보도·작은 객체 OBB; 접근 중 감속과 침범 후 제동은 사건별 검토')
        item(11,'REVIEW' if contact.events else 'NO_EVENT',len(contact.events),'OBB 겹침 대리 지표; 실제 충돌·객체 의미 분류 미확정')
        item(12,'REVIEW' if crossstop.events else 'NO_EVENT' if crossing_seen else 'NOT_OBSERVED',
             len(crossstop.events),'후륜축이 지도 횡단보도 안, 1km/h 이하 3초')
        item(13,'REVIEW' if no_signal else 'NO_EVENT' if changes else 'NOT_OBSERVED',
             len(no_signal),'차선 경계 통과 전 3초 구간의 명령; RDB 실제 지시등 검증은 metrics.lamp_samples 참조')
        if lamp_mismatch.events:result['items']['13']['status']='REVIEW'
        item(14,'REVIEW' if contact.events else 'NO_EVENT',len(contact.events),'11번과 중복 대리 관측; 공식 감점 합산하지 않음')
        item(15,'UNAVAILABLE',len(jump),'좌표 점프만 검토 후보; 공식 respawn 이벤트 없음')
        return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run-id',required=True)
    parser.add_argument('--retry-run-id')
    args=parser.parse_args();out=ROOT/'output/verification'/args.run_id
    auditor=Auditor(args.run_id,args.retry_run_id);results=[]
    for i in range(5,19):
        r=auditor.audit(f'M{i:02}',args.run_id);results.append(r)
        if args.retry_run_id and (ROOT/'output/verification'/f'M{i:02}'/args.retry_run_id/'run.csv').exists():
            retry=auditor.audit(f'M{i:02}',args.retry_run_id)
            if r['status'] in ('NO_RUN','AUTOMATION_FAILED'):
                retry['first_attempt_status']=r['status'];r=retry;results[-1]=r
            else:r['recheck']=retry
        print(r['scenario'],r['status'],r['metrics'],flush=True)
    (out/'evaluation.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
    with (out/'evaluation_15_items.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['scenario','run_status','run_id','item','name','status','count','note'])
        for r in results:
            for attempt in [r]+([r['recheck']] if r.get('recheck') else []):
                for k,v in attempt['items'].items():writer.writerow([attempt['scenario'],attempt['status'],attempt['source_run_id'],k,v['name'],v['status'],v['count'],v['note']])
    text=['# M05~M18 횡단보도·방향지시등 회귀 평가','',
          '첨부 PDF 항목을 따른 자체 관측표이며 공식 점수가 아니다. NO_EVENT는 실행한 구간에서 미검출이지 전체 안전 보장이 아니다.',
          'PARTIAL은 실행 시간 제한 등으로 목표 도착을 확인하지 못한 경우다. 미실행/관측 불가 항목을 통과로 처리하지 않는다.','',
          'PARTIAL_AUTOMATION_FAILED는 일부 주행 후 VTD/실행기 오류로 중단된 경우다. 후반 시나리오의 짧은 재실행도 완주 검증이 아니다.','',
          '|시나리오|도착|실행 초|Hz|횡단보도 3초 정지|보행자 동시 진입 검토|지시등 누락/차선변경|OBB 접촉 검토|',
          '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in results:
        m=r['metrics'];get=lambda k:m.get(k,'—')
        text.append(f"|{r['scenario']}|{r['status']}|{get('duration_s')}|{get('control_hz')}|{get('crosswalk_stop_episodes')}|{get('pedestrian_conflict_episodes')}|{get('missing_pre_signal')}/{get('lane_changes')}|{get('obb_contact_episodes')}|")
    text+=['','전체 15항목: [evaluation_15_items.csv](evaluation_15_items.csv). 사건 시각과 근거: [evaluation.json](evaluation.json).',
           '차선 변경은 동일 road/section에 한해 측정. 보행자/접촉은 GT 의미 분류가 없어 치수와 OBB로 근사한다.',
           '제한속도는 일반 50·보호구역 30km/h 가정. 구간별 공식 경계·리스폰 신호 미제공으로 공식 합산 점수는 산출하지 않는다.',
           '실제 지시등은 RDB 수집이 있는 구간에서만 별도 대조한다. 0개 표본은 검증 불가이지 정상 판정이 아니다.']
    text+=['','주의: 후륜축 3초 정차 0회는 앞범퍼 침범 0회를 뜻하지 않는다. 보행자 미노출 구간의 0회도 대응 검증 완료가 아니다.',
           'M18의 늦은 출발 트리거·앞범퍼 침범과 기타 미해결 사건은 [원인 및 한계 보고서](../../../docs/crosswalk_indicator_audit_20260910.md)를 함께 확인한다.']
    text+=['','## 실행 코드 및 추가 재검증','',
           '|시나리오|실행 코드 SHA 앞 12자리|추가 재검증|','|---|---|---|']
    for r in results:
        retry=r.get('recheck');m=retry['metrics'] if retry else {}
        note=(f"{m.get('duration_s')}초 / 지시등 누락 {m.get('missing_pre_signal')}/{m.get('lane_changes')} / 횡단보도 정지 {m.get('crosswalk_stop_episodes')} / 코드 {retry.get('controller_sha256','')[:12]}" if retry else '—')
        text.append(f"|{r['scenario']}|{str(r.get('controller_sha256',''))[:12]}|{note}|")
    (out/'report.md').write_text('\n'.join(text)+'\n')

if __name__=='__main__':main()
