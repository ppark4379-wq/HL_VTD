#!/usr/bin/env python3
"""Real VTD M05-M15 audit; preserve every run and stop after four failures."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]


def poll_logged_collision(path, cursor):
    """Read every completed CSV row, including overlaps shorter than a poll."""
    try:
        with path.open() as stream:
            header=next(csv.reader([stream.readline()]))
            if 'collision' not in header:return False,cursor
            index=header.index('collision')
            if cursor:stream.seek(cursor)
            collision=False
            while True:
                position=stream.tell();line=stream.readline()
                if not line or not line.endswith('\n'):return collision,position
                values=next(csv.reader([line]))
                if len(values)>index and values[index]=='1':collision=True
    except (OSError,StopIteration):return False,cursor


def summarize(directory):
    path=directory/'run.csv'
    if not path.exists():return {'frames':0,'severe':True,'reason':'no_log'}
    with path.open() as f:rows=list(csv.DictReader(f))
    def num(row,key):
        try:return float(row.get(key,'') or 0)
        except (ValueError,TypeError):return 0.
    deltas=[];rates=[];desired=[]
    for a,b in zip(rows,rows[1:]):
        dt=num(b,'elapsed_wall_time_s')-num(a,'elapsed_wall_time_s')
        if 0<dt<.5:
            deltas.append(abs(num(b,'command_steering')-num(a,'command_steering')))
            rates.append(deltas[-1]/dt)
            if num(b,'ego_speed')>.5:desired.append(abs(num(b,'planned_steering')-num(a,'planned_steering')))
    collision=[r for r in rows if num(r,'collision')>0]
    collision_episodes=[]
    active_collision=None
    for row in rows:
        frame=int(num(row,'frame'))
        if num(row,'collision')>0:
            if active_collision is None:active_collision=[frame,frame]
            else:active_collision[1]=frame
        elif active_collision is not None:
            collision_episodes.append(active_collision);active_collision=None
    if active_collision is not None:collision_episodes.append(active_collision)
    goal=any(num(r,'goal_reached')>0 for r in rows)
    last={}
    try:last=json.loads((directory/'rviz/latest.json').read_text())
    except (OSError,ValueError):pass
    decisions=(directory/'decisions.log').read_text(errors='replace') if (directory/'decisions.log').exists() else ''
    selected=[line for line in decisions.splitlines() if '[corridor_lattice] selected' in line]
    rejected=[line for line in decisions.splitlines() if '[corridor_lattice] no_safe_candidate' in line]
    travelled=sum(math.hypot(num(b,'ego_x')-num(a,'ego_x'),num(b,'ego_y')-num(a,'ego_y')) for a,b in zip(rows,rows[1:]))
    return dict(frames=len(rows),duration_s=num(rows[-1],'elapsed_wall_time_s') if rows else 0.,
                travelled_m=travelled,goal_reached=goal,collision_frames=len(collision),
                collision_episodes=len(collision_episodes),
                collision_episode_frames=collision_episodes,
                first_collision_s=num(collision[0],'elapsed_wall_time_s') if collision else None,
                max_command_steering_rad=max((abs(num(r,'command_steering')) for r in rows),default=0),
                max_command_step_rad=max(deltas,default=0),max_wall_time_steering_rate=max(rates,default=0),
                desired_step_over_02_rad=sum(d>.2 for d in desired),
                max_desired_step_rad=max(desired,default=0),
                invalid_plan_frames=sum(num(r,'plan_valid')==0 for r in rows),
                avoidance_selections=len(selected),no_candidate_frames=len(rejected),
                last_mission={'active_leg':last.get('decision',{}).get('active_leg')},last_decision=last.get('decision',{}),
                # Judge the command actually sent to VTD.  Raw pure-pursuit
                # steering can jump when its lookahead point crosses a path
                # sample boundary, but the controller's rate limiter removes
                # that jump before transmission.  Treat the raw value as a
                # diagnostic warning, not as a failed drive.
                severe=len(collision_episodes)>1 or not goal or max(deltas,default=0)>.1,
                raw_desired_steering_warning=sum(d>.2 for d in desired)>=4,
                collision_definition='logged current OBB overlap; not native VTD contact telemetry',
                steering_definition='command step and desired step; wall-time rate is not simulation-time rate')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--tag',required=True)
    parser.add_argument('--first',type=int,default=5)
    parser.add_argument('--last',type=int,default=15)
    parser.add_argument('--timeout',type=float,default=420)
    args=parser.parse_args()
    dest=ROOT/'output/verification'/args.tag;dest.mkdir(parents=True,exist_ok=True)
    report=dict(tag=args.tag,binary_sha256=hashlib.sha256((ROOT/'build/hl_vtd').read_bytes()).hexdigest(),runs=[],failure_limit=4)
    network=ROOT/'output/precomputed_lane_network'
    manifest=json.loads((network/'manifest.json').read_text())
    actual=hashlib.sha256((ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr').read_bytes()).hexdigest()
    if actual!=manifest['source_sha256']:raise SystemExit('XODR/precomputed hash mismatch')
    report['source_sha256']=actual
    failures=0
    for number in range(args.first,args.last+1):
        scenario=f'M{number:02d}';run_id=f'{args.tag}_{scenario.lower()}'
        directory=ROOT/'output/verification'/scenario/run_id
        if (directory/'run.csv').exists():raise SystemExit(f'Refusing to overwrite {directory}')
        directory.mkdir(parents=True,exist_ok=True)
        if hashlib.sha256((ROOT/'build/hl_vtd').read_bytes()).hexdigest()!=report['binary_sha256']:
            raise SystemExit('Binary changed during evaluation; start a separately labelled batch')
        command=[sys.executable,str(ROOT/'tools/verification/run_scenario.py'),str(ROOT/'scenario/mixed'/scenario),
                 '--auto-vtd','--inline-scenario','--skip-source-sync','--run-id',run_id,'--run-timeout',str(args.timeout),'--rviz-data',
                 '--decision-log',str(directory/'decisions.log')]
        print('START',scenario,run_id,flush=True)
        reason='';invalid_since=None;stopped_since=None;started=time.monotonic();csv_cursor=0
        with (directory/'console.log').open('w') as log:
            process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            while process.poll() is None:
                now=time.monotonic()
                try:
                    data=json.loads((directory/'rviz/latest.json').read_text())
                    decision=data.get('decision',{});ego=data.get('ego',{})
                    if decision.get('plan_valid') is False:
                        invalid_since=invalid_since or now
                    else:invalid_since=None
                    if abs(ego.get('speed',1.))<.1 and decision.get('stop_reason',0)==0 and decision.get('target_speed',1.)<.5 and not decision.get('emergency_brake',False):
                        stopped_since=stopped_since or now
                    else:stopped_since=None
                    # Preserve the run through one allowed contact episode;
                    # summarize() evaluates the completed collision episodes.
                    if invalid_since and now-invalid_since>15:reason='invalid_plan_stall_15s'
                    if stopped_since and now-stopped_since>30:reason='non_traffic_zero_target_30s'
                except (OSError,ValueError):pass
                # One contact episode is tolerated by the current competition
                # target.  Preserve the complete run and count independent
                # episodes in summarize() instead of aborting on the first
                # overlapping CSV frame.
                _,csv_cursor=poll_logged_collision(directory/'run.csv',csv_cursor)
                if now-started>args.timeout+100:reason='runner_watchdog'
                if reason:
                    os.kill(process.pid,signal.SIGINT)
                    try:process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        # Only this task's process group; runner normally stops
                        # VTD in its finally block after SIGINT.
                        os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
                    break
                time.sleep(.5)
        result=summarize(directory)
        result.update(scenario=scenario,run_id=run_id,returncode=process.returncode,watchdog_reason=reason,
                      log_directory=str(directory))
        environment_failure=result['frames']==0
        result['environment_failure']=environment_failure
        failures+=bool(result['severe']) and not environment_failure
        report['runs'].append(result);report['failure_count']=failures
        report['paused_after_failure_limit']=failures>=4
        (dest/'results.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        print('DONE',scenario,json.dumps(result,ensure_ascii=False),flush=True)
        if environment_failure:
            print('PAUSED: no driving data; repair simulator setup before evaluating.',flush=True);break
        if failures>=4:
            print('PAUSED: four failing scenarios; diagnose before more validation.',flush=True);break
    print('REPORT',dest/'results.json',flush=True)


if __name__=='__main__':main()
