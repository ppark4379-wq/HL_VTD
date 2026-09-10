#!/usr/bin/env python3
"""Passive lamp/object-class evidence on RDB; never sends control commands."""
import argparse
import datetime as dt
import gzip
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/dashboard'))
from rdb_stream import RdbObserver

def main():
    p=argparse.ArgumentParser();p.add_argument('--run-id',required=True)
    p.add_argument('--deadline-utc',required=True);args=p.parse_args()
    end=dt.datetime.fromisoformat(args.deadline_utc).timestamp()
    out=ROOT/'output/verification'/args.run_id/'rdb_audit.jsonl.gz'
    profile={};last_profile=0.;count=0;lamp_count=0
    with gzip.open(out,'xt',compresslevel=1) as stream:
        def frame(d):
            nonlocal profile,last_profile,count,lamp_count
            if time.monotonic()-last_profile>.5:
                try:profile=json.loads(Path('/tmp/hl_vtd_active_assessment.json').read_text())
                except (OSError,ValueError):profile={}
                last_profile=time.monotonic()
            if profile.get('run_id')!=args.run_id:return
            ego=next((o for o in d['objects'].values() if o['name']=='Ego'),None)
            if ego is None:return
            light=d['lights'].get(ego['id'])
            stream.write(json.dumps(dict(scenario=profile.get('scenario_id'),frame=d['frame'],
                sim_time=d['time'],monotonic_s=d['received_monotonic_ns']/1e9,
                ego=ego,light=light,road=d['road'].get(ego['id']),
                objects=list(d['objects'].values())),separators=(',',':'))+'\n')
            count+=1;lamp_count+=light is not None
            if count%200==0:stream.flush()
        observer=RdbObserver('127.0.0.1',48190,frame,lambda:None);observer.start()
        try:
            while time.time()<end:time.sleep(.5)
        finally:observer.close()
    print(json.dumps(dict(frames=count,lamp_frames=lamp_count,path=str(out))),flush=True)

if __name__=='__main__':main()
