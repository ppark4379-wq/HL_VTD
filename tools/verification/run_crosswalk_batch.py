#!/usr/bin/env python3
"""Bounded sequential VTD regression; no scenario/control parameter overrides."""
import argparse
import datetime as dt
import json
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[2]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--deadline-utc',required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--scenarios',nargs='+',type=int,default=list(range(5,19)))
    parser.add_argument('--max-runtime',type=int,default=600)
    args=parser.parse_args()
    if any(i not in range(5,19) for i in args.scenarios):
        parser.error('scenarios must be between 5 and 18')
    deadline=dt.datetime.fromisoformat(args.deadline_utc).timestamp()
    out=ROOT/'output/verification'/args.run_id
    out.mkdir(parents=True,exist_ok=False)
    results=[]
    # Prior observed durations allocate the fixed user budget, not driving rules.
    weights={5:240,6:190,7:220,8:185,9:285,10:325,11:160,
             12:300,13:185,14:350,15:285,16:490,17:190,18:190}
    for index,scene in enumerate(args.scenarios):
        left=len(args.scenarios)-index
        remaining=deadline-time.time()
        if remaining<25:
            results.append(dict(scenario=f'M{scene:02}',status='NOT_RUN_DEADLINE'))
            continue
        available=max(10,remaining-18*left)
        runtime=max(30,min(args.max_runtime,weights[scene]+60,
            int(available*weights[scene]/sum(weights[i] for i in args.scenarios[index:]))))
        name=f'M{scene:02}'
        command=[sys.executable,'-u','tools/verification/run_scenario.py',
                 f'scenario/mixed/{name}','--auto-vtd','--skip-source-sync',
                 '--run-id',args.run_id,'--run-timeout',str(runtime),
                 '--vtd-timeout','25','--evidence']
        item=dict(scenario=name,status='RUNNING',runtime_limit_s=runtime,
                  started_at=dt.datetime.now(dt.timezone.utc).isoformat(),
                  command=command)
        results.append(item)
        (out/'batch.json').write_text(json.dumps(results,indent=2))
        print(f'BEGIN {name} runtime_limit={runtime}s remaining={remaining:.0f}s',flush=True)
        started=time.time()
        with (out/f'{name}.console.log').open('w') as log:
            result=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        item.update(status='FINISHED',exit_code=result.returncode,
                    wall_seconds=round(time.time()-started,3))
        (out/'batch.json').write_text(json.dumps(results,indent=2))
        print(f'END {name} exit={result.returncode} wall={item["wall_seconds"]}',flush=True)
    (out/'batch.json').write_text(json.dumps(results,indent=2))

if __name__=='__main__':
    main()
