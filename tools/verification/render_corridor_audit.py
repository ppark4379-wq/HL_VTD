#!/usr/bin/env python3
"""Plot preserved run data; no simulator or source changes."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('report',type=Path)
    args=parser.parse_args();report=json.loads(args.report.read_text())
    runs=[r for r in report['runs'] if r.get('frames',0)]
    fig,axes=plt.subplots(3,len(runs),figsize=(5*len(runs),11),squeeze=False)
    for column,run in enumerate(runs):
        with (Path(run['log_directory'])/'run.csv').open() as stream:rows=list(csv.DictReader(stream))
        def values(key):return [float(r.get(key) or 0) for r in rows]
        t=values('elapsed_wall_time_s');x=values('ego_x');y=values('ego_y')
        ax=axes[0,column];ax.plot(x,y,color='#1178af',lw=1.5)
        ax.scatter([x[0]],[y[0]],c='green',s=30,label='Start')
        ax.scatter([x[-1]],[y[-1]],c='black',s=30,label='End')
        overlaps=[i for i,r in enumerate(rows) if r.get('collision')=='1']
        if overlaps:ax.scatter([x[i] for i in overlaps],[y[i] for i in overlaps],c='red',s=25,label='OBB overlap')
        ax.set_aspect('equal',adjustable='datalim');ax.set_title(f"{run['scenario']} - NOT COMPLETED");ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.legend(fontsize=8)
        ax=axes[1,column];ax.plot(t,values('ego_speed'),label='Actual');ax.plot(t,values('target_speed'),label='Target',alpha=.7)
        ax.set_ylabel('Speed (m/s)');ax.set_xlabel('Wall time (s)');ax.legend(fontsize=8)
        ax=axes[2,column];ax.plot(t,values('planned_steering'),label='Planned',alpha=.7);ax.plot(t,values('command_steering'),label='Command',lw=.8)
        ax.set_xlabel('Wall time (s)');ax.set_ylabel('Steering (rad)');ax.legend(fontsize=8)
        for row in range(3):axes[row,column].grid(alpha=.2)
    fig.suptitle(report['tag']+' | Four failures: further scenarios paused',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.96])
    for extension in ('svg','png'):
        path=args.report.parent/f'driving_audit.{extension}';fig.savefig(path,dpi=130);print(path)


if __name__=='__main__':main()
