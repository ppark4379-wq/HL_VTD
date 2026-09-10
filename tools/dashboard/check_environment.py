#!/usr/bin/env python3
"""Read-only preflight for the VTD PC. Does not send controller commands."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import socket
import time
from live_assessment import LiveAssessment
from rdb_stream import Decoder
import server


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assessment-config',type=Path,default=Path(__file__).with_name('assessment_rules.yaml'))
    parser.add_argument('--rdb-host',default='127.0.0.1')
    parser.add_argument('--rdb-port',type=int,default=48190)
    parser.add_argument('--probe-rdb',action='store_true',help='passively receive for up to 3 seconds; run while VTD is playing')
    args=parser.parse_args()
    if args.rdb_port==9910: parser.error('9910 is the controller port; use an RDB output port')
    report={'display':os.environ.get('DISPLAY'),'session_type':os.environ.get('XDG_SESSION_TYPE'),
            'ffmpeg':shutil.which('ffmpeg'),'xwininfo':shutil.which('xwininfo')}
    try:
        engine=LiveAssessment(args.assessment_config,Path(__file__).resolve().parents[2]/'output/assessment')
        payload=engine.snapshot()
        report.update(config_valid=True,map_loaded=engine.map is not None,
                      missing_metadata=payload['missing_metadata'],pending_lane_keep=payload['pending_criteria'],
                      start_trigger=payload['start_source'],control_monitor_required=True,
                      completion=payload['completion'],finish_line=payload['finish_line'],finish_candidates=payload['finish_candidates'],
                      school_zone_validation=payload['school_zone_validation'],feature_counts=payload['feature_counts'],
                      respawn_detection=(
                          'CONFIRMED_EVENT_SOURCE'
                          if engine.respawn_detection_confirmed
                          else 'REVIEW_ONLY_COORDINATE_JUMP'
                          if engine.session.get('respawn_jump_m') is not None
                          else 'UNCONFIGURED'
                      ),
                      object_feed_complete=engine.meta.get('object_feed_complete',False))
        server.VIDEO_RTSP_URL=server.configured_rtsp_url()
        server.VIDEO_RTSP_URLS={"mainrs":server.VIDEO_RTSP_URL,"birdrs":None}
        views={}
        for view in ("mainrs","birdrs"):
            command,source,error=server.video_capture_command(view)
            views[view]={'candidate':source,'error':error,'capture_started':False}
        # Preserve the former top-level mainRS fields for existing consumers.
        report['video']={**views['mainrs'],'views':views}
        if args.probe_rdb:
            decoder=Decoder(); end=time.monotonic()+3; sample=None
            try:
                with socket.create_connection((args.rdb_host,args.rdb_port),timeout=2) as conn:
                    conn.settimeout(.5)
                    while time.monotonic()<end and sample is None:
                        try: data=conn.recv(262144)
                        except socket.timeout: continue
                        if not data: break
                        frames=decoder.feed(data)
                        if frames: sample=frames[-1]
                report['rdb']={'complete_frame_received':sample is not None}
                if sample:
                    ego=next((o for o in sample['objects'].values() if o['name']==engine.meta.get('ego_name','Ego')),None)
                    report['rdb'].update(frame=sample['frame'],sim_time=sample['time'],packages=sorted(set(sample['packages'])),
                                         ego_found=ego is not None,extended_speed_available=ego is not None and ego['speed'] is not None,
                                         object_count=len(sample['objects']),signal_count=len(sample['signals']))
            except OSError as error: report['rdb']={'complete_frame_received':False,'error':str(error)}
        engine.close()
    except (ValueError,OSError,KeyError) as error:
        report.update(config_valid=False,error=str(error))
    # This probe cannot certify geometry, video capture or real-world scoring accuracy.
    report['acceptance']='VTD on-site end-to-end verification required; not a certification'
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report.get('config_valid') else 2


if __name__=='__main__': raise SystemExit(main())
