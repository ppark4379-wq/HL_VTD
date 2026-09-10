"""Boundary, lifecycle and wire-level tests for the passive assessment dashboard."""
import copy
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import threading
import socket
import time
import urllib.request
from http.server import ThreadingHTTPServer
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/dashboard'))
sys.path.insert(0,str(ROOT/'tools/verification'))
import evaluate as ev
import run_repository as repository
from live_assessment import LiveAssessment
from rdb_stream import Decoder, RdbObserver
from assessment_geometry import obb, overlaps, crosses, validate_metadata
from video_relay import VideoRelay
from opendrive_assessment import OpenDriveMap
from route_assessment import RouteProgress
from school_zones import SchoolZones
from control_monitor import CommandDecoder, ControlEvents
from map_features import extract_features, finish_from_object


def row(t=0,section='1',**kw):
    data=dict(scenario_id='MIXED',run_id='test',frame=t*100,elapsed_wall_time_s=t,
              evaluation_section_id=section,ego_speed=0,nearest_object_distance='',goal_reached=0,
              command_steering='',target_accel='')
    data.update(kw)
    return {k:str(int(v)) if isinstance(v,bool) else str(v) for k,v in data.items()}


def series(seconds, **kw):
    return [row(i/10,**kw) for i in range(round(seconds*10)+1)]


def obj(x=0,y=0,speed=1,ident=1,name='Ego',kind=1):
    return dict(id=ident,name=name,category=1,type=kind,x=x,y=y,z=0,h=0,p=0,r=0,
                length=4,width=2,height=1.5,off_x=1,off_y=0,off_z=.75,speed=speed)


def frame(t,x=0,speed=1,number=None):
    return dict(frame=round(t*100) if number is None else number,time=t,version=0x126,
                objects={1:obj(x=x,speed=speed)},road={},lanes={},lights={},signals={},packages=[9,2])


class CsvReadRobustnessTests(unittest.TestCase):
    def test_concurrent_partial_tail_is_ignored_but_middle_corruption_is_not(self):
        columns=sorted(ev.REQUIRED_COLUMNS)
        complete={name:'0' for name in columns}
        complete.update(scenario_id='M15',run_id='r',frame='1')
        header=','.join(columns)+'\n'
        row_text=','.join(complete[name] for name in columns)+'\n'
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'run.csv'
            path.write_text(header+row_text+'M15,r,2',encoding='utf-8')
            rows=ev.load_rows(path)
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0]['frame'],'1')
            path.write_text(
                header+'M15,r,0\n'+row_text,
                encoding='utf-8',
            )
            with self.assertRaisesRegex(ValueError,'잘린 데이터 행'):
                ev.load_rows(path)

    def test_history_path_components_cannot_escape_verification_root(self):
        for value in ('..','../assessment','x/y',''):
            with self.subTest(value=value):
                if value:
                    with self.assertRaises(ValueError):
                        repository.list_runs(value)
                else:
                    self.assertEqual(repository.list_runs(value),[])
        with self.assertRaises(ValueError):
            repository.read_run_rows('M15','../../secret')


class CriteriaTests(unittest.TestCase):
    def test_speed_edges_and_upgrade_once(self):
        for kph,expected in ((51,0),(51.01,-3),(70,-3),(70.01,-6)):
            self.assertEqual(ev.evaluate_speed_limit([row(ego_speed=kph/3.6)],50).penalty,expected)
        rows=[row(i*.1,ego_speed=v/3.6) for i,v in enumerate([60,60,50,60,75,75])]
        self.assertEqual(ev.evaluate_speed_limit(rows,50).penalty,-6)
        rows.append(row(1,section='2',ego_speed=75/3.6))
        self.assertEqual(ev.evaluate_speed_limit(rows,50).penalty,-12)

    def test_planner_speed_cap_is_not_treated_as_posted_limit(self):
        planner_cap = row(
            ego_speed=40/3.6,
            current_speed_limit_kmh=5.4,
            speed_limit_source='planner_behavior_cap',
        )
        self.assertEqual(ev.evaluate_speed_limit([planner_cap], 50).penalty, 0)
        posted = row(ego_speed=40/3.6, posted_speed_limit_kmh=30)
        self.assertEqual(ev.evaluate_speed_limit([posted], 50).penalty, -3)

    def test_school_edges(self):
        for kph,expected in ((31,0),(31.01,-3),(35,-3),(35.01,-6)):
            self.assertEqual(ev.evaluate_school_zone([row(ego_speed=kph/3.6,school_zone_active=True)],30).penalty,expected)

    def test_intrusion_starts_at_first_intruding_sample(self):
        rows=[row(0,centerline_intrusion_m=0)]+[row(.1+i*.1,centerline_intrusion_m=.6) for i in range(6)]
        self.assertEqual(ev.evaluate_sustained_intrusion(rows,4,'centerline_intrusion_m',.6,.6).penalty,0)
        rows.append(row(.7,centerline_intrusion_m=.6))
        self.assertEqual(ev.evaluate_sustained_intrusion(rows,4,'centerline_intrusion_m',.6,.6).penalty,-6)
        self.assertEqual(ev.evaluate_sustained_intrusion(series(.3,sidewalk_intrusion_m=.5),5,'sidewalk_intrusion_m',.5,.3).penalty,-6)

    def test_gaps_and_section_transitions_do_not_add_duration(self):
        rows=[row(0,centerline_intrusion_m=.6),row(100,centerline_intrusion_m=.6)]
        self.assertEqual(ev.evaluate_sustained_intrusion(rows,4,'centerline_intrusion_m',.6,.6).penalty,0)
        rows=series(.5,centerline_intrusion_m=.6)+[row(.6,section='2',centerline_intrusion_m=.6)]
        self.assertEqual(ev.evaluate_sustained_intrusion(rows,4,'centerline_intrusion_m',.6,.6).penalty,0)

    def test_lane_keep_is_excluded_and_solid_count_edges(self):
        rows=[row(i/10,solid_line_crossing=x) for i,x in enumerate([0,1,1,0,1,0,1])]
        self.assertEqual(ev.evaluate_event_count(rows,8,'solid_line_crossing').penalty,-6)
        self.assertNotIn(3,[item.number for item in ev.evaluate_official_for(rows)])
        self.assertEqual(len(ev.evaluate_official_for(rows)),14)

    def test_red_exact_two_and_crossing(self):
        for distance,penalty in ((1.999,0),(2,-3),(2.001,-3)):
            rows=series(.5,traffic_light_state='RED',stop_line_id='L',stop_line_distance_m=distance)
            self.assertEqual(ev.evaluate_signals(rows)[0].penalty,penalty)
        rows += [row(.6,traffic_light_state='RED',stop_line_id='L',stop_line_distance_m=-.01),
                 row(.7,traffic_light_state='RED',stop_line_id='L',stop_line_distance_m=1)]
        self.assertEqual(ev.evaluate_signals(rows)[0].penalty,-6)

    def test_green_five_ten_and_reason(self):
        data=dict(traffic_light_state='GREEN',stop_line_id='L',stop_line_distance_m=10,justified_stop=False)
        self.assertEqual(ev.evaluate_signals(series(4.9,**data))[1].penalty,0)
        self.assertEqual(ev.evaluate_signals(series(5,**data))[1].penalty,-3)
        self.assertEqual(ev.evaluate_signals(series(10,**data))[1].penalty,-6)
        self.assertEqual(ev.evaluate_signals(series(10,**{**data,'justified_stop':True}))[1].penalty,0)
        self.assertEqual(ev.evaluate_signals(series(10,ego_speed=.001,**data))[1].penalty,0)

    def test_flashing_stop_less_than_two_normal_exact_two_minor(self):
        data=dict(traffic_light_state='FLASHING_RED',stop_line_id='L',stop_line_distance_m=1.999)
        rows=series(.5,**data)+[row(.6,**{**data,'stop_line_distance_m':-.1})]
        self.assertEqual(ev.evaluate_signals(rows)[2].penalty,0)
        data['stop_line_distance_m']=2
        rows=series(.5,**data)+[row(.6,**{**data,'stop_line_distance_m':-.1})]
        self.assertEqual(ev.evaluate_signals(rows)[2].penalty,-3)
        rows=[row(0,ego_speed=1,**data),row(.1,ego_speed=1,**{**data,'stop_line_distance_m':-.1})]
        self.assertEqual(ev.evaluate_signals(rows)[2].penalty,-6)

    def test_crosswalk(self):
        fields=dict(crosswalk_active=True,crosswalk_source='map_polygon_rear_axle',ego_speed=1/3.6)
        self.assertEqual(ev.evaluate_crosswalk_stop(series(2.9,**fields)).penalty,0)
        self.assertEqual(ev.evaluate_crosswalk_stop(series(3,**fields)).penalty,-3)
        planner=series(5,crosswalk_active=True,crosswalk_source='planner_behavior_control',ego_speed=0)
        self.assertEqual(ev.evaluate_crosswalk_stop(planner).status,'UNAVAILABLE')

    def test_collision_uses_type_not_scenario_prefix(self):
        rows=[row(static_collision=True,road_user_collision=False)]
        results=ev.evaluate_collision(rows,'V01')
        self.assertEqual([r.penalty for r in results],[-3,0])
        self.assertEqual(ev.evaluate_collision([row(collision=True)],'O01')[0].status,'UNAVAILABLE')

    def test_pedestrian_and_indicator_fields(self):
        r=ev.evaluate_official_for([row(pedestrian_violation=True,turn_signal_violation=True)])
        by={item.number:item for item in r}
        self.assertEqual(by[14].penalty,-6)
        self.assertEqual(by[7].penalty,-3)

    def test_respawn_unbounded_from_initial_500(self):
        rows=[row(respawn_count=30)]
        result=ev.evaluate_respawn(rows)
        self.assertEqual(result.penalty,-174)
        self.assertEqual(result.penalty_by_section,{'1':174})
        args=type('Args',(),dict(official=True,require_goal=False,min_object_distance=None,max_abs_steering=None,max_abs_accel=None,speed_limit_kmh=50,school_zone_limit_kmh=30))()
        summary=ev.summarize(rows,args)
        self.assertIsNone(summary['official_score'])
        self.assertEqual(summary['evidence_only_score'],'326')


class WireTests(unittest.TestCase):
    def packet(self):
        data=bytearray(208)
        struct.pack_into('<IBBH',data,0,1,1,1,0)
        data[8:12]=b'Ego\0'
        struct.pack_into('<6f',data,40,4,2,1.5,1,0,.75)
        struct.pack_into('<dddfffBBH',data,64,10,20,0,0,0,0,3,0,0)
        struct.pack_into('<dddfffBBH',data,112,3,4,0,0,0,0,1,0,0)
        entry=struct.pack('<IIIHH',16,len(data),len(data),9,1)+data
        entry+=struct.pack('<IIIHH',16,0,0,2,0)
        return struct.pack('<HHIIId',35712,0x126,24,len(entry),5,.1)+entry

    def test_fragmented_wire_and_resynchronization(self):
        decoder=Decoder()
        packet=self.packet()
        self.assertEqual(decoder.feed(b'bad'+packet[:19]),[])
        frames=decoder.feed(packet[19:])
        self.assertEqual(len(frames),1)
        self.assertEqual(frames[0]['objects'][1]['speed'],5)
        self.assertEqual(frames[0]['objects'][1]['off_x'],1)

    def test_invalid_position_is_not_scored(self):
        packet=bytearray(self.packet())
        packet[24+16+100]=0
        self.assertFalse(Decoder().feed(packet)[0]['objects'])

    def test_malformed_end_frame_is_discarded(self):
        packet=bytearray(self.packet())
        struct.pack_into('<I',packet,8,len(packet)-24+1)
        self.assertEqual(Decoder().feed(packet+b'!'),[])

    def test_read_only_tcp_observer(self):
        listener=socket.socket()
        listener.bind(('127.0.0.1',0)); listener.listen(1)
        received=threading.Event(); frames=[]; sent_by_observer=[]
        def serve():
            with listener.accept()[0] as conn:
                conn.sendall(self.packet())
                conn.settimeout(.2)
                try: sent_by_observer.append(conn.recv(1024))
                except socket.timeout: pass
        thread=threading.Thread(target=serve,daemon=True); thread.start()
        def collect(f): frames.append(f); received.set()
        observer=RdbObserver('127.0.0.1',listener.getsockname()[1],collect,lambda:None)
        observer.start()
        try:
            self.assertTrue(received.wait(3))
            self.assertEqual(frames[0]['objects'][1]['name'],'Ego')
            self.assertIsInstance(frames[0]['received_wall_time_ns'],int)
            self.assertIsInstance(frames[0]['received_monotonic_ns'],int)
            self.assertGreater(frames[0]['received_wall_time_ns'],0)
            self.assertGreater(frames[0]['received_monotonic_ns'],0)
            thread.join(1)
            self.assertFalse(any(sent_by_observer))
        finally:
            observer.close(); listener.close()


class VideoAndConfigTests(unittest.TestCase):
    def test_route_projection_and_point(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'route.csv'
            path.write_text('x,y,yaw,s,road_id,lane_id,road_s\n0,0,0,0,1,-1,0\n10,0,0,10,1,-1,10\n')
            route=RouteProgress(path,max_distance_m=3)
            projected=route.project(obj(x=4,y=1),dict(road=1),None)
            self.assertAlmostEqual(projected['s'],4)
            self.assertAlmostEqual(projected['distance_m'],1)
            self.assertEqual(route.point_at(8),(8,0))
            self.assertIsNone(route.project(obj(x=4,y=4),dict(road=1),None))

    def test_shared_jpeg_frames_and_health(self):
        command=[sys.executable,'-u','-c',"import sys,time; sys.stdout.buffer.write(bytes.fromhex('ffd8ffdb00040000ffd9')); sys.stdout.flush(); time.sleep(10)"]
        relay=VideoRelay(lambda:(command,'test_generator',None))
        relay.start()
        try:
            seq,jpeg=relay.wait_frame(0,3)
            self.assertEqual(seq,1)
            self.assertEqual(jpeg,bytes.fromhex('ffd8ffdb00040000ffd9'))
            self.assertEqual(relay.wait_frame(0,.1),(seq,jpeg))
            self.assertTrue(relay.status()['found'])
            relay.last_frame_at=time.monotonic()-4
            self.assertFalse(relay.status()['found'])
        finally: relay.close()

    def test_invalid_geometry_rejected(self):
        for meta in ({'finish_line':{'begin':[0,0],'end':[0,0]}},
                     {'finish_line':{'begin':[0,0],'end':[1,0],'direction':0}},
                     {'sections':[{'id':'1','polygon':[[0,0],[1,0],[2,0]]}]},
                     {'rear_axle_offset_m':float('nan')}):
            with self.assertRaises(ValueError): validate_metadata(meta)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)
        rules=yaml.safe_load((ROOT/'tools/dashboard/assessment_rules.yaml').read_text())
        rules['completion'] = dict(mode='finish_line')  # Explicit legacy-mode regression coverage.
        rules['session'].update(respawn_jump_m=20)
        rules['session']['respawn_detection_confirmed']=True
        rules.pop('finish_stop_line',None)
        rules['map'] = {}
        rules.pop('school_zones_file',None)
        rules['route'] = {}
        rules['metadata']=dict(scenario_id='TEST',rear_axle_offset_m=0,object_feed_complete=True,
          sections=[dict(id='1',polygon=[[-100,-10],[100,-10],[100,10],[-100,10]])],
          finish_line=dict(begin=[10,5],end=[10,-5]),school_zones=[],crosswalks=[],stop_lines=[],centerlines=[],sidewalk_boundaries=[],lane_boundaries=[])
        p=self.path/'rules.yaml'
        p.write_text(yaml.safe_dump(rules))
        self.engine=LiveAssessment(p,self.path/'output')
        self.engine.on_control_received(dict(steering=0,accel=0,indicator=0))

    def install_straight_lane_map(self):
        path=self.path/'turn-map.xodr'
        path.write_text('''<OpenDRIVE><road id="1" length="100" junction="-1" rule="RHT">
          <planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
          <lanes><laneSection s="0"><center><lane id="0"><roadMark sOffset="0" type="solid"/></lane></center>
          <right><lane id="-1" type="driving"><width sOffset="0" a="3"/><roadMark sOffset="0" type="broken"/></lane>
          <lane id="-2" type="driving"><width sOffset="0" a="3"/><roadMark sOffset="0" type="broken"/></lane></right>
          </laneSection></lanes></road></OpenDRIVE>''')
        self.engine.map=OpenDriveMap(path)

    def tearDown(self):
        self.engine.close()
        self.tmp.cleanup()

    def test_start_finish_rear_axle_freezes_and_reset(self):
        e=self.engine
        e.pending_control=None
        e.on_frame(frame(0,x=0,speed=0))
        self.assertIsNone(e.session_id)
        e.on_frame(frame(.1,x=0,speed=1))
        self.assertIsNone(e.session_id)  # Movement/Play alone must not start scoring.
        e.on_control_received(dict(steering=0,accel=0,indicator=0))
        self.assertEqual(e.status,'RUNNING')
        first=e.session_id
        for i in range(1,12):
            e.on_frame(frame(.1+i*.1,x=i,speed=10))
        self.assertEqual(e.status,'COMPLETED')
        count=len(e.rows)
        e.on_frame(frame(2,x=20,speed=10))
        self.assertEqual(len(e.rows),count)
        e.on_frame(frame(0,x=0,speed=0,number=0))
        self.assertEqual(e.status,'WAITING')
        e.on_control_received(dict(steering=0,accel=0,indicator=0))
        e.on_frame(frame(.1,x=0,speed=1))
        self.assertNotEqual(e.session_id,first)
        self.assertEqual(sum(e.respawns.values()), 0)

    def test_pause_and_disconnect_no_distance(self):
        e=self.engine
        e.on_frame(frame(0,x=0))
        e.on_frame(frame(0,x=50))
        self.assertEqual(e.distance,0)
        e.on_gap()
        e.on_frame(frame(5,x=5))
        self.assertEqual(e.distance,0)

    def test_zero_command_starts_stationary_ego(self):
        e=self.engine
        e.pending_control=None
        e.on_frame(frame(5,speed=0))
        e.on_control_received(dict(steering=0,accel=0,indicator=0))
        self.assertEqual(e.status,'RUNNING')
        self.assertEqual(e.started_at,5)
        e.on_frame(frame(5.1,speed=0))
        self.assertAlmostEqual(float(e.rows[-1]['elapsed_wall_time_s']),.1)
        self.assertEqual(e.snapshot()['start_source'],'controller_packet_arrival')

    def test_manual_finalize_records_incomplete_result(self):
        e=self.engine
        received=frame(0,speed=0)
        received.update(
            received_wall_time_ns=1_700_000_000_000_000_000,
            received_monotonic_ns=8_000_000_000,
        )
        e.on_frame(received)
        path=next((self.path/'output').glob('*.jsonl'))
        self.assertTrue(e.finalize_incomplete())
        self.assertFalse(e.finalize_incomplete())
        self.assertEqual(e.status,'ABORTED')
        records=[json.loads(line) for line in path.read_text().splitlines()]
        result=records[-1]
        self.assertEqual(result['kind'],'result')
        self.assertEqual(result['status'],'ABORTED')
        self.assertEqual(result['finish_reason'],'manual_stop')
        self.assertEqual(
            result['latest']['received_wall_time_ns'],
            '1700000000000000000',
        )
        self.assertEqual(result['latest_received_monotonic_ns'],'8000000000')

    def test_route_proximity_does_not_override_finish(self):
        e=self.engine
        routefile=self.path/'route.csv'
        routefile.write_text('x,y,s,road_id,lane_id\n0,0,0,1,-1\n5,0,5,1,-1\n')
        e.route=RouteProgress(routefile)
        e.route_start_s=0
        e.route_goal_s=5
        for i in range(6): e.on_frame(frame(i*.1,x=i,speed=10))
        self.assertEqual(e.status,'RUNNING')
        for i in range(6,11): e.on_frame(frame(i*.1,x=i,speed=10))
        self.assertEqual(e.status,'COMPLETED')

    def test_finish_ignores_reverse_outside_and_teleport(self):
        e=self.engine
        e.on_frame(frame(0,x=11))
        e.on_frame(frame(.1,x=9))
        self.assertEqual(e.status,'RUNNING')
        e.on_frame(frame(.2,x=90))
        self.assertEqual(e.status,'RUNNING')

    def test_unknown_indicator_history_not_penalized(self):
        e=self.engine
        for i,lid in enumerate([-1,-1,-2]):
            f=frame(i*.1,x=i)
            f['road'][1]=dict(road=1,lane=lid,s=i,heading=0,offset=0)
            f['lights'][1]='OFF'
            f['lanes'][(1,1,lid)]=dict(left=-1,right=-2)
            e.on_frame(f)
        self.assertEqual(e.rows[-1]['turn_signal_violation'],'')

    def test_turn_signal_event_records_lane_and_signal_evidence(self):
        e=self.engine
        self.install_straight_lane_map()
        for i in range(34):
            lid=-1 if i < 31 else -2
            f=frame(i*.1,x=i*.1)
            f.update(
                received_wall_time_ns=1_700_000_000_000_000_000+i,
                received_monotonic_ns=8_000_000_000+i,
            )
            f['road'][1]=dict(road=1,lane=lid,s=i*.1,heading=0,offset=0)
            f['lights'][1]='OFF'
            f['lanes'][(1,1,lid)]=dict(left=-1,right=-2)
            e.on_frame(f)
        event=next(item for item in e.events if item['event']=='turn_signal_violation')
        self.assertEqual(event['evidence']['expected'],'RIGHT')
        self.assertEqual(event['evidence']['direction_source'],'xodr_lane_centres_in_ego_travel_frame')
        self.assertEqual(event['evidence']['observed'],'OFF')
        self.assertEqual(event['evidence']['from_lane_id'],-1)
        self.assertEqual(event['evidence']['to_lane_id'],-2)
        self.assertGreaterEqual(event['evidence']['observation_history_s'],3)
        self.assertEqual(event['evidence']['confirmation_frames'],3)
        self.assertAlmostEqual(event['evidence']['transition_sim_time_s'],3.1)
        self.assertEqual(
            event['received_wall_time_ns'],
            '1700000000000000033',
        )

    def test_one_frame_lane_id_flicker_is_not_a_signal_violation(self):
        e=self.engine
        self.install_straight_lane_map()
        lane_ids=[-1]*31+[-2,-1,-1,-1]
        for i,lid in enumerate(lane_ids):
            f=frame(i*.1,x=i*.1)
            f['road'][1]=dict(road=1,lane=lid,s=i*.1,heading=0,offset=0)
            f['lights'][1]='OFF'
            f['lanes'][(1,1,lid)]=dict(left=-1,right=-2)
            e.on_frame(f)
        self.assertFalse(any(
            item['event']=='turn_signal_violation' for item in e.events
        ))
        self.assertFalse(any(
            row.get('turn_signal_violation')=='1' for row in e.rows
        ))

    def test_stable_lane_change_with_three_second_signal_still_passes(self):
        e=self.engine
        self.install_straight_lane_map()
        for i in range(34):
            lid=-1 if i < 31 else -2
            f=frame(i*.1,x=i*.1)
            f['road'][1]=dict(road=1,lane=lid,s=i*.1,heading=0,offset=0)
            f['lights'][1]='RIGHT'
            f['lanes'][(1,1,lid)]=dict(left=-1,right=-2)
            e.on_frame(f)
        self.assertEqual(e.rows[-1]['turn_signal_violation'],'0')
        self.assertEqual(e.rows[-1]['turn_signal_confirmation_frames'],'3')
        self.assertFalse(any(
            item['event']=='turn_signal_violation' for item in e.events
        ))

    def test_m16_positive_lane_change_is_left_in_vehicle_frame(self):
        e=self.engine
        e.map=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        heading=math.radians(18.6)
        for i in range(34):
            lid=2 if i < 31 else 1
            f=frame(i*.1,x=1310.5,speed=1)
            f['objects'][1].update(y=163.9,h=heading)
            f['road'][1]=dict(road=2626,lane=lid,s=7.883,heading=0,offset=0,flags=2)
            f['lights'][1]='LEFT'
            # Deliberately preserve the misleading RDB adjacency observed in
            # the real run; physical XODR geometry must take precedence.
            f['lanes'][(1,2626,lid)]=dict(left=2,right=1)
            e.on_frame(f)
        last=e.rows[-1]
        self.assertEqual(last['turn_signal_expected'],'LEFT')
        self.assertEqual(last['turn_signal_violation'],'0')
        self.assertGreater(float(last['turn_signal_direction_signed_lateral_m']),2.9)

    def test_obb_static_and_stationary_vehicle_classification(self):
        e=self.engine
        f=frame(0)
        f['objects'][2]=obj(x=1,ident=2,name='NPC',speed=0)
        f['objects'][3]=obj(x=-1,ident=3,name='Barrier',kind=9,speed=0)
        e.on_frame(f)
        p=e.snapshot()
        by={r['number']:r for r in p['official_grid']}
        self.assertEqual(by[13]['penalty'],-3)
        self.assertEqual(by[15]['penalty'],-6)
        self.assertEqual(p['summary']['major_count'],1)
        self.assertTrue(list((self.path/'output').glob('*.jsonl')))
        self.assertEqual(e.rows[-1]['road_user_collision_ids'], '2')
        self.assertEqual(e.rows[-1]['static_collision_ids'], '3')
        collision_event = next(event for event in e.events if event['event'] == 'road_user_collision')
        self.assertEqual(collision_event['evidence']['object_ids'], '2')

    def test_route_mismatch_withholds_aggregate_score(self):
        routefile=self.path/'far-route.csv'
        routefile.write_text('x,y,s,road_id,lane_id\n1000,1000,0,1,-1\n1010,1000,10,1,-1\n')
        self.engine.route=RouteProgress(routefile,20)
        self.engine.route_path=routefile
        self.engine.route_start_s=0
        self.engine.route_goal_s=10
        self.engine.route_section_count=5
        self.engine.section_ids=[str(i) for i in range(1,6)]
        for i in range(26):
            self.engine.on_frame(frame(i*.1,x=0,speed=1))
        snapshot=self.engine.snapshot()
        self.assertIsNone(snapshot['summary']['official_score'])
        self.assertEqual(snapshot['summary']['official_evaluation'],'CONFIG_MISMATCH')
        self.assertTrue(snapshot['assessment_profile']['route_mismatch'])
        self.assertTrue(any('평가 경로 불일치' in warning for warning in snapshot['warnings']))

    def test_run_scenario_profile_selects_current_route_and_start(self):
        self.engine.close()
        routefile=self.path/'active-route.csv'
        routefile.write_text(
            'x,y,s,road_id,lane_id\n0,0,0,1,-1\n10,0,10,1,-1\n',
            encoding='utf-8',
        )
        profile_path=self.path/'active.json'
        profile_path.write_text(json.dumps(dict(
            schema_version=1,status='RUNNING',publisher_pid=os.getpid(),
            created_wall_time_ns=time.time_ns(),scenario_id='M16',run_id='live',
            route_csv=str(routefile),section_count=5,
            max_projection_distance_m=20,
        )),encoding='utf-8')
        rules=yaml.safe_load((ROOT/'tools/dashboard/assessment_rules.yaml').read_text())
        rules['completion']=dict(mode='final_checkpoint',tolerance_m=1.5)
        rules['map']={}
        rules.pop('school_zones_file',None)
        rules.pop('school_zone_corrections_file',None)
        rules['route']=dict(csv=str(self.path/'does-not-match.csv'),section_count=5)
        # The configured default must exist so initialization itself is valid.
        (self.path/'does-not-match.csv').write_text(
            'x,y,s,road_id,lane_id\n100,100,0,2,-1\n110,100,10,2,-1\n',
            encoding='utf-8',
        )
        rules['metadata']=dict(object_feed_complete=False)
        rules_path=self.path/'profile-rules.yaml'
        rules_path.write_text(yaml.safe_dump(rules),encoding='utf-8')
        self.engine=LiveAssessment(rules_path,self.path/'profile-output',profile_path)
        self.engine.on_control_received(dict(steering=0,accel=0,indicator=0))
        self.engine.on_frame(frame(0,x=2,speed=0))
        payload=self.engine.snapshot()
        self.assertEqual(payload['latest']['scenario_id'],'M16')
        self.assertEqual(payload['assessment_profile']['source'],'run_scenario')
        self.assertEqual(payload['assessment_profile']['route_csv'],str(routefile.resolve()))
        self.assertAlmostEqual(payload['assessment_profile']['section_boundaries'][0]['start_s_m'],2)
        self.assertEqual(payload['latest']['evaluation_section_id'],'1')

    def test_positive_lane_entry_is_not_false_centerline_intrusion(self):
        map_path=self.path/'two-sided-road.xodr'
        map_path.write_text('''<OpenDRIVE><road id="1" length="100" junction="-1" rule="RHT">
          <planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
          <lanes><laneSection s="0"><center><lane id="0"><roadMark sOffset="0" type="solid"/></lane></center>
          <left><lane id="1" type="driving"><width sOffset="0" a="3"/></lane></left>
          <right><lane id="-1" type="driving"><width sOffset="0" a="3"/></lane></right>
          </laneSection></lanes></road></OpenDRIVE>''')
        self.engine.map=OpenDriveMap(map_path)
        self.engine.meta.pop('centerlines',None)
        f=frame(0,x=20,speed=1)
        f['objects'][1].update(y=1.5,h=0)
        # A near-zero RDB road heading previously selected the negative side,
        # even though Ego is correctly centred in positive lane 1.
        f['road'][1]=dict(road=1,lane=1,s=20,t=1.5,offset=0,heading=0,flags=2)
        f['lanes'][(1,1,1)]=dict(width=3,left=0,right=0)
        self.engine.on_frame(f)
        self.assertEqual(float(self.engine.rows[-1]['centerline_intrusion_m']),0)
        self.assertEqual(self.engine.expected_side,1)

    def test_crosswalk_polygon_on_different_road_is_not_active(self):
        self.engine.meta['crosswalks']=[dict(
            id='other-road',road_id=2,
            polygon=[[-5,-5],[5,-5],[5,5],[-5,5]],
        )]
        f=frame(0,x=0,speed=0)
        f['road'][1]=dict(road=1,lane=-1,s=1,t=-1,offset=0,heading=0,flags=1)
        self.engine.on_frame(f)
        self.assertEqual(self.engine.rows[-1]['crosswalk_active'],'0')
        self.assertEqual(self.engine.rows[-1]['crosswalk_ids'],'')

    def test_crosswalk_overlap_without_road_position_is_unavailable(self):
        self.engine.meta['crosswalks']=[dict(
            id='road-unknown',road_id=2,
            polygon=[[-5,-5],[5,-5],[5,5],[-5,5]],
        )]
        f=frame(0,x=0,speed=0)
        f['road'].clear()
        self.engine.on_frame(f)
        self.assertNotIn('crosswalk_active',self.engine.rows[-1])
        self.assertNotIn('crosswalk_source',self.engine.rows[-1])
        by_number={item['number']:item for item in self.engine.snapshot()['official_grid']}
        self.assertEqual(by_number[12]['status'],'UNAVAILABLE')

    def test_no_metadata_keeps_base_score_but_marks_partial(self):
        self.engine.meta={}
        self.engine.on_frame(frame(0))
        p=self.engine.snapshot()
        self.assertIsNone(p['summary']['official_score'])
        self.assertEqual(p['summary']['official_evaluation'],'PARTIAL')
        self.assertIn('sections',p['missing_metadata'])

    def test_respawn_first_per_section_free_then_six_each(self):
        e=self.engine
        e.on_frame(frame(0,x=0,speed=1))
        e.on_frame(frame(.1,x=30,speed=1))
        first=e.snapshot()
        self.assertIsNone(first['summary']['official_score'])
        self.assertEqual(next(item for item in first['official_grid'] if item['number']==6)['penalty'],0)
        e.on_frame(frame(.2,x=0,speed=1))
        second=e.snapshot()
        by={item['number']:item for item in second['official_grid']}
        self.assertEqual(by[6]['penalty'],-6)
        self.assertIsNone(second['summary']['official_score'])
        self.assertEqual(second['summary']['major_count'],1)
        e.meta['sections']=[dict(id='2',polygon=[[-100,-10],[100,-10],[100,10],[-100,10]])]
        e.on_frame(frame(.3,x=1,speed=1))
        e.on_frame(frame(.4,x=30,speed=1))
        self.assertEqual(e.respawns['2'],1)
        self.assertIsNone(e.snapshot()['summary']['official_score'])
        self.assertEqual(len([event for event in e.events if event['event']=='respawn']),3)

    def test_solid_contact_requires_actual_lane_change(self):
        e=self.engine
        e.meta['lane_boundaries']=[dict(begin=[0,-5],end=[0,5],type='solid')]
        def sample(t,x,lid):
            f=frame(t,x=x,speed=10)
            f['road'][1]=dict(road=1,lane=lid,s=10,heading=0,offset=0)
            e.on_frame(f)
        sample(0,-2,-1)
        sample(.1,-1,-1)
        self.assertEqual(e.rows[-1]['solid_line_crossing'],'0')
        sample(.2,1,-2)
        self.assertEqual(e.rows[-1]['solid_line_crossing'],'')
        sample(.3,2,-2)
        sample(.4,3,-2)
        self.assertEqual(e.rows[-1]['solid_line_crossing'],'1')
        result=next(r for r in e.snapshot()['official_grid'] if r['number']==8)
        self.assertEqual(result['penalty'],-3)

    def test_green_start_exception_uses_forward_30m_obb_corridor(self):
        e=self.engine
        e.meta['stop_lines']=[dict(id='L',begin=[10,-5],end=[10,5],direction=-1,
                                   signal_id=7,state_masks={1:'GREEN'})]
        def measured(other):
            f=frame(0,x=0,speed=0)
            f['road'][1]=dict(road=1,lane=-1,s=0,heading=0,offset=0)
            f['signals'][7]=dict(mask=1)
            if other is not None:
                f['objects'][2]=other
            result={}
            e._objects_and_signals(f,f['objects'][1],f['road'][1],result)
            return result
        inside=measured(obj(x=30,y=0,ident=2,name='NPC'))
        self.assertTrue(inside['justified_stop'])
        self.assertEqual(inside['green_blocker_forward_m'],30.0)
        self.assertEqual(inside['green_blocker_ids'],'2')
        self.assertFalse(measured(obj(x=35,y=0,ident=2,name='NPC'))['justified_stop'])
        self.assertFalse(measured(obj(x=20,y=3,ident=2,name='NPC'))['justified_stop'])
        self.assertFalse(measured(obj(x=-5,y=0,ident=2,name='NPC'))['justified_stop'])

    def test_green_start_exception_follows_curved_route(self):
        e=self.engine
        routefile=self.path/'turn_route.csv'
        routefile.write_text('x,y,s,road_id,lane_id\n0,0,0,1,-1\n10,0,10,1,-1\n10,20,30,2,-1\n')
        e.route=RouteProgress(routefile)
        e.route_goal_s=30
        corridors=e._forward_path_corridors(obj(x=0,y=0),dict(route_s_m=0))
        self.assertTrue(any(overlaps(corridor,obb(obj(x=10,y=10,ident=2))) for corridor in corridors))
        self.assertFalse(any(overlaps(corridor,obb(obj(x=16,y=10,ident=2))) for corridor in corridors))

    def test_finite_finish_and_obb_offsets(self):
        self.assertFalse(crosses(dict(begin=[1,1],end=[1,-1]),(0,20),(2,20)))
        box=obb(obj())
        self.assertEqual(max(p[0] for p in box),3)
        self.assertTrue(overlaps(box,obb(obj(x=4))))

    def test_http_live_endpoint_score_and_assets(self):
        import server
        server.LIVE_ASSESSMENT=self.engine
        server.RDB_OBSERVER=type('Receiver',(),{'status':dict(connected=True,error=None)})()
        http=ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
        thread=threading.Thread(target=http.serve_forever,daemon=True)
        thread.start()
        try:
            self.engine.on_frame(frame(0,speed=60/3.6))
            url=f'http://127.0.0.1:{http.server_port}'
            with urllib.request.urlopen(url+'/api/live') as response:
                payload=json.load(response)
            self.assertIsNone(payload['summary']['official_score'])
            self.assertEqual(payload['official_grid'][0]['penalty_by_section'],{'1':3})
            with urllib.request.urlopen(url+'/static/app.js') as response:
                self.assertEqual(response.headers['Cache-Control'],'no-store')
                self.assertIn(b'refreshLive',response.read())
        finally:
            http.shutdown(); http.server_close(); thread.join()
            server.LIVE_ASSESSMENT=server.RDB_OBSERVER=None


class MapTests(unittest.TestCase):
    def test_zebra_extraction_and_explicit_final_stop_line(self):
        m=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        stops,walks=extract_features(m)
        self.assertEqual(len(stops),710)
        self.assertEqual(len(walks),197)
        self.assertFalse(any('Warning' in item['source'] for item in walks))
        finish=finish_from_object(stops,dict(road_id=1928,object_id='2304',whole_carriageway=True))
        self.assertAlmostEqual(finish['s'],94.4112959818)
        self.assertGreater(math.dist(finish['begin'],finish['end']),11)

    def test_supplied_school_boundaries_and_summary_agree(self):
        import csv
        m=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        zones=SchoolZones(ROOT/'tools/dashboard/living_lab_school_zones.yaml',m)
        self.assertEqual(len(zones.accepted),38)  # Raw input + reverse junction traversal.
        self.assertEqual({p['id'] for p in zones.pending},{f'school_zone_{n:02d}' for n in (15,28,39,40,42,44,45)})
        self.assertLess(zones.max_s_error,.01)
        with (ROOT/'tools/dashboard/living_lab_school_zones_summary.csv').open(encoding='utf-8-sig') as stream:
            summary=list(csv.DictReader(stream))
        raw=yaml.safe_load((ROOT/'tools/dashboard/living_lab_school_zones.yaml').read_text())['school_zones']
        self.assertEqual(len(summary),len(raw))
        for record,source in zip(summary,raw):
            self.assertEqual(record['zone_id'],source['id'])
            self.assertEqual(float(record['point1_s']),source['point_1']['s'])
            if source['point_2']:
                self.assertEqual(float(record['point2_s']),source['point_2']['s'])
        self.assertTrue(any(r['id']=='school_zone_10' and r['lane']==-2 for r in zones.ranges))
        samples=m.reference(2264)
        idx=min(range(len(samples)-1),key=lambda i:abs(samples[i][0]-40))
        s,x,y=samples[idx]
        _,nx,ny=samples[idx+1]
        h=math.atan2(ny-y,nx-x)
        lane=m.lanes_at(2264,s)['lanes'][1]
        t=(lane['inner']+lane['outer'])/2
        rear=(x-math.sin(h)*t,y+math.cos(h)*t)
        active,ids=zones.measure(dict(road=2264,s=s),rear)
        self.assertTrue(active)
        self.assertIn('school_zone_31',ids)
        self.assertEqual(zones.measure(None,rear),(None,[]))
        result=ev.evaluate_school_zone([row(school_zone_active=active,ego_speed=36/3.6)],30)
        self.assertEqual(result.penalty,-6)
        with tempfile.TemporaryDirectory() as tmp:
            rules=yaml.safe_load((ROOT/'tools/dashboard/assessment_rules.yaml').read_text())
            path=Path(tmp)/'rules.yaml'
            path.write_text(yaml.safe_dump(rules))
            engine=LiveAssessment(path,Path(tmp)/'runs')
            engine.on_control_received(dict(steering=0,accel=0,indicator=0))
            try:
                f=frame(0,speed=10)
                f['objects'][1].update(x=rear[0],y=rear[1],h=h,length=4.848,width=1.886,off_x=1.384)
                f['road'][1]=dict(road=2264,lane=1,s=s,heading=math.pi,offset=0)
                engine.on_frame(f)
                snapshot=engine.snapshot()
                by={item['number']:item for item in snapshot['official_grid']}
                self.assertEqual(by[2]['penalty'],-6)
                self.assertIsNone(snapshot['summary']['official_score'])
                self.assertEqual(len(snapshot['school_zone_validation']['accepted']),43)
                self.assertEqual(snapshot['summary']['official_evaluation'],'PARTIAL')
            finally:
                engine.close()

    def test_world_obb_depth_curves_and_solid_boundary(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'map.xodr'
            path.write_text('''<OpenDRIVE><road id="1" length="100" junction="-1">
            <planView><geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
            <lanes><laneSection s="0"><center><lane id="0"><roadMark sOffset="0" type="solid"/></lane></center>
            <left><lane id="1" type="driving"><width sOffset="0" a="3"/><roadMark sOffset="0" type="broken"/></lane>
            <lane id="2" type="sidewalk"><width sOffset="0" a="4"/></lane></left>
            <right><lane id="-1" type="driving"><width sOffset="0" a="3"/><roadMark sOffset="0" type="solid"/></lane>
            <lane id="-2" type="driving"><width sOffset="0" a="3"/></lane>
            <lane id="-3" type="sidewalk"><width sOffset="0" a="4"/></lane></right>
            </laneSection></lanes></road></OpenDRIVE>''')
            m=OpenDriveMap(path)
            data=m.measure(obj(x=20,y=-.4),dict(road=1,s=20,lane=-1),-1)
            self.assertAlmostEqual(data['centerline_intrusion_m'],.6)
            data=m.measure(obj(x=20,y=-5.5),dict(road=1,s=20,lane=-2),-1)
            self.assertAlmostEqual(data['sidewalk_intrusion_m'],.5)
            self.assertEqual(m.boundary_type(1,20,-1,-2),'solid')
            self.assertTrue(m.boundary_contact(obj(x=20,y=-2.8),obj(x=20,y=-3.2),1,20,-1,-2))
            self.assertFalse(m.boundary_contact(obj(x=20,y=-1),obj(x=20,y=-1.1),1,20,-1,-2))
            self.assertEqual(m.project(1,(20,-2),20),(20,-2))

    def test_current_map_load_and_finite_measurements(self):
        m=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
        points=m.reference(30)
        self.assertTrue(len(points)>10)
        self.assertTrue(all(math.isfinite(v) for p in points for v in p))
        self.assertIsNotNone(m.lanes_at(30,10))


if __name__=='__main__':
    unittest.main()
