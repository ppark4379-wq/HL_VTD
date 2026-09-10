import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/dashboard'))
from control_monitor import CommandDecoder, ControlEvents, ControlEvidenceLog


class ControlEvidenceLogTests(unittest.TestCase):
    def test_commands_heartbeats_and_final_stats_are_flushed_without_overwrite(self):
        wall=iter((100,101,102,103))
        mono=iter((200,201,202,203))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'control.jsonl'
            evidence=ControlEvidenceLog(
                path,wall_clock=lambda:next(wall),
                monotonic_clock=lambda:next(mono),
            )
            evidence.command(dict(
                controller_ip='10.0.0.2',steering=.25,accel=-.5,indicator=2,
            ))
            evidence.heartbeat({'decoded_commands':1})
            evidence.close({'decoded_commands':1,'unframed_bytes':9})
            events=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([event['kind'] for event in events],[
                'start','command','heartbeat','final',
            ])
            self.assertEqual(events[1],{
                'kind':'command','wall_time_ns':101,'monotonic_ns':201,
                'controller_ip':'10.0.0.2','steering':.25,'accel':-.5,
                'indicator':2,
            })
            self.assertEqual(events[-1]['decoder']['unframed_bytes'],9)
            self.assertEqual(path.stat().st_mode & 0o777,0o644)
            with self.assertRaises(FileExistsError):
                ControlEvidenceLog(path)


class ControlTests(unittest.TestCase):
    @staticmethod
    def packet(payload=b'',seq=100,flags=0x18,dst='10.0.0.1',port=9910):
        ethernet=bytes(12)+struct.pack('!H',0x0800)
        ip=bytearray(20)
        ip[0]=0x45; ip[9]=6
        struct.pack_into('!H',ip,2,40+len(payload))
        ip[12:16]=socket.inet_aton('10.0.0.2')
        ip[16:20]=socket.inet_aton(dst)
        tcp=bytearray(20)
        struct.pack_into('!HHI',tcp,0,12345,port,seq)
        tcp[12]=0x50; tcp[13]=flags
        return ethernet+ip+tcp+payload

    def test_fragment_reorder_duplicate_and_zero_command(self):
        decoder=CommandDecoder({'10.0.0.1'},controller_ip='10.0.0.2')
        command=struct.pack('<ffB',0,0,0)
        self.assertEqual(decoder.feed(self.packet(seq=99,flags=2)),[])
        self.assertEqual(decoder.feed(self.packet(command[3:6],seq=103)),[])
        self.assertEqual(decoder.feed(self.packet(command[:3],seq=100)),[])
        result=decoder.feed(self.packet(command[6:],seq=106))
        self.assertEqual(len(result),1)
        self.assertEqual(result[0]['accel'],0)
        self.assertEqual(decoder.feed(self.packet(command,seq=100)),[])
        self.assertEqual(decoder.feed(self.packet(seq=109,flags=0x10)),[])
        self.assertEqual(len(decoder.feed(self.packet(command*2,seq=109))),2)

    def test_only_local_valid_controller_payload(self):
        decoder=CommandDecoder({'10.0.0.1'})
        command=struct.pack('<ffB',0,0,0)
        self.assertEqual(decoder.feed(self.packet(command,dst='10.0.0.3')),[])
        self.assertEqual(decoder.feed(self.packet(command,port=48190)),[])
        self.assertEqual(decoder.feed(self.packet(seq=99,flags=2)),[])
        self.assertEqual(decoder.feed(self.packet(struct.pack('<ffB',float('nan'),0,0))),[])
        self.assertEqual(CommandDecoder({'10.0.0.1'},controller_ip='10.0.0.9').feed(self.packet(command)),[])

    def test_permanent_gap_resyncs_only_at_syn_command_phase(self):
        now=[0.0]
        decoder=CommandDecoder(
            {'10.0.0.1'},controller_ip='10.0.0.2',
            reorder_grace_s=.5,clock=lambda:now[0],
        )
        commands=[struct.pack('<ffB',index,index+.25,index%3) for index in range(5)]
        self.assertEqual(decoder.feed(self.packet(seq=99,flags=2)),[])
        first=decoder.feed(self.packet(commands[0],seq=100))
        self.assertEqual(first[0]['steering'],0)

        # Command 1 is permanently absent and the observed segment begins in
        # the middle of command 2. Do not guess-decode either partial command.
        future=commands[2][4:]+commands[3]+commands[4]
        self.assertEqual(decoder.feed(self.packet(future,seq=122)),[])
        now[0]=.49
        self.assertEqual(decoder.poll(),[])
        now[0]=.51
        recovered=decoder.poll()
        self.assertEqual([item['steering'] for item in recovered],[3,4])
        self.assertEqual(decoder.status()['resyncs'],1)
        self.assertEqual(decoder.status()['dropped_bytes'],18)

        # A late copy of the missing command remains a retransmission and must
        # not disturb the recovered phase or produce a duplicate event.
        self.assertEqual(decoder.feed(self.packet(commands[1],seq=109)),[])
        recovered=decoder.feed(self.packet(commands[0],seq=145))
        self.assertEqual([item['steering'] for item in recovered],[0])
        self.assertEqual(decoder.status()['decoded_commands'],4)

    def test_delayed_reorder_before_grace_keeps_all_commands(self):
        now=[0.0]
        decoder=CommandDecoder({'10.0.0.1'},reorder_grace_s=.5,clock=lambda:now[0])
        commands=[struct.pack('<ffB',index,index+.5,index%3) for index in range(3)]
        self.assertEqual(decoder.feed(self.packet(seq=99,flags=2)),[])
        self.assertEqual(decoder.feed(self.packet(commands[1]+commands[2],seq=109)),[])
        now[0]=.49
        recovered=decoder.feed(self.packet(commands[0],seq=100))
        self.assertEqual([item['steering'] for item in recovered],[0,1,2])
        self.assertEqual(decoder.status()['resyncs'],0)
        self.assertEqual(decoder.status()['dropped_bytes'],0)

    def test_overlapping_out_of_order_segment_is_trimmed_not_stalled(self):
        decoder=CommandDecoder({'10.0.0.1'})
        commands=[struct.pack('<ffB',index,index+.5,index%3) for index in range(2)]
        self.assertEqual(decoder.feed(self.packet(seq=99,flags=2)),[])
        self.assertEqual(decoder.feed(self.packet(commands[0][3:]+commands[1],seq=103)),[])
        recovered=decoder.feed(self.packet(commands[0][:6],seq=100))
        self.assertEqual([item['steering'] for item in recovered],[0,1])
        self.assertEqual(decoder.status()['resyncs'],0)
        self.assertEqual(decoder.status()['duplicate_bytes'],3)

    def test_shorter_retransmission_cannot_truncate_queued_segment(self):
        decoder=CommandDecoder({'10.0.0.1'})
        commands=[struct.pack('<ffB',index,index+.5,index%3) for index in range(3)]
        self.assertEqual(decoder.feed(self.packet(seq=99,flags=2)),[])
        self.assertEqual(decoder.feed(self.packet(commands[1]+commands[2],seq=109)),[])
        self.assertEqual(decoder.feed(self.packet(commands[1][:3],seq=109)),[])
        recovered=decoder.feed(self.packet(commands[0],seq=100))
        self.assertEqual([item['steering'] for item in recovered],[0,1,2])
        self.assertEqual(decoder.status()['duplicate_bytes'],3)

    def test_gap_without_observed_syn_is_not_guess_resynchronized(self):
        now=[0.0]
        decoder=CommandDecoder({'10.0.0.1'},reorder_grace_s=.5,clock=lambda:now[0])
        command=struct.pack('<ffB',1,2,0)
        self.assertEqual(decoder.feed(self.packet(command,seq=100)),[])
        self.assertEqual(decoder.feed(self.packet(command,seq=118)),[])
        now[0]=1
        self.assertEqual(decoder.poll(),[])
        self.assertEqual(decoder.status()['resyncs'],0)
        self.assertEqual(decoder.status()['flows'],0)
        self.assertEqual(decoder.status()['unframed_segments'],2)
        self.assertEqual(decoder.status()['unframed_bytes'],2*len(command))

    def test_private_event_receiver_stale_events_do_not_start(self):
        calls=[]
        receiver=ControlEvents(calls.append)
        receiver.start()
        self.assertIsNotNone(receiver.status['socket'])
        client=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        try:
            event=dict(kind='command',at=time.monotonic()-10,steering=0,accel=0,indicator=0)
            client.sendto(json.dumps(event).encode(),receiver.status['socket'])
            event['kind']='heartbeat'; event['at']=time.monotonic()
            event['decoder']={'decoded_commands':7,'dropped_bytes':9,'resyncs':1}
            client.sendto(json.dumps(event).encode(),receiver.status['socket'])
            deadline=time.monotonic()+2
            while not receiver.status['connected'] and time.monotonic()<deadline: time.sleep(.01)
            self.assertTrue(receiver.status['connected'])
            self.assertEqual(receiver.status['decoder']['resyncs'],1)
            self.assertEqual(calls,[])
            event['kind']='command'
            client.sendto(json.dumps(event).encode(),receiver.status['socket'])
            while not calls and time.monotonic()<deadline: time.sleep(.01)
            self.assertEqual(len(calls),1)
        finally:
            client.close(); receiver.close()
        self.assertFalse(Path(receiver.status['socket']).exists())


if __name__=='__main__': unittest.main()
