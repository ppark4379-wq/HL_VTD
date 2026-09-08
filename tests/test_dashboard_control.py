import json
from pathlib import Path
import socket
import struct
import sys
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/dashboard'))
from control_monitor import CommandDecoder, ControlEvents


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
        self.assertEqual(decoder.feed(self.packet(struct.pack('<ffB',float('nan'),0,0))),[])
        self.assertEqual(CommandDecoder({'10.0.0.1'},controller_ip='10.0.0.9').feed(self.packet(command)),[])

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
            client.sendto(json.dumps(event).encode(),receiver.status['socket'])
            deadline=time.monotonic()+2
            while not receiver.status['connected'] and time.monotonic()<deadline: time.sleep(.01)
            self.assertTrue(receiver.status['connected'])
            self.assertEqual(calls,[])
            event['kind']='command'
            client.sendto(json.dumps(event).encode(),receiver.status['socket'])
            while not calls and time.monotonic()<deadline: time.sleep(.01)
            self.assertEqual(len(calls),1)
        finally:
            client.close(); receiver.close()
        self.assertFalse(Path(receiver.status['socket']).exists())


if __name__=='__main__': unittest.main()
