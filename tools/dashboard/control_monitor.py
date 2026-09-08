#!/usr/bin/env python3
"""Read-only Linux observer of incoming 9-byte HLVTD controller commands.

Run this small helper with sudo, NOT the web server. It never binds/connects to
9910 or forwards/modifies traffic. AF_PACKET observes arrival at the VTD host;
it cannot certify when the closed-source plugin consumes the command.
"""
import argparse
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time


class CommandDecoder:
    def __init__(self, local_addresses, port=9910, controller_ip=None):
        self.local_addresses=set(local_addresses)
        self.port=port
        self.controller_ip=controller_ip
        self.flows={}

    def feed(self, packet):
        if len(packet)<54:
            return []
        offset=14
        ethertype=struct.unpack_from('!H',packet,12)[0]
        while ethertype in (0x8100,0x88a8):
            if len(packet)<offset+4: return []
            ethertype=struct.unpack_from('!H',packet,offset+2)[0]
            offset+=4
        if ethertype!=0x0800 or packet[offset]>>4!=4:
            return []
        ihl=(packet[offset]&15)*4
        total=struct.unpack_from('!H',packet,offset+2)[0]
        if ihl<20 or total<ihl+20 or len(packet)<offset+total or packet[offset+9]!=6:
            return []
        if struct.unpack_from('!H',packet,offset+6)[0]&0x3fff:
            return []  # Fragmented IP datagrams require another capture adapter.
        src=socket.inet_ntoa(packet[offset+12:offset+16])
        dst=socket.inet_ntoa(packet[offset+16:offset+20])
        if dst not in self.local_addresses or (self.controller_ip and src!=self.controller_ip):
            return []
        tcp=offset+ihl
        sport,dport,seq=struct.unpack_from('!HHI',packet,tcp)
        thl=(packet[tcp+12]>>4)*4
        if dport!=self.port or thl<20 or ihl+thl>total:
            return []
        flags=packet[tcp+13]
        key=(src,sport,dst,dport)
        payload=packet[tcp+thl:offset+total]
        if flags&2:
            self.flows[key]={'next':(seq+1)&0xffffffff,'buffer':bytearray(),'pending':{}}
            seq=(seq+1)&0xffffffff
        if flags&4:
            self.flows.pop(key,None)
            return []
        if not payload: return []
        if len(self.flows)>32: self.flows.clear()
        flow=self.flows.setdefault(key,{'next':seq,'buffer':bytearray(),'pending':{}})
        delta=(seq-flow['next'])&0xffffffff
        if delta>=0x80000000:
            skipped=(flow['next']-seq)&0xffffffff
            if skipped>=len(payload): return []  # Retransmission/loopback duplicate.
            payload=payload[skipped:]
            seq=flow['next']
        if seq!=flow['next']:
            if len(flow['pending'])<64: flow['pending'][seq]=payload
            return []
        while payload:
            flow['buffer'].extend(payload)
            flow['next']=(flow['next']+len(payload))&0xffffffff
            payload=flow['pending'].pop(flow['next'],b'')
        commands=[]
        while len(flow['buffer'])>=9:
            steer,accel,indicator=struct.unpack_from('<ffB',flow['buffer'])
            del flow['buffer'][:9]
            if not math.isfinite(steer) or not math.isfinite(accel) or indicator not in (0,1,2):
                # No guessed byte-wise resynchronization of a control protocol.
                self.flows.pop(key,None)
                return commands
            commands.append(dict(steering=steer,accel=accel,indicator=indicator,controller_ip=src))
        return commands


class ControlEvents:
    """Private local IPC; not a public HTTP endpoint capable of starting a run."""
    def __init__(self,on_command):
        self.on_command=on_command
        self.status={'connected':False,'error':None,'socket':None}
        self.stop=threading.Event()
        self.last_seen=None
        self.sock=None
        self.directory=None
        self.thread=None

    def start(self):
        try:
            self.directory=Path(tempfile.mkdtemp(prefix='hl_vtd_control_'))
            self.path=self.directory/'events.sock'
            self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            self.sock.bind(str(self.path))
            os.chmod(self.path,0o600)
            self.sock.settimeout(.5)
            self.status['socket']=str(self.path)
            self.thread=threading.Thread(target=self.run,daemon=True)
            self.thread.start()
        except OSError as error:
            self.status['error']=str(error)

    def run(self):
        while not self.stop.is_set():
            try:
                event=json.loads(self.sock.recv(4096))
                # Local monotonic clock prevents delayed/previous-process starts.
                if not isinstance(event,dict) or abs(time.monotonic()-float(event['at']))>2:
                    continue
                kind=event.get('kind')
                if kind not in ('heartbeat','command'): continue
                self.last_seen=time.monotonic()
                self.status.update(connected=True,error=None)
                if kind=='command':
                    values=[event.get('steering'),event.get('accel')]
                    if all(isinstance(v,(int,float)) and math.isfinite(v) for v in values) and event.get('indicator') in (0,1,2):
                        self.on_command(event)
            except socket.timeout:
                pass
            except (OSError,ValueError,KeyError,TypeError):
                pass
            if self.last_seen is None or time.monotonic()-self.last_seen>3:
                self.status['connected']=False

    def close(self):
        self.stop.set()
        if self.thread: self.thread.join(timeout=2)
        if self.sock: self.sock.close()
        if self.directory:
            if self.path.exists(): self.path.unlink()
            self.directory.rmdir()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--event-socket',required=True)
    parser.add_argument('--interface',help='VTD LAN interface, e.g. enp3s0; default all')
    parser.add_argument('--controller-ip',help='Only commands from this controller IPv4 address')
    parser.add_argument('--port',type=int,default=9910)
    args=parser.parse_args()
    if args.controller_ip: ipaddress.IPv4Address(args.controller_ip)
    ips={'127.0.0.1'}
    for link in json.loads(subprocess.check_output(['ip','-j','-4','address','show'],text=True)):
        ips.update(addr['local'] for addr in link.get('addr_info',[]) if addr['family']=='inet')
    decoder=CommandDecoder(ips,args.port,args.controller_ip)
    ipc=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    try:
        capture=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3))
    except (PermissionError,AttributeError) as error:
        parser.exit(2,f'Linux CAP_NET_RAW required. Run only this helper with sudo: {error}\n')
    if args.interface: capture.bind((args.interface,0))
    capture.settimeout(.5)
    print(f'[control-monitor] read-only controller arrival observation on TCP {args.port}',flush=True)
    last_heartbeat=0
    try:
        while True:
            now=time.monotonic()
            if now-last_heartbeat>=1:
                ipc.sendto(json.dumps(dict(kind='heartbeat',at=now)).encode(),args.event_socket)
                last_heartbeat=now
            try: packet=capture.recv(65535)
            except socket.timeout: continue
            commands=decoder.feed(packet)
            if commands:
                ipc.sendto(json.dumps(dict(kind='command',at=time.monotonic(),**commands[-1])).encode(),args.event_socket)
    except KeyboardInterrupt:
        pass
    finally:
        capture.close()
        ipc.close()


if __name__=='__main__':
    main()
