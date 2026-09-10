#!/usr/bin/env python3
"""Read-only Linux observer of incoming 9-byte HLVTD controller commands.

Run this small helper with sudo, NOT the web server, before connecting or
starting the controller. It never binds/connects to 9910 or forwards/modifies
traffic. AF_PACKET observes arrival at the VTD host; it cannot certify when the
closed-source plugin consumes the command. A TCP SYN is required so an already
open stream is never guessed as 9-byte command framing.
"""
import argparse
import io
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


COMMAND_SIZE = struct.calcsize('<ffB')
TCP_SEQUENCE_MASK = 0xffffffff
TCP_FORWARD_HALF = 0x80000000


class ControlEvidenceLog:
    """Exclusive, line-flushed JSONL evidence for decoded control arrivals."""

    def __init__(self, path, wall_clock=time.time_ns, monotonic_clock=time.monotonic_ns):
        self.path=Path(path).expanduser().resolve()
        self.wall_clock=wall_clock
        self.monotonic_clock=monotonic_clock
        self.closed=False
        # The unprivileged dashboard creates the parent. The sudo helper only
        # creates this leaf with O_EXCL, preventing accidental overwrite (and
        # refusing a pre-existing symlink as well).
        descriptor=os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o644)
        os.fchmod(descriptor,0o644)  # remain readable after sudo even with a restrictive umask
        self.stream=io.TextIOWrapper(
            os.fdopen(descriptor,'wb',closefd=True),encoding='utf-8',
            write_through=True,
        )
        self.write('start',schema_version=1)

    def write(self,kind,*,wall_time_ns=None,monotonic_ns=None,**payload):
        if self.closed:
            raise ValueError('control evidence log is closed')
        event=dict(
            kind=kind,
            wall_time_ns=self.wall_clock() if wall_time_ns is None else wall_time_ns,
            monotonic_ns=(
                self.monotonic_clock()
                if monotonic_ns is None else monotonic_ns
            ),
            **payload,
        )
        self.stream.write(
            json.dumps(event,ensure_ascii=False,separators=(',',':'))+'\n'
        )
        self.stream.flush()

    def command(self,command,*,wall_time_ns=None,monotonic_ns=None):
        self.write(
            'command',
            wall_time_ns=wall_time_ns,
            monotonic_ns=monotonic_ns,
            controller_ip=command['controller_ip'],
            steering=command['steering'],
            accel=command['accel'],
            indicator=command['indicator'],
        )

    def heartbeat(self,decoder):
        self.write('heartbeat',decoder=decoder)

    def close(self,decoder=None):
        if self.closed:
            return
        try:
            self.write('final',decoder=decoder or {})
        finally:
            self.closed=True
            self.stream.close()


class CommandDecoder:
    def __init__(self, local_addresses, port=9910, controller_ip=None,
                 reorder_grace_s=.5, clock=time.monotonic):
        self.local_addresses=set(local_addresses)
        self.port=port
        self.controller_ip=controller_ip
        self.reorder_grace_s=reorder_grace_s
        self.clock=clock
        self.flows={}
        self.stats={
            'decoded_commands':0,
            'duplicate_bytes':0,
            'queued_segments':0,
            'dropped_bytes':0,
            'resyncs':0,
            'invalid_commands':0,
            'unframed_segments':0,
            'unframed_bytes':0,
        }

    @staticmethod
    def _advance(sequence, amount):
        return (sequence+amount)&TCP_SEQUENCE_MASK

    @staticmethod
    def _forward_distance(sequence, reference):
        return (sequence-reference)&TCP_SEQUENCE_MASK

    def _new_flow(self, next_sequence, origin=None):
        return {
            'next':next_sequence,
            'origin':origin,
            'buffer':bytearray(),
            'pending':{},
            'gap_since':None,
        }

    def _decode_buffer(self, key, flow, controller_ip):
        commands=[]
        while len(flow['buffer'])>=COMMAND_SIZE:
            steer,accel,indicator=struct.unpack_from('<ffB',flow['buffer'])
            del flow['buffer'][:COMMAND_SIZE]
            if not math.isfinite(steer) or not math.isfinite(accel) or indicator not in (0,1,2):
                # The SYN-derived phase is the only safe framing source. If a
                # frame is invalid, wait for a new connection instead of
                # guessing a byte offset from value plausibility.
                self.stats['invalid_commands']+=1
                self.flows.pop(key,None)
                return commands
            commands.append(dict(steering=steer,accel=accel,indicator=indicator,
                                 controller_ip=controller_ip))
            self.stats['decoded_commands']+=1
        return commands

    def _queue(self, flow, sequence, payload, now):
        existing=flow['pending'].get(sequence)
        if existing is not None:
            # Retransmission may be re-segmented. Never let a shorter copy at
            # the same sequence number erase bytes already captured from the
            # longer segment; that would manufacture a second permanent gap.
            common=min(len(existing),len(payload))
            if existing[:common]==payload[:common]:
                self.stats['duplicate_bytes']+=common
                if len(payload)>len(existing):
                    flow['pending'][sequence]=payload
            else:
                # Conflicting bytes cannot both belong to one TCP stream.
                # Preserve the first observation and wait for a new SYN rather
                # than selecting data based on command-value plausibility.
                self.stats['duplicate_bytes']+=len(payload)
        elif len(flow['pending'])<64:
            self.stats['queued_segments']+=1
            flow['pending'][sequence]=payload
        else:
            self.stats['dropped_bytes']+=len(payload)
        if flow['gap_since'] is None:
            flow['gap_since']=now

    def _take_contiguous_pending(self, flow):
        payload=flow['pending'].pop(flow['next'],None)
        if payload is not None:
            return payload
        for sequence,pending in list(flow['pending'].items()):
            overlap=self._forward_distance(flow['next'],sequence)
            if overlap>=TCP_FORWARD_HALF:
                continue
            del flow['pending'][sequence]
            if overlap>=len(pending):
                self.stats['duplicate_bytes']+=len(pending)
                continue
            self.stats['duplicate_bytes']+=overlap
            return pending[overlap:]
        return b''

    def _accept(self, key, flow, sequence, payload, controller_ip, now):
        """Accept one TCP segment, preserving retransmit and reorder handling."""
        delta=self._forward_distance(sequence,flow['next'])
        if delta>=TCP_FORWARD_HALF:
            skipped=self._forward_distance(flow['next'],sequence)
            if skipped>=len(payload):
                self.stats['duplicate_bytes']+=len(payload)
                return []  # Retransmission/loopback duplicate.
            self.stats['duplicate_bytes']+=skipped
            payload=payload[skipped:]
            sequence=flow['next']
        if sequence!=flow['next']:
            self._queue(flow,sequence,payload,now)
            return []

        while payload:
            flow['buffer'].extend(payload)
            flow['next']=self._advance(flow['next'],len(payload))
            payload=self._take_contiguous_pending(flow)
        if flow['pending']:
            if flow['gap_since'] is None:
                flow['gap_since']=now
        else:
            flow['gap_since']=None
        return self._decode_buffer(key,flow,controller_ip)

    def _resync(self, key, flow, now):
        """Skip a persistent capture gap only at a SYN-derived command phase."""
        if (flow['origin'] is None or not flow['pending']
                or flow['gap_since'] is None
                or now-flow['gap_since']<self.reorder_grace_s):
            return []

        old_next=flow['next']
        ordered=sorted(
            flow['pending'].items(),
            key=lambda item:self._forward_distance(item[0],old_next),
        )
        choice=None
        for sequence,payload in ordered:
            distance=self._forward_distance(sequence,old_next)
            if distance>=TCP_FORWARD_HALF:
                continue
            phase=self._forward_distance(sequence,flow['origin'])%COMMAND_SIZE
            trim=(-phase)%COMMAND_SIZE
            if trim<len(payload):
                choice=(self._advance(sequence,trim),ordered)
                break
        if choice is None:
            return []

        boundary,segments=choice
        skipped=self._forward_distance(boundary,old_next)
        if skipped>=TCP_FORWARD_HALF:
            return []
        self.stats['dropped_bytes']+=len(flow['buffer'])+skipped
        self.stats['resyncs']+=1
        flow['next']=boundary
        flow['buffer'].clear()  # It ended in the permanently missing command.
        flow['pending']={}
        flow['gap_since']=None

        commands=[]
        controller_ip=key[0]
        for sequence,payload in segments:
            commands.extend(self._accept(key,flow,sequence,payload,controller_ip,now))
            if key not in self.flows:
                break
        return commands

    def poll(self, now=None):
        """Expire persistent gaps even when only unrelated packets arrive."""
        now=self.clock() if now is None else now
        commands=[]
        for key,flow in list(self.flows.items()):
            commands.extend(self._resync(key,flow,now))
        return commands

    def status(self):
        return {
            **self.stats,
            'flows':len(self.flows),
            'pending_flows':sum(bool(flow['pending']) for flow in self.flows.values()),
        }

    def feed(self, packet, now=None):
        now=self.clock() if now is None else now
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
            origin=self._advance(seq,1)
            if len(self.flows)>=32 and key not in self.flows:
                self.flows.clear()
            self.flows[key]=self._new_flow(origin,origin)
            seq=origin
        if flags&4:
            self.flows.pop(key,None)
            return []
        if not payload: return []
        flow=self.flows.get(key)
        if flow is None or flow['origin'] is None:
            # Starting in the middle of an established TCP byte stream gives
            # no trustworthy 9-byte phase. Plausible floats plus a 0/1/2 byte
            # are not framing evidence and must never start an assessment.
            self.stats['unframed_segments']+=1
            self.stats['unframed_bytes']+=len(payload)
            return []
        commands=self._accept(key,flow,seq,payload,src,now)
        if key in self.flows:
            commands.extend(self._resync(key,flow,now))
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
                decoder=event.get('decoder')
                if kind=='heartbeat' and isinstance(decoder,dict):
                    self.status['decoder']=decoder
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
    parser.add_argument(
        '--log',type=Path,
        help=(
            'Exclusively create a JSONL evidence file for decoded commands, '
            'periodic decoder heartbeats, and final statistics'
        ),
    )
    args=parser.parse_args()
    if args.controller_ip: ipaddress.IPv4Address(args.controller_ip)
    ips={'127.0.0.1'}
    for link in json.loads(subprocess.check_output(['ip','-j','-4','address','show'],text=True)):
        ips.update(addr['local'] for addr in link.get('addr_info',[]) if addr['family']=='inet')
    decoder=CommandDecoder(ips,args.port,args.controller_ip)
    evidence=None
    if args.log is not None:
        if not args.log.expanduser().parent.is_dir():
            parser.error('--log parent directory must already exist')
        try:
            evidence=ControlEvidenceLog(args.log)
        except FileExistsError:
            parser.error(f'--log refuses to overwrite existing file: {args.log}')
        except OSError as error:
            parser.error(f'cannot create --log evidence file: {error}')
    ipc=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    try:
        capture=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3))
    except (PermissionError,AttributeError) as error:
        if evidence is not None:
            evidence.close(decoder.status())
        ipc.close()
        parser.exit(2,f'Linux CAP_NET_RAW required. Run only this helper with sudo: {error}\n')
    if args.interface: capture.bind((args.interface,0))
    capture.settimeout(.5)
    print(f'[control-monitor] read-only controller arrival observation on TCP {args.port}',flush=True)
    last_heartbeat=0
    last_status=0
    try:
        while True:
            now=time.monotonic()
            if now-last_heartbeat>=1:
                status=decoder.status()
                ipc.sendto(json.dumps(dict(kind='heartbeat',at=now,decoder=status)).encode(),args.event_socket)
                if evidence is not None:
                    evidence.heartbeat(status)
                last_heartbeat=now
            commands=[]
            try:
                packet=capture.recv(65535)
            except socket.timeout:
                pass
            else:
                commands.extend(decoder.feed(packet))
            commands.extend(decoder.poll())
            if commands:
                latest_event=None
                for command in commands:
                    wall_time_ns=time.time_ns()
                    monotonic_ns=time.monotonic_ns()
                    if evidence is not None:
                        evidence.command(
                            command,
                            wall_time_ns=wall_time_ns,
                            monotonic_ns=monotonic_ns,
                        )
                    latest_event=dict(
                        kind='command',at=monotonic_ns/1_000_000_000,
                        wall_time_ns=wall_time_ns,
                        monotonic_ns=monotonic_ns,
                        **command,
                    )
                ipc.sendto(json.dumps(latest_event).encode(),args.event_socket)
            now=time.monotonic()
            if now-last_status>=10:
                status=decoder.status()
                print('[control-monitor] '
                      f"commands={status['decoded_commands']} "
                      f"dropped_bytes={status['dropped_bytes']} "
                      f"resyncs={status['resyncs']} "
                      f"unframed_bytes={status['unframed_bytes']} "
                      f"pending_flows={status['pending_flows']}",flush=True)
                last_status=now
    except KeyboardInterrupt:
        pass
    finally:
        capture.close()
        ipc.close()
        if evidence is not None:
            evidence.close(decoder.status())


if __name__=='__main__':
    main()
