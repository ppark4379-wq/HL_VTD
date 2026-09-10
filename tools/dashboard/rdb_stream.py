"""Read-only VTD RDB observer, layouts verified against supplied viRDBIcd.h.

Never connects to the participant control socket (9910) or sends driver commands.
CONTACT_POINT (19) describes tire/road contact; it is NOT a collision event.
"""
from __future__ import annotations

import math
import socket
import struct
import threading
import time

HEADER = struct.Struct('<HHIIId')
ENTRY = struct.Struct('<IIIHH')
MAGIC = 35712
MAX_MESSAGE = 32 * 1024 * 1024


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.frame = None

    def feed(self, data):
        self.buffer.extend(data)
        completed = []
        while len(self.buffer) >= HEADER.size:
            magic, version, header_size, size, frame_no, sim_time = HEADER.unpack_from(self.buffer)
            if magic != MAGIC or not 24 <= header_size <= MAX_MESSAGE or size > MAX_MESSAGE - header_size:
                del self.buffer[0]
                continue
            total = header_size + size
            if len(self.buffer) < total:
                break
            message = bytes(self.buffer[:total])
            del self.buffer[:total]
            if not math.isfinite(sim_time):
                continue
            if self.frame is None or self.frame['frame'] != frame_no or self.frame['time'] != sim_time:
                # Incomplete frames are discarded, never combined with new ones.
                self.frame = dict(frame=frame_no, time=sim_time, version=version,
                                  objects={}, road={}, lanes={}, lights={}, signals={}, packages=[])
            offset = header_size
            valid = True
            while offset + ENTRY.size <= total:
                eh, ds, es, package, flags = ENTRY.unpack_from(message, offset)
                if eh < 16 or offset + eh + ds > total or (ds and (es == 0 or ds % es)):
                    valid = False
                    break
                payload = message[offset + eh:offset + eh + ds]
                self.frame['packages'].append(package)
                for position in range(0, ds, es or 1):
                    self._element(package, flags, payload[position:position + es])
                offset += eh + ds
                if package == 2:
                    if offset == total:
                        completed.append(self.frame)
                    self.frame = None
                    if offset != total:
                        valid = False
                    break
            if not valid or offset != total:
                self.frame = None
        return completed

    def _element(self, package, flags, data):
        if package == 9 and len(data) >= 112:
            oid, category, kind = struct.unpack_from('<IBB', data)
            length, width, height, ox, oy, oz = struct.unpack_from('<6f', data, 40)
            x, y, z, h, p, r, coord_flags, coord_type, system = struct.unpack_from('<dddfffBBH', data, 64)
            if coord_type != 0 or coord_flags & 3 != 3 or not all(map(math.isfinite, (x,y,z,h,p,r,length,width,height,ox,oy,oz))):
                return
            name = data[8:40].split(b'\0', 1)[0].decode('utf-8', 'replace')
            obj = dict(id=oid, name=name, category=category, type=kind, x=x, y=y, z=z,
                       h=h, p=p, r=r, length=length, width=width, height=height,
                       off_x=ox, off_y=oy, off_z=oz, speed=None)
            if flags & 1 and len(data) >= 208 and data[148] & 1:
                velocity = struct.unpack_from('<ddd', data, 112)
                if all(map(math.isfinite, velocity)):
                    obj['speed'] = math.sqrt(sum(v*v for v in velocity))
            self.frame['objects'][oid] = obj
        elif package == 5 and len(data) >= 40:
            oid, road, lane, flags = struct.unpack_from('<IHbB', data)
            s, t, lane_offset, heading = struct.unpack_from('<4f', data, 8)
            if all(map(math.isfinite, (s,t,lane_offset,heading))):
                self.frame['road'][oid] = dict(road=road, lane=lane, s=s, t=t,
                                              offset=lane_offset, heading=heading, flags=flags)
        elif package == 6 and len(data) >= 56:
            road, lane = struct.unpack_from('<Hb', data)
            width = struct.unpack_from('<f', data, 12)[0]
            player = struct.unpack_from('<I', data, 48)[0]
            self.frame['lanes'][(player, road, lane)] = dict(width=width, left=struct.unpack_from('<b', data, 4)[0], right=struct.unpack_from('<b', data, 5)[0])
        elif package == 10 and len(data) >= 8:
            player, mask = struct.unpack_from('<II', data)
            self.frame['lights'][player] = 'HAZARD' if mask & 0x60 == 0x60 else 'LEFT' if mask & 0x20 else 'RIGHT' if mask & 0x40 else 'OFF'
        elif package == 27 and len(data) >= 12:
            ident, state, mask = struct.unpack_from('<ifI', data)
            # state is a normalized cycle position, not the participant enum.
            # Only explicit metadata maps the display mask to a signal meaning.
            self.frame['signals'][ident] = dict(state=state, mask=mask)


class RdbObserver:
    def __init__(self, host, port, on_frame, on_gap):
        self.host, self.port = host, port
        self.on_frame, self.on_gap = on_frame, on_gap
        self.stop_event = threading.Event()
        self.status = {'connected': False, 'error': None, 'last_frame_at': None}
        self.thread = threading.Thread(target=self.run, daemon=True, name='vtd-rdb-observer')

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)

    def run(self):
        while not self.stop_event.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=2) as connection:
                    connection.settimeout(1)
                    self.status.update(connected=True, error=None)
                    decoder = Decoder()
                    while not self.stop_event.is_set():
                        try:
                            data = connection.recv(262144)
                        except socket.timeout:
                            self.on_gap()
                            continue
                        if not data:
                            raise ConnectionError('VTD closed RDB connection')
                        for frame in decoder.feed(data):
                            # Stamp the completed RDB frame immediately before
                            # handing it to assessment. These clocks match the
                            # video/control evidence indexes and allow the
                            # three streams to be aligned without treating VTD
                            # simulation time as a host timestamp.
                            frame['received_wall_time_ns'] = time.time_ns()
                            frame['received_monotonic_ns'] = time.monotonic_ns()
                            self.on_frame(frame)
                            self.status['last_frame_at'] = time.time()
            except (OSError, ValueError, struct.error) as error:
                self.status.update(connected=False, error=str(error))
                self.on_gap()
            self.stop_event.wait(1)
