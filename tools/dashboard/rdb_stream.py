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

from assessment_signals import decode_traffic_light
from rdb_transport import DiskSpool, DEFAULT_LIMIT, CONNECTED, DATA, GAP, DISCONNECTED

HEADER = struct.Struct('<HHIIId')
ENTRY = struct.Struct('<IIIHH')
MAGIC = 35712
MAX_MESSAGE = 32 * 1024 * 1024


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.frame = None
        self.stats = dict(resynchronization_bytes=0, incomplete_frames_discarded=0,
                          malformed_messages=0, invalid_simulation_times=0)

    def feed(self, data):
        self.buffer.extend(data)
        completed = []
        while len(self.buffer) >= HEADER.size:
            magic, version, header_size, size, frame_no, sim_time = HEADER.unpack_from(self.buffer)
            if magic != MAGIC or not 24 <= header_size <= MAX_MESSAGE or size > MAX_MESSAGE - header_size:
                del self.buffer[0]
                self.stats['resynchronization_bytes'] += 1
                continue
            total = header_size + size
            if len(self.buffer) < total:
                break
            message = bytes(self.buffer[:total])
            del self.buffer[:total]
            if not math.isfinite(sim_time):
                self.stats['invalid_simulation_times'] += 1
                continue
            if self.frame is None or self.frame['frame'] != frame_no or self.frame['time'] != sim_time:
                # Incomplete frames are discarded, never combined with new ones.
                if self.frame is not None:
                    self.stats['incomplete_frames_discarded'] += 1
                self.frame = dict(frame=frame_no, time=sim_time, version=version,
                                  objects={}, road={}, lanes={}, lights={}, signals={}, scoring={}, packages=[])
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
                self.stats['malformed_messages'] += 1
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
        elif package == 27 and len(data) >= 4:
            signal = decode_traffic_light(data, flags)
            self.frame['signals'][signal['id']] = signal
        elif package == 10000 and len(data) == 44:
            # Supplied viRDBIcd.h: collision/off-road flags, NOT reset/respawn.
            player, path_s, road_s, fuel, average, state_flags, slip = struct.unpack_from('<IffffIf', data)
            self.frame['scoring'][player] = dict(
                player_id=player, state_flags=state_flags,
                collision_reported=bool(state_flags & 1), offroad_reported=bool(state_flags & 2),
                unknown_flags=state_flags & ~3, source='RDB_PACKAGE_10000', raw_hex=data.hex())


class RdbObserver:
    def __init__(self, host, port, on_frame, on_gap, recorder=None, *,
                 spool_dir=None, max_spool_bytes=DEFAULT_LIMIT, drain_timeout_s=10):
        self.host, self.port = host, port
        self.on_frame, self.on_gap = on_frame, on_gap
        self.recorder = recorder
        self.stop_event = threading.Event()
        self.processing_stop = threading.Event()
        self.drain_timeout_s = max(0., float(drain_timeout_s))
        if spool_dir is None and recorder is not None:
            spool_dir = recorder.directory/'transport'
        self.spool = DiskSpool(spool_dir, max_bytes=max_spool_bytes)
        self._status = {'connected': False, 'error': None, 'last_frame_at': None,
                       'processing_error': None, 'shutdown_incomplete': False,
                       'transport': self.spool.status(), 'worker_alive': False,
                       'processing_lag_s': 0., 'processing_frame_wall_time_ns': None,
                       'receiver_received_bytes': 0}
        self.connection = None
        self.worker = threading.Thread(target=self._process, daemon=True, name='vtd-rdb-process')
        self.thread = threading.Thread(target=self.run, daemon=True, name='vtd-rdb-observer')

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()
        connection = self.connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except (OSError, AttributeError):
                pass
        if self.thread.ident is not None:
            self.thread.join(timeout=3+self.drain_timeout_s)
        else:
            self.spool.finish_input()
            if self.worker.ident is None:
                self.spool.persist(shutdown_incomplete=False)
                self.spool.close_files()
                if self.recorder is not None:
                    self.recorder.close()
        self._transport_status()
        return not self.thread.is_alive() and not self.worker.is_alive()

    def _transport_status(self):
        state = self.spool.status()
        oldest = state['oldest_pending_monotonic_ns']
        self._status.update(transport=state, worker_alive=self.worker.is_alive(),
                            processing_lag_s=max(0., (time.monotonic_ns()-oldest)/1e9)
                            if oldest is not None else 0.,
                            unpreserved_received_bytes=max(0, self._status['receiver_received_bytes']-state['received_bytes']))

    @property
    def status(self):
        # Dynamic lag keeps increasing while the very first callback is
        # blocked, even if the receiver has already read every current byte.
        self._transport_status()
        return dict(self._status)

    def _put(self, kind, cid, payload=b''):
        self.spool.put(kind, cid, time.time_ns(), time.monotonic_ns(), payload)
        self._transport_status()

    def _process(self):
        """One consumer preserves callbacks, reconnect boundaries and recording order."""
        decoder = None
        try:
            while not self.processing_stop.is_set():
                record = self.spool.get(self.processing_stop)
                if record is None:
                    break
                kind, cid, wall_ns, mono_ns, payload, _ = record
                if kind == CONNECTED:
                    decoder = Decoder()
                    if self.recorder is not None:
                        self.recorder.event('connected', wall_time_ns=wall_ns, monotonic_ns=mono_ns)
                elif kind == DATA:
                    if decoder is None:
                        raise ValueError('RDB data received without an ordered connection boundary')
                    if self.recorder is not None:
                        self.recorder.record_chunk(payload, wall_ns, mono_ns)
                    frames = decoder.feed(payload)
                    if self.recorder is not None:
                        self.recorder.record_decoder_state(decoder)
                    for frame in frames:
                        if self.processing_stop.is_set():
                            break
                        # Timestamp the actual last-byte receive, not the
                        # callback invocation after a potentially long backlog.
                        frame.update(received_wall_time_ns=wall_ns, received_monotonic_ns=mono_ns,
                                     connection_id=cid)
                        if self.recorder is not None:
                            self.recorder.record_frame(frame)
                        self._status['processing_frame_wall_time_ns'] = wall_ns
                        self.on_frame(frame)
                        self._status['last_frame_at'] = time.time()
                    if self.processing_stop.is_set():
                        break  # Do not acknowledge a partly processed chunk.
                elif kind in (GAP, DISCONNECTED):
                    if self.recorder is not None:
                        details = dict(wall_time_ns=wall_ns, monotonic_ns=mono_ns)
                        if kind == DISCONNECTED:
                            details['error'] = payload.decode('utf-8', 'replace')
                        self.recorder.event('receive_timeout' if kind == GAP else 'connection_error', **details)
                        if decoder is not None:
                            self.recorder.record_decoder_state(decoder)
                    self.on_gap()
                self.spool.ack(record)
                self._transport_status()
                if self.recorder is not None:
                    self._status['recording'] = self.recorder.status()
        except Exception as error:
            self._status.update(processing_error=f'{type(error).__name__}: {error}', shutdown_incomplete=True)
            self.stop_event.set()
        finally:
            self._transport_status()
            if self.spool.status()['backlog_bytes']:
                self._status['shutdown_incomplete'] = True
            if self.recorder is not None:
                try:
                    self.recorder.event('transport_processing_finished',
                        incomplete=self._status['shutdown_incomplete'], processing_error=self._status['processing_error'],
                        spool_path=str(self.spool.path), backlog_bytes=self.spool.status()['backlog_bytes'])
                    self.recorder.close()
                except OSError as error:
                    self._status.update(error=f'recording close: {error}', shutdown_incomplete=True)
            try:
                self.spool.persist(shutdown_incomplete=self._status['shutdown_incomplete'],
                                   processing_error=self._status['processing_error'])
            except OSError as error:
                self._status['error'] = f'transport manifest: {error}'
            if self.spool.closed_input:
                self.spool.close_files()

    def run(self):
        self.worker.start()
        cid = 0
        try:
            while not self.stop_event.is_set():
                try:
                    with socket.create_connection((self.host, self.port), timeout=2) as connection:
                        self.connection = connection
                        connection.settimeout(1)
                        cid += 1
                        self._put(CONNECTED, cid)
                        self._status.update(connected=True, error=None)
                        while not self.stop_event.is_set():
                            try:
                                data = connection.recv(262144)
                            except socket.timeout:
                                self._put(GAP, cid)
                                continue
                            if not data:
                                if self.stop_event.is_set():
                                    break
                                raise ConnectionError('VTD closed RDB connection')
                            self._status['receiver_received_bytes'] += len(data)
                            self._put(DATA, cid, data)
                except OSError as error:
                    self._status.update(connected=False, error=str(error))
                    if self.spool.error:
                        self._status['shutdown_incomplete'] = True
                        self.stop_event.set()
                        break
                    if not self.stop_event.is_set():
                        self._put(DISCONNECTED, cid, str(error).encode('utf-8')[:4096])
                finally:
                    self.connection = None
                self.stop_event.wait(1)
        except Exception as error:
            self._status.update(error=f'{type(error).__name__}: {error}', shutdown_incomplete=True)
            self.stop_event.set()
        finally:
            self._status['connected'] = False
            self.spool.finish_input()
            self.worker.join(timeout=self.drain_timeout_s)
            if self.worker.is_alive():
                self.processing_stop.set()
                self._status.update(shutdown_incomplete=True,
                                   processing_error='shutdown_drain_timeout; unprocessed raw bytes retained in spool')
            self._transport_status()
            try:
                self.spool.persist(shutdown_incomplete=self._status['shutdown_incomplete'],
                                   processing_error=self._status['processing_error'])
            except OSError as error:
                self._status['error'] = f'transport manifest: {error}'
            if not self.worker.is_alive():
                self.spool.close_files()
