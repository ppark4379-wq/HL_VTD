"""Receive-only SCP evidence: no registration, commands or control writes.

VTD ReleaseNotes document SimCtrl/Assessment reason=offroad continue=auto.
That message is a cause notification, NOT a completed respawn acknowledgement.
Neither it, Reset, nor Set is converted to a confirmed respawn by this module.
"""
from __future__ import annotations

import socket
import struct
import re
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

HEADER = struct.Struct('<HH64s64sI')
MAX_PAYLOAD = 4 * 1024 * 1024


class ScpObservationDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= HEADER.size:
            magic, version, sender, receiver, size = HEADER.unpack_from(self.buffer)
            if magic != 40108 or version != 1 or size > MAX_PAYLOAD:
                raise ValueError('invalid SCP header')
            end = HEADER.size + size
            if len(self.buffer) < end:
                break
            payload = bytes(self.buffer[HEADER.size:end])
            del self.buffer[:end]
            raw = payload.rstrip(b'\0').decode('utf-8', 'replace')
            if '<!DOCTYPE' in raw.upper() or '<!ENTITY' in raw.upper():
                raise ValueError('SCP XML entities are unsupported')
            try:
                root = ET.fromstring(raw)
            except ET.ParseError:
                continue
            for node in root.iter():
                tag = node.tag.rsplit('}', 1)[-1]
                if tag not in ('Assessment', 'Reset', 'Restart', 'Set'):
                    continue
                result.append(dict(source='VTD_SCP_RECEIVE_ONLY',
                    sender=sender.split(b'\0')[0].decode('utf-8', 'replace'),
                    receiver=receiver.split(b'\0')[0].decode('utf-8', 'replace'),
                    event_type=tag, attributes=dict(node.attrib), raw_xml=raw,
                    officially_confirmed_respawn=False,
                    interpretation='cause_or_command_observation_not_completion'))
        return result


class PassiveScpObserver:
    def __init__(self, host, port, on_event):
        self.host, self.port, self.on_event = host, port, on_event
        self.stop_event = threading.Event()
        self.status = dict(connected=False, error=None, events=0, receive_only=True,
                           respawn_completion_verified=False)
        self.thread = threading.Thread(target=self.run, daemon=True, name='vtd-passive-scp')

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
                    decoder = ScpObservationDecoder()
                    self.status.update(connected=True, error=None)
                    while not self.stop_event.is_set():
                        try:
                            data = connection.recv(65536)
                        except socket.timeout:
                            continue
                        if not data:
                            raise ConnectionError('VTD closed SCP observation connection')
                        for event in decoder.feed(data):
                            event.update(received_wall_time_ns=time.time_ns(),
                                         received_monotonic_ns=time.monotonic_ns())
                            self.on_event(event)
                            self.status['events'] += 1
            except (OSError, ValueError) as error:
                self.status.update(connected=False, error=str(error))
            self.stop_event.wait(1)


RESPAWN_PHASE = re.compile(
    r'\[RESPAWN\] phase=(\S+) reason=(\S+) section=(-?\d+) distance=([\d.eE+\-]+)m '
    r'frame=(\d+) simTime=([\d.eE+\-]+)')


def parse_plugin_respawn_line(line):
    """Parse the installed plugin's actual log format without inventing semantics."""
    if '[RESPAWN]' not in line:
        return None
    event = dict(source='HLVTD_PLUGIN_LOG_READ_ONLY', raw_log=line.rstrip(),
                 officially_confirmed_respawn=False, event_type='PLUGIN_RESPAWN_DIAGNOSTIC')
    match = RESPAWN_PHASE.search(line)
    if match:
        phase, reason, section, distance, frame, sim_time = match.groups()
        event.update(phase=phase, reason=reason, plugin_section=int(section),
                     distance_m=float(distance), plugin_frame=int(frame),
                     plugin_sim_time_s=float(sim_time),
                     interpretation='plugin_phase_observed_completion_semantics_pending_verification')
    elif 'operator position change accepted' in line:
        event['interpretation'] = 'operator_reposition_not_automatic_respawn'
    elif 'simulation restart detected' in line:
        event['interpretation'] = 'simulation_restart_not_automatic_respawn'
    else:
        event['interpretation'] = 'plugin_command_or_diagnostic_not_completion'
    return event


class PluginRespawnLogObserver:
    """Tail only newly appended plugin log bytes; never consume a live PTY."""
    def __init__(self, path, on_event):
        self.path, self.on_event = Path(path), on_event
        self.stop_event = threading.Event()
        self.status = dict(path=str(self.path), connected=False, error=None, events=0,
                           receive_only=True, respawn_completion_verified=False)
        self.thread = threading.Thread(target=self.run, daemon=True, name='vtd-plugin-log-observer')

    def start(self): self.thread.start()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def run(self):
        identity = None
        offset = None
        pending = b''
        while not self.stop_event.is_set():
            try:
                with self.path.open('rb') as stream:
                    stat = self.path.stat()
                    current = (stat.st_dev, stat.st_ino)
                    if identity is None:
                        offset = stat.st_size  # Never replay a previous run at startup.
                    elif identity != current or stat.st_size < offset:
                        offset, pending = 0, b''
                    identity = current
                    stream.seek(offset)
                    data = stream.read(1024 * 1024)
                    offset += len(data)
                    pending += data
                    if len(pending) > MAX_PAYLOAD:
                        pending = b''
                        raise ValueError('plugin log line exceeds evidence size limit')
                    lines = pending.split(b'\n')
                    pending = lines.pop()
                    for line in lines:
                        event = parse_plugin_respawn_line(line.decode('utf-8', 'replace'))
                        if event:
                            event.update(log_file=str(self.path), observed_file_offset=offset,
                                received_wall_time_ns=time.time_ns(), received_monotonic_ns=time.monotonic_ns())
                            self.on_event(event)
                            self.status['events'] += 1
                    self.status.update(connected=True, error=None)
            except (OSError, ValueError) as error:
                self.status.update(connected=False, error=str(error))
            self.stop_event.wait(.25)
