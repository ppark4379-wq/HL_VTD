"""Bounded on-disk FIFO preserving receive order independently of assessment.

The uncompressed spool is retained as transport evidence. Completed records
have a fixed header followed by exactly payload_length bytes; a partial tail
after an I/O error is not published to the consumer. There is no RAM backlog.
This guarantees process-visible writes, not survival of a power loss before
the operating system flushes its page cache.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import struct
import tempfile
import threading
import time

RECORD = struct.Struct('<BIQQI')
CONNECTED, DATA, GAP, DISCONNECTED = 1, 2, 3, 4
MAX_PAYLOAD = 262144
DEFAULT_LIMIT = 8 * 1024**3
DEFAULT_RESERVE = 1024**3


class SpoolLimitError(OSError):
    pass


def iter_spool_records(path, *, committed_bytes=None):
    """Read retained transport evidence without modifying it or running VTD.

    Pass transport.json ``written_bytes`` to exclude an explicitly uncommitted
    tail. By default the complete file is validated and a torn record raises;
    unknown/truncated bytes are never silently represented as a complete run.
    ``end_offset`` is the acknowledgement cursor, not a simulation frame ID.
    For offline reconstruction start at zero and reset Decoder on CONNECTED.
    """
    path = Path(path)
    limit = path.stat().st_size if committed_bytes is None else int(committed_bytes)
    if limit < 0 or limit > path.stat().st_size:
        raise ValueError('invalid transport committed byte limit')
    offset = 0
    with path.open('rb') as stream:
        while offset < limit:
            if limit-offset < RECORD.size:
                raise ValueError('incomplete transport header at end of evidence')
            header = stream.read(RECORD.size)
            kind, cid, wall_ns, mono_ns, size = RECORD.unpack(header)
            if kind not in {CONNECTED, DATA, GAP, DISCONNECTED} or size > MAX_PAYLOAD:
                raise ValueError('invalid transport record header')
            if offset+RECORD.size+size > limit:
                raise ValueError('incomplete transport payload at end of evidence')
            payload = stream.read(size)
            if len(payload) != size:
                raise ValueError('truncated transport evidence')
            offset += RECORD.size+size
            yield dict(kind=kind, connection_id=cid, received_wall_time_ns=wall_ns,
                       received_monotonic_ns=mono_ns, payload=payload, end_offset=offset)


class DiskSpool:
    def __init__(self, directory=None, *, max_bytes=DEFAULT_LIMIT, reserve_bytes=DEFAULT_RESERVE):
        if max_bytes < RECORD.size:
            raise ValueError('transport spool limit is too small')
        if directory is None:
            parent = Path(__file__).resolve().parents[2]/'output/rdb_transport'
            parent.mkdir(parents=True, exist_ok=True)
            directory = tempfile.mkdtemp(prefix='rdb-', dir=parent)
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory/'receive.spool'
        self.writer = self.path.open('xb', buffering=0)
        self.reader = self.path.open('rb', buffering=0)
        self.condition = threading.Condition()
        self.max_bytes = int(max_bytes)
        self.reserve_bytes = max(0, int(reserve_bytes))
        self.last_space_check = -float('inf')
        self.written = self.read_offset = self.acknowledged = 0
        self.received_bytes = self.processed_bytes = 0
        self.records = self.processed_records = 0
        self.closed_input = self.closed_files = False
        self.error = None
        self.created_wall_time_ns = time.time_ns()
        self.last_received_monotonic_ns = self.last_processed_monotonic_ns = None
        self.oldest_pending_monotonic_ns = None
        self.persist()

    @staticmethod
    def _write_all(stream, data):
        view = memoryview(data)
        while view:
            count = stream.write(view)
            if not count:
                raise OSError('zero-length transport spool write')
            view = view[count:]

    def put(self, kind, connection_id, wall_ns, mono_ns, payload=b''):
        if len(payload) > MAX_PAYLOAD:
            raise ValueError('transport record exceeds maximum receive chunk')
        header = RECORD.pack(kind, connection_id, wall_ns, mono_ns, len(payload))
        with self.condition:
            if self.closed_input:
                raise OSError('transport spool input is closed')
            try:
                now = time.monotonic()
                if now-self.last_space_check >= 1:
                    self.last_space_check = now
                    free = shutil.disk_usage(self.directory).free
                    if free < self.reserve_bytes+len(header)+len(payload):
                        raise SpoolLimitError(f'transport disk reserve reached: free={free}, reserve={self.reserve_bytes}')
                if self.written + len(header) + len(payload) > self.max_bytes:
                    raise SpoolLimitError(f'transport spool capacity exceeded ({self.max_bytes} bytes)')
                self._write_all(self.writer, header)
                self._write_all(self.writer, payload)
                if self.written == self.acknowledged:
                    self.oldest_pending_monotonic_ns = mono_ns
                self.written += len(header) + len(payload)
                self.records += 1
                if kind == DATA:
                    self.received_bytes += len(payload)
                    self.last_received_monotonic_ns = mono_ns
            except (OSError, ValueError) as error:
                self.error = f'{type(error).__name__}: {error}'
                self.closed_input = True
                self.condition.notify_all()
                raise
            self.condition.notify()

    def get(self, cancelled):
        with self.condition:
            while self.read_offset == self.written and not self.closed_input and not cancelled.is_set():
                self.condition.wait(.1)
            if cancelled.is_set() or self.read_offset == self.written:
                return None
            # Separate file handles keep reads from changing the append offset.
            header = self.reader.read(RECORD.size)
            if len(header) != RECORD.size:
                raise OSError('committed transport header is truncated')
            kind, cid, wall_ns, mono_ns, size = RECORD.unpack(header)
            if size > MAX_PAYLOAD or self.read_offset + RECORD.size + size > self.written:
                raise OSError('invalid committed transport record length')
            payload = self.reader.read(size)
            if len(payload) != size:
                raise OSError('committed transport payload is truncated')
            self.read_offset += RECORD.size + size
            return kind, cid, wall_ns, mono_ns, payload, self.read_offset

    def ack(self, record):
        kind, _, _, mono_ns, payload, end = record
        with self.condition:
            self.acknowledged = end
            self.processed_records += 1
            if kind == DATA:
                self.processed_bytes += len(payload)
                self.last_processed_monotonic_ns = mono_ns
            if self.acknowledged < self.written:
                header = os.pread(self.reader.fileno(), RECORD.size, self.acknowledged)
                self.oldest_pending_monotonic_ns = RECORD.unpack(header)[3]
            else:
                self.oldest_pending_monotonic_ns = None

    def finish_input(self):
        with self.condition:
            self.closed_input = True
            self.condition.notify_all()

    def status(self):
        with self.condition:
            return dict(schema='hl_vtd_rdb_transport/v1', spool_path=str(self.path),
                        record_layout='<BIQQI: kind,connection_id,wall_ns,mono_ns,payload_length',
                        created_wall_time_ns=self.created_wall_time_ns,
                        capacity_bytes=self.max_bytes, written_bytes=self.written,
                        disk_reserve_bytes=self.reserve_bytes,
                        acknowledged_bytes=self.acknowledged, backlog_bytes=self.written-self.acknowledged,
                        received_bytes=self.received_bytes, processed_bytes=self.processed_bytes,
                        records=self.records, processed_records=self.processed_records,
                        input_closed=self.closed_input, error=self.error,
                        last_received_monotonic_ns=self.last_received_monotonic_ns,
                        last_processed_monotonic_ns=self.last_processed_monotonic_ns,
                        oldest_pending_monotonic_ns=self.oldest_pending_monotonic_ns,
                        retained_for_replay=True)

    def persist(self, **extra):
        with self.condition:
            state = self.status()
            state.update(extra)
            temporary = self.directory/'transport.json.tmp'
            temporary.write_text(json.dumps(state, indent=2)+'\n', encoding='utf-8')
            temporary.replace(self.directory/'transport.json')

    def close_files(self):
        with self.condition:
            if self.closed_files:
                return
            self.closed_files = True
            self.writer.close()
            self.reader.close()
