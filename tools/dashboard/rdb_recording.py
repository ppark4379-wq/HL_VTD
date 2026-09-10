"""Opt-in, loss-accounted evidence from the dashboard's own RDB connection."""
from __future__ import annotations

import gzip
import json
import os
import threading
import time
from pathlib import Path


class RdbRecordingSession:
    """Preserve wire bytes and every decoded frame, without filtering by Ego.

    Wire offsets are offsets in the *decompressed* stream. Connection IDs in
    the chunk index delimit reconnects for replay with a fresh Decoder. A
    frame-number gap is an observation, not proof that the network lost data.
    Recording errors disable recording and remain visible in the manifest.
    """

    def __init__(self, directory, host, port):
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self._claim = self.directory / '.rdb-recording.lock'
        with self._claim.open('x') as stream:
            stream.write(str(os.getpid()))
        if any(p != self._claim for p in self.directory.iterdir()):
            self._claim.unlink()
            raise ValueError(f'RDB evidence directory must be new or empty: {self.directory}')
        self._lock = threading.RLock()
        self._streams = {}
        self._last_flush = time.monotonic()
        self._previous_frame = None
        self._connection_id = 0
        self._closed = False
        self._state = dict(
            schema='hl_vtd_rdb_recording/v1', directory=str(self.directory),
            active=True, error=None, host=host, port=port,
            created_wall_time_ns=time.time_ns(), created_monotonic_ns=time.monotonic_ns(),
            wire_file='stream.rdb.gz', chunk_index_file='chunks.jsonl.gz',
            frame_file='frames.jsonl.gz', events_file='events.jsonl.gz',
            received_bytes=0, chunks=0, frames=0, connections=0,
            receive_timeout_count=0, observed_missing_frame_numbers=0,
            cross_connection_missing_frame_numbers=0, cross_connection_frame_gaps=0,
            duplicate_frame_numbers=0, frame_resets=0,
            first_frame=None, last_frame=None,
            decoder_connections={},
            completeness_note=('Captures every byte received by this observer and every '
                'frame accepted by Decoder. Unknown RDB packages remain in raw wire bytes; '
                'unobserved frames before connection and untransmitted objects cannot be certified.'),
        )
        try:
            for name, filename in [('wire', 'stream.rdb.gz'), ('chunks', 'chunks.jsonl.gz'),
                                   ('frames', 'frames.jsonl.gz'), ('events', 'events.jsonl.gz')]:
                self._streams[name] = gzip.open(self.directory / filename, 'xb', compresslevel=1)
            self._manifest()
        except BaseException:
            for stream in self._streams.values():
                stream.close()
            self._claim.unlink(missing_ok=True)
            raise

    def _json(self, stream, value):
        self._streams[stream].write((json.dumps(value, ensure_ascii=False,
            separators=(',', ':')) + '\n').encode('utf-8'))

    def _manifest(self):
        temporary = self.directory / 'manifest.json.tmp'
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(self._state, stream, ensure_ascii=False, indent=2)
        temporary.replace(self.directory / 'manifest.json')

    def _write(self, action):
        with self._lock:
            if not self._state['active'] or self._closed:
                return
            try:
                action()
                if time.monotonic() - self._last_flush >= 1:
                    for stream in self._streams.values():
                        stream.flush()
                    self._manifest()
                    self._last_flush = time.monotonic()
            except (OSError, ValueError, TypeError) as error:
                self._state.update(active=False, error=f'{type(error).__name__}: {error}')
                try:
                    self._manifest()
                except OSError:
                    pass

    def event(self, kind, **details):
        def write():
            if kind == 'connected':
                self._connection_id += 1
                self._state['connections'] += 1
            elif kind == 'receive_timeout':
                self._state['receive_timeout_count'] += 1
            self._json('events', dict(kind=kind, connection_id=self._connection_id,
                **{'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(), **details}))
        self._write(write)

    def record_chunk(self, data, wall_time_ns, monotonic_ns):
        def write():
            offset = self._state['received_bytes']
            self._streams['wire'].write(data)
            self._json('chunks', dict(connection_id=self._connection_id,
                offset=offset, length=len(data), received_wall_time_ns=wall_time_ns,
                received_monotonic_ns=monotonic_ns))
            self._state['received_bytes'] += len(data)
            self._state['chunks'] += 1
        self._write(write)

    def record_frame(self, frame):
        def write():
            # Tuple dictionary keys are not JSON-compatible. Preserve every
            # identity explicitly; all other dictionaries retain their fields.
            saved = dict(frame)
            saved['lanes'] = [dict(player_id=key[0], road_id=key[1], lane_id=key[2], **lane)
                              for key, lane in frame.get('lanes', {}).items()]
            saved['connection_id'] = self._connection_id
            self._json('frames', saved)
            current = (frame['frame'], frame['time'], self._connection_id)
            previous = self._previous_frame
            if previous is not None:
                if current[0] > previous[0] + 1 and current[1] >= previous[1]:
                    missing = current[0] - previous[0] - 1
                    self._state['observed_missing_frame_numbers'] += missing
                    across = current[2] != previous[2]
                    if across:
                        self._state['cross_connection_missing_frame_numbers'] += missing
                        self._state['cross_connection_frame_gaps'] += 1
                    self.event('frame_number_gap', previous_frame=previous[0],
                               next_frame=current[0], missing_frame_numbers=missing,
                               previous_sim_time_s=previous[1], next_sim_time_s=current[1],
                               across_connections=across, previous_connection_id=previous[2])
                elif current[0] == previous[0] and current[1] == previous[1]:
                    self._state['duplicate_frame_numbers'] += 1
                elif current[0] < previous[0] or current[1] < previous[1]:
                    self._state['frame_resets'] += 1
                    self.event('frame_reset', previous_frame=previous[0], next_frame=current[0])
            self._previous_frame = current
            self._state['frames'] += 1
            bounds = dict(frame=current[0], sim_time=current[1],
                wall_time_ns=frame.get('received_wall_time_ns'),
                monotonic_ns=frame.get('received_monotonic_ns'))
            if self._state['first_frame'] is None:
                self._state['first_frame'] = bounds
            self._state['last_frame'] = bounds
        self._write(write)

    def record_decoder_state(self, decoder):
        with self._lock:
            self._state['decoder_connections'][str(self._connection_id)] = dict(
                decoder.stats, pending_bytes=len(decoder.buffer),
                pending_frame=decoder.frame['frame'] if decoder.frame is not None else None)

    def status(self):
        with self._lock:
            return json.loads(json.dumps(self._state))

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._state.update(active=False, closed_wall_time_ns=time.time_ns(),
                               closed_monotonic_ns=time.monotonic_ns())
            for stream in self._streams.values():
                try:
                    stream.close()
                except OSError as error:
                    self._state['error'] = str(error)
            try:
                self._manifest()
            finally:
                self._claim.unlink(missing_ok=True)
