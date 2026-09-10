import gzip
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'dashboard'))
from rdb_recording import RdbRecordingSession
from rdb_stream import Decoder, RdbObserver


def packet(frame=1):
    # Include an unknown package: the complete raw stream must still retain it.
    entries = struct.pack('<IIIHH', 16, 4, 4, 65000, 0) + b'test'
    entries += struct.pack('<IIIHH', 16, 0, 0, 2, 0)
    return struct.pack('<HHIIId', 35712, 0x126, 24, len(entries), frame, frame / 50) + entries


class RdbRecordingTests(unittest.TestCase):
    def test_all_decoded_fields_raw_chunks_and_gap_accounting(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            recorder = RdbRecordingSession(root, '127.0.0.1', 48190)
            recorder.event('connected')
            wire = packet(10) + packet(13)
            decoder = Decoder()
            offset = 0
            for chunk in (b'bad' + wire[:19], wire[19:]):
                recorder.record_chunk(chunk, 1000 + offset, 2000 + offset)
                offset += len(chunk)
                for frame in decoder.feed(chunk):
                    frame.update(received_wall_time_ns=123, received_monotonic_ns=456)
                    frame['objects'] = {1: {'name': 'Ego'}, 2: {'name': 'Pedestrian'}}
                    frame['road'] = {1: {'road': 2626, 'lane': 2}, 2: {'road': 2}}
                    frame['lanes'] = {(1, 2626, 2): dict(width=3.0, left=1, right=3)}
                    frame['lights'] = {1: 'LEFT', 2: 'OFF'}
                    frame['signals'] = {181: {'state': .4, 'mask': 2}}
                    recorder.record_frame(frame)
                    frame['objects'].clear()  # Subsequent consumer mutation cannot erase evidence.
                recorder.record_decoder_state(decoder)
            recorder.event('receive_timeout')
            recorder.close()
            with gzip.open(root / 'stream.rdb.gz', 'rb') as stream:
                self.assertEqual(stream.read(), b'bad' + wire)
            with gzip.open(root / 'chunks.jsonl.gz', 'rt') as stream:
                chunks = [json.loads(line) for line in stream]
            self.assertEqual(chunks[1]['offset'], chunks[0]['length'])
            self.assertEqual(sum(c['length'] for c in chunks), len(wire) + 3)
            with gzip.open(root / 'frames.jsonl.gz', 'rt') as stream:
                frames = [json.loads(line) for line in stream]
            self.assertEqual(len(frames), 2)
            self.assertEqual(set(frames[0]['objects']), {'1', '2'})
            self.assertEqual(set(frames[0]['road']), {'1', '2'})
            self.assertEqual(frames[0]['lanes'][0]['player_id'], 1)
            self.assertEqual(frames[0]['lanes'][0]['lane_id'], 2)
            self.assertEqual(frames[0]['lights']['1'], 'LEFT')
            self.assertEqual(frames[0]['signals']['181']['mask'], 2)
            self.assertIn(65000, frames[0]['packages'])
            self.assertEqual(frames[0]['received_monotonic_ns'], 456)
            manifest = json.loads((root / 'manifest.json').read_text())
            self.assertEqual(manifest['observed_missing_frame_numbers'], 2)
            self.assertEqual(manifest['receive_timeout_count'], 1)
            self.assertEqual(manifest['decoder_connections']['1']['resynchronization_bytes'], 3)
            self.assertFalse(manifest['active'])
            self.assertIsNone(manifest['error'])

    def test_reconnect_gaps_are_counted_but_simulation_resets_are_not(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = RdbRecordingSession(Path(tmp), '127.0.0.1', 48190)
            recorder.event('connected')
            for number in [100, 0, 0]:
                recorder.record_frame(Decoder().feed(packet(number))[0])
            recorder.event('connected')
            recorder.record_frame(Decoder().feed(packet(1000))[0])
            status = recorder.status()
            self.assertEqual(status['observed_missing_frame_numbers'], 999)
            self.assertEqual(status['cross_connection_missing_frame_numbers'], 999)
            self.assertEqual(status['frame_resets'], 1)
            self.assertEqual(status['duplicate_frame_numbers'], 1)
            self.assertEqual(status['connections'], 2)
            recorder.close()

    def test_reconnect_with_simulation_restart_is_not_a_missing_frame_gap(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = RdbRecordingSession(Path(tmp), '127.0.0.1', 48190)
            recorder.event('connected')
            recorder.record_frame(Decoder().feed(packet(1000))[0])
            recorder.event('connected')
            recorder.record_frame(Decoder().feed(packet(0))[0])
            self.assertEqual(recorder.status()['observed_missing_frame_numbers'], 0)
            self.assertEqual(recorder.status()['frame_resets'], 1)
            recorder.close()

    def test_existing_evidence_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            marker = Path(tmp) / 'evidence.txt'
            marker.write_text('keep')
            with self.assertRaisesRegex(ValueError, 'new or empty'):
                RdbRecordingSession(Path(tmp), '127.0.0.1', 48190)
            self.assertEqual(marker.read_text(), 'keep')
            self.assertFalse((Path(tmp) / '.rdb-recording.lock').exists())

    def test_recording_failure_is_visible_and_does_not_escape_to_assessment(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = RdbRecordingSession(Path(tmp), '127.0.0.1', 48190)
            recorder._streams['wire'].close()
            recorder.record_chunk(b'data', 1, 2)
            self.assertFalse(recorder.status()['active'])
            self.assertIn('ValueError', recorder.status()['error'])
            recorder.close()

    def test_observer_records_same_socket_bytes_before_assessment(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = RdbRecordingSession(Path(tmp), '127.0.0.1', 48190)
            collected = []
            observer = RdbObserver('127.0.0.1', 48190, collected.append, lambda: None,
                                   recorder=recorder)
            class Connection:
                def __enter__(self): return self
                def __exit__(self, *args): return False
                def settimeout(self, timeout): pass
                def recv(self, size):
                    observer.stop_event.set()
                    return packet(3)
            with patch('rdb_stream.socket.create_connection', return_value=Connection()):
                observer.run()
            recorder.close()
            self.assertEqual(len(collected), 1)
            with gzip.open(Path(tmp) / 'frames.jsonl.gz', 'rt') as stream:
                saved = json.loads(stream.readline())
            self.assertEqual(saved['frame'], collected[0]['frame'])
            self.assertEqual(saved['received_wall_time_ns'], collected[0]['received_wall_time_ns'])
            with gzip.open(Path(tmp) / 'stream.rdb.gz', 'rb') as stream:
                self.assertEqual(stream.read(), packet(3))


if __name__ == '__main__':
    unittest.main()
