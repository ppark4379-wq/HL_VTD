"""Transport tests never connect to VTD or a controller endpoint."""
import json
import hashlib
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/dashboard'))
from rdb_stream import RdbObserver
from rdb_transport import DiskSpool, DATA, CONNECTED, RECORD, SpoolLimitError, iter_spool_records
from test_rdb_recording import packet


class ScriptedConnection:
    def __init__(self, chunks, done=None):
        self.chunks = iter(chunks)
        self.done = done
    def __enter__(self): return self
    def __exit__(self, *args): return False
    def settimeout(self, value): pass
    def recv(self, size):
        try: return next(self.chunks)
        except StopIteration:
            if self.done: self.done()
            return b''


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_disk_fifo_capacity_and_original_bytes(self):
        spool = DiskSpool(self.root, max_bytes=RECORD.size+3)
        spool.put(DATA, 3, 100, 200, b'abc')
        with self.assertRaises(SpoolLimitError): spool.put(DATA, 3, 101, 201, b'd')
        self.assertEqual(spool.status()['received_bytes'], 3)
        record = spool.get(threading.Event())
        self.assertEqual(record[:5], (DATA,3,100,200,b'abc'))
        spool.ack(record)
        self.assertEqual(spool.status()['backlog_bytes'], 0)
        self.assertIn('capacity', spool.status()['error'])
        spool.close_files()
        self.assertTrue((self.root/'receive.spool').read_bytes().endswith(b'abc'))
        self.assertEqual(list(iter_spool_records(self.root/'receive.spool'))[0]['payload'],b'abc')

    def test_recovery_reader_rejects_incomplete_tail_unless_committed_limit_is_explicit(self):
        path=self.root/'test.spool'
        complete=RECORD.pack(DATA,1,10,20,3)+b'abc'
        path.write_bytes(complete+b'partial')
        with self.assertRaisesRegex(ValueError,'incomplete transport header'):
            list(iter_spool_records(path))
        self.assertEqual(list(iter_spool_records(path,committed_bytes=len(complete)))[0]['payload'],b'abc')

    def test_disk_reserve_halts_before_overwriting_committed_evidence(self):
        spool=DiskSpool(self.root,reserve_bytes=1024)
        spool.put(DATA,1,10,20,b'preserved')
        spool.last_space_check=-float('inf')
        with patch('rdb_transport.shutil.disk_usage',return_value=SimpleNamespace(free=1023)):
            with self.assertRaisesRegex(SpoolLimitError,'reserve reached'):
                spool.put(DATA,1,11,21,b'new')
        self.assertEqual(spool.status()['received_bytes'],9)
        record=spool.get(threading.Event())
        self.assertEqual(record[4],b'preserved')
        spool.ack(record);spool.close_files()

    def test_slow_callback_does_not_block_receiver_and_receive_times_are_preserved(self):
        entered, release, received = threading.Event(), threading.Event(), threading.Event()
        collected=[]
        def callback(frame):
            entered.set()
            if not release.wait(3): raise RuntimeError('test callback timeout')
            collected.append(frame)
        observer=RdbObserver('unused',0,callback,lambda:None,spool_dir=self.root,drain_timeout_s=4)
        def finish(): received.set();observer.stop_event.set()
        connection=ScriptedConnection([packet(i) for i in range(100)],finish)
        with patch('rdb_stream.socket.create_connection',return_value=connection):
            observer.start()
            self.assertTrue(entered.wait(1))
            self.assertTrue(received.wait(1),'receiver waited for the blocked assessment callback')
            self.assertGreater(observer.spool.status()['backlog_bytes'],0)
            self.assertEqual(collected,[])
            first_lag=observer.status['processing_lag_s']
            time.sleep(.01)
            self.assertGreater(observer.status['processing_lag_s'],first_lag)
            released_at=time.time_ns();release.set()
            observer.thread.join(5)
        self.assertFalse(observer.thread.is_alive())
        self.assertFalse(observer.worker.is_alive())
        self.assertEqual([f['frame'] for f in collected],list(range(100)))
        self.assertLess(collected[-1]['received_wall_time_ns'],released_at)
        self.assertEqual(observer.spool.status()['backlog_bytes'],0)
        self.assertFalse(observer.status['shutdown_incomplete'])

    def test_disconnect_boundaries_reset_decoder_and_preserve_callback_order(self):
        callbacks=[]
        observer=RdbObserver('unused',0,lambda f:callbacks.append(('frame',f['connection_id'],f['frame'])),
                             lambda:callbacks.append(('gap',)),spool_dir=self.root)
        first=ScriptedConnection([packet(10),packet(11)[:19]])
        second=ScriptedConnection([packet(20)],observer.stop_event.set)
        with patch('rdb_stream.socket.create_connection',side_effect=[first,second]):
            observer.start();observer.thread.join(4)
        self.assertEqual(callbacks,[('frame',1,10),('gap',),('frame',2,20)])
        self.assertFalse(observer.status['shutdown_incomplete'])

    def test_shutdown_timeout_is_explicit_and_retains_unprocessed_spool(self):
        entered, release=threading.Event(),threading.Event()
        def callback(frame): entered.set();release.wait(3)
        observer=RdbObserver('unused',0,callback,lambda:None,spool_dir=self.root,drain_timeout_s=.03)
        connection=ScriptedConnection([packet(1),packet(2)],observer.stop_event.set)
        with patch('rdb_stream.socket.create_connection',return_value=connection):
            observer.start();self.assertTrue(entered.wait(1));observer.thread.join(1)
        self.assertTrue(observer.status['shutdown_incomplete'])
        self.assertGreater(observer.spool.status()['backlog_bytes'],0)
        self.assertTrue((self.root/'receive.spool').is_file())
        release.set();observer.worker.join(2)
        self.assertFalse(observer.worker.is_alive())
        self.assertTrue(observer.spool.closed_files)
        self.assertTrue(json.loads((self.root/'transport.json').read_text())['shutdown_incomplete'])

    def test_callback_exception_is_processing_error_not_a_network_reconnect(self):
        def callback(frame): raise RuntimeError('deliberate assessment failure')
        observer=RdbObserver('unused',0,callback,lambda:None,spool_dir=self.root)
        connection=ScriptedConnection([packet(1)],observer.stop_event.set)
        with patch('rdb_stream.socket.create_connection',return_value=connection) as factory:
            observer.start();observer.thread.join(2)
        self.assertEqual(factory.call_count,1)
        self.assertIn('deliberate assessment failure',observer.status['processing_error'])
        self.assertTrue(observer.status['shutdown_incomplete'])

    def test_spool_write_failure_halts_receiver_without_claiming_complete(self):
        observer=RdbObserver('unused',0,lambda f:None,lambda:None,spool_dir=self.root)
        observer.spool.writer.close()
        with patch('rdb_stream.socket.create_connection',return_value=ScriptedConnection([packet(1)])):
            observer.start();observer.thread.join(2)
        self.assertTrue(observer.status['shutdown_incomplete'])
        self.assertIsNotNone(observer.spool.error)

    def test_real_tcp_sender_finishes_while_callback_is_blocked(self):
        listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.addCleanup(listener.close)
        listener.bind(('127.0.0.1',0));listener.listen(1)
        release,entered,sent=threading.Event(),threading.Event(),threading.Event()
        collected=[]
        count=1024
        entry=struct.pack('<IIIHH',16,65536,65536,65000,0)+b'X'*65536
        entry+=struct.pack('<IIIHH',16,0,0,2,0)
        def wire(frame):
            return struct.pack('<HHIIId',35712,0x126,24,len(entry),frame,frame/25)+entry
        expected_bytes=len(wire(0))*count
        def callback(frame):
            entered.set();release.wait(4);collected.append(frame['frame'])
        observer=RdbObserver('127.0.0.1',listener.getsockname()[1],callback,lambda:None,
                             spool_dir=self.root,drain_timeout_s=5)
        def serve():
            connection,_=listener.accept()
            with connection:
                for frame in range(count):connection.sendall(wire(frame))
                sent.set()
                release.wait(4)
        sender=threading.Thread(target=serve,daemon=True);sender.start();observer.start()
        try:
            self.assertTrue(entered.wait(1));self.assertTrue(sent.wait(2))
            deadline=time.monotonic()+2
            while observer.spool.status()['received_bytes']<expected_bytes and time.monotonic()<deadline:
                time.sleep(.005)
            # >64 MiB exceeds the VTD overflow size seen in the field and
            # cannot merely sit in normal localhost socket send buffers.
            self.assertEqual(observer.spool.status()['received_bytes'],expected_bytes)
            self.assertEqual(collected,[])
        finally:
            release.set();sender.join(2);observer.close()
        self.assertEqual(collected,list(range(count)))
        self.assertFalse(observer.status['shutdown_incomplete'])
        expected_hash,actual_hash=hashlib.sha256(),hashlib.sha256()
        for frame in range(count):expected_hash.update(wire(frame))
        total=0
        for record in iter_spool_records(self.root/'receive.spool'):
            if record['kind']==DATA:
                actual_hash.update(record['payload']);total+=len(record['payload'])
        self.assertEqual(total,expected_bytes)
        self.assertEqual(actual_hash.digest(),expected_hash.digest())


if __name__=='__main__':unittest.main()
