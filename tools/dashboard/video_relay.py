"""A shared video capture process for all browser clients, with truthful health."""
from __future__ import annotations
import subprocess
import tempfile
import threading
import time


class VideoRelay:
    def __init__(self, command_factory, recording=None, view=None):
        self.command_factory = command_factory
        self.recording = recording
        self.view = view
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frame = None
        self.sequence = 0
        self.last_frame_at = None
        self.source = None
        self.error = None
        self.recording_error = None
        self.proc = None
        self.started_at = None
        self.watchdog = threading.Thread(target=self.watch, daemon=True, name='vtd-video-watchdog')
        self.thread = threading.Thread(target=self.run, daemon=True, name='vtd-video')

    def start(self):
        self.thread.start()
        self.watchdog.start()

    def watch(self):
        while not self.stop_event.wait(1):
            proc = self.proc
            started = self.started_at
            last = max(started or 0,self.last_frame_at or 0)
            if proc and started and time.monotonic()-last>8 and proc.poll() is None:
                try:
                    proc.kill()
                except OSError:
                    pass

    def status(self):
        with self.condition:
            live = self.last_frame_at is not None and time.monotonic()-self.last_frame_at < 3
            status = dict(found=live,error=self.error,frame_count=self.sequence)
            if isinstance(self.source,dict):
                status.update(self.source)
            else:
                status['source']=self.source
            if self.recording is not None and self.view is not None:
                try:
                    status['recording']=self.recording.status(self.view)
                except Exception as error:
                    status['recording']={
                        'status':'ERROR', 'active':False, 'error':str(error),
                    }
                if self.recording_error:
                    status['recording']['relay_error']=self.recording_error
            return status

    def wait_frame(self, sequence, timeout=2):
        with self.condition:
            self.condition.wait_for(lambda:self.sequence>sequence or self.stop_event.is_set(),timeout)
            if self.sequence>sequence and self.status()['found']:
                return self.sequence,self.frame
        return sequence,None

    def close(self):
        self.stop_event.set()
        if self.proc and self.proc.poll() is None:
            self.proc.kill()
        with self.condition:
            self.condition.notify_all()
        if self.thread.is_alive():
            self.thread.join(timeout=3)
        if self.watchdog.is_alive():
            self.watchdog.join(timeout=2)

    def run(self):
        while not self.stop_event.is_set():
            command,source,error = self.command_factory()
            with self.condition:
                self.source,self.error = source,error
                # A source label must only describe frames produced by this
                # capture attempt.  Otherwise a fast window/RTSP failover can
                # expose the previous source's JPEG as ``found: true`` under a
                # new title/window ID for up to the health timeout.
                self.frame=None
                self.last_frame_at=None
                self.condition.notify_all()
            if command is None:
                self.stop_event.wait(2)
                continue
            try:
                with tempfile.TemporaryFile() as errors:
                    self.proc=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=errors)
                    self.started_at=time.monotonic()
                    buffer=b''
                    while not self.stop_event.is_set():
                        chunk=self.proc.stdout.read1(65536)
                        if not chunk:
                            break
                        buffer+=chunk
                        if len(buffer)>16*1024*1024:
                            raise ValueError('video frame exceeds 16 MiB limit')
                        while True:
                            start=buffer.find(b'\xff\xd8')
                            if start<0:
                                buffer=buffer[-1:]  # preserve a split JPEG marker
                                break
                            end=buffer.find(b'\xff\xd9',start+2)
                            if end<0:
                                buffer=buffer[start:]
                                break
                            jpeg=buffer[start:end+2]
                            buffer=buffer[end+2:]
                            wall_time_ns=time.time_ns()
                            monotonic_ns=time.monotonic_ns()
                            with self.condition:
                                self.frame=jpeg
                                self.sequence+=1
                                sequence=self.sequence
                                self.last_frame_at=monotonic_ns/1_000_000_000
                                self.error=None
                                self.condition.notify_all()
                            if self.recording is not None and self.view is not None:
                                try:
                                    recorded=self.recording.record_frame(
                                        self.view,
                                        sequence,
                                        jpeg,
                                        source,
                                        wall_time_ns=wall_time_ns,
                                        monotonic_ns=monotonic_ns,
                                    )
                                    if recorded:
                                        self.recording_error=None
                                except Exception as error:
                                    # Recording is an evidence sidecar; never
                                    # sacrifice the live browser stream for it.
                                    self.recording_error=str(error)
                    errors.seek(0)
                    capture_error=errors.read(2048).decode('utf-8','replace').strip() or 'VTD video ended; reconnecting'
                    with self.condition:
                        self.error=capture_error
            except (OSError,ValueError) as error:
                with self.condition:
                    self.error=str(error)
            finally:
                if self.proc:
                    if self.proc.poll() is None:
                        self.proc.kill()
                    self.proc.wait(timeout=3)
                    if self.proc.stdout:
                        self.proc.stdout.close()
                    self.proc=None
                with self.condition:
                    # Do not serve a recently produced but now disconnected
                    # frame while waiting for the next discovery attempt.
                    self.frame=None
                    self.last_frame_at=None
                    self.condition.notify_all()
            self.stop_event.wait(2)
