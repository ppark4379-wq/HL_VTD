"""A shared video capture process for all browser clients, with truthful health."""
from __future__ import annotations
import subprocess
import tempfile
import threading
import time


class VideoRelay:
    def __init__(self, command_factory):
        self.command_factory = command_factory
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frame = None
        self.sequence = 0
        self.last_frame_at = None
        self.source = None
        self.error = None
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
            return dict(found=live,source=self.source,error=self.error,frame_count=self.sequence)

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
        self.thread.join(timeout=3)
        self.watchdog.join(timeout=2)

    def run(self):
        while not self.stop_event.is_set():
            command,source,error = self.command_factory()
            self.source,self.error = source,error
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
                            with self.condition:
                                self.frame=jpeg
                                self.sequence+=1
                                self.last_frame_at=time.monotonic()
                                self.error=None
                                self.condition.notify_all()
                    errors.seek(0)
                    self.error=errors.read(2048).decode('utf-8','replace').strip() or 'VTD video ended; reconnecting'
            except (OSError,ValueError) as error:
                self.error=str(error)
            finally:
                if self.proc:
                    if self.proc.poll() is None:
                        self.proc.kill()
                    self.proc.wait(timeout=3)
                    if self.proc.stdout:
                        self.proc.stdout.close()
                    self.proc=None
            self.stop_event.wait(2)
