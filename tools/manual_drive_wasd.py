#!/usr/bin/env python3
"""Focused-window WASD control for VTD (not a real-vehicle controller).

Uses VTD 2025.2 viRDBIcd.h DRIVER_CTRL pedal/steering-wheel inputs.
Stop HL_VTD and other driver tools first; load/run the scenario in VTD.
"""

import argparse
import math
import os
import socket
import struct
import time

HEADER = struct.Struct('<HHIIId')
ENTRY = struct.Struct('<IIIHH')
MAGIC = 35712
VALIDITY = 0x2000 | 0x80 | 0x08 | 0x04 | 0x01
MAX_MESSAGE = 64 * 1024 * 1024


def controls(keys, focused, steering, dt):
    if not focused:
        return 0.0, 0.0, 1.0
    direction = int('a' in keys) - int('d' in keys)
    target = direction * 2.0  # steering wheel radians, not road-wheel angle
    step = 3.0 * min(max(dt, 0.0), 0.1)
    steering += max(-step, min(step, target - steering))
    brake = 1.0 if 'space' in keys else 0.6 if 's' in keys else 0.0
    throttle = 0.25 if 'w' in keys and not brake else 0.0
    if not throttle and not brake:
        brake = 0.2  # releasing W gently brakes, instead of latching throttle
    return steering, throttle, brake


def driver_packet(player, frame, sim_time, steering, throttle, brake):
    payload = bytearray(80)  # sizeof(RDB_DRIVER_CTRL_t), packed VTD wire layout
    struct.pack_into('<Iffff', payload, 0, player, steering, 0.0, throttle, brake)
    payload[52] = 4  # RDB_GEAR_BOX_POS_D; S is brake, not reverse
    struct.pack_into('<I', payload, 56, VALIDITY)
    entry = ENTRY.pack(ENTRY.size, len(payload), len(payload), 26, 0)
    return HEADER.pack(MAGIC, 0x0125, HEADER.size, len(entry) + len(payload),
                       frame, sim_time) + entry + payload


class RdbStream:
    def __init__(self):
        self.buffer = bytearray()
        self.player = None
        self.clock = None

    def feed(self, data):
        self.buffer.extend(data)
        updated = False
        while len(self.buffer) >= HEADER.size:
            magic, _, header_size, size, frame, sim_time = HEADER.unpack_from(self.buffer)
            if magic != MAGIC or header_size < HEADER.size or header_size + size > MAX_MESSAGE:
                raise ValueError('Invalid RDB message header')
            total = header_size + size
            if len(self.buffer) < total:
                break
            pos = header_size
            while pos < total:
                if pos + ENTRY.size > total:
                    raise ValueError('Truncated RDB entry')
                hs, ds, es, pkg, _ = ENTRY.unpack_from(self.buffer, pos)
                end = pos + hs + ds
                if hs < ENTRY.size or end > total:
                    raise ValueError('Invalid RDB entry')
                if pkg == 9 and ds and (es < 112 or ds % es):
                    raise ValueError('Invalid RDB object state')
                if pkg == 9 and es >= 112:
                    for offset in range(pos + hs, end, es):
                        name = self.buffer[offset + 8:offset + 40].split(b'\0', 1)[0]
                        if name == b'Ego':
                            self.player = struct.unpack_from('<I', self.buffer, offset)[0]
                pos = end
            if not math.isfinite(sim_time):
                raise ValueError('Invalid RDB simulation time')
            self.clock = (frame, sim_time)
            updated = True
            del self.buffer[:total]
        return updated


class DriverWindow:
    def __init__(self, root, sock, tk):
        self.root, self.sock = root, sock
        self.stream = RdbStream()
        self.keys = set()
        self.releases = {}
        self.focused = False
        self.steering = 0.0
        self.pending = bytearray()
        self.last_tick = time.monotonic()
        self.last_receive = self.last_tick
        self.closed = False
        root.title('VTD WASD manual drive')
        root.geometry('620x230')
        tk.Label(root, text='Click this window, then hold WASD', font=('', 18)).pack(pady=12)
        tk.Label(root, text='W: throttle   S: brake   A/D: steer   Space: full brake\n'
                 'Q / Escape: quit   |   Forward gear only\n'
                 'Release W: gentle brake. Focus lost: full brake.\n'
                 'Stop HL_VTD and other driver tools before using this window.').pack()
        self.status = tk.StringVar(value='Waiting for RDB Ego state...')
        tk.Label(root, textvariable=self.status).pack(pady=14)
        root.bind('<KeyPress>', self.press)
        root.bind('<KeyRelease>', self.release)
        root.bind('<FocusIn>', lambda _: setattr(self, 'focused', True))
        root.bind('<FocusOut>', self.focus_out)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(20, self.tick)

    def press(self, event):
        key = event.keysym.lower()
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        if key in ('q', 'escape'):
            self.close()
        else:
            self.keys.add(key)

    def release(self, event):
        key = event.keysym.lower()
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        # X11 repeat may emit release/press pairs: cancel the idle release on press.
        def finish():
            self.releases.pop(key, None)
            self.keys.discard(key)
        self.releases[key] = self.root.after_idle(finish)

    def focus_out(self, _):
        self.focused = False
        self.keys.clear()

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        self.steering, throttle, brake = controls(
            self.keys, self.focused, self.steering, now - self.last_tick)
        self.last_tick = now
        try:
            updated = False
            for _ in range(16):  # bound work per UI tick
                try:
                    data = self.sock.recv(65536)
                except BlockingIOError:
                    break
                if not data:
                    raise ConnectionError('VTD disconnected; stop/pause the simulation')
                updated = self.stream.feed(data) or updated
                self.last_receive = now
            if updated and self.stream.player is not None and not self.pending:
                self.pending.extend(driver_packet(self.stream.player, *self.stream.clock,
                                                  self.steering, throttle, brake))
            if self.pending:
                try:
                    sent = self.sock.send(self.pending)
                    if not sent:
                        raise ConnectionError('RDB socket closed')
                    del self.pending[:sent]
                except BlockingIOError:
                    pass
            if now - self.last_receive > 1.0:
                self.keys.clear()
                self.status.set('No RDB frames: run VTD / check connection. Release keys.')
            elif self.stream.player is None:
                self.status.set('Waiting for object named Ego; no controls sent.')
            else:
                self.status.set(f'Ego ID {self.stream.player} | steering {self.steering:+.2f} rad | '
                                f'throttle {throttle:.0%} | brake {brake:.0%}')
        except (OSError, ValueError) as error:
            self.keys.clear()
            self.sock.close()
            self.status.set(f'{error}\nControl stopped. Pause VTD before reconnecting.')
            return
        self.root.after(20, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.stream.player is not None and self.stream.clock is not None:
                self.sock.settimeout(0.1)
                # Finish any partial packet before the final best-effort brake command.
                self.sock.sendall(self.pending + driver_packet(
                    self.stream.player, *self.stream.clock, 0.0, 0.0, 1.0))
        except OSError:
            pass
        finally:
            self.sock.close()
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=os.environ.get('VTD_RDB_HOST', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('VTD_RDB_PORT', '48190')))
    args = parser.parse_args()
    import tkinter as tk
    root = tk.Tk()
    try:
        sock = socket.create_connection((args.host, args.port), timeout=2.0)
    except OSError as error:
        root.destroy()
        parser.exit(1, f'Cannot connect to VTD: {error}\nLoad/run VTD first.\n')
    sock.setblocking(False)
    window = DriverWindow(root, sock, tk)
    try:
        root.mainloop()
    finally:
        if not window.closed:
            window.close()


if __name__ == '__main__':
    main()
