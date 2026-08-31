#!/usr/bin/env python3
"""Minimal VTD Simulation Control Protocol client for scenario automation."""

from __future__ import annotations

import socket
import struct
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import quoteattr


SCP_MAGIC = 40108
SCP_VERSION = 1
SCP_NAME_LENGTH = 64
SCP_HEADER = struct.Struct("<HH64s64sI")


class ScpError(RuntimeError):
    """Raised when VTD SCP communication or a state transition fails."""


def _name_field(value: str) -> bytes:
    encoded = value.encode("utf-8")
    if len(encoded) >= SCP_NAME_LENGTH:
        raise ValueError(f"SCP name is too long: {value!r}")
    return encoded + bytes(SCP_NAME_LENGTH - len(encoded))


def pack_message(
    payload: str,
    sender: str = "HL_VTD_Runner",
    receiver: str = "TaskControl",
) -> bytes:
    data = payload.encode("utf-8")
    return SCP_HEADER.pack(
        SCP_MAGIC,
        SCP_VERSION,
        _name_field(sender),
        _name_field(receiver),
        len(data),
    ) + data


def scenario_load_command(path: Path, *, inline: bool = False) -> str:
    """Build VTD LoadScenario using its normal file path or inline XML."""
    if not inline:
        return f"<SimCtrl><LoadScenario filename={quoteattr(str(path))}/></SimCtrl>"
    try:
        scenario = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ScpError(f"cannot read VTD scenario XML: {path}: {error}") from error
    if scenario.tag != "Scenario":
        raise ScpError(f"VTD scenario root must be <Scenario>: {path}")
    content = ET.tostring(scenario, encoding="unicode")
    return f"<SimCtrl><LoadScenario>{content}</LoadScenario></SimCtrl>"


class VtdScpClient:
    def __init__(self, host: str, port: int = 48179, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: socket.socket | None = None
        self._buffer = bytearray()

    def __enter__(self) -> "VtdScpClient":
        self.connect()
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def connect(self, timeout: float | None = None) -> None:
        if self._socket is not None:
            return
        connect_timeout = self.timeout if timeout is None else timeout
        try:
            self._socket = socket.create_connection(
                (self.host, self.port), timeout=connect_timeout
            )
            self._socket.settimeout(self.timeout)
        except OSError as error:
            raise ScpError(
                f"cannot connect to VTD SCP at {self.host}:{self.port}: {error}"
            ) from error

    def reconnect(self, timeout: float) -> None:
        self.close()
        deadline = time.monotonic() + timeout
        last_error: ScpError | None = None
        while time.monotonic() < deadline:
            try:
                self.connect(timeout=min(1.0, max(0.1, deadline - time.monotonic())))
                return
            except ScpError as error:
                last_error = error
                time.sleep(0.2)
        detail = f": {last_error}" if last_error is not None else ""
        raise ScpError(f"VTD SCP did not reconnect within {timeout:g}s{detail}")

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._buffer.clear()

    def send(self, payload: str) -> None:
        if self._socket is None:
            raise ScpError("VTD SCP client is not connected")
        try:
            self._socket.sendall(pack_message(payload))
        except OSError as error:
            raise ScpError(f"failed to send VTD SCP command: {error}") from error

    def receive(self, timeout: float | None = None) -> tuple[str, str, str]:
        if self._socket is None:
            raise ScpError("VTD SCP client is not connected")
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            message = self._pop_message()
            if message is not None:
                return message
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for a VTD SCP response")
            self._socket.settimeout(remaining)
            try:
                chunk = self._socket.recv(65536)
            except socket.timeout as error:
                raise TimeoutError(
                    "timed out waiting for a VTD SCP response"
                ) from error
            except OSError as error:
                raise ScpError(f"failed to receive VTD SCP response: {error}") from error
            if not chunk:
                raise ScpError("VTD closed the SCP connection")
            self._buffer.extend(chunk)

    def _pop_message(self) -> tuple[str, str, str] | None:
        if len(self._buffer) < SCP_HEADER.size:
            return None
        magic, version, sender, receiver, data_size = SCP_HEADER.unpack_from(
            self._buffer
        )
        if magic != SCP_MAGIC or version != SCP_VERSION:
            raise ScpError(
                f"invalid VTD SCP header: magic={magic}, version={version}"
            )
        total_size = SCP_HEADER.size + data_size
        if len(self._buffer) < total_size:
            return None
        payload_bytes = bytes(self._buffer[SCP_HEADER.size:total_size]).rstrip(
            bytes(1)
        )
        payload = payload_bytes.decode("utf-8", errors="replace")
        del self._buffer[:total_size]
        return (
            sender.split(bytes(1), 1)[0].decode("utf-8", errors="replace"),
            receiver.split(bytes(1), 1)[0].decode("utf-8", errors="replace"),
            payload,
        )

    def wait_for_state(self, expected: str, timeout: float) -> None:
        expected_normalized = expected.upper().replace("_", "-")
        deadline = time.monotonic() + timeout
        next_query = 0.0
        last_state = "unknown"
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self.send('<Query entity="taskControl"><SimState/></Query>')
                next_query = now + 0.5
            try:
                _, _, payload = self.receive(
                    timeout=min(0.5, max(0.01, deadline - time.monotonic()))
                )
            except TimeoutError:
                continue
            try:
                root = ET.fromstring(payload)
            except ET.ParseError:
                continue
            for element in root.iter("SimState"):
                state = element.attrib.get("state", "")
                source = element.attrib.get("source", "TaskControl")
                if source.lower() != "taskcontrol":
                    continue
                last_state = state or last_state
                normalized = state.upper().replace("_", "-")
                if normalized == expected_normalized:
                    return
        raise ScpError(
            f"VTD did not reach {expected} within {timeout:g}s "
            f"(last state: {last_state})"
        )

    def get_state(self, timeout: float) -> str | None:
        self.send('<Query entity="taskControl"><SimState/></Query>')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                _, _, payload = self.receive(
                    timeout=min(0.5, max(0.01, deadline - time.monotonic()))
                )
            except TimeoutError:
                continue
            try:
                root = ET.fromstring(payload)
            except ET.ParseError:
                continue
            for element in root.iter("SimState"):
                source = element.attrib.get("source", "TaskControl")
                if source.lower() == "taskcontrol":
                    state = element.attrib.get("state")
                    if state:
                        return state.upper().replace("_", "-")
        return None

    def wait_for_scenario(self, expected: Path, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        next_query = 0.0
        last_filename = "none"
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self.send('<Query entity="taskControl"><Scenario/></Query>')
                next_query = now + 0.5
            try:
                _, _, payload = self.receive(
                    timeout=min(0.5, max(0.01, deadline - time.monotonic()))
                )
            except TimeoutError:
                continue
            try:
                root = ET.fromstring(payload)
            except ET.ParseError:
                continue
            for element in root.iter("Scenario"):
                filename = element.attrib.get("filename", "")
                last_filename = filename or last_filename
                if filename == str(expected):
                    return
        raise ScpError(
            f"VTD did not register scenario {expected} within {timeout:g}s "
            f"(last scenario: {last_filename})"
        )

    def configure_scenario(
        self,
        scenario_path: Path,
        timeout: float,
        *,
        inline: bool = False,
        settle_delay: float = 1.0,
    ) -> None:
        state = self.get_state(min(2.0, timeout))
        if state in {"RUN", "PAUSE", "POST-INIT", "INIT", "WAIT-FOR-INIT-CONFIRM"}:
            self.send("<SimCtrl><Stop/></SimCtrl>")
            self.wait_for_state("READY", timeout)
            state = "READY"
        if state != "READY":
            self.send("<SimCtrl><Stop/></SimCtrl>")
            self.send("<SimCtrl><Config/></SimCtrl>")
            time.sleep(max(0.0, settle_delay))
            self.reconnect(timeout)
            self.send("<SimCtrl><Apply/></SimCtrl>")
            self.wait_for_state("READY", timeout)
        self.send(scenario_load_command(scenario_path, inline=inline))
        if inline:
            time.sleep(max(0.0, settle_delay))
        else:
            self.wait_for_scenario(scenario_path, timeout)

    def initialize(self, timeout: float) -> None:
        self.send("<SimCtrl><Init/></SimCtrl>")
        self.wait_for_state("POST-INIT", timeout)

    def show_camera_owner(self, camera_name: str) -> None:
        quoted_name = quoteattr(camera_name)
        self.send(
            f"<Camera name={quoted_name} showOwner=\"true\"><Set/></Camera>"
        )

    def start(self, timeout: float) -> None:
        self.send("<SimCtrl><Start/></SimCtrl>")
        self.wait_for_state("RUN", timeout)

    def stop(self, timeout: float) -> None:
        self.send("<SimCtrl><Stop/></SimCtrl>")
        self.wait_for_state("READY", timeout)
