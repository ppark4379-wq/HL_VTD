#!/usr/bin/env python3

from __future__ import annotations

import socket
import sys
import tempfile
import threading
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "verification"))

from vtd_scp import (  # noqa: E402
    SCP_HEADER,
    SCP_MAGIC,
    VtdScpClient,
    pack_message,
    scenario_load_command,
)


def receive_packet(connection: socket.socket) -> str:
    header = bytearray()
    while len(header) < SCP_HEADER.size:
        header.extend(connection.recv(SCP_HEADER.size - len(header)))
    magic, _, _, _, size = SCP_HEADER.unpack(header)
    if magic != SCP_MAGIC:
        raise AssertionError(f"bad SCP magic: {magic}")
    payload = bytearray()
    while len(payload) < size:
        payload.extend(connection.recv(size - len(payload)))
    return payload.decode("utf-8")


class VtdScpTests(unittest.TestCase):
    def test_pack_message_uses_vtd_header(self) -> None:
        packet = pack_message("<Info/>")
        magic, version, sender, receiver, size = SCP_HEADER.unpack_from(packet)
        self.assertEqual(magic, 40108)
        self.assertEqual(version, 1)
        self.assertEqual(sender.rstrip(bytes(1)), b"HL_VTD_Runner")
        self.assertEqual(receiver.rstrip(bytes(1)), b"TaskControl")
        self.assertEqual(packet[SCP_HEADER.size:], b"<Info/>")
        self.assertEqual(size, len(b"<Info/>"))

    def test_scenario_is_embedded_for_remote_vtd(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.xml"
            path.write_text(
                '<?xml version="1.0"?><Scenario RevMajor="2025" RevMinor="2">'
                "<TrafficControl/></Scenario>",
                encoding="utf-8",
            )
            command = scenario_load_command(path, inline=True)
        self.assertTrue(command.startswith("<SimCtrl><LoadScenario><Scenario"))
        self.assertIn("<TrafficControl", command)
        self.assertTrue(command.endswith("</Scenario></LoadScenario></SimCtrl>"))

    def test_scenario_file_load_matches_vtd_gui_path(self) -> None:
        command = scenario_load_command(Path('/tmp/V04_cut_in.xml'))
        self.assertEqual(
            command,
            '<SimCtrl><LoadScenario filename="/tmp/V04_cut_in.xml"/></SimCtrl>',
        )

    def test_wait_for_state_queries_until_expected_response(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        port = server.getsockname()[1]
        received: list[str] = []

        def serve() -> None:
            connection, _ = server.accept()
            with connection:
                received.append(receive_packet(connection))
                response = (
                    '<Info><SimState source="TaskControl" state="READY"/></Info>'
                    "\x00"
                )
                connection.sendall(
                    pack_message(response, sender="TaskControl", receiver="any")
                )
            server.close()

        thread = threading.Thread(target=serve)
        thread.start()
        with VtdScpClient("127.0.0.1", port, timeout=2.0) as client:
            client.wait_for_state("READY", timeout=2.0)
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(
            received, ['<Query entity="taskControl"><SimState/></Query>']
        )

    def test_full_configure_start_stop_sequence(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        port = server.getsockname()[1]
        received: list[str] = []

        def respond(connection: socket.socket, state: str) -> None:
            response = (
                f'<Info><SimState source="TaskControl" state="{state}"/></Info>'
            )
            connection.sendall(
                pack_message(response, sender="TaskControl", receiver="any")
            )

        def respond_scenario(connection: socket.socket, filename: str) -> None:
            response = (
                '<Reply entity="taskControl"><Scenario '
                f'filename="{filename}" contentViaSCP="false"/></Reply>'
            )
            connection.sendall(
                pack_message(response, sender="TaskControl", receiver="any")
            )

        def serve() -> None:
            state = "READY"
            connection, _ = server.accept()
            with connection:
                for _ in range(2):
                    received.append(receive_packet(connection))
            connection, _ = server.accept()
            with connection:
                for _ in range(10):
                    command = receive_packet(connection)
                    received.append(command)
                    if command == "<SimCtrl><Init/></SimCtrl>":
                        state = "POST-INIT"
                    elif command == "<SimCtrl><Start/></SimCtrl>":
                        state = "RUN"
                    elif command == "<SimCtrl><Stop/></SimCtrl>":
                        state = "READY"
                    elif "<SimState/>" in command:
                        respond(connection, state)
                    elif "<Scenario/>" in command:
                        respond_scenario(connection, str(scenario))
            server.close()

        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "test.xml"
            scenario.write_text("<Scenario><TrafficControl/></Scenario>")
            thread = threading.Thread(target=serve)
            thread.start()
            with VtdScpClient("127.0.0.1", port, timeout=2.0) as client:
                client.configure_scenario(
                    scenario, timeout=2.0, settle_delay=0.05
                )
                client.initialize(timeout=2.0)
                client.start(timeout=2.0)
                client.stop(timeout=2.0)
            thread.join(timeout=2.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(received[0], "<SimCtrl><Stop/></SimCtrl>")
        self.assertEqual(received[1], "<SimCtrl><Config/></SimCtrl>")
        self.assertEqual(received[2], "<SimCtrl><Apply/></SimCtrl>")
        self.assertTrue(received[4].startswith("<SimCtrl><LoadScenario filename="))
        self.assertEqual(received[6], "<SimCtrl><Init/></SimCtrl>")
        self.assertEqual(received[8], "<SimCtrl><Start/></SimCtrl>")
        self.assertEqual(received[10], "<SimCtrl><Stop/></SimCtrl>")


if __name__ == "__main__":
    unittest.main()
