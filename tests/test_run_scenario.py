#!/usr/bin/env python3

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "verification"))

from run_scenario import (  # noqa: E402
    TrafficHealthMonitor,
    scenario_requires_traffic,
)


class TrafficHealthMonitorTests(unittest.TestCase):
    def test_old_warnings_are_ignored_when_monitor_is_armed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.write_text(
                "no traffic info available for 5000 wait frames\n",
                encoding="utf-8",
            )
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            self.assertIsNone(monitor.poll(now=10.0))

    def test_single_transient_warning_recovers_without_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.touch()
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            with log.open("a", encoding="utf-8") as stream:
                stream.write("no traffic info available for 100 wait frames\n")
            self.assertIsNone(monitor.poll(now=10.0))
            self.assertIsNone(monitor.poll(now=10.7))
            self.assertIsNone(monitor.poll(now=11.2))

    def test_repeated_missing_traffic_fails_after_grace_period(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.touch()
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            for now in (10.0, 10.3, 10.6, 10.9):
                with log.open("a", encoding="utf-8") as stream:
                    stream.write(
                        "no traffic info available for 5000 wait frames\n"
                    )
                self.assertIsNone(monitor.poll(now=now))
            with log.open("a", encoding="utf-8") as stream:
                stream.write("waiting for connection <Traffic>....\n")
            failure = monitor.poll(now=11.1)
            self.assertIsNotNone(failure)
            self.assertIn("Traffic health check failed", failure)

    def test_connected_message_clears_pending_loss(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "TaskControl.txt"
            log.touch()
            monitor = TrafficHealthMonitor(log, 1.0, monitor_process=False)
            monitor.arm()
            with log.open("a", encoding="utf-8") as stream:
                stream.write("waiting for connection <Traffic>....\n")
            self.assertIsNone(monitor.poll(now=10.0))
            with log.open("a", encoding="utf-8") as stream:
                stream.write('NOTICE: "<Traffic> connected"\n')
            self.assertIsNone(monitor.poll(now=11.5))

    def test_internal_player_requires_traffic_but_empty_scene_does_not(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            internal = Path(directory) / "internal.xml"
            internal.write_text(
                "<Scenario><TrafficControl><Player><Description "
                'Name="NPC" Control="internal"/></Player>'
                "</TrafficControl></Scenario>",
                encoding="utf-8",
            )
            empty = Path(directory) / "empty.xml"
            empty.write_text(
                "<Scenario><TrafficControl/></Scenario>", encoding="utf-8"
            )
            self.assertTrue(scenario_requires_traffic(internal))
            self.assertFalse(scenario_requires_traffic(empty))


if __name__ == "__main__":
    unittest.main()
