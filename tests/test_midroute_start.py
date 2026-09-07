"""Middle starts must trim the guide and advance the mission together."""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "tools" / "verification"))

from sync_vtd_scenario import (
    EgoAbsolutePosition, SyncError, read_route_sync_options,
    sync_manifest_from_xml, trim_route_to_ego_start,
)
from run_scenario import build_command
import xml.etree.ElementTree as ET
import yaml


class MidrouteStartTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.package = Path(self.temporary.name)
        self.route = self.package / "route.csv"
        self.checkpoints = self.package / "checkpoints.csv"
        with self.route.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["x", "y", "yaw", "s", "road_id", "lane_id", "road_s"])
            for x in range(91):
                writer.writerow([x, 0, 0, x, 1, -1, x])
        with self.checkpoints.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["seq", "x", "y"])
            for seq in range(1, 11):
                writer.writerow([seq, (seq - 1) * 10, 0])

    def test_middle_start_preserves_checkpoints_and_selects_remaining_leg(self):
        original = self.checkpoints.read_bytes()
        leg = trim_route_to_ego_start(
            self.route, EgoAbsolutePosition(65, 6.5), 8.0, self.checkpoints)
        self.assertEqual(leg, 3)
        with self.route.open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(float(rows[0]["x"]), 65)
        self.assertEqual(float(rows[0]["s"]), 0)
        self.assertEqual(float(rows[-1]["s"]), 25)
        self.assertEqual(self.checkpoints.read_bytes(), original)

    def test_returning_to_start_restores_first_leg(self):
        leg = trim_route_to_ego_start(
            self.route, EgoAbsolutePosition(0, 0), checkpoints_path=self.checkpoints)
        self.assertEqual(leg, 0)

    def test_exit_anchor_starts_next_leg(self):
        leg = trim_route_to_ego_start(
            self.route, EgoAbsolutePosition(60, 0), checkpoints_path=self.checkpoints)
        self.assertEqual(leg, 3)

    def test_distant_start_and_finish_do_not_overwrite_route(self):
        original = self.route.read_bytes()
        for ego in (EgoAbsolutePosition(65, 9), EgoAbsolutePosition(90, 0)):
            with self.subTest(ego=ego), self.assertRaises(SyncError):
                trim_route_to_ego_start(self.route, ego, 8.0, self.checkpoints)
            self.assertEqual(self.route.read_bytes(), original)

    def test_sync_replaces_previous_start_leg(self):
        manifest = self.package / "scenario.yaml"
        manifest.write_text("runtime:\n  mission_start_leg: 3\nentities: []\n")
        sync_manifest_from_xml(
            self.package, "M13.xml", ET.fromstring("<Scenario/>"), 0)
        self.assertEqual(yaml.safe_load(manifest.read_text())["runtime"]["mission_start_leg"], 0)

    def test_invalid_trim_distance_rejected(self):
        for value in ("true", "0", "-1", ".nan", ".inf"):
            with self.subTest(value=value):
                (self.package / "scenario.yaml").write_text(
                    f"route:\n  trim_max_distance_m: {value}\n")
                with self.assertRaises(SyncError):
                    read_route_sync_options(self.package)

    def test_runner_passes_resolved_leg_to_controller(self):
        command = build_command(
            Path("hl_vtd"), Path("network.yaml"), self.route,
            "M13", "test", Path("run.csv"),
            checkpoints_file=self.checkpoints, mission_start_leg=3)
        index = command.index("--mission-start-leg")
        self.assertEqual(command[index + 1], "3")


if __name__ == "__main__":
    unittest.main()
