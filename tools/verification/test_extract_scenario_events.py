import tempfile
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path

from extract_scenario_events import CHECK, REPO_ROOT, build_registry, extract_events, main


class ExtractionTests(unittest.TestCase):
    def test_ego_initialization_and_resource_lists_are_not_events(self):
        root = ET.fromstring('''<Scenario xmlns="urn:vtd">
            <VehicleList/><CharacterList/><ObjectList/>
            <TrafficControl><Player><Description Name="Ego"/>
            <Init><Speed Value="10"/><PosAbsolute/></Init></Player></TrafficControl>
            <MovingObjectsControl><PathShape><Waypoint/></PathShape>
            <ObjectActions Object="absent"/></MovingObjectsControl><LightSigns/>
            </Scenario>''')
        self.assertFalse(any(extract_events(root).values()))

    def test_npc_external_control_and_sign_obstacles(self):
        root = ET.fromstring('''<Scenario><TrafficControl>
            <Player><Description Name="Ego"/></Player>
            <Player><Description Name="NPC" Control="external"/></Player>
            </TrafficControl><MovingObjectsControl>
            <Object Name="cone" Type="sign"/><ObjectActions Object="cone"/>
            <Character Name="walker" Class="pedestrian"/>
            <CharacterActions><Action><PosRelative/><Motion Speed="2"/>
            </Action></CharacterActions></MovingObjectsControl>
            <LightSigns><SignalController Id="1"/></LightSigns></Scenario>''')
        flags = extract_events(root)
        for key in ("Vehicle", "Static obstacle", "Pedestrian", "Traffic light", "Trigger"):
            self.assertTrue(flags[key], key)
        self.assertFalse(flags["Speed change"])
        self.assertFalse(flags["Lane change"])

    def test_moving_object_is_not_static_and_absolute_action_is_trigger(self):
        root = ET.fromstring('''<Scenario><MovingObjectsControl>
            <Object Name="moving"/><ObjectActions Object="moving"><Action>
            <PosAbsolute/><Motion Speed="2"/></Action></ObjectActions>
            </MovingObjectsControl></Scenario>''')
        flags = extract_events(root)
        self.assertFalse(flags["Static obstacle"])
        self.assertTrue(flags["Trigger"])

    def test_catalog_order_new_entries_and_bom(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            for name in ("M18", "M01"):
                (base / name).mkdir()
                (base / name / "source.xml").write_text("<Scenario/>", encoding="utf-8")
            catalog = base / "catalog.csv"
            catalog.write_text("scenario_id,package_path\nM18,M18\nV01,ignored\nM01,M01\n",
                               encoding="utf-8-sig")
            self.assertEqual([r["Scenario"] for r in build_registry(catalog)], ["M01", "M18"])

    def test_bad_inputs_preserve_existing_output(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            package = base / "M01"
            package.mkdir()
            catalog = base / "catalog.csv"
            catalog.write_text("scenario_id,package_path\nM01,M01\n", encoding="utf-8")
            output = base / "registry.csv"
            output.write_bytes(b"previous report")
            for case in ("missing", "malformed", "unsupported", "ambiguous", "duplicate"):
                with self.subTest(case=case):
                    if case == "malformed":
                        (package / "M01.xml").write_text("<Scenario>", encoding="utf-8")
                    elif case == "unsupported":
                        (package / "M01.xml").write_text("<OpenSCENARIO/>", encoding="utf-8")
                    elif case == "ambiguous":
                        (package / "extra.xml").write_text("<Scenario/>", encoding="utf-8")
                    elif case == "duplicate":
                        (package / "extra.xml").unlink()
                        (package / "M01.xml").write_text("<Scenario/>", encoding="utf-8")
                        with catalog.open("a", encoding="utf-8") as stream:
                            stream.write("M01,M01\n")
                    with redirect_stderr(StringIO()):
                        self.assertEqual(main(["--catalog", str(catalog), "--output", str(output)]), 1)
                    self.assertEqual(output.read_bytes(), b"previous report")

    def test_reviewed_repository_examples(self):
        rows = {r["Scenario"]: r for r in build_registry(REPO_ROOT / "scenario/scenario_catalog.csv")}
        self.assertEqual(rows["M02"]["Lane change"], CHECK)
        self.assertEqual(rows["M02"]["Speed change"], "")
        self.assertEqual(rows["M03"]["Pedestrian"], CHECK)
        self.assertEqual(rows["M03"]["Traffic light"], CHECK)
        for column in ("Static obstacle", "Lane change", "Speed change"):
            self.assertEqual(rows["M05"][column], "")
        self.assertEqual(rows["M14"]["Pedestrian"], "")
        self.assertEqual(list(rows), [f"M{i:02d}" for i in range(1, 19)])
        for scenario_id in ("M15", "M16", "M17"):
            self.assertTrue(all(v == CHECK for k, v in rows[scenario_id].items() if k != "Scenario"))
        self.assertEqual(rows["M18"]["Lane change"], "")
        self.assertTrue(all(v == CHECK for k, v in rows["M18"].items()
                            if k not in {"Scenario", "Lane change"}))


if __name__ == "__main__":
    unittest.main()
