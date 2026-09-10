import pathlib
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'tools/dashboard'))
from scp_observation import HEADER, ScpObservationDecoder, PassiveScpObserver, parse_plugin_respawn_line
from rdb_stream import Decoder


def message(xml):
    data = xml.encode()
    return HEADER.pack(40108, 1, b'TaskControl', b'all', len(data)) + data


class PassiveScpTests(unittest.TestCase):
    def test_plugin_phase_retains_exact_frame_but_does_not_guess_completion(self):
        event = parse_plugin_respawn_line('[RESPAWN] phase=GRACE reason=rdb_offroad section=2 distance=0.300m frame=240 simTime=9.600')
        self.assertEqual(event['plugin_frame'], 240)
        self.assertEqual(event['plugin_sim_time_s'], 9.6)
        self.assertFalse(event['officially_confirmed_respawn'])
        self.assertEqual(parse_plugin_respawn_line('[RESPAWN] operator position change accepted section=2')['interpretation'],
                         'operator_reposition_not_automatic_respawn')
        self.assertIsNone(parse_plugin_respawn_line('unrelated traffic log'))

    def test_fragmented_offroad_is_cause_not_respawn_completion(self):
        raw = message('<SimCtrl><Assessment reason="offroad" continue="auto"/></SimCtrl>')
        decoder = ScpObservationDecoder()
        self.assertEqual(decoder.feed(raw[:139]), [])
        event, = decoder.feed(raw[139:])
        self.assertEqual(event['attributes']['reason'], 'offroad')
        self.assertFalse(event['officially_confirmed_respawn'])
        self.assertNotIn('player_id', event)
        self.assertNotIn('sim_time_s', event)

    def test_commands_do_not_prove_completion(self):
        events = ScpObservationDecoder().feed(message(
            '<SimCtrl><Reset/><Restart/><Set player="Ego"/></SimCtrl>'))
        self.assertEqual(len(events), 3)
        self.assertTrue(all(not e['officially_confirmed_respawn'] for e in events))

    def test_invalid_header_and_entity_are_rejected(self):
        with self.assertRaises(ValueError):
            ScpObservationDecoder().feed(bytes(HEADER.size))
        with self.assertRaises(ValueError):
            ScpObservationDecoder().feed(message('<!DOCTYPE x><SimCtrl/>'))

    def test_observer_never_sends_any_bytes(self):
        events = []
        observer = PassiveScpObserver('127.0.0.1', 48179, events.append)
        class Connection:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def settimeout(self, value): pass
            def recv(self, size):
                observer.stop_event.set()
                return message('<SimCtrl><Assessment reason="offroad" continue="auto"/></SimCtrl>')
            def send(self, *args): raise AssertionError('SCP sends forbidden')
            def sendall(self, *args): raise AssertionError('SCP sends forbidden')
        with patch('scp_observation.socket.create_connection', return_value=Connection()):
            observer.run()
        self.assertEqual(len(events), 1)
        self.assertIn('received_monotonic_ns', events[0])

    def test_scoring_offroad_collision_are_not_respawns(self):
        decoder = Decoder()
        decoder.frame = dict(scoring={})
        data = struct.pack('<IffffIf4I', 1, 2., 3., 0., 0., 3, .1, 0, 0, 0, 0)
        decoder._element(10000, 0, data)
        evidence = decoder.frame['scoring'][1]
        self.assertTrue(evidence['offroad_reported'])
        self.assertTrue(evidence['collision_reported'])
        self.assertNotIn('respawn_count', evidence)
        self.assertEqual(evidence['raw_hex'], data.hex())


if __name__ == '__main__':
    unittest.main()
