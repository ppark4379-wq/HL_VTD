import struct
import unittest
from unittest.mock import Mock

from tools.manual_drive_wasd import (
    DriverWindow, ENTRY, HEADER, MAGIC, RdbStream, controls, driver_packet,
)


class ManualDriveTests(unittest.TestCase):
    def test_keys_and_release(self):
        left, throttle, brake = controls({'w', 'a'}, True, 0, 0.02)
        self.assertGreater(left, 0)
        self.assertEqual((throttle, brake), (0.25, 0))
        right, _, _ = controls({'d'}, True, 0, 0.02)
        self.assertLess(right, 0)
        self.assertEqual(controls(set(), True, 0, .02), (0, 0, .2))

    def test_brake_overrides_throttle_and_focus(self):
        self.assertEqual(controls({'w', 's'}, True, 0, .02)[1:], (0, .6))
        self.assertEqual(controls({'w', 'space'}, True, 0, .02)[1:], (0, 1))
        self.assertEqual(controls({'w', 'a'}, False, 1, .02), (0, 0, 1))

    def test_packet_layout(self):
        packet = driver_packet(7, 12, 1.25, .5, .25, 0)
        self.assertEqual(len(packet), 120)
        self.assertEqual(HEADER.unpack_from(packet), (MAGIC, 0x125, 24, 96, 12, 1.25))
        self.assertEqual(ENTRY.unpack_from(packet, 24), (16, 80, 80, 26, 0))
        self.assertEqual(struct.unpack_from('<Iffff', packet, 40), (7, .5, 0, .25, 0))
        self.assertEqual(packet[40 + 52], 4)
        self.assertEqual(struct.unpack_from('<I', packet, 40 + 56)[0], 0x208d)

    def test_fragmented_coalesced_messages_and_ego_detection(self):
        obj = bytearray(112)
        struct.pack_into('<I', obj, 0, 42)
        obj[8:12] = b'Ego\0'
        payload = ENTRY.pack(16, 112, 112, 9, 0) + obj
        packet = HEADER.pack(MAGIC, 0x125, 24, len(payload), 8, .4) + payload
        stream = RdbStream()
        self.assertFalse(stream.feed(packet[:15]))
        self.assertIsNone(stream.player)
        self.assertTrue(stream.feed(packet[15:] + packet))
        self.assertEqual(stream.player, 42)
        self.assertEqual(stream.clock, (8, .4))
        self.assertEqual(stream.buffer, b'')

    def test_reject_invalid_header_and_entry(self):
        stream = RdbStream()
        with self.assertRaises(ValueError):
            stream.feed(HEADER.pack(0, 0, 24, 0, 0, 0))
        stream = RdbStream()
        with self.assertRaises(ValueError):
            stream.feed(HEADER.pack(MAGIC, 0x125, 24, 16, 0, 0)
                        + ENTRY.pack(16, 112, 0, 9, 0))

    def test_focus_loss_clears_keys(self):
        window = DriverWindow(Mock(), Mock(), Mock())
        window.keys = {'w', 'a'}
        window.focused = True
        window.focus_out(None)
        self.assertFalse(window.focused)
        self.assertEqual(window.keys, set())

    def test_close_sends_brake_and_closes(self):
        sock, root = Mock(), Mock()
        window = DriverWindow(root, sock, Mock())
        window.stream.player = 42
        window.stream.clock = (12, 1.25)
        window.close()
        packet = sock.sendall.call_args.args[0]
        self.assertEqual(struct.unpack_from('<ff', packet, 40 + 12), (0, 1))
        sock.close.assert_called_once()
        root.destroy.assert_called_once()
        window.close()
        sock.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
