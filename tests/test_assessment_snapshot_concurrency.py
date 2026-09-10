"""HTTP batch scoring must not stop ordered GT measurements or finalization."""
import json
import threading
import unittest
from unittest.mock import patch

import test_dashboard_assessment as fixtures
import live_assessment

frame = fixtures.frame

class SnapshotConcurrencyTests(unittest.TestCase):
    setUp = fixtures.SessionTests.setUp
    tearDown = fixtures.SessionTests.tearDown

    def blocked_snapshot(self):
        entered, release = threading.Event(), threading.Event()
        original = live_assessment.evaluate.evaluate_official_for
        output, errors = [], []
        worker = None

        def score(*args, **kwargs):
            if threading.current_thread() is worker and not entered.is_set():
                entered.set()
                if not release.wait(5):
                    raise AssertionError('test did not release HTTP summary')
            return original(*args, **kwargs)

        def run():
            try:
                output.append(self.engine.snapshot())
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=run, daemon=True)
        mock = patch.object(live_assessment.evaluate, 'evaluate_official_for', side_effect=score)
        mock.start()
        worker.start()
        self.assertTrue(entered.wait(2))

        def finish():
            release.set()
            worker.join(3)
            mock.stop()
            self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])

        self.addCleanup(finish)
        return release, worker, output

    def test_frame_can_arrive_during_full_history_scoring(self):
        self.engine.on_frame(frame(0, speed=0))
        release, worker, output = self.blocked_snapshot()
        arrived = threading.Event()

        def receive():
            self.engine.on_frame(frame(.04, speed=0))
            arrived.set()

        producer = threading.Thread(target=receive, daemon=True)
        producer.start()
        try:
            self.assertTrue(arrived.wait(.5), 'HTTP scoring blocked GT measurement lock')
        finally:
            release.set()
            producer.join(2)
            worker.join(2)
        self.assertEqual(len(self.engine.rows), 2)
        self.assertEqual(output[0]['latest']['sim_time_s'], '0')
        self.assertEqual(self.engine.snapshot()['latest']['sim_time_s'], '0.04')

    def test_finalize_during_http_summary_does_not_deadlock_or_lose_deduction(self):
        self.engine.on_frame(frame(0, speed=20))
        release, worker, output = self.blocked_snapshot()
        self.assertTrue(self.engine.finalize_incomplete())
        release.set()
        worker.join(2)
        self.assertEqual(output[0]['status'], 'ABORTED')
        records = [json.loads(line) for line in next((self.path/'output').glob('*.jsonl')).read_text().splitlines()]
        deductions = [r for r in records if r['kind'] == 'deduction' and r['item'] == 1]
        self.assertEqual(len(deductions), 1)
        self.assertEqual(deductions[0]['delta'], 6)
        self.assertEqual(records[-1]['kind'], 'result')
        self.assertEqual(records[-1]['summary']['official_penalty'], -6)

    def test_reset_during_http_summary_cannot_publish_old_session(self):
        self.engine.on_frame(frame(1, speed=20, number=25))
        previous_session = self.engine.session_id
        release, worker, output = self.blocked_snapshot()
        self.engine.on_frame(frame(0, speed=0, number=0))
        self.engine.on_control_received(dict(steering=0, accel=0, indicator=0))
        self.engine.on_frame(frame(.04, speed=0, number=1))
        release.set()
        worker.join(2)
        self.assertNotEqual(self.engine.session_id, previous_session)
        self.assertEqual(output[0]['session_id'], self.engine.session_id)
        self.assertEqual(output[0]['summary']['official_penalty'], 0)
        self.assertEqual(self.engine.deductions, [])


if __name__ == '__main__':
    unittest.main()
