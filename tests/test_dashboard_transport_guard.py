import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/dashboard'))
from dashboard_api import DashboardRuntime, live_payload


class TransportScoreGuardTests(unittest.TestCase):
    def setUp(self):
        self.cached = dict(summary=dict(official_score=500, official_evaluation='COMPLETE',
                                       score_withheld_reasons=[]), warnings=[])
        self.status = dict(connected=True, transport={}, processing_lag_s=0)
        self.runtime = DashboardRuntime(
            live_assessment=SimpleNamespace(snapshot=lambda: dict(self.cached)),
            rdb_observer=SimpleNamespace(status=self.status))

    def test_backlog_withholds_without_changing_cached_score(self):
        before = copy.deepcopy(self.cached)
        self.status['processing_lag_s'] = 3
        payload = live_payload(self.runtime)
        self.assertIsNone(payload['summary']['official_score'])
        self.assertIn('assessment_processing_backlog', payload['summary']['score_withheld_reasons'])
        self.assertEqual(self.cached, before)
        self.status['processing_lag_s'] = 0
        self.assertEqual(live_payload(self.runtime)['summary']['official_score'], 500)

    def test_disk_or_processing_error_cannot_appear_complete(self):
        for state in ({'shutdown_incomplete': True}, {'processing_error': 'callback failed'},
                      {'transport': {'error': 'disk full'}}):
            with self.subTest(state=state):
                self.status.clear()
                self.status.update(state)
                payload = live_payload(self.runtime)
                self.assertIsNone(payload['summary']['official_score'])
                self.assertEqual(payload['summary']['official_evaluation'], 'PARTIAL')
                self.assertIn('incomplete_transport_evidence', payload['summary']['score_withheld_reasons'])


if __name__ == '__main__':
    unittest.main()
