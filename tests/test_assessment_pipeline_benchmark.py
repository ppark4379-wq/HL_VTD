"""The offline load benchmark must not alter its source or invent GT input."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/verification'))
from benchmark_assessment_pipeline import benchmark, load_evidence


class AssessmentPipelineBenchmarkTests(unittest.TestCase):
    def evidence(self, path):
        records = [{'kind': 'session', 'session_id': 'fixture', 'rules': {
            'base_score': 500, 'normal_speed_kmh': 50, 'school_speed_kmh': 30,
            'session': {'max_sample_gap_s': .5, 'time_limit_s': 900,
                        'stop_speed_kmh': 0}, 'metadata': {}}}]
        for index in range(1, 5):
            records.append({'kind': 'frame', 'frame': index, 'scenario_id': 'TEST',
                'run_id': 'fixture', 'sim_time_s': index * .04,
                'elapsed_wall_time_s': index * .04, 'ego_speed': 0,
                'evaluation_section_id': '1', 'max_sample_gap_s': .5,
                'sample_boolean': True, 'sample_missing': None})
        path.write_text('\n'.join(json.dumps(record) for record in records) + '\n', encoding='utf-8')

    def test_loaded_rows_match_live_scalar_representation_and_preserve_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'evidence.jsonl'
            self.evidence(source)
            before = source.read_bytes()
            _, rows, digest = load_evidence(source)
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[0]['sample_boolean'], '1')
            self.assertEqual(rows[0]['sample_missing'], '')
            self.assertNotIn('kind', rows[0])
            self.assertEqual(len(digest), 64)
            self.assertEqual(source.read_bytes(), before)

    def test_snapshot_comparison_and_concurrent_append_remain_coherent(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'evidence.jsonl'
            self.evidence(source)
            before = source.read_bytes()
            result = benchmark(source, repeats=1, profile_lines=3)
            self.assertTrue(result['checks_passed'])
            self.assertEqual(result['row_count'], 4)
            self.assertEqual(result['contention']['rows_after_append'], 4)
            self.assertTrue(result['contention']['appended_row_preserved'])
            self.assertEqual(result['serialized_lock_counterfactual']['mode'], 'SIMULATED_OUTER_LOCK')
            self.assertEqual(source.read_bytes(), before)

    def test_missing_frames_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'empty.jsonl'
            source.write_text('{"kind":"session","session_id":"empty"}\n', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'session and frame'):
                load_evidence(source)


if __name__ == '__main__':
    unittest.main()
