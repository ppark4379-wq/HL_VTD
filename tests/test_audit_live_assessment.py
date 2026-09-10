"""Independent audit regressions: lane renumbering and signal evidence gaps."""
import tempfile
import math
import json
from argparse import Namespace
from pathlib import Path
import unittest

from tools.verification.audit_live_assessment import LaneTopology, indicator_history, run_audit, scenario_entities


class AuditTests(unittest.TestCase):
    def test_all_entity_kinds_preserve_exact_names(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'scenario.xml'
            path.write_text('''<Scenario><Player><Description Name="Ego"/></Player>
            <Character Name="Walker "/><Object Name="Cone"/><Object Name="Cone "/>
            <ObjectActions Object="Cone"/><CharacterActions Character="Walker "/>
            </Scenario>''')
            entities=scenario_entities(path)
            self.assertEqual([(e['kind'],e['name']) for e in entities],
                [('Player','Ego'),('Character','Walker '),('Object','Cone'),('Object','Cone ')])

    def test_empty_capture_is_never_a_successful_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'assessment.jsonl'
            path.write_text(json.dumps(dict(kind='session'))+'\n')
            with self.assertRaisesRegex(ValueError,'with frame rows'):
                run_audit(Namespace(assessment=path))

    def test_same_frame_time_from_other_session_does_not_match(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            session=dict(kind='session',start_sim_time=0,rules={})
            row=dict(kind='frame',frame='1',sim_time_s='0',received_monotonic_ns='10000000000',
                elapsed_wall_time_s='0',ego_x='1',ego_y='2',ego_speed='0',distance_m='0',evaluation_section_id='1')
            (folder/'assessment.jsonl').write_text('\n'.join(json.dumps(r) for r in (session,row))+'\n')
            ego=dict(id=1,name='Ego',x=1,y=2,h=0,speed=0)
            foreign=dict(frame=1,time=0,received_monotonic_ns=9000000000,objects={'1':ego})
            own=dict(foreign,received_monotonic_ns=10000000000)
            (folder/'frames.jsonl').write_text('\n'.join(json.dumps(r) for r in (foreign,own))+'\n')
            (folder/'scenario.xml').write_text('<Scenario><Player><Description Name="Ego"/></Player></Scenario>')
            (folder/'map.xodr').write_text('<OpenDRIVE/>')
            args=Namespace(assessment=folder/'assessment.jsonl',frames=folder/'frames.jsonl',
                scenario=folder/'scenario.xml',xodr=folder/'map.xodr',controller=None,ego_name='Ego')
            report=run_audit(args)
            self.assertEqual(report['transport']['matched_frames'],1)
            self.assertEqual(report['transport']['ambiguous_frame_time_keys'],[])
            self.assertEqual(report['audit_status'],'PARTIAL_EVIDENCE')
            self.assertEqual(report['items'][0]['scoring_input_coverage']['posted_speed_limit_kmh']['missing'],1)

    def test_positive_lane_ids_reverse_left_right_against_reference(self):
        xml = '''<OpenDRIVE><road id="1"><planView><geometry s="0" x="0" y="0" hdg="0" length="30"><line/></geometry></planView><lanes>
        <laneSection s="0"><left><lane id="1"><width sOffset="0" a="3" b="0" c="0" d="0"/></lane>
        <lane id="2"><width sOffset="0" a="3" b="0" c="0" d="0"/></lane></left></laneSection>
        </lanes></road></OpenDRIVE>'''
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'map.xodr'
            path.write_text(xml)
            topology=LaneTopology(path)
            self.assertEqual(topology.direction(1,10,2,1,math.pi)['expected'],'LEFT')
            self.assertEqual(topology.direction(1,10,2,1,0)['expected'],'RIGHT')

    def test_lane_section_renumber_is_continuation_both_directions(self):
        xml = '''<OpenDRIVE><road id="1"><lanes>
          <laneSection s="0"><left>
            <lane id="1"><link><successor id="2"/></link></lane>
            <lane id="2"><link><successor id="3"/></link></lane>
          </left></laneSection>
          <laneSection s="10"><left>
            <lane id="2"><link><predecessor id="1"/></link></lane>
            <lane id="3"><link><predecessor id="2"/></link></lane>
          </left></laneSection>
        </lanes></road></OpenDRIVE>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'map.xodr'
            path.write_text(xml)
            topology = LaneTopology(path)
            before = dict(road=1,lane=1,s=9.99)
            after = dict(road=1,lane=2,s=10.01)
            self.assertEqual(topology.classify(before,after)['classification'],'LANE_CONTINUATION_RENUMBER')
            self.assertEqual(topology.classify(after,before)['classification'],'LANE_CONTINUATION_RENUMBER')
            self.assertEqual(topology.classify(before,dict(road=1,lane=3,s=10.01))['classification'],'LANE_CHANGE_CANDIDATE')

    def test_short_correct_signal_is_violation_only_with_known_history(self):
        raw = [dict(time=i/10,indicator='LEFT' if i>=5 else 'OFF') for i in range(36)]
        result = indicator_history(raw,35,'LEFT',.5)
        self.assertEqual(result['verdict'],'CORRECT_INDICATOR_OBSERVED_FOR_3S')
        result = indicator_history(raw,34,'LEFT',.5)
        self.assertEqual(result['verdict'],'INDICATOR_VIOLATION_CANDIDATE')
        raw[30]['indicator'] = None
        self.assertEqual(indicator_history(raw,35,'LEFT',.5)['verdict'],'UNKNOWN')

    def test_missing_time_interval_does_not_prove_three_second_signal(self):
        raw = [dict(time=0,indicator='LEFT'),dict(time=4,indicator='LEFT')]
        self.assertEqual(indicator_history(raw,1,'LEFT',.5)['verdict'],'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
