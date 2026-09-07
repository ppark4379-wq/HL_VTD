"""Geometry/topology regression tests with hand-checkable OpenDRIVE fixtures."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from scipy.integrate import quad

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/map_precompute'))
from xodr_network import Geometry, LaneKey, LaneNetwork, Road
from precompute_lane_network import export
from render_lane_network import Drawing


def lane(lid, link='', a=3.5, b=0., c=0., d=0., mark='broken', change='both'):
    return (f'<lane id="{lid}" type="driving"><link>{link}</link>'
            f'<width sOffset="0" a="{a}" b="{b}" c="{c}" d="{d}"/>'
            f'<roadMark sOffset="0" type="{mark}" color="white" width=".13" laneChange="{change}"/></lane>')


def section(s, right, left=''):
    return f'<laneSection s="{s}"><left>{left}</left><center><lane id="0" type="none"/></center><right>{right}</right></laneSection>'


def road(rid=1, length=40, sections=None, links='', x=0, y=0, h=0, shape='<line/>'):
    if sections is None:
        sections=section(0,lane(-1)+lane(-2),lane(1))
    return (f'<road id="{rid}" length="{length}" junction="-1" rule="RHT"><link>{links}</link>'
            f'<planView><geometry s="0" x="{x}" y="{y}" hdg="{h}" length="{length}">{shape}</geometry></planView>'
            f'<lanes>{sections}</lanes></road>')


class PrecomputedLaneTests(unittest.TestCase):
    def make_network(self, roads, junction=''):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        source=Path(temp.name)/'fixture.xodr'
        source.write_text('<OpenDRIVE>'+roads+junction+'</OpenDRIVE>')
        return LaneNetwork(source)

    def test_lane_centers_travel_direction_and_adjacent_permissions(self):
        n=self.make_network(road())
        a=n.lanes[LaneKey(1,0,-1)].points
        b=n.lanes[LaneKey(1,0,1)].points
        self.assertAlmostEqual(a[0].y,-1.75)
        self.assertAlmostEqual(a[-1].x,40)
        self.assertAlmostEqual(b[0].x,40)
        self.assertAlmostEqual(abs(b[0].heading),math.pi)
        self.assertEqual(a[0].left_id,-1)  # no crossing centerline
        self.assertTrue(a[0].right_change)
        self.assertEqual(n.points[a[0].right_id].key.lane,-2)
        self.assertEqual(a[-1].next_id,-1)

    def test_vtd_screenshot_road211_position_width_and_offset_match(self):
        source=ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr'
        root=ET.parse(source).getroot()
        r=Road(root.find("road[@id='211']"))
        s,t=15.307,1.197
        x,y=r.at_offset(s,t)
        self.assertAlmostEqual(x,375.657,delta=.0005)
        self.assertAlmostEqual(y,-260.763,delta=.0005)
        self.assertTrue(r.starts[2]<=s<r.end(2))
        self.assertEqual(r.lanes[2][2].get('type'),'driving')
        self.assertAlmostEqual(r.widths[2][2].jet(s)[0],3.200,places=6)
        self.assertAlmostEqual(t-r.offset_jet(2,2,s)[0],-.403,places=6)
        self.assertEqual(r.lanes[2][1].get('type'),'none')
        # The isolated branch is 211:1:2 -> 211:0:1. Junction12 only
        # declares from=2 (and nonexistent boundary lanes 3/4), not from=1.
        outbound=[l.get('from') for c in root.find("junction[@id='12']").findall('connection')
                  if c.get('incomingRoad')=='211' for l in c.findall('laneLink')]
        self.assertIn('2',outbound)
        self.assertNotIn('1',outbound)

    def test_taper_tangent_uses_width_derivative(self):
        n=self.make_network(road(sections=section(0,lane(-1,b=-.04))))
        p=n.lanes[LaneKey(1,0,-1)].points[20]
        self.assertAlmostEqual(p.y,-(3.5-.04*p.road_s)/2,places=8)
        self.assertAlmostEqual(p.heading,math.atan(.02),places=8)
        self.assertAlmostEqual(p.curvature,0.,places=8)

    def test_offset_lane_curvature_matches_analytic_arc(self):
        n=self.make_network(road(length=20,sections=section(0,lane(-1)),shape='<arc curvature=".02"/>'))
        p=n.points[10]
        self.assertAlmostEqual(p.curvature,.02/(1+.02*1.75),places=8)

    def test_poly3_is_parameterized_by_arc_length(self):
        g=Geometry(ET.fromstring('<geometry s="0" x="0" y="0" hdg="0" length="30"><poly3 a="0" b="0" c=".03" d=".001"/></geometry>'))
        x,y,h,k,_=g.pose(20)
        measured=quad(lambda u: math.hypot(1,.06*u+.003*u*u),0,x,epsabs=1e-10)[0]
        self.assertAlmostEqual(measured,20.,places=7)
        self.assertAlmostEqual(y,.03*x*x+.001*x*x*x,places=8)
        self.assertLess(x,20)

    def test_spiral_position_matches_independent_quadrature(self):
        g=Geometry(ET.fromstring('<geometry s="0" x="3" y="4" hdg=".2" length="30"><spiral curvStart=".01" curvEnd=".04"/></geometry>'))
        x,y,h,k,_=g.pose(25)
        ex=3+quad(lambda s:math.cos(.2+.01*s+.0005*s*s),0,25,epsabs=1e-10)[0]
        ey=4+quad(lambda s:math.sin(.2+.01*s+.0005*s*s),0,25,epsabs=1e-10)[0]
        self.assertAlmostEqual(x,ex,places=7)
        self.assertAlmostEqual(y,ey,places=7)

    def test_no_nearest_lane_or_implicit_same_id_connection(self):
        sections=section(0,lane(-1))+section(20,lane(-1))
        n=self.make_network(road(sections=sections))
        self.assertEqual(n.connections,[])
        self.assertEqual(n.lanes[LaneKey(1,0,-1)].points[-1].next_id,-1)

    def test_declared_successor_gap_has_separate_shared_anchor_candidate(self):
        sections=section(0,lane(-1,link='<successor id="-1"/>'))+section(20,lane(-1,a=10))
        n=self.make_network(road(sections=sections))
        self.assertEqual(len(n.connections),1)
        self.assertEqual(n.connections[0].status,'position_gap')
        self.assertAlmostEqual(n.connections[0].gap,3.25)
        self.assertEqual(n.lanes[LaneKey(1,0,-1)].points[-1].next_id,-1)
        self.assertEqual(len(n.transitions),1)
        t=n.transitions[0]
        self.assertEqual(t.kind,'continuation')
        self.assertEqual(n.gap_diagnostics[0]['status'],'generated')
        self.assertEqual(n.points[t.from_point].key,LaneKey(1,0,-1))
        self.assertEqual(n.points[t.to_point].key,LaneKey(1,1,-1))
        for p,pid in ((t.points[0],t.from_point),(t.points[-1],t.to_point)):
            q=n.points[pid]
            self.assertAlmostEqual(p.x,q.x,places=7)
            self.assertAlmostEqual(p.y,q.y,places=7)
            self.assertAlmostEqual(p.heading,q.heading,places=7)
            self.assertAlmostEqual(p.curvature,q.curvature,places=7)
            self.assertNotIn(pid,n.replaced_points)

    def test_world_candidate_rejects_opposing_and_reverse_anchors(self):
        from lane_transitions import RoadGuard
        from reviewed_repairs import world_candidate
        n=self.make_network(road())
        track=n.lanes[LaneKey(1,0,-1)].points
        guard=RoadGuard(n.roads[1],1,n.vehicle_width,defer_marks=True)
        result,reason=world_candidate(n,n.roads[1],track[-10],track[10],guard)
        self.assertIsNone(result)
        self.assertEqual(reason,'wrong_travel_direction')
        other=n.lanes[LaneKey(1,0,1)].points[10]
        result,reason=world_candidate(n,n.roads[1],track[10],other,guard)
        self.assertIsNone(result)
        self.assertEqual(reason,'wrong_travel_direction')

    def test_transition_planning_flags_export_and_cpp_validation(self):
        n=self.lifecycle_network(mark='solid')
        with tempfile.TemporaryDirectory() as td:
            out=Path(td);export(n,out)
            with (out/'transition_checks.csv').open(newline='') as f:
                records=list(csv.DictReader(f))
            self.assertEqual(records[0]['crosses_solid'],'1')
            self.assertEqual(records[0]['requires_planner_validation'],'1')
            self.assertEqual(records[0]['vehicle_footprint_valid'],'1')
            binary=ROOT/'build/check_precomputed_network'
            if not binary.is_file():self.skipTest('build check_precomputed_network first')
            result=subprocess.run([str(binary),str(out)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            records[0]['requires_planner_validation']='0'
            with (out/'transition_checks.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=records[0].keys());writer.writeheader();writer.writerows(records)
            result=subprocess.run([str(binary),str(out)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('invalid transition planning flags',result.stderr)

    def test_junction_only_explicit_from_to_lanes(self):
        incoming=road(1,length=10,links='<successor elementType="junction" elementId="10"/>')
        connecting=road(2,length=10,x=10,sections=section(0,lane(-1)+lane(-2)))
        junction='<junction id="10"><connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start"><laneLink from="-2" to="-2"/></connection></junction>'
        n=self.make_network(incoming+connecting,junction)
        self.assertEqual(len(n.connections),1)
        self.assertEqual(n.connections[0].source,LaneKey(1,0,-2))
        self.assertEqual(n.connections[0].target,LaneKey(2,0,-2))
        self.assertEqual(n.connections[0].status,'continuous')

    def test_solid_mark_never_gives_lateral_permission(self):
        n=self.make_network(road(sections=section(0,lane(-1,mark='solid',change='both')+lane(-2))))
        self.assertTrue(all(not p.right_change for p in n.lanes[LaneKey(1,0,-1)].points))

    def test_opposing_pavement_arrow_is_reported_without_reversing_lane(self):
        xml=road().replace('</road>', '<objects><object id="7" name="RM_537_LT.flt" s="10" t="1.75" hdg="0"/></objects></road>')
        n=self.make_network(xml)
        issues=[i for i in n.issues if i['kind']=='pavement_lane_direction_conflict']
        self.assertEqual(len(issues),1)
        self.assertEqual(issues[0]['lane'],'1:0:1')
        self.assertEqual(issues[0]['object_id'],'7')
        p=n.lanes[LaneKey(1,0,1)].points[0]
        self.assertAlmostEqual(abs(p.heading),math.pi)

    def test_source_heading_jump_is_not_silently_connected(self):
        xml=road(sections=section(0,lane(-1)))
        xml=xml.replace('<width sOffset="0" a="3.5" b="0.0" c="0.0" d="0.0"/>',
                        '<width sOffset="0" a="3.5" b="0" c="0" d="0"/><width sOffset="20" a="3.5" b="1" c="0" d="0"/>')
        n=self.make_network(xml)
        self.assertTrue(any(i['kind']=='source_heading_jump' for i in n.issues))
        self.assertGreater(sum(p.next_id==-1 for p in n.points),1)

    def lifecycle_network(self, positive=False, mark='broken', change='both', birth=False):
        sign = 1 if positive else -1
        through0 = lane(sign, link=f'<successor id="{sign}"/>', mark=mark, change=change)
        through1 = lane(sign, link=f'<predecessor id="{sign}"/>', mark=mark, change=change)
        def side(s, content):
            return section(s, '' if positive else content, content if positive else '')
        if birth:
            through1 = lane(sign, link=f'<predecessor id="{sign}"/><successor id="{sign}"/>', mark=mark, change=change)
            sections = (side(0, through0) +
                        side(40, through1 + lane(2*sign, a=0, b=.175, link=f'<successor id="{2*sign}"/>')) +
                        side(60, lane(sign, link=f'<predecessor id="{sign}"/>', mark=mark, change=change) +
                             lane(2*sign, link=f'<predecessor id="{2*sign}"/>')))
        else:
            sections = side(0, through0 + lane(2*sign)) + side(40, through1)
        return self.make_network(road(length=100, sections=sections))

    def assert_shared_smooth_anchors(self, n, t):
        for p, pid in ((t.points[0], t.from_point), (t.points[-1], t.to_point)):
            q = n.points[pid]
            self.assertAlmostEqual(p.x, q.x, places=7)
            self.assertAlmostEqual(p.y, q.y, places=7)
            self.assertAlmostEqual(math.remainder(p.heading-q.heading,2*math.pi),0.,places=7)
            self.assertAlmostEqual(p.curvature, q.curvature, places=7)
            self.assertNotIn(pid, n.replaced_points)
        direction = -1 if t.event_lane.lane > 0 else 1
        for a, b in zip(t.points, t.points[1:]):
            self.assertGreater(direction*(b.road_s-a.road_s), 0)
            self.assertLessEqual(math.hypot(b.x-a.x, b.y-a.y), .510001)
            self.assertGreater(b.s, a.s)
        self.assertLessEqual(t.max_curvature, n.max_curvature)
        self.assertGreaterEqual(t.min_clearance, .05)

    def test_disappearing_lane_merges_before_dead_end_with_shared_anchors(self):
        n = self.lifecycle_network()
        self.assertEqual(len(n.transitions), 1)
        t = n.transitions[0]
        self.assertEqual(t.kind, 'merge')
        self.assertEqual(n.points[t.from_point].key.lane, -2)
        self.assertEqual(n.points[t.to_point].key.lane, -1)
        self.assertLess(t.points[-1].road_s, 40)
        self.assertIn(n.lanes[LaneKey(1, 0, -2)].points[-1].id, n.replaced_points)
        self.assert_shared_smooth_anchors(n, t)
        # No fabricated source laneLink: the lateral maneuver is a separate edge.
        self.assertTrue(all(e.source.lane == -1 and e.target.lane == -1 for e in n.connections))

    def test_zero_width_birth_branches_from_through_lane_across_sections(self):
        n = self.lifecycle_network(birth=True)
        self.assertEqual(len(n.transitions), 1)
        t = n.transitions[0]
        self.assertEqual(t.kind, 'split')
        self.assertEqual(n.points[t.from_point].key.lane, -1)
        self.assertEqual(n.points[t.to_point].key.lane, -2)
        self.assertGreaterEqual(n.points[t.to_point].width, n.vehicle_width)
        self.assertEqual(n.points[t.to_point].key.section, 2)
        self.assertIn(n.lanes[LaneKey(1, 1, -2)].points[0].id, n.replaced_points)
        self.assert_shared_smooth_anchors(n, t)

    def test_positive_lane_lifecycle_follows_decreasing_road_s(self):
        for birth, kind in ((False, 'split'), (True, 'merge')):
            with self.subTest(kind=kind):
                n = self.lifecycle_network(positive=True, birth=birth)
                self.assertEqual(len(n.transitions), 1)
                self.assertEqual(n.transitions[0].kind, kind)
                self.assert_shared_smooth_anchors(n, n.transitions[0])
                if kind=='split':
                    self.assertEqual(n.transitions[0].shape_method,'advanced_split_C2')
                    self.assertGreater(n.transitions[0].points[0].road_s,40.)

    def test_taper_guide_is_boundary_parallel_and_local_to_actual_widening(self):
        n = self.lifecycle_network(birth=True)
        t = n.transitions[0]
        self.assertEqual(t.shape_method, 'boundary_parallel_taper')
        self.assertLess(t.points[0].road_s, 40)
        self.assertLess(t.points[-1].road_s, 70)
        middle = [p for p in t.points if 46 < p.road_s < 53]
        self.assertTrue(middle)
        for p in middle:
            # Outer border t=-(3.5 + .175*(s-40)); constant 1.75m inward
            # offset gives precisely the border's slope, not a global S-curve.
            self.assertAlmostEqual(p.y, -1.75-.175*(p.road_s-40), places=7)
            self.assertAlmostEqual(p.heading, math.atan(-.175), places=7)
            self.assertAlmostEqual(p.curvature, 0, places=7)
        self.assert_shared_smooth_anchors(n, t)

    def test_lane_without_junction_exit_is_visible_but_topology_flagged(self):
        sections = (section(0,lane(-1,link='<successor id="-1"/>')) +
                    section(40,lane(-1,link='<predecessor id="-1"/><successor id="-1"/>')+lane(-2)))
        first = road(1,length=100,sections=sections,
                     links='<successor elementType="junction" elementId="10"/>')
        second = road(2,length=40,x=100,sections=section(0,lane(-1)))
        junction = '<junction id="10"><connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start"><laneLink from="-1" to="-1"/></connection></junction>'
        n = self.make_network(first+second,junction)
        key = LaneKey(1,1,-2)
        self.assertTrue(n.lane_access[key]['has_entry'])  # generated split exists
        self.assertFalse(n.lane_access[key]['has_exit']) # no arbitrary roundabout link
        self.assertIn(key,n.topology_unconnected_lanes)
        self.assertNotIn(key,n.excluded_lanes)
        drawing = Drawing(n)
        root = ET.fromstring(drawing.svg())
        paths = next(e for e in root.iter() if e.get('id')=='paths')
        self.assertIn('1:1:-2',ET.tostring(paths,encoding='unicode'))
        inspected = drawing.svg(inspect=True)
        self.assertIn('no_exit 1:1:-2',inspected)

    def test_cpp_rejects_inconsistent_topological_access_flags(self):
        binary=ROOT/'build/check_precomputed_network'
        if not binary.is_file():
            self.skipTest('build check_precomputed_network first')
        n=self.lifecycle_network()
        with tempfile.TemporaryDirectory() as td:
            out=Path(td);export(n,out)
            with (out/'lane_access.csv').open(newline='') as f:
                rows=list(csv.reader(f))
            rows[1][5]='no_exit'
            with (out/'lane_access.csv').open('w',newline='') as f:
                csv.writer(f).writerows(rows)
            result=subprocess.run([str(binary),str(out)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('lane access status mismatch',result.stderr)

    def test_geometric_merge_retains_solid_and_permission_flags_for_planning(self):
        for mark, change, reason in (('solid', 'both', 'solid_boundary'),
                                     ('broken', 'none', 'roadmark_lane_change_forbidden')):
            with self.subTest(mark=mark):
                n = self.lifecycle_network(mark=mark, change=change)
                self.assertEqual(len(n.transitions),1)
                self.assertTrue(n.replaced_points)
                self.assertEqual(n.transition_diagnostics[0]['status'], 'generated')
                self.assertIn(reason,n.transitions[0].roadmark_conflicts)

    def test_renderer_draws_real_transition_samples_and_omits_replaced_tail(self):
        n = self.lifecycle_network()
        drawing = Drawing(n)
        root = ET.fromstring(drawing.svg())
        paths = next(e for e in root.iter() if e.get('id') == 'paths')
        rendered = ET.tostring(paths, encoding='unicode')
        t = n.transitions[0]
        p = t.points[len(t.points)//2]
        self.assertIn(f'{p.x:.3f},{-p.y:.3f}', rendered)
        hidden = n.lanes[LaneKey(1, 0, -2)].points[-1]
        self.assertNotIn(f'{hidden.x:.3f},{-hidden.y:.3f}', rendered)

    def test_cpp_transition_round_trip_and_endpoint_rejection(self):
        binary = ROOT/'build/check_precomputed_network'
        if not binary.is_file():
            self.skipTest('build check_precomputed_network first')
        n = self.lifecycle_network()
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            export(n, out)
            result = subprocess.run([str(binary), str(out)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('1 merge/split/continuation transitions', result.stdout)
            with (out/'transition_points.csv').open(newline='') as f:
                rows = list(csv.reader(f))
            rows[1][4] = str(float(rows[1][4]) + .1)
            with (out/'transition_points.csv').open('w', newline='') as f:
                csv.writer(f).writerows(rows)
            result = subprocess.run([str(binary), str(out)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('endpoint mismatch', result.stderr)

    def test_cpp_round_trip_and_bad_reference_rejection(self):
        binary=ROOT/'build/check_precomputed_network'
        if not binary.is_file():
            self.skipTest('build check_precomputed_network first')
        n=self.make_network(road())
        with tempfile.TemporaryDirectory() as td:
            out=Path(td)
            export(n,out)
            result=subprocess.run([str(binary),str(out)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            with (out/'lane_points.csv').open(newline='') as f:
                rows=list(csv.reader(f))
            rows[1][12]='99999999'
            with (out/'lane_points.csv').open('w',newline='') as f:
                csv.writer(f).writerows(rows)
            result=subprocess.run([str(binary),str(out)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('dangling',result.stderr)

    def test_renderer_keeps_sample_vertices_and_both_base_layers(self):
        n=self.make_network(road(length=8))
        drawing=Drawing(n)
        base=drawing.svg(overlay=False)
        overlay=drawing.svg()
        self.assertNotIn('stroke="#ffff00"',base)
        self.assertIn('stroke="#ffff00"',overlay)
        self.assertIn('id="markings"',base)
        p=n.points[5]
        self.assertIn(f'{p.x:.3f},{-p.y:.3f}',overlay)
        ET.fromstring(overlay)


if __name__=='__main__':
    unittest.main()
