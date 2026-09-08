import unittest
import numpy as np
from whole_mission_corridors import Network


def lane(road,x0,x1,y=0,lane_id=-1,junction=-1,y1=None):
    y1=y if y1 is None else y1
    count=max(2,int(abs(x1-x0)*2)+1)
    x=np.linspace(x0,x1,count);ys=np.linspace(y,y1,count)
    yaw=np.full(count,np.arctan2(y1-y,x1-x0));s=np.linspace(0,np.hypot(x1-x0,y1-y),count)
    width=np.full(count,3.)
    points=np.column_stack([x,ys,yaw,s,width,x-np.sin(yaw)*1.5,ys+np.cos(yaw)*1.5,x+np.sin(yaw)*1.5,ys-np.cos(yaw)*1.5])
    return (road,0,lane_id),dict(junction=junction,points=points)


def mission(coords,name='test'):
    return dict(scenario=name,checkpoints=[dict(seq=i+1,x=x,y=y,lanes=[[999,999,999]]) for i,(x,y) in enumerate(coords)])


class WholeMissionTests(unittest.TestCase):
    def parallel(self):
        geo=dict([lane(1,0,10),lane(1,0,10,3,-2),lane(10,10,20,junction=1),
                  lane(11,10,20,3,junction=1),lane(2,20,30),lane(2,20,30,3,-2)])
        edges=[((1,0,-1),(10,0,-1)),((10,0,-1),(2,0,-1)),
               ((1,0,-2),(11,0,-1)),((11,0,-1),(2,0,-2))]
        return geo,edges

    def test_all_parallel_gateways_not_cp_lane(self):
        geo,edges=self.parallel()
        d=Network(geo,edges).build(mission([(1,3),(9,3),(21,3),(29,3)]))
        self.assertTrue(d['initialized_ok'])
        self.assertEqual(len(d['legs'][0]['entry_edges']),2)
        self.assertEqual(len(d['legs'][0]['exit_edges']),2)
        self.assertNotIn((999,999,999),d['witness_route'])

    def test_start_intermediate_and_finish_junctions_all_included(self):
        geo={};edges=[];last=None
        for i in range(6):
            k,g=lane(i+1,-20+i*20,-10+i*20);geo[k]=g
            if last:edges.append((last,k))
            if i<5:
                j,h=lane(100+i,-10+i*20,i*20,junction=i+1);geo[j]=h;edges.append((k,j));last=j
        d=Network(geo,edges).build(mission([(-19,0),(9,0),(21,0),(49,0),(61,0),(89,0)]))
        self.assertTrue(d['initialized_ok'])
        self.assertEqual(len(d['legs']),5)
        self.assertEqual(len(d['intervals']),5)
        self.assertEqual(d['witness_route'][0][0],1)
        self.assertEqual(d['witness_route'][-1][0],6)

    def test_junction_exit_lift_cannot_select_incoming_corridor(self):
        geo=dict([lane(1,0,10),lane(10,10,20,junction=1,y1=10),lane(2,20,30,10)])
        edges=[((1,0,-1),(10,0,-1)),((10,0,-1),(2,0,-1))]
        d=Network(geo,edges).build(mission([(1,0),(9,0),(11,1),(29,10)]))
        self.assertTrue(d['initialized_ok'])
        self.assertEqual(d['checkpoint_matches'][2]['component'],(2,0,-1))
        self.assertEqual(d['checkpoint_matches'][2]['association'],'junction_exit')

    def test_next_gateway_prunes_dead_end_alternative(self):
        geo,edges=self.parallel()
        geo.update(dict([lane(12,30,40,junction=2),lane(3,40,50)]))
        edges.extend([((2,0,-1),(12,0,-1)),((12,0,-1),(3,0,-1))])
        d=Network(geo,edges).build(mission([(1,3),(9,3),(21,3),(29,3),(41,0),(49,0)]))
        self.assertTrue(d['initialized_ok'])
        self.assertEqual(len(d['legs'][0]['entry_edges']),1)
        self.assertIn((2,0,-2),d['corridor_reachability'][1]['not_reachable'])

    def test_unreachable_input_is_failure_not_fake_connection(self):
        geo=dict([lane(1,0,10),lane(2,500,510)])
        d=Network(geo,[]).build(mission([(1,0),(509,0)]))
        self.assertFalse(d['initialized_ok'])
        self.assertEqual(d['failure']['to_seq'],2)

    def test_road_id_boundary_within_one_corridor_is_visited(self):
        geo=dict([lane(1,0,10),lane(2,10,20)])
        d=Network(geo,[((1,0,-1),(2,0,-1))]).build(mission([(1,0),(19,0)]))
        self.assertEqual(d['witness_route'],[(1,0,-1),(2,0,-1)])
        self.assertEqual(d['legs'][0]['status'],'same_corridor')

    def test_invalid_pair_count_is_not_competition_valid(self):
        geo,edges=self.parallel()
        d=Network(geo,edges).build(mission([(1,0),(9,0),(29,0)]))
        self.assertTrue(d['initialized_ok'])
        self.assertFalse(d['validation']['checkpoint_rules_valid'])

    def test_scenario_name_does_not_change_planning(self):
        geo,edges=self.parallel();net=Network(geo,edges);coords=[(1,0),(9,0),(21,0),(29,0)]
        a=net.build(mission(coords,'M09'));b=net.build(mission(coords,'unseen'))
        a.pop('scenario');b.pop('scenario');self.assertEqual(a,b)

    def test_nonfinite_and_missing_sequence_rejected(self):
        geo,edges=self.parallel();net=Network(geo,edges)
        with self.assertRaises(ValueError):net.build(mission([(float('nan'),0),(2,0)]))
        d=mission([(1,0),(29,0)]);d['checkpoints'][1]['seq']=3
        with self.assertRaises(ValueError):net.build(d)


if __name__=='__main__':unittest.main()
