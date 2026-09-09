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


    def taper_network(self, opening=False, count=1, positive=False, gap=0.):
        through=(10,0,-3)
        geo=dict([lane(1,0,10),lane(10,10,20,lane_id=-3,junction=1),lane(2,20,30)])
        edges=[((1,0,-1),through),(through,(2,0,-1))]
        for i in range(count):
            key,g=lane(10,10,20,lane_id=-2+i,junction=1)
            p=g['points'];width=np.linspace(0,3,len(p)) if opening else np.linspace(3,0,len(p))
            inner=1.5+i*width+gap
            p[:,1]=inner+width*.5;p[:,4]=width
            p[:,5:7]=np.column_stack([p[:,0],inner])
            p[:,7:9]=np.column_stack([p[:,0],inner+width])
            geo[key]=g
            normal,ng=lane(2,20,30,3*(i+1),lane_id=-2-i) if opening else lane(1,0,10,3*(i+1),lane_id=-2-i)
            geo[normal]=ng;edges.append((key,normal) if opening else (normal,key))
        if positive:
            rename=lambda k:(k[0],k[1],-k[2])
            geo={rename(k):dict(g,points=g['points'].copy()) for k,g in geo.items()}
            for g in geo.values():g['points'][:,3]=g['points'][-1,3]-g['points'][:,3]
            edges=[(rename(a),rename(b)) for a,b in edges]
        return geo,edges

    def test_merging_taper_is_surface_without_fabricated_route(self):
        geo,edges=self.taper_network()
        net=Network(geo,edges);d=net.build(mission([(1,0),(9,0),(21,0),(29,0)]))
        gate=d['legs'][0];key=(10,0,-2)
        self.assertIn(key,gate['surface_lanes'])
        self.assertIn(key,gate['allowed'])
        self.assertNotIn(key,gate['connection_lanes'])
        self.assertEqual(gate['surface_extensions'][0]['reason'],'merging_taper')
        self.assertEqual(len(gate['surface_entry_edges']),2)
        self.assertTrue(d['validation']['surface_interfaces_connected'])
        self.assertEqual({(a,b) for a,bs in net.forward.items() for b in bs},set(edges))

    def test_multiple_opening_tapers_follow_adjacent_boundaries(self):
        geo,edges=self.taper_network(opening=True,count=2)
        d=Network(geo,edges).build(mission([(1,0),(9,0),(21,0),(29,0)]))
        gate=d['legs'][0]
        self.assertEqual(set(gate['surface_lanes']),{(10,0,-3),(10,0,-2),(10,0,-1)})
        self.assertEqual(len(gate['surface_exit_edges']),3)
        self.assertTrue(d['validation']['surface_interfaces_connected'])

    def test_positive_lane_decreasing_station_taper(self):
        geo,edges=self.taper_network(positive=True)
        d=Network(geo,edges).build(mission([(1,0),(9,0),(21,0),(29,0)]))
        self.assertIn((10,0,2),d['legs'][0]['surface_lanes'])

    def test_surface_does_not_bridge_physical_gap_or_other_road(self):
        for gap in (0.,.5):
            geo,edges=self.taper_network(gap=gap)
            if not gap:
                geo[(11,0,-2)]=geo.pop((10,0,-2))
                edges=[(a,(11,0,-2) if b==(10,0,-2) else b) for a,b in edges]
            d=Network(geo,edges).build(mission([(1,0),(9,0),(21,0),(29,0)]))
            self.assertEqual(d['legs'][0]['surface_lanes'],[(10,0,-3)])

    def test_full_width_dead_end_is_not_taper_surface(self):
        geo,edges=self.parallel()
        geo.update(dict([lane(12,30,40,junction=2),lane(3,40,50)]))
        edges.extend([((2,0,-1),(12,0,-1)),((12,0,-1),(3,0,-1))])
        d=Network(geo,edges).build(mission([(1,0),(9,0),(21,0),(29,0),(41,0),(49,0)]))
        self.assertNotIn((11,0,-1),d['legs'][0]['surface_lanes'])

    def test_renderer_uses_taper_surface_and_expanded_interface(self):
        import xml.etree.ElementTree as ET
        from render_mission_corridors import draw
        geo,edges=self.taper_network()
        d=Network(geo,edges).build(mission([(1,0),(9,0),(21,0),(29,0)]))
        svg=ET.fromstring(draw(d,geo,[],[0]))
        ns='{http://www.w3.org/2000/svg}'
        layer=svg.find(ns+"g[@id='gateways']")
        self.assertTrue(any('10:0:-2' in (v.text or '') for v in layer.iter(ns+'title')))
        self.assertEqual(len(layer.findall(ns+'polyline')),3)


if __name__=='__main__':unittest.main()
