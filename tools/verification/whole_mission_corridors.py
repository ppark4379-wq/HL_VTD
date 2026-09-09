"""Whole-mission XODR corridor/gateway precomputation, independent of route.csv.

Checkpoint coordinates identify road corridors, not mandatory points or lanes.
A staged directed lane search preserves connectivity over ALL checkpoint gaps.
"""
from collections import defaultdict
from heapq import heappop, heappush
import math
import numpy as np


def segment_distance(a,b,c,d):
    """Distance between two lane boundary cross-sections (not center points)."""
    a,b,c,d=map(np.asarray,(a,b,c,d));u=b-a;v=d-c
    cross=lambda x,y:float(x[0]*y[1]-x[1]*y[0])
    den=cross(u,v)
    if abs(den)>1e-12:
        t=cross(c-a,v)/den;s=cross(c-a,u)/den
        if 0<=t<=1 and 0<=s<=1:return 0.
    def point(p,x,y):
        delta=y-x;length=float(delta@delta)
        t=np.clip(float((p-x)@delta)/length,0.,1.) if length>1e-20 else 0.
        return float(np.linalg.norm(p-(x+t*delta)))
    return min(point(a,c,d),point(b,c,d),point(c,a,b),point(d,a,b))


class Network:
    def __init__(self, geometry, longitudinal_edges, full_edges=None):
        self.geometry=geometry
        self.forward,self.reverse=defaultdict(set),defaultdict(set)
        self.parent={k:k for k,g in geometry.items() if g['junction']<0}
        groups=defaultdict(list)
        for k in self.parent: groups[k[0],k[1],k[2]>0].append(k)
        for keys in groups.values():
            for k in keys[1:]: self.union(keys[0],k)
        for a,b in longitudinal_edges:
            a,b=tuple(a),tuple(b)
            self.forward[a].add(b);self.reverse[b].add(a)
            if a in self.parent and b in self.parent: self.union(a,b)
        self.components=defaultdict(set)
        for k in self.parent: self.components[self.find(k)].add(k)
        self.owner={k:c for c,keys in self.components.items() for k in keys}
        self.length={k:float(np.linalg.norm(np.diff(np.asarray(g['points'])[:,:2],axis=0),axis=1).sum()) for k,g in geometry.items()}
        self.search_graph=defaultdict(list);self.lateral=set()
        for a,b in (full_edges if full_edges is not None else longitudinal_edges):
            a,b=tuple(a),tuple(b)
            if a[:2]==b[:2]:
                if a not in self.owner or b not in self.owner: continue
                cost=22.+4.*abs(a[2]-b[2]);self.lateral.add((a,b))
            else: cost=max(.01,(self.length[a]+self.length[b])*.5)
            self.search_graph[a].append((b,cost))
        for edges in self.search_graph.values(): edges.sort()

    def find(self,k):
        while self.parent[k]!=k:
            self.parent[k]=self.parent[self.parent[k]];k=self.parent[k]
        return k

    def union(self,a,b):
        a,b=self.find(a),self.find(b)
        if a!=b:self.parent[max(a,b)]=min(a,b)

    def candidates(self,cp,heading,role='road',max_distance=30.):
        candidates={};xy=np.array([cp['x'],cp['y']]);junction_distances={}
        for k,g in self.geometry.items():
            p=np.asarray(self.geometry[k]['points']);diff=xy-p[:,:2]
            along=diff[:,0]*np.cos(p[:,2])+diff[:,1]*np.sin(p[:,2])
            across=-diff[:,0]*np.sin(p[:,2])+diff[:,1]*np.cos(p[:,2])
            surface=along**2+np.maximum(np.abs(across)-p[:,4]*.5,0.)**2
            i=int(np.argmin(surface));distance=math.sqrt(float(surface[i]))
            if g['junction']>=0:
                j=g['junction'];junction_distances[j]=min(junction_distances.get(j,float('inf')),distance)
                continue
            c=self.owner[k]
            angle=math.atan2(math.sin(p[i,2]-heading),math.cos(p[i,2]-heading))
            score=25.*distance**2+30.*(1.-math.cos(angle))
            value=dict(component=c,distance_m=distance,heading_error_rad=angle,score=score,
                       nearest_lane=k,road_s=float(p[i,3]))
            if c not in candidates or (score,k)<(candidates[c]['score'],candidates[c]['nearest_lane']):candidates[c]=value
        best_road=min((v['distance_m'] for v in candidates.values()),default=float('inf'))
        best_junction=min(junction_distances.values(),default=float('inf'))
        if role in ('entry','exit') and best_junction<best_road and best_junction<=max_distance:
            junction_ids={j for j,d in junction_distances.items() if d<=best_junction+1.}
            graph=self.reverse if role=='entry' else self.forward
            pending=[k for k,g in self.geometry.items() if g['junction'] in junction_ids]
            seen=set();adjacent=set()
            while pending:
                k=pending.pop()
                if k in seen:continue
                seen.add(k)
                if k in self.owner:adjacent.add(self.owner[k])
                else:pending.extend(graph[k]-seen)
            # Use the junction area, NOT the particular turn lane under the CP.
            # Directional lifting prevents an exit point being assigned back to
            # the incoming corridor, which otherwise causes a whole-block loop.
            return {c:dict(v,score=25.*best_junction**2+v['distance_m']+30.*(1.-math.cos(v['heading_error_rad'])),
                           association='junction_'+role,junction_ids=sorted(junction_ids)) for c,v in candidates.items() if c in adjacent}
        # Never choose a distant reachable road merely to manufacture success.
        return {c:dict(v,association='road_surface') for c,v in candidates.items() if v['distance_m']<=max_distance}

    def advance(self,current,candidates,require_junction):
        dist={};previous={};queue=[]
        for k,record in current.items():
            state=(k,False);dist[state]=record['cost'];previous[state]=None
            heappush(queue,(record['cost'],state))
        while queue:
            cost,state=heappop(queue)
            if cost!=dist[state]:continue
            k,crossed=state
            for b,step in self.search_graph[k]:
                flag=crossed or self.geometry[k]['junction']>=0 or self.geometry[b]['junction']>=0
                nxt=(b,flag);value=cost+step
                if value>=dist.get(nxt,float('inf')):continue
                dist[nxt]=value;previous[nxt]=state;heappush(queue,(value,nxt))
        result={}
        for c,match in candidates.items():
            for k in self.components[c]:
                # All lanes are candidates, but a CP's road/section must actually
                # be visited; membership in a large component alone can skip it.
                if k[:2]!=match['nearest_lane'][:2]:continue
                possible=[(dist[(k,flag)],(k,flag)) for flag in (False,True)
                          if (not require_junction or flag) and (k,flag) in dist]
                if not possible:continue
                cost,state=min(possible);chain=[]
                while previous[state] is not None:
                    chain.append(state[0]);state=previous[state]
                seed=state[0];route=[seed]+list(reversed(chain));history=current[seed]
                result[k]=dict(cost=cost+match['score'],route=history['route']+route[1:],
                    matches=history['matches']+[match],
                    spans=history['spans']+[(len(history['route'])-1,len(history['route'])+len(route)-2)])
        return result

    def gateway(self,source_component,target_component):
        source=self.components[source_component];target=self.components[target_component]
        starts={(a,b) for a in source for b in self.forward[a]
                if b not in source and (b not in self.owner or b in target)}
        edges=set(starts);seen=set();pending=[b for a,b in starts]
        while pending:
            k=pending.pop()
            if k in seen:continue
            seen.add(k)
            if k in target:continue
            for b in self.forward[k]:
                if b not in self.owner or b in target:edges.add((k,b));pending.append(b)
        back=defaultdict(set)
        for a,b in edges:back[b].add(a)
        viable=set();pending=list(seen&target)
        while pending:
            k=pending.pop()
            if k in viable:continue
            viable.add(k);pending.extend(back[k]-viable)
        valid=sorted((a,b) for a,b in edges if a in viable and b in viable)
        entry=sorted((a,b) for a,b in valid if a in source and b not in source)
        exit_=sorted((a,b) for a,b in valid if b in target and a not in target)
        return dict(connection_lanes=sorted(viable-set(self.owner)),connection_edges=valid,
            entry_edges=entry,exit_edges=exit_,source_component=source_component,target_component=target_component,
            status='connected' if entry and exit_ else 'UNRESOLVED')


    def gateway_surface(self, gate):
        """Include adjoining XODR lane tapers without inventing routing edges.

        A lane born/ending at zero width has only one longitudinal interface.
        It belongs to the gateway surface even though it cannot independently
        provide an entry-to-exit route. Keep that distinction in the artifact.
        """
        core=set(gate['connection_lanes']);surface=set(core)
        source=self.components[gate['source_component']]
        target=self.components[gate['target_component']]
        sections={(k[0],k[1],k[2]>0) for k in core}
        candidates={}
        for k,g in self.geometry.items():
            if k in core or (k[0],k[1],k[2]>0) not in sections:continue
            p=np.asarray(g['points'])
            if np.max(p[:,4])<=.01:continue
            entry=sorted((a,k) for a in self.reverse[k]&source)
            exit_=sorted((k,b) for b in self.forward[k]&target)
            if p[-1,4]<=.01 and entry:
                candidates[k]=('merging_taper',entry+exit_)
            elif p[0,4]<=.01 and exit_:
                candidates[k]=('opening_taper',entry+exit_)
        added=[]
        while candidates:
            progress=False
            for k,(reason,edges) in list(sorted(candidates.items())):
                p=np.asarray(self.geometry[k]['points'])
                for neighbor in sorted(surface):
                    if k[:2]!=neighbor[:2] or k[2]*neighbor[2]<=0 or abs(k[2]-neighbor[2])!=1:continue
                    q=np.asarray(self.geometry[neighbor]['points'])
                    # Compare the shared boundary over the entire section,
                    # including curved tapers and positive-lane travel order.
                    order=np.argsort(q[:,3]);stations=q[order,3]
                    if p[:,3].min()<stations[0]-1e-6 or p[:,3].max()>stations[-1]+1e-6:continue
                    distance=min(float(np.max(np.linalg.norm(p[:,a:a+2]-np.column_stack([
                        np.interp(p[:,3],stations,q[order,b+j]) for j in (0,1)]),axis=1)))
                        for a in (5,7) for b in (5,7))
                    if distance>.1:continue
                    surface.add(k);del candidates[k];progress=True
                    added.append(dict(key=k,reason=reason,adjacent_lane=neighbor,
                                      boundary_gap_m=distance,interface_edges=edges))
                    break
            if not progress:break
        gate['surface_lanes']=sorted(surface)
        gate['surface_extensions']=added
        gate['surface_entry_edges']=sorted(set(gate['entry_edges'])|
            {(a,k) for k in surface-core for a in self.reverse[k]&source})
        gate['surface_exit_edges']=sorted(set(gate['exit_edges'])|
            {(k,b) for k in surface-core for b in self.forward[k]&target})
        gate['allowed']=sorted(set(gate['allowed'])|surface)

    def build(self,mission):
        cps=sorted(mission['checkpoints'],key=lambda p:p['seq'])
        if len(cps)<2 or [p['seq'] for p in cps]!=list(range(1,len(cps)+1)):
            raise ValueError('Need consecutive checkpoint seq values starting at 1, with start and finish.')
        if any(not math.isfinite(p[v]) for p in cps for v in ('x','y')):raise ValueError('Non-finite checkpoint coordinate')
        result=dict(scenario=mission.get('scenario',''),checkpoints=cps,legs=[],mode='whole_mission_xodr_precomputed',
            initialized_ok=False,checkpoint_pairing_warning=len(cps)%2!=0,corridor_components=[],
            between_gate_warnings=[],endpoint_warnings=[],source='raw checkpoints + XODR only')
        candidates=[]
        for i,cp in enumerate(cps):
            # Direction along a road interval; never average across a turn.
            if i==0:a,b=cp,cps[i+1]
            elif i==len(cps)-1 or i%2==1:a,b=cps[i-1],cp
            else:a,b=cp,cps[i+1]
            role='road' if i in (0,len(cps)-1) or result['checkpoint_pairing_warning'] else ('entry' if i%2==1 else 'exit')
            candidates.append(self.candidates(cp,math.atan2(b['y']-a['y'],b['x']-a['x']),role))
        result['candidate_counts']=[len(c) for c in candidates]
        current={}
        for c,match in candidates[0].items():
            for k in self.components[c]:
                if k[:2]!=match['nearest_lane'][:2]:continue
                current[k]=dict(cost=match['score'],route=[k],matches=[match],spans=[])
        for i in range(1,len(cps)):
            current=self.advance(current,candidates[i],require_junction=(i%2==0 and i<len(cps)-1 and not result['checkpoint_pairing_warning']))
            if not current:
                result['failure']=dict(from_seq=i,to_seq=i+1,reason='No directed XODR sequence through nearby road corridor candidates; no fabricated link.')
                return result
        best=min(current.values(),key=lambda r:(r['cost'],r['route']));route=best['route'];selected=[]
        for k in route:
            c=self.owner.get(k)
            if c is not None and (not selected or selected[-1]!=c):selected.append(c)
        for i,c in enumerate(selected,1):
            result['corridor_components'].append(dict(id=i,key=c,lanes=sorted(self.components[c]),roads=sorted({k[0] for k in self.components[c]})))
        for i,(a,b) in enumerate(zip(selected,selected[1:])):
            gate=self.gateway(a,b)
            interval=next(((j+1,j+2) for j,(lo,hi) in enumerate(best['spans'])
                if any(self.owner.get(k)==a for k in route[lo:hi+1]) and any(self.owner.get(k)==b for k in route[lo:hi+1])),(1,len(cps)))
            gate.update(index=i,from_seq=interval[0],entry_seq=interval[0],exit_seq=interval[1],topology=True,
                allowed=sorted(self.components[a]|self.components[b]|set(gate['connection_lanes'])),
                entry_candidates=[],targets=[],preferred=[],cached_routes=[],notes=[])
            result['legs'].append(gate)
        if not result['legs']:
            result['legs']=[dict(index=0,from_seq=1,entry_seq=1,exit_seq=len(cps),topology=True,
                allowed=sorted(set().union(*(self.components[c] for c in selected))),entry_candidates=[],targets=[],
                preferred=[],cached_routes=[],notes=[],connection_lanes=[],connection_edges=[],entry_edges=[],exit_edges=[],status='same_corridor')]
        result['checkpoint_matches']=[dict(seq=cp['seq'],**match) for cp,match in zip(cps,best['matches'])]
        result['witness_route']=route
        result['witness_lateral_edges']=[(a,b) for a,b in zip(route,route[1:]) if (a,b) in self.lateral]
        result['checkpoint_spans']=best['spans'];result['search_cost']=best['cost']
        result['intervals']=[dict(from_seq=i+1,to_seq=i+2,lane_route=route[a:b+1],
            intermediate_roads=list(dict.fromkeys(k[0] for k in route[a:b+1]))) for i,(a,b) in enumerate(best['spans'])]
        # An alternative must also be able to continue through later gateways,
        # not merely arrive somewhere on the next road.
        next_good={k for k in self.components[selected[-1]] if k[:2]==best['matches'][-1]['nearest_lane'][:2]}
        result['corridor_reachability']=[]
        for i in range(len(selected)-1,-1,-1):
            corridor=self.components[selected[i]]
            gate=result['legs'][i] if i<len(selected)-1 else None
            permitted=set(corridor)
            if gate:permitted|=self.components[selected[i+1]]|set(gate['connection_lanes'])
            reverse=defaultdict(set)
            for a in permitted:
                for b,_ in self.search_graph[a]:
                    if b in permitted:reverse[b].add(a)
            good=set();pending=list(next_good)
            while pending:
                k=pending.pop()
                if k in good:continue
                good.add(k);pending.extend(reverse[k]-good)
            if gate:
                for field in ('entry_edges','exit_edges','connection_edges'):
                    gate[field]=[(a,b) for a,b in gate[field] if a in good and b in good]
                gate['connection_lanes']=[k for k in gate['connection_lanes'] if k in good]
                gate['allowed']=sorted(corridor|self.components[selected[i+1]]|set(gate['connection_lanes']))
                gate['status']='connected' if gate['entry_edges'] and gate['exit_edges'] else 'UNRESOLVED'
            next_good=good&corridor
            result['corridor_reachability'].append(dict(corridor=i+1,reachable=sorted(next_good),
                not_reachable=sorted(corridor-next_good)))
        result['corridor_reachability'].reverse()
        for gate in result['legs']:
            if gate.get('connection_lanes'):self.gateway_surface(gate)
        # User-facing mission regions use start->entry / exit->next-entry /
        # last-exit->finish. Small XODR junctions inside those road intervals
        # are internal connector pieces, not extra declared mission gateways.
        result['corridor_groups']=[];result['gateway_groups']=[]
        for i,interval in enumerate(result['intervals']):
            keys=set(interval['lane_route'])
            owners={self.owner[k] for k in keys if k in self.owner}
            related=[g for g in result['legs'] if g['entry_seq']==i+1 and g['exit_seq']==i+2]
            is_gateway=i%2==1 and not result['checkpoint_pairing_warning']
            group=dict(index=len(result['gateway_groups'] if is_gateway else result['corridor_groups'])+1,
                from_seq=i+1,to_seq=i+2,component_keys=sorted(owners),
                connector_indices=[g['index'] for g in related])
            region=set().union(*(self.components[c] for c in owners)) if owners else set()
            for g in related:region.update(g.get('surface_lanes',g['connection_lanes']))
            group['lanes']=sorted(region)
            if is_gateway:
                result['gateway_groups'].append(group)
                for g in related:g['declared_gateway_index']=group['index']
            else:result['corridor_groups'].append(group)
        for g in result['legs']:g.setdefault('declared_gateway_index',None)
        # Start/finish are longitudinal bounds across the road width, not lane
        # targets. Do not paint an entire road behind spawn or beyond finish.
        result['surface_clips']=[];excluded=set();limits={}
        for at_start,match in ((True,best['matches'][0]),(False,best['matches'][-1])):
            c=match['component']
            if selected.count(c)>1:continue  # another visit may need that area
            graph=self.forward if at_start else self.reverse
            pending=[k for k in self.components[c] if k[:2]==match['nearest_lane'][:2]]
            seen=set()
            while pending:
                k=pending.pop()
                if k in seen:continue
                seen.add(k);pending.extend(b for b in graph[k] if b in self.components[c] and b not in seen)
            sections={k[:2] for k in seen}
            excluded.update(k for k in self.components[c] if k[:2] not in sections)
            for k in self.components[c]:
                if k[:2]!=match['nearest_lane'][:2]:continue
                bounds=limits.setdefault(k,{})
                field='road_s_min' if (at_start and k[2]<0) or (not at_start and k[2]>0) else 'road_s_max'
                bounds[field]=match['road_s']
        result['surface_excluded_lanes']=sorted(excluded)
        result['surface_clips']=[dict(key=k,**bounds) for k,bounds in sorted(limits.items())]
        gaps=[];center_steps=[];checked=set()
        for a,b in zip(route,route[1:]):
            if (a,b) in self.lateral:continue
            pa=np.asarray(self.geometry[a]['points'])[-1];pb=np.asarray(self.geometry[b]['points'])[0]
            gap=float(np.linalg.norm(pa[:2]-pb[:2]))
            if gap>.1:center_steps.append(dict(from_lane=a,to_lane=b,gap_m=gap))
        # Check every selected gateway alternative as well as the witness route.
        pairs=list(zip(route,route[1:]))+[edge for gate in result['legs'] for edge in gate['connection_edges']]
        pairs += [edge for gate in result['legs'] for extension in gate.get('surface_extensions',[])
                  for edge in extension['interface_edges']]
        for a,b in pairs:
            if (a,b) in self.lateral or (a,b) in checked:continue
            checked.add((a,b));sources=[a];targets=[b]
            if a in self.owner and b in self.owner and self.owner[a]==self.owner[b]:
                # A corridor is a whole same-direction road surface. A lane ID
                # handover can step sideways while the corridor surface overlaps.
                sources=[k for k in self.components[self.owner[a]] if k[:2]==a[:2]]
                targets=[k for k in self.components[self.owner[b]] if k[:2]==b[:2]]
            separation=float('inf')
            for aa in sources:
                pa=np.asarray(self.geometry[aa]['points'])[-1]
                for bb in targets:
                    pb=np.asarray(self.geometry[bb]['points'])[0]
                    separation=min(separation,segment_distance(pa[5:7],pa[7:9],pb[5:7],pb[7:9]))
            if separation>.1:gaps.append(dict(from_lane=a,to_lane=b,surface_gap_m=separation))
        result['centerline_step_warnings']=center_steps
        result['geometry_gap_warnings']=gaps
        result['initialized_ok']=all(g['status']!='UNRESOLVED' for g in result['legs'])
        result['validation']=dict(ordered_lane_graph_connected=True,gateway_interfaces_connected=result['initialized_ok'],
            checkpoint_rules_valid=not result['checkpoint_pairing_warning'],scenario_specific_rules=False,geometry_gap_count=len(gaps),
            surface_interfaces_connected=not gaps,centerline_step_count=len(center_steps),
            runtime_applied=False,vehicle_feasibility_validated=False)
        return result
