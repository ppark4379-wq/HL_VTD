"""Source-based road drawing, sampled lane overlay and standalone inspection UI."""
from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
import html
import json
import math
from pathlib import Path
import re

import numpy as np

from xodr_network import number


@dataclass
class Shape:
    layer: str
    kind: str
    xy: list
    color: str
    width: float = .13
    label: str = ""
    road: int = -1


class Drawing:
    def __init__(self, network):
        self.network = network
        self.shapes = []
        self.feature_counts = Counter()
        self.feature_issues = []
        self.defaults = Counter()
        self.draw_roads()
        self.draw_objects()
        self.draw_paths()
        all_points = np.array([(p.x, p.y) for p in network.points])
        lo, hi = all_points.min(axis=0)-25, all_points.max(axis=0)+25
        self.bounds = [float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])]

    def add(self, layer, kind, xy, color, width=.13, label="", road=-1):
        self.shapes.append(Shape(layer, kind, np.asarray(xy).tolist(), color, width, label, road))

    def boundary(self, road, section, lane, stations, shift=0.):
        return [road.at_offset(s, road.offset_jet(section, lane, s, outer=True)[0]+shift) for s in stations]

    def draw_roads(self):
        for rid, road in sorted(self.network.roads.items()):
            for si, lanes in enumerate(road.lanes):
                start, end = road.starts[si], road.end(si)
                stations = road.stations(si, .5)
                for lid, lane in sorted(lanes.items()):
                    if lid:
                        outer = self.boundary(road, si, lid, stations)
                        inner = []
                        for s in stations:
                            t = road.offset_jet(si, lid, s, outer=True)[0]
                            t -= (1 if lid > 0 else -1)*road.widths[si][lid].jet(s)[0]
                            inner.append(road.at_offset(s, t))
                        color = {"driving":"#343d43", "sidewalk":"#737b7c",
                                 "border":"#575d59", "parking":"#46535a"}.get(lane.get("type"), "#51533f")
                        self.add("surface", "polygon", outer+inner[::-1], color,
                                 label=f"road {rid} section {si} lane {lid} {lane.get('type')}", road=rid)
                    marks = lane.findall("roadMark")
                    for mi, mark in enumerate(marks):
                        a = start+number(mark, "sOffset")
                        b = min(end, start+number(marks[mi+1], "sOffset") if mi+1 < len(marks) else end)
                        kind = mark.get("type", "none")
                        self.feature_counts["roadmark_"+kind] += 1
                        if kind == "none" or b <= a:
                            continue
                        color = {"yellow":"#dfac32", "white":"#f1f1e8", "standard":"#f1f1e8",
                                 "blue":"#388ce8", "red":"#e15858", "green":"#59ab76"}.get(mark.get("color"), "#f1f1e8")
                        width = number(mark, "width", .13) or .13
                        label = f"road {rid}/{si}/{lid} {kind} {mark.get('color')} laneChange={mark.get('laneChange')}"
                        lines = mark.findall("type/line")
                        if lines:
                            for line in lines:
                                dash, gap = number(line,"length"), number(line,"space")
                                self.mark_line(road, si, lid, a+number(line,"sOffset"), b,
                                               number(line,"tOffset"), color, number(line,"width",width),
                                               dash, gap, label)
                        else:
                            parts = kind.split()
                            if len(parts) > 1:
                                self.defaults["double_line_separation"] += 1
                            for i, part in enumerate(parts):
                                if part not in ("solid", "broken"):
                                    self.feature_issues.append(dict(road=rid, kind="unsupported_roadmark", name=kind))
                                    continue
                                shift = (i-(len(parts)-1)/2)*width*2
                                if part == "broken":
                                    self.defaults["broken_dash_3m_gap_6m"] += 1
                                self.mark_line(road, si, lid, a, b, shift, color, width,
                                               3. if part == "broken" else b-a, 6. if part == "broken" else 0., label)

    def mark_line(self, road, si, lid, start, end, shift, color, width, dash, gap, label):
        if dash <= 0 or end <= start:
            return
        for a in np.arange(start, end, dash+gap if gap else end-start+1):
            b = min(end, a+dash)
            ss = np.linspace(a, b, max(1, math.ceil((b-a)/.5))+1)
            self.add("markings", "line", self.boundary(road, si, lid, ss, shift), color, width, label, road.id)

    def draw_objects(self):
        for rid, road in sorted(self.network.roads.items()):
            for obj in road.node.findall("objects/object"):
                name = obj.get("name", "")
                lower = name.lower()
                if not (lower.startswith(("rm_", "roadmark_", "crosswalk")) or obj.get("type") == "crosswalk"):
                    continue
                try:
                    s,t,h,length,width = (number(obj,n) for n in ("s","t","hdg","length","width"))
                    if not (0 <= s <= road.length+.01 and abs(t) < 60 and .05 <= length <= 60 and .05 <= width <= 60):
                        raise ValueError("invalid source pose/dimensions")
                except ValueError as exc:
                    self.feature_issues.append(dict(road=rid,id=obj.get("id"),name=name,kind="invalid_object",detail=str(exc)))
                    continue
                center = road.at_offset(s,t)
                heading = road.ref(s)[2]+h
                cs,sn = math.cos(heading),math.sin(heading)
                def world(points):
                    return [[center[0]+u*cs-v*sn,center[1]+u*sn+v*cs] for u,v in points]
                label = f"road {rid} object {obj.get('id')} {name}; s={s:.3f} t={t:.3f} hdg={h:.4f}"
                def polygon(points, color="#f5f3e8"):
                    self.add("objects","polygon",world(points),color,label=label,road=rid)
                def line(points,w=.18):
                    self.add("objects","line",world(points),"#f5f3e8",w,label,rid)
                def rectangle(u0,u1,v0,v1):
                    polygon([(u0,v0),(u1,v0),(u1,v1),(u0,v1)])
                if "stopline" in lower:
                    self.feature_counts["stop_line"] += 1
                    rectangle(-length/2,length/2,-width/2,width/2)
                elif lower.startswith("rm_532_") or lower.startswith("crosswalk") or obj.get("type") == "crosswalk":
                    self.feature_counts["crosswalk"] += 1
                    self.defaults["crosswalk_symbol_stripes"] += 1
                    # XODR supplies the rectangle, not the FLT's stripe texture.
                    for v in np.arange(-width/2,width/2,.9):
                        rectangle(-length/2,length/2,float(v),min(width/2,float(v)+.45))
                elif re.match(r"RM_53[789]_",name,re.I):
                    code = name.rsplit("_",1)[-1].split(".")[0].upper()
                    self.feature_counts["arrow_"+code] += 1
                    self.defaults["arrow_symbol_from_model_name"] += 1
                    # Symbolic top-down glyph; position, size, heading are source values.
                    def head(tip,base,left,right):
                        polygon([tip,left,base,right])
                    line([(-length/2,0),(length*.16,0)],width*.24)
                    if "S" in code:
                        line([(length*.1,0),(length*.30,0)],width*.24)
                        head((length/2,0),(length*.24,0),(length*.24,width/2),(length*.24,-width/2))
                    for turn,sign in (("L",1),("R",-1)):
                        if turn in code:
                            line([(length*.08,0),(length*.22,sign*width*.24)],width*.22)
                            polygon([(length*.22,sign*width/2),(length*.38,sign*width*.12),
                                     (length*.22,sign*width*.20),(length*.06,sign*width*.12)])
                    if "U" in code:
                        line([(length*.12,0),(length*.32,0),(length*.38,width*.3),(length*.1,width*.4)],width*.2)
                        polygon([(0,width*.4),(length*.18,width*.2),(length*.18,width*.5)])
                elif "warning_crosswalk" in lower:
                    self.feature_counts["crosswalk_warning_diamond"] += 1
                    line([(-length/2,0),(0,width/2),(length/2,0),(0,-width/2),(-length/2,0)],.15)
                elif "speed" in lower or lower.startswith("rm_517_"):
                    self.feature_counts["speed_marking"] += 1
                    label_text = re.findall(r"\d+",name)[-1]
                    self.add("objects","text",[center],"#f5f3e8",min(length,width)*.7,label_text,rid)
                else:
                    # Keep every other pavement object visible and identifiable;
                    # don't silently invent the meaning of numeric model names.
                    self.feature_counts["other_pavement_object"] += 1
                    corners=[(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2),(-length/2,-width/2)]
                    self.add("objects","line",world(corners),"#9ba3ac",.08,label,rid)
                    self.add("object_labels","text",[center],"#cad0d5",.6,name.removesuffix('.flt'),rid)
                    self.feature_issues.append(dict(road=rid,id=obj.get("id"),name=name,kind="model_footprint_only"))

    def draw_paths(self):
        for key,lane in self.network.lanes.items():
            label=f"{key.text()} | {len(lane.points)} samples | {lane.points[-1].lane_s:.1f} m"
            self.add("raw_paths","line",[[p.x,p.y] for p in lane.points],"#73b9d1",.12,
                     'RAW XODR '+label,key.road)
            if key in self.network.excluded_lanes or key in self.network.topology_unconnected_lanes:
                self.add('unconnected_paths','line',[[p.x,p.y] for p in lane.points],'#ff9b63',.18,
                         self.network.lane_access[key]['status']+' '+label,key.road)
                if key in self.network.excluded_lanes:
                    continue
            run=[]
            for p in lane.points:
                if p.id in self.network.replaced_points:
                    if len(run)>1:
                        self.add("paths","line",run,"#ffff00",.20,label,key.road)
                    run=[]
                    continue
                run.append([p.x,p.y])
                if p.next_id == -1:
                    if len(run)>1:
                        self.add("paths","line",run,"#ffff00",.20,label,key.road)
                    run=[]
            last=-100.
            for p in lane.points:
                if p.id in self.network.replaced_points:
                    continue
                if p.lane_s-last < 18 or p.lane_s < min(3.,lane.points[-1].lane_s/2):
                    continue
                last=p.lane_s
                c,s=math.cos(p.heading),math.sin(p.heading)
                local=[[-.85,.42],[0,0],[-.85,-.42]]
                self.add("direction","line",[[p.x+u*c-v*s,p.y+u*s+v*c] for u,v in local],
                         "#ffff00",.19,label,key.road)
        for t in self.network.transitions:
            if t.id in self.network.excluded_transitions:
                continue
            label=f"{t.kind} #{t.id} {t.event_lane.text()} | anchors {t.from_point} → {t.to_point} | {t.shape_method}"
            self.add('paths','line',[[p.x,p.y] for p in t.points],'#ffff00',.20,label,t.event_lane.road)
            last=-4.
            for p in t.points:
                if p.s-last<12 or p.s<4:
                    continue
                last=p.s
                c,s=math.cos(p.heading),math.sin(p.heading)
                self.add('direction','line',[[p.x+u*c-v*s,p.y+u*s+v*c] for u,v in ((-.85,.42),(0,0),(-.85,-.42))],
                         '#ffff00',.19,label,t.event_lane.road)
            for p in (t.points[0],t.points[-1]):
                self.add('connections','dot',[[p.x,p.y]],'#61ddff',.32,label,t.event_lane.road)
        for e in self.network.connections:
            if e.target not in self.network.lanes:
                continue
            a=self.network.lanes[e.source].points[-1]
            b=self.network.lanes[e.target].points[0]
            label=f"{e.source.text()} → {e.target.text()} {e.provenance} [{e.status}] gap={e.gap:.3f}m"
            # A cyan dot identifies the declared connection; no yellow interpolated geometry.
            if e.status == "continuous":
                self.add("connections","dot",[[a.x,a.y]],"#61ddff",.32,label,e.source.road)
            else:
                self.add("connections","line",[[a.x,a.y],[b.x,b.y]],"#ff626d",.12,label,e.source.road)
        for i in self.network.issues:
            self.add("issues","ring",[[i['x'],i['y']]],"#ff626d",.8,
                     f"{i['lane']} {i['kind']}: {i['detail']}",int(i['lane'].split(':')[0]))

    def svg(self, overlay=True, inspect=False, bounds=None):
        x0,y0,x1,y1=bounds or self.bounds
        w,h=x1-x0,y1-y0
        groups=defaultdict(list)
        for sh in self.shapes:
            if not overlay and sh.layer in ('paths','raw_paths','unconnected_paths','direction','connections','issues'):
                continue
            if not inspect and sh.layer in ('connections','issues','object_labels','raw_paths','unconnected_paths'):
                continue
            title=f'<title>{html.escape(sh.label)}</title>' if sh.label else ''
            attrs=f'data-road="{sh.road}"'
            points=' '.join(f'{x:.3f},{-y:.3f}' for x,y in sh.xy)
            if sh.kind=='polygon':
                el=f'<polygon {attrs} points="{points}" fill="{sh.color}">{title}</polygon>'
            elif sh.kind=='line':
                el=f'<polyline {attrs} points="{points}" fill="none" stroke="{sh.color}" stroke-width="{sh.width:.3f}" stroke-linejoin="round">{title}</polyline>'
            elif sh.kind in ('dot','ring'):
                x,y=sh.xy[0]
                el=f'<circle {attrs} cx="{x:.3f}" cy="{-y:.3f}" r="{sh.width}" fill="{sh.color if sh.kind=="dot" else "none"}" stroke="{sh.color}" stroke-width=".13">{title}</circle>'
            else:
                x,y=sh.xy[0]
                el=f'<text {attrs} x="{x:.3f}" y="{-y:.3f}" text-anchor="middle" fill="{sh.color}" font-size="{sh.width}" font-family="sans-serif">{html.escape(sh.label)}</text>'
            groups[sh.layer].append(el)
        parts=[f'<svg xmlns="http://www.w3.org/2000/svg" id="map" width="{w:.1f}" height="{h:.1f}" viewBox="{x0:.3f} {-y1:.3f} {w:.3f} {h:.3f}">',
               '<rect x="-100000" y="-100000" width="200000" height="200000" fill="#182126"/>']
        for layer in ('surface','markings','objects','object_labels','raw_paths','unconnected_paths','paths','direction','connections','issues'):
            style=' style="display:none"' if inspect and layer in ('connections','issues','object_labels','raw_paths','unconnected_paths') else ''
            parts.append(f'<g id="{layer}"{style}>')
            parts.extend(groups[layer])
            parts.append('</g>')
        parts.append('<g id="sample_points"/></svg>')
        return '\n'.join(parts)

    def png(self, output: Path, overlay=True, bounds=None, pixels=8000, diagnostic=False):
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.collections import LineCollection, PolyCollection
        from matplotlib.patches import Circle
        x0,y0,x1,y1=bounds or self.bounds
        w,h=x1-x0,y1-y0
        dpi=160
        scale=pixels/max(w,h)
        fig=Figure(figsize=(w*scale/dpi,h*scale/dpi),dpi=dpi,facecolor='#182126')
        FigureCanvasAgg(fig)
        ax=fig.add_axes([0,0,1,1],facecolor='#182126')
        ax.set(xlim=(x0,x1),ylim=(y0,y1),aspect='equal')
        ax.set_axis_off()
        for layer in ('surface','markings','objects','paths','direction','connections','issues'):
            if layer in ('connections','issues') and not diagnostic:
                continue
            if not overlay and layer in ('paths','direction','connections','issues'):
                continue
            shapes=[sh for sh in self.shapes if sh.layer==layer and
                    max(p[0] for p in sh.xy)>=x0 and min(p[0] for p in sh.xy)<=x1 and
                    max(p[1] for p in sh.xy)>=y0 and min(p[1] for p in sh.xy)<=y1]
            polygons=[s for s in shapes if s.kind=='polygon']
            lines=[s for s in shapes if s.kind=='line']
            if polygons:
                ax.add_collection(PolyCollection([s.xy for s in polygons],facecolors=[s.color for s in polygons],edgecolors='none',antialiased=False))
            if lines:
                ax.add_collection(LineCollection([s.xy for s in lines],colors=[s.color for s in lines],
                                                 linewidths=[s.width*scale*72/dpi for s in lines],antialiased=True))
            for sh in shapes:
                if sh.kind=='text':
                    ax.text(*sh.xy[0],sh.label,color=sh.color,fontsize=max(1,sh.width*scale*72/dpi),ha='center',va='center')
                elif sh.kind in ('ring','dot'):
                    ax.add_patch(Circle(sh.xy[0],sh.width,fill=sh.kind=='dot',color=sh.color,linewidth=.5))
        fig.savefig(output,dpi=dpi,facecolor=fig.get_facecolor())
        fig.clear()

    def inspection_html(self, report):
        template=Path(__file__).with_name('inspector.html').read_text(encoding='utf-8')
        issues=list(self.network.issues)
        for key in sorted(self.network.excluded_lanes | self.network.topology_unconnected_lanes):
            p=self.network.lanes[key].points[0]
            issues.append(dict(kind='topology_'+self.network.lane_access[key]['status'],lane=key.text(),x=p.x,y=p.y,
                               detail='Topology diagnostic only: raw geometric candidates remain visible unless explicitly excluded by user review. Runtime planning decides usability; see lane_access.csv.'))
        data=json.dumps({'bounds':self.bounds,'issues':issues,'counts':report['counts']},ensure_ascii=False).replace('</','<\\/')
        return template.replace('__SVG__',self.svg(inspect=True)).replace('__DATA__',data)
