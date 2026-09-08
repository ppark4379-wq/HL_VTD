"""Read-only map/boundary diagnostics and SVG raster previews for review."""
import argparse
import collections
import ctypes as C
import ctypes.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/dashboard'))
from opendrive_assessment import OpenDriveMap
from school_zones import SchoolZones


def render_svg(source, destination, bounds=None):
    tree=ET.parse(source)
    root=tree.getroot()
    if bounds: root.set('viewBox',' '.join(map(str,bounds)))
    size=800
    root.set('width',str(size))
    root.set('height',str(size))
    data=ET.tostring(root)
    r=C.CDLL(ctypes.util.find_library('rsvg-2'))
    c=C.CDLL(ctypes.util.find_library('cairo'))
    g=C.CDLL(ctypes.util.find_library('gobject-2.0'))
    r.rsvg_handle_new_from_data.argtypes=[C.c_char_p,C.c_size_t,C.c_void_p]; r.rsvg_handle_new_from_data.restype=C.c_void_p
    c.cairo_image_surface_create.argtypes=[C.c_int,C.c_int,C.c_int]; c.cairo_image_surface_create.restype=C.c_void_p
    c.cairo_create.argtypes=[C.c_void_p]; c.cairo_create.restype=C.c_void_p
    class Rectangle(C.Structure): _fields_=[(key,C.c_double) for key in ('x','y','width','height')]
    r.rsvg_handle_render_document.argtypes=[C.c_void_p,C.c_void_p,C.POINTER(Rectangle),C.c_void_p]
    c.cairo_surface_write_to_png.argtypes=[C.c_void_p,C.c_char_p]
    c.cairo_destroy.argtypes=c.cairo_surface_destroy.argtypes=g.g_object_unref.argtypes=[C.c_void_p]
    handle=r.rsvg_handle_new_from_data(data,len(data),None)
    if not handle: raise ValueError('could not parse SVG')
    surface=c.cairo_image_surface_create(0,size,size); context=c.cairo_create(surface)
    if not r.rsvg_handle_render_document(handle,context,C.byref(Rectangle(0,0,size,size)),None):
        raise ValueError('SVG render failed')
    if c.cairo_surface_write_to_png(surface,str(destination).encode()): raise ValueError('PNG write failed')
    c.cairo_destroy(context); c.cairo_surface_destroy(surface); g.g_object_unref(handle)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--svg')
    parser.add_argument('--render')
    parser.add_argument('--bounds',type=float,nargs=4)
    parser.add_argument('--details',action='store_true')
    parser.add_argument('--roads',type=int,nargs='+')
    parser.add_argument('--corrected',action='store_true')
    args=parser.parse_args()
    if args.render:
        render_svg(args.svg,args.render,args.bounds)
        print(args.render)
        return
    road_map=OpenDriveMap(ROOT/'src/data/map/HL_FMA_VTD_LivingLab.xodr')
    source=ROOT/'tools/dashboard/living_lab_school_zones.yaml'
    corrections=ROOT/'tools/dashboard/living_lab_school_zone_corrections.yaml' if args.corrected else None
    zones=SchoolZones(source,road_map,corrections)
    data=yaml.safe_load(source.read_text())
    if args.roads:
        for rid in args.roads:
            road=road_map.roads[rid]
            print('ROAD',rid,'length',road.get('length'),'link',ET.tostring(road.find('link'),encoding='unicode'))
            for i,sec in enumerate(zones.sections[rid]):
                print('SECTION',i,sec.get('s'),[(l.attrib,ET.tostring(l.find('link'),encoding='unicode') if l.find('link') is not None else '')
                         for l in sec.findall('left/lane')+sec.findall('right/lane') if l.get('type')=='driving'])
        return
    ids={p['id'] for p in zones.pending}
    print('SUMMARY',len(zones.accepted),'accepted',len(zones.pending),'pending',len(zones.ranges),'ranges')
    print('PENDING',zones.pending)
    if corrections:
        for fix in yaml.safe_load(corrections.read_text())['corrections']:
            for key in ('point_1','point_2'):
                if key in fix:
                    p=fix[key]
                    print('CORRECTION',fix['id'],key,'claimed',p['s'],'projected',road_map.project(p['track_id'],(p['x'],p['y']),p['s']))
    for item in zones.accepted:
        if item['id'] in {'school_zone_12','school_zone_27','school_zone_29'}:
            print('RESOLVED',item,[r for r in zones.ranges if r['id']==item['id']])
    if args.svg:
        root=ET.parse(args.svg).getroot()
        ns={'s':'http://www.w3.org/2000/svg'}
        print('SVG',json.dumps(dict(bounds=root.get('viewBox'),groups=[g.get('id') for g in root.findall('s:g',ns)],
                  tags=dict(collections.Counter(n.tag.split('}')[-1] for n in root.iter())),
                  texts=[''.join(n.itertext()) for n in root.findall('.//s:text',ns)][-15:]),ensure_ascii=False))
    for zone in data['school_zones']:
        if zone['id'] not in ids: continue
        print('\nZONE',zone['id'],json.dumps(zone,ensure_ascii=False))
        if zone.get('point_2'): print('PATHS',zones._paths(zone['point_1'],zone['point_2']))
        if not args.details: continue
        for rid in sorted({p['track_id'] for key,p in zone.items() if key in ('point_1','point_2') and p}):
            road=road_map.roads[rid]
            print('ROAD',road.attrib,'LINK',ET.tostring(road.find('link'),encoding='unicode'))
            for i,sec in enumerate(zones.sections[rid]):
                lanes=[]
                for lane in sec.findall('left/lane')+sec.findall('right/lane'):
                    if lane.get('type') not in ('driving','none'): continue
                    lid=int(lane.get('id'))
                    lanes.append(dict(id=lid,type=lane.get('type'),
                                      next=zones._next(rid,i,lid,1),prev=zones._next(rid,i,lid,-1)))
                print('SECTION',i,sec.get('s'),json.dumps(lanes))


if __name__=='__main__': main()
