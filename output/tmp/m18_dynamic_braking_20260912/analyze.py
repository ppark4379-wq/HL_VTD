"""Read-only reconstruction of logged M18 OBB overlaps; does not simulate commands."""
import csv,json,math
from pathlib import Path
root=Path(__file__).resolve().parent
source=Path('output/verification/M18/m18_rviz')
rows=list(csv.DictReader((source/'run.csv').open()))
latest=json.loads((source/'rviz/latest.json').read_text())['ego']
c,s=math.cos(latest['heading']),math.sin(latest['heading'])
local=[((x-latest['x'])*c+(y-latest['y'])*s,-(x-latest['x'])*s+(y-latest['y'])*c) for x,y,*_ in latest['obb']]
front=max(x for x,y in local);rear=-min(x for x,y in local);width=max(y for x,y in local)-min(y for x,y in local)
# Round only the 4-decimal visualization serialization noise; matches vehicle config.
front,rear,width=round(front,3),round(rear,3),round(width,3)
def box(x,y,h,length,width,offset=0):
 c,s=math.cos(h),math.sin(h)
 return [(x+(offset+a)*c-d*s,y+(offset+a)*s+d*c) for a,d in [(length/2,width/2),(length/2,-width/2),(-length/2,-width/2),(-length/2,width/2)]]
def ego(row,back=0):
 x,y,h=(float(row[k]) for k in ['ego_x','ego_y','ego_heading'])
 return box(x-back*math.cos(h),y-back*math.sin(h),h,front+rear,width,(front-rear)/2)
def overlap(a,b):
 for points in (a,b):
  for i in range(2):
   dx=points[(i+1)%4][0]-points[i][0];dy=points[(i+1)%4][1]-points[i][1]
   u=[-x*dy+y*dx for x,y in a];v=[-x*dy+y*dx for x,y in b]
   if max(u)<min(v) or max(v)<min(u):return False
 return True
held=rows[1373]
report={'source':str(source),'ego_footprint':{'front':front,'rear':rear,'width':width},'restart_frame':1373,'restart_wall_s':float(held['elapsed_wall_time_s']),'objects':{}}
records=[]
for oid in ['6','7']:
 hits=[];frozen=[];back=[];mismatch=0
 for row in rows[1250:1600]:
  if row['nearest_object_id']!=oid:continue
  obb=box(*(float(row[k]) for k in ['nearest_object_x','nearest_object_y','nearest_object_heading','nearest_object_length','nearest_object_width']))
  actual=overlap(ego(row),obb);hold=overlap(ego(held),obb);earlier=overlap(ego(held,.5),obb)
  t=float(row['elapsed_wall_time_s'])
  if actual:hits.append(t)
  if hold:frozen.append(t)
  if earlier:back.append(t)
  mismatch+=actual!=(row['collision']=='1')
  records.append({'frame':row['frame'],'wall_time_s':t,'object_id':oid,'collision_logged':row['collision'],'collision_reconstructed':actual,'frozen_stop_pose_overlap':hold,'frozen_pose_0_5m_back_overlap':earlier})
 report['objects'][oid]={'logged_vs_reconstructed_mismatches':mismatch,'actual_contact_frames':len(hits),'actual_first_contact_s':hits[0] if hits else None,'actual_last_contact_s':hits[-1] if hits else None,'held_pose_overlap_frames':len(frozen),'held_pose_first_overlap_s':frozen[0] if frozen else None,'held_pose_0_5m_back_overlap_frames':len(back)}
(root/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
with (root/'obb_check.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
with (root/'event_window.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows[1250:1600])
print(json.dumps(report,indent=2))
