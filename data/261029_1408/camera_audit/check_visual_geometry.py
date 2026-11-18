"""현재 URDF의 실제 visual 매쉬와 광학 원점/전방 교차를 확인한다. 모델 수정 없음."""
import json,sys,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial import ConvexHull
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'Cartlidar/sim_world'));from robot_geom import Robot,origin_T
urdf=root/'261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf';pkg=urdf.parent.parent
q={'RMX52_Base_Yaw':0.,'RMX52_Shoulder':1.4,'RMX52_Elbow':-1.,'RMX52_Wrist_Pitch':-1.3,'RMX52_Wrist_Roll_Reversed':0.,'gripper_left_joint':0.}
rb=Robot(urdf,pkg);poses=rb.link_poses(q);visuals=[];visual_info=[]
for link in ET.parse(urdf).getroot().findall('link'):
 for v in link.findall('visual'):
  elem=v.find('geometry/mesh')
  if elem is None:continue
  fn=elem.get('filename');rel=fn.split('://',1)[1].split('/',1)[1] if fn.startswith('package://') else fn
  mesh=trimesh.load(pkg/rel,force='mesh');mesh.apply_scale([float(x) for x in elem.get('scale','1 1 1').split()]);mesh.apply_transform(poses[link.get('name')]@origin_T(v));visuals.append((link.get('name'),mesh));visual_info.append({'link':link.get('name'),'faces':len(mesh.faces),'watertight':bool(mesh.is_watertight),'bounds':mesh.bounds.tolist()})
collisions=rb.world_shapes(q);rows=[]
for side in ('front','rear'):
 for sensor in ('color','infra1','infra2'):
  name=f'top_camera_{side}_{sensor}_optical_frame';T=poses[name];origin=T[:3,3];inside=[]
  for link,mesh in visuals:
   if np.all(origin>=mesh.bounds[0]) and np.all(origin<=mesh.bounds[1]) and mesh.contains([origin])[0]:inside.append(link)
  inside_collision=[]
  for link,V in collisions:
   if np.all(origin>=V.min(0)) and np.all(origin<=V.max(0)):
    eq=ConvexHull(V).equations
    if np.max(eq[:,:3]@origin+eq[:,3])<=1e-8:inside_collision.append(link)
  # 축/시야 진단용 5x5광선. 교차 t는광학z거리(미터),실제pixelcoverage판정아님.
  xy=[(x,y) for y in (-.4,-.2,0,.2,.4) for x in (-.5,-.25,0,.25,.5)];dc=np.array([[x,y,1.] for x,y in xy]);directions=dc@T[:3,:3].T;origins=np.repeat(origin[None],len(xy),axis=0);nearest=np.full(len(xy),np.inf);nearest_link=[None]*len(xy);points=[None]*len(xy)
  for link,mesh in visuals:
   hits,which,_=mesh.ray.intersects_location(origins,directions,multiple_hits=True)
   for point,i in zip(hits,which):
    t=float((point-origin).dot(directions[i])/directions[i].dot(directions[i]))
    if t>1e-7 and t<nearest[i]:nearest[i]=t;nearest_link[i]=link;points[i]=point.tolist()
  center=xy.index((0,0));row={'frame':name,'origin_base_footprint_m':origin.tolist(),'forward_axis_base':T[:3,2].tolist(),'inside_visual_links':inside,'inside_collision_links':sorted(set(inside_collision)),'center_forward_hit_z_m':float(nearest[center]) if np.isfinite(nearest[center]) else None,'center_hit_link':nearest_link[center],'center_hit_point_base_m':points[center],'rays':25,'hits_under_near_10mm':int((nearest<.01).sum()),'hits_under_25mm':int((nearest<.025).sum()),'hits_under_100mm':int((nearest<.1).sum()),'ray_hits':[{'optical_dir':d.tolist(),'hit_z_m':float(t) if np.isfinite(t) else None,'link':l} for d,t,l in zip(dc,nearest,nearest_link)]};rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='ray_hits'},ensure_ascii=False),flush=True)
report={'urdf':str(urdf),'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'q':q,'visual_meshes':visual_info,'rows':rows,'existing_render_near_m':.01,'note':'포함 검사는 visual 매쉬의 닫힘 상태에 의존하며 collision은 근사다.'}
out=root/'Cartlidar/sim_render/results/followup_261028_1457/camera_geometry_audit';out.mkdir(exist_ok=True);(out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
