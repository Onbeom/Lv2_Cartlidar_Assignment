"""URDF visual 삼각형의 CAD 전방투영. 환경영상/정답깊이 사용 없음."""
import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import cv2
import numpy as np
import trimesh

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'))
from robot_geom import Robot,origin_T
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=BASE/'ir_visual_masks';OUT.mkdir(exist_ok=False)
old=json.loads((BASE/'ir_projection_masks/summary.json').read_text())
urdf=Path(old['rows'][0]['urdf']);robot=Robot(urdf,urdf.parent.parent);xml=ET.parse(urdf).getroot()
visuals=[]
for link in xml.findall('link'):
    for visual in link.findall('visual'):
        geom=visual.find('geometry');mesh=geom.find('mesh')
        if mesh is not None:
            filename=mesh.get('filename');rel=filename.split('://',1)[1].split('/',1)[1] if filename.startswith('package://') else filename
            obj=trimesh.load(urdf.parent.parent/rel,force='mesh')
            scale=np.array([float(x) for x in mesh.get('scale','1 1 1').split()])
            vertices=np.array(obj.vertices)*scale;faces=np.array(obj.faces)
        elif geom.find('box') is not None:
            obj=trimesh.creation.box(extents=list(map(float,geom.find('box').get('size').split())));vertices=np.array(obj.vertices);faces=np.array(obj.faces)
        elif geom.find('cylinder') is not None:
            c=geom.find('cylinder');obj=trimesh.creation.cylinder(radius=float(c.get('radius')),height=float(c.get('length')),sections=64);vertices=np.array(obj.vertices);faces=np.array(obj.faces)
        else:continue
        visuals.append((link.get('name'),origin_T(visual),vertices,faces))
def clip(poly,z,keep_above):
    out=[]
    for a,b in zip(poly[-1:]+poly[:-1],poly):
        ina=a[2]>=z if keep_above else a[2]<=z
        inb=b[2]>=z if keep_above else b[2]<=z
        if ina!=inb:out.append(a+(b-a)*(z-a[2])/(b[2]-a[2]))
        if inb:out.append(b)
    return out
reports=[]
for entry in old['rows']:
    start=time.perf_counter();poses=robot.link_poses(entry['q']);C=np.linalg.inv(poses[entry['frame']]);k=entry['K']
    masks={'two_sided':np.zeros((480,640),np.uint8),'front_faces':np.zeros((480,640),np.uint8)}
    triangles=0
    for link,T,V,F in visuals:
        M=C@poses[link]@T;v=V@M[:3,:3].T+M[:3,3];tris=v[F]
        valid=(tris[:,:,2].max(1)>=.01)&(tris[:,:,2].min(1)<=4.)
        for triangle in tris[valid]:
            facing=float(np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0])@triangle.mean(0))<0
            poly=clip(clip(list(triangle),.01,True),4.,False)
            if len(poly)<3:continue
            p=np.array(poly);uv=p[:,:2]/p[:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']]
            if uv[:,0].max()<0 or uv[:,0].min()>=640 or uv[:,1].max()<0 or uv[:,1].min()>=480:continue
            pixels=np.rint(uv).astype(np.int32)
            cv2.fillConvexPoly(masks['two_sided'],pixels,1)
            if facing:cv2.fillConvexPoly(masks['front_faces'],pixels,1)
            triangles+=1
    for mode,mask in masks.items():
        mask=cv2.dilate(mask,np.ones((3,3),np.uint8)).astype(bool)
        np.save(OUT/f"{entry['light']}_{entry['camera']}_{mode}.npy",mask)
        reports.append({**{k:v for k,v in entry.items() if k in ['light','camera','frame','reference_joint_stamp','q','K','urdf']},
                        'mode':mode,'masked_pixels':int(mask.sum()),'masked_fraction':float(mask.mean()),'guard_pass':float(mask.mean())<=.75,
                        'projected_triangles':triangles,'raster_s':time.perf_counter()-start})
    print(entry['light'],entry['camera'],[(r['mode'],r['masked_fraction']) for r in reports[-2:]],flush=True)
(OUT/'summary.json').write_text(json.dumps({'rows':reports,'forward_model_not_environment_ground_truth':True},ensure_ascii=False,indent=2))
