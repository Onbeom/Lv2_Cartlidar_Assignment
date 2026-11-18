"""기존 18시야 형상을 준비. GPU 필터에 숨김정답을 넣지 않는다."""
import hashlib
import json
import sys
import time
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

ROOT=Path('/home/pa4/git/lv2_next_module')
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'))
from robot_geom import Robot
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
OUT=BASE/'improvement_261030_1325/filter_inputs'
OUT.mkdir(exist_ok=False)
first=json.loads((BASE/'camera_new_model_spec/summary.json').read_text())
urdf=Path(first['urdf']);start=time.perf_counter();robot=Robot(urdf,urdf.parent.parent)
local=[]
for link,items in robot.shapes.items():
    for T,V in items:
        local.append((link,T,V,ConvexHull(V).equations))
load_s=time.perf_counter()-start
rows=[]
for source,reference in [('camera_new_model_spec','nvblox_self_filter_new_model'),
                         ('camera_new_model_stow_holdout','nvblox_self_filter_stow_holdout'),
                         ('camera_new_model_zero_holdout','nvblox_self_filter_zero_holdout')]:
    meta=json.loads((BASE/source/'summary.json').read_text())
    for row in [v for v in meta['rows'] if v['visible']]:
        start=time.perf_counter();poses=robot.link_poses(meta['arm_q']);C=np.linalg.inv(poses[row['frame']]);k=row['K']
        planes=[];lohi=[];bounds=[];offsets=[0]
        for link,T,V,eq in local:
            M=C@poses[link]@T;R=M[:3,:3];t=M[:3,3]
            v=V@R.T+t;lo=v.min(0)-.005;hi=v.max(0)+.005
            if hi[2]<=0:continue
            if lo[2]<=0:b=(0,640,0,480)
            else:
                corners=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
                p=corners[:,:2]/corners[:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']]
                a=np.floor(p.min(0)).astype(int)-1;z=np.ceil(p.max(0)).astype(int)+2
                b=(max(0,int(a[0])),min(640,int(z[0])),max(0,int(a[1])),min(480,int(z[1])))
            if b[0]>=b[1] or b[2]>=b[3]:continue
            n=eq[:,:3]@R.T;d=eq[:,3]-n@t
            planes.extend(np.c_[n,d]);offsets.append(len(planes));lohi.append(np.r_[lo,hi]);bounds.append(b)
        prep_s=time.perf_counter()-start
        name=source+'__'+row['frame'];path=OUT/(name+'.npz')
        np.savez(path,planes=np.asarray(planes,np.float32),offsets=np.asarray(offsets,np.int32),
                 lohi=np.asarray(lohi,np.float32),bounds=np.asarray(bounds,np.int32),
                 k=np.array([k['fx'],k['fy'],k['cx'],k['cy']],np.float32),depth=np.load(BASE/source/row['depth']))
        rows.append({'input':str(path),'frame':row['frame'],'source':source,'reference':str(BASE/reference),
                     'prepare_geometry_s':prep_s,'hulls':len(bounds),'planes':len(planes),'arm_q':meta['arm_q']})
(OUT/'manifest.json').write_text(json.dumps({'rows':rows,'load_local_hulls_s':load_s,'urdf':str(urdf),
    'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'local_planes_precomputed':True},ensure_ascii=False,indent=2))
print(json.dumps({'views':len(rows),'load_local_hulls_s':load_s,'geometry_prepare_range_s':[min(v['prepare_geometry_s'] for v in rows),max(v['prepare_geometry_s'] for v in rows)]}))
