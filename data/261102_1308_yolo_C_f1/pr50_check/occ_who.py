import sys,os,numpy as np,trimesh
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from band import geoms
S=os.path.dirname(os.path.abspath(__file__))
for tag,u,p in (('dev',S+'/dev.urdf',S+'/dev/workspace/urdf_and_usd/waffle_4wd_description_261028'),('int',S+'/int.urdf',S+'/int/workspace/urdf_and_usd/waffle_4wd_description')):
    print('==',tag)
    for i,(n,M) in enumerate(geoms(u,p)):
        if n.startswith('lds') or any(s in n for s in ('arm','gripper','RMX','camera')): continue
        b=M.bounds
        if b[0,2]<=0.0755<=b[1,2] and (b[1,0]-b[0,0]>0.02 or b[1,1]-b[0,1]>0.02):
            print(f'  {n} #{i} x[{b[0,0]:+.3f},{b[1,0]:+.3f}] y[{b[0,1]:+.3f},{b[1,1]:+.3f}] z[{b[0,2]:+.4f},{b[1,2]:+.4f}]')
