import sys,os,numpy as np,trimesh
sys.path.insert(0,os.path.dirname(__file__))
from band import geoms
from fk import load,fk
def occl(urdf,pkg,tag):
    J=load(urdf); P=[]
    for n,M in geoms(urdf,pkg):
        if n.startswith('lds') or n.startswith('laser'): continue
        if any(s in n for s in ('arm','gripper','RMX','camera')): continue
        V,F=trimesh.remesh.subdivide_to_size(M.vertices,M.faces,max_edge=0.002,max_iter=30)
        P.append(np.hstack([V,np.full((len(V),1),0)]) )
    P=np.vstack(P)[:,:3]
    for lid in ('lds_front','lds_rear'):
        o=fk(J,lid)[1][:3,3]
        Q=P[np.abs(P[:,2]-o[2])<0.003]
        d=Q[:,:2]-o[:2]; r=np.hypot(*d.T); a=np.degrees(np.arctan2(d[:,1],d[:,0]))%360
        Q=a[(r>0.03)&(r<0.5)]
        bins=np.zeros(720,bool); bins[(Q*2).astype(int)%720]=True
        # 닫힘: 1° 안 빈칸 메움
        b=bins.copy()
        for k in range(720):
            if not bins[k] and bins[k-1] and bins[(k+1)%720]: b[k]=True
        print(f'{tag} {lid} z={o[2]:.4f} 가림 {b.sum()/2:.1f}°', end=' ')
        # 보이는 구간 출력
        vis=~b; segs=[];k=0
        idx=np.where(np.diff(np.r_[vis,vis[0]].astype(int))!=0)[0]
        print('경계(°):',[round((i+1)/2,1) for i in idx])
S='/tmp/claude-1000/-home-pa4-git-lv2-next-module/dedff723-3979-4d03-ace8-ea4024e8f543/scratchpad/pr50'
for tag,u,p in (('dev',S+'/dev.urdf',S+'/dev/workspace/urdf_and_usd/waffle_4wd_description_261028'),('int',S+'/int.urdf',S+'/int/workspace/urdf_and_usd/waffle_4wd_description')):
    occl(u,p,tag)
