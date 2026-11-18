"""스캔면에서 각도마다 처음 막는 부품(충돌 형상)을 찾는다."""
import sys,os,numpy as np,trimesh,xml.etree.ElementTree as ET
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from fk import load,fk
from bounds import T_of
S=os.path.dirname(os.path.abspath(__file__))
def parts(urdf,pkg):
    J=load(urdf); r=ET.parse(urdf).getroot(); out=[]
    for L in r.findall('link'):
        n=L.get('name'); _,Tl=fk(J,n)
        if n.startswith('lds') or any(s in n for s in ('arm','gripper','RMX','camera')): continue
        for c in L.findall('collision'):
            g=c.find('geometry'); To=Tl@T_of(c.find('origin')); b,m,cy=g.find('box'),g.find('mesh'),g.find('cylinder')
            if b is not None: M=trimesh.creation.box([float(v) for v in b.get('size').split()])
            elif cy is not None: M=trimesh.creation.cylinder(float(cy.get('radius')),float(cy.get('length')),sections=48)
            elif m is not None: M=trimesh.load(os.path.join(pkg,m.get('filename').split('://',1)[1].split('/',1)[1]),force='mesh')
            else: continue
            M.apply_transform(To); V,F=trimesh.remesh.subdivide_to_size(M.vertices,M.faces,max_edge=0.002,max_iter=30)
            out.append((f"{n}:{c.get('name') or ''}",V))
    return J,out
u,p=sys.argv[1],sys.argv[2]; J,P=parts(u,p)
for lid,angs in (('lds_front',[85,89,90,91,95,135,180]),('lds_rear',[105,109,111,115,180,250,252])):
    o=fk(J,lid)[1][:3,3]
    for a in angs:
        best=None
        for name,V in P:
            Q=V[np.abs(V[:,2]-o[2])<0.003]
            if not len(Q): continue
            d=Q[:,:2]-o[:2]; r=np.hypot(*d.T); aa=np.degrees(np.arctan2(d[:,1],d[:,0]))%360
            k=(np.abs(((aa-a+180)%360)-180)<0.6)&(r>0.03)&(r<0.6)
            if k.any():
                rr=r[k].min()
                if best is None or rr<best[1]: best=(name,rr,Q[k][np.argmin(r[k])])
        print(lid,a,'막는 것 없음' if best is None else f'{best[0]} {best[1]*1000:.0f} mm, 점 x {best[2][0]*1000:.0f} y {best[2][1]*1000:.0f} z(땅) {best[2][2]*1000+6.8:.1f}')
