import sys, os, numpy as np, trimesh, xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(__file__))
from fk import rpy, load, fk
def T_of(o):
    xyz=[float(v) for v in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]
    r=[float(v) for v in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
    T=np.eye(4); T[:3,:3]=rpy(*r); T[:3,3]=xyz; return T
def pts(urdf, pkgroot, frame_z_off=0.0, links_skip=('arm','gripper','RMX','link_')):
    J=load(urdf); root=ET.parse(urdf).getroot(); out={}
    for L in root.findall('link'):
        n=L.get('name'); _,Tl=fk(J,n)
        for c in L.findall('collision'):
            g=c.find('geometry'); To=Tl@T_of(c.find('origin'))
            b=g.find('box'); m=g.find('mesh'); cy=g.find('cylinder'); sp=g.find('sphere')
            if b is not None:
                s=np.array([float(v) for v in b.get('size').split()])/2
                P=np.array([[i,j,k] for i in (-1,1) for j in (-1,1) for k in (-1,1)])*s
            elif cy is not None:
                r=float(cy.get('radius')); h=float(cy.get('length'))/2
                a=np.linspace(0,2*np.pi,36,endpoint=False)
                P=np.array([[r*np.cos(t),r*np.sin(t),z] for t in a for z in (-h,h)])
            elif sp is not None:
                r=float(sp.get('radius')); P=trimesh.creation.icosphere(2,r).vertices
            else:
                f=m.get('filename'); f=f.split('://',1)[1]; f=os.path.join(pkgroot, f.split('/',1)[1])
                sc=np.array([float(v) for v in m.get('scale','1 1 1').split()])
                P=trimesh.load(f,force='mesh').vertices*sc
            P=(To[:3,:3]@P.T).T+To[:3,3]
            out.setdefault(n,[]).append(P)
    return {k:np.vstack(v) for k,v in out.items()}
def rep(name, urdf, pkg):
    P=pts(urdf,pkg); J=load(urdf)
    _,Tb=fk(J,'base_link'); print(f'## {name}: base_link z in base_footprint = {Tb[2,3]:.4f}')
    arm=lambda k: any(s in k for s in ('arm','gripper','RMX','camera'))
    body=np.vstack([v for k,v in P.items() if not arm(k)])
    allp=np.vstack(list(P.values()))
    f=lambda A:' '.join(f'[{A[:,i].min():+.4f},{A[:,i].max():+.4f}]' for i in range(3))
    print(' body(non-arm) AABB xyz (base_footprint):', f(body))
    print(' all AABB:', f(allp))
    zb=Tb[2,3]
    for lo,hi in ((0.048,0.088),):
        band=body[(body[:,2]-zb>=lo)&(body[:,2]-zb<=hi)]
        print(f' lidar band base_link z {lo}-{hi}: x [{band[:,0].min():+.4f},{band[:,0].max():+.4f}] y [{band[:,1].min():+.4f},{band[:,1].max():+.4f}]  (vertex-only)')
    low=body[body[:,2]<=0.45]
    print(' rear end x (z<=0.45):', f'{low[:,0].min():+.4f}', ' half width:', f'{np.abs(low[:,1]).max():.4f}')
    big=sorted(((k,v[:,0].min(),v[:,0].max()) for k,v in P.items() if not arm(k)), key=lambda t:t[1])[:3]
    print(' rearmost links:', big)
if __name__=='__main__': rep('dev', sys.argv[1], sys.argv[2]); rep('int', sys.argv[3], sys.argv[4])
