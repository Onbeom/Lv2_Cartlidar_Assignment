import sys, os, numpy as np, trimesh, xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(__file__))
from fk import load, fk
from bounds import T_of
def geoms(urdf, pkg):
    J=load(urdf); root=ET.parse(urdf).getroot()
    for L in root.findall('link'):
        n=L.get('name'); _,Tl=fk(J,n)
        for c in L.findall('collision'):
            g=c.find('geometry'); To=Tl@T_of(c.find('origin'))
            b,m,cy=g.find('box'),g.find('mesh'),g.find('cylinder')
            if b is not None: M=trimesh.creation.box([float(v) for v in b.get('size').split()])
            elif cy is not None: M=trimesh.creation.cylinder(float(cy.get('radius')),float(cy.get('length')),sections=48)
            elif m is not None:
                f=os.path.join(pkg, m.get('filename').split('://',1)[1].split('/',1)[1]); M=trimesh.load(f,force='mesh')
                M.apply_scale([float(v) for v in m.get('scale','1 1 1').split()]) if m.get('scale') else None
            else: continue
            M.apply_transform(To); yield n,M
def band(urdf,pkg,lo,hi,skip=('arm','gripper','RMX','camera')):
    xs=[];ys=[];who=[]
    for n,M in geoms(urdf,pkg):
        if any(s in n for s in skip): continue
        zmin,zmax=M.bounds[:,2]
        if zmax<lo or zmin>hi: continue
        V,F=trimesh.remesh.subdivide_to_size(M.vertices,M.faces,max_edge=0.003,max_iter=20)
        V=V[(V[:,2]>=lo)&(V[:,2]<=hi)]
        if len(V)==0: continue
        bb=np.array([V.min(0),V.max(0)])
        xs+= [bb[0,0],bb[1,0]]; ys+=[bb[0,1],bb[1,1]]; who.append((n,bb[0,0],bb[1,0],bb[0,1],bb[1,1]))
    return min(xs),max(xs),min(ys),max(ys),who
if __name__=='__main__':
 for name,u,p in (('dev',sys.argv[1],sys.argv[2]),('int',sys.argv[3],sys.argv[4])):
     for lo,hi in ((0.048,0.088),(0.0,0.45)):
         a=band(u,p,lo,hi)
         print(f'{name} z[{lo},{hi}] x[{a[0]:+.4f},{a[1]:+.4f}] y[{a[2]:+.4f},{a[3]:+.4f}]')
         if lo>0:
             w=sorted(a[4],key=lambda t:t[1])[:2]+sorted(a[4],key=lambda t:-t[2])[:2]
             for t in w: print('   ',t[0],' '.join(f'{v:+.4f}' for v in t[1:]))
