import sys, numpy as np, xml.etree.ElementTree as ET
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def load(f):
    t=ET.parse(f).getroot(); J={}
    for j in t.findall('joint'):
        o=j.find('origin'); xyz=[float(v) for v in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]
        r=[float(v) for v in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
        T=np.eye(4); T[:3,:3]=rpy(*r); T[:3,3]=xyz
        J[j.find('child').get('link')]=(j.find('parent').get('link'),T,j.get('type'),j.get('name'))
    return J
def fk(J,l):
    T=np.eye(4)
    while l in J: p,Tj,_,_=J[l]; T=Tj@T; l=p
    return l,T
if __name__=="__main__":
  A,B=load(sys.argv[1]),load(sys.argv[2])
  keys=sorted(set(A)|set(B))
  for k in keys:
      if not any(s in k for s in ('camera','lds','scan','tfmini','imu','wheel','optical','laser','range','base_link','pallet','puck','gripper','end')): continue
      a=fk(A,k)[1] if k in A else None; b=fk(B,k)[1] if k in B else None
      fmt=lambda T: 'none' if T is None else ' '.join(f'{v:+.4f}' for v in T[:3,3])+' rpy?'+('' )
      d='' if a is None or b is None else f'dpos {np.linalg.norm(a[:3,3]-b[:3,3])*1000:6.1f}mm drot {np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1))):5.2f}deg'
      print(f'{k:40s} dev[{fmt(a)}] int[{fmt(b)}] {d}')
