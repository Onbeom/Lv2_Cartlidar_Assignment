"""인지용 URDF 좌표 변환. 루트/관절명을 모델에서 읽고 잘못된 옛 관절명을 거부. 구동 명령 없음."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

class ModelFrames:
    def __init__(self,path):
        self.path=Path(path);root=ET.parse(self.path).getroot();self.links={x.attrib['name'] for x in root.findall('link')};self.joints={};self.parents={}
        for j in root.findall('joint'):
            name=j.attrib['name'];parent=j.find('parent').attrib['link'];child=j.find('child').attrib['link']
            if child in self.parents:raise ValueError(('복수부모',child))
            if parent not in self.links or child not in self.links:raise ValueError(('알수없는링크',name))
            o=j.find('origin');xyz=np.array([float(x) for x in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]);rpy=[float(x) for x in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
            r,p,y=rpy;cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
            R=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
            T=np.eye(4);T[:3,:3]=R;T[:3,3]=xyz;a=j.find('axis');axis=np.array([float(x) for x in (a.get('xyz','1 0 0') if a is not None else '1 0 0').split()]);self.parents[child]=name;self.joints[name]=dict(parent=parent,child=child,T=T,axis=axis,kind=j.attrib['type'])
        roots=self.links-set(self.parents)
        if len(roots)!=1:raise ValueError(('루트 수',roots))
        self.root=next(iter(roots))
    def poses(self,q):
        unknown=set(q)-set(self.joints)
        if unknown:raise ValueError(('모델에없는관절명',sorted(unknown)))
        result={self.root:np.eye(4)}
        def pose(link,stack):
            if link in result:return result[link]
            if link in stack:raise ValueError(('순환링크',link))
            j=self.joints[self.parents[link]];T=j['T'].copy();v=q.get(self.parents[link],0.);a=j['axis'];M=np.eye(4)
            if j['kind'] in ('revolute','continuous'):
                a=a/np.linalg.norm(a);x,y,z=a;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);M[:3,:3]=np.eye(3)+np.sin(v)*K+(1-np.cos(v))*(K@K)
            elif j['kind']=='prismatic':M[:3,3]=v*a
            result[link]=pose(j['parent'],stack|{link})@T@M;return result[link]
        for link in self.links:pose(link,set())
        return result
    def transform(self,source,target,q):
        p=self.poses(q);return np.linalg.inv(p[source])@p[target]
