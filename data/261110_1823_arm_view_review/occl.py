"""앞 D435 컬러 광학 원점 → 퍽 선분이 팔 충돌 상자(OBB)를 지나는지. 사용: occl.py urdf j1..j5"""
import sys, xml.etree.ElementTree as ET, numpy as np
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def tf(x,R): T=np.eye(4);T[:3,:3]=R;T[:3,3]=x;return T
def axr(a,q):
    a=np.asarray(a,float);a/=np.linalg.norm(a);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]]);return np.eye(3)+np.sin(q)*K+(1-np.cos(q))*K@K
def org(e):
    o=e.find('origin');g=lambda k,d:[float(v) for v in (o.get(k,d) if o is not None else d).split()]
    return tf(g('xyz','0 0 0'),rpy(*g('rpy','0 0 0')))
root=ET.parse(sys.argv[1]).getroot(); q=[float(v) for v in sys.argv[2:]]
J={j.find('child').get('link'):j for j in root.findall('joint')}
arm=sorted([j for j in root.findall('joint') if j.get('type') in('revolute','continuous') and j.get('name').startswith('joint')],key=lambda j:j.get('name'))
qm={j.get('name'):(q[i] if i<len(q) else 0) for i,j in enumerate(arm)}
C={'base_link':np.eye(4)}
def T(l):
    if l in C: return C[l]
    j=J[l];M=T(j.find('parent').get('link'))@org(j)
    if j.get('type') in('revolute','continuous'): M=M@tf([0,0,0],axr([float(v) for v in j.find('axis').get('xyz').split()],qm.get(j.get('name'),0)))
    C[l]=M;return M
# base_footprint → base_link 높이
bf=0.00674998168  # base_link → base_footprint z −0.00675 (URDF)
cam=T('top_camera_front_color_optical_frame')[:3,3]; print('D435 컬러 광학 원점 base',np.round(cam,3))
# 팔 쪽 링크(관절 사슬 위) 충돌 상자
armlinks=set()
for j in arm:
    armlinks.add(j.find('child').get('link'))
# 자식 링크 전부 (fixed 포함)
changed=True
while changed:
    changed=False
    for c,j in J.items():
        if j.find('parent').get('link') in armlinks and c not in armlinks: armlinks.add(c);changed=True
boxes=[]
for l in root.findall('link'):
    n=l.get('name')
    if n not in armlinks: continue
    for c in l.findall('collision'):
        b=c.find('geometry').find('box')
        if b is None: continue
        M=T(n)@org(c); boxes.append((n,M,np.array([float(v) for v in b.get('size').split()])/2))
def seg_obb(p0,p1,M,h):
    Ri=M[:3,:3].T; a=Ri@(p0-M[:3,3]); b=Ri@(p1-M[:3,3]); d=b-a; t0,t1=0.0,1.0
    for k in range(3):
        if abs(d[k])<1e-12:
            if abs(a[k])>h[k]: return False
        else:
            u=(-h[k]-a[k])/d[k]; v=(h[k]-a[k])/d[k]; u,v=min(u,v),max(u,v); t0=max(t0,u); t1=min(t1,v)
            if t0>t1: return False
    return True
FRONT,FACE2PUCK=0.1328,0.1025
for face in (0.45,0.40,0.3475,0.33,0.31,0.30,0.20,0.127):
    x=FRONT+face+FACE2PUCK
    for y in (-0.04,0.04):
        for zf,lab in ((0.13,'윗면'),(0.10,'중간')):
            p=np.array([x,y,zf-bf]); hits=[n for n,M,h in boxes if seg_obb(cam,p,M,h)]
            ang=np.degrees(np.arctan2(cam[2]-p[2],np.hypot(*(p-cam)[:2])))
            print(f'면 {face:.3f} 퍽 y {y:+.2f} {lab}: 내려보는 각 {ang:4.1f}°  가림 {hits if hits else "없음"}')
