"""PR #50 전(dev 261028) · 후(B안) URDF 를 y=0 단면으로 잘라 지면 · 1층 판 · TFmini 를 옆에서 그린다."""
import sys,os,numpy as np,trimesh,xml.etree.ElementTree as ET
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib import font_manager
for f in font_manager.findSystemFonts():
    if 'NotoSansCJK' in f or 'NotoSansKR' in f: font_manager.fontManager.addfont(f)
plt.rcParams['font.family']=['Noto Sans CJK KR','Noto Sans CJK JP','DejaVu Sans']; plt.rcParams['axes.unicode_minus']=False
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from fk import load,fk
from bounds import T_of
S=os.path.dirname(os.path.abspath(__file__))
G=0.0068  # base_footprint 이 땅보다 위인 높이(바퀴 중심 z 0.0262 − 반지름 0.033)
def meshes(urdf,pkg):
    J=load(urdf); r=ET.parse(urdf).getroot()
    for L in r.findall('link'):
        n=L.get('name'); _,Tl=fk(J,n)
        if any(s in n for s in ('arm','gripper','RMX','camera','wheel')): continue
        use='visual' if n!='base_link' else 'collision'
        for c in L.findall(use):
            g=c.find('geometry'); To=Tl@T_of(c.find('origin'))
            b,m,cy=g.find('box'),g.find('mesh'),g.find('cylinder')
            if b is not None: M=trimesh.creation.box([float(v) for v in b.get('size').split()])
            elif cy is not None: M=trimesh.creation.cylinder(float(cy.get('radius')),float(cy.get('length')),sections=48)
            elif m is not None:
                f=os.path.join(pkg,m.get('filename').split('://',1)[1].split('/',1)[1])
                if not f.endswith('.stl') and not f.endswith('.STL'): continue
                M=trimesh.load(f,force='mesh')
                if m.get('scale'): M.apply_scale([float(v) for v in m.get('scale').split()])
            else: continue
            M.apply_transform(To); yield n,M
def section(M,y0=0.0):
    V=M.vertices[M.faces]; d=V[:,:,1]-y0; segs=[]
    s=np.sign(d); mask=(s.max(1)>0)&(s.min(1)<0)
    for tri,dd in zip(V[mask],d[mask]):
        pts=[]
        for i,j in ((0,1),(1,2),(2,0)):
            if (dd[i]>0)!=(dd[j]>0):
                t=dd[i]/(dd[i]-dd[j]); p=tri[i]+t*(tri[j]-tri[i]); pts.append((p[0],p[2]+G))
        if len(pts)==2: segs.append(pts)
    return segs
COL={'base_link':'0.55','tfmini_bracket':'tab:orange','tfmini_link':'tab:red','lds_front':'tab:blue','lds_rear':'tab:blue'}
def panel(ax,urdf,pkg,title):
    J=load(urdf)
    for n,M in meshes(urdf,pkg):
        segs=section(M)
        if segs: ax.add_collection(LineCollection([[(x*1000,z*1000) for x,z in s] for s in segs],colors=COL.get(n,'tab:green' if 'opencr' in n else '0.55'),linewidths=0.8))
    for side in ('wheel_left_front_link','wheel_left_rear_link'):
        T=fk(J,side)[1]; ax.add_patch(plt.Circle((T[0,3]*1000,(T[2,3]+G)*1000),33,fill=False,ls='--',color='0.7',lw=0.8))
    ax.axhline(0,color='k',lw=1.5); ax.text(-245,-6,'지면 (z = 0)',va='top',fontsize=9)
    ax.axhline(G*1000,color='0.6',lw=0.6,ls=':'); ax.text(-245,G*1000+0.5,'base_footprint (+6.8)',fontsize=7,color='0.4')
    ax.axhspan(9.8,18.8,xmin=0,xmax=1,color='0.85',alpha=0.4,zorder=0)
    T=fk(J,'tfmini_link')[1]; x,z=T[0,3]*1000,(T[2,3]+G)*1000
    ax.plot(x,z,'o',color='tab:red',ms=5,zorder=5)
    ax.annotate(f'TFmini 가운데\n땅 위 {z:.1f} mm\n1층 판 윗면 위 {z-18.8:.1f} mm',(x,z),(x+35,z+28),fontsize=8,color='tab:red',arrowprops=dict(arrowstyle='->',color='tab:red'))
    Tl=fk(J,'lds_front')[1]; ax.plot(Tl[0,3]*1000,(Tl[2,3]+G)*1000,'s',color='tab:blue',ms=4)
    ax.text(Tl[0,3]*1000-5,(Tl[2,3]+G)*1000+4,'라이다 스캔면 82.3',fontsize=7,color='tab:blue',ha='right')
    ax.text(-130,11,'1층 판 (땅 위 9.8 ~ 18.8 mm)',fontsize=8,color='0.3')
    ax.set_title(title,fontsize=10); ax.set_xlim(-250,320); ax.set_ylim(-12,130); ax.set_aspect('equal'); ax.set_xlabel('x (mm, base_footprint, + = 앞)'); ax.set_ylabel('땅 위 높이 (mm)'); ax.grid(alpha=0.2)
    return z
fig=plt.figure(figsize=(12,8.5))
a1=fig.add_subplot(3,1,1); a2=fig.add_subplot(3,1,2); a3=fig.add_subplot(3,1,3)
z0=panel(a1,S+'/dev.urdf',S+'/dev/workspace/urdf_and_usd/waffle_4wd_description_261028','이전: dev 261028 (PR #50 전) — y = 0 단면')
z1=panel(a2,S+'/int.urdf',S+'/int/workspace/urdf_and_usd/waffle_4wd_description','이후: PR #50 B안 — y = 0 단면')
th=np.radians(1.8)
for z,c,lab in ((z0,'tab:gray','이전'),(z1,'tab:red','이후')):
    d=z/1000/np.tan(th)
    xs=np.array([0,d]); a3.plot(xs,[z,z],color=c,lw=1,ls=':'); a3.plot(xs,[z,0],color=c,lw=1.5); a3.plot(xs,[z,2*z],color=c,lw=0.8,alpha=0.5)
    a3.plot(d,0,'v',color=c); a3.annotate(f'{lab}: 땅 위 {z:.1f} mm → 빛 아래 끝이 약 {d:.2f} m 에서 땅에 닿음',(d,0),(d-0.55,z+18),fontsize=8,color=c,arrowprops=dict(arrowstyle='->',color=c))
a3.axhline(0,color='k',lw=1.5); a3.axvspan(0,0.25,color='tab:red',alpha=0.08); a3.text(0.01,3,'정지 0.25 m',fontsize=7)
a3.axvline(0.35,color='0.5',lw=0.6,ls='--'); a3.text(0.36,3,'해제 0.35 m',fontsize=7)
a3.set_xlim(0,1.5); a3.set_ylim(-5,95); a3.set_xlabel('TFmini 앞 거리 (m)'); a3.set_ylabel('높이 (mm, 세로 과장)')
a3.set_title('TFmini 빛 퍼짐 (전체 3.6°, 수평 장착 가정) — 땅에 닿기 시작하는 거리',fontsize=10); a3.grid(alpha=0.2)
fig.tight_layout(); out=sys.argv[1]; fig.savefig(out,dpi=140); print(out,z0,z1)
