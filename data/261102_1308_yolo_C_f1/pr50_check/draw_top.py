"""두 라이다가 스캔면(땅 위 82.3 mm)에서 차체에 가리지 않고 보는 각도를 위에서 그린다(PR #50 전 · 후)."""
import sys,os,numpy as np,trimesh
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Wedge,Rectangle
from matplotlib import font_manager
for f in font_manager.findSystemFonts():
    if 'NotoSansCJK' in f: font_manager.fontManager.addfont(f)
plt.rcParams['font.family']=['Noto Sans CJK JP','DejaVu Sans']
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from band import geoms
from fk import load,fk
S=os.path.dirname(os.path.abspath(__file__))
def vis(urdf,pkg):
    J=load(urdf); P=[]
    for n,M in geoms(urdf,pkg):
        if n.startswith('lds') or any(s in n for s in ('arm','gripper','RMX','camera')): continue
        V,F=trimesh.remesh.subdivide_to_size(M.vertices,M.faces,max_edge=0.002,max_iter=30); P.append(V)
    P=np.vstack(P); out={}
    for lid in ('lds_front','lds_rear'):
        o=fk(J,lid)[1][:3,3]; Q=P[np.abs(P[:,2]-o[2])<0.003]
        d=Q[:,:2]-o[:2]; r=np.hypot(*d.T); a=np.degrees(np.arctan2(d[:,1],d[:,0]))%360
        b=np.zeros(720,bool); b[(a[(r>0.03)&(r<0.5)]*2).astype(int)%720]=True
        for k in range(720):
            if not b[k] and b[k-1] and b[(k+1)%720]: b[k]=True
        out[lid]=(o,~b)
    return out
fig,axs=plt.subplots(1,2,figsize=(12,6))
for ax,(tag,u,p) in zip(axs,(('이전 dev 261028',S+'/dev.urdf',S+'/dev/workspace/urdf_and_usd/waffle_4wd_description_261028'),('이후 PR #50 B안',S+'/int.urdf',S+'/int/workspace/urdf_and_usd/waffle_4wd_description'))):
    V=vis(u,p); R=0.6
    for lid,c in (('lds_front','tab:blue'),('lds_rear','tab:orange')):
        o,v=V[lid]; k=0
        while k<720:
            if v[k]:
                j=k
                while j<720 and v[j]: j+=1
                ax.add_patch(Wedge((o[0],o[1]),R,k/2,j/2,color=c,alpha=0.25,lw=0)); k=j
            else: k+=1
        ax.plot(o[0],o[1],'o',color=c)
        ax.text(o[0],o[1]+0.03,f"{'앞' if 'front' in lid else '뒤'} 라이다 {v.sum()/2:.0f}°",color=c,ha='center',fontsize=9)
    ax.add_patch(Rectangle((-0.2135 if 'PR' in tag else -0.226,-0.1676),(0.2720+0.2135) if 'PR' in tag else (0.2720+0.226),0.3352,fill=False,ec='k',lw=0.8))
    for y in (0.3,-0.3): ax.axhline(y,color='0.4',lw=0.8,ls='--')
    ax.text(0.45,0.31,'옆 벽 0.3 m',fontsize=8,color='0.3')
    ax.set_xlim(-0.65,0.65); ax.set_ylim(-0.65,0.65); ax.set_aspect('equal'); ax.grid(alpha=0.2)
    ax.set_title(f'{tag} — 스캔면에서 가리지 않는 방향(0.6 m까지)',fontsize=10); ax.set_xlabel('x (m, + = 앞)'); ax.set_ylabel('y (m, + = 왼쪽)')
fig.text(0.5,0.01,'파랑 = 앞 라이다, 주황 = 뒤 라이다가 보는 방향. 둘 다 칠해지지 않은 옆 흰 쐐기 = 두 라이다 모두 못 보는 곳. 검은 사각 = 차체 외곽. URDF 충돌 형상으로 계산(실물 아님).',ha='center',fontsize=9)
fig.tight_layout(rect=(0,0.03,1,0.96)); fig.savefig(sys.argv[1],dpi=130); print(sys.argv[1])
