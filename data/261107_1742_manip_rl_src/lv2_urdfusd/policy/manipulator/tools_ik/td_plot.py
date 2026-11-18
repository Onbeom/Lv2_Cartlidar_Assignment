from td_ik import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"]=["NanumGothic","Noto Sans CJK KR","DejaVu Sans"]
def pts(q):
    P=[np.array([0.096,0,0.131])]; R=Rz(q[0])
    for off,rot in (([0,0,0.0595],Ry(q[1])),([0.024,0,0.128],Ry(q[2])),([0.124,0,0],Ry(q[3])),([0.028,0,0],Rx(q[4]))):
        P.append(P[-1]+R@off); R=R@rot
    P.append(P[-1]+R@[0.14,0,0]); return np.array(P)
cases=[("지금 정책 (옮기기 끝, 관절3 = 1.6 완전히 굽힘)",[3.0,-0.95,1.6,-0.65,0],"gray"),
       ("옆에서 쥠, 높게 (잡는 점 입구+120)",[3.0,-1.07,1.09,-0.02,0],"tab:blue"),
       ("위에서 (사진처럼) G_2 넣기 끝",[2.95,-0.02,-0.22,1.81,0],"tab:red"),
       ("위에서 B_2 넣기 끝 (집게 4° 기울임)",[3.0,0.32,-0.67,1.86,0],"tab:orange")]
fig,ax=plt.subplots(figsize=(8,6))
for name,q,col in cases:
    P=pts(np.array(q)); r=np.hypot(P[:,0]-0.096,P[:,1])*np.sign(np.cos(q[0]))*-1*-1
    r=-(P[:,0]-0.096)*np.cos(q[0])-P[:,1]*np.sin(q[0]); r=-r
    ax.plot(r,P[:,2],"-o",color=col,label=name,lw=2.5,ms=4)
for k,c in CUPS.items():
    d=math.hypot(c[0]-0.096,c[1]); ax.add_patch(plt.Rectangle((d-0.016,0.135),0.032,0.015,color="royalblue",alpha=0.3))
ax.axhline(0.15,ls=":",c="k",lw=0.8); ax.text(0.0,0.152,"컵 입구",fontsize=8)
ax.set_aspect("equal"); ax.set_xlabel("관절1 축에서 컵 쪽 거리 [m]"); ax.set_ylabel("높이 (base_link) [m]"); ax.legend(fontsize=8,loc="upper left"); ax.grid(alpha=0.3)
ax.set_title("옆에서 본 팔 자세 (관절1 을 컵 쪽으로 돌린 평면)")
plt.tight_layout(); plt.savefig("td_pose.png",dpi=110)
