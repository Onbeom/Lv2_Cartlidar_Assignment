from pitch_ik import *
from td_plot import pts
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"]=["NanumGothic","DejaVu Sans"]
fig,axs=plt.subplots(1,2,figsize=(12,5))
res={}
for n,x,y,wt in SL:
    r=solve(x,y,wt,0)
    if r: res[n]=r[1]; print(n, np.round(r[1],2), "여유", round(r[0],2))
for ax,names,title in ((axs[0],["3-0","2-0","1-0"],"정면 (관절1 을 컵 쪽으로)"),(axs[1],["2-1","2-0","1-1","1-0"],"어깨 너머 (관절1 앞쪽, 팔 뒤로 젖힘)")):
    for n in names:
        if n not in res: continue
        q=res[n]; P=pts(q)
        if title.startswith("정면") and abs(q[0])<1.5: continue
        if title.startswith("어깨") and abs(q[0])>1.5: continue
        # 그림 평면: x (앞 +) - z, 옆에서 봄
        ax.plot(P[:,0],P[:,2],"-o",lw=2.5,ms=4,label=f"{n}: q {np.round(q[:4],2)}")
    for n,x,y,wt in SL: ax.add_patch(plt.Rectangle((x-0.016,wt-0.03),0.032,0.03,color="royalblue",alpha=0.25))
    ax.plot([0.096],[0.131],"ks"); ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_title(title); ax.set_xlabel("x 앞쪽 + [m] (base_link)"); ax.legend(fontsize=7,loc="upper left"); ax.set_ylim(0.08,0.42)
plt.tight_layout(); plt.savefig("p0.png",dpi=100)
