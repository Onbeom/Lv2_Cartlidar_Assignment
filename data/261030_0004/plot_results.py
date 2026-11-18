"""저장된 실제 모의 출력에서 동적 지도와 Nav2 명령 그림을 생성."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
p=BASE/'dynamic_261030_holdout';fig,ax=plt.subplots(2,3,figsize=(18,9))
for j,(label,title) in enumerate([('warm','진입 전'),('hold','길막 중'),('clear','퇴장·재관측 후')]):
    image=np.load(p/(label+'_depth_m.npy'));im=ax[0,j].imshow(image,vmin=0,vmax=3,cmap='viridis');ax[0,j].set_title(title+' · 입력 깊이');ax[0,j].axis('off');fig.colorbar(im,ax=ax[0,j],label='m',fraction=.04)
    z=np.load(p/(label+'_combined.npz'));a=np.ma.array(z['distance'],mask=~z['known']);o=z['origin'];res=float(z['resolution']);h,w=a.shape;cmap=plt.colormaps['viridis'].copy();cmap.set_bad('lightgray');im=ax[1,j].imshow(a,origin='lower',extent=[o[0],o[0]+w*res,o[1],o[1]+h*res],vmin=0,vmax=.6,cmap=cmap,interpolation='nearest');ax[1,j].set_title(title+' · 결합 거리 지도');ax[1,j].set_xlim(0,2.5);ax[1,j].set_ylim(-.7,.7);ax[1,j].set_xlabel('x (m)');ax[1,j].set_ylabel('y (m)');fig.colorbar(im,ax=ax[1,j],label='m',fraction=.04)
fig.suptitle('nvblox 동적 장애물 · 새 입력 재확인',fontsize=18);fig.text(.5,.025,'분석식 깊이·고정 기준 자세·가정 잡음 3mm / 색=거리 / 사람 의미분류·실물 시험 아님',ha='center');fig.subplots_adjust(top=.9,bottom=.10,hspace=.25,wspace=.50);fig.savefig(ROOT/'photo/261030_동적장애물_진입퇴장.png',dpi=140);plt.close(fig)
p=BASE/'nav2_261030_design_v3';rows=json.loads((p/'trajectory.json').read_text());t=np.array([v['t'] for v in rows]);x=np.array([v['x'] for v in rows]);y=np.array([v['y'] for v in rows]);raw=[v['raw_v'] if v['raw_v'] is not None else np.nan for v in rows];safe=[v['safe_v'] for v in rows]
fig,ax=plt.subplots(2,1,figsize=(14,8));ax[0].axhline(.5,color='black');ax[0].axhline(-.5,color='black');ax[0].plot(x,y,label='최종 명령으로 적분한 2D 모의 경로');ax[0].add_patch(Rectangle((1.025,-.25),.35,.5,fill=False,edgecolor='red',linestyle='--',label='길막 위치 · t=2~10초만 존재'));ax[0].scatter([.35,2.5],[0,0],color=['green','red']);ax[0].set_xlim(0,3);ax[0].set_ylim(-.65,.65);ax[0].set_xlabel('x (m)');ax[0].set_ylabel('y (m)');ax[0].legend(loc='upper left');ax[0].grid(alpha=.2)
ax[1].plot(t,raw,label='Nav2 원시 vx',alpha=.6);ax[1].plot(t,safe,label='최종 /cmd_vel_nav vx',lw=1.7)
for start,end,color,label in [(2,10,'red','길막'),(16,17,'gray','관측 무효'),(18,19,'orange','TFmini 정지 요청')]:ax[1].axvspan(start,end,alpha=.15,color=color,label=label)
ax[1].set_xlabel('시간 (s)');ax[1].set_ylabel('전진 속도 (m/s)');ax[1].legend(loc='upper right');ax[1].grid(alpha=.2)
fig.suptitle('실제 Nav2 경로·속도 출력 → 정지·재개 연결 시험',fontsize=17);fig.text(.5,.025,'저장된 nvblox 지도 재생 + 2D 운동학 / 실제 모터·제동·연속 센서 주행 성능 아님',ha='center');fig.subplots_adjust(top=.9,bottom=.10,hspace=.35);fig.savefig(ROOT/'photo/261030_Nav2_정지재개_명령.png',dpi=140)
