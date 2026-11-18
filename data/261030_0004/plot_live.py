"""실제 연속 연결 새 입력 시험의 저장 궤적·명령 시각화."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
p=BASE/'nav2_261030_live_gate_v3';rows=json.loads((p/'trajectory.json').read_text());t=np.array([v['t'] for v in rows]);x=np.array([v['x'] for v in rows]);y=np.array([v['y'] for v in rows]);raw=[v['raw_v'] if v['raw_v'] is not None else np.nan for v in rows];safe=[v['safe_v'] for v in rows]
fig,ax=plt.subplots(2,1,figsize=(14,8));ax[0].axhline(.5,color='black');ax[0].axhline(-.5,color='black');ax[0].plot(x,y,label='최종 명령으로 적분한 2D 모의 경로');ax[0].add_patch(Rectangle((1.025,-.30),.35,.6,fill=False,edgecolor='red',linestyle='--',label='완전 길막 2~26초 · 퇴장27초'));ax[0].scatter([.35,2.5],[0,0],color=['green','red']);ax[0].set_xlim(0,3);ax[0].set_ylim(-.65,.65);ax[0].set_xlabel('x (m)');ax[0].set_ylabel('y (m)');ax[0].legend(loc='upper left');ax[0].grid(alpha=.2)
ax[1].plot(t,raw,label='Nav2 원시 vx',alpha=.6);ax[1].plot(t,safe,label='최종 /cmd_vel_nav vx',lw=1.7)
for start,end,color,label in [(2,26,'red','길막'),(32,33,'gray','깊이 입력 단절'),(34,35,'orange','TFmini 정지 요청')]:ax[1].axvspan(start,end,alpha=.15,color=color,label=label)
ax[1].set_xlabel('시간 (s)');ax[1].set_ylabel('전진 속도 (m/s)');ax[1].legend(loc='upper right');ax[1].grid(alpha=.2)
fig.suptitle('실시간 깊이 → nvblox → Nav2 · 최종 게이트 회귀',fontsize=17);fig.text(.5,.025,'분석식 이동 카메라·가정 잡음 3mm·기준 자세 TF·2D 운동학 / 실물·Isaac·VO·제동 시험 아님',ha='center');fig.subplots_adjust(top=.9,bottom=.10,hspace=.35);fig.savefig(ROOT/'photo/261030_nvblox_Nav2_연속연결.png',dpi=140)
