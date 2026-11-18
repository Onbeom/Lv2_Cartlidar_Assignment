from pathlib import Path
import json,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']=font.get_name()
R=Path('/home/pa4/git/lv2_next_module');O=R/'Cartlidar/sim_render/results/a1_code_261028'
a=json.loads((O/'final_v8_perception_comparison.json').read_text());labels=['원래 오차 조건\n접근·복귀','원래 오차 조건\n집기 전','보정 일치 대조\n집기 전'];names=['replay_inout_v8','replay_grasp_v8','replay_grasp_v8_calibrated'];values=[a[n]['P2']['rate']*100 for n in names]
fig,ax=plt.subplots(figsize=(10,6));bars=ax.bar(labels,values,color=['#2c7fb8' if v>=95 else '#d95f0e' for v in values]);ax.axhline(95,color='#555',linestyle='--',label='인지 중심오차 기준 충족률 문턱95%');ax.set_ylim(0,108);ax.set_ylabel('관측 가능 프레임의 기준 충족률(%)');ax.set_title('A1 인지 최종 후보 · 기존모델 모의 영상\n화면안1·실제보임≥0.5, 중심오차≤0.05 및 같은 목표일 때 정답')
for bar,name in zip(bars,names):
 v=a[name]['P2'];ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+1,f"{v['ok']}/{v['n']} = {v['rate']*100:.2f}%",ha='center')
ax.legend(loc='lower left');fig.text(.5,.015,'전체 팔/집기 성공 판정이 아님. 기존 J4 실패·계획/실제 목표 불일치는 원본 결과에 보존.',ha='center',fontsize=10);fig.tight_layout(rect=(0,.04,1,1));fig.savefig(R/'photo/261029_A1_인지최종후보_비교.png',dpi=150);fig.savefig(O/'final_v8_comparison.png',dpi=150)
