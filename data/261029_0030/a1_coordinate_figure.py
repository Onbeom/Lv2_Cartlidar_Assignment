from pathlib import Path
import json, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=font.get_name()
root=Path('/home/pa4/git/lv2_next_module');o=root/'Cartlidar/sim_render/results/a1_code_261028'
a=json.loads((o/'a1_coordinate_error_decomposition.json').read_text());r=next(x for x in a if x['scene']=='w1_grasp00_L0' and x['k']==0)
meta=json.loads((o/'render_mission_grasp/render_a1.json').read_text());sc=next(x for x in meta['scenes'] if x['name']==r['scene']);f=sc['arm_frames'][0]
fig,ax=plt.subplots(figsize=(10,8));ax.imshow(plt.imread(o/'render_mission_grasp'/f['image']))
for key,label,color,marker in [('estimated_projection','현재 추정 위치의 투영','#00d8ff','x'),('true_projection','실제 3D 중심의 투영','#ffe600','+'),('bbox_center','보이는 사각형의 중심','#ff60df','o')]:
 xy=np.array(r[key])*[320,240]+[320,240];ax.scatter(*xy,c=color,marker=marker,s=160,linewidths=2.5,label=f'{label} ({xy[0]:.1f}, {xy[1]:.1f})')
ax.set_title('A1 seed1 · 집기 전 첫 영상: 서로 다른 세 위치\n현재 위치 추정의 투영 오차 32.6px / 실제 중심과 보이는 중심 차이 18.5px',fontsize=13)
ax.set_xlabel('영상 가로 좌표(px)');ax.set_ylabel('영상 세로 좌표(px)');ax.legend(loc='upper left',fontsize=10)
fig.tight_layout();p=root/'photo/261029_A1_좌표오차_분리.png';fig.savefig(p,dpi=150);fig.savefig(o/'coordinate_error_figure.png',dpi=150)
plt.close(fig)
d=json.loads((o/'d435_extrinsic_diagnostic.json').read_text());d=[x for x in d if not x['scene'].startswith('w2_')];x=np.arange(len(d));v1=[np.linalg.norm(t['raw_error_mm']) for t in d];v2=[np.linalg.norm(t['calibrated_error_mm']) for t in d]
fig,ax=plt.subplots(figsize=(9,5));ax.bar(x-.18,v1,.36,label='명목 카메라 자세');ax.bar(x+.18,v2,.36,label='렌더 카메라 자세로 계산상 보정');ax.set_xticks(x,[t['scene'].split('_')[0].replace('w','seed ') for t in d]);ax.set_ylabel('퍽 3D 중심 위치 오차(mm)');ax.set_title('카메라 외부 파라미터 차이의 영향 · 계산상 진단\nseed2는 다른 퍽 선택 문제로 이 비교에서 제외; 정식 재생 통과를 뜻하지 않음');ax.legend()
for i,(a,b) in enumerate(zip(v1,v2)):
 ax.text(i-.18,a+.25,f'{a:.1f}',ha='center');ax.text(i+.18,b+.25,f'{b:.1f}',ha='center')
fig.tight_layout();fig.savefig(root/'photo/261029_A1_카메라보정_영향.png',dpi=150);fig.savefig(o/'extrinsic_effect_figure.png',dpi=150)
