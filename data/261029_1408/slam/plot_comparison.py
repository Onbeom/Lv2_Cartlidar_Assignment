"""실제 기록한 모의 SLAM 비교 결과를 그림으로 저장한다."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
from PIL import Image
root=Path(__file__).resolve().parents[2];base=root/'Cartlidar/sim_render/results/followup_261028_1457';d=json.loads((base/'slam_coupled_comparison.json').read_text());font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(2,2,figsize=(12,10))
for i,row in enumerate(d['rows']):
 a=axes[i,0];truth=np.array(row['coupled']['traj']['ref']);a.plot(truth[:,0],truth[:,1],color='black',label='기준 궤적',linewidth=2)
 for key,label,color in [('baseline_carto','기존 Cartographer','royalblue'),('coupled','cuVSLAM odom + Cartographer','darkorange')]:
  t=np.array(row[key]['traj']['est']);a.plot(t[:,0],t[:,1],label=label,color=color,linewidth=1)
 a.set_title(('밝음' if row['light']=='bright' else '어두움')+' · 같은 모의 입력',fontproperties=font);a.set_xlabel('x (m)');a.set_ylabel('y (m)');a.axis('equal');a.grid(alpha=.25);a.legend(prop=font,fontsize=8)
 b=axes[i,1];values=[row['baseline_carto']['A1_rmse_m']*100,row['coupled']['A1_rmse_m']*100];b.bar([0,1],values,color=['royalblue','darkorange']);b.set_xticks([0,1],['기존 Cartographer','cuVSLAM 결합'],fontproperties=font);b.set_ylabel('위치 RMSE (cm)',fontproperties=font);b.axhline(5,color='red',linestyle='--',label='사전 기준 5 cm');b.legend(prop=font)
 for j,v in enumerate(values):b.text(j,v,f'{v:.2f} cm',ha='center',va='bottom')
 b.set_ylim(0,max(values)*1.2);b.grid(axis='y',alpha=.2)
fig.suptitle('기존 모델·옛 맵의 모의 SLAM 비교 — 현재 결합은 운용 기준 미달',fontproperties=font,fontsize=15);fig.text(.5,.015,'초기 자세 기준으로 비교 · 궤적 전체 맞춤 없음 · 실물/새 모델 성능 아님',ha='center',fontproperties=font);fig.tight_layout(rect=[0,.04,1,.96]);fig.savefig(root/'photo/261029_SLAM_결합비교.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(10,10))
for i,row in enumerate(d['rows']):
 rec=root/'261027_0953/results'/('m1711_bright_1_261027_203413' if row['light']=='bright' else 'm1711_dark_1_261027_203822')
 paths=[next(rec.glob('carto_*'))/'map.pgm',base/('slam_coupled_'+row['light'])/'map.pgm']
 for j,p in enumerate(paths):
  axes[i,j].imshow(Image.open(p),cmap='gray',vmin=0,vmax=255);axes[i,j].axis('off');axes[i,j].set_title(('밝음' if i==0 else '어두움')+' · '+('기존 Cartographer' if j==0 else 'cuVSLAM 결합'),fontproperties=font)
fig.suptitle('실제로 저장한 점유 지도 · 각 지도 원점/방향은 독립',fontproperties=font,fontsize=14);fig.text(.5,.025,'기존 모델·옛 맵 모의 결과 / 실물 주행용 지도 확정본 아님',ha='center',fontproperties=font);fig.tight_layout(rect=[0,.05,1,.95]);fig.savefig(root/'photo/261029_SLAM_점유지도_비교.png',dpi=150)
