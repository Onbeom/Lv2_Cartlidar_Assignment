"""모의 추적 비교의 출력 ID·보류를 사진으로 남긴다."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT=Path('/home/pa4/git/lv2_next_module')
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
items=[('최근접 3D',BASE/'tracker_v2_holdout/holdout_nearest3d_rows.json'),
       ('RGB-D KF + 태그 제약 v2',BASE/'tracker_v2_holdout/holdout_kf_hungarian_tag_rows.json'),
       ('ByteTrack · 측정점의 2D 투영',BASE/'bytetrack_rows.json')]
for ax,(title,path) in zip(axes,items):
    rows=json.loads(path.read_text())
    for label,color in zip(['a','b','c'],['#d34b4b','#3478c5','#3d945b']):
        selected=[r for r in rows if r['truth_for_evaluation']==label]
        times=[r['stamp'] for r in selected]
        ids=[(r['output'].get('track_id',np.nan) if r['output']['valid'] else np.nan) if 'output' in r else (r['track_id'] if r['track_id'] is not None else np.nan) for r in selected]
        ax.plot(times,ids,'.-',lw=.8,ms=3,label={'a':'물체 A','b':'물체 B','c':'물체 C'}[label],color=color)
    ax.set_title(title,loc='left');ax.set_ylabel('출력 track_id');ax.grid(alpha=.2)
    ax.axvspan(2,3.2,color='grey',alpha=.15);ax.axvspan(5.5,7.7,color='orange',alpha=.10)
    ax.axvline(8,color='black',ls='--',lw=.8)
axes[0].legend(loc='upper left',ncol=3)
axes[-1].set_xlabel('모의 시간 (초) · 빈 구간은 보류 또는 미출력 · 점선은 태그 재관측')
fig.suptitle('모의 3물체 교차·가림 비교 — 실제 YOLO/실물 성능 아님',fontsize=15)
fig.text(.5,.015,'같은 측정점 사용. 후보는 3D·태그를 추가로 사용하므로 정보량이 다름.\nID 교체: 최근접 5 / 후보 0 / ByteTrack 62 · 유효 출력률: 100% / 80.1% / 76.7%',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.07,1,.95])
target=ROOT/'photo/261030_추적기_모의비교.png';fig.savefig(target,dpi=150);plt.close(fig)
print(target)
