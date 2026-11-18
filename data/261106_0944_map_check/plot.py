"""세 자료 겹침 그림(발제 좌표). 왼쪽 전체, 오른쪽 북동 구석 확대."""
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
fm.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family'] = 'Noto Sans CJK JP'
D = np.load('pts.npz')
S = [('SDF', '#2a78d6', '발제 도면 SDF (Gazebo · 팀 Isaac)'), ('USD', '#eb6834', '우리 Isaac 미로 maze.usda'), ('slam', '#1baf7a', 'SLAM 지도 slam_walls_001 (dev 기본 /map)')]
fig, ax = plt.subplots(1, 2, figsize=(12.5, 8.2), facecolor='#fcfcfb', gridspec_kw={'width_ratios': [1, 1.05]})
for a in ax:
    a.set_facecolor('#fcfcfb')
    for k, c, l in S:
        a.scatter(D[k][:, 0], D[k][:, 1], s=1.4 if k != 'slam' else 2.4, c=c, label=l, linewidths=0)
    a.set_aspect('equal'); a.set_xlabel('x (m)', color='#52514e'); a.grid(color='#e4e3df', lw=.5)
    a.tick_params(colors='#52514e')
    for s in ('top', 'right'): a.spines[s].set_visible(False)
ax[0].set_ylabel('y (m)', color='#52514e'); ax[0].set_title('전체 (발제 좌표)', loc='left', color='#0b0b0b')
ax[1].set_xlim(3.3, 4.65); ax[1].set_ylim(5.3, 6.55); ax[1].set_title('북동 구석 확대: SDF 와 나머지 둘이 약 8~10 cm 다름', loc='left', color='#0b0b0b')
ax[0].plot([3.3, 4.65, 4.65, 3.3, 3.3], [5.3, 5.3, 6.55, 6.55, 5.3], c='#52514e', lw=.8)
h = [plt.Line2D([], [], ls='', marker='o', ms=6, c=c, label=l) for _, c, l in S]
fig.legend(handles=h, loc='upper center', ncol=3, frameon=False, labelcolor='#0b0b0b')
fig.tight_layout(rect=(0, 0, 1, .95)); fig.savefig('/home/pa4/git/lv2_next_module/photo/261106_미로지도비교.png', dpi=130)
