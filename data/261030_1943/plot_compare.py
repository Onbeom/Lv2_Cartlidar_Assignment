"""새 maze 단면(z=0.2) 위에 이전 발제 도면 판(중심 맞춤)을 겹쳐 그림."""
import json
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
F = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(F)
import matplotlib.pyplot as plt
matplotlib.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
new = json.load(open('maze_slice_z0.20.json'))
old = json.load(open('/home/pa4/git/lv2_next_module/Cartlidar/sim_world/data/maze_dice_mission.json'))
ox, oy = 2.202, 3.152  # 이전 도면 중심(격자 0~4.404, 0~6.304)
fig, ax = plt.subplots(figsize=(8, 10.5))
for g, a, b in new['segments']:
    ax.plot([a[0], b[0]], [a[1], b[1]], '-', color='k', lw=1.2)
for p in old['panels']:
    cx, cy = p['center'][0] - ox, p['center'][1] - oy; h = p['length'] / 2
    xs, ys = ([cx - h, cx + h], [cy, cy]) if p['axis'] == 'x' else ([cx, cx], [cy - h, cy + h])
    ax.plot(xs, ys, '-', color='tab:red', lw=2.5, alpha=.45)
ax.plot([], [], 'k-', label='새 maze.zip (USD 충돌, z = 0.2 m 단면)')
ax.plot([], [], '-', color='tab:red', lw=2.5, alpha=.45, label='이전 발제 도면 maze_dice_mission.json (중심 맞춤)')
ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='upper center', bbox_to_anchor=(.5, -.06))
ax.set_xlabel('x (m, 새 USD 좌표)'); ax.set_ylabel('y (m)'); ax.set_title('새 미로와 이전 발제 맵 비교')
fig.tight_layout(); fig.savefig('/home/pa4/git/lv2_next_module/photo/261030_새미로_이전맵_비교.png', dpi=100)
