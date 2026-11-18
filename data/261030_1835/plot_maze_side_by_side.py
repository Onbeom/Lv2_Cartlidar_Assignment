"""이전 발제 도면과 새 maze.zip 벽을 발제 좌표(왼쪽 아래 기둥 중심 원점)로 나란히 그림.
물체(시작 박스 · 퍽 · 출발 · 신호등 · 바코드)는 사용자 확인(2026-10-30)대로 두 그림 모두 같은 발제 좌표.
새 미로 → 발제 변환은 YOLO 1030 1 측정값: 발제 = USD + (2.2345, 3.2136), 회전 0 (261030_1943/README.md)."""
import json
import math
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
from matplotlib.patches import Rectangle, Circle, FancyArrow
F = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(F)
import matplotlib.pyplot as plt
matplotlib.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()

ROOT = '/home/pa4/git/lv2_next_module'
old = json.load(open(f'{ROOT}/Cartlidar/sim_world/data/maze_dice_mission.json'))
new = json.load(open(f'{ROOT}/261030_1943/maze_slice_z0.20.json'))
DX, DY = 2.2345, 3.2136
BOXES = ((1.468, .950), (3.670, 3.152), (2.936, 5.354), (.734, 5.354), (2.202, 5.354), (3.670, 3.886))  # perception_survey core.py


def objects(ax):
    for i, (x, y) in enumerate(BOXES, 1):
        ax.add_patch(Rectangle((x - .1025, y - .1025), .205, .205, fc='#e8d5b0', ec='#8a6d3b', lw=1))
        for s in (-1, 1):
            ax.add_patch(Circle((x + s * .04, y), .015, fc='tab:red' if s < 0 else 'tab:green', ec='none'))
        ax.text(x, y + .14, f'박스{i}', ha='center', va='bottom', fontsize=8)
    ax.add_patch(FancyArrow(.734, .950, -.25, 0, width=.03, head_width=.1, head_length=.08, color='tab:blue'))
    ax.text(.734, .80, '출발 (734, 950)', ha='center', va='top', fontsize=8, color='tab:blue')
    ax.plot(.080, .950, 's', color='orange', ms=8)
    ax.text(.13, 1.02, '신호등 (80, 950)', fontsize=8, color='darkorange')
    ax.plot(.080, .650, 'D', color='purple', ms=6)
    ax.text(.13, .58, '바코드 (80, 650) 근처 벽', fontsize=8, color='purple')


fig, axes = plt.subplots(1, 2, figsize=(13, 9.6), sharex=True, sharey=True)
ax = axes[0]
for p in old['panels']:
    cx, cy = p['center']
    h = p['length'] / 2
    xs, ys = ([cx - h, cx + h], [cy, cy]) if p['axis'] == 'x' else ([cx, cx], [cy - h, cy + h])
    ax.plot(xs, ys, '-', color='tab:red', lw=2.5)
ax.set_title('이전 발제 도면 (maze_dice_mission.json)\n기둥 간격 4.404 × 6.304 m')
ax = axes[1]
for g, a, b in new['segments']:
    ax.plot([a[0] + DX, b[0] + DX], [a[1] + DY, b[1] + DY], '-', color='k', lw=1.2)
ax.set_title('새 maze.zip (USD 충돌 z = 0.2 m 단면, 발제 원점으로 옮김)\n기둥 간격 약 4.475 × 6.37~6.40 m')
for ax in axes:
    objects(ax)
    ax.plot(0, 0, 'k+', ms=12, mew=2)
    ax.set_aspect('equal')
    ax.grid(alpha=.3)
    ax.set_xlim(-.3, 4.8)
    ax.set_ylim(-.3, 6.7)
    ax.set_xlabel('x (m, 발제 좌표)')
axes[0].set_ylabel('y (m, 발제 좌표)')
fig.suptitle('이전 맵과 새 미로 — 같은 발제 좌표, 물체 위치는 같음(사용자 확인 2026-10-30)', fontsize=13)
fig.tight_layout()
fig.savefig(f'{ROOT}/photo/261030_이전맵_새미로_나란히.png', dpi=100)
