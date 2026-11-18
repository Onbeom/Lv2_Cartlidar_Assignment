#!/usr/bin/env python3
"""'어느 자세에서든 STOW 로 접기' 의 중간 자세 (FOLD_UP) 후보: J1 을 돌려도 차체 · 팔레트 · 퍽에서 먼 자세 찾기 (계산만, 설계 URDF)."""
import itertools, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fold_clearance as fc

m = fc.Model(os.path.join(fc.DESC, 'urdf/waffle_4wd_loaded.urdf'))
obs, tags = m.obstacle_points('base_link', with_pucks=True)
boxes = m.arm_boxes()
from scipy.spatial import cKDTree
tree = cKDTree(obs)

def clearance(q6):
    qm = fc.qmap(q6)
    best = 1e9
    for (ln, To, half) in boxes:
        Tw = m.chain_T(ln, qm) @ To
        idx = tree.query_ball_point(Tw[:3, 3], float(np.linalg.norm(half)) + 0.05)
        if idx:
            best = min(best, float(fc.box_sdf(Tw, half, obs[np.asarray(idx)]).min()))
    return min(best, 0.05)   # 5 cm 넘으면 5 cm 로 (멀다)

res = []
out = open(sys.argv[1], 'w') if len(sys.argv) > 1 else sys.stdout
for j2, j3, j4 in itertools.product(np.linspace(-1.6, 0.8, 7), np.linspace(-1.6, 1.0, 7), np.linspace(-1.6, 1.6, 7)):
    worst = 1e9
    for j1 in np.linspace(-np.pi, np.pi, 13):
        worst = min(worst, clearance([j1, j2, j3, j4, 0.0, 0.923]))
        if worst < 0.02:
            break   # 2 cm 안이면 후보 아님 — 더 안 봄
    res.append((worst, j2, j3, j4))
    out.write(f'{worst*1000:+.1f} {j2:+.2f} {j3:+.2f} {j4:+.2f}\n'); out.flush()
res.sort(reverse=True)
for r in res[:12]:
    print(f'J1 한 바퀴 내내 최소 틈 {r[0]*1000:+.1f} mm  J2 {r[1]:+.2f} J3 {r[2]:+.2f} J4 {r[3]:+.2f}')
