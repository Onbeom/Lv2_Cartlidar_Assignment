#!/usr/bin/env python3
"""'어느 자세에서 켜도 STOW 로 접기' 경로 확인 (계산만, 설계 URDF waffle_4wd_loaded = 차체 · 팔레트 v5 · 퍽 10 개).
시작 자세: 관절 한계 안 무작위 중 그 자리 자체가 닿지 않는 것 (실제로 팔을 둘 수 있는 자리). 경로 = manipulator startup.fold_path 와 같은 순서.
판정 (계산 전에 정함): 경로 내내 최소 틈 ≥ 5 mm 인 시작 자세 비율. 출력: 비율, 실패 사례 (어느 단계 · 어느 부품).
쓰기: fold_any_check.py --up J2 J3 J4 [--n 400] [--seed 1]
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import fold_clearance as fc
from scipy.spatial import cKDTree

STOW = [2.200, -2.088, 1.549, -1.868, 0.0, 0.923]
LO = [-np.pi, -2.14, -1.6, -1.92, -2.8, -0.7946]
HI = [np.pi, 1.6, 1.6, 2.0, 2.8, 0.9541]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--up', type=float, nargs=3, required=True)
    ap.add_argument('--n', type=int, default=400)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--margin', type=float, default=0.005)
    ap.add_argument('--order', default='234U,56S,1S,4S,2S,3S',
                    help='단계: 관절 번호 + S(STOW) 또는 U(FOLD_UP), 쉼표로 (startup.fold_path 와 같은 순서)')
    a = ap.parse_args()
    m = fc.Model(os.path.join(fc.DESC, 'urdf/waffle_4wd_loaded.urdf'))
    obs, tags = m.obstacle_points('base_link', with_pucks=True)
    boxes = m.arm_boxes()
    tree = cKDTree(obs)

    def clr(q6):
        qm = fc.qmap(q6)
        best = (1e9, '', '')
        for (ln, To, half) in boxes:
            Tw = m.chain_T(ln, qm) @ To
            idx = tree.query_ball_point(Tw[:3, 3], float(np.linalg.norm(half)) + 0.03)
            if idx:
                idx = np.asarray(idx)
                d = fc.box_sdf(Tw, half, obs[idx])
                k = int(np.argmin(d))
                if d[k] < best[0]:
                    best = (float(d[k]), ln, str(tags[idx[k]]))
        return best

    up = [0.0, a.up[0], a.up[1], a.up[2], 0.0, 0.923]
    # startup.fold_path 기본: 2,3,4:FOLD_UP → 5,6:STOW → 1:STOW → 4:STOW → 2:STOW → 3:STOW
    steps = [([int(c) - 1 for c in tok[:-1]], STOW if tok[-1] == 'S' else up) for tok in a.order.split(',')]
    rng = np.random.default_rng(a.seed)
    ok = tried = near = 0
    fails = {}
    worst_all = []
    while tried < a.n:
        q = rng.uniform(LO, HI)
        q[5] = 0.923
        c0 = clr(q)[0]
        if c0 < 0:
            continue   # 그 자리에 팔을 둘 수 없음 (이미 닿음)
        tried += 1
        near += c0 < a.margin   # 출발부터 5 mm 안 (어느 경로든 실패로 셈)
        cur = q.copy()
        worst = (1e9, '', '', '')
        for joints, pose in steps:
            tgt = cur.copy()
            for j in joints:
                tgt[j] = pose[j]
            n = max(2, int(np.abs(tgt - cur).max() * 40))
            for s in np.linspace(0, 1, n)[1:]:
                d, ln, tg = clr(cur + (tgt - cur) * s)
                if d < worst[0]:
                    worst = (d, ln, tg, 'J' + '·'.join(str(j + 1) for j in joints))
            cur = tgt
        worst_all.append(worst[0])
        if worst[0] >= a.margin:
            ok += 1
        else:
            key = f'{worst[3]} {worst[1]}↔{worst[2].split("_")[0]}'
            fails[key] = fails.get(key, 0) + 1
    w = np.array(worst_all)
    print(f'순서 {a.order} · FOLD_UP J2 {a.up[0]:+.2f} J3 {a.up[1]:+.2f} J4 {a.up[2]:+.2f}: 시작 {tried} 곳 (출발부터 5 mm 안 {near}) 중 틈 ≥ {a.margin*1000:.0f} mm {ok} 곳 ({100*ok/tried:.1f} %, 출발 5 mm 밖만 {100*ok/max(1, tried-near):.1f} %), '
          f'최소 틈 중앙 {np.median(w)*1000:.1f} mm · 하위 5 % {np.percentile(w, 5)*1000:.1f} mm')
    for k, v in sorted(fails.items(), key=lambda x: -x[1])[:8]:
        print(f'  실패 {v:3d}  {k}')


if __name__ == '__main__':
    main()
