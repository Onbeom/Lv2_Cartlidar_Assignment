#!/usr/bin/env python3
"""모의: 태그 배치(새 100 mm 6 장 · 과제 큰 태그 3 장 · 큰 3 + 작은 2)별 장착 각 되찾기 정확도. layout_compare.py [반복=3]"""
import math, os, sys, numpy as np
import front_cal as F
REP = int(sys.argv[1]) if len(sys.argv) > 1 else 3
LAY = {
    'A 새 100mm x6': [(20 + i, x, s * 0.11, 0.10) for i, (s, x) in enumerate([(s, x) for s in (1, -1) for x in (0.40, 0.65, 0.90)])],
    'B 큰 48mm x3': [(12, 0.45, 0.13, 0.048), (13, 0.85, 0.13, 0.048), (14, 0.65, -0.13, 0.048)],
    'C 큰 x3 + 작은 16mm x2': [(12, 0.45, 0.13, 0.048), (13, 0.85, 0.13, 0.048), (14, 0.65, -0.13, 0.048), (0, 0.38, -0.13, 0.016), (1, 0.50, -0.13, 0.016)],
    'D 큰 x3 + 작은 x4(양쪽)': [(12, 0.45, 0.13, 0.048), (13, 0.85, 0.13, 0.048), (14, 0.65, -0.13, 0.048), (0, 0.38, -0.13, 0.016), (1, 0.50, -0.13, 0.016), (2, 0.36, 0.13, 0.016), (3, 0.58, 0.13, 0.016)],
}
cases = [(0.5, -0.8, 1.2), (-0.3, 0.5, -0.9), (0.0, 0.0, 0.7)]
Tn, _ = F.T_bf_opt(F.URDF_XYZ, F.URDF_RPY, F.DEF_LINK_OPT); c = Tn[:3, 3]
for name, lay in LAY.items():
    err = {'roll': [], 'pitch': [], 'yaw': []}; miss = 0
    for r in range(REP):
        for ci, (ro, pi, ya) in enumerate(cases):
            RD = F.rpy2R(*[math.radians(a) for a in (ro, pi, ya)])
            D = F.T(RD, c - RD @ c)
            f = f'lay_tmp/{name[0]}_{r}_{ci}'
            F.render_scene(f, D, n=10, seed=100 * r + ci, layout=lay, place_err=0.002)
            res = F.solve_one(f, 0.10, F.URDF_XYZ, F.URDF_RPY)
            if "error" in res: miss += 1; print("  ", f, res["error"], res.get("counts", [])[:5]); continue
            e = res['err_deg']
            for k, t in zip(('roll', 'pitch', 'yaw'), (ro, pi, ya)): err[k].append(abs(e[k] - t))
            miss += len(lay) - len(res['tags'])
    print(f"{name}: " + ', '.join(f"{k} 중앙 {np.median(v):.2f} 최대 {max(v):.2f}°" for k, v in err.items() if v) + f" | 못 찾은 태그 {miss}")
