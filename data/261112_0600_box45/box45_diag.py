"""box45.py 진단: 자리가 안 나는 대상에서 |θ| 구간별로 막히는 까닭 (stage 없음 / 보기 자리 · final 막힘), 가장 가까운 면 · 다음 면 각각.
    python3 -u box45_diag.py > box45_diag.txt
"""
import itertools
import math
import sys
from multiprocessing import Pool

import numpy as np

sys.argv = ["box45.py"]
src = open(__file__.replace("box45_diag.py", "box45.py")).read().split("targets = [")[0]
exec(src)
BINS = [(0, 5), (10, 15), (20, 25), (30, 35), (40, 45)]


def reason(sol, c, n, std):
    r = stage_for(sol, c, n, std)
    if r is None:
        return "stage"
    if r["look_clear"] <= 0:
        return "look"
    if r["final_clear"] <= 0:
        return "final"
    return "ok"


def diag(key):
    a = appr[key]
    c0 = np.array(P.START_BOXES[a.extra["box"]] if a.kind == "box" else P.PLACES[key]["center"], float)
    fdir = -np.array([math.cos(a.stage.yaw), math.sin(a.stage.yaw)])
    bs = box_solid(c0)
    out = [f"  {key} (계획 standoff {a.standoff:.2f})"]
    for lo, hi in BINS:
        cnt = {}
        n = 0
        for dx, dy, yd in itertools.product((-D, 0, D), (-D, 0, D), [y for y in YAWS if lo <= abs(y) <= hi]):
            c = c0 + np.array([dx, dy])
            th = math.radians(yd)
            sol = [s for s in solids if s is not bs] + [Solid("moved_box", c[0], c[1], bs.z0, bs.z1, sx=bs.sx, sy=bs.sy, yaw=bs.yaw + th)]
            fl = faces(c, th, fdir)
            r1 = reason(sol, c, fl[0][1], a.standoff)
            r2 = reason(sol, c, fl[1][1], a.standoff) if r1 != "ok" else "-"
            cnt[(r1, r2)] = cnt.get((r1, r2), 0) + 1
            n += 1
        ok = sum(v for (r1, r2), v in cnt.items() if "ok" in (r1, r2))
        out.append(f"    |θ| {lo:2d}~{hi:2d}°: 자리 남 {ok}/{n} · (가장 가까운 면, 다음 면) 까닭 " + ", ".join(f"{k[0]}/{k[1]} {v}" for k, v in sorted(cnt.items())))
    return "\n".join(out)


if __name__ == "__main__":
    keys = ["B2:S", "B3:S", "B3:N", "B4:S", "B4:N", "B6:W", "L3"]
    print(f"stage 여유 {STAGE_MARGIN} · standoff 하한 {STD_MIN}"); print("까닭: stage = standoff 0.45 ~ 0.30 어디도 외곽 + 0.05 가 비지 않음, look = 면 0.20 보기 자리 외곽이 닿음, final = 면 0.127 자리 외곽이 닿음")
    with Pool(len(keys)) as pool:
        for t in pool.map(diag, keys):
            print(t, flush=True)
