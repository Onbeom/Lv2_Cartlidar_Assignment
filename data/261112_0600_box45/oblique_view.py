"""비스듬히 서기: β (로봇 기준 상자 요) 가 클 때 place · 9 칸 · 태그 · 관절 · 차체 · 실은 퍽을 모두 지키는 보기 (서는 틈 g, box_look 거리 · 각) 찾기,
그리고 놓기 자리 틈 범위 (J1 → 9 칸 0.12 ~ 0.35). 계산, oblique.py 와 같은 판정 값.
    ~/isaacsim-venv/bin/python -u oblique_view.py > oblique_view.txt
"""
import itertools
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
U3 = HERE.parent / "261112_0130_u3_render"
ns = {"__name__": "x", "__file__": str(U3 / "approach_resid.py")}
src = open(U3 / "approach_resid.py").read().split("\nrng = np.random.default_rng(0)\nprint(")[0]
sys.argv = ["approach_resid.py", "--dist", "0.40", "--ang", "50"]
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(src, ns)
sys.stdout = _o
sys.path.insert(0, str(U3))
os.chdir(U3)
import cand_eval as CE  # noqa: E402

FRONT, HALF, J1_X, CELL = 0.1328, 0.1025, 0.096, 0.06
E = 0.018 / math.sqrt(2)
DIRS = [(0, 0)] + list(itertools.product((-1, 1), repeat=2))


def evaluate(g, dist, ang, betas):
    ns["a"].dist, ns["a"].ang = dist, ang
    bc = np.array([FRONT + g + HALF, 0.0])
    w = dict(place=1e9, cells=1e9, tag=1e9, jm=1e9)
    inbox = 0
    qs = []
    for sx, sy in DIRS:
        q = ns["look"](bc + np.array([sx * E, sy * E]))
        if q is None:
            return None
        qs.append(q)
        for b in betas:
            r = ns["judge"](q, bc, math.radians(b))
            for k in w:
                w[k] = min(w[k], r[k])
            inbox += r["inbox"]
    return w, inbox, qs


def ok(w, inbox):
    return w["place"] > 30 and w["cells"] > 30 and w["tag"] >= 16 and w["jm"] > 0.1 and inbox == 0


print("== 보기 후보 (β 0 · ±15 · ±30 · ±45, 보는 점 오차 18 mm 대각 네 방향 + 0): 화면 · 태그 · 관절 · 상자 안 통과한 것")
cands = []
for g, dist, ang in itertools.product((0.20, 0.25, 0.30, 0.35), (0.40, 0.44, 0.48, 0.52, 0.56), (35, 40, 45, 50)):
    v = evaluate(g, dist, ang, (0, 15, -15, 30, -30, 45, -45))
    if v is None:
        continue
    w, inbox, qs = v
    if ok(w, inbox):
        bd = [CE.body(q, True) for q in qs]
        b, pk = min(x[0] for x in bd), min(x[1] for x in bd)
        cands.append((g, dist, ang, w, b, pk))
        print(f"  틈 {g:.2f} · {dist:.2f} m · {ang}°: place {w['place']:.1f} · 9 칸 {w['cells']:.1f} · 태그 {w['tag']:.1f} px · 관절 {w['jm']:.2f} · "
              f"차체 {b:.1f} · 실은 퍽 {pk:.1f} mm {'(통과)' if b >= 15 and pk >= 15 else '(차체 · 퍽 불통과)'}", flush=True)
good = [c for c in cands if c[4] >= 15 and c[5] >= 15]
if good:
    good.sort(key=lambda c: -min(c[3]["place"] / 30, c[3]["cells"] / 30, c[3]["tag"] / 16, c[3]["jm"] / 0.1, c[4] / 15, c[5] / 15))
    g, dist, ang = good[0][:3]
    print(f"\n== 고른 후보 틈 {g:.2f} · {dist:.2f} m · {ang}° 를 β −45 ~ 45° (5° 간격) 로")
    w, inbox, qs = evaluate(g, dist, ang, range(-45, 46, 5))
    print(f"  place {w['place']:.1f} · 9 칸 {w['cells']:.1f} · 태그 {w['tag']:.1f} px · 관절 {w['jm']:.2f} · 상자 안 {inbox} · q(오차 0) {np.round(qs[0], 4).tolist()}")

print("\n== 놓기 자리: 가장 가까운 상자 점까지 앞 끝 틈 g 의 범위 (J1 → 9 칸, 칸 중심 오차 ±0.0127 두 축, 45° 놓기 0.12 ~ 0.35 m)")


def cell_range(beta, g):
    r = FRONT + g + HALF * (abs(math.cos(beta)) + abs(math.sin(beta)))
    ds = []
    for e in itertools.product((-0.0127, 0, 0.0127), repeat=2):
        c = np.array([r, 0.0]) + np.array(e)
        for i, j in itertools.product((-1, 0, 1), repeat=2):
            p = c + np.array([[math.cos(beta), -math.sin(beta)], [math.sin(beta), math.cos(beta)]]) @ np.array([i * CELL, j * CELL])
            ds.append(math.hypot(p[0] - J1_X, p[1]))
    return min(ds), max(ds)


for b in range(0, 46, 5):
    gs = [g / 1000 for g in range(0, 200)]
    okg = [g for g in gs if 0.12 <= cell_range(math.radians(b), g)[0] and cell_range(math.radians(b), g)[1] <= 0.35]
    print(f"  |β| {b:2d}°: 틈 " + (f"{min(okg) * 1000:.0f} ~ {max(okg) * 1000:.0f} mm" if okg else "없음")
          + f" (지금 final 틈 127 mm 에서 칸 거리 {cell_range(math.radians(b), 0.1272)[0]:.3f} ~ {cell_range(math.radians(b), 0.1272)[1]:.3f} m)")
