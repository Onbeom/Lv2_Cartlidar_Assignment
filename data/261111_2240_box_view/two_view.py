"""두 번 보기로 3 × 3 칸(survey · verify) 을 나눠 덮을 수 있는지 (계산, 2026-11-12 미션 파트 261111 1).

box_view_eval.py 와 같은 자세 계산 (look_tool = box_look) 과 판정 값을 쓰되, 보는 점을 상자 가운데에서 x 로 o 만큼 옮겨 가며
칸마다 B1(퍽 윗면 30 mm 가 화면 안 + 30 px, 서 있는 · 누운 두 높이, 로봇 yaw ±0.05 · 상자 회전 0 · ±10°) 과
B2(태그 16 mm 짧은 변 ≥ 16 px, 640 폭) 를 따로 판정 → 앞 · 뒤 두 보기로 9 칸을 다 덮는 짝을 찾음. 관절 여유 > 0.1 · 상자 침범 없음도 봄.
  python two_view.py <dev urdf> [--face 0.127]
"""
import argparse
import itertools
import math
import os
import subprocess
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("dev")
ap.add_argument("--face", type=float, default=0.127)
ap.add_argument("--b2", type=float, default=16.0)
a = ap.parse_args()
AV = "/home/pa4/git/lv2_next_module/261111_1700_align_view"
sys.argv = ["align_search.py", a.dev, "--pose", "-1.37", "-0.01", "1.85"]
ns = {"__name__": "x"}
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(AV, "align_search.py")).read(), ns)
sys.stdout = _o
fk, LINKS = ns["fk"], ns["LINKS"]
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
FRONT, HALF, ZTOP, CELL, TAG = 0.1328, 0.1025, 0.07, 0.0683, 0.016
LO = np.array([-3.14159, -2.14, -1.6, -1.92, -2.8])
HI = np.array([3.14159, 1.6, 1.6, 2.0, 2.8])
ICR = (-0.0658, 0.0)
T_ab = fk("arm_base_link", {})
xc = FRONT + a.face + HALF
DISTS = [round(0.16 + 0.02 * i, 2) for i in range(10)]
ANGLES = [90, 85, 80, 75, 70, 65, 60]
CELLS = list(itertools.product((-CELL, 0, CELL), repeat=2))   # (앞뒤 x, 옆 y)


def rot(p, yaw):
    s, k = math.sin(-yaw), math.cos(-yaw)
    x, y = p[:, 0] - ICR[0], p[:, 1] - ICR[1]
    q = p.copy()
    q[:, 0] = ICR[0] + k * x - s * y
    q[:, 1] = ICR[1] + s * x + k * y
    return q


def proj(Ti, P):
    k = np.c_[P, np.ones(len(P))] @ Ti.T
    return FX * k[:, 0] / k[:, 2] + CX, FY * k[:, 1] / k[:, 2] + CY, k[:, 2]


def cell_ok(Ti, cx_, cy_):
    b1 = 1e9
    for yw, by in itertools.product((-0.05, 0.0, 0.05), (math.radians(-10), 0.0, math.radians(10))):
        c, s = math.cos(by), math.sin(by)
        P = []
        for z in (ZTOP + 0.06, ZTOP + 0.03):
            for dx, dy in itertools.product((-0.015, 0.015), repeat=2):
                x, y = cx_ + dx, cy_ + dy
                P.append([xc + c * x - s * y, s * x + c * y, z])
        u, v, zz = proj(Ti, rot(np.array(P), yw))
        if (zz <= 0.02).any():
            return -1e9, 0.0
        b1 = min(b1, np.minimum.reduce([u, W - u, v, H - v]).min())
    b2 = 1e9
    for z in (ZTOP + 0.03, ZTOP + 0.06):
        for tr in (0.0, math.pi / 4):
            c, s = math.cos(tr), math.sin(tr)
            P = np.array([[xc + cx_ + c * dx - s * dy, cy_ + s * dx + c * dy, z] for dx, dy in ((-TAG / 2, -TAG / 2), (TAG / 2, -TAG / 2), (TAG / 2, TAG / 2), (-TAG / 2, TAG / 2))])
            u, v, _ = proj(Ti, P)
            uv = np.c_[u, v] / 2.0
            b2 = min(b2, min(np.linalg.norm(uv[(i + 1) % 4] - uv[i]) for i in range(4)))
    return b1, b2


def arm_ok(q):
    Tg = fk("gripper_roll_link", q)
    pts = np.array([fk(l, q)[:3, 3] for l in LINKS] + [(Tg @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))])
    return not ((pts[:, 0] > FRONT + a.face) & (np.abs(pts[:, 1]) < HALF + 0.03) & (pts[:, 2] < ZTOP + 0.04)).any()


views = []   # (o, d, ang, q, mask, worst b1 of covered, worst b2 of covered)
for o in np.round(np.arange(-0.08, 0.0801, 0.02), 3):
    aim = np.linalg.inv(T_ab) @ np.array([xc + o, 0.0, ZTOP, 1.0])
    line = f"{aim[0]} {aim[1]} {aim[2]} 0.04 {len(DISTS)} {' '.join(map(str, DISTS))} {len(ANGLES)} {' '.join(map(str, ANGLES))}\n"
    out = subprocess.run([os.path.join(os.path.dirname(os.path.abspath(__file__)), "look_tool")], input=line, capture_output=True, text=True, check=True).stdout
    for l in out.split("\n"):
        v = l.split()
        if len(v) < 8 or v[2] != "1":
            continue
        q5 = [float(x) for x in v[3:8]]
        qs = [q5[0], q5[1] - 0.02, q5[2], q5[3] + 0.02, q5[4]]
        jm = min(np.min(np.array(qs) - LO), np.min(HI - np.array(qs)), np.min(np.array(q5) - LO), np.min(HI - np.array(q5)))
        if jm <= 0.1:
            continue
        mask, b1s, b2s = [], [], []
        okarm = True
        for qq in (q5, qs):
            q = {f"joint{i + 1}": qq[i] for i in range(5)}
            okarm &= arm_ok(q)
        if not okarm:
            continue
        per = []
        for cx_, cy_ in CELLS:
            worst1 = worst2 = 1e9
            for qq in (q5, qs):
                Ti = np.linalg.inv(fk("arm_camera_optical_frame", {f"joint{i + 1}": qq[i] for i in range(5)}))
                b1, b2 = cell_ok(Ti, cx_, cy_)
                worst1, worst2 = min(worst1, b1), min(worst2, b2)
            per.append((worst1, worst2))
        m = tuple(b1 > 30 and b2 >= a.b2 for b1, b2 in per)
        views.append(dict(o=o, d=float(v[0]), ang=float(v[1]), q=q5, jm=jm, mask=m, per=per))

full = [vw for vw in views if all(vw["mask"])]
print(f"상자 면 {a.face} m, 쓸 수 있는 보기 {len(views)}, 한 번에 9 칸 {len(full)}")
best = []
for v1, v2 in itertools.combinations(views, 2):
    cov = [x or y for x, y in zip(v1["mask"], v2["mask"])]
    if all(cov):
        # 칸마다 덮는 보기 중 나은 쪽의 여유, 그중 최소
        mb1 = min(max(p1[0] if m1 else -1e9, p2[0] if m2 else -1e9) for p1, p2, m1, m2 in zip(v1["per"], v2["per"], v1["mask"], v2["mask"]))
        mb2 = min(max(p1[1] if m1 else -1e9, p2[1] if m2 else -1e9) for p1, p2, m1, m2 in zip(v1["per"], v2["per"], v1["mask"], v2["mask"]))
        best.append((min(mb1 / 30, mb2 / a.b2, v1["jm"] / 0.1, v2["jm"] / 0.1), mb1, mb2, v1, v2))
best.sort(key=lambda t: -t[0])
print(f"두 번으로 9 칸을 덮는 짝 {len(best)}")
for sc, mb1, mb2, v1, v2 in best[:8]:
    print(f"  빠듯한 비 {sc:.2f} | B1 최소 {mb1:.0f} px · B2 최소 {mb2:.1f} px | "
          f"보기1 o {v1['o']:+.2f} d {v1['d']:.2f} {v1['ang']:.0f}° 여유 {v1['jm']:.2f} q {np.round(v1['q'], 3).tolist()} | "
          f"보기2 o {v2['o']:+.2f} d {v2['d']:.2f} {v2['ang']:.0f}° 여유 {v2['jm']:.2f} q {np.round(v2['q'], 3).tolist()}")
cov_rows = {}
for vw in views:
    k = sum(vw["mask"])
    cov_rows[k] = cov_rows.get(k, 0) + 1
print("보기 하나가 덮는 칸 수 분포", dict(sorted(cov_rows.items())))
