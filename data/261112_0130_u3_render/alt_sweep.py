"""U3 B1 불통과 뒤 참고 탐색 (계산): 면 0.20, 보는 점 = 계획 상자 중심 + 앞뒤 오프셋 o, box_look 거리 · 각 → place_box 여유(밀림 ±s) · 서 있는 태그 짧은 변(검출 폭 960).
    python3 alt_sweep.py"""
import itertools, math, subprocess, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa
exec(open("b1_check.py").read().split("h = BOX / 2 + 0.015")[0].split("T = fixed_transform")[0])
LOOK = str(Path.home() / "git/lv2_next_module/261111_2240_box_view/look_tool")
T_ab = fixed_transform(str(URDF), "arm_base_link", root="base_footprint", q={})
h = BOX / 2 + 0.015
box_local = np.array([[sx * h, sy * h] for sx in (-1, 1) for sy in (-1, 1)])
TAG = 0.016
def run(o, d, ang, s):
    aim = np.linalg.inv(T_ab) @ np.array([PRIOR[0] + o, 0, BOX_H, 1])
    out = subprocess.run([LOOK], input=f"{aim[0]} {aim[1]} {aim[2]} 0.04 1 {d} 1 {ang}\n", capture_output=True, text=True).stdout.split()
    if len(out) < 8 or out[2] != "1":
        return None
    q = {f"joint{i+1}": float(out[3 + i]) for i in range(5)}
    T = fixed_transform(str(URDF), "arm_camera_optical_frame", root="base_footprint", q=q)
    Ti = np.linalg.inv(T)
    def proj(P):
        v = np.c_[P, np.ones(len(P))] @ Ti.T
        return np.c_[FX * v[:, 0] / v[:, 2] + CX, FY * v[:, 1] / v[:, 2] + CY]
    def marg(P):
        uv = proj(P); return min(uv[:, 0].min(), W - 1 - uv[:, 0].max(), uv[:, 1].min(), H - 1 - uv[:, 1].max())
    mb = 1e9; tag = 1e9
    for r, b, dx, dy in itertools.product((-0.05, 0, 0.05), (-10.0, 0, 10.0), (-s, 0, s), (-s, 0, s)):
        bb = math.radians(b)
        def P(local, z):
            c, si = math.cos(bb), math.sin(bb)
            R = np.c_[c * local[:, 0] - si * local[:, 1], si * local[:, 0] + c * local[:, 1]]
            cr, sr = math.cos(-r), math.sin(-r)
            xy = PRIOR + np.array([dx, dy]) + R - ICR
            xy = ICR + np.c_[cr * xy[:, 0] - sr * xy[:, 1], sr * xy[:, 0] + cr * xy[:, 1]]
            return np.c_[xy, np.full(len(xy), z)]
        mb = min(mb, marg(P(box_local, BOX_H)))
    # 태그 (밀림 없음, 9 칸, 돌림 0 · 45°) 짧은 변 x 0.75 (960 폭)
    for cx_, cy_ in itertools.product((-CELL, 0, CELL), repeat=2):
        for rot in (0, math.pi / 4):
            sq = np.array([[math.cos(rot + k * math.pi / 2), math.sin(rot + k * math.pi / 2)] for k in range(4)]) * TAG / math.sqrt(2)
            uv = proj(np.c_[PRIOR[0] + cx_ + sq[:, 0], cy_ + sq[:, 1], np.full(4, BOX_H + 0.06)])
            tag = min(tag, min(np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4)) * 0.75)
    return mb, tag, [round(q[f"joint{i+1}"], 3) for i in range(5)]
rows = []
for o in (-0.04, -0.02, 0.0, 0.02, 0.04):
    for d in (0.36, 0.38, 0.40, 0.42, 0.44):
        for ang in (60, 55, 50, 45):
            r = run(o, d, ang, 0.03)
            if r:
                rows.append((o, d, ang) + r)
ok = [r for r in rows if r[3] >= 30]
print(f"후보 {len(rows)} 개 (IK · 한계 ok), place_box 여유(밀림 ±0.03) ≥ 30 px: {len(ok)} 개")
for r in sorted(rows, key=lambda r: -min(r[3] / 30, r[4] / 16))[:12]:
    print(f"  보는 점 {r[0]:+.2f}  {r[1]:.2f} m {r[2]}°: place 여유 {r[3]:6.1f} px · 태그(960) {r[4]:5.1f} px · q {r[5]}")
