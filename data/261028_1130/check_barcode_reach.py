"""발제 바코드(중심 (80,650,450) mm, +X 를 봄)를 팔 카메라(Arducam)로 정면 가까이 볼 수 있는지 (모의 · 기구학만, Isaac Sim 없이).

URDF 261027_1711 팔 관절 한계 · 충돌 형상(arm_pose.ArmRealizer, 여유 5 mm)으로, 차체가 −X 를 보고 바코드 앞에 섰을 때
카메라 광학 중심을 바코드 앞 D(m)에, 광축을 −X(± pitch)로 둘 수 있는지 푼다. 판 · 신호등 크기는 가정(아래 BOXES).
실행: ~/isaacsim-venv/bin/python 261028_1130/check_barcode_reach.py
"""
import math
import sys
from pathlib import Path

import numpy as np

TOP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOP / "Cartlidar/sim_render"))
from arm_pose import ArmRealizer  # noqa: E402
from urdf_chain import planar_pose  # noqa: E402

U = TOP / "261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf"
BOXES = [(0.04, 0.75, 0.04, 0.45, 0.60, "board(가정: 바코드 판 x 0~0.08, 높이 0.6)"),
         (0.13, 0.95, 0.05, 0.05, 0.40, "signal(가정: 반지름 0.05, 높이 0.4)")]


class S:
    boxes = BOXES


ar = ArmRealizer(U, U.parent.parent, S(), {"pucks": []}, gripper=0.0)
rng = np.random.default_rng(0)
lo, hi = [l for l, _ in ar.lim], [h for _, h in ar.lim]
seeds = [rng.uniform(lo, hi) for _ in range(12)] + [np.array([0.0, 0.3, 0.3, 0.0, 0.0]), np.array([0.0, 0.8, -0.5, -0.3, 0.0])]


def cam_T(D, pitch_deg, y=0.65):
    a = math.radians(pitch_deg)
    z = np.array([-math.cos(a), 0.0, math.sin(a)])
    x = np.array([0.0, 1.0, 0.0])
    x -= z * (x @ z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, [0.08 + D, y, 0.45]
    return T


ar.backoff_max = 0.0
for D in (0.07, 0.10, 0.15):
    for pitch in (0, 15, -15):
        found = None
        for bx in np.arange(0.32, 0.62, 0.03):
            T_bf = planar_pose(bx, 0.65, math.pi)
            if ar.body_hits(T_bf):
                continue
            r = ar.frame(T_bf, cam_T(D, pitch), seeds)
            if r:
                found = (round(float(bx), 2), [round(float(v), 2) for v in r[0]])
                break
        print(f"바코드 앞 {D:.2f} m, 광축 {pitch:+d}°: " + (f"됨 — 차체 중심 x {found[0]} m, 관절 {found[1]}" if found else "못 감(차체 x 0.32~0.59)"), flush=True)
