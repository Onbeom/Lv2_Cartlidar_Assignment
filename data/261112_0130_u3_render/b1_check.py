"""U3 렌더 전 B1 계산: 상자 밀림(정지 오차 + 지도 자리에서 밀림) dx · dy, 로봇 요 ±0.05, 상자 요 ±10° 에서 화면 여유.
  place_box : 윗면 네 모서리 + 15 mm (윗면 높이 0.07) 가 화면 가장자리에서 30 px 넘게 (11-12 검토 기준)
  9 칸 (기록): 3 x 3 칸 중심 퍽 윗면 30 mm (서 있는 +0.06 · 누운 +0.03) — survey 는 지도 자리를 겨눠 밀림이 그대로 들어감,
               verify 는 잰 중심을 겨눠 밀림이 빠짐 (여기 값은 survey 쪽)
카메라 = newK (fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53, 1280 x 720), 자세 q = look_tool (면 0.20 · 0.36 m · 60°), B URDF 정기구학.
    python3 b1_check.py
"""
import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

URDF = Path.home() / "git/lv2_next_module/261102_1440_urdf_B/isaac/urdf/waffle_4wd_light.urdf"
Q = {"joint1": 0.0, "joint2": -0.702179, "joint3": -0.189372, "joint4": 1.67696, "joint5": 0.0}
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
BOX, BOX_H, FRONT, FACE, CELL = 0.205, 0.07, 0.1328, 0.20, 0.0683
PRIOR = np.array([FRONT + FACE + BOX / 2, 0.0])
ICR = np.array([-0.0658, 0.0])
T = fixed_transform(str(URDF), "arm_camera_optical_frame", root="base_footprint", q=Q)
Ti = np.linalg.inv(T)


def margin(P):
    v = np.c_[P, np.ones(len(P))] @ Ti.T
    u, w = FX * v[:, 0] / v[:, 2] + CX, FY * v[:, 1] / v[:, 2] + CY
    return float(min(u.min(), W - 1 - u.max(), w.min(), H - 1 - w.max()))


def to_robot(p, ryaw):
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    v = p - ICR
    return ICR + np.c_[c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1]]


def pts(local, z, d, byaw, ryaw):
    c, s = math.cos(byaw), math.sin(byaw)
    R = np.c_[c * local[:, 0] - s * local[:, 1], s * local[:, 0] + c * local[:, 1]]
    xy = to_robot(PRIOR + np.asarray(d) + R, ryaw)
    return np.c_[xy, np.full(len(xy), z)]


h = BOX / 2 + 0.015
box_local = np.array([[sx * h, sy * h] for sx in (-1, 1) for sy in (-1, 1)])
cell_local = np.array([[cx + dx, cy + dy] for cx, cy in itertools.product((-CELL, 0, CELL), repeat=2)
                       for dx, dy in itertools.product((-0.015, 0.015), repeat=2)])
print(f"카메라 base_footprint ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f}), 계획 상자 중심 x {PRIOR[0]:.4f}")
print("밀림 ±s | place_box 모서리+15mm 최소 여유 px (가장 나쁜 꼭짓점) | 9 칸 서 있는 · 누운 최소 px (survey)")
for s in (0.0, 0.01, 0.02, 0.03, 0.04, 0.05):
    worst = (1e9, None)
    cw = [1e9, 1e9]
    for ryaw, bdeg, dx, dy in itertools.product((-0.05, 0.0, 0.05), (-10.0, 0.0, 10.0), (-s, 0.0, s), (-s, 0.0, s)):
        m = margin(pts(box_local, BOX_H, (dx, dy), math.radians(bdeg), ryaw))
        if m < worst[0]:
            worst = (m, (ryaw, bdeg, dx, dy))
        for k, z in enumerate((BOX_H + 0.06, BOX_H + 0.03)):
            cw[k] = min(cw[k], margin(pts(cell_local, z, (dx, dy), math.radians(bdeg), ryaw)))
    r, b, x, y = worst[1]
    print(f"  {s:.2f}  | {worst[0]:7.1f}  (요 {r:+.2f} 상자 {b:+.0f}° dx {x:+.2f} dy {y:+.2f}) | {cw[0]:7.1f} · {cw[1]:7.1f}")
# 한 방향씩 (어디가 먼저 깨지는지)
print("한 방향 밀림 (요 0 · 상자 0) place_box 여유 px:")
for dx, dy in ((0.03, 0), (-0.03, 0), (0, 0.03), (0, -0.03), (0.05, 0), (-0.05, 0), (0, 0.05), (0, -0.05)):
    print(f"  dx {dx:+.2f} dy {dy:+.2f}: {margin(pts(box_local, BOX_H, (dx, dy), 0.0, 0.0)):7.1f}")
# 참고: 앞뒤는 면 거리로 맞춘다고 보면 (dx = 정지 오차 ±0.01 만), 옆 밀림만 넓힘
print("참고 — dx ±0.01 고정, 옆 dy ±s (요 ±0.05 · 상자 ±10° 모두):")
for s in (0.03, 0.05):
    m = min(margin(pts(box_local, BOX_H, (dx, dy), math.radians(b), r))
            for r, b, dx, dy in itertools.product((-0.05, 0, 0.05), (-10.0, 0, 10.0), (-0.01, 0, 0.01), (-s, 0, s)))
    m0 = min(margin(pts(box_local, BOX_H, (dx, dy), math.radians(b), r))
             for r, b, dx, dy in itertools.product((-0.05, 0, 0.05), (-10.0, 0, 10.0), (0.0,), (-s, 0, s)))
    print(f"  dy ±{s:.2f}: {m:7.1f} px (dx 0 이면 {m0:7.1f})")
