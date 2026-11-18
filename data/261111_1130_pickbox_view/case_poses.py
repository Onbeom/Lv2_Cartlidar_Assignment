"""세 접근 경우 (왼쪽 회전 +17.06° · 오른쪽 −17.06° · inline 0°) 마다 joint2~4 를 고정하고 joint1 만 골라 상자 + 여유가 화면 안에 드는지 (계산).
로봇 정지 오차 (앞뒤 · 옆 ±0.01 m, 방향 ±3°) 의 꼭짓점 경우도 모두 화면 안인지 본다."""
import math, sys, itertools, numpy as np
sys.path.insert(0, "/home/pa4/git/lv2_next_module/Cartlidar/sim_render")
from urdf_chain import fixed_transform
URDF = "/home/pa4/git/lv2_next_module/261102_1440_urdf_B/isaac/urdf/waffle_4wd_light.urdf"
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
C, PRE, HALF, ZTOP, MARG = -0.04035, 0.3625, 0.1025, 0.07, 0.015
Q234 = [float(v) for v in sys.argv[1:4]] if len(sys.argv) > 3 else [-1.80, 0.50, 1.90]

def box_in_robot(turn_deg, dx=0.0, dy=0.0, dth=0.0):
    psi = math.radians(turn_deg) + dth
    R = np.array([[math.cos(-psi), -math.sin(-psi)], [math.sin(-psi), math.cos(-psi)]])
    c = R @ np.array([PRE + dx - C, dy]) + np.array([C, 0.0])
    return c, -psi

def margin(q, c, yaw, marg=MARG):
    T = fixed_transform(URDF, "arm_camera_optical_frame", root="base_footprint", q=q); Ti = np.linalg.inv(T)
    Rb = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]); m = 1e9
    for sx, sy in itertools.product((-1, 1), (-1, 1)):
        p = np.r_[c + Rb @ np.array([sx, sy]) * (HALF + marg), ZTOP]; v = Ti[:3, :3] @ p + Ti[:3, 3]
        if v[2] <= 0.05: return -1e9
        u, w = FX * v[0] / v[2] + CX, FY * v[1] / v[2] + CY; m = min(m, u, W - u, w, H - w)
    return m

for turn in (17.06, -17.06, 0.0):
    c0, y0 = box_in_robot(turn)
    best = None
    for j1 in np.arange(-0.8, 0.81, 0.005):
        q = dict(joint1=j1, joint2=Q234[0], joint3=Q234[1], joint4=Q234[2], joint5=0.0)
        worst = min(margin(q, *box_in_robot(turn, dx, dy, math.radians(dt))) for dx, dy, dt in itertools.product((-0.01, 0.01), (-0.01, 0.01), (-3, 3)))
        if best is None or worst > best[1]: best = (j1, worst, margin(q, c0, y0))
    print(f"turn {turn:+.2f}: 상자 중심 ({c0[0]:.4f}, {c0[1]:+.4f}) 요 {math.degrees(y0):+.2f}°  joint1 {best[0]:+.3f}  여유 정상 {best[2]:.0f} px · 정지 오차 꼭짓점 최소 {best[1]:.0f} px")
