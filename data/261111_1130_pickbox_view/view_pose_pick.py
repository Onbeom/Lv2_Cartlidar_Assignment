"""누운 퍽 스킬에서 집는 상자(시작 박스) 윗면 전체를 팔 Arducam 한 장에 담는 팔 자세 찾기 (B URDF 정기구학, 계산).

로봇 자리 = 미션 계획기 회전 접근 자리 (poses.py): 앞 끝 -> 상자 면 0.1272 m 에서 로봇 중심 (base_link x -0.04035) 둘레로
turn 17.06° (퍽 쪽) 돈 상태. 돌기 전 상자 중심 = base x 0.3625, 상자 요 0.
카메라 = 인지 노드가 편 영상의 내부값 (실측 K · D 를 180° 돌린 뒤 getOptimalNewCameraMatrix alpha 0: fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53).
조건: 상자 윗면 네 꼭짓점 + 여유 margin 이 화면 안, 집게 · 카메라 원점이 서 있는 퍽 윗면 (바닥 0.13) + 0.05 위.
    python3 view_pose_pick.py <urdf> [--turn 17.06] [--margin 0.015]
"""
import argparse
import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("urdf")
ap.add_argument("--turn", type=float, default=17.06)
ap.add_argument("--margin", type=float, default=0.015)
ap.add_argument("--top", type=int, default=10)
ap.add_argument("--j2_min", type=float, default=-9.0, help="joint2 하한 (팔-차체 접촉 피하기, 11-11 모의 −1.76 에서 몸체에 막힘)")
ap.add_argument("--j4_max", type=float, default=9.0)
ap.add_argument("--sort", choices=["res", "margin"], default="res")
ap.add_argument("--max_res", type=float, default=0.6, help="mm/px 상한 (margin 정렬 때)")
a = ap.parse_args()

r = ET.parse(a.urdf).getroot()
J = {}


def rpy(x, y, z):
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                     [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx], [-sy, cy * sx, cy * cx]])


for j in r.findall("joint"):
    o = j.find("origin")
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = rpy(*map(float, o.get("rpy", "0 0 0").split()))
        T[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
    ax = j.find("axis")
    J[j.find("child").get("link")] = (j.find("parent").get("link"), T, j.get("name"), j.get("type"),
                                      np.array(list(map(float, ax.get("xyz").split()))) if ax is not None else None)
lim = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
       for j in r.findall("joint") if j.find("limit") is not None and j.get("type") == "revolute"}


def rot(ax, q):
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R = np.eye(4)
    R[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return R


def fk(link, q, root="base_footprint"):
    T = np.eye(4)
    while link in J and link != root:
        p, Tj, n, t, ax = J[link]
        if t in ("revolute", "continuous") and n in q:
            Tj = Tj @ rot(ax, q[n])
        T = Tj @ T
        link = p
    return T


# 인지 노드는 영상을 먼저 편다 (perception_node undistort: getOptimalNewCameraMatrix alpha 0) -> place_box 는 편 영상 + newK 를 씀.
# 실측 K · D (arm_camera_1280x720.yaml) 를 180° 돌린 값 (cx 584.12 · cy 359.36, p1 p2 부호 반대) 의 newK
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
CENTER_X, PRE_C, HALF, ZTOP = -0.04035, 0.3625, 0.1025, 0.07
psi = math.radians(a.turn)
R2 = np.array([[math.cos(-psi), -math.sin(-psi)], [math.sin(-psi), math.cos(-psi)]])
bc = R2 @ np.array([PRE_C - CENTER_X, 0.0]) + np.array([CENTER_X, 0.0])
yaw = -psi
Rb = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
h = HALF + a.margin
corners = [np.r_[bc + Rb @ np.array([sx * h, sy * h]), ZTOP] for sx in (-1, 1) for sy in (-1, 1)]
print(f"상자 중심 base_footprint ({bc[0]:.4f}, {bc[1]:.4f}) 요 {math.degrees(yaw):.2f}°, J1 -> 상자 중심 방향 "
      f"{math.degrees(math.atan2(bc[1], bc[0] - 0.096)):.2f}°")

best = []
j1c = math.atan2(bc[1], bc[0] - 0.096)
for q1, q2, q3, q4 in itertools.product(np.arange(j1c - 0.3, j1c + 0.31, 0.05), np.arange(-2.1, 1.6, 0.05),
                                        np.arange(-1.6, 1.6, 0.05), np.arange(-1.9, 2.0, 0.05)):
    q = {"joint1": q1, "joint2": q2, "joint3": q3, "joint4": q4, "joint5": 0.0}
    if any(not (lim[k][0] <= v <= lim[k][1]) for k, v in q.items() if k in lim):
        continue
    if q2 < a.j2_min or q4 > a.j4_max:
        continue
    T = fk("arm_camera_optical_frame", q)
    if T[2, 3] < 0.18:
        continue
    Ti = np.linalg.inv(T)
    ok, m = True, 1e9
    for p in corners:
        c = Ti[:3, :3] @ p + Ti[:3, 3]
        if c[2] <= 0.05:
            ok = False
            break
        u, v = FX * c[0] / c[2] + CX, FY * c[1] / c[2] + CY
        m = min(m, u, W - u, v, H - v)
        if m < 0:
            ok = False
            break
    if not ok:
        continue
    g = fk("gripper_roll_link", q)[2, 3]
    if g < 0.13 + 0.05:
        continue
    zc = T[:3, 2]
    down = math.degrees(math.acos(max(-1.0, min(1.0, -zc[2]))))
    dist = float(np.linalg.norm(T[:3, 3] - np.r_[bc, ZTOP]))
    # 윗면 해상도: 가장 먼 꼭짓점의 mm / 화소 (거리 / f, 비스듬함 무시)
    far = max(float(np.linalg.norm(T[:3, 3] - p)) for p in corners)
    best.append((far / FX * 1000, down, dist, m, g, q1, q2, q3, q4, T[2, 3]))
best = [b for b in best if b[0] <= a.max_res] if a.sort == "margin" else best
best.sort(key=(lambda b: -b[3]) if a.sort == "margin" else None)
print("후보 수", len(best))
for b in best[:a.top]:
    print(f"먼 꼭짓점 {b[0]:.2f} mm/px  아래와 {b[1]:.1f}°  카메라→윗면 중심 {b[2]:.3f} m  여유 {b[3]:.0f} px  "
          f"집게 z {b[4]:.3f}  카메라 z {b[9]:.3f}  q {b[5]:.3f} {b[6]:.2f} {b[7]:.2f} {b[8]:.2f}")
