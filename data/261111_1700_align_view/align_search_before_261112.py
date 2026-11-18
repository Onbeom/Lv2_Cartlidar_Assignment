"""FINE 정렬 팔 자세 (ALIGN) 후보 찾기 — 기준 3판 A1 · A3 · A4 일부 (계산, B안 URDF 정기구학).

A1: 상자 면 0.127 ~ 0.37 m (12 점) × yaw −0.05 · 0 · +0.05 rad × 배치 3 가지
    (±Y 접근에서 조준 퍽이 가운데이고 다른 퍽이 옆 +0.08 · −0.08, inline 은 두 퍽이 앞뒤 ±0.04) 에서
    퍽 2 개 (서 있는 30 × 30 × 60, 상자 윗면 0.07 ~ 0.13) 8 꼭짓점 투영이 화면 안 · 가장자리 30 px 넘게.
    카메라 = 편 영상 newK (fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53, 1280 × 720).
A3: 관절 한계 여유 0.1 rad.
A4 일부 (계산): 면 0.127 에서 팔 점 (링크 원점 · 잡는 점 · 손가락 끝) 이 상자 안 (면 앞 · 상자 윗면 + 0.04 아래) 에 들지 않음,
    바닥 위 0.02, 회전 반지름 (로봇 중심 x −0.04035 에서 수평) ≤ 0.218 m. 차체 접촉 · 최소 거리는 Isaac 에서 따로.
    python3 align_search.py <urdf> [--j2_min ..] [--top 15]
"""
import argparse
import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("urdf")
ap.add_argument("--j2_min", type=float, default=-9.0)
ap.add_argument("--top", type=int, default=15)
ap.add_argument("--step", type=float, default=0.05)
ap.add_argument("--box", type=float, nargs=6, help="j2 lo hi · j3 lo hi · j4 lo hi 격자 범위")
ap.add_argument("--a2", action="store_true", help="앞 D435 → 퍽 시선 팔 충돌 상자 가림 (면 0.37 · 0.35) 도 검사")
ap.add_argument("--pose", type=float, nargs=3, help="이 j2 j3 j4 하나만 검사 (탈락 이유 출력)")
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


# A2: 팔 쪽 링크 (관절 사슬 아래 모두) 의 충돌 상자 (URDF box), 앞 D435 컬러 광학 원점
_arm = {"link1"}
_ch = True
while _ch:
    _ch = False
    for c, v in J.items():
        if (v[0] in _arm or v[2] == "joint1") and c not in _arm:
            _arm.add(c)
            _ch = True
_boxes = []
for l in r.findall("link"):
    if l.get("name") not in _arm:
        continue
    for col in l.findall("collision"):
        b = col.find("geometry").find("box")
        if b is None:
            continue
        o = col.find("origin")
        To = np.eye(4)
        if o is not None:
            To[:3, :3] = rpy(*map(float, o.get("rpy", "0 0 0").split()))
            To[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
        _boxes.append((l.get("name"), To, np.array(list(map(float, b.get("size").split()))) / 2))


def seg_obb(p0, p1, M, h):
    Ri = M[:3, :3].T
    a_ = Ri @ (p0 - M[:3, 3])
    b_ = Ri @ (p1 - M[:3, 3])
    d = b_ - a_
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(a_[k]) > h[k]:
                return False
        else:
            u_, v_ = (-h[k] - a_[k]) / d[k], (h[k] - a_[k]) / d[k]
            t0, t1 = max(t0, min(u_, v_)), min(t1, max(u_, v_))
            if t0 > t1:
                return False
    return True


D435 = fk("top_camera_front_color_optical_frame", {})[:3, 3]


def a2_blocked(q):
    """면 0.37 · 0.35, 배치 3 가지, 퍽 윗면 · 중간 높이 시선 중 팔 상자에 막히는 것 목록"""
    bx = [(n, fk(n, q) @ To, h) for n, To, h in _boxes]
    out = []
    for d in (0.37, 0.35):
        xc = FRONT + d + HALF
        for lname, pucks in LAYOUTS.items():
            for dx, dy in pucks:
                for zf in (ZTOP + 0.06, ZTOP + 0.03):
                    p = np.array([xc + dx, dy, zf])
                    hit = [n for n, M, h in bx if seg_obb(D435, p, M, h)]
                    if hit:
                        out.append((d, lname, (dx, dy), zf, hit[0]))
    return out


FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
FRONT, C_X, HALF, ZTOP = 0.1328, -0.04035, 0.1025, 0.07
DS = np.linspace(0.127, 0.37, 12)
YAWS = (-0.05, 0.0, 0.05)
LAYOUTS = {"옆+": [(0.0, 0.0), (0.0, 0.08)], "옆-": [(0.0, 0.0), (0.0, -0.08)], "inline": [(-0.04, 0.0), (0.04, 0.0)]}
corner = np.array([[sx * 0.015, sy * 0.015, z] for sx in (-1, 1) for sy in (-1, 1) for z in (ZTOP, ZTOP + 0.06)])

# 장면 점 미리 계산: [배치][d][yaw] → (16, 3) 퍽 꼭짓점 (base_footprint)
scenes = []
for lname, pucks in LAYOUTS.items():
    for d in DS:
        xc = FRONT + d + HALF
        for yw in YAWS:
            c, s = math.cos(yw), math.sin(yw)
            pts = []
            for dx, dy in pucks:
                P = corner + np.array([xc + dx, dy, 0.0])
                Px = P[:, 0] * c - P[:, 1] * s
                Py = P[:, 0] * s + P[:, 1] * c
                pts.append(np.stack([Px, Py, P[:, 2]], 1))
            scenes.append((lname, d, yw, np.vstack(pts)))
ALL = np.stack([sc[3] for sc in scenes])          # (n, 16, 3)
ALLh = np.concatenate([ALL, np.ones(ALL.shape[:2] + (1,))], 2)
LINKS = ["link2", "link3", "link4", "link5", "gripper_roll_link", "arm_camera_link"]

res = []
st = a.step
_b = a.box or (-2.04, 1.5, -1.5, 1.5, -1.82, 1.9)
grid = [tuple(a.pose)] if a.pose else itertools.product(np.arange(_b[0], _b[1] + 1e-9, st), np.arange(_b[2], _b[3] + 1e-9, st), np.arange(_b[4], _b[5] + 1e-9, st))
why = print if a.pose else (lambda *x: None)
for q2, q3, q4 in grid:
    if q2 < a.j2_min:
        continue
    q = {"joint1": 0.0, "joint2": q2, "joint3": q3, "joint4": q4, "joint5": 0.0}
    if any(not (lim[k][0] + 0.1 <= v <= lim[k][1] - 0.1) for k, v in q.items() if k in lim and k != "joint1" and k != "joint5"):
        why("A3 관절 한계 여유 0.1 rad 안 됨", {k: (round(v - lim[k][0], 3), round(lim[k][1] - v, 3)) for k, v in q.items() if k in lim})
        if not a.pose:
            continue
    T = fk("arm_camera_optical_frame", q)
    if T[2, 2] > -0.05:          # 광축이 아래를 보지 않으면 버림
        continue
    Ti = np.linalg.inv(T)
    c = ALLh @ Ti.T                # (n, 16, 4)
    zc = c[..., 2]
    if (zc <= 0.03).any():
        continue
    u = FX * c[..., 0] / zc + CX
    v = FY * c[..., 1] / zc + CY
    m = np.minimum.reduce([u, W - u, v, H - v]).min()
    if a.pose:
        bad = [(sc[0], round(float(sc[1]), 3), sc[2]) for sc, mm in zip(scenes, np.minimum.reduce([u, W - u, v, H - v]).min(1)) if mm < 30]
        why(f"A1 최소 여유 {m:.0f} px, 30 px 못 넘는 장면 {len(bad)}/{len(scenes)}", bad[:6])
    if m < 30 and not a.pose:
        continue
    # 팔 점: 링크 원점, 잡는 점, 손가락 끝 (벌림, 손가락 0.010 m: y ± 0.031)
    Tg = fk("gripper_roll_link", q)
    pts = [fk(l, q)[:3, 3] for l in LINKS] + [(Tg @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))]
    pts = np.array(pts)
    if pts[:, 2].min() < 0.02:
        continue
    face = FRONT + 0.127
    if ((pts[:, 0] > face) & (pts[:, 2] < ZTOP + 0.04)).any():
        why("A4 면 0.127 에서 팔 점이 상자 영역")
        if not a.pose:
            continue
    rad = np.hypot(pts[:, 0] - C_X, pts[:, 1]).max()
    why(f"회전 반지름 {rad:.3f} m, 팔 앞 끝 x {pts[:, 0].max():.3f}, 가장 낮은 팔 점 z {pts[:, 2].min():.3f}")
    if rad > 0.218 and not a.pose:
        continue
    if a.a2 or a.pose:
        bl = a2_blocked(q)
        why(f"A2 가림 {len(bl)} 시선", bl[:4])
        if bl and not a.pose:
            continue
    down = math.degrees(math.acos(max(-1.0, min(1.0, -T[2, 2]))))
    res.append((m, q2, q3, q4, down, T[2, 3], rad, pts[:, 0].max()))
res.sort(key=lambda x: -x[0])
print("후보 수", len(res))
for x in res[:a.top]:
    print(f"최소 여유 {x[0]:.0f} px  q [0, {x[1]:.2f}, {x[2]:.2f}, {x[3]:.2f}, 0]  광축 아래 {x[4]:.1f}°  카메라 z {x[5]:.3f}  "
          f"회전 반지름 {x[6]:.3f}  팔 앞 끝 x {x[7]:.3f}")
