"""5번 (#154) ALIGN 후보 D1 ~ D3 남은 기준 확인 (계산, 2026-11-11 미션 파트 261111 1).

  python check_d.py <dev urdf (align_search 와 같은 것)> <isaac full urdf 폴더>

(c) A1 집게 가림 (기준 3판): 벌린 집게 (손가락 0.020 = open_position, 0.010 도) · gripper_roll_link ·
    손가락 링크 충돌 상자 꼭짓점을 newK 로 투영한 볼록 껍질 (화면 밖은 화면 가장자리로 자름) 과
    퍽 2 개 각각 (가장 가까운 면 기준 8 꼭짓점 투영의 사각형) 사이 최소 화면 거리 > 30 px. A1 의 장면 108 개 전부.
(d) A4 퍽 실은 팔레트: loaded URDF 의 퍽 12 개 (원기둥도 30 × 30 × 60 상자로 크게 봄) 와 팔 · 집게 충돌 상자 최소 거리
    ≥ 15 mm. Isaac 측정 끝 관절 · 궤적 관절 (align_probe_D.json) 로.
관절은 명령값과 Isaac 측정 끝 관절 (처짐 포함) 두 가지.
"""
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
DEV, ISAAC = sys.argv[1], sys.argv[2]

# align_search 의 FK · 장면 · 카메라 값을 그대로 씀 (한 자세 모드로 불러서 A1 ~ A4 이유도 출력)
CANDS = {"D1": [0, -1.63, 0.18, 1.89, 0], "D2": [0, -1.62, 0.21, 1.88, 0], "D3": [0, -1.62, 0.20, 1.89, 0]}
probe = json.load(open(os.path.join(HERE, "rl_probe", "align_probe_D.json")))
MEAS = {x["job"]["name"]: x["q_end"] for x in probe["jobs"] if x["job"]["kind"] == "static"}

ns = {}
for name, q in CANDS.items():
    for tag, qq in (("명령", q), ("측정", MEAS[name])):
        print(f"\n===== {name} {tag} {np.round(qq, 3).tolist()} — align_search 한 자세 검사")
        sys.argv = ["align_search.py", DEV, "--pose", str(qq[1]), str(qq[2]), str(qq[3])]
        ns = {"__name__": "align_search"}
        exec(open(os.path.join(HERE, "align_search.py")).read(), ns)

fk, scenes, ALL = ns["fk"], ns["scenes"], ns["ALL"]
FX, FY, CX, CY, W, H = ns["FX"], ns["FY"], ns["CX"], ns["CY"], ns["W"], ns["H"]
rpy = ns["rpy"]

import xml.etree.ElementTree as ET
root = ET.parse(DEV).getroot()
GBOX = []  # (링크, To, 반 크기)
for l in root.findall("link"):
    if l.get("name") not in ("gripper_roll_link", "gripper_left_link", "gripper_right_link"):
        continue
    for c in l.findall("collision"):
        b = c.find("geometry").find("box")
        o = c.find("origin")
        To = np.eye(4)
        if o is not None:
            To[:3, :3] = rpy(*map(float, o.get("rpy", "0 0 0").split()))
            To[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
        GBOX.append((l.get("name"), To, np.array(list(map(float, b.get("size").split()))) / 2))
SGN = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def grip_pts(q5, finger):
    q = {f"joint{i + 1}": q5[i] for i in range(5)}
    q.update(gripper_left_joint=finger, gripper_right_joint=finger)
    out = []
    for ln, To, h in GBOX:
        M = fk(ln, q) @ To
        P = SGN * h
        out.append((M[:3, :3] @ P.T).T + M[:3, 3])
    return np.vstack(out), q


def proj(P, Ti):
    c = np.c_[P, np.ones(len(P))] @ Ti.T
    z = c[:, 2]
    k = z > 0.005
    return np.c_[FX * c[k, 0] / z[k] + CX, FY * c[k, 1] / z[k] + CY]


def seg_dist(p, a, b):
    d = b - a
    t = np.clip(((p - a) @ d) / max(d @ d, 1e-12), 0, 1)
    return np.linalg.norm(p - (a + t * d))


def poly_rect_dist(poly, r):
    """볼록 다각형 (꼭짓점 순서) 과 축 정렬 사각형 r = (u0, v0, u1, v1) 사이 거리, 겹치면 0"""
    rc = np.array([[r[0], r[1]], [r[2], r[1]], [r[2], r[3]], [r[0], r[3]]])
    # 겹침: 분리축 (다각형 변 법선 + 사각형 축)
    def sep(A, B, axes):
        for ax in axes:
            pa, pb = A @ ax, B @ ax
            if pa.max() < pb.min() or pb.max() < pa.min():
                return True
        return False
    axes = [np.array([1.0, 0]), np.array([0, 1.0])]
    n = len(poly)
    for i in range(n):
        e = poly[(i + 1) % n] - poly[i]
        axes.append(np.array([-e[1], e[0]]))
    if not sep(poly, rc, axes):
        return 0.0
    best = 1e9
    for i in range(n):
        for p in rc:
            best = min(best, seg_dist(p, poly[i], poly[(i + 1) % n]))
    for i in range(4):
        for p in poly:
            best = min(best, seg_dist(p, rc[i], rc[(i + 1) % 4]))
    return best


print("\n===== (c) A1 집게 가림 (장면 108 개, 퍽 사각형 ↔ 집게 투영 볼록 껍질, 기준 > 30 px)")
for name, q in CANDS.items():
    for tag, qq in (("명령", q), ("측정", MEAS[name])):
        for finger in (0.020, 0.010):
            G, qd = grip_pts(qq, finger)
            T = fk("arm_camera_optical_frame", qd)
            Ti = np.linalg.inv(T)
            g = proj(G, Ti)
            g[:, 0] = np.clip(g[:, 0], 0, W)
            g[:, 1] = np.clip(g[:, 1], 0, H)
            in_img = len(g) and ((g[:, 0] > 0) & (g[:, 0] < W) & (g[:, 1] > 0) & (g[:, 1] < H)).any()
            if len(g) < 3:
                print(f"{name} {tag} 손가락 {finger:.3f}: 집게 화면 밖 (투영 점 {len(g)})")
                continue
            hull = g[ConvexHull(g, qhull_options="QJ").vertices]
            worst = (1e9, None)
            for sc in scenes:
                P = sc[3]
                for k in range(2):
                    uv = proj(P[8 * k:8 * k + 8], Ti)
                    r = (uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max())
                    d = poly_rect_dist(hull, r)
                    if d < worst[0]:
                        worst = (d, (sc[0], round(float(sc[1]), 3), sc[2], k))
            bb = (hull[:, 0].min(), hull[:, 1].min(), hull[:, 0].max(), hull[:, 1].max())
            ok = "통과" if worst[0] > 30 else "불통과"
            print(f"{name} {tag} 손가락 {finger:.3f}: 집게 화면 안 {bool(in_img)}, 껍질 범위 u {bb[0]:.0f}~{bb[2]:.0f} v {bb[1]:.0f}~{bb[3]:.0f} | "
                  f"퍽과 최소 {worst[0]:.0f} px ({worst[1]}) → {ok}")

print("\n===== (d) A4 퍽 실은 팔레트: 팔 · 집게 ↔ 실은 퍽 12 개 (30 × 30 × 60 상자) 최소 거리 (기준 ≥ 15 mm)")
sys.path.insert(0, os.path.join(HERE, "rl_probe"))
from align_geom import Geom, box_pts, tf  # noqa: E402
LG = Geom(os.path.join(ISAAC, "waffle_4wd_loaded.urdf"))
lroot = ET.parse(os.path.join(ISAAC, "waffle_4wd_loaded.urdf")).getroot()
PUCKS = []
for j in lroot.findall("joint"):
    ch = j.find("child").get("link")
    if ch.startswith("puck_"):
        o = j.find("origin")
        T = tf(list(map(float, o.get("xyz").split())), list(map(float, o.get("rpy").split())))
        P = box_pts([0.03, 0.03, 0.06])
        PUCKS.append((ch, (T[:3, :3] @ P.T).T + T[:3, 3]))


def puck_min(q5, finger=0.010):
    best = (9.0, None, None)
    for ln, an, A in LG.arm_pts(q5, finger):
        for pn, B in PUCKS:
            d = Geom.dist(A, B)
            if d < best[0]:
                best = (d, an, pn)
    return best


for x in probe["jobs"]:
    steps = x["steps"]
    rows = [puck_min(s[4:9]) for s in steps[::3]] + [puck_min(steps[-1][4:9])]
    m = min(rows, key=lambda r: r[0])
    print(f"{x['job']['kind']:6s} {x['job']['name']:18s} 최소 {m[0] * 1000:6.1f} mm ({m[1]} – {m[2]}) → {'통과' if m[0] >= 0.015 else '불통과'}")
