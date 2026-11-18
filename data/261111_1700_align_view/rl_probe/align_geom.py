"""2026-11-11 (RL 261111 1, 미션 파트 5번 #154 요청) 팔 · 집게 충돌 상자와 차체 (base_link) 충돌 형상 사이 최소 거리 — 정밀 URDF (waffle_4wd_full) FK.

  python align_geom.py <urdf> --q j1 j2 j3 j4 j5 [--finger 0.010]     (한 자세)
  모듈로 쓰기: G = Geom(urdf); G.min_dist(q5, finger) -> (거리 m, 팔 링크, 차체 형상 이름)

- 팔 쪽 = link3 · link4 · link5 · gripper_roll_link · gripper_left/right_link 의 충돌 상자 (link2 = 관절1 회전대는 차체 바로 위 장착이라 뺌).
- 차체 쪽 = base_link 의 충돌 형상 전부 (CoACD 볼록 메시 + 상자). 메시는 볼록 껍질로 봄.
- 두 볼록 형상 거리 = 민코프스키 차 (A − B) 의 볼록 껍질에서 원점까지 거리. 원점이 안에 있으면 음수 (겹친 깊이의 근사 = 가장 가까운 면까지).
- 먼저 AABB 거리로 거름 (0.05 m 넘으면 계산 안 함).
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

ARM = ("link3", "link4", "link5", "gripper_roll_link", "gripper_left_link", "gripper_right_link")


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


def axis_rot(ax, a):
    ax = np.asarray(ax, float) / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return T


def read_stl(path):
    d = open(path, "rb").read()
    if d[:5] == b"solid" and b"facet" in d[:300]:
        v = [list(map(float, l.split()[1:4])) for l in d.decode(errors="ignore").splitlines() if l.strip().startswith("vertex")]
        return np.array(v)
    n = int.from_bytes(d[80:84], "little")
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("x", "<u2")]))
    return a["v"].reshape(-1, 3).astype(float)


def box_pts(size):
    s = np.asarray(size) / 2
    return np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])])


def org(e):
    o = e.find("origin")
    if o is None:
        return np.eye(4)
    return tf(list(map(float, o.get("xyz", "0 0 0").split())), list(map(float, o.get("rpy", "0 0 0").split())))


class Geom:
    def __init__(self, urdf):
        self.base = os.path.dirname(os.path.abspath(urdf))
        root = ET.parse(urdf).getroot()
        self.joints = {}
        for j in root.iter("joint"):
            a = j.find("axis")
            self.joints[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), T=org(j),
                                                            axis=list(map(float, a.get("xyz").split())) if a is not None else [1, 0, 0])
        self.shapes = {}  # link -> [(이름, 점 (링크 좌표))]
        for l in root.iter("link"):
            out = []
            for k, c in enumerate(l.findall("collision")):
                g = c.find("geometry")[0]
                T = org(c)
                if g.tag == "box":
                    P = box_pts(list(map(float, g.get("size").split())))
                    nm = f"{l.get('name')}/box{k}"
                elif g.tag == "mesh":
                    P = read_stl(os.path.normpath(os.path.join(self.base, g.get("filename"))))
                    P = P * np.array(list(map(float, g.get("scale", "1 1 1").split())))
                    P = np.unique(np.round(P, 6), axis=0)
                    nm = os.path.basename(g.get("filename"))
                else:
                    continue
                out.append((nm, (T[:3, :3] @ P.T).T + T[:3, 3]))
            self.shapes[l.get("name")] = out

    def link_T(self, link, q):
        """base_link 좌표 link 자세. q = 관절 이름 -> 값."""
        if link == "base_link":
            return np.eye(4)
        j = self.joints[link]
        T = self.link_T(j["parent"], q) @ j["T"]
        v = q.get(j["name"], 0.0)
        if j["type"] in ("revolute", "continuous"):
            T = T @ axis_rot(j["axis"], v)
        elif j["type"] == "prismatic":
            S = np.eye(4)
            S[:3, 3] = np.asarray(j["axis"]) * v
            T = T @ S
        return T

    def arm_pts(self, q5, finger=0.010):
        q = {f"joint{i + 1}": q5[i] for i in range(5)}
        q.update(gripper_left_joint=finger, gripper_right_joint=finger)
        out = []
        for ln in ARM:
            T = self.link_T(ln, q)
            for nm, P in self.shapes.get(ln, []):
                out.append((ln, nm, (T[:3, :3] @ P.T).T + T[:3, 3]))
        return out

    @staticmethod
    def dist(A, B):
        M = (A[:, None, :] - B[None, :, :]).reshape(-1, 3)
        try:
            h = ConvexHull(M)
        except Exception:
            h = ConvexHull(M, qhull_options="QJ")
        eq = h.equations  # n·x + d <= 0 안쪽
        s = eq[:, 3]  # 원점에서 각 면 값 = d
        if (s <= 1e-12).all():
            return float(s.max())  # 원점이 안: 가장 가까운 면까지 (음수)
        best = 1e9
        for f in h.simplices:
            best = min(best, pt_tri(np.zeros(3), M[f[0]], M[f[1]], M[f[2]]))
        return best

    def min_dist(self, q5, finger=0.010, cut=0.05):
        arm = self.arm_pts(q5, finger)
        body = self.shapes["base_link"]
        bmin = [(nm, P, P.min(0), P.max(0)) for nm, P in body]
        best = (9.0, None, None)
        for ln, an, A in arm:
            amn, amx = A.min(0), A.max(0)
            for bn, B, bmn, bmx in bmin:
                gap = np.maximum(0, np.maximum(bmn - amx, amn - bmx))
                if np.linalg.norm(gap) > min(cut, best[0]):
                    continue
                d = self.dist(A, B)
                if d < best[0]:
                    best = (d, f"{ln}/{an}", bn)
        return best


def pt_tri(p, a, b, c):
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = ab @ ap, ac @ ap
    if d1 <= 0 and d2 <= 0:
        return np.linalg.norm(ap)
    bp = p - b
    d3, d4 = ab @ bp, ac @ bp
    if d3 >= 0 and d4 <= d3:
        return np.linalg.norm(bp)
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        return np.linalg.norm(p - (a + ab * d1 / (d1 - d3)))
    cp = p - c
    d5, d6 = ab @ cp, ac @ cp
    if d6 >= 0 and d5 <= d6:
        return np.linalg.norm(cp)
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        return np.linalg.norm(p - (a + ac * d2 / (d2 - d6)))
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        return np.linalg.norm(p - (b + (c - b) * (d4 - d3) / ((d4 - d3) + (d5 - d6))))
    n = np.cross(ab, ac)
    return abs(n @ ap) / np.linalg.norm(n)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--q", type=float, nargs=5, required=True)
    ap.add_argument("--finger", type=float, default=0.010)
    a = ap.parse_args()
    G = Geom(a.urdf)
    d, an, bn = G.min_dist(a.q, a.finger)
    print(f"최소 거리 {d * 1000:.1f} mm  팔 {an}  차체 {bn}")
