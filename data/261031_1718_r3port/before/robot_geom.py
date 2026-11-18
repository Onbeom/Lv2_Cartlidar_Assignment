"""URDF 충돌 형상으로 로봇 겹침 계산 (가상 월드 배치·W4 판정 공용).

- 순방향 기구학: URDF 관절(origin xyz·rpy, axis, mimic)로 base_footprint 기준 링크 자세(4x4)를 구한다.
- 충돌 형상: mesh는 STL 꼭짓점의 볼록 껍질(Isaac Sim 가져오기 설정 convexHull과 같은 근사),
  box는 꼭짓점 8개, cylinder는 바깥에 접하는 24각기둥(실제보다 약간 큼 -> 겹침 판정이 보수적).
- 겹침: 볼록 껍질과 회전한 상자의 교집합을 선형계획으로 찾는다.
  depth = max_{x in 껍질} min_k (h_k - |x_k|) (상자 좌표). depth >= 0 이면 겹침(닿음 포함).
"""
import math
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from scipy.optimize import linprog

ARM_LINKS = ("arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link", "gripper_right_link")


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_R(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def make_T(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def origin_T(elem):
    o = elem.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return make_T(rpy_to_R(*rpy), xyz)


@lru_cache(maxsize=None)
def hull_vertices(path):
    mesh = trimesh.load(path, force="mesh")
    return np.asarray(mesh.convex_hull.vertices, float)


class Robot:
    def __init__(self, urdf_path, package_dir):
        self.urdf_path = Path(urdf_path)
        self.package_dir = Path(package_dir)
        root = ET.parse(self.urdf_path).getroot()
        self.pkg_name = None
        self.joints = {}      # child link -> joint
        self.joint_by_name = {}
        for j in root.findall("joint"):
            a = j.find("axis")
            lim = j.find("limit")
            mim = j.find("mimic")
            d = {"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                 "child": j.find("child").get("link"), "T0": origin_T(j),
                 "axis": [float(v) for v in a.get("xyz").split()] if a is not None else [1.0, 0.0, 0.0],
                 "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                 "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
                 "mimic": (mim.get("joint"), float(mim.get("multiplier", 1)), float(mim.get("offset", 0)))
                 if mim is not None else None}
            self.joints[d["child"]] = d
            self.joint_by_name[d["name"]] = d
        children = set(self.joints)
        self.links = [l.get("name") for l in root.findall("link")]
        roots = [l for l in self.links if l not in children]
        if len(roots) != 1:
            raise ValueError(f"root link가 하나가 아님: {roots}")
        self.root = roots[0]
        self.shapes = {}      # link -> [(T_link_shape, local vertices)]
        for l in root.findall("link"):
            items = []
            for c in l.findall("collision"):
                T = origin_T(c)
                g = c.find("geometry")
                if g.find("mesh") is not None:
                    m = g.find("mesh")
                    fn = m.get("filename")
                    rel = fn.split("://", 1)[1].split("/", 1)[1] if fn.startswith("package://") else fn
                    scale = np.array([float(v) for v in m.get("scale", "1 1 1").split()])
                    items.append((T, hull_vertices(str(self.package_dir / rel)) * scale))
                elif g.find("box") is not None:
                    s = np.array([float(v) for v in g.find("box").get("size").split()]) / 2
                    items.append((T, np.array([[sx, sy, sz] for sx in (-s[0], s[0]) for sy in (-s[1], s[1])
                                               for sz in (-s[2], s[2])])))
                elif g.find("cylinder") is not None:
                    r = float(g.find("cylinder").get("radius")) / math.cos(math.pi / 24)
                    h = float(g.find("cylinder").get("length")) / 2
                    ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
                    ring = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)
                    items.append((T, np.vstack([np.c_[ring, np.full(24, z)] for z in (-h, h)])))
                elif g.find("sphere") is not None:
                    r = float(g.find("sphere").get("radius"))
                    items.append((T, np.array([[sx, sy, sz] for sx in (-r, r) for sy in (-r, r) for sz in (-r, r)])))
            if items:
                self.shapes[l.get("name")] = items
        self.adjacent = {frozenset((d["parent"], d["child"])) for d in self.joints.values()}

    def full_q(self, q):
        """mimic 관절까지 채운 관절값."""
        out = dict(q)
        for d in self.joint_by_name.values():
            if d["mimic"]:
                src, mul, off = d["mimic"]
                out[d["name"]] = mul * out.get(src, 0.0) + off
        return out

    def link_poses(self, q, T_world_root=None):
        q = self.full_q(q)
        poses = {self.root: np.eye(4) if T_world_root is None else T_world_root}

        def pose(link):
            if link in poses:
                return poses[link]
            d = self.joints[link]
            T = pose(d["parent"]) @ d["T0"]
            v = q.get(d["name"], 0.0)
            if d["type"] in ("revolute", "continuous"):
                T = T @ make_T(axis_R(d["axis"], v))
            elif d["type"] == "prismatic":
                T = T @ make_T(p=np.asarray(d["axis"]) * v)
            poses[link] = T
            return T

        for l in self.links:
            pose(l)
        return poses

    def world_shapes(self, q, T_world_root=None):
        """[(link, 꼭짓점 Nx3 월드 좌표)]"""
        poses = self.link_poses(q, T_world_root)
        out = []
        for link, items in self.shapes.items():
            for T, V in items:
                W = (poses[link] @ T)
                out.append((link, V @ W[:3, :3].T + W[:3, 3]))
        return out


def box_depth(V, center, half, yaw=0.0):
    """볼록 껍질(꼭짓점 V)과 상자(중심, 반 길이, z축 회전 yaw)의 겹침 깊이(m, L-무한 기준). >= 0 이면 겹침."""
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    P = (np.asarray(V) - np.asarray(center)) @ R      # 상자 좌표
    half = np.asarray(half, float)
    # 빠른 거름: 축정렬 범위가 떨어져 있으면 그 간격으로 판정
    gap = np.maximum(P.min(0) - half, -half - P.max(0))
    if (gap > 0).any():
        return -float(gap.max())
    n = len(P)
    # 변수 [lambda_1..n, s], 목적 max s
    cobj = np.zeros(n + 1)
    cobj[-1] = -1.0
    A, b = [], []
    for k in range(3):
        A.append(np.r_[P[:, k], 1.0]); b.append(half[k])
        A.append(np.r_[-P[:, k], 1.0]); b.append(half[k])
    res = linprog(cobj, A_ub=np.array(A), b_ub=np.array(b), A_eq=np.r_[np.ones(n), 0.0][None, :], b_eq=[1.0],
                  bounds=[(0, None)] * n + [(None, None)], method="highs")
    if not res.success:
        raise RuntimeError(f"linprog 실패: {res.message}")
    return float(res.x[-1])


def hull_depth(V1, V2):
    """두 볼록 껍질 겹침 여부(선형계획, 공통점 존재). >= 0 이면 겹침, 값은 의미 없음(0 또는 음수 거리 근사 아님)."""
    n1, n2 = len(V1), len(V2)
    # 빠른 거름
    gap = np.maximum(V1.min(0) - V2.max(0), V2.min(0) - V1.max(0))
    if (gap > 0).any():
        return -float(gap.max())
    # sum a_i V1_i = sum b_j V2_j, sum a = 1, sum b = 1
    A_eq = np.zeros((5, n1 + n2))
    A_eq[:3, :n1] = V1.T
    A_eq[:3, n1:] = -V2.T
    A_eq[3, :n1] = 1
    A_eq[4, n1:] = 1
    res = linprog(np.zeros(n1 + n2), A_eq=A_eq, b_eq=[0, 0, 0, 1, 1], bounds=[(0, None)] * (n1 + n2), method="highs")
    return 0.0 if res.status == 0 else -1.0
