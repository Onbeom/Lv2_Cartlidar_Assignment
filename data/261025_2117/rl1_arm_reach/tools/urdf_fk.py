"""URDF 순기구학(FK). numpy만 쓴다.

URDF의 관절 origin(xyz, rpy)·axis·type을 읽어 base_footprint 기준 링크 변환을 계산한다.
mimic 관절은 multiplier·offset으로 따라간다. 값 단위는 rad, m(SI).
여러 자세를 한꺼번에 계산할 수 있게 관절 값은 (N, ) 배열로 받는다.
"""
import xml.etree.ElementTree as ET

import numpy as np

URDF_DEFAULT = ("/home/pa4/git/lv2_next_module/261022_2119/urdf/waffle_4wd_description_261022_2119/"
                "urdf/waffle_4wd.urdf")


def _rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q):
    """축 axis(3,), 각 q(N,) → 회전 행렬 (N, 3, 3) (로드리게스)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    s, c = np.sin(q)[:, None, None], np.cos(q)[:, None, None]
    return np.eye(3)[None] + s * kx[None] + (1 - c) * (kx @ kx)[None]


class UrdfFK:
    def __init__(self, urdf_path=URDF_DEFAULT, root="base_footprint"):
        tree = ET.parse(urdf_path).getroot()
        self.joints = {}
        self.child_joint = {}
        for j in tree.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
            ax = j.find("axis")
            lim = j.find("limit")
            mim = j.find("mimic")
            info = {
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "T": np.block([[_rpy_to_mat(*rpy), np.array(xyz)[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]),
                "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0],
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
                "mimic": (mim.get("joint"), float(mim.get("multiplier", 1)), float(mim.get("offset", 0)))
                if mim is not None else None,
            }
            self.joints[info["name"]] = info
            self.child_joint[info["child"]] = info["name"]
        self.root = root

    def _chain(self, link):
        names = []
        while link != self.root:
            jn = self.child_joint[link]
            names.append(jn)
            link = self.joints[jn]["parent"]
        return names[::-1]

    def link_transforms(self, q, links):
        """q: {관절 이름: (N,) 배열}. 빠진 관절은 0. 반환 {링크: (N, 4, 4)} (root 기준)."""
        n = len(next(iter(q.values()))) if q else 1
        out = {}
        for link in links:
            T = np.tile(np.eye(4), (n, 1, 1))
            for jn in self._chain(link):
                j = self.joints[jn]
                T = T @ j["T"][None]
                if j["type"] == "fixed":
                    continue
                if j["mimic"] is not None:
                    src, mult, off = j["mimic"]
                    v = mult * np.asarray(q.get(src, np.zeros(n)), float) + off
                else:
                    v = np.asarray(q.get(jn, np.zeros(n)), float)
                M = np.tile(np.eye(4), (n, 1, 1))
                if j["type"] in ("revolute", "continuous"):
                    M[:, :3, :3] = _axis_angle(j["axis"], v)
                elif j["type"] == "prismatic":
                    M[:, :3, 3] = v[:, None] * (np.asarray(j["axis"], float) / np.linalg.norm(j["axis"]))[None]
                T = T @ M
            out[link] = T
        return out
