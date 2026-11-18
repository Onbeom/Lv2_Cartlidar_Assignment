"""URDF 고정 관절 사슬로 base_footprint → 링크 변환(4 × 4)을 구한다. numpy 만 쓴다(Isaac Sim 파이썬 · 시스템 파이썬 공통).

fixed_transform: 움직이는 관절(바퀴 · 팔)은 0 위치로 본다. 카메라처럼 base_link 에 고정된 링크에만 쓴다.
joint_locals: 팔 자세를 USD 링크 변환에 넣을 때 쓴다(USD 링크 Xform = URDF 관절 원점, 2119 가져오기 결과).
"""
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    """URDF rpy(고정 축 x → y → z) 회전 행렬."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def fixed_transform(urdf_path, link, root="base_footprint", q=None):
    """root → link 변환. q 가 없을 때 사이에 고정이 아닌 관절이 있으면 오류. q 를 주면 움직이는 관절은 그 값(joint_locals)."""
    parents = {}
    for j in ET.parse(urdf_path).getroot().findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        t = np.eye(4)
        t[:3, :3] = rpy_matrix(*rpy)
        t[:3, 3] = xyz
        parents[j.find("child").get("link")] = (j.find("parent").get("link"), t, j.get("type"), j.get("name"))
    moving = joint_locals(urdf_path, q) if q is not None else {}

    def to_top(lk, stop):   # 트리 꼭대기(부모 없는 링크) → lk. stop 을 만나면 거기서 멈춤
        t = np.eye(4)
        while lk != stop and lk in parents:
            parent, tj, kind, name = parents[lk]
            if kind != "fixed":
                if q is None:
                    raise ValueError(f"{name} 는 {kind} 관절 — 고정 사슬이 아님")
                tj = moving[lk][1]
            t = tj @ t
            lk = parent
        return lk, t

    top, t_link = to_top(link, root)
    if top == root:
        return t_link
    # root 가 트리 꼭대기가 아님(새 모델 261028: base_link 가 뿌리, base_footprint 는 그 자식) → 꼭대기 기준으로 맞춤
    top_r, t_root = to_top(root, None)
    if top_r != top:
        raise KeyError(f"{root} 와 {link} 가 같은 트리에 없음")
    return np.linalg.inv(t_root) @ t_link


def axis_matrix(axis, angle):
    """단위 축 axis 둘레 angle 회전 행렬(로드리게스)."""
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def joint_locals(urdf_path, q):
    """움직이는 관절마다 자식 링크의 부모 기준 변환(관절 원점 × 관절 값). mimic 은 q 의 원래 관절에서 계산한다.

    반환: {자식 링크: (관절 이름, 4 × 4)}. q 에 없는 관절은 0.
    """
    out = {}
    joints = ET.parse(urdf_path).getroot().findall("joint")
    q = dict(q)
    for j in joints:
        m = j.find("mimic")
        if m is not None:
            q[j.get("name")] = float(m.get("multiplier", 1)) * q.get(m.get("joint"), 0.0) + float(m.get("offset", 0))
    for j in joints:
        kind = j.get("type")
        if kind == "fixed":
            continue
        o = j.find("origin")
        t = np.eye(4)
        t[:3, :3] = rpy_matrix(*[float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()])
        t[:3, 3] = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
        a = j.find("axis")
        axis = [float(v) for v in a.get("xyz").split()] if a is not None else [1.0, 0.0, 0.0]
        v = q.get(j.get("name"), 0.0)
        m = np.eye(4)
        if kind in ("revolute", "continuous"):
            m[:3, :3] = axis_matrix(axis, v)
        elif kind == "prismatic":
            m[:3, 3] = np.asarray(axis) * v
        out[j.find("child").get("link")] = (j.get("name"), t @ m)
    return out


def tree_root(urdf_path):
    """부모가 없는 링크(이전 모델 base_footprint, 새 모델 261028 base_link)."""
    root = ET.parse(urdf_path).getroot()
    children = {j.find("child").get("link") for j in root.findall("joint")}
    tops = [l.get("name") for l in root.findall("link") if l.get("name") not in children]
    if len(tops) != 1:
        raise ValueError(f"root link 가 하나가 아님: {tops}")
    return tops[0]


def planar_pose(x, y, yaw):
    """바닥 위 로봇 자세(x, y, yaw) → 4 × 4."""
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(0.0, 0.0, yaw)
    t[:3, 3] = [x, y, 0.0]
    return t
