"""URDF 두 판의 바닥 투영 외곽(base_link 기준 x · y 범위)과 바퀴 축 x 를 같은 방식으로 계산. 충돌 형상 기준, 팔은 주어진 자세(drive).
사용: isaacsim-venv python footprint_compare.py <urdf> <이름> [z_max]  — 정적 계산(모의 주행 아님)."""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

DRIVE = {"joint1": 0.0, "joint2": -1.35, "joint3": 1.1497, "joint4": 0.8094, "joint5": 0.0, "gripper_roll": 0.0}  # PR #50 robot_params.json drive
urdf = Path(sys.argv[1]); name = sys.argv[2]; zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 9.0
root = ET.parse(urdf).getroot()


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def tf(el):
    T = np.eye(4)
    if el is not None:
        T[:3, 3] = [float(v) for v in el.get('xyz', '0 0 0').split()]
        T[:3, :3] = rpy(*[float(v) for v in el.get('rpy', '0 0 0').split()])
    return T


def axis_rot(ax, q):
    ax = np.asarray(ax) / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    T = np.eye(4); T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K; return T


children = {}
for j in root.findall('joint'):
    children.setdefault(j.find('parent').get('link'), []).append(j)
world = {'base_link': np.eye(4)}; wheel_x = {}; stack = ['base_link']
while stack:
    p = stack.pop()
    for j in children.get(p, []):
        c = j.find('child').get('link'); T = world[p] @ tf(j.find('origin'))
        if j.get('type') in ('revolute', 'continuous', 'prismatic'):
            ax = [float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0').split()]
            if j.get('type') != 'prismatic': T = T @ axis_rot(ax, DRIVE.get(j.get('name'), 0.0))
            if 'wheel' in j.get('name'): wheel_x[j.get('name')] = round(float(T[0, 3]), 4)
        world[c] = T; stack.append(c)
pts = {}
for link in root.findall('link'):
    n = link.get('name')
    if n not in world: continue
    for col in link.findall('collision'):
        T = world[n] @ tf(col.find('origin')); g = col.find('geometry')[0]
        if g.tag == 'box': m = trimesh.creation.box([float(v) for v in g.get('size').split()])
        elif g.tag == 'cylinder': m = trimesh.creation.cylinder(float(g.get('radius')), float(g.get('length')), sections=48)
        elif g.tag == 'sphere': m = trimesh.creation.icosphere(radius=float(g.get('radius')))
        else:
            f = g.get('filename'); f = f.split('://', 1)[-1] if '://' in f else f
            fp = (urdf.parent / f) if not f.startswith('/') else Path(f)
            if not fp.exists():
                cand = list(urdf.parents[2].rglob(Path(f).name)); fp = cand[0] if cand else fp
            m = trimesh.load(fp, force='mesh')
            if g.get('scale'): m.apply_scale([float(v) for v in g.get('scale').split()])
        v = trimesh.transform_points(m.vertices, T); v = v[v[:, 2] <= zmax]
        if len(v): pts.setdefault(n, []).append(v)
allp = np.vstack([np.vstack(v) for v in pts.values()])
arm = [n for n in pts if n.startswith(('link', 'gripper', 'end_effector'))]
body = np.vstack([np.vstack(pts[n]) for n in pts if n not in arm])
out = {'name': name, 'urdf': str(urdf), 'z_max': zmax, 'wheel_joint_x': wheel_x,
       'wheel_axle_mid_x': round(float(np.mean([min(wheel_x.values()), max(wheel_x.values())])), 4) if wheel_x else None,
       'all_x': [round(float(allp[:, 0].min()), 4), round(float(allp[:, 0].max()), 4)], 'all_abs_y': round(float(np.abs(allp[:, 1]).max()), 4),
       'body_x': [round(float(body[:, 0].min()), 4), round(float(body[:, 0].max()), 4)], 'body_abs_y': round(float(np.abs(body[:, 1]).max()), 4),
       'arm_links': arm, 'max_radius_xy_from_base_link': round(float(np.hypot(allp[:, 0], allp[:, 1]).max()), 4)}
print(json.dumps(out, ensure_ascii=False))
