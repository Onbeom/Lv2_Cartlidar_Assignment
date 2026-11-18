"""설계 v15 차체 거리 벌점용 기하 준비(CPU, 정책 학습과 무관).

1. 차체 거리 격자: 차체 충돌 형상 표면 점(make_chassis_points.py 결과)까지의 최소 거리를 base_link 좌표 격자(간격 grid_m)에 미리 계산.
2. 팔 몸체 점: arm_link_3·4·5·gripper_left/right_link의 URDF 충돌 형상 표면에서 고르게 뽑은 점(각 링크 좌표).
3. 확인: 평가 표(seed 2)·학습 표 목표 자세(MoveIt 충돌 검사 통과)에서 팔 점 ↔ 차체 최소 거리 분포.

사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/make_clearance_geom.py
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from urdf_fk import UrdfFK, _rpy_to_mat  # noqa: E402

PKG = Path("/home/pa4/git/lv2_next_module/261022_2119/urdf/waffle_4wd_description_261022_2119")
URDF = PKG / "urdf" / "waffle_4wd.urdf"
RES = Path(__file__).resolve().parents[1] / "results"
BODIES = ["arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link", "gripper_right_link"]

ap = argparse.ArgumentParser()
ap.add_argument("--grid_m", type=float, default=0.005)
ap.add_argument("--arm_spacing", type=float, default=0.012)
args = ap.parse_args()


def link_mesh(root, name):
    ms = []
    for link in root.findall("link"):
        if link.get("name") != name:
            continue
        for c in link.findall("collision"):
            o = c.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            g = c.find("geometry")[0]
            if g.tag == "mesh":
                m = trimesh.load(PKG / g.get("filename").split("/", 3)[3], force="mesh")
            elif g.tag == "box":
                m = trimesh.creation.box(extents=[float(v) for v in g.get("size").split()])
            else:
                raise ValueError(g.tag)
            T = np.eye(4)
            T[:3, :3] = _rpy_to_mat(*rpy)
            T[:3, 3] = xyz
            m.apply_transform(T)
            ms.append(m)
    return trimesh.util.concatenate(ms)


root = ET.parse(URDF).getroot()
# 1. 차체 거리 격자
cp = np.load(RES / "geom" / "chassis_points_base_link.npz")["points"]
tree = cKDTree(cp)
lo = np.array([-0.45, -0.45, -0.10])
hi = np.array([0.55, 0.45, 0.75])
shape = np.ceil((hi - lo) / args.grid_m).astype(int) + 1
ax = [lo[i] + args.grid_m * np.arange(shape[i]) for i in range(3)]
G = np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3)
dist, _ = tree.query(G, workers=-1)
dist = dist.reshape(shape).astype(np.float32)

# 2. 팔 몸체 점
arm_pts, arm_idx = [], []
for i, b in enumerate(BODIES):
    m = link_mesh(root, b)
    n = max(20, int(m.area / args.arm_spacing ** 2))
    p, _ = trimesh.sample.sample_surface_even(m, n, seed=0)
    arm_pts.append(p)
    arm_idx.append(np.full(len(p), i))
arm_pts = np.vstack(arm_pts).astype(np.float32)
arm_idx = np.concatenate(arm_idx)
np.savez_compressed(RES / "geom" / "clearance_geom.npz", grid=dist, grid_lo=lo.astype(np.float32), grid_m=args.grid_m,
                    arm_points=arm_pts, arm_body_index=arm_idx, arm_bodies=np.array(BODIES), urdf=str(URDF))
print("grid", dist.shape, "arm points", len(arm_pts), {b: int((arm_idx == i).sum()) for i, b in enumerate(BODIES)})

# 3. 목표 자세 여유 거리
fk = UrdfFK(str(URDF), root="base_link")


def clearance(qtab, gripper):
    n = len(qtab)
    names = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch"]
    qd = {nm: qtab[:, i] for i, nm in enumerate(names)}
    qd["gripper_left_joint"] = np.full(n, gripper)
    Ts = fk.link_transforms(qd, BODIES)
    out = np.full((n, len(BODIES)), np.inf)
    for i, b in enumerate(BODIES):
        P = arm_pts[arm_idx == i]
        W = np.einsum("nij,pj->npi", Ts[b][:, :3, :3], P) + Ts[b][:, None, :3, 3]
        out[:, i] = tree.query(W.reshape(-1, 3), workers=-1)[0].reshape(n, -1).min(1)
    return out


for name in ["targets_eval_seed2.npz", "targets_train_seed1.npz"]:
    f = RES / "targets" / name
    if not f.exists():
        continue
    t = np.load(f)
    import json
    info = json.loads(str(t["info"]))
    c = clearance(t["q"], info["gripper"])
    cm = c.min(1)
    print(name, "n", len(cm), "min clearance mm percentiles 0/1/5/10/50:",
          np.percentile(cm * 1000, [0, 1, 5, 10, 50]).round(1).tolist(),
          " ≤10mm", int((cm <= 0.01).sum()), " ≤20mm", int((cm <= 0.02).sum()), " ≤30mm", int((cm <= 0.03).sum()))
    print("   per body ≤20mm", {b: int((c[:, i] <= 0.02).sum()) for i, b in enumerate(BODIES)})
