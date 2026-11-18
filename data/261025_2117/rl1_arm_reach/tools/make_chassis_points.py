"""차체 충돌 형상(URDF base_link·lidar_front·lidar_rear의 collision 메시, 모두 base_link 좌표) 표면에서 점을 고르게 뽑아 저장.
설계 v15의 차체 거리 벌점(팔 몸체 점 ↔ 차체 점 최소 거리)에 쓴다. 정책 학습과 무관한 준비 단계(CPU).

사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/make_chassis_points.py [--spacing 0.01]
"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

PKG = Path("/home/pa4/git/lv2_next_module/261022_2119/urdf/waffle_4wd_description_261022_2119")
URDF = PKG / "urdf" / "waffle_4wd.urdf"
OUT = Path(__file__).resolve().parents[1] / "results" / "geom" / "chassis_points_base_link.npz"
LINKS = ("base_link", "lidar_front", "lidar_rear")   # lidar_* ↔ base_link 고정 관절은 원점·회전 0(URDF)

ap = argparse.ArgumentParser()
ap.add_argument("--spacing", type=float, default=0.01, help="점 간격(m) 목표")
args = ap.parse_args()

root = ET.parse(URDF).getroot()
meshes = []
for link in root.findall("link"):
    if link.get("name") not in LINKS:
        continue
    for c in link.findall("collision"):
        o = c.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        assert max(abs(v) for v in rpy) < 1e-9, "회전 있는 충돌 형상은 아직 다루지 않음"
        g = c.find("geometry")[0]
        if g.tag == "mesh":
            m = trimesh.load(PKG / g.get("filename").split("/", 3)[3], force="mesh")
        elif g.tag == "box":
            m = trimesh.creation.box(extents=[float(v) for v in g.get("size").split()])
        elif g.tag == "cylinder":
            m = trimesh.creation.cylinder(radius=float(g.get("radius")), height=float(g.get("length")))
        elif g.tag == "sphere":
            m = trimesh.creation.icosphere(radius=float(g.get("radius")))
        else:
            raise ValueError(g.tag)
        m.apply_translation(xyz)
        meshes.append(m)
whole = trimesh.util.concatenate(meshes)
n = int(whole.area / args.spacing ** 2)
pts, _ = trimesh.sample.sample_surface_even(whole, n, seed=0)
pts = np.vstack([pts, whole.vertices]).astype(np.float32)
OUT.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(OUT, points=pts, urdf=str(URDF), spacing=args.spacing, n_meshes=len(meshes))
print("meshes", len(meshes), "area m²", round(float(whole.area), 4), "points", len(pts),
      "bbox", pts.min(0).round(3).tolist(), pts.max(0).round(3).tolist(), "→", OUT)
