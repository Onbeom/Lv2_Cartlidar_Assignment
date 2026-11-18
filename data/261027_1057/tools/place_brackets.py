"""출력용 브라켓 STL(출력 자세)을 조립 위치(base_link 좌표)로 옮긴다.

축 방향 24가지 회전 후보마다 조립 외곽(최소점)에 맞춰 옮기고, URDF 충돌 메시(같은 부품의 볼록 분해, base_link 좌표)
표면까지 거리 평균이 가장 작은 회전을 고른다. 결과: 각 부품의 4×4 변환과 조립 위치 STL(mm).
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/place_brackets.py
"""
import glob
import itertools
import json
from pathlib import Path

import numpy as np
import trimesh

TOP = Path(__file__).resolve().parents[2]
PRINT = TOP / "mine/CADai/CAD/Print"
COLL = TOP / "261022_2119/urdf/waffle_4wd_description_261022_2119/meshes/collision"
OUT = Path(__file__).resolve().parents[1] / "placed"
PARTS = {"Floor_Bracket_Left_Main": "146", "Floor_Bracket_Left_Profile_Carrier": "157",
         "Floor_Bracket_Right_Main": "165", "Floor_Bracket_Right_Profile_Carrier": "176"}


def rotations():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                R[i, j] = s
            if np.isclose(np.linalg.det(R), 1.0):
                out.append(R)
    return out


OUT.mkdir(parents=True, exist_ok=True)
res = {}
for name, part in PARTS.items():
    m = trimesh.load(PRINT / f"{name}.stl")
    hull = trimesh.util.concatenate([trimesh.load(f) for f in sorted(glob.glob(str(COLL / f"base_link_part{part}_*.stl")))])
    hull.apply_scale(1000.0)
    pts, _ = trimesh.sample.sample_surface(m, 3000, seed=0)
    best = None
    for R in rotations():
        p = pts @ R.T
        lo = (m.vertices @ R.T).min(0)
        t = hull.bounds[0] - lo
        d = np.abs(trimesh.proximity.signed_distance(hull, p + t))
        score = float(np.mean(d))
        if best is None or score < best[0]:
            best = (score, R, t)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = best[1], best[2]
    placed = m.copy()
    placed.apply_transform(T)
    placed.export(OUT / f"{name}_assembled_mm.stl")
    res[name] = {"mean_dist_to_collision_hull_mm": best[0], "T_print_to_base_link_mm": T.tolist(),
                 "bounds_mm": placed.bounds.round(2).tolist(), "volume_cm3": m.volume / 1000.0}
    print(name, round(best[0], 3), placed.bounds.round(1).tolist())
json.dump(res, open(OUT / "placement.json", "w"), indent=2)
