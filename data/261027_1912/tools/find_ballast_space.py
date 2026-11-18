"""앞 무게추 자리 찾기(정적 계산): 261027_1711 차체 표시 메시(base_link · 앞 라이다 · 바퀴)가 차지하지 않는 빈 상자 중에서
강철 750 g(95.5 cm³) + 받침 상자 벽 2 mm가 들어가는 것을 1 · 2층 사이(상판 아래)에서 찾는다.
고르는 순서: ① 무게중심을 앞으로 많이 옮기는 것(상자 가운데 x 큼) ② 좌우 가운데에 가까운 것(|y| 작음).
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/find_ballast_space.py
결과: results/ballast_space.json
"""
import itertools
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

W = Path(__file__).resolve().parents[1]
TOP = W.parent
PKG = TOP / "261027_1711/urdf/waffle_4wd_description_261027_1711"
sys.path.insert(0, str(TOP / "261025_2117/rl1_arm_reach/tools"))
from urdf_fk import UrdfFK  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

V_STEEL = 750 / 7.85            # cm³
WALL = 2.0                      # 받침 상자 벽 mm
VOX = 2.0                       # 칸 크기 mm
LO = np.array([40.0, -120.0, 54.0]); HI = np.array([133.0, 120.0, 102.0])   # 위 판 아랫면 z 102에 매담. z 54 아래는 보드 앞 커넥터 · 배선 자리(벨트 자료 README v2.1), 판 끝 x 132.8
fk = UrdfFK(str(PKG / "urdf/waffle_4wd.urdf"), root="base_link")
root = ET.parse(PKG / "urdf/waffle_4wd.urdf").getroot()


def link_mesh(ln):
    vo = root.find(f"link[@name='{ln}']/visual/origin")
    Tv = trimesh.transformations.euler_matrix(*[float(v) for v in vo.get("rpy").split()], axes="sxyz")
    Tv[:3, 3] = [float(v) for v in vo.get("xyz").split()]
    T = fk.link_transforms({}, [ln])[ln][0] @ Tv
    T[:3, 3] *= 1000
    m = trimesh.load(PKG / f"meshes/main/{ln}.dae", force="mesh")
    m.apply_scale(1000); m.apply_transform(T)
    return m


meshes = [trimesh.load(PKG / "meshes/main/base_link.dae", force="mesh").apply_scale(1000)]
meshes += [link_mesh(ln) for ln in ("lidar_front", "wheel_front_left", "wheel_front_right")]
n = np.ceil((HI - LO) / VOX).astype(int)
occ = np.zeros(n, bool)
for m in meshes:   # 면을 촘촘히 표본으로 찍어 차지한 칸 표시(속이 빈 부품 안쪽은 빈 칸으로 남을 수 있어 표면에서 1칸 넓힘)
    sel = np.all((m.triangles_center > LO - 20) & (m.triangles_center < HI + 20), axis=1)
    if not sel.any():
        continue
    sub = m.submesh([np.where(sel)[0]], append=True)
    pts, _ = trimesh.sample.sample_surface_even(sub, int(sub.area / 0.5) + 1000, seed=0)
    idx = np.floor((pts - LO) / VOX).astype(int)
    ok = np.all((idx >= 0) & (idx < n), axis=1)
    for d in itertools.product((-1, 0, 1), repeat=3):
        j = np.clip(idx[ok] + d, 0, n - 1)
        occ[j[:, 0], j[:, 1], j[:, 2]] = True
S = np.zeros(n + 1, int)
S[1:, 1:, 1:] = occ.cumsum(0).cumsum(1).cumsum(2)


def filled(i0, j0, k0, a, b, c):
    i1, j1, k1 = i0 + a, j0 + b, k0 + c
    return (S[i1, j1, k1] - S[i0, j1, k1] - S[i1, j0, k1] - S[i1, j1, k0] + S[i0, j0, k1] + S[i0, j1, k0] + S[i1, j0, k0] - S[i0, j0, k0])


best = []
for tx in range(14, 42, 2):                       # 강철 두께(앞뒤) mm
    for ty in range(50, 161, 10):                 # 강철 폭(좌우)
        tz = V_STEEL * 1000 / (tx * ty)           # 강철 높이
        if tz > 44 or tz < 15:
            continue
        a, b, c = (int(np.ceil((s + 2 * WALL) / VOX)) for s in (tx, ty, tz))   # 받침 상자 바깥 칸 수
        for i0 in range(n[0] - a, -1, -1):
            for j0 in range(0, n[1] - b + 1):
                for k0 in range(n[2] - c, -1, -1):
                    if filled(i0, j0, k0, a, b, c) == 0:
                        lo = LO + VOX * np.array([i0, j0, k0]); hi = lo + VOX * np.array([a, b, c])
                        ctr = (lo + hi) / 2
                        best.append({"steel_mm": [tx, ty, round(tz, 1)], "box_outer_lo_mm": lo.tolist(), "box_outer_hi_mm": hi.tolist(),
                                     "center_mm": ctr.tolist()})
                        break
best.sort(key=lambda e: (-round(e["center_mm"][0]), abs(e["center_mm"][1]), e["center_mm"][2]))
for e in best[:8]:
    print(e["steel_mm"], "바깥", np.round(e["box_outer_lo_mm"], 1).tolist(), "~", np.round(e["box_outer_hi_mm"], 1).tolist(), "가운데", np.round(e["center_mm"], 1).tolist())
(W / "results/ballast_space.json").write_text(json.dumps({"steel_volume_cm3": V_STEEL, "wall_mm": WALL, "voxel_mm": VOX, "candidates": best[:30]}, indent=1))
