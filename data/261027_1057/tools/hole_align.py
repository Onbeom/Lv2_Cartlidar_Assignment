"""긴 브라켓 플랜지의 Ø4 구멍(판 체결)이 와플 판 구멍 위에 오는지, 브라켓 앞뒤 이동량별로 센다(정적 계산, 시험 아님).
판 재료 = 판 높이 가운데 단면 다각형. 구멍 중심에서 반지름 1.4 mm 원(M3 나사 몸통 근사)이 판 재료와 겹치지 않으면 '맞음'.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/hole_align.py
"""
import json
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Point
from shapely.ops import unary_union

H = Path(__file__).resolve().parents[1]


def material(mesh, z):
    s = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    p2, T = s.to_planar()
    polys = []
    for poly in p2.polygons_full:   # 평면 좌표 → base_link xy
        from shapely.affinity import affine_transform
        polys.append(affine_transform(poly, [T[0, 0], T[0, 1], T[1, 0], T[1, 1], T[0, 3], T[1, 3]]))
    return unary_union(polys)


def holes(mesh, z, dmin=3.5, dmax=4.5):
    s = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    p2, T = s.to_planar()
    out = []
    for poly in p2.polygons_full:
        for ring in poly.interiors:
            pts = np.asarray(ring.coords)[:, :2]
            c = pts.mean(0)
            d = 2 * np.mean(np.linalg.norm(pts - c, axis=1))
            if dmin < d < dmax:
                c3 = T @ np.r_[c, 0, 1]
                out.append((float(c3[0]), float(c3[1])))
    return out


lower = material(trimesh.load(H / "asm/solids/s008.stl"), 7.5)
upper = material(trimesh.load(H / "asm/solids/s009.stl"), 106.5)
pv3 = json.loads((H / "placed_v3/placement.json").read_text())
res = {}
for nm in ("Floor_Bracket_Left_Main", "Floor_Bracket_Right_Main"):
    e = pv3[nm]
    bm = trimesh.load(H / "out_TO_A" / f"{e['source_print']}_TO.stl")
    bm.apply_transform(np.asarray(e["T_print_to_base_link_mm"]))
    lo, up = holes(bm, 13.0), holes(bm, 100.5)
    for sh in np.arange(-15.0, 15.5, 0.5):
        fl = [not lower.intersects(Point(x + sh, y).buffer(1.4)) for x, y in lo]
        fu = [not upper.intersects(Point(x + sh, y).buffer(1.4)) for x, y in up]
        res.setdefault(float(sh), {})[nm] = (sum(fl), len(fl), sum(fu), len(fu))
for sh, v in res.items():
    tot = sum(a + c for a, b, c, d in v.values()); n = sum(b + d for a, b, c, d in v.values())
    if tot >= n - 4 or sh in (0.0, 12.5):
        print(f"이동 {sh:+5.1f} mm: 맞는 구멍 {tot}/{n}  " + "  ".join(f"{k.split('_')[2]} 아래 {a}/{b} 위 {c}/{d}" for k, (a, b, c, d) in v.items()))
(H / "results/hole_align.json").write_text(json.dumps({str(k): v for k, v in res.items()}, ensure_ascii=False, indent=1))
