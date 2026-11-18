"""한 퍽 진단: 주변 장애물 · 차체 후보(비껴 섬 포함) · 후보마다 팔 기부 → 카메라 수평 거리 · IK 결과."""
import sys, json, math, os
from pathlib import Path
import numpy as np
sys.argv = [sys.argv[0], "--worlds", sys.argv[1], "--out", "/tmp/unused", "--only-plan"]
sys.path.insert(0, "Cartlidar/sim_render")
src = Path("Cartlidar/sim_render/render_b0.py").read_text().split("# ---------- 계획 ----------")[0]
g = {"__name__": "rb", "__file__": "Cartlidar/sim_render/render_b0.py"}
exec(compile(src, "render_b0_head", "exec"), g)
wdir = Path(g["args"].worlds[0]); lay = json.loads((wdir / "layout.json").read_text())
sc = g["Scene"](lay, g["drawing"], g["args"].urdf, g["pkg_dir"], frame=g["args"].frame, arm_q=g["DRIVE_Q"], shelf_top=0.07)
ar = g["ArmRealizer"](g["args"].urdf, g["pkg_dir"], sc, lay, gripper=0.0)
name = os.environ["PUCK"]
p = next(p for p in lay["pucks"] if p["name"] == name)
top, _, _ = g["top_face"](p)
print("퍽", name, p["shape"], p["standing"], "top", np.round(top, 3))
near = [(round(math.hypot(b[0] - top[0], b[1] - top[1]), 3), b[5], round(b[0], 3), round(b[1], 3), round(b[2], 3), round(b[3], 3)) for b in sc.boxes]
print("가까운 장애물", sorted(near)[:8])
print("차체 윤곽 x 범위", sc.footprint[:, 0].min().round(3), sc.footprint[:, 0].max().round(3), "y", sc.footprint[:, 1].min().round(3), sc.footprint[:, 1].max().round(3))
rng = np.random.default_rng(1)
heads = tuple(math.radians(h) for h in (0, 45, -45, 90, -90))
for tilt in ((0, 5), (15, 30), (30, 45)):
    T = g["down_view"](rng, top, 0.10, tilt)
    cands = [b for b in ar.base_candidates(T, [], heads) if not ar.body_hits(b)]
    lo, hi = [l for l, _ in ar.lim], [h for _, h in ar.lim]
    seeds = ar.prof["seeds"] + [rng.uniform(lo, hi) for _ in range(20)]
    rows = []
    for b in cands:
        armb = (b @ ar.T_bf_root @ np.array([0.096, 0, 0.131, 1]))[:3]
        sols = list(ar.solve_arm(b, T, seeds))
        hits = [ar.arm_hits(b, v) for v, _, _ in sols]
        rows.append((round(float(np.hypot(*(T[:2, 3] - armb[:2]))), 3), len(sols), [h[0][1] if h else "ok" for h in hits][:3]))
    print("tilt", tilt, "후보", len(cands), sorted(rows)[:10])
