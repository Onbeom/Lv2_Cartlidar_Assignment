"""한 퍽의 계획 실패 진단: 시도 · 차체 후보마다 장 1 · 2 가 aim_frame 으로 풀리는지."""
import sys, json, math, os
from pathlib import Path
import numpy as np
sys.argv = [sys.argv[0], "--worlds", sys.argv[1], "--out", "/tmp/unused", "--only-plan"]
src = Path("Cartlidar/sim_render/render_b0.py").read_text().split("# ---------- 계획 ----------")[0]
g = {"__name__": "rb", "__file__": "Cartlidar/sim_render/render_b0.py"}
exec(compile(src, "render_b0_head", "exec"), g)
wdir = Path(g["args"].worlds[0]); lay = json.loads((wdir / "layout.json").read_text())
sc = g["Scene"](lay, g["drawing"], g["args"].urdf, g["pkg_dir"], frame=g["args"].frame, arm_q=g["DRIVE_Q"], shelf_top=0.07)
ar = g["ArmRealizer"](g["args"].urdf, g["pkg_dir"], sc, lay, gripper=0.0)
p = next(p for p in lay["pucks"] if p["name"] == os.environ["PUCK"])
top, _, _ = g["top_face"](p)
r_ik = np.random.default_rng(0)
seeds = ar.prof["seeds"] + [r_ik.uniform([l for l, _ in ar.lim], [h for _, h in ar.lim]) for _ in range(12)]
rng = np.random.default_rng(5)
heads = tuple(math.radians(h) for h in (0, 45, -45, 90, -90))
for tilt in ((0, 5), (15, 30)):
    for att in range(3):
        T = g["down_view"](rng, top, 0.10, tilt)
        cands = [b for b in ar.base_candidates(T, [], heads) if not ar.body_hits(b)]
        res = []
        for b in cands:
            f0 = ar.frame(b, T, seeds)
            fa = g["aim_frame"](ar, b, T, top, seeds)
            v = None
            if f0 is not None:
                q = np.linalg.inv(f0[2]) @ np.append(top, 1.0); v = round(283 * q[1] / q[2] + 240)
            res.append((f0 is not None, v, fa is not None))
        print("tilt", tilt, "후보", len(cands), "frame 됨", sum(r[0] for r in res), "aim 됨", sum(r[2] for r in res), "v", [r[1] for r in res if r[0]][:8], flush=True)
