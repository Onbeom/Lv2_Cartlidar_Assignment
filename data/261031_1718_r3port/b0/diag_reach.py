"""B0′ 계획 진단: 팔 자세가 없는 상자에서 퍽 하나 · 장 하나씩 따로 풀리는지, 차체 후보가 몇 개인지."""
import sys, json, math
from pathlib import Path
import numpy as np
sys.argv = [sys.argv[0], "--worlds", sys.argv[1], "--out", "/tmp/unused", "--only-plan"] + sys.argv[2:]
sys.path.insert(0, "Cartlidar/sim_render")
import importlib.util
spec = importlib.util.spec_from_file_location("rb", "Cartlidar/sim_render/render_b0.py")
src = Path("Cartlidar/sim_render/render_b0.py").read_text().split("# ---------- 계획 ----------")[0]
g = {"__name__": "rb", "__file__": "Cartlidar/sim_render/render_b0.py"}
exec(compile(src, "render_b0_head", "exec"), g)
wdir = Path(g["args"].worlds[0])
lay = json.loads((wdir / "layout.json").read_text())
sc = g["Scene"](lay, g["drawing"], g["args"].urdf, g["pkg_dir"], frame=g["args"].frame, arm_q=g["DRIVE_Q"], shelf_top=0.07)
ar = g["ArmRealizer"](g["args"].urdf, g["pkg_dir"], sc, lay, gripper=0.0)
rng = np.random.default_rng(0)
import os
for bi in map(int, os.environ.get("BOXES", "2 6").split()):
    pucks = [p for p in lay["pucks"] if p.get("box") == bi and p["location"].startswith("start")]
    for p in pucks:
        top, _, _ = g["top_face"](p)
        T = g["down_view"](rng, top, 0.10)
        cands = [b for b in ar.base_candidates(T, []) if not ar.body_hits(b)]
        ok = 0
        for b in cands:
            f = g["aim_frame"](ar, b, T, top, ar.prof["seeds"])
            ok += f is not None
        print("box", bi, p["name"], p["shape"], "standing", p["standing"], "top", np.round(top, 3), "후보", len(cands), "풀림", ok, flush=True)
# 실패 원인: 첫 후보 차체 자리에서 높이별 · 물림 없이 IK 해(충돌 무시)가 있는지, 충돌이면 무엇과
for bi in map(int, os.environ.get("BOXES", "2 6").split()):
    for p in [p for p in lay["pucks"] if p.get("box") == bi and p["location"].startswith("start")]:
        top, _, _ = g["top_face"](p)
        T = g["down_view"](rng, top, 0.10)
        cands = [b for b in ar.base_candidates(T, []) if not ar.body_hits(b)]
        hits = {}
        lo, hi = [l for l, _ in ar.lim], [h for _, h in ar.lim]
        seeds = ar.prof["seeds"] + [rng.uniform(lo, hi) for _ in range(12)]
        for b in cands[:6]:
            for v, pe, ae in ar.solve_arm(b, T, seeds):
                h = ar.arm_hits(b, v)
                key = tuple(sorted({x[1] for x in h})) if h else ("없음",)
                hits[key] = hits.get(key, 0) + 1
        print("box", bi, p["name"], p["shape"], p["standing"], "후보", len(cands), "IK해 충돌 상대", hits, flush=True)
