"""발제 맵 월드 정적 확인 (W1 꼴): 새 프로세스에서 열어 합성 오류 · 못 찾는 참조 수, prim 수. + 위에서 본 그림(판정 아님).
실행 (PC): ~/isaacsim-venv/bin/python -I check_mission_world.py worlds/dice_mission/seed_1 ... --png <그림>
"""
import json
import sys
from pathlib import Path

from pxr import Usd, UsdUtils

args = sys.argv[1:]
png = args[args.index("--png") + 1] if "--png" in args else None
dirs = [a for a in args if a != "--png" and a != png]
out = {}
for d in dirs:
    st = Usd.Stage.Open(str(Path(d) / "task_world.usda"))
    errs = st.GetCompositionErrors()
    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(Path(d) / "task_world.usda"))
    cnt = {k: len(st.GetPrimAtPath(f"/World/{k}").GetChildren()) for k in ("Maze", "Shelves", "Pucks", "PlaceTags", "Barcodes")}
    out[d] = {"composition_errors": len(errs), "unresolved": [str(u) for u in unresolved], **cnt}
print(json.dumps(out, ensure_ascii=False, indent=1))
if png:
    import math
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    lay = json.loads((Path(dirs[0]) / "layout.json").read_text())
    dr = json.loads(Path(lay["sources"]["drawing"]).read_text())
    fig, ax = plt.subplots(figsize=(7, 9.5))
    for pn in dr["panels"]:
        c, L = pn["center"], pn["length"] / 2
        xs, ys = ([c[0] - L, c[0] + L], [c[1]] * 2) if pn["axis"] == "x" else ([c[0]] * 2, [c[1] - L, c[1] + L])
        ax.plot(xs, ys, color="r" if pn["kind"] == "blocked" else "k", lw=2)
    for b in lay["boxes"]:
        cx, cy, _ = b["center"]
        hx, hy = b["size"][0] / 2, b["size"][1] / 2
        c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
        ax.add_patch(Polygon([(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))],
                             fc="#e8d9b5", ec="gray"))
    for p in lay["pucks"]:
        ax.plot(*p["center"][:2], "s" if p["shape"] == "cuboid" else "o", ms=4 if p["standing"] else 7,
                color="#c0201a" if p["color"] == "red" else "#1a8a30")
    for t in lay["place_tags"]:
        ax.plot(*t["center"][:2], "b^")
        ax.text(t["center"][0], t["center"][1] + 0.08, f"tag {t['id']}", fontsize=7, color="b")
    bc, sl, rb = lay["barcodes"][0], lay["signal_light"], lay["robot"]
    ax.plot(*bc["center"][:2], "m|", ms=14)
    ax.text(0.1, bc["center"][1], f"barcode z {bc['center'][2]:.2f}", fontsize=7, color="m")
    ax.plot(sl["x"], sl["y"], "y*", ms=12)
    ax.arrow(rb["x"], rb["y"], 0.25 * math.cos(rb["yaw"]), 0.25 * math.sin(rb["yaw"]), width=0.02, color="c")
    ax.set_aspect("equal")
    ax.set_title(f"dice_mission seed {lay['seed']} (sim, 판정 아님) — 신호 {sl['sequence']}", fontsize=9)
    fig.tight_layout()
    fig.savefig(png, dpi=110)
