"""저장소 rail: 옛 입력 STEP(mine/CADai/CAD/Input_References, 2026-10-20)·조립 STEP 고체(Assembly_Modular_261021)와
새로 받은 STEP 6개(~/snap/slack/261/Desktop/STEP/rail_*, 2026-10-24)의 고체 수·외곽·부피를 비교한다(정적 계산, 시험 아님).
사용(PC): ~/venvs/cad/bin/python tools/rail_compare.py
결과: results/rail_compare.json
"""
import json
from pathlib import Path

import cadquery as cq

H = Path(__file__).resolve().parents[1]
TOP = H.parent
NEW = Path.home() / "snap/slack/261/Desktop/STEP"
FILES = {f"old_input/{n}": TOP / f"mine/CADai/CAD/Input_References/{n}.step" for n in ("rail_hi", "rail_lo")}
FILES.update({f"new/{n}": NEW / f"{n}.step" for n in ("rail_hi_1EA", "rail_hi_L_1EA", "rail_hi_R_1EA", "rail_lo_1EA", "rail_lo_L_1EA", "rail_lo_R_1EA")})
out = {}
for key, path in FILES.items():
    shape = cq.importers.importStep(str(path)).val()
    solids = shape.Solids()
    bb = shape.BoundingBox()
    out[key] = {"file": str(path), "solids": len(solids),
                "volume_cm3": round(sum(s.Volume() for s in solids) / 1000, 2),
                "size_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
                "min_mm": [round(bb.xmin, 2), round(bb.ymin, 2), round(bb.zmin, 2)],
                "max_mm": [round(bb.xmax, 2), round(bb.ymax, 2), round(bb.zmax, 2)],
                "solid_sizes_mm": [[round(s.BoundingBox().xlen, 1), round(s.BoundingBox().ylen, 1), round(s.BoundingBox().zlen, 1)] for s in solids][:6]}
    print(key, json.dumps(out[key], ensure_ascii=False))
b = {s["i"]: s for s in json.loads((H / "asm/solids_bounds.json").read_text())}
for sid, nm in ((32, "rail_hi"), (33, "rail_lo")):
    s = b[sid]
    out[f"old_assembly/{nm}"] = {"solid": sid, "volume_cm3": round(s["vol"] / 1000, 2),
                                 "size_mm": [round(s["bb"][3] - s["bb"][0], 2), round(s["bb"][4] - s["bb"][1], 2), round(s["bb"][5] - s["bb"][2], 2)],
                                 "min_mm": s["bb"][:3], "max_mm": s["bb"][3:]}
    print(f"old_assembly/{nm}", out[f"old_assembly/{nm}"])
(H / "results").mkdir(exist_ok=True)
(H / "results/rail_compare.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
