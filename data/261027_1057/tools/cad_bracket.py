"""cuts.json(조립 좌표 구멍 다각형, 왼쪽)을 원본 STEP에서 깎아 새 STEP·STL을 만든다(오른쪽은 y 거울).

원본: mine/CADai/CAD/Print/Floor_Bracket_{Left,Right}_{Main,Profile_Carrier}.step(출력 자세 좌표, 261021)
변환: placed/placement.json의 출력 자세 → 조립(base_link mm) 4×4. 깎은 뒤 출력 자세로 되돌려 저장(원본과 같은 좌표).
결과: <out>/Floor_Bracket_*_TO.step · .stl, cad_report.json(부피, 외곽, 아랫면 기울기 검사)
사용(PC): ~/venvs/cad/bin/python tools/cad_bracket.py results/to_p15 <out 폴더>
"""
import json
import sys
from pathlib import Path

import cadquery as cq
import numpy as np

R = Path(sys.argv[1])
OUT = Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
TOP = Path(__file__).resolve().parents[2]
SRC = TOP / "mine/CADai/CAD/Print"
PL = json.loads((Path(__file__).resolve().parents[1] / "placed/placement.json").read_text())
cuts = json.loads((R / "cuts.json").read_text())


def prism(poly, axis, rng):
    """판 다각형(2D a, b)을 축 방향으로 rng 범위만큼 돌출한 입체(조립 좌표)."""
    lo, hi = rng
    pts = []
    for a, b in poly:
        if axis == 1:
            pts.append(cq.Vector(a, lo, b))          # web: (x, z), y 방향
        elif axis == 2:
            pts.append(cq.Vector(a, b, lo))          # flange: (x, y), z 방향
        else:
            pts.append(cq.Vector(lo, a, b))          # carrier 뒷벽: (y, z), x 방향
    w = cq.Wire.makePolygon(pts, close=True)
    f = cq.Face.makeFromWires(w)
    d = [0, 0, 0]
    d[axis] = hi - lo
    return cq.Solid.extrudeLinear(f, cq.Vector(*d))


def move(shape, T):
    """강체 변환(4 × 4). cq.Matrix → GTrsf 변환은 이 회전에서 '직교 아님' 오류가 나서 gp_Trsf를 직접 쓴다."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*[float(T[i][j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


tools_main, tools_carr = [], []
for name, p in cuts["plates"].items():
    for poly in p["polygons"]:
        s = prism(poly, p["axis"], p["range"])
        (tools_carr if name == "cwall" else tools_main).append(s)
print(f"cut prisms: Main {len(tools_main)}, Carrier {len(tools_carr)}", flush=True)
report = {"source": str(SRC), "cuts": str(R / "cuts.json"), "parts": {}}
for side in ("Left", "Right"):
    for part, tl in (("Main", tools_main), ("Profile_Carrier", tools_carr)):
        name = f"Floor_Bracket_{side}_{part}"
        T = np.array(PL[name]["T_print_to_base_link_mm"])
        orig = cq.importers.importStep(str(SRC / f"{name}.step")).val()
        asm = move(orig, T)
        new_asm = asm
        for t in tl:                                 # 하나씩 차례로 깎음(묶음으로 한 번에 깎으면 얇은 조각·형상 오류가 생김)
            new_asm = new_asm.cut(t.mirror("XZ") if side == "Right" else t)    # 오른쪽은 y → −y
        new = move(new_asm, np.linalg.inv(T))
        new = new.clean()
        solids = new.Solids()
        if len(solids) > 1:                          # 불리언에서 생긴 부피 0에 가까운 조각 버림(가장 큰 것만)
            dropped = sorted(x.Volume() for x in solids)[:-1]
            new = max(solids, key=lambda x: x.Volume())
            print(f"  {name}: 작은 조각 {len(dropped)}개 버림, 부피 {[round(v, 4) for v in dropped]} mm³", flush=True)
        if not new.isValid():
            new = new.fix()
        solids = new.Solids()
        cq.exporters.export(cq.Workplane().add(new), str(OUT / f"{name}_TO.step"))
        cq.exporters.export(cq.Workplane().add(new), str(OUT / f"{name}_TO.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(cq.Workplane().add(new_asm), str(OUT / f"{name}_TO_assembled_mm.stl"), tolerance=0.02, angularTolerance=0.1)
        bb0, bb1 = orig.BoundingBox(), new.BoundingBox()
        report["parts"][name] = {"volume_orig_cm3": orig.Volume() / 1000, "volume_new_cm3": new.Volume() / 1000,
                                 "solids": len(solids), "valid": bool(new.isValid()),
                                 "bbox_orig": [bb0.xmin, bb0.ymin, bb0.zmin, bb0.xmax, bb0.ymax, bb0.zmax],
                                 "bbox_new": [bb1.xmin, bb1.ymin, bb1.zmin, bb1.xmax, bb1.ymax, bb1.zmax]}
        print(name, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report["parts"][name].items() if "bbox" not in k}, flush=True)
(OUT / "cad_report.json").write_text(json.dumps(report, indent=2))
