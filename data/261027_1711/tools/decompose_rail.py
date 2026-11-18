"""새 rail 충돌 형상: CoACD 볼록 분해 → 패키지 meshes/collision/base_link_rail_{hi,lo}_new_hullNN.stl(m, base_link 좌표).
외곽 상자 하나로 넣었더니 MoveIt에서 접은 팔(arm_link_2)과 겹쳐(상자가 rail의 빈 곳까지 채움) 모양을 따르게 바꾼다.
옛 rail 볼록 분해(part032·033, 32개씩)는 URDF에서 뺀다(tools/make_revision_261027_1711.py).
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/decompose_rail.py
"""
import json
from pathlib import Path

import coacd
import numpy as np
import trimesh

W = Path(__file__).resolve().parents[1]
H = W.parent / "261027_1057"
OUT = W / "build_base/rail_hulls"
OUT.mkdir(parents=True, exist_ok=True)
rep = {}
for name in ("rail_hi", "rail_lo"):
    m = trimesh.load(H / f"parts_storage/{name}_new.stl")   # mm 그대로 넣는다(m 단위로 넣으면 CoACD 교차 검사 assert로 멈춤)
    m.merge_vertices()
    parts = coacd.run_coacd(coacd.Mesh(m.vertices.astype(np.float64), m.faces.astype(np.int64)), threshold=0.03,
                            max_convex_hull=32, preprocess_mode="on", seed=0)
    m.apply_scale(1e-3)
    files = []
    for k, (v, f) in enumerate(parts):
        h = trimesh.Trimesh(np.asarray(v) * 1e-3, f).convex_hull
        fn = f"base_link_{name}_new_hull{k:02d}.stl"
        h.export(OUT / fn)
        files.append(fn)
    hv = sum(trimesh.load(OUT / fn).volume for fn in files)
    rep[name] = {"hulls": len(files), "files": files, "mesh_volume_cm3": m.volume * 1e6, "hull_volume_cm3": hv * 1e6}
    print(name, f"볼록 {len(files)}개, 부피 rail {m.volume * 1e6:.1f} · 볼록 합 {hv * 1e6:.1f} cm³")
(OUT / "decompose_report.json").write_text(json.dumps(rep, indent=1))
