"""새 rail(rail_{hi,lo}_1EA)을 옛 rail과 같은 자리에 놓는다(사용자 2026-10-27 "새 rail은 기존과 같은 위치").
옛 입력 STEP(mine/CADai/CAD/Input_References/rail_*.step, 2026-10-20)을 조립 STEP 고체(32·33)로 옮긴 강체 변환을 구하고
(축 뒤집기 8가지 중 표면 거리 중앙값 최소), 같은 입력 좌표계인 새 STEP에 그 변환을 그대로 쓴다.
결과: parts_storage/rail_{hi,lo}_new.stl(base_link mm), parts_storage/rail_placement.json
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/place_rail.py
"""
import itertools
import json
from pathlib import Path

import numpy as np
import trimesh

H = Path(__file__).resolve().parents[1]
res = {}
for name, sid in (("rail_hi", 32), ("rail_lo", 33)):
    old_in = trimesh.load(H / "parts_src" / f"old_input_{name}.stl")
    old_asm = trimesh.load(H / "asm/solids" / f"s{sid:03d}.stl")
    pts, _ = trimesh.sample.sample_surface(old_in, 6000, seed=0)
    best = None
    for sg in itertools.product((1, -1), repeat=3):
        if np.prod(sg) < 0:
            continue                                   # 오른손 좌표만(거울상 제외)
        R = np.diag(sg).astype(float)
        c_in = old_in.bounds.mean(0)
        c_asm = old_asm.bounds.mean(0)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = c_asm - R @ c_in
        q = pts @ R.T + T[:3, 3]
        _, d, _ = trimesh.proximity.closest_point(old_asm, q)
        med = float(np.median(d))
        if best is None or med < best[0]:
            best = (med, T, sg, float(np.percentile(d, 99)))
    med, T, sg, p99 = best
    new = trimesh.load(H / "parts_src" / f"{name}_1EA.stl")
    new.apply_transform(T)
    new.export(H / "parts_storage" / f"{name}_new.stl")
    res[name] = {"T_input_to_base_link_mm": T.tolist(), "axis_signs": list(sg), "fit_median_mm": med, "fit_p99_mm": p99,
                 "old_bounds_mm": old_asm.bounds.tolist(), "new_bounds_mm": new.bounds.tolist(),
                 "new_volume_cm3": new.volume / 1000}
    print(name, "축", sg, f"맞춤 중앙값 {med:.3f} mm p99 {p99:.3f}", "옛", np.round(old_asm.bounds, 1).tolist(), "새", np.round(new.bounds, 1).tolist())
(H / "parts_storage" / "rail_placement.json").write_text(json.dumps(res, indent=1))
