"""새 리비전 정적 간섭 검사(모의, 기본 자세): 다른 형상의 면이 새 부품(닫힌 메시) 안으로 0.2 mm 넘게 들어간 삼각형 수를 센다.
비교 대상: ① 그대로 둔 차체 메시(옛 base_link에서 뺀 나머지) ② 바퀴 4개(새 관절 위치) ③ 새 부품끼리. 면 접촉(0 mm)은 간섭으로 세지 않는다.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/check_interference.py <새 패키지 폴더>
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

H = Path(__file__).resolve().parents[1]
PKG = Path(sys.argv[1])
sys.path.insert(0, "/home/pa4/git/lv2_next_module/261025_2117/rl1_arm_reach/tools")
from urdf_fk import UrdfFK  # noqa: E402

old = trimesh.load(H.parent / "261022_2119/urdf/waffle_4wd_description_261022_2119/meshes/main/base_link.dae", force="mesh")
old.apply_scale(1000)
rep = json.loads((H / "build_base/build_report.json").read_text())
# 그대로 둔 부분 = 새 메시의 앞부분(빼고 남은 면 수만큼)
newm = trimesh.load(PKG / "meshes/main/base_link.dae", force="mesh"); newm.apply_scale(1000)
n_kept = rep["faces_old"] - rep["faces_removed"]
kept = newm.submesh([np.arange(n_kept)], append=True)
fk = UrdfFK(str(PKG / "urdf/waffle_4wd.urdf"), root="base_link")
wheels = []
for w in ("wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"):
    m = trimesh.load(PKG / f"meshes/main/{w}.dae", force="mesh")
    Tw = fk.link_transforms({}, [w])[w][0].copy(); Tw[:3, 3] *= 1000
    m.apply_scale(1000); m.apply_transform(Tw); wheels.append((w, m))
parts = []
for f in sorted((H / "parts_belt").glob("*.stl")) + sorted((H / "parts_storage").glob("*.stl")):
    if f.stem.startswith("rail"):
        continue
    parts.append((f.stem, trimesh.load(f)))
pv3 = json.loads((H / "placed_v3/placement.json").read_text())
for name, e in pv3.items():
    m = trimesh.load(H / "out_TO_A" / f"{e['source_print']}_TO.stl"); m.apply_transform(np.asarray(e["T_print_to_base_link_mm"]))
    parts.append((name + "_TO_v3", m))


def inside(part, pts):
    lo, hi = part.bounds[0] - 1, part.bounds[1] + 1
    sel = np.where(np.all((pts > lo) & (pts < hi), axis=1))[0]
    if not len(sel):
        return 0, 0.0
    d = trimesh.proximity.signed_distance(part, pts[sel])     # 안쪽 +
    k = d > 0.2
    return int(k.sum()), float(d[k].max()) if k.any() else 0.0


res = []
kc = kept.triangles_center
for name, part in parts:
    if not part.is_watertight:
        res.append({"part": name, "note": "닫힌 메시 아님 — 검사 생략"}); continue
    n1, d1 = inside(part, kc)
    nw = [(w, *inside(part, wm.triangles_center)) for w, wm in wheels]
    no = [(on, *inside(part, om.triangles_center)) for on, om in parts if on != name]
    hitw = [x for x in nw if x[1]]
    hito = [x for x in no if x[1]]
    res.append({"part": name, "kept_tris_inside": n1, "kept_max_depth_mm": round(d1, 2),
                "wheel_hits": hitw, "part_hits": hito})
    if n1 or hitw or hito:
        print(name, "차체", n1, round(d1, 2), "바퀴", hitw, "부품", hito, flush=True)
(PKG.parent.parent / "interference_check.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print("검사 부품", len(parts), "간섭 있는 부품", sum(1 for r in res if r.get("kept_tris_inside") or r.get("wheel_hits") or r.get("part_hits")))
