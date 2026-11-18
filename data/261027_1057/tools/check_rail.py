"""새 rail(옛 자리, 깊이 87 mm) 정적 간섭·간격 검사(모의, 정적). 261027_1530 base_link 메시에서 옛 rail 면을 빼고,
남은 차체 면 · 팔(0 자세, 접은 자세 S0)의 삼각형 중심이 새 rail 안으로 0.2 mm 넘게 들어가는지 센다. 가장 가까운 거리도 적는다.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/check_rail.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

H = Path(__file__).resolve().parents[1]
PKG = H.parent / "261027_1530/urdf/waffle_4wd_description_261027_1530"
sys.path.insert(0, "/home/pa4/git/lv2_next_module/261025_2117/rl1_arm_reach/tools")
from urdf_fk import UrdfFK  # noqa: E402

base = trimesh.load(PKG / "meshes/main/base_link.dae", force="mesh"); base.apply_scale(1000)
cent, nrm = base.triangles_center, base.face_normals
rm = np.zeros(len(base.faces), bool)
for sid in (32, 33):     # 옛 rail 면 빼기(build_base_mesh.py와 같은 방법)
    sm = trimesh.load(H / "asm/solids" / f"s{sid:03d}.stl")
    lo, hi = sm.bounds[0] - 0.5, sm.bounds[1] + 0.5
    c = np.where(np.all((cent > lo) & (cent < hi), axis=1))[0]
    cp, d, tid = trimesh.proximity.closest_point(sm, cent[c])
    rm[c[(d < 0.05) & (np.einsum("ij,ij->i", nrm[c], sm.face_normals[tid]) > 0.5)]] = True
kept = base.submesh([np.where(~rm)[0]], append=True)
print("옛 rail 면 뺌", int(rm.sum()))
fk = UrdfFK(str(PKG / "urdf/waffle_4wd.urdf"), root="base_link")
ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
ARM_LINKS = ["arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link", "gripper_right_link"]


def arm_mesh(q):
    """팔 표시 메시(base_link mm). URDF <visual><origin>을 링크 변환 뒤에 곱한다(처음 판은 빠뜨림 — 2026-10-27 고침)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(PKG / "urdf/waffle_4wd.urdf").getroot()
    tf = fk.link_transforms({k: np.array([v], float) for k, v in zip(ARM, q)}, ARM_LINKS)
    ms = []
    for ln in ARM_LINKS:
        vo = root.find(f"link[@name='{ln}']/visual/origin")
        Tv = trimesh.transformations.euler_matrix(*[float(v) for v in vo.get("rpy").split()], axes="sxyz")
        Tv[:3, 3] = [float(v) for v in vo.get("xyz").split()]
        T = tf[ln][0] @ Tv
        T[:3, 3] *= 1000
        m = trimesh.load(PKG / f"meshes/main/{ln}.dae", force="mesh")
        m.apply_scale(1000); m.apply_transform(T); ms.append(m)
    return trimesh.util.concatenate(ms)


others = {"차체(옛 rail 뺀 나머지)": kept, "팔 0 자세": arm_mesh([0] * 5), "팔 S0": arm_mesh([0, 1.4, -1.0, -1.3, 0])}
out = {}
for name in ("rail_hi", "rail_lo"):
    r = trimesh.load(H / "parts_storage" / f"{name}_new.stl")
    print(name, "닫힌 메시", r.is_watertight, "범위", np.round(r.bounds, 1).tolist())
    for on, om in others.items():
        lo, hi = r.bounds[0] - 20, r.bounds[1] + 20
        sel = np.where(np.all((om.triangles_center > lo) & (om.triangles_center < hi), axis=1))[0]
        if not len(sel):
            out[f"{name}/{on}"] = {"near_tris": 0}; print(" ", on, "20 mm 안 면 없음"); continue
        d = trimesh.proximity.signed_distance(r, om.triangles_center[sel])   # 안쪽 +
        inside = d > 0.2
        hit_x = om.triangles_center[sel][inside]
        out[f"{name}/{on}"] = {"near_tris": int(len(sel)), "inside_0.2mm": int(inside.sum()), "max_depth_mm": float(d.max()),
                               "min_gap_mm": float(-d[d <= 0].max()) if (d <= 0).any() else None,
                               "inside_bounds_mm": [hit_x.min(0).round(1).tolist(), hit_x.max(0).round(1).tolist()] if inside.any() else None}
        print(" ", on, out[f"{name}/{on}"])
(H / "results/check_rail.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
