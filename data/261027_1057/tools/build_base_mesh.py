"""새 리비전 base_link 표시 메시와 질량 성질을 만든다(URDF base_link 좌표).

1. 지금 base_link.dae(261022_2119)에서 바꿀 부품의 삼각형을 뺀다: 조립 STEP 고체(asm/solids)의 표면 위(0.05 mm 안)에 있고
   법선이 같은 쪽인 삼각형만(맞닿은 다른 부품 면은 법선이 반대라 남음). 뺀 면적을 고체 표면적과 비교해 확인한다.
2. 새 부품을 더한다: 긴 브라켓 후보 A(v3 방향), 저장소 housing·magazine(구멍판), 옮긴 모터 4개, 증속 벨트 모듈 4개, 무게추.
3. 질량 성질: 옛 base_link(질량·무게중심·관성)에서 뺀 부품을 빼고 새 부품을 더한다. 밀도: CAD와 같은 PLA 1.24 g/cm³(채움 100 %),
   모터 82 g(CAD XM430 값), 강철 7.85 g/cm³, 688ZZ 3.5 g(추정), 벨트 1.2 g/cm³(추정).
결과: <out>/base_link.dae, <out>/base_link_mass.json, <out>/build_report.json
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/build_base_mesh.py <out 폴더>
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

H = Path(__file__).resolve().parents[1]
TOP = H.parent
OLD_DAE = TOP / "261022_2119/urdf/waffle_4wd_description_261022_2119/meshes/main/base_link.dae"
OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
rows = json.loads((TOP / "mine/CADai/Data/link_component_body_map.json").read_text())["rows"]
r2s = {int(k): v for k, v in json.loads((H / "asm/row_to_solid.json").read_text()).items()}
REMOVE_ROWS = [0, 1, 2, 3, 39, 40, 43, 44,          # 바퀴 모터 4개(본체 2개씩)
               33, 71, 72, 34, 35, 36,              # housing 3, magazine 3
               665, 666, 667, 668]                  # 긴 브라켓 4
OLD = {"mass": 3.7218030224487602, "com": [-0.053547954996508128, -2.2024531309009041e-05, 0.14667466611266705],
       "I": [[0.11556446013925142, -0.00019271830760513178, 0.025205750302463154],
             [-0.00019271830760513178, 0.11477421546923385, 0.0001088149297917809],
             [0.025205750302463154, 0.0001088149297917809, 0.053620011264812958]]}


def inertia_origin(m_kg, com_m, I_com):
    c = np.asarray(com_m)
    return np.asarray(I_com) + m_kg * (c @ c * np.eye(3) - np.outer(c, c))


def props(mesh_mm, mass_kg):
    """메시(mm)와 질량(kg) → (질량, 무게중심 m, 원점 기준 관성 kg·m²). 밀도는 질량/부피로 고르게."""
    m = mesh_mm.copy()
    m.apply_scale(1e-3)
    if m.volume < 0:
        m.invert()
    m.density = mass_kg / m.volume
    return mass_kg, m.center_mass, inertia_origin(mass_kg, m.center_mass, m.moment_inertia)


# ---------- 1. 옛 메시에서 빼기 ----------
dae = trimesh.load(OLD_DAE, force="mesh")
dae.apply_scale(1000.0)
cent = dae.triangles_center
nrm = dae.face_normals
remove = np.zeros(len(dae.faces), bool)
rep = {"removed": []}
tot = {"mass": 0.0, "m1": np.zeros(3), "Io": np.zeros((3, 3))}
removed_props = []
for r in REMOVE_ROWS:
    sid = r2s[r]
    sm = trimesh.load(H / "asm/solids" / f"s{sid:03d}.stl")
    lo, hi = sm.bounds[0] - 0.5, sm.bounds[1] + 0.5
    cand = np.where(np.all((cent > lo) & (cent < hi), axis=1))[0]
    hit = np.zeros(len(cand), bool)
    if len(cand):
        cp, dist, tid = trimesh.proximity.closest_point(sm, cent[cand])
        dot = np.einsum("ij,ij->i", nrm[cand], sm.face_normals[tid])
        hit = (dist < 0.05) & (dot > 0.5)
    remove[cand[hit]] = True
    area_hit = float(dae.area_faces[cand[hit]].sum())
    rep["removed"].append({"row": r, "path": rows[r]["path"], "body": rows[r]["body"], "solid": sid,
                           "area_removed_mm2": area_hit, "area_solid_mm2": float(sm.area), "mass_g": rows[r]["mass_g"]})
    removed_props.append(props(sm, rows[r]["mass_g"] / 1000.0))
    print(f"row {r:3d} {rows[r]['path'][:34]:34s} 면적 뺌 {area_hit:9.1f} / 고체 {sm.area:9.1f} mm²", flush=True)
# 옛 바퀴 모터가 차지하던 공간 안에 남은 면(부품표에 따로 없는 혼 · 나사 등)도 뺀다 — 모터가 옮겨져 그 자리는 베어링 블록이 차지
for r in (1, 2, 40, 43):                            # 모터 본체(혼 포함 외곽)
    b = rows[r]["bounds_world_mm"]
    lo, hi = np.asarray(b["min"]) + 0.05, np.asarray(b["max"]) - 0.05
    extra = np.all((cent > lo) & (cent < hi), axis=1) & ~remove
    rep.setdefault("removed_in_old_motor_box", []).append({"row": r, "faces": int(extra.sum()), "area_mm2": float(dae.area_faces[extra].sum())})
    remove |= extra
    print(f"옛 모터 공간(row {r}) 남은 면 {int(extra.sum())}개, {dae.area_faces[extra].sum():.1f} mm² 뺌", flush=True)
kept = dae.submesh([np.where(~remove)[0]], append=True)

# ---------- 2. 새 부품 ----------
add = []   # (이름, 메시 mm, 질량 kg, 재료)
PLA = 1.24e-3   # g/mm³
pv3 = json.loads((H / "placed_v3/placement.json").read_text())
for name, e in pv3.items():
    src = e["source_print"]
    m = trimesh.load(H / "out_TO_A" / f"{src}_TO.stl")
    m.apply_transform(np.asarray(e["T_print_to_base_link_mm"]))
    add.append((f"{name}_TO_v3", m, m.volume * PLA / 1000, "PLA"))
for f in sorted((H / "parts_storage").glob("*.stl")):
    if f.stem.startswith("rail"):
        continue                                   # rail은 받은 STEP과 모양이 달라 옛 것을 그대로 둠(사용자 확인 대기)
    m = trimesh.load(f)
    add.append((f.stem, m, m.volume * PLA / 1000, "PLA"))
belt = json.loads((H / "parts_belt/manifest.json").read_text())["parts"]
DENS = {"PLA": PLA, "steel": 7.85e-3, "steel_block": 7.85e-3, "rubber": 1.2e-3}
for p in belt:
    m = trimesh.load(H / "parts_belt" / p["file"])
    if p["material"] == "XM430":
        mass = rows[1]["mass_g"] + rows[0]["mass_g"]          # CAD 바퀴 모터 본체 2개 합(82.0 g)
    elif p["material"] == "bearing":
        mass = 3.5                                            # 688ZZ 추정
    else:
        mass = m.volume * DENS[p["material"]]
    add.append((f"{p['name']}_{p['corner']}", m, mass / 1000, p["material"]))

# ---------- 3. 질량 성질 ----------
M0, c0 = OLD["mass"], np.asarray(OLD["com"])
Io = inertia_origin(M0, c0, OLD["I"])
m1 = M0 * c0
for mk, ck, Ik in removed_props:
    M0 -= mk; m1 -= mk * ck; Io -= Ik
for _, mesh, mk, _ in add:
    mk_, ck, Ik = props(mesh, mk)
    M0 += mk_; m1 += mk_ * ck; Io += Ik
com = m1 / M0
Icom = Io - M0 * (com @ com * np.eye(3) - np.outer(com, com))
mass_out = {"mass_kg": M0, "com_m": com.tolist(), "inertia_com_kg_m2": Icom.tolist(),
            "old": OLD, "removed_mass_kg": sum(p[0] for p in removed_props), "added_mass_kg": sum(a[2] for a in add),
            "eig_inertia": np.linalg.eigvalsh(Icom).tolist(),
            "added": [{"name": a[0], "mass_g": a[2] * 1000, "material": a[3]} for a in add]}
(OUT / "base_link_mass.json").write_text(json.dumps(mass_out, indent=1, ensure_ascii=False))
print(f"질량 {OLD['mass']:.4f} → {M0:.4f} kg (뺌 {mass_out['removed_mass_kg']:.4f}, 더함 {mass_out['added_mass_kg']:.4f}), "
      f"무게중심 {np.round(com, 4).tolist()}, 관성 고윳값 {np.round(mass_out['eig_inertia'], 5).tolist()}", flush=True)

# ---------- 4. 내보내기 ----------
parts = [kept] + [a[1] for a in add]
new = trimesh.util.concatenate(parts)
new.apply_scale(1e-3)
new.export(OUT / "base_link.dae")
rep.update({"faces_old": int(len(dae.faces)), "faces_removed": int(remove.sum()), "faces_new": int(len(new.faces)),
            "added": [a[0] for a in add], "bounds_new_m": new.bounds.tolist()})
(OUT / "build_report.json").write_text(json.dumps(rep, indent=1, ensure_ascii=False))
print("saved", OUT / "base_link.dae", "faces", len(new.faces), "bounds", np.round(new.bounds, 4).tolist())
