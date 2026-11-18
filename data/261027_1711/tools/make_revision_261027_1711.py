"""261027_1530 URDF 패키지를 복사해 261027_1711 리비전을 만든다(원본은 읽기만).

바꾸는 것(사용자 2026-10-27 17:0x):
 1. 새 rail(rail_hi/lo_1EA, 깊이 87.1 mm)을 옛 rail과 같은 자리에. 충돌은 옛 rail 볼록 분해(convex_32_*·convex_33_*, 32개씩)를 빼고
    새 rail CoACD 볼록 분해(tools/decompose_rail.py, 31·32개)를 넣음. 처음에 쓴 외곽 상자는 rail 빈 곳까지 채워 MoveIt에서 접은 팔과 겹쳐서 바꿈
 2. 앞 무게추(컵 + 강철 블록) 없음: 표시 메시·질량·충돌 상자 ballast_cup에서 뺌(벨트 자료에만 있던 부품)
 3. 출력 부품(PLA) 채움률 반영(껍질 0.8 mm + 채움 15 %, 가정): base_link, 바퀴 4, 라이다 2, arm_link_5
 바꾸지 않은 것: 바퀴 관절 x ±0.08(브라켓 핀 구멍 가운데), Arducam 브라켓 형상·카메라 위치, 나머지 관절·충돌
입력: build_base/base_link.dae · base_link_mass.json(tools/build_base_mesh_1711.py), build_base/link_inertials.json(tools/link_infill.py)
사용(PC): python3 tools/make_revision_261027_1711.py
"""
import json
import re
import shutil
import sys
from pathlib import Path

W = Path(__file__).resolve().parents[1]
TOP = W.parent
SRC = TOP / "261027_1530/urdf/waffle_4wd_description_261027_1530"
DST = W / "urdf/waffle_4wd_description_261027_1711"
BUILD = W / "build_base"
OLD_PKG, NEW_PKG = "waffle_4wd_description_261027_1530", "waffle_4wd_description_261027_1711"
TEXT_SUFFIX = {".xml", ".txt", ".md", ".yaml", ".xacro", ".urdf", ".py", ".json", ".rviz", ".csv"}
STEEL = "--steel" in sys.argv   # 강철 블록은 없는 부품(사용자 2026-10-27). 기본은 넣지 않음

if DST.exists():
    raise SystemExit(f"already exists: {DST}")
shutil.copytree(SRC, DST)
n_pkg = 0
for f in DST.rglob("*"):
    if f.is_file() and f.suffix in TEXT_SUFFIX and "meshes" not in f.parts and not f.name.startswith("REVISION_"):
        t = f.read_text(encoding="utf-8")
        if OLD_PKG in t:
            n_pkg += t.count(OLD_PKG)
            f.write_text(t.replace(OLD_PKG, NEW_PKG), encoding="utf-8")
shutil.copy2(BUILD / "base_link.dae", DST / "meshes/main/base_link.dae")

f17 = lambda v: repr(float(v))   # noqa: E731
base = json.loads((BUILD / "base_link_mass.json").read_text())
links = json.loads((BUILD / "link_inertials.json").read_text())
NEW_INERTIAL = {"base_link": (base["mass_kg"], base["com_m"], base["inertia_com_kg_m2"])}
NEW_INERTIAL.update({k: (v["mass_kg"], v["com_m"], v["inertia_com_kg_m2"]) for k, v in links.items()})


def inertial_xml(indent, m, c, I):
    p = indent
    return (f"<inertial>\n{p}  <origin xyz=\"{f17(c[0])} {f17(c[1])} {f17(c[2])}\" rpy=\"0 0 0\" />\n"
            f"{p}  <mass value=\"{f17(m)}\" />\n"
            f"{p}  <inertia ixx=\"{f17(I[0][0])}\" ixy=\"{f17(I[0][1])}\" ixz=\"{f17(I[0][2])}\" iyy=\"{f17(I[1][1])}\" "
            f"iyz=\"{f17(I[1][2])}\" izz=\"{f17(I[2][2])}\" />\n{p}</inertial>")


def box(name, lo_mm, hi_mm):
    c = [(a + b) / 2000 for a, b in zip(lo_mm, hi_mm)]
    s = [(b - a) / 1000 for a, b in zip(lo_mm, hi_mm)]
    return (f'<collision name="{name}"><origin xyz="{c[0]!r} {c[1]!r} {c[2]!r}" rpy="0 -0 0" />'
            f'<geometry><box size="{s[0]!r} {s[1]!r} {s[2]!r}" /></geometry></collision>')


rail = json.loads((TOP / "261027_1057/parts_storage/rail_placement.json").read_text())
hull_rep = json.loads((BUILD / "rail_hulls/decompose_report.json").read_text())
NEW_BOXES = ""   # 이름은 그대로 두고 rail 볼록 분해를 넣는다(ballast_cup 자리)
for n in ("rail_hi", "rail_lo"):
    for k, fn in enumerate(hull_rep[n]["files"]):
        shutil.copy2(BUILD / "rail_hulls" / fn, DST / "meshes/collision" / fn)
        NEW_BOXES += (f'<collision name="{n}_new_{k}"><origin xyz="0 0 0" rpy="0 0 0" /><geometry><mesh filename="package://{NEW_PKG}/'
                      f'meshes/collision/{fn}" scale="1 1 1" /></geometry></collision>')
OLD_RAIL = re.compile(r'<collision name="convex_3[23]_\d+">.*?</collision>')
STEEL_BOX = box("ballast_steel", (115.1, -85.3, 56.9), (131.1, 4.7, 101.9)) if STEEL else ""   # parts_belt/ballast_steel.stl 외곽
CUP = re.compile(r'<collision name="ballast_cup">.*?</collision>')
for rel in ("urdf/waffle_4wd.urdf", "urdf/assemblies/main.urdf.xacro"):
    p = DST / rel
    t = p.read_text(encoding="utf-8")
    for link, (m, c, I) in NEW_INERTIAL.items():
        mo = re.search(rf'<link name="(?:\$\{{prefix\}})?{link}">\s*\n(\s*)<inertial>.*?</inertial>', t, re.S)
        if not mo:
            raise SystemExit(f"{rel}: {link} inertial 없음")
        start = mo.start() + mo.group(0).index("<inertial>")
        t = t[:start] + inertial_xml(mo.group(1), m, c, I) + t[mo.end():]
    if len(CUP.findall(t)) != 1:
        raise SystemExit(f"{rel}: ballast_cup 충돌 {len(CUP.findall(t))}곳")
    t = CUP.sub(STEEL_BOX + NEW_BOXES, t)
    n_old = len(OLD_RAIL.findall(t))
    if n_old != 64:
        raise SystemExit(f"{rel}: 옛 rail 볼록 {n_old}개(예상 64)")
    t = OLD_RAIL.sub("", t)
    p.write_text(t, encoding="utf-8")

rd = DST / "robot_data.yaml"
d = json.loads(rd.read_text(encoding="utf-8"))
dm = 0.0
for link, (m, c, I) in NEW_INERTIAL.items():
    L = d["links"][link]
    dm += m - L["mass_kg"]
    L["mass_kg"] = m
    L["com_link_m"] = " ".join(f17(v) for v in c)
    L["inertia_com_link_kg_m2"] = {"ixx": f17(I[0][0]), "ixy": f17(I[0][1]), "ixz": f17(I[0][2]),
                                   "iyy": f17(I[1][1]), "iyz": f17(I[1][2]), "izz": f17(I[2][2])}
d["metadata"]["total_mass_kg"] += dm
rd.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
sm = DST / "simulation_metadata.json"
s = json.loads(sm.read_text(encoding="utf-8"))
s["print_infill_assumption"] = {**base["infill_assumption"], "note": "estimated, not measured (user 2026-10-27)"}
sm.write_text(json.dumps(s, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps({"dst": str(DST), "steel": STEEL, "package_renames": n_pkg, "total_mass_kg": d["metadata"]["total_mass_kg"],
                  "delta_mass_kg": dm, "links": {k: v[0] for k, v in NEW_INERTIAL.items()}}, ensure_ascii=False, indent=1))
