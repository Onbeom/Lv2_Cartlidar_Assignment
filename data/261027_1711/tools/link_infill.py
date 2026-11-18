"""base_link 밖 링크의 출력 부품 채움률 반영(바퀴 허브 4, 라이다 받침 2, 팔 끝 카메라 브라켓 1). 정적 계산.
링크 관성(261027_1530 URDF 값)에서 꽉 찬 부품 질량의 (1 − f)를 같은 모양으로 뺀다(f: tools/infill.py).
부품 자리: CAD 조립 고체(base_link 좌표 = CAD 세계 좌표) → 261022_2119 URDF 0 자세 FK로 링크 좌표(바퀴는 그 리비전 자리 기준이라 링크 좌표 값은 같음).
결과: build_base/link_inertials.json
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/link_infill.py
"""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

W = Path(__file__).resolve().parents[1]
TOP = W.parent
H = TOP / "261027_1057"
sys.path.insert(0, str(W / "tools"))
sys.path.insert(0, str(TOP / "261025_2117/rl1_arm_reach/tools"))
from infill import fill_fraction  # noqa: E402
from urdf_fk import UrdfFK  # noqa: E402

rows = json.loads((TOP / "mine/CADai/Data/link_component_body_map.json").read_text())["rows"]
PARTS = {5: 5, 7: 7, 42: 37, 46: 41, 523: 211, 525: 213, 664: 352}   # 행 → 조립 고체
OLD_URDF = TOP / "261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf"
CUR_URDF = TOP / "261027_1530/urdf/waffle_4wd_description_261027_1530/urdf/waffle_4wd.urdf"
fk = UrdfFK(str(OLD_URDF), root="base_link")
root = ET.parse(CUR_URDF).getroot()


def inertial(link):
    el = root.find(f"link[@name='{link}']/inertial")
    m = float(el.find("mass").get("value"))
    c = np.array([float(v) for v in el.find("origin").get("xyz").split()])
    rpy = el.find("origin").get("rpy", "0 0 0")
    assert all(abs(float(v)) < 1e-12 for v in rpy.split()), (link, rpy)
    a = el.find("inertia").attrib
    I = np.array([[a["ixx"], a["ixy"], a["ixz"]], [a["ixy"], a["iyy"], a["iyz"]], [a["ixz"], a["iyz"], a["izz"]]], float)
    return m, c, I


def shift(m, c, I_c, to):   # 무게중심 관성 → 점 to 기준
    d = c - to
    return I_c + m * (d @ d * np.eye(3) - np.outer(d, d))


out = {}
for r, sid in PARTS.items():
    link = rows[r]["link"]
    T = fk.link_transforms({}, [link])[link][0].copy()
    sm = trimesh.load(H / "asm/solids" / f"s{sid:03d}.stl")
    f = fill_fraction(sm)
    sm.apply_scale(1e-3)
    sm.apply_transform(np.linalg.inv(T))         # 링크 좌표(m)
    if sm.volume < 0:
        sm.invert()
    dm = rows[r]["mass_g"] / 1000 * (1 - f)       # 빼는 질량
    sm.density = dm / sm.volume
    cp, Ip = sm.center_mass, sm.moment_inertia   # 빼는 부분의 무게중심·관성(무게중심 기준)
    m0, c0, I0 = out[link]["new"] if link in out else inertial(link)
    m1 = m0 - dm
    c1 = (m0 * c0 - dm * cp) / m1
    I1 = shift(m0, c0, I0, c1) - shift(dm, cp, Ip, c1)
    out[link] = {"new": (m1, c1, I1), "old": out.get(link, {}).get("old", inertial(link)), "part_rows": out.get(link, {}).get("part_rows", []) + [r],
                 "fill_fraction": f}
res = {}
for link, v in out.items():
    m1, c1, I1 = v["new"]
    m0 = v["old"][0]
    res[link] = {"mass_kg": m1, "com_m": c1.tolist(), "inertia_com_kg_m2": I1.tolist(), "old_mass_kg": m0,
                 "eig": np.linalg.eigvalsh(I1).tolist(), "rows": v["part_rows"], "fill_fraction": v["fill_fraction"]}
    print(f"{link}: {m0 * 1000:.1f} → {m1 * 1000:.1f} g, 무게중심 {np.round(v['old'][1] * 1000, 2).tolist()} → {np.round(c1 * 1000, 2).tolist()} mm, "
          f"관성 고윳값 최소 {min(res[link]['eig']):.3e}")
(W / "build_base/link_inertials.json").write_text(json.dumps(res, indent=1))
