"""앞 무게추 실험(모의)용 URDF 사본: 261027_1711 base_link 관성에 무게추(상자)를 더한 것. 원본 패키지는 바꾸지 않는다.

무게추 자리: 증속 벨트 자료의 자리(상판 아래 앞쪽 오른쪽, 라이다 받침을 피한 칸) — 외곽 x 115.1~131.1, y −85.3~4.7, z 56.9~101.9 mm(16 × 90 × 45).
질량: 인자로 받음(기본 0.5 · 1.0 · 1.5 kg, 촘촘한 추가 0.25 · 0.75 · 1.25 kg). 0.5 kg은 벨트 자료의 강철 509 g과 같은 크기. 1.0 · 1.5 kg은 같은 상자에 넣을 수 없는 밀도라 '그 자리에 모인 질량'으로만 본다.
충돌 · 표시 형상은 넣지 않는다(상판 아래 안쪽이라 바깥 외곽이 바뀌지 않음).
결과: variants/m{05,10,15}/waffle_4wd_description_261027_1711/ (meshes는 원본에 심볼릭 링크),
      variants/m{05,10,15}/waffle_4wd_mujoco_drive.urdf (MuJoCo 주행 시험용, 메시 절대 경로)
사용(PC): python3 tools/make_ballast_variants.py
"""
import json
import sys
import re
import shutil
from pathlib import Path

import numpy as np

W = Path(__file__).resolve().parents[1]
TOP = W.parent
SRC = TOP / "261027_1711/urdf/waffle_4wd_description_261027_1711"
MJ_SRC = TOP / "261027_1744/mujoco/drive/waffle_4wd_mujoco_drive.urdf"
LO, HI = np.array([115.1, -85.3, 56.9]) / 1000, np.array([131.1, 4.7, 101.9]) / 1000
C_B, S_B = (LO + HI) / 2, HI - LO
# base_link 좌표 = base_footprint + z 0.00675. 무게추 자리는 base_link 좌표(CAD)라 그대로 쓴다
f17 = lambda v: repr(float(v))  # noqa: E731
rd = json.loads((SRC / "robot_data.yaml").read_text())
bl = rd["links"]["base_link"]
m0 = bl["mass_kg"]
c0 = np.array([float(v) for v in bl["com_link_m"].split()])
I = bl["inertia_com_link_kg_m2"]
I0 = np.array([[I["ixx"], I["ixy"], I["ixz"]], [I["ixy"], I["iyy"], I["iyz"]], [I["ixz"], I["iyz"], I["izz"]]], float)


def shift(m, c, Ic, to):
    d = c - to
    return Ic + m * (d @ d * np.eye(3) - np.outer(d, d))


out = {}
MASSES = [float(a) for a in sys.argv[1:]] or [0.5, 1.0, 1.5]
for mb in MASSES:
    Ib = mb / 12 * np.diag([S_B[1] ** 2 + S_B[2] ** 2, S_B[0] ** 2 + S_B[2] ** 2, S_B[0] ** 2 + S_B[1] ** 2])
    m1 = m0 + mb
    c1 = (m0 * c0 + mb * C_B) / m1
    I1 = shift(m0, c0, I0, c1) + shift(mb, C_B, Ib, c1)
    new_inertial = (f'<origin xyz="{f17(c1[0])} {f17(c1[1])} {f17(c1[2])}" rpy="0 0 0" />\n      <mass value="{f17(m1)}" />\n'
                    f'      <inertia ixx="{f17(I1[0, 0])}" ixy="{f17(I1[0, 1])}" ixz="{f17(I1[0, 2])}" iyy="{f17(I1[1, 1])}" '
                    f'iyz="{f17(I1[1, 2])}" izz="{f17(I1[2, 2])}" />')
    tag = f"g{round(mb * 1000):04d}"   # 처음 6개는 m05 식 이름(0.1 kg 단위)으로 만들었음
    vd = W / "variants" / tag
    pkg = vd / SRC.name
    if pkg.exists():
        shutil.rmtree(pkg)
    pkg.mkdir(parents=True)
    for f in SRC.iterdir():                        # meshes 등은 링크, urdf 폴더만 복사해 고친다
        if f.name == "urdf":
            shutil.copytree(f, pkg / "urdf")
        else:
            (pkg / f.name).symlink_to(f)
    for p in (pkg / "urdf/waffle_4wd.urdf", vd / "waffle_4wd_mujoco_drive.urdf"):
        t = (pkg / "urdf/waffle_4wd.urdf" if p.name == "waffle_4wd.urdf" else MJ_SRC).read_text()
        if p.name != "waffle_4wd.urdf":       # MuJoCo 사본: 메시 경로를 절대 경로로
            t = t.replace("../../../261027_1711/", str(TOP / "261027_1711") + "/")
        t, n = re.subn(r'(<link name="base_link">\s*<inertial>\s*)<origin[^>]*/>\s*<mass[^>]*/>\s*<inertia[^>]*/>', r"\g<1>" + new_inertial.replace("\\", "\\\\"), t, count=1)
        assert n == 1, (p, n)
        p.write_text(t)
    out[tag] = {"ballast_kg": mb, "ballast_center_m": C_B.tolist(), "base_link_mass_kg": m1, "base_link_com_m": c1.tolist(),
                "total_mass_kg": rd["metadata"]["total_mass_kg"] + mb}
    print(tag, f"base_link {m0:.4f} → {m1:.4f} kg, 무게중심 x {c0[0] * 1000:.1f} → {c1[0] * 1000:.1f} mm, 전체 {out[tag]['total_mass_kg']:.4f} kg")
vj = W / "variants/variants.json"
old = json.loads(vj.read_text()) if vj.exists() else {}
old.update(out)
vj.write_text(json.dumps(old, indent=1))
