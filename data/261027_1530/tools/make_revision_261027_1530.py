"""261022_2119 URDF 패키지를 복사해 261027_1530 리비전을 만든다(원본은 읽기만).

바꾸는 것(사용자 2026-10-27, URDF 수정 5가지 중 1~4 — 5는 USD):
 1. 층간 긴 브라켓: 위상최적화 후보 A, v3 방향(실제 장착 방향, 원래 URDF와 위아래 반대)
 2. 저장소 housing 3·magazine 3: 구멍판(사용자 STEP). rail 2개는 받은 STEP이 모양이 달라 그대로(확인 대기)
 3. Arducam(팔 카메라) 브라켓: 광축이 장착면 기준 15°(카메라 판–장착면 75°) → camera_link_fixed pitch 0.858164 → 1.308997 rad
 4. 증속 벨트 1 : 1.5(모터 36T → 바퀴 24T): 바퀴 관절 y ±0.1396 → ±0.1546, 속도 한계 7.330383 → 10.995574 rad/s,
    effort 2.7 → 1.8 N·m(바퀴 쪽), 바퀴 모터 4개 x ±80 → ±34.54 · z 32.25(축), 베어링 블록·모터 받침·풀리·축·베어링·벨트·덮개 4모듈,
    앞 무게추(컵 + 강철 509 g)
 - base_link 표시 메시: 261027_1057/build_base/base_link.dae(옛 부품 삼각형을 빼고 새 부품을 더함)
 - base_link 질량·무게중심·관성: 261027_1057/build_base/base_link_mass.json
 - 충돌: 바퀴 모터 상자 4개 이동, 베어링 블록 상자 4개·벨트 덮개 상자 4개·무게추 상자 1개 추가. 브라켓·저장소 볼록 분해는 외곽이 같아 그대로
사용(PC): python3 make_revision_261027_1530.py
"""
import json
import math
import shutil
from pathlib import Path

TOP = Path("/home/pa4/git/lv2_next_module")
SRC = TOP / "261022_2119/urdf/waffle_4wd_description_261022_2119"
DST = TOP / "261027_1530/urdf/waffle_4wd_description_261027_1530"
BUILD = TOP / "261027_1057/build_base"
OLD_PKG, NEW_PKG = "waffle_4wd_description_261022_2119", "waffle_4wd_description_261027_1530"
TEXT_SUFFIX = {".xml", ".txt", ".md", ".yaml", ".xacro", ".urdf", ".py", ".json", ".rviz", ".csv"}
RATIO = 1.5
WHEEL_V = f"{70 * 2 * math.pi / 60 * RATIO:.6f}"      # 10.995574
WHEEL_E = f"{2.7 / RATIO:.1f}"                         # 1.8
CAM_PITCH = repr(math.radians(90 - 15))                # 1.308996938995747

if DST.exists():
    raise SystemExit(f"already exists: {DST}")
shutil.copytree(SRC, DST)
counts = {"package": 0}
for f in DST.rglob("*"):
    if f.is_file() and f.suffix in TEXT_SUFFIX and "meshes" not in f.parts and not f.name.startswith("REVISION_"):
        t = f.read_text(encoding="utf-8")
        n = t.count(OLD_PKG)
        if n:
            f.write_text(t.replace(OLD_PKG, NEW_PKG), encoding="utf-8")
            counts["package"] += n
shutil.copy2(BUILD / "base_link.dae", DST / "meshes/main/base_link.dae")

mass = json.loads((BUILD / "base_link_mass.json").read_text())
I = mass["inertia_com_kg_m2"]
c = mass["com_m"]
f17 = lambda v: repr(float(v))   # noqa: E731
OLD_INERTIAL = [
    ('<origin xyz="-0.053547954996508128 -2.2024531309009041e-05 0.14667466611266705" rpy="0 0 0" />',
     f'<origin xyz="{f17(c[0])} {f17(c[1])} {f17(c[2])}" rpy="0 0 0" />', 1),
    ('<mass value="3.7218030224487602" />', f'<mass value="{f17(mass["mass_kg"])}" />', 1),
    ('<inertia ixx="0.11556446013925142" ixy="-0.00019271830760513178" ixz="0.025205750302463154" iyy="0.11477421546923385" iyz="0.0001088149297917809" izz="0.053620011264812958" />',
     f'<inertia ixx="{f17(I[0][0])}" ixy="{f17(I[0][1])}" ixz="{f17(I[0][2])}" iyy="{f17(I[1][1])}" iyz="{f17(I[1][2])}" izz="{f17(I[2][2])}" />', 1),
]
MX = 34.539999 / 1000   # 모터 상자 가운데 x(= 축 x), parts_belt/motor_FL 외곽 가운데
MOTOR_BOX = [
    ('xyz="0.080000000000000002 0.11879999999999953 0.038249999999999999"', f'xyz="{MX!r} 0.11879999999999953 0.04425"', 1),
    ('xyz="0.080000000000000002 -0.11879999999999953 0.038249999999999999"', f'xyz="{MX!r} -0.11879999999999953 0.04425"', 1),
    ('xyz="-0.080000000000000002 0.11879999999999953 0.038249999999999999"', f'xyz="{-MX!r} 0.11879999999999953 0.04425"', 1),
    ('xyz="-0.080000000000000002 -0.11879999999999953 0.038249999999999999"', f'xyz="{-MX!r} -0.11879999999999953 0.04425"', 1),
]
WHEEL = [
    ('<origin xyz="0.080000 0.139600 0.026250"', '<origin xyz="0.080000 0.154600 0.026250"', 1),
    ('<origin xyz="0.080000 -0.139600 0.026250"', '<origin xyz="0.080000 -0.154600 0.026250"', 1),
    ('<origin xyz="-0.080000 0.139600 0.026250"', '<origin xyz="-0.080000 0.154600 0.026250"', 1),
    ('<origin xyz="-0.080000 -0.139600 0.026250"', '<origin xyz="-0.080000 -0.154600 0.026250"', 1),
    ('<limit effort="2.7" velocity="7.330383" />', f'<limit effort="{WHEEL_E}" velocity="{WHEEL_V}" />', 4),
]
CAM = [('rpy="3.1415921537716143 0.858163666456628 1.5707966218158638"', f'rpy="3.1415921537716143 {CAM_PITCH} 1.5707966218158638"', 1)]


def box(name, ctr_mm, size_mm):
    x, y, z = (v / 1000 for v in ctr_mm)
    a, b, cc = (v / 1000 for v in size_mm)
    return (f'<collision name="{name}"><origin xyz="{x!r} {y!r} {z!r}" rpy="0 -0 0" /><geometry><box size="{a!r} {b!r} {cc!r}" /></geometry></collision>')


NEW_BOXES = "".join(
    [box(f"belt_bearing_block_{n}", (sx * 80.0, sy * 117.35, 56.9), (25.0, 42.3, 83.8))
     for n, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1))]
    # 벨트 덮개(guard): 옛 충돌 외곽(y ±143.7 mm) 밖으로 152.1 mm까지 나온다. 외곽 상자(parts_belt/guard_*.stl)
    + [box(f"belt_guard_{n}", (sx * 50.5, sy * 142.65, 29.375), (88.0, 18.9, 36.75))
       for n, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1))]
    + [box("ballast_cup", (123.1, -40.3, 78.35), (19.4, 111.4, 47.3))])
for rel in ("urdf/waffle_4wd.urdf", "urdf/assemblies/main.urdf.xacro"):
    p = DST / rel
    t = p.read_text(encoding="utf-8")
    for old, new, n in OLD_INERTIAL + MOTOR_BOX + WHEEL + CAM:
        got = t.count(old)
        if got != n:
            raise SystemExit(f"{rel}: '{old[:60]}' {got}곳(예상 {n})")
        t = t.replace(old, new)
    anchor = t.index('<collision name="static_444">')
    end = t.index("</collision>", anchor) + len("</collision>")
    t = t[:end] + NEW_BOXES + t[end:]
    p.write_text(t, encoding="utf-8")

rd = DST / "robot_data.yaml"
d = json.loads(rd.read_text(encoding="utf-8"))
bl = d["links"]["base_link"]
dm = mass["mass_kg"] - bl["mass_kg"]
bl["mass_kg"] = mass["mass_kg"]
bl["com_link_m"] = " ".join(f17(v) for v in c)
bl["inertia_com_link_kg_m2"] = {"ixx": f17(I[0][0]), "ixy": f17(I[0][1]), "ixz": f17(I[0][2]),
                                "iyy": f17(I[1][1]), "iyz": f17(I[1][2]), "izz": f17(I[2][2])}
d["metadata"]["total_mass_kg"] += dm
for j, (sx, sy) in {"Wheel_Revolute_Front_Left": (1, 1), "Wheel_Revolute_Front_Right": (1, -1),
                    "Wheel_Revolute_Rear_Left": (-1, 1), "Wheel_Revolute_Rear_Right": (-1, -1)}.items():
    d["joints"][j]["origin"]["xyz"] = f"{sx * 0.08:.6f} {sy * 0.1546:.6f} 0.026250"
    d["joints"][j]["limits"] = {"effort": WHEEL_E, "velocity": WHEEL_V}
cj = d["joints"]["camera_link_fixed"]["origin"]
cj["rpy"] = cj["rpy"].replace("0.858163666456628", CAM_PITCH)
rd.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")

sm = DST / "simulation_metadata.json"
s = json.loads(sm.read_text(encoding="utf-8"))
s["wheel_peak_effort_N_mm"] = 1800
s["wheel_drive"] = {"type": "GT2 belt speed-up 1:1.5 (motor 36T -> wheel 24T)", "wheel_velocity_limit_rad_s": float(WHEEL_V),
                    "wheel_effort_N_m": float(WHEEL_E), "motor_axis_mm": [34.54, 118.8, 32.25]}
sm.write_text(json.dumps(s, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps({"dst": str(DST), "base_link_mass_kg": mass["mass_kg"], "delta_mass_kg": dm, "wheel": [WHEEL_E, WHEEL_V],
                  "camera_pitch": CAM_PITCH, "counts": counts}, ensure_ascii=False, indent=1))
