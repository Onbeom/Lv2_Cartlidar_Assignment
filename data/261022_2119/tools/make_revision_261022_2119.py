"""261022_1800 URDF 패키지를 복사해 관절 속도 한계를 ROBOTIS 공식 무부하 속도(11.1 V)로 바꾼 261022_2119 리비전을 만든다.

사용(PC): python3 make_revision_261022_2119.py --src <261022_1800 패키지 폴더> --dst <새 패키지 폴더>
- 바퀴(XM430-W210) velocity 2.0 → 7.330383 rad/s (70 rev/min), 팔(XM430-W350) 0.5 → 4.502949 rad/s (43 rev/min)
- 집게(0.01 m/s)·effort·한계 각도·형상·질량은 그대로
- 패키지 이름 waffle_4wd_description_261022_1800 → waffle_4wd_description_261022_2119
원본 폴더는 읽기만 한다.
"""
import argparse
import json
import math
import shutil
from pathlib import Path

OLD_PKG, NEW_PKG = "waffle_4wd_description_261022_1800", "waffle_4wd_description_261022_2119"
WHEEL = f"{70 * 2 * math.pi / 60:.6f}"   # 7.330383
ARM = f"{43 * 2 * math.pi / 60:.6f}"     # 4.502949
TEXT_SUFFIX = {".xml", ".txt", ".md", ".yaml", ".xacro", ".urdf", ".py", ".json", ".rviz", ".csv"}

ap = argparse.ArgumentParser()
ap.add_argument("--src", required=True, type=Path)
ap.add_argument("--dst", required=True, type=Path)
a = ap.parse_args()
if a.dst.exists():
    raise SystemExit(f"already exists: {a.dst}")
shutil.copytree(a.src, a.dst)
counts = {"package": 0}
for f in a.dst.rglob("*"):
    if f.is_file() and f.suffix in TEXT_SUFFIX and "meshes" not in f.parts and not f.name.startswith("REVISION_"):
        t = f.read_text(encoding="utf-8")
        n = t.count(OLD_PKG)
        if n:
            f.write_text(t.replace(OLD_PKG, NEW_PKG), encoding="utf-8")
            counts["package"] += n
edits = {
    "urdf/waffle_4wd.urdf": [('effort="2.7" velocity="2.0"', f'effort="2.7" velocity="{WHEEL}"', 4),
                             ('effort="3.8" velocity="0.5"', f'effort="3.8" velocity="{ARM}"', 5)],
    "urdf/assemblies/main.urdf.xacro": [('effort="2.7" velocity="2.0"', f'effort="2.7" velocity="{WHEEL}"', 4),
                                        ('effort="3.8" velocity="0.5"', f'effort="3.8" velocity="{ARM}"', 5)],
    "robot_data.yaml": [('"velocity": "2.0"', f'"velocity": "{WHEEL}"', 4), ('"velocity": "0.5"', f'"velocity": "{ARM}"', 5)],
}
for rel, reps in edits.items():
    p = a.dst / rel
    t = p.read_text(encoding="utf-8")
    for old, new, expect in reps:
        got = t.count(old)
        if got != expect:
            raise SystemExit(f"{rel}: '{old}' {got}곳(예상 {expect})")
        t = t.replace(old, new)
        counts[f"{rel}: {old}"] = got
    p.write_text(t, encoding="utf-8")
print(json.dumps({"dst": str(a.dst), "wheel_velocity": WHEEL, "arm_velocity": ARM, "counts": counts},
                 ensure_ascii=False, indent=1))
