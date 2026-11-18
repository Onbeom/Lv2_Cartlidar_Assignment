"""실측 관절각 (도) -> 잡는 점 좌표 (2026-11-10, 실측 보정용). 팀 URDF FK (td_ik.fk), 잡는 점 = 손가락 패드 가운데.
    ~/.venvs/mujoco/bin/python meas_fk.py 0 -14 17 57 0            관절 1 ~ 5 (도) 하나
    ~/.venvs/mujoco/bin/python meas_fk.py --slot 3-1 172 -10 -22 92 0  칸 맞춤 (방법 A): 칸 설계 중심과 차이도
출력: 관절 1 축 기준 x (앞 +) · y (왼 +), 바닥 위 높이 [mm], 집게 피치 [°].
"""
import json
import math
import os
import sys

import numpy as np

from td_ik import fk

J1X, BASE_Z = 0.096, 0.00675
args = sys.argv[1:]
slot = None
if args and args[0] == "--slot":
    slot, args = args[1], args[2:]
q = np.radians([float(v) for v in args[:5]])
P, R = fk(q)
pitch = math.degrees(math.asin(np.clip(-R[2, 0], -1, 1)))
x, y, h = (P[0] - J1X) * 1000, P[1] * 1000, (P[2] + BASE_Z) * 1000
print(f"잡는 점: 관절1 축 기준 x {x:+.1f} y {y:+.1f} mm, 수평 거리 {math.hypot(x, y):.1f} mm, 바닥 위 {h:.1f} mm, 집게 피치 {pitch:.1f}° 아래")
if slot:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    s = [s for s in json.load(open(os.path.join(here, "pallet_place_traj.json")))["slots"] if f"{s['row']}-{s['index']}" == slot][0]
    sx, sy = (s["x"] - J1X) * 1000, s["y"] * 1000
    gz = (s["floor_z"] + 0.03 + 0.015 + BASE_Z) * 1000  # 칸 바닥 + 퍽 반 + 15 mm
    print(f"칸 {slot} 설계: x {sx:+.1f} y {sy:+.1f}, 잡는 높이 (바닥 위) {gz:.1f} mm -> 실제 - 설계: x {x - sx:+.1f} y {y - sy:+.1f} 높이 {h - gz:+.1f} mm")
