"""ALIGN 후보에서 FINE 정렬 가로 오차 흐름 (계산, 2026-11-11 미션 파트 261111 1).

FINE 은 팔 카메라 화면 가로 오프셋 arm_offset.x (반폭 정규화, 허용 0.04) 로만 로봇을 돌림 (drive_node fine_.step).
j1 · j5 = 0 이면 좌우 대칭이라 퍽 높이 · 거리가 바뀌어도 가로 위치가 그대로이지만, 팔을 옆으로 돌리거나 손목을 굴리면 섞임.
로봇이 바르게 선 상태 (yaw 0, 조준 퍽 가로 0) 에서:
  D: 면 0.37 → 0.127 동안 조준 퍽 (서 있음) 의 arm_offset.x 변화 (기준점 = 면 0.127 서 있는 퍽, align_point 를 여기서 잰다고 봄)
  Hgt: 면 0.127 에서 누운 퍽 (가운데 높이 −0.015) 과 서 있는 퍽의 차이
  Rng: 면 0.117 · 0.137 (TFmini 허용 ±0.01) 과 0.127 의 차이
  → 각 차이를 그 거리의 d(arm_offset.x)/d(yaw) 로 나눠 yaw 오차 (°) 와 퍽 자리 가로 오차 (mm, 로봇 중심 → 퍽 거리 곱) 로.
  python fine_drift.py <dev urdf> j1 j2 j3 j4 j5 [...5 개씩 더]
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
dev, vals = sys.argv[1], [float(x) for x in sys.argv[2:]]
sys.argv = ["align_search.py", dev, "--pose", "-1.62", "0.21", "1.88"]
ns = {"__name__": "align_search"}
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(HERE, "align_search.py")).read(), ns)
sys.stdout = _o
fk, FX, FY, CX, CY, W = ns["fk"], ns["FX"], ns["FY"], ns["CX"], ns["CY"], ns["W"]
FRONT, C_X, HALF, ZTOP = ns["FRONT"], ns["C_X"], ns["HALF"], ns["ZTOP"]


def ox(Ti, p, yaw):
    """로봇이 yaw 만큼 돌았을 때 (= 장면이 −yaw 돈 것) 점 p 의 arm_offset.x (반폭 정규화, 화면 중심 기준)"""
    c, s = math.cos(-yaw), math.sin(-yaw)
    # 회전 중심 = 로봇 중심 (C_X, 0)
    x, y = p[0] - C_X, p[1]
    q = np.array([C_X + c * x - s * y, s * x + c * y, p[2], 1.0])
    k = Ti @ q
    u = FX * k[0] / k[2] + CX
    return (u - W / 2) / (W / 2)


for i in range(0, len(vals), 5):
    q5 = vals[i:i + 5]
    q = {f"joint{k + 1}": q5[k] for k in range(5)}
    Ti = np.linalg.inv(fk("arm_camera_optical_frame", q))
    zs, zl = ZTOP + 0.03, ZTOP + 0.015

    def P(d, z):
        return np.array([FRONT + d + HALF, 0.0, z])

    ref = ox(Ti, P(0.127, zs), 0.0)
    gain = (ox(Ti, P(0.127, zs), 0.01) - ox(Ti, P(0.127, zs), -0.01)) / 0.02  # per rad
    reach = FRONT + 0.127 + HALF - C_X
    D = max(abs(ox(Ti, P(d, zs), 0.0) - ref) for d in np.linspace(0.127, 0.37, 25))
    Hg = abs(ox(Ti, P(0.127, zl), 0.0) - ref)
    Rg = max(abs(ox(Ti, P(d, zs), 0.0) - ref) for d in (0.117, 0.137))
    out = []
    for name, v in (("D 접근 중", D), ("Hgt 누운 퍽", Hg), ("Rng ±0.01", Rg)):
        yaw = v / abs(gain)
        out.append(f"{name} {v:.4f} (yaw {math.degrees(yaw):.2f}°, 퍽 자리 {yaw * reach * 1000:.1f} mm)")
    print(f"q {q5}  기준 arm_offset.x {ref:+.3f} · 이득 {gain:.2f}/rad | " + " | ".join(out))
