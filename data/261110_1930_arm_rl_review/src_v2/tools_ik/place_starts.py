"""놓기 시작 자세 표 (2026-11-09): 스토리지에서 꺼내 상자 (L1 · L3 = 205 상자, 60°) / L2 받침 (45°) 위로 가져온 직후를 흉내.
잡는 점 (base_link) 을 격자 위 넓게 + 높이 여러 개로 뽑아 피치 고정 IK (손가락 닫는 축 수평, 관절5 0) -> ../place_starts<이름>.json.
환경 (place_env.py) 이 리셋 때 하나를 골라 관절을 쓰고 퍽을 그 잡는 점 아래에 둠 (FK 가 Isaac 과 같음: 기존 시작 자세 0.296 / 0.356 확인).
    ~/.venvs/mujoco/bin/python place_starts.py
"""
import json
import math
import os

import numpy as np
from scipy.optimize import least_squares

from selfcol import self_dist
from td_ik import LIM, fk

BASE_Z_GROUND = 0.00675  # base_link 원점 지면 위
BOX_TOP_B = 0.070 - BASE_Z_GROUND  # 상자 · 받침 윗면 (base_link z)
GRASP_DZ, PUCK_HALF = 0.015, 0.030
FRONT = 0.140

KINDS = {
    # 이름: (피치, 격자 가운데 앞면에서 [m], 잡는 점 x 범위 (격자 가운데 기준), y 범위, 퍽 바닥 높이 (윗면 위) 범위, 시작 자세 씨앗)
    "60": (60.0, 0.1025, (-0.075, 0.075), (-0.10, 0.10), (0.03, 0.10), [0.0, -0.2454, 0.2999, 0.9927, 0.0]),
    "45l2": (45.0, 0.21, (-0.075, 0.075), (-0.10, 0.10), (0.03, 0.10), [0.0, -0.0204, 0.2807, 0.5251, 0.0]),
}


def solve(p, pitch, seed):
    def res(q):
        P, R = fk(q)
        return list((P - p) * 100) + [(-R[2, 0] - math.sin(math.radians(pitch))) * 3, R[2, 1] * 3, q[4] * 3]
    best = None
    for s2 in (0.0, -0.4, 0.4):
        q0 = np.array(seed, dtype=float)
        q0[0] = math.atan2(p[1], p[0] - 0.096)
        q0[1] += s2
        s = least_squares(res, q0, bounds=(LIM[:, 0], LIM[:, 1]))
        q = s.x
        P, R = fk(q)
        e = np.linalg.norm(P - p)
        pe = abs(math.degrees(math.asin(np.clip(-R[2, 0], -1, 1))) - pitch)
        m = min(min(q[i] - LIM[i, 0], LIM[i, 1] - q[i]) for i in range(4))
        if e < 0.001 and pe < 1.0 and m > 0.08 and abs(q[2]) < 1.3 and self_dist(q) > 0.06:
            if best is None or np.linalg.norm(q - seed) < np.linalg.norm(best - seed):
                best = q
    return best


def main():
    rng = np.random.default_rng(0)
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for name, (pitch, gc, xr, yr, hr, seed) in KINDS.items():
        out, tried = [], 0
        cx = FRONT + 0.055 + gc
        while len(out) < 1500 and tried < 6000:
            tried += 1
            h = rng.uniform(*hr)
            p = np.array([cx + rng.uniform(*xr), rng.uniform(*yr), BOX_TOP_B + h + PUCK_HALF + GRASP_DZ])
            q = solve(p, pitch, seed)
            if q is not None:
                out.append({"q": [round(float(v), 5) for v in q], "g": [round(float(v), 5) for v in p]})
        fn = os.path.join(here, f"place_starts{name}.json")
        json.dump({"note": __doc__.strip().splitlines()[0], "pitch_deg": pitch, "starts": out}, open(fn, "w"))
        print(f"{name}: {len(out)}/{tried} -> {fn}")


if __name__ == "__main__":
    main()
