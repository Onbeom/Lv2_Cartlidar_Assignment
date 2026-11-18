"""누운 직육면체 B안 (위에서 쥐고 손목으로 세워 바로 넣기) IK 범위 찾기 (2026-11-04).
세운 뒤 집게는 수평 (퍽 축 ⟂ 집게 방향). 바꾸는 것: 쥐는 높이 (퍽 중심 + off), 놓는 높이 (퍽 바닥 - 벽 윗면), 집게 방향 (칸 반지름 바깥 / 안쪽), 허용 기울기."""
import json, math, sys, numpy as np
from td_ik import fk, LIM, least_squares
T = json.load(open("../pallet_place_traj.json"))
J1 = np.array([0.096, 0.0])

def solve(p, d, th, tilt_max):
    lo, hi = LIM[:, 0].copy(), LIM[:, 1].copy()
    lo[0] = max(-3.0, th - 0.6); hi[0] = min(3.0, th + 0.6)
    def res(q):
        P, R = fk(q)
        a = math.degrees(math.acos(np.clip(R[:, 0] @ d, -1, 1)))
        return list((P - p) * 100) + [max(0.0, a - tilt_max) * 0.05] + [R[2, 1] * 3] + [ (R[:, 0] @ np.cross(d, [0, 0, 1])) * 3 ]
    best = None
    for a in (-1.2, -0.6, 0.0, 0.6, 1.2):
        for b in (-1.2, -0.4, 0.4, 1.2):
            for c in (-1.5, -0.5, 0.5, 1.5):
                s = least_squares(res, np.array([np.clip(th, lo[0] + 1e-3, hi[0] - 1e-3), a, b, c, 0.0]), bounds=(lo, hi))
                if best is None or s.cost < best.cost: best = s
    q = best.x; P, R = fk(q)
    e = np.linalg.norm(P - p) * 1000; ang = math.degrees(math.acos(np.clip(R[:, 0] @ d, -1, 1)))
    m = min(min(q[i] - LIM[i, 0], LIM[i, 1] - q[i]) for i in range(1, 4))
    return e < 1.5 and ang <= tilt_max + 0.5 and m > 0.03, q, m, ang

rows = []
for tilt in (0.0, 3.0, 6.0):
    for sgn, dn in ((1, "바깥"), (-1, "안쪽")):
        for off in (0.0, 0.010, 0.015, 0.020, 0.025):
            for gap in (-0.010, -0.005, 0.0, 0.005):
                ok = []
                for s in T["slots"]:
                    h = np.array([s["x"], s["y"]]) - J1; h /= np.linalg.norm(h); th = math.atan2(h[1], h[0])
                    d = np.array([sgn * h[0], sgn * h[1], 0.0])
                    p = np.array([s["x"], s["y"], s["wall_top_z"] + gap + 0.030 + off])
                    if solve(p, d, th, tilt)[0]: ok.append(f"{s['row']}:{s['index']}")
                fb = gap + off + 0.001  # 손가락 밑 - 벽 윗면 (손가락은 집게 축 아래 29 mm)
                print(f"기울기 허용 {tilt:.0f}° 집게 {dn} 쥐는 높이 +{off*1000:2.0f} mm 놓는 높이 {gap*1000:+3.0f} mm (손가락 밑 벽 위 {fb*1000:+3.0f} mm): {len(ok):2d}/12 {' '.join(ok)}", flush=True)
