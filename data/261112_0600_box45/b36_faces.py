"""B3 · B6 시작 박스: 네 면 (N · S · E · W) 마다 수직으로 설 때 무엇이 막는지 (계산, 11-12 사용자 질문 · 검토 261111 1 요청).

상자 = 지도 자리 그대로 (회전 0, 밀림 0) 와, 상자 회전 ±30° · ±45° (가장 가까운 면 = 그 면을 그만큼 돌린 것).
계획기 규칙 (dev c689a38 poses.make_approaches): stage = 외곽 + 0.05 가 비고 제자리 회전 여유 0.005, standoff 0.45 → 0.30 중 가장 큰 것.
보기 자리 = 면 0.20 (box_look), final = 면 0.127 (집기 · 놓기). 막는 물체 = 그 자리 외곽 (+ 여유) 과 겹치는 SDF 충돌 형상, 거리 = 외곽에서 가장 가까운 물체까지.
    python3 -u b36_faces.py > b36_faces.txt
"""
import math
import sys

import numpy as np

sys.argv = ["box45.py"]
exec(open(__file__.replace("b36_faces.py", "box45.py")).read().split("targets = [")[0])

FACES = {"N": (0.0, 1.0), "S": (0.0, -1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}


def poly_dist(a, b):
    """두 볼록 다각형 사이 거리 (겹치면 0), 꼭짓점-변 최소."""
    if P._sat_overlap(a, b):
        return 0.0

    def seg(p, q, r):
        d = q - p
        t = max(0.0, min(1.0, float((r - p) @ d / (d @ d))))
        return float(np.linalg.norm(p + t * d - r))
    return min(min(seg(a[i], a[(i + 1) % len(a)], r) for i in range(len(a)) for r in b),
               min(seg(b[i], b[(i + 1) % len(b)], r) for i in range(len(b)) for r in a))


def nearest(sol, pose, k=2, skip=("moved_box",)):
    fp = P.footprint_poly(pose, 0.0)
    ds = sorted(((poly_dist(fp, s.footprint()), s.name) for s in sol if s.name not in skip), key=lambda t: t[0])
    out = []
    for d, n in ds:
        short = n.replace("arena/", "").replace("_collision", "")
        key = "_".join(short.split("_")[:2])
        if key not in ["_".join(o[1].split("_")[:2]) for o in out]:
            out.append((d, short))
        if len(out) == k:
            break
    return out


def report(b, side, th_deg):
    c = np.array(P.START_BOXES[b], float)
    bs = box_solid(c)
    th = math.radians(th_deg)
    sol = [s for s in solids if s is not bs] + [Solid("moved_box", c[0], c[1], bs.z0, bs.z1, sx=bs.sx, sy=bs.sy, yaw=bs.yaw + th)]
    n = R2(th) @ np.array(FACES[side])
    best = None
    for so in [round(0.45 - 0.01 * k, 2) for k in range(16)]:
        st = P.pose_facing(c, n, HALF, so)
        if P.footprint_clear(sol, st, margin=0.05):
            best = (so, st, P.rotate_clear(sol, st))
            break
    st45 = P.pose_facing(c, n, HALF, 0.45)
    st30 = P.pose_facing(c, n, HALF, 0.30)
    look = P.pose_facing(c, n, HALF, LOOK_FACE)
    fin = P.pose_facing(c, n, HALF, P.FINE_GAP)
    f = lambda L: ", ".join(f"{nm} {d * 1000:.0f} mm" for d, nm in L)
    stage_txt = (f"stage standoff {best[0]:.2f} 가능(회전 {'됨' if best[2] else '안 됨'})" if best else
                 f"stage 없음: standoff 0.45 자리 외곽에서 {f(nearest(sol, st45))} · 0.30 자리 {f(nearest(sol, st30))} (필요 50 mm)")
    print(f"  B{b} {side} 면 (상자 {th_deg:+d}°, 면 법선 {np.round(n, 2)}): {stage_txt} | "
          f"보기 자리(면 0.20) {f(nearest(sol, look, 1))} | final(면 0.127) {f(nearest(sol, fin, 1))}")


print("B3 · B6 네 면에 수직으로 설 때 (dev c689a38 지도 · 계획기 규칙). 거리 = 로봇 외곽에서 물체 바닥 투영까지, 0 = 겹침")
for b in (3, 6):
    print(f"\n시작 박스 B{b} 중심 {P.START_BOXES[b]}, 계획기 접근표: " + ", ".join(f"{k} ok={a.ok} {a.note}" for k, a in appr.items() if k.startswith(f"B{b}:")))
    for th in (0, 30, -30, 45, -45):
        for side in ("N", "S", "E", "W"):
            report(b, side, th)
