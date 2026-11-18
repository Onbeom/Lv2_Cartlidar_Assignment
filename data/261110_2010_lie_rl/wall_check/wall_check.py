"""2026-11-11 (RL 261111 1, 사용자 승인 '둘 모두 해'): 누운 퍽 스킬 팔 작업 범위와 경기장 벽 · 고정물 거리 (계산, 모의 아님).

  python3 wall_check.py <mission_planner 패키지 폴더 (dev 7cb49b0)>

접근 자세 = 미션 계획기 make_approaches (World) 의 시작 박스 접근 (ok 인 것) 과, ±Y 면이면 퍽마다 로봇 중심 둘레로 돈 자세.
팔 작업 범위 (보수적으로 잡음) = J1 축 둘레 부채꼴:
  반경 R = 잡는 점 J1 수평 거리 최대 0.336 m (r2e 끝자세 x 최대 0.432 − J1 0.096) + 손가락 끝 · 집게 반폭 · 여유 0.064 = 0.40 m.
  각 = 관절1 ±HALF_DEG (인자, 기본 45°). r2e 기록 −16 ~ +3° (반대쪽 퍽이면 −3 ~ +16°), 상자 보기 자세 ±17.2°, target_look ±17.1° → 판정용 25°, 참고 45°.
고정물 = SDF 충돌 형상 중 높이가 팔 높이 범위 (바닥 위 0.07 ~ 0.45 m) 와 겹치는 것. 그 접근의 박스 자신은 뺌 (상자는 학습 환경에 있음).
"""
import math
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])
from mission_planner.planner import World  # noqa: E402
from mission_planner.poses import J1_X, START_BOXES, turned  # noqa: E402

R_ARM = 0.40
HALF_DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
w = World(sys.argv[1] + "/worlds/dice_mission.sdf")
sol = [s for s in w.solids if s.z1 > 0.07 and s.z0 < 0.45]


def seg_pts(poly, n=200):
    out = []
    for k in range(len(poly)):
        a, b = poly[k], poly[(k + 1) % len(poly)]
        t = np.linspace(0, 1, n)[:, None]
        out.append(a + (b - a) * t)
    return np.vstack(out)


rows = []
for a in w.approaches.values():
    if a.kind != "box" or not a.ok:
        continue
    b = a.extra["box"]
    bx, by = START_BOXES[b]
    poses = [("직진", a.final)]
    if not a.extra.get("inline"):
        for sx, v in a.extra["pucks"].items():
            poses.append((f"퍽 {'+' if sx > 0 else '−'}X 돈 {v['turn_deg']:+.1f}°", turned(a.final, v["turn_deg"])))
    for nm, p in poses:
        jx, jy = p.tf(J1_X, 0.0)
        best = (9.0, None)
        for s in sol:
            if math.hypot(s.x - bx, s.y - by) < 0.01 and s.r == 0 and abs(s.sx - 0.205) < 0.01:
                continue  # 이 박스 자신
            pts = seg_pts(s.footprint()) if s.r == 0 else np.c_[s.x + s.r * np.cos(np.linspace(0, 2 * np.pi, 200)), s.y + s.r * np.sin(np.linspace(0, 2 * np.pi, 200))]
            d = np.hypot(pts[:, 0] - jx, pts[:, 1] - jy)
            ang = np.degrees(np.arctan2(pts[:, 1] - jy, pts[:, 0] - jx)) - math.degrees(p.yaw)
            ang = (ang + 180) % 360 - 180
            m = np.abs(ang) <= HALF_DEG
            if m.any() and d[m].min() < best[0]:
                best = (float(d[m].min()), s.name)
        rows.append((a.key, nm, round(best[0], 3), round(best[0] - R_ARM, 3), best[1]))
print(f"팔 범위: J1 반경 {R_ARM} m, 관절1 ±{HALF_DEG}°, 고정물 {len(sol)} 개 (팔 높이 0.07 ~ 0.45 m)")
print("| 접근 | 자세 | J1 → 가장 가까운 고정물 (부채꼴 안) m | 여유 (− 반경) m | 고정물 |")
print("|---|---|---|---|---|")
for r in sorted(rows):
    print(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[4]} |")
print(f"최소 여유 {min(r[3] for r in rows):.3f} m")
