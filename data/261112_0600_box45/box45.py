"""상자 회전 ±45° 가정에서 face 접근 · 놓기 칸 계산 (계산, 11-12 미션 파트 261112 2, 검토 261111 1 요청 ①②③).

가정 (발제 값 아님): 상자 = 지도 칸 중심 ±0.05 m, 요 ±45° (사용자 11-12 정정, 정사각형이라 가장 가까운 면 기준 모든 경우).
지도 · 장애물 · 접근 자세표 = dev c689a38 mission_planner (poses.make_approaches, dice_mission.sdf 주행 높이 장애물) 그대로.
① 가장 가까운 면 (바깥 법선이 계획 접근 방향과 가장 가까운 면) 을 고르고, 그 면 앞으로 대기 자리를 다시 정함:
   stage = 그 면 앞 standoff (계획기와 같은 규칙: 외곽 + 0.05 빔 · 제자리 회전 0.005, standoff 0.45 → 0.30 중 가장 큰 것),
   보기 자리 = 면 0.20 (box_look), 집기 · 놓기 final = 면 gap 0.1272. 막히면 다음 면 (90° 옆).
   제자리 회전만 하는 face 접근 (대기 자리 그대로) 이 TFmini 빔을 면에 맞히는지도 셈 (11-12 오전 계산은 빔이 안 닿는 경우를 빼고 셌음).
② 45° 근처: 두 면이 같은 거리 → 두 면 모두 자리가 나는지.
③ 놓기 칸: 지도 축 3 x 3 (±60 mm, 잰 상자 중심 기준) 을 돈 상자 위에 둘 때 상자 가장자리 · 180 배치 영역까지 여유.
    python3 box45.py > box45.txt
"""
import itertools
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "workspace/ros2_ws/src/mission_planner"))
from mission_planner import poses as P  # noqa: E402
from mission_planner.arena import Solid, load_sdf  # noqa: E402

SDF = HERE / "workspace/urdf_and_usd/waffle_4wd_description/worlds/dice_mission.sdf"
solids = [s for s in load_sdf(str(SDF)) if s.z1 > 0.002 and s.z0 < 0.45]   # planner.py 와 같음
appr = P.make_approaches(solids)
HALF = P.BOX_HALF
LOOK_FACE = 0.20
D = 0.05
YAWS = list(range(-45, 46, 5))


def box_solid(c):
    best = min(solids, key=lambda s: math.hypot(s.x - c[0], s.y - c[1]))
    assert math.hypot(best.x - c[0], best.y - c[1]) < 1e-3 and abs(best.sx - 0.205) < 1e-3, best
    return best


def fast_clearance(sol, pose, limit=0.2):
    """P.clearance 와 같은 값 (5 mm 단위) 을 이분 탐색으로 (외곽 넓힘에 단조)."""
    if not P.footprint_clear(sol, pose, 0.0):
        return 0.0
    lo, hi = 0, int(round(limit / 0.005))
    if P.footprint_clear(sol, pose, hi * 0.005):
        return limit
    while hi - lo > 1:
        m = (lo + hi) // 2
        lo, hi = (m, hi) if P.footprint_clear(sol, pose, m * 0.005) else (lo, m)
    return lo * 0.005


def R2(t):
    return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])


def faces(c, th, face_dir):
    """상자 요 th 의 네 바깥 법선, 계획 접근 방향과 이루는 각 순 [(각 deg, 법선)]."""
    out = []
    for k in range(4):
        n = R2(th + k * math.pi / 2) @ np.asarray(face_dir, float)
        out.append((math.degrees(math.acos(max(-1.0, min(1.0, float(n @ face_dir))))), n))
    return sorted(out, key=lambda t: t[0])


import os
STAGE_MARGIN = float(os.environ.get("STAGE_MARGIN", "0.05"))   # 계획기 make_approaches margin
STD_MIN = float(os.environ.get("STD_MIN", "0.30"))             # 계획기 STANDOFFS 끝


def stage_for(sol, c, n, std_max):
    """계획기 best() 와 같은 규칙으로 그 면 앞 stage · 보기 · final. (standoff, 회전 가능, 보기 여유, final 여유) 또는 None."""
    so_list = [round(std_max - 0.01 * k, 2) for k in range(100) if std_max - 0.01 * k >= STD_MIN - 1e-9]
    fits = [(so, P.pose_facing(c, n, HALF, so)) for so in so_list]
    fits = [(so, st) for so, st in fits if P.footprint_clear(sol, st, margin=STAGE_MARGIN)]
    if not fits:
        return None
    rot = next(((so, st) for so, st in fits if P.rotate_clear(sol, st)), None)   # 계획기처럼 회전 되는 가장 큰 standoff, 없으면 외곽만
    so, st = rot or fits[0]
    others = [s for s in sol if s.name != "moved_box"]
    look = P.pose_facing(c, n, HALF, LOOK_FACE)
    fin = P.pose_facing(c, n, HALF, P.FINE_GAP)
    return dict(standoff=so, rotate=rot is not None, stage=st, look_clear=fast_clearance(others, look), final_clear=fast_clearance(others, fin),
                stage_clear=fast_clearance(sol, st))


targets = [(k, a) for k, a in appr.items() if a.kind in ("box", "place") and a.ok]
print("① face 접근: 가장 가까운 면 앞으로 대기 자리 다시 정하기 (dev c689a38 지도 · 접근 규칙)")
print(f"   경우: 상자 앞뒤 · 옆 {{−{D}, 0, +{D}}} m × 요 {YAWS[0]} ~ {YAWS[-1]}° (5° 간격), 대상 = 계획기 접근 자세표에서 ok 인 것")


def run_target(item):
    key, a = item
    import io, contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        one(key, a)
    return buf.getvalue()


def one(key, a):
    c0 = np.array(P.START_BOXES[a.extra["box"]] if a.kind == "box" else P.PLACES[key]["center"], float)
    fdir = -np.array([math.cos(a.stage.yaw), math.sin(a.stage.yaw)])   # 계획 면 바깥 법선
    bs = box_solid(c0)
    n_ok = n_first = n_second = n_fail = 0
    min_std, min_look, min_fin, worst = 9.0, 9.0, 9.0, None
    tie_both = []
    for dx, dy, yd in itertools.product((-D, 0, D), (-D, 0, D), YAWS):
        c = c0 + np.array([dx, dy])
        th = math.radians(yd)
        sol = [s for s in solids if s is not bs] + [Solid("moved_box", c[0], c[1], bs.z0, bs.z1, sx=bs.sx, sy=bs.sy, yaw=bs.yaw + th)]
        fl = faces(c, th, fdir)
        got = None
        for rank, (ang, n) in enumerate(fl[:2]):
            r = stage_for(sol, c, n, a.standoff)
            if r is not None and r["look_clear"] > 0 and r["final_clear"] > 0:
                got = (rank, ang, r)
                break
        if abs(abs(yd) - 45) < 1e-9:
            both = [stage_for(sol, c, n, a.standoff) for _, n in fl[:2]]
            tie_both.append(all(b is not None and b["look_clear"] > 0 and b["final_clear"] > 0 for b in both))
        if got is None:
            n_fail += 1
            worst = worst or (dx, dy, yd)
            continue
        rank, ang, r = got
        n_ok += 1
        n_first += rank == 0
        n_second += rank == 1
        min_std = min(min_std, r["standoff"])
        min_look = min(min_look, r["look_clear"])
        min_fin = min(min_fin, r["final_clear"])
    tot = n_ok + n_fail
    print(f"  {key:5s} 계획 standoff {a.standoff:.2f}: 자리 남 {n_ok}/{tot} (가장 가까운 면 {n_first} · 다음 면 {n_second}) · 못 섬 {n_fail}"
          + (f" (예 {worst})" if worst else "")
          + f" · 쓴 standoff 최소 {min_std:.2f} · 보기 자리 여유 최소 {min_look:.3f} · final 여유 최소 {min_fin:.3f} m"
          + f" · ±45° 에서 두 면 모두 자리 남 {sum(tie_both)}/{len(tie_both)}")


if __name__ == "__main__":
    from multiprocessing import Pool
    with Pool(len(targets)) as pool:
        for txt in pool.map(run_target, targets):
            print(txt, end="", flush=True)

print("\n① 대기 자리를 옮기지 않고 제자리에서 면에 수직으로만 돌 때 (11-12 오전 'face' 계산의 동작), 상자 중심이 로봇 x 축에서 벗어난 거리")
print("   로봇 = 계획 stage (Nav2 오차 0), 회전 = 로봇 중심 둘레. 옆 어긋남 > 반 변 0.1025 면 TFmini 빔이 그 면에 안 닿음")
for std in (0.40, 0.45):
    R = P.FRONT + std + HALF - P.CENTER_X   # 로봇 중심 → 계획 상자 중심
    for yd in (5, 10, 15, 20, 30, 45):
        lat = R * math.sin(math.radians(yd))
        print(f"  standoff {std:.2f} · 돈 각 {yd:2d}°: 옆 어긋남 {lat:.3f} m {'(빔 안 닿음)' if lat > HALF else ''}")

print("\n③ 놓기 칸: 지도 축 3 x 3 (±0.060, 잰 상자 중심 기준) 을 요 θ 상자 위에 둘 때, 상자 축 방향 가장 먼 칸 중심 · 퍽 가장자리")
print("   퍽 반 폭: 원기둥 0.015, 직육면체 30 x 30 은 상자 축에 맞추면 0.015 · 지도 축에 맞추면 0.015(|cos|+|sin|)")
print("   θ° | 칸 중심 | 원기둥 끝 → 상자 끝(0.1025) · 180 영역(0.090) 여유 | 지도 축 직육면체 끝 → 상자 끝 여유 | 놓기 허용 10 mm 넣은 원기둥 상자 끝 여유")
for yd in range(0, 46, 5):
    t = math.radians(yd)
    k = abs(math.cos(t)) + abs(math.sin(t))
    cmax = 0.060 * k
    cyl = cmax + 0.015
    cub = cmax + 0.015 * k
    print(f"   {yd:2d} | {cmax * 1000:5.1f} | {(0.1025 - cyl) * 1000:5.1f} · {(0.090 - cyl) * 1000:6.1f} mm | {(0.1025 - cub) * 1000:5.1f} mm | {(0.1025 - cyl - 0.010) * 1000:5.1f} mm")
t17 = math.degrees(math.asin(1.25 / math.sqrt(2))) - 45
print(f"   지도 축 칸 + 원기둥이 180 영역 안인 한계 θ = {t17:.1f}°  (0.060(|cos|+|sin|) + 0.015 ≤ 0.090)")
print("   상자 축 칸 (지금 mission measuredPlace · perception place_cells): 칸 중심 60 mm · 원기둥 끝 75 mm 로 θ 와 무관 (상자 끝 여유 27.5 mm).")
print("   다만 주사위 무늬가 지도 축에서 θ 만큼 돌고, 잰 요를 지도 축 ±45° 로 접으므로 θ 가 +45° ↔ −45° 를 넘나들면 무늬가 약 90° 뒤집힘.")
