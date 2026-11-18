"""'비스듬히 서서 보기' 설계 계산 (계산, 11-12 사용자 결정 '우선 비스듬히 서서 보기', 검토 261111 1 요청).

설계 (안 O):
  1. 계획 대기 자리 (dev 접근표 stage, Nav2) 에서 앞 D435 로 상자 중심 · 요를 잰다 (④ 렌더 판정 통과 범위).
  2. 그 자리에서 제자리 (ICR 둘레) 로 돌아 로봇 x 축이 잰 상자 중심을 지나게 한다 (작은 각, 상자 밀림 ±5 cm 만큼).
  3. 그 축을 따라 곧게 다가가 상자 중심이 base_footprint 앞 R_look = 앞 끝 0.1328 + 0.20 + 반 변 = 0.4353 에 오게 선다 (수직 접근과 같은 거리).
     상자는 로봇 기준 β (= 상자 요 − 로봇 방향, ±45° 접음) 만큼 비스듬하다.
  4. box_look 은 잰 중심을 겨눔 (보는 점 오차만큼 j1 ≠ 0). 0.40 m · 50° (1 순위) · 55° (참고).
  5. 놓기 · 집기는 같은 축으로 더 다가가 '가장 가까운 상자 점까지 앞 끝 0.1272' (지금 final gap 과 같은 틈) 에서.
거리 재기: 축이 중심을 지나면 TFmini 빔은 가까운 면의 가운데에서 옆 반 변 · tan|β| 에 닿는다. 모서리까지 남는 거리 =
  반 변 (1 − tan|β|) − 옆 오차 − 빔 반지름 (fov 3.6° → r · tan 1.8°) 이 0.02 m 이상이면 TFmini, 아니면 D435 잰 중심 + odom 이동 거리.
판정 (U3 · face 와 같은 값): place 네 모서리 + 15 mm > 30 px, 9 칸 > 30 px, 태그 1280 ≥ 16 px, 관절 > 0.1 (처짐 포함),
  팔이 상자 영역 안 (A4) 없음, 차체 CoACD · 실은 퍽 ≥ 15 mm. 벽: 보기 · final 자리 외곽 + 0.01 비고, stage → final 직선 길 외곽이 안 닿음.
놓기 닿음: final 에서 J1 (base x 0.096) → 9 칸 (상자 축) 거리가 45° 놓기 범위 0.12 ~ 0.35 m (poses.REACH_TILT_MIN · REACH_TILT).
    ~/isaacsim-venv/bin/python -u oblique.py > oblique.txt     (scipy: 차체 거리)
"""
import itertools
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
U3 = HERE.parent / "261112_0130_u3_render"
ANGS = [float(x) for x in os.environ.get("ANGS", "50 55").split()]

# ---- 지도 · 계획기 (box45 와 같은 것)
_argv = sys.argv
sys.argv = ["box45.py"]
exec(open(HERE / "box45.py").read().split("targets = [")[0])
PL = dict(P=P, solids=solids, appr=appr, box_solid=box_solid, Solid=Solid, faces=faces)

# ---- 팔 · 카메라 판정 (approach_resid 의 fk · look · judge)
ns = {"__name__": "x", "__file__": str(U3 / "approach_resid.py")}
src = open(U3 / "approach_resid.py").read().split("\nrng = np.random.default_rng(0)\nprint(")[0]
sys.argv = ["approach_resid.py", "--dist", "0.40", "--ang", "50"]
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(src, ns)
sys.stdout = _o
sys.path.insert(0, str(U3))
os.chdir(U3)
import cand_eval as CE  # noqa: E402  (차체 CoACD · 실은 퍽)
sys.argv = _argv

FRONT, HALF = 0.1328, 0.1025
R_LOOK = FRONT + 0.20 + HALF
GAP_F = 0.1272
J1_X = 0.096
ICR = np.array([-0.0658, 0.0])
TF_X = 0.13
BEAM = math.tan(math.radians(1.8))


def R2(t):
    return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])


def fold(a):
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


# =========================================================== 1. 설 자리 (지도)
def aim_axis(stage, c):
    """stage (P.Pose) 에서 ICR 둘레로 돌아 x 축이 c (지도) 를 지나게 한 자세."""
    x, y, yaw = stage.x, stage.y, stage.yaw
    for _ in range(6):
        cr = R2(-yaw) @ (np.asarray(c) - [x, y])
        dpsi = math.atan2(cr[1], cr[0] - ICR[0])
        icr_w = np.array([x, y]) + R2(yaw) @ ICR
        xy = icr_w + R2(dpsi) @ (np.array([x, y]) - icr_w)
        x, y, yaw = xy[0], xy[1], yaw + dpsi
    return P.Pose(x, y, yaw)


def along(pose, c, r):
    """pose 방향 그대로 c 가 앞 r 에 오게 옮긴 자세."""
    u = np.array([math.cos(pose.yaw), math.sin(pose.yaw)])
    xy = np.asarray(c) - u * r
    return P.Pose(xy[0], xy[1], pose.yaw)


def stand(key, a, d, th_deg):
    c0 = np.array(P.START_BOXES[a.extra["box"]] if a.kind == "box" else P.PLACES[key]["center"], float)
    bs = box_solid(c0)
    c = c0 + np.asarray(d)
    th = math.radians(th_deg)
    walls = [s for s in solids if s is not bs]
    st = a.stage
    p_aim = aim_axis(st, c)
    beta = fold(th - p_aim.yaw + bs.yaw)
    k = abs(math.cos(beta)) + abs(math.sin(beta))
    r_fin = FRONT + GAP_F + HALF * k
    look = along(p_aim, c, R_LOOK)
    fin = along(p_aim, c, r_fin)
    turn = math.degrees(P.wrap(p_aim.yaw - st.yaw))
    icr_w = np.array([st.x, st.y]) + R2(st.yaw) @ ICR

    def turned(t):
        xy = icr_w + R2(t) @ (np.array([st.x, st.y]) - icr_w)
        return P.Pose(xy[0], xy[1], st.yaw + t)
    ok_turn = all(P.footprint_clear(walls, turned(math.radians(turn) * f), 0.005) for f in np.linspace(0, 1, 6))
    path = [P.Pose(p_aim.x + (fin.x - p_aim.x) * f, p_aim.y + (fin.y - p_aim.y) * f, p_aim.yaw) for f in np.linspace(0, 1, 12)]
    ok_path = all(P.footprint_clear(walls, p, 0.0) for p in path)
    cl_look = fast_clearance(walls, look)
    cl_fin = fast_clearance(walls, fin)
    # TFmini: 렌즈 → 면 거리 (보기 자리), 빔이 닿는 자리의 모서리까지
    r_tf = R_LOOK - TF_X - HALF / max(math.cos(beta), 1e-6)
    edge = HALF * (1 - abs(math.tan(beta))) - 0.015 - r_tf * BEAM
    return dict(turn=turn, beta=math.degrees(beta), ok_turn=ok_turn, ok_path=ok_path, cl_look=cl_look, cl_fin=cl_fin,
                r_fin=r_fin, tf_edge=edge, ok=ok_turn and ok_path and cl_look > 0.01 - 1e-9 and cl_fin > 0.01 - 1e-9)


def run_target(item):
    key, a = item
    rows = []
    for dx, dy, th in itertools.product((-0.05, 0, 0.05), (-0.05, 0, 0.05), range(-45, 46, 5)):
        rows.append((dx, dy, th, stand(key, a, (dx, dy), th)))
    return key, a.standoff, rows


targets = [(k, a) for k, a in appr.items() if a.kind in ("box", "place") and a.ok]
print("== 1. 설 자리 (dev c689a38 지도 · 계획 stage 에서 잰 중심 쪽으로 돌고 곧게 다가감, 상자 ±0.05 m × −45 ~ 45°)")
print("   ok = stage 제자리 회전 외곽 +0.005 · 다가가는 길 외곽 · 보기 자리 · final 자리 외곽 +0.01 모두 벽 · 기둥 · 다른 상자와 안 겹침")
from multiprocessing import Pool  # noqa: E402
with Pool(len(targets)) as pool:
    res = pool.map(run_target, targets)
betas = []
for key, so, rows in res:
    n_ok = sum(r["ok"] for *_, r in rows)
    bad = [(dx, dy, th, r) for dx, dy, th, r in rows if not r["ok"]]
    why = {}
    for *_, r in bad:
        w = "회전" if not r["ok_turn"] else "길" if not r["ok_path"] else "보기 자리" if r["cl_look"] < 0.01 else "final"
        why[w] = why.get(w, 0) + 1
    tm = max(abs(r["turn"]) for *_, r in rows)
    print(f"  {key:5s}: 자리 남 {n_ok}/{len(rows)} · 못 섬 까닭 {why or '-'} · 돌 각 최대 {tm:.1f}° · β 범위 "
          f"{min(r['beta'] for *_, r in rows):+.1f} ~ {max(r['beta'] for *_, r in rows):+.1f}° · 보기 자리 벽 여유 최소 "
          f"{min(r['cl_look'] for *_, r in rows) * 1000:.0f} mm · final 최소 {min(r['cl_fin'] for *_, r in rows) * 1000:.0f} mm")
    betas += [r["beta"] for *_, r in rows]

print("\n== 2. 거리 재기: TFmini 빔 자리에서 모서리까지 남는 거리 (옆 오차 0.015 · 빔 반지름 넣음), ≥ 0.02 m 이면 TFmini")
for b in (0, 10, 15, 20, 25, 30, 35, 40, 45):
    beta = math.radians(b)
    r_tf = R_LOOK - TF_X - HALF / math.cos(beta)
    edge = HALF * (1 - math.tan(beta)) - 0.015 - r_tf * BEAM
    print(f"  |β| {b:2d}°: 면까지 {r_tf:.3f} m · 빔 반지름 {r_tf * BEAM * 1000:.1f} mm · 모서리까지 남음 {edge * 1000:6.1f} mm → "
          f"{'TFmini' if edge >= 0.02 else 'D435 + odom'}")

print("\n== 3. 보기 자리에서 box_look (보는 점 = 잰 중심 + 오차, 대각 네 방향), β = −45 ~ 45° (5° 간격)")
CELL = 0.06


def view(ang, e_mm):
    ns["a"].ang = ang
    e = e_mm / 1000 / math.sqrt(2)
    worst = dict(place=1e9, cells=1e9, tag=1e9, jm=1e9)
    wb = {}
    inbox = 0
    qs = {}
    for sx, sy in itertools.product((-1, 1), repeat=2):
        bc = np.array([R_LOOK, 0.0])
        aim = bc + np.array([sx * e, sy * e])
        q = ns["look"](aim)
        if q is None:
            return None
        qs[(sx, sy)] = q
        for b in range(-45, 46, 5):
            r = ns["judge"](q, bc, math.radians(b))
            for kk in worst:
                if r[kk] < worst[kk]:
                    worst[kk], wb[kk] = r[kk], (b, sx, sy)
            inbox += r["inbox"]
    body = [CE.body(q, True) for q in qs.values()]
    return worst, wb, inbox, min(x[0] for x in body), min(x[1] for x in body), qs


for ang in ANGS:
    for e_mm in (12.0, 18.0):
        v = view(ang, e_mm)
        if v is None:
            print(f"  0.40 m · {ang:.0f}° · 보는 점 오차 {e_mm:.0f} mm: IK 못 풂")
            continue
        worst, wb, inbox, bd, pk, qs = v
        j1 = [q[0] for q in qs.values()]
        print(f"  0.40 m · {ang:.0f}° · 보는 점 오차 {e_mm:.0f} mm: place {worst['place']:.1f} px (β {wb['place'][0]:+d}) · 9 칸 {worst['cells']:.1f} px "
              f"(β {wb['cells'][0]:+d}) · 태그 1280 {worst['tag']:.1f} px · 관절 {worst['jm']:.2f} rad · 상자 안 {inbox} · 차체 {bd:.1f} mm · "
              f"실은 퍽 {pk:.1f} mm · j1 {min(j1):+.3f} ~ {max(j1):+.3f}")

print("\n== 4. 놓기 · 집기 자리 (가장 가까운 상자 점까지 앞 끝 0.1272) 에서 J1 → 칸 · 퍽 거리, 45° 놓기 범위 0.12 ~ 0.35 m")
for b in range(0, 46, 5):
    beta = math.radians(b)
    r_fin = FRONT + GAP_F + HALF * (abs(math.cos(beta)) + abs(math.sin(beta)))
    ds = []
    for e in itertools.product((-0.0127, 0, 0.0127), repeat=2):
        c = np.array([r_fin, 0.0]) + np.array(e)
        for i, j in itertools.product((-1, 0, 1), repeat=2):
            p = c + R2(beta) @ np.array([i * CELL, j * CELL])
            ds.append(math.hypot(p[0] - J1_X, p[1]))
    pk = []
    for s in (-1, 1):   # 시작 박스 퍽 = 상자 축 x ±0.040
        p = np.array([r_fin, 0.0]) + R2(beta) @ np.array([s * 0.040, 0.0])
        v = p - [J1_X, 0.0]
        mis = abs(math.degrees(fold(math.atan2(v[1], v[0]) - beta)))
        pk.append((math.hypot(*v), mis))
    print(f"  |β| {b:2d}°: final 상자 중심 앞 {r_fin:.3f} m · 칸 J1 거리 {min(ds):.3f} ~ {max(ds):.3f} m "
          f"{'(범위 안)' if 0.12 <= min(ds) and max(ds) <= 0.35 else '(범위 밖)'} · 시작 박스 퍽 J1 거리 · J1→퍽 방향과 퍽 면 어긋남 "
          + ", ".join(f"{d:.3f} m {m:.1f}°" for d, m in pk))
