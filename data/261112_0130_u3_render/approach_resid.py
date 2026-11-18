"""상자 범위 가정 × 접근 방식별로, 접근 뒤 로봇 기준 상자 어긋남과 보기 자세(box_look) 판정 (계산, 11-12 미션 파트 261112 1).

가정 (발제 · 사용자 값 아님 — 사용자 '지도 자리 언저리', 검토 261111 1 예시):
  상자: 지도 칸 중심에서 앞뒤 · 옆 ±BOX_D, 요 ±BOX_YAW (사람이 세팅할 때 틀어짐)
  Nav2 도착: xy_goal_tolerance 0.05 · yaw_goal_tolerance 0.1 (nav2_params.yaml), 위치 추정 오차는 0 으로 둠 (모름)
접근 방식 (dev c689a38 코드 읽기):
  box   : approach_box — 도착 방향(odom yaw) 유지, 1층 TFmini(차체 앞 끝 y 0, 앞으로) 가 상자 면까지 FACE (±0.01)
  puck  : approach — COARSE · FINE 으로 추적 퍽 하나를 겨눔 (팔 카메라 가로 가운데 = 로봇 x 축이 퍽을 지남), TFmini 로 면 FACE
  box_meas : approach_box 그대로, 보는 점만 미리 잰 상자 중심 (±0.01 가정) — 회전은 그대로 남음
  face  : (제안) approach_box 앞에 상자 앞면 각도를 재서 (라이다 선 맞춤 등) 면에 수직으로 돌린 뒤 TFmini — 남는 각도 ±FACE_YAW
보기 (box_look, 노드와 같은 look_tool): 보는 점 = (a) 지도 상자 중심 (TF, survey · place_box), (b) 잰 중심 ±0.01 (verify)
판정 (U3 와 같은 값): place = 윗면 네 모서리 + 15 mm, survey · verify = 9 칸 퍽 윗면 (서 있는 · 누운), 화면 가장자리 30 px 넘게,
  태그(1280) ≥ 16 px, 관절 여유 > 0.1 (명령 · 처짐), 팔이 실제 상자 영역 (윗면 + 0.04 아래) 밖.
    python3 approach_resid.py [--box_d 0.05] [--box_yaw 15] [--dist 0.40] [--ang 55] [--face 0.20]
"""
import argparse
import itertools
import math
import os
import subprocess
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--box_d", type=float, default=0.05)
ap.add_argument("--box_yaw", type=float, default=15.0)
ap.add_argument("--nav_xy", type=float, default=0.05)
ap.add_argument("--nav_yaw", type=float, default=0.1)
ap.add_argument("--face_yaw", type=float, default=2.0, help="face 접근 뒤 남는 각도 (가정) [deg]")
ap.add_argument("--face", type=float, default=0.20)
ap.add_argument("--dist", type=float, default=0.40)
ap.add_argument("--ang", type=float, default=55.0)
ap.add_argument("--modes", nargs="+", default=["box", "puck", "face"])
ap.add_argument("--aim_err", type=float, default=0.01, help="face · box_meas 보는 점 오차 [m]")
ap.add_argument("--body", action="store_true", help="자세마다 차체 CoACD · 실은 퍽 거리 (명령 · 처짐), isaacsim python 필요 (scipy)")
ap.add_argument("--aim", choices=["map", "hybrid"], default="hybrid", help="box · puck 의 보는 점: map = 지도 상자 중심, hybrid = 앞뒤는 TFmini 로 선 면 + 반 변, 옆은 지도 상자 중심 (face 는 늘 잰 앞면 가운데 + 반 변 ±0.01)")
a = ap.parse_args()

AV = os.path.expanduser("~/git/lv2_next_module/261111_1700_align_view")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.argv = ["align_search.py", os.path.join(HERE, "waffle_4wd_dev_c689a38.urdf"), "--pose", "-1.37", "-0.01", "1.85"]
ns = {"__name__": "x"}
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(AV, "align_search.py")).read(), ns)   # 11-12 고친 fk (base_footprint 기준)
sys.stdout = _o
fk = ns["fk"]
LOOK = os.path.expanduser("~/git/lv2_next_module/261111_2240_box_view/look_tool")
FX, FY, CX, CY, W, H = 602.35, 793.36, 530.64, 359.53, 1280, 720
FRONT, HALF, ZTOP, CELL, TAG = 0.1328, 0.1025, 0.07, 0.0683, 0.016
LO = np.array([-3.14159, -2.14, -1.6, -1.92, -2.8])
HI = np.array([3.14159, 1.6, 1.6, 2.0, 2.8])
ICR = np.array([-0.0658, 0.0])
T_ab = fk("arm_base_link", {})
LINKS = ["link2", "link3", "link4", "link5", "gripper_roll_link", "arm_camera_link"]
NOM = np.array([FRONT + a.face + HALF, 0.0])   # 계획: 로봇 원점에서 본 지도 상자 중심 (로봇이 계획 자리에 정확히 섰을 때)


def R2(t):
    return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])


def box_corners(c, yaw, h=HALF):
    return np.array([c + R2(yaw) @ np.array([sx * h, sy * h]) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def ray_hit(c, yaw):
    """로봇 좌표: TFmini 빔 (y 0, x 앞으로, 차체 앞 끝에서) 이 상자 테두리와 처음 만나는 x. 없으면 None."""
    P = box_corners(c, yaw)
    xs = []
    for i in range(4):
        p, q = P[i], P[(i + 1) % 4]
        if (p[1] - 0) * (q[1] - 0) <= 0 and p[1] != q[1]:
            xs.append(p[0] + (0 - p[1]) * (q[0] - p[0]) / (q[1] - p[1]))
    xs = [x for x in xs if x > FRONT - 1e-6]
    return min(xs) if xs else None


def to_robot(p_map, pose):
    """지도(=계획 로봇 좌표) 점 -> 로봇 좌표. pose = (x, y, yaw) 로봇 원점 (지도 기준)."""
    return R2(-pose[2]) @ (np.asarray(p_map) - pose[:2])


def turn_about_icr(pose, dpsi):
    c = pose[:2] + R2(pose[2]) @ ICR
    xy = c + R2(dpsi) @ (pose[:2] - c)
    return np.array([xy[0], xy[1], pose[2] + dpsi])


def advance_to_face(pose, box_c, box_yaw, target):
    """로봇 x 방향으로 움직여 TFmini 거리 = target. 실패면 None."""
    bc = to_robot(box_c, pose)
    hit = ray_hit(bc, box_yaw - pose[2])
    if hit is None:
        return None
    step = hit - FRONT - target
    return np.array([pose[0] + step * math.cos(pose[2]), pose[1] + step * math.sin(pose[2]), pose[2]])


def look(aim_robot):
    aim = np.linalg.inv(T_ab) @ np.array([aim_robot[0], aim_robot[1], ZTOP, 1.0])
    out = subprocess.run([LOOK], input=f"{aim[0]} {aim[1]} {aim[2]} 0.04 1 {a.dist} 1 {a.ang}\n", capture_output=True, text=True).stdout.split()
    if len(out) < 8 or out[2] != "1":
        return None
    return [float(x) for x in out[3:8]]


def sag(q):
    return [q[0], q[1] - 0.02, q[2], q[3] + 0.02, q[4]]


def margin(Ti, P3):
    k = np.c_[P3, np.ones(len(P3))] @ Ti.T
    if (k[:, 2] <= 0.02).any():
        return -1e9
    u, v = FX * k[:, 0] / k[:, 2] + CX, FY * k[:, 1] / k[:, 2] + CY
    return float(min(u.min(), W - u.max(), v.min(), H - v.max()))


def judge(q, bc, by):
    """bc · by = 로봇 좌표 실제 상자 중심 · 요."""
    res = dict(place=1e9, cells=1e9, tag=1e9, jm=1e9, inbox=False)
    for qq in (q, sag(q)):
        qd = {f"joint{i + 1}": qq[i] for i in range(5)}
        Ti = np.linalg.inv(fk("arm_camera_optical_frame", qd))
        pc = box_corners(bc, by, HALF + 0.015)
        res["place"] = min(res["place"], margin(Ti, np.c_[pc, np.full(4, ZTOP)]))
        cells = []
        for cx_, cy_ in itertools.product((-CELL, 0, CELL), repeat=2):
            for dx, dy in itertools.product((-0.015, 0.015), repeat=2):
                cells.append(bc + R2(by) @ np.array([cx_ + dx, cy_ + dy]))
        cells = np.array(cells)
        for z in (ZTOP + 0.06, ZTOP + 0.03):
            res["cells"] = min(res["cells"], margin(Ti, np.c_[cells, np.full(len(cells), z)]))
        for cx_, cy_ in itertools.product((-CELL, 0, CELL), repeat=2):
            for tr in (0.0, math.pi / 4):
                sq = np.array([[math.cos(tr + k * math.pi / 2), math.sin(tr + k * math.pi / 2)] for k in range(4)]) * TAG / math.sqrt(2)
                P = np.array([bc + R2(by) @ (np.array([cx_, cy_]) + s) for s in sq])
                k = np.c_[P, np.full(4, ZTOP + 0.06), np.ones(4)] @ Ti.T
                uv = np.c_[FX * k[:, 0] / k[:, 2], FY * k[:, 1] / k[:, 2]]
                res["tag"] = min(res["tag"], min(np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4)))
        res["jm"] = min(res["jm"], float(min(np.min(np.array(qq) - LO), np.min(HI - np.array(qq)))))
        Tg = fk("gripper_roll_link", qd)
        pts = np.array([fk(l, qd)[:3, 3] for l in LINKS] + [(Tg @ np.array([x, y, 0, 1]))[:3] for x, y in ((0.14, 0.0), (0.17, 0.031), (0.17, -0.031))])
        loc = (R2(-by) @ (pts[:, :2] - bc).T).T
        res["inbox"] |= bool(((np.abs(loc) < HALF + 0.03).all(axis=1) & (pts[:, 2] < ZTOP + 0.04)).any())
    return res


def run(mode):
    if mode == "box_meas":   # approach_box 그대로 (각도 안 맞춤), 보는 점만 잰 상자 중심 (라이다 · D435 로 미리 잼, ±0.01 가정)
        return run("box")
    rows = []
    D, BY = a.box_d, math.radians(a.box_yaw)
    for bdx, bdy, byaw, nx, ny, nyaw in itertools.product((-D, 0, D), (-D, 0, D), (-BY, 0, BY), (-a.nav_xy, 0, a.nav_xy),
                                                          (-a.nav_xy, 0, a.nav_xy), (-a.nav_yaw, 0, a.nav_yaw)):
        box_c = NOM + np.array([bdx, bdy])        # 지도(계획 로봇) 좌표 실제 상자
        pose = np.array([nx, ny, nyaw])            # Nav2 도착
        if mode == "puck":
            worst = None
            for cell in itertools.product((-CELL, 0, CELL), repeat=2):   # 겨누는 퍽 칸 (어느 것이든)
                puck = box_c + R2(byaw) @ np.array(cell)
                pr = to_robot(puck, pose)
                p2 = pose.copy()
                for _ in range(4):   # 로봇 x 축이 퍽을 지나게 (ICR 둘레로 돎)
                    pr = to_robot(puck, p2)
                    p2 = turn_about_icr(p2, math.atan2(pr[1], pr[0] - ICR[0]))
                p2 = advance_to_face(p2, box_c, byaw, a.face)
                if p2 is None:
                    continue
                rows.append((p2, box_c, byaw, (bdx, bdy, math.degrees(byaw), nx, ny, nyaw, cell)))
            continue
        if mode == "face":
            rel = rng.uniform(-a.face_yaw, a.face_yaw) if False else 0.0
            pose = turn_about_icr(pose, (byaw - pose[2]) + math.radians(a.face_yaw) * np.sign(byaw - pose[2] + 1e-9))
        p2 = advance_to_face(pose, box_c, byaw, a.face)
        if p2 is None:
            continue
        rows.append((p2, box_c, byaw, (bdx, bdy, math.degrees(byaw), nx, ny, nyaw, None)))
    return rows


def summarize(mode, rows):
    print(f"\n== 접근 {mode}: 경우 {len(rows)} (TFmini 빔이 상자에 안 닿는 경우는 뺌)")
    rel_yaw, lat, fwd = [], [], []
    worst = {"place": (1e9, None), "cells": (1e9, None), "tag": (1e9, None), "jm": (1e9, None)}
    inbox = 0
    nik = 0
    cache = {}
    for p2, box_c, byaw, meta in rows:
        bc = to_robot(box_c, p2)
        by = byaw - p2[2]
        mc = to_robot(NOM, p2)                      # 지도 상자 중심 (위치 추정 오차 0)
        if mode in ("face", "box_meas"):
            aim = bc + np.array([a.aim_err, a.aim_err])   # 잰 중심 오차 (가정, 나쁜 쪽 하나)
        elif a.aim == "map":
            aim = mc
        else:
            aim = np.array([FRONT + a.face + HALF, mc[1]])
        rel_yaw.append(math.degrees((by + math.pi / 4) % (math.pi / 2) - math.pi / 4))
        lat.append(bc[1])
        fwd.append(bc[0] - NOM[0])
        key = (round(aim[0], 3), round(aim[1], 3))
        if key not in cache:
            cache[key] = look(aim)
        q = cache[key]
        if q is None:
            nik += 1
            continue
        r = judge(q, bc, by)
        for k in ("place", "cells", "tag", "jm"):
            if r[k] < worst[k][0]:
                worst[k] = (r[k], meta + (tuple(np.round(q, 3)),))
        inbox += r["inbox"]
    print(f"  로봇 기준 상자 요(±45° 접음) {min(rel_yaw):+.1f} ~ {max(rel_yaw):+.1f}°, 옆 {min(lat):+.3f} ~ {max(lat):+.3f} m, "
          f"앞뒤(계획 대비) {min(fwd):+.3f} ~ {max(fwd):+.3f} m")
    for k, lab in (("place", "place_box 여유 px"), ("cells", "9 칸 여유 px"), ("tag", "태그 1280 px"), ("jm", "관절 여유 rad")):
        v, m = worst[k]
        print(f"  가장 나쁜 {lab}: {v:.2f}  (상자 dx dy 요 · Nav2 x y 요 · 칸 · q = {m})")
    print(f"  팔이 실제 상자 영역에 듦: {inbox} 경우 · box_look IK 못 풂(한계 밖): {nik} 경우")
    if a.body:
        import cand_eval as ce   # 차체 CoACD · 실은 퍽 (dev 정밀 URDF), 명령 · 처짐 중 작은 값
        qs = [q for q in cache.values() if q is not None]
        bd = [ce.body(q, True) for q in qs]
        b = min(x[0] for x in bd); pk = min(x[1] for x in bd)
        print(f"  자세 {len(qs)} 개: 차체 CoACD 최소 {b:.1f} mm · 실은 퍽 최소 {pk:.1f} mm · j1 범위 {min(q[0] for q in qs):+.3f} ~ {max(q[0] for q in qs):+.3f}")


rng = np.random.default_rng(0)
print(f"보는 점 {a.aim} · 가정: 상자 ±{a.box_d} m · ±{a.box_yaw}°, Nav2 ±{a.nav_xy} m · ±{a.nav_yaw} rad, 면 {a.face} m, box_look {a.dist} m {a.ang}°, "
      f"face 접근 남는 각도 {a.face_yaw}°")
for m in a.modes:
    summarize(m, run(m))
