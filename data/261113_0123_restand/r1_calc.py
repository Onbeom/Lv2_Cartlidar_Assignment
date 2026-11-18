"""R1 (계산, 모의 아님): 다시 서기 막힘 · 남는 면 어긋남. 계획기 기하 (mission_planner poses · arena) 로 직접 셈.

  python3 r1_calc.py [--ws ~/git/Lv2_Cartlidar_Assignment_restand] [--out r1_result.json]
  - 8 개 ok 시작 박스 접근 × β ±25 ~ ±45 (5°) × 밀림 (dx, dy) ∈ {-0.05, 0, 0.05}² (잰 상자 = 참 상자로 둠)
  - #247 (회전만): 계획 final 에서 0.01° 격자로 남는 면 어긋남이 가장 작은 이름 각 (범위 turn_range ∩ ±30, 실제 회전 중심 icr_x)
  - 다시 서기: 자세표 접근을 상자 중심 둘레로 β 돌려 잰 중심으로 옮긴 final · stage, 막힘 (final · 길 · 회전) 을
    계획기 Solid 로 직접 (footprint_clear) + mission 이 읽는 다각형 파일 (restand_obstacles.yaml) 로 따로 판정해 같은지 봄.
    막히지 않으면 새 final 에서 0.01° 격자 (범위 ±turn_max ∩ ±30) 로 남는 어긋남.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--ws", default=os.path.expanduser("~/git/Lv2_Cartlidar_Assignment_restand"))
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "r1_result.json"))
ap.add_argument("--icr_x", type=float, default=-0.0658)
a = ap.parse_args()
SRC = os.path.join(a.ws, "workspace/ros2_ws/src")
sys.path.insert(0, os.path.join(SRC, "mission/tools"))
import make_restand_obstacles as rs  # noqa: E402
from mission_planner.poses import CENTER_X, J1_X, PUCK_DX, START_BOXES, Pose  # noqa: E402

BOX_X = 0.3625
MAXT = 30.0
solids = rs.solids_of(os.path.join(SRC, "mission_planner/worlds/dice_mission.sdf"))
table = rs.table_from(os.path.join(SRC, "mission_planner/config/approach_poses.yaml"))
rfile = yaml.safe_load(open(os.path.join(SRC, "mission/config/restand_obstacles.yaml"), encoding="utf-8"))


def fold(d):
    return (d + 45.0) % 90.0 - 45.0


def grid_best(pose: Pose, puck, face_deg, lo, hi):
    """0.01° 격자: 이름 각 n 으로 돈 뒤 (drive 와 같이 icr 둘레 phi) J1 -> 퍽 방향과 면의 어긋남 최소 (같으면 |n| 작은 쪽)."""
    n = np.arange(round(lo * 100), round(hi * 100) + 1) / 100.0
    r = J1_X - a.icr_x
    phi = np.arcsin((J1_X - CENTER_X) * np.sin(np.radians(n)) / r)
    ix, iy = pose.x + a.icr_x * math.cos(pose.yaw), pose.y + a.icr_x * math.sin(pose.yaw)
    ang = pose.yaw + phi
    jx, jy = ix + r * np.cos(ang), iy + r * np.sin(ang)
    d = np.degrees(np.arctan2(puck[1] - jy, puck[0] - jx))
    m = np.abs((d - face_deg + 45.0) % 90.0 - 45.0)
    k = np.lexsort((np.abs(n), np.round(m, 9)))[0]
    return float(n[k]), float(m[k])


rows = []
betas = [-45, -40, -35, -30, -25, 25, 30, 35, 40, 45]
shifts = [(dx, dy) for dx in (-0.05, 0.0, 0.05) for dy in (-0.05, 0.0, 0.05)]
for key, e in table.items():
    F, S = Pose(*e["final"]), Pose(*e["stage"])
    c0 = (F.x + BOX_X * math.cos(F.yaw), F.y + BOX_X * math.sin(F.yaw))   # mission 사전값 (= 상자 지도 중심)
    box = START_BOXES[e["box"]]
    target_free = [s for s in solids if not rs.is_target(s, box)]
    polys = [np.array(q).reshape(-1, 2) for q in rfile["approaches"][key]["obstacles"]]
    tmax = rfile["approaches"][key]["turn_max_deg"]
    lo, hi = max(e["turn_range_deg"][0], -MAXT), min(e["turn_range_deg"][1], MAXT)
    inline = e["inline"]
    for b in [0] + betas:
        for dx, dy in (shifts if b else [(0.0, 0.0)]):
            C = (box[0] + dx, box[1] + dy)
            rot = math.radians(b)
            pk = {sx: (C[0] + sx * PUCK_DX * math.cos(rot), C[1] + sx * PUCK_DX * math.sin(rot)) for sx in (-1, 1)}
            fb = {sx: grid_best(F, p, b, 0.0, 0.0) if inline else grid_best(F, p, b, lo, hi) for sx, p in pk.items()}
            Fn, Sn = rs.rs_poses(F, S, c0, C, rot)
            blk_s = rs.rs_blocked_solids(target_free, Fn, Sn, tmax)
            blk_p = rs.rs_blocked_polys(polys, Fn, Sn, tmax)
            rlo, rhi = max(lo, -tmax), min(hi, tmax)
            after = {sx: grid_best(Fn, p, b, rlo, rhi) for sx, p in pk.items()}
            need = (not inline) and max(v[1] for v in fb.values()) > 20.0
            amb = abs(b) >= 40
            rows.append(dict(key=key, beta=b, dx=dx, dy=dy, inline=inline, fb={sx: [round(v[0], 2), round(v[1], 3)] for sx, v in fb.items()},
                             need=need, ambiguous=amb, blocked=blk_s, blocked_polys=blk_p,
                             restand=bool(need and not amb and blk_s == "none"),
                             after={sx: [round(v[0], 2), round(v[1], 3)] for sx, v in after.items()},
                             final=[round(v, 4) for v in (Fn.x, Fn.y, Fn.yaw)], stage=[round(v, 4) for v in (Sn.x, Sn.y, Sn.yaw)]))

json.dump(rows, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=0)
# 요약
base = [r for r in rows if r["beta"] == 0]
cases = [r for r in rows if r["beta"] != 0]
print("β 0 (계획 그대로) 막힘:", {r["key"]: r["blocked"] for r in base})
print(f"경우 {len(cases)} (접근 {len(table)} × β {len(betas)} × 밀림 {len(shifts)})")
dis = [r for r in rows if r["blocked"] != r["blocked_polys"]]
print("Solid 판정 vs 다각형 파일 판정 다름:", len(dis), dis[:3])
fails = []
print("\n| 접근 | β | 회전만 (#247) 최대 r [°] | 20° 넘음 | 모호 (|β|≥40) | 막힘 final · 길 · 회전 | 다시 섬 | 다시 선 뒤 최대 r [°] |")
print("|---|---|---|---|---|---|---|---|")
for key in table:
    for b in betas:
        g = [r for r in cases if r["key"] == key and r["beta"] == b]
        fbmax = max(max(v[1] for v in r["fb"].values()) for r in g)
        fbmin = min(max(v[1] for v in r["fb"].values()) for r in g)
        need = sum(r["need"] for r in g)
        amb = len(g) if abs(b) >= 40 else 0
        bl = {k: sum(r["blocked"] == k for r in g) for k in ("final", "path", "turn")}
        rsd = sum(r["restand"] for r in g)
        ok_cases = [r for r in g if not r["inline"] and not r["ambiguous"] and r["blocked"] == "none"]
        amax = max((max(v[1] for v in r["after"].values()) for r in ok_cases), default=None)
        for r in ok_cases:
            if max(v[1] for v in r["after"].values()) > 5.0:
                fails.append(r)
        print(f"| {key}{' (inline)' if g[0]['inline'] else ''} | {b:+d} | {fbmin:.1f} ~ {fbmax:.1f} | {need}/9 | {amb}/9 | "
              f"{bl['final']} · {bl['path']} · {bl['turn']} | {rsd}/9 | {'-' if amax is None else f'{amax:.2f}'} |")
sn = [r for r in cases if not r["inline"]]
tot = dict(cases=len(cases), sn=len(sn), need=sum(r["need"] for r in sn), ambiguous=sum(r["ambiguous"] for r in sn),
           need_and_amb=sum(r["need"] and r["ambiguous"] for r in sn),
           blocked={k: sum(r["blocked"] == k for r in sn) for k in ("final", "path", "turn")},
           need_blocked={k: sum(r["need"] and not r["ambiguous"] and r["blocked"] == k for r in sn) for k in ("final", "path", "turn")},
           restand=sum(r["restand"] for r in sn),
           after_max=max(max(v[1] for v in r["after"].values()) for r in sn if not r["ambiguous"] and r["blocked"] == "none"),
           fallback_max_when_not_restood=max((max(v[1] for v in r["fb"].values()) for r in sn if r["need"] and not r["restand"]), default=None),
           fails_after_gt5=len(fails), polys_disagree=len(dis))
print("\n합계 (inline 제외):", json.dumps(tot, ensure_ascii=False))
