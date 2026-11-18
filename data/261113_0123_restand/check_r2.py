"""R2 판정 (모의): mock_robot.py 기록 (참 상자 · 차체 자세) 과 계획기 기하로 mission 결정을 따로 다시 셈.

  python3 check_r2.py <결과 폴더> [--odom_offset 0.3,-0.2,10] [--icr_x -0.0658]
  묶음 (measure_front 하나 = 시작 박스 접근 하나) 마다:
  - 직육면체 집기 (turn 뒤 target_look) 의 참 퍽 · 계획 final (자세표, mock 의 navigate 목표로 접근 찾음) 에서 0.01° 격자로 #247 회전 r 을 셈
  - 기대 결정: r 최대 > 20 이고 |β| < 40 이고 계획기 Solid 로 새 자리 (final · 길 · 회전) 가 안 막히면 '다시 섬', 아니면 '회전만'
  - 실제: mock 의 FINE 자세가 자세표 final 이 아니면 '다시 섬'. 기대와 같아야 함, mission 기록 decision 과도 같아야 함
  - 다시 선 집기: mock 이 실제로 돈 뒤 J1 -> 참 퍽 방향과 참 면의 어긋남 <= 5°, 보낸 n 이 격자 최소와 0.1° 안
  - 회전만 집기: 실제 r 이 격자 최소와 0.1° 안 (#247 S2 와 같은 판정)
  - target_look:odom 이 참 퍽과 2 mm 안, 최종 DONE
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--odom_offset", default="0.3,-0.2,10")
ap.add_argument("--icr_x", type=float, default=-0.0658)
ap.add_argument("--ws", default=os.path.expanduser("~/git/Lv2_Cartlidar_Assignment_restand"))
ap.add_argument("--restand_file", default="", help="mission 에 준 막힘 자료 (기본 = worktree mission/config, 기대 결정은 이것과 계획기 Solid 둘 다로)")
a = ap.parse_args()
SRC = os.path.join(a.ws, "workspace/ros2_ws/src")
sys.path.insert(0, os.path.join(SRC, "mission/tools"))
import make_restand_obstacles as rs  # noqa: E402
from mission_planner.poses import CENTER_X, J1_X, Pose  # noqa: E402

out = json.load(open(os.path.join(a.run, "mock_out.json"), encoding="utf-8"))
logs = glob.glob(os.path.join(a.run, "log", "*", "mission_log.jsonl"))
mlog = [json.loads(l) for l in open(logs[0], encoding="utf-8")] if logs else []
rs_log = [e for e in mlog if e.get("kind") == "start_box" and e.get("event") == "restand"]
table = rs.table_from(os.path.join(SRC, "mission_planner/config/approach_poses.yaml"))
solids = rs.solids_of(os.path.join(SRC, "mission_planner/worlds/dice_mission.sdf"))
rfile = yaml.safe_load(open(a.restand_file or os.path.join(SRC, "mission/config/restand_obstacles.yaml"), encoding="utf-8"))
ox, oy, oyaw = (float(v) for v in a.odom_offset.split(","))
oyaw = math.radians(oyaw)
mission_log_text = open(os.path.join(a.run, "mission.log"), encoding="utf-8", errors="replace").read()


def odom_to_map(px, py):
    return ox + px * math.cos(oyaw) - py * math.sin(oyaw), oy + px * math.sin(oyaw) + py * math.cos(oyaw)


def name_deg(name):
    s = name[5:]
    return 0.0 if s == "0" else (1.0 if s[0] == "p" else -1.0) * float(s[1:])


def j1_after(pose, n_deg):
    x, y, yaw = pose
    r = J1_X - a.icr_x
    phi = np.arcsin((J1_X - CENTER_X) * np.sin(np.radians(n_deg)) / r)
    ix, iy = x + a.icr_x * math.cos(yaw), y + a.icr_x * math.sin(yaw)
    return ix + r * np.cos(yaw + phi), iy + r * np.sin(yaw + phi)


def mis(j, puck, face):
    d = np.degrees(np.arctan2(puck[1] - j[1], puck[0] - j[0]))
    return np.abs((d - face + 45.0) % 90.0 - 45.0)


def grid(pose, puck, face, lo, hi):
    n = np.arange(round(lo * 100), round(hi * 100) + 1) / 100.0
    m = mis(j1_after(pose, n), puck, face)
    k = np.lexsort((np.abs(n), np.round(m, 9)))[0]
    return float(n[k]), float(m[k])


# 묶음 나누기
groups, cur, pend = [], None, None
for ev in out["events"]:
    if ev["kind"] == "measure_front":
        cur = dict(truth=ev, picks=[], fine=None)
        groups.append(cur)
        continue
    if cur is None:
        continue
    if ev["kind"] == "drive_skill" and ev["name"].startswith("turn_") and name_deg(ev["name"]) != 0.0:
        pend = ev
    elif ev["kind"] == "skill" and ev["name"].startswith("target_look:"):
        f = ev["name"].split(":")
        cur["picks"].append(dict(turn=pend, look=(f[1], float(f[2]), float(f[3]))))
        pend = None
fails, rows = [], []
li = 0
for g in groups:
    t = g["truth"]
    key = next((k for k, e in table.items() if math.hypot(e["stage"][0] - t["nav"][0], e["stage"][1] - t["nav"][1]) < 0.01), None)
    e = table[key]
    F = Pose(*e["final"])
    face = t["rot_deg"]
    lo, hi = max(e["turn_range_deg"][0], -30.0), min(e["turn_range_deg"][1], 30.0)
    cub = [p for p in g["picks"] if p["turn"] is not None]
    for p in g["picks"]:
        mx, my = odom_to_map(p["look"][1], p["look"][2])
        sx, puck = min(t["pucks"].items(), key=lambda kv: math.hypot(kv[1][0] - mx, kv[1][1] - my))
        p["puck"], p["sx"] = puck, sx
        err = math.hypot(puck[0] - mx, puck[1] - my) * 1000
        p["look_err_mm"] = err
        if p["look"][0] != "odom" or err > 2.0:
            fails.append(f"{key} target_look {p['look']} 참 퍽에서 {err:.2f} mm")
    if not cub:
        continue
    r_pred = max(grid((F.x, F.y, F.yaw), p["puck"], face, lo, hi)[1] for p in cub)
    c0 = (F.x + 0.3625 * math.cos(F.yaw), F.y + 0.3625 * math.sin(F.yaw))
    Fn, Sn = rs.rs_poses(F, Pose(*e["stage"]), c0, t["center"], math.radians(face))
    box = rs.START_BOXES[e["box"]]
    tmax = rs.turn_max_of(e)
    blk = rs.rs_blocked_solids([s for s in solids if not rs.is_target(s, box)], Fn, Sn, tmax)
    blk_file = rs.rs_blocked_polys([np.array(q).reshape(-1, 2) for q in rfile["approaches"][key]["obstacles"]], Fn, Sn, tmax)
    amb = abs(face) >= 40.0
    expect = "small" if r_pred <= 20.0 else "ambiguous" if amb else ("restand" if blk_file == "none" else "blocked")
    before = cub[0]["turn"]["before"]
    restood = math.hypot(before[0] - F.x, before[1] - F.y) > 0.005
    lg = rs_log[li] if li < len(rs_log) else {}
    li += 1
    if (expect == "restand") != restood:
        fails.append(f"{key} 기대 {expect} 인데 실제 {'다시 섬' if restood else '회전만'}")
    if lg.get("decision") != expect:
        fails.append(f"{key} mission 기록 {lg.get('decision')} vs 기대 {expect}")
    if expect == "blocked" and lg.get("block") != blk_file:
        fails.append(f"{key} 막힘 종류 기록 {lg.get('block')} vs {blk_file}")
    if restood and math.hypot(before[0] - Fn.x, before[1] - Fn.y) > 0.002:
        fails.append(f"{key} 다시 선 FINE 자리 {before} vs 계산 {Fn.as_list()}")
    for p in cub:
        n_sent = name_deg(p["turn"]["name"])
        bp = p["turn"]["before"]
        glo, ghi = (max(lo, -tmax), min(hi, tmax)) if restood else (lo, hi)
        ng, rg = grid(bp, p["puck"], face, glo, ghi)
        af = p["turn"]["after"]
        jr = (af[0] + J1_X * math.cos(af[2]), af[1] + J1_X * math.sin(af[2]))
        r_real = float(mis(jr, p["puck"], face))
        if abs(n_sent - ng) > 0.1:
            fails.append(f"{key} {p['turn']['name']}: 격자 {ng:+.2f}")
        if abs(r_real - rg) > 0.1:
            fails.append(f"{key} {p['turn']['name']}: 실제 r {r_real:.2f} vs 격자 {rg:.2f}")
        if restood and r_real > 5.0:
            fails.append(f"{key} 다시 선 뒤 r {r_real:.2f} > 5")
        rows.append(dict(key=key, puck=p["sx"], beta=face, expect=expect, mission=lg.get("decision"), block=lg.get("block"),
                         restood=restood, plan_r=round(r_pred, 2), sent=p["turn"]["name"], r_real=round(r_real, 3),
                         look_mm=round(p["look_err_mm"], 3), polys_vs_solids=blk_file == blk))
amb_warn = mission_log_text.count("부호 모호")
skip_warn = mission_log_text.count("다시 서지 않음")
if out["final"].get("state") != "DONE":
    fails.append(f"final {out['final']}")
if li != len(rs_log):
    fails.append(f"restand 기록 {len(rs_log)} vs 묶음 {li}")
print(json.dumps(dict(run=a.run, final=out["final"].get("state"), navigate=len(out["navigate"]), elapsed_s=out["elapsed_s"],
                      restood=sum(1 for r in rows if r["restood"]), amb_warn_lines=amb_warn, skip_warn_lines=skip_warn,
                      max_look_mm=max((p["look_err_mm"] for g in groups for p in g["picks"]), default=None),
                      rows=rows, fails=fails), ensure_ascii=False))
