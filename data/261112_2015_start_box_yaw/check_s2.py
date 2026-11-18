"""S2 판정 (모의): mock_robot.py 기록 (참 상자 · 차체 자세) 으로 mission 결정을 따로 다시 셈.

  python3 check_s2.py <결과 폴더> [--odom_offset 0.3,-0.2,10] [--icr_x -0.0658]
  - target_look:odom 자리 vs 참 퍽 (참 상자 중심 + R(돌림) (±0.04, 0)) <= 2 mm
  - turn 마다: 회전 전 자세 (= FINE 자리) 에서 0.01° 격자로 남는 면 어긋남이 가장 작은 이름 각 (mission 과 다른 방법: 격자) 과
    보낸 이름 각 차 <= 0.1°, mock 이 실제로 돈 자세의 J1 -> 참 퍽 방향 vs 참 면 어긋남과 격자 최소의 차 <= 0.1°,
    mission 기록 residual_sent 와 실제 어긋남 차 <= 0.1°
  - drive 거부 0: turn 이름이 drive 규칙 (0 < |deg| < 90, |deg| <= 30) 안, mock 기록 ok
  - 최종 DONE
"""
import argparse
import glob
import json
import math
import os

import yaml

J1 = 0.096          # base_link -> J1 x (poses.J1_X)
C = -0.04035        # 계획기 회전 중심 (poses.CENTER_X)
MAX = 30.0          # drive turn.max_deg


def ang_fold(a_deg):
    """[deg] -> -45..45 (90° 주기)."""
    return (a_deg + 45.0) % 90.0 - 45.0


def name_deg(name):
    s = name[5:]
    if s == "0":
        return 0.0
    return (1.0 if s[0] == "p" else -1.0) * float(s[1:])


def j1_after(pose, n_deg, icr):
    """FINE 자세 pose (지도) 에서 이름 각 n 으로 돈 뒤 J1 (지도): drive 와 같이 icr 둘레 phi."""
    x, y, yaw = pose
    r = J1 - icr
    phi = math.asin((J1 - C) * math.sin(math.radians(n_deg)) / r)
    ix, iy = x + icr * math.cos(yaw), y + icr * math.sin(yaw)
    a = yaw + phi
    return ix + r * math.cos(a), iy + r * math.sin(a)


def misalign(j1, puck, face_deg):
    d = math.degrees(math.atan2(puck[1] - j1[1], puck[0] - j1[0]))
    return abs(ang_fold(d - face_deg))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--odom_offset", default="0.3,-0.2,10")
    ap.add_argument("--icr_x", type=float, default=-0.0658)
    ap.add_argument("--approach", default=os.path.expanduser(
        "~/git/Lv2_Cartlidar_Assignment_startbox/workspace/ros2_ws/src/mission_planner/config/approach_poses.yaml"))
    a = ap.parse_args()
    out = json.load(open(os.path.join(a.run, "mock_out.json"), encoding="utf-8"))
    logs = glob.glob(os.path.join(a.run, "log", "*", "mission_log.jsonl"))
    mlog = [json.loads(l) for l in open(logs[0], encoding="utf-8")] if logs else []
    turns_logged = [e for e in mlog if e.get("kind") == "start_box" and e.get("event") == "turn"]
    table = yaml.safe_load(open(a.approach, encoding="utf-8"))
    finals = {k: v for k, v in table.items() if isinstance(v, dict) and v.get("kind") == "box"}
    ox, oy, oyaw = (float(v) for v in a.odom_offset.split(","))
    oyaw = math.radians(oyaw)

    def odom_to_map(px, py):
        return ox + px * math.cos(oyaw) - py * math.sin(oyaw), oy + px * math.sin(oyaw) + py * math.cos(oyaw)

    fails, rows = [], []
    truth = None
    pending = None      # 직전 turn (다음 target_look 과 짝)
    ti = 0
    looks = 0
    for ev in out["events"]:
        if ev["kind"] == "measure_front":
            truth = ev
            continue
        name = ev["name"]
        if not ev["ok"]:
            fails.append(f"거부됨: {name}")
        if ev["kind"] == "drive_skill" and name.startswith("turn_"):
            d = name_deg(name)
            if not (abs(d) <= MAX and (d == 0.0 or 0 < abs(d) < 90)):
                fails.append(f"drive 규칙 밖: {name}")
            if d != 0.0:
                pending = (ev, d)
            continue
        if ev["kind"] == "skill" and name.startswith("target_look:"):
            f = name.split(":")
            if truth is None:
                continue
            looks += 1
            if f[1] != "odom":
                fails.append(f"odom 아님: {name}")
                continue
            mx, my = odom_to_map(float(f[2]), float(f[3]))
            sx, puck = min(((k, p) for k, p in truth["pucks"].items()), key=lambda kp: math.hypot(kp[1][0] - mx, kp[1][1] - my))
            err = math.hypot(puck[0] - mx, puck[1] - my) * 1000
            if err > 2.0:
                fails.append(f"target_look {name} -> 참 퍽에서 {err:.2f} mm")
            row = dict(box=truth["box"], puck=sx, look_err_mm=round(err, 3))
            if pending is not None:
                tev, n_sent = pending
                pose = tev["before"]
                key = next(k for k, v in finals.items() if math.hypot(v["final"][0] - pose[0], v["final"][1] - pose[1]) < 0.005)
                lo, hi = finals[key].get("turn_range_deg", [-90, 90])
                lo, hi = max(lo, -MAX), min(hi, MAX)
                face = truth["rot_deg"]
                best = None
                for i in range(round(lo * 100), round(hi * 100) + 1):
                    n = i / 100
                    m = misalign(j1_after(pose, n, a.icr_x), puck, face)
                    if best is None or m < best[1] - 1e-9 or (abs(m - best[1]) <= 1e-9 and abs(n) < abs(best[0])):
                        best = (n, m)
                after = tev["after"]
                j1_real = (after[0] + J1 * math.cos(after[2]), after[1] + J1 * math.sin(after[2]))
                r_real = misalign(j1_real, puck, face)
                lg = turns_logged[ti] if ti < len(turns_logged) else {}
                ti += 1
                row.update(key=key, plan=lg.get("plan"), sent=tev["name"], n_sent=n_sent, n_grid=best[0], r_grid=round(best[1], 3),
                           r_real=round(r_real, 3), r_logged=lg.get("residual_sent_deg"), beta=lg.get("beta_deg"))
                if abs(n_sent - best[0]) > 0.1:
                    fails.append(f"{key} {tev['name']}: 격자 {best[0]:.2f}° 와 {abs(n_sent - best[0]):.2f}° 차")
                if abs(r_real - best[1]) > 0.1:
                    fails.append(f"{key} {tev['name']}: 실제 어긋남 {r_real:.2f}° vs 격자 {best[1]:.2f}°")
                if lg and abs(lg["residual_sent_deg"] - r_real) > 0.1:
                    fails.append(f"{key} {tev['name']}: 기록 {lg['residual_sent_deg']:.2f}° vs 실제 {r_real:.2f}°")
                pending = None
            rows.append(row)
    if out["final"].get("state") != "DONE":
        fails.append(f"final {out['final']}")
    print(json.dumps(dict(run=a.run, final=out["final"].get("state"), looks=looks, turns=sum(1 for r in rows if "n_sent" in r),
                          max_look_err_mm=max((r["look_err_mm"] for r in rows), default=None),
                          residuals=[r["r_real"] for r in rows if "r_real" in r], rows=rows, fails=fails), ensure_ascii=False))


if __name__ == "__main__":
    main()
