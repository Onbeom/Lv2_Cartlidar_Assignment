"""mujoco_drive.py 결과 폴더(log.csv, summary.json)를 판정해 verdict.json을 쓴다.

M 시험 판정 기준은 Isaac 검증계획(P1·P5~P8, v3 이후 위치 차분 기준)과 같다(2026-10-23 사용자 승인).
K1·K2(물리 일관성)는 M1에서, K4는 M6에서 계산한다. K3(dt 수렴)은 compare_drive.py가 한다.
속도는 위치 차분(0.1 s 창)으로 판정한다. 엔진 보고 속도는 참고로 남긴다.

판정 표기: 성공 / 실패. 실행(PC): python3 drive/analyze_drive.py <결과 폴더>
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

SETTLE = 1.0
WHEEL_LIMIT = 70 * 2 * math.pi / 60  # XM430-W210 무부하 속도(11.1 V)


def load(folder):
    s = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    with (folder / "log.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    c = {}
    for k in rows[0]:
        if k == "segment":
            c[k] = [r[k] for r in rows]
        elif k == "finite":
            c[k] = np.array([r[k] == "True" for r in rows])
        else:
            c[k] = np.array([float(r[k]) for r in rows])
    return s, c


def fd_rate(x, t, win):
    v = np.full_like(x, np.nan)
    v[win:] = (x[win:] - x[:-win]) / (t[win:] - t[:-win])
    return v


def win_of(s):
    return max(1, int(round(0.1 / s["conditions"]["dt_s"])))  # 0.1 s 창


def at(c, t):
    return int(np.argmin(np.abs(c["t_s"] - t)))


def finite_ok(s):
    return s["nonfinite_first_t"] is None and s["steps_recorded"] == s["expected_steps"]


def verdict(ok):
    return "성공" if ok else "실패"


def link_names(c):
    return sorted({k[:-4] for k in c if k.endswith("_x_m") and k not in ("base_x_m", "com_x_m", "cop_x_m")})


def m1(s, c):
    t, win = c["t_s"], win_of(s)
    i1, after = at(c, 1.0), c["t_s"] >= 1.0
    dz = float(np.max(np.abs(c["base_z_m"][after] - c["base_z_m"][i1])))
    droll = math.degrees(float(np.max(np.abs(c["roll_rad"][after] - c["roll_rad"][i1]))))
    dpitch = math.degrees(float(np.max(np.abs(c["pitch_rad"][after] - c["pitch_rad"][i1]))))
    sp = np.stack([np.sqrt(sum(fd_rate(c[f"{n}_{a}_m"], t, win) ** 2 for a in "xyz")) for n in link_names(c)], axis=1)
    vmax = float(np.nanmax(sp[after]))
    checks = {"유한값·스텝 수": finite_ok(s), "base 높이 변화 ≤ 3 mm": dz <= 0.003,
              "roll·pitch 변화 ≤ 1°": max(droll, dpitch) <= 1.0, "모든 링크 속도(위치 차분) ≤ 0.01 m/s": vmax <= 0.01}
    res = {"M1": {"measured": {"base_z_at_1s_m": float(c["base_z_m"][i1]), "base_z_change_max_m": dz,
                               "roll_change_max_deg": droll, "pitch_change_max_deg": dpitch,
                               "링크속도_위치차분_최대_1s이후_mps": vmax, "자기충돌_접촉수_최대": int(np.max(c["self_contacts"]))},
                  "checks": checks, "verdict": verdict(all(checks.values()))}}

    # K1: 정지 상태에서 바닥 수직 반력 합 = 무게. K2: 반력 작용점(압력 중심) = 질량 중심(수평 위치)
    W = s["model"]["weight_N"]
    Fn = c["Fn_floor_total_N"][after]
    k1_err = float(abs(np.mean(Fn) - W) / W)
    k1c = {"평균 수직 반력 합이 무게의 ±1% 안": k1_err <= 0.01, "바퀴 아닌 부분의 바닥 접촉 없음": float(np.max(c["Fn_floor_other_N"])) == 0.0}
    res["K1"] = {"measured": {"weight_N(질량×g)": W, "mean_Fn_total_N": float(np.mean(Fn)), "relative_error": k1_err,
                              "Fn_total_min_max_N": [float(np.min(Fn)), float(np.max(Fn))],
                              "mean_Fn_each_wheel_N": {k[3:]: float(np.mean(c[k][after])) for k in c if k.startswith("Fn_Wheel_")}},
                 "checks": k1c, "verdict": verdict(all(k1c.values()))}
    d_cop = np.hypot(c["cop_x_m"][after] - c["com_x_m"][after], c["cop_y_m"][after] - c["com_y_m"][after])
    k2 = float(np.mean(d_cop))
    res["K2"] = {"measured": {"mean_|CoP−CoM|_xy_m": k2, "max_m": float(np.max(d_cop)),
                              "com_xy_m": [float(np.mean(c["com_x_m"][after])), float(np.mean(c["com_y_m"][after]))],
                              "cop_xy_m": [float(np.mean(c["cop_x_m"][after])), float(np.mean(c["cop_y_m"][after]))]},
                 "checks": {"압력 중심과 질량 중심의 수평 거리(평균) ≤ 1 mm": k2 <= 0.001}, "verdict": verdict(k2 <= 0.001)}
    return res


def m5(s, c):
    t, win = c["t_s"], win_of(s)
    w = [k[2:] for k in c if k.startswith("q_Wheel_")]
    vfd = {n: float(np.nanmax(np.abs(fd_rate(c["q_" + n], t, win)))) for n in w}
    vrep = {n: float(np.max(np.abs(c["dq_" + n]))) for n in w}
    lim = WHEEL_LIMIT * 1.01
    checks = {"유한값·스텝 수": finite_ok(s), f"모든 스텝 |ω|(위치 차분) ≤ {lim:.4f} rad/s": max(vfd.values()) <= lim}
    return {"M5": {"measured": {"바퀴속도_위치차분_최대_rad_s": vfd, "참고_보고속도_최대_rad_s": vrep,
                                "명령_최대_rad_s(제어기에서 자름)": float(np.max(np.abs(c["cmd_" + w[0]])))},
                   "checks": checks, "verdict": verdict(all(checks.values()))}}


def stop_time(c, speed, t_cmd, t_end):
    idx = np.where((c["t_s"] > t_cmd) & (c["t_s"] <= t_end))[0]
    above = idx[speed[idx] > 0.005]
    if len(above) == 0:
        return 0.0
    if above[-1] == idx[-1]:
        return None  # 끝까지 0.005 m/s 아래로 내려가지 않음
    return float(c["t_s"][above[-1] + 1] - t_cmd)


def m6_m8(s, c):
    t, win = c["t_s"], win_of(s)
    i0 = at(c, SETTLE)
    yaw0 = c["yaw_rad"][i0]
    h = np.array([math.cos(yaw0), math.sin(yaw0)])
    n = np.array([-h[1], h[0]])
    rel = np.stack([c["base_x_m"] - c["base_x_m"][i0], c["base_y_m"] - c["base_y_m"][i0]], axis=1)
    along, side = rel @ h, rel @ n
    i_fwd, i_back = at(c, SETTLE + 6.0), at(c, SETTLE + 12.0)
    fwd = float(along[i_fwd] - along[i0])
    back = float(along[i_fwd] - along[i_back])
    run = t >= SETTLE
    side_max = float(np.max(np.abs(side[run])))
    yaw_max = math.degrees(float(np.max(np.abs(np.unwrap(c["yaw_rad"][run]) - yaw0))))
    speed = np.hypot(fd_rate(c["base_x_m"], t, win), fd_rate(c["base_y_m"], t, win))
    st1, st2 = stop_time(c, speed, SETTLE + 4.0, SETTLE + 6.0), stop_time(c, speed, SETTLE + 10.0, SETTLE + 12.0)
    m6c = {"유한값·스텝 수": finite_ok(s), "전진 0.18~0.22 m": 0.18 <= fwd <= 0.22, "후진 0.18~0.22 m": 0.18 <= back <= 0.22,
           "옆 이탈 ≤ 10 mm": side_max <= 0.010, "yaw 변화 ≤ 2°": yaw_max <= 2.0}
    m8c = {"정지 명령 후 0.5 s 안에 ≤ 0.005 m/s(위치 차분, 후진 끝)": st2 is not None and st2 <= 0.5}
    # K4: 직진 미끄럼률 = 1 − 실제 거리 / 바퀴 회전으로 계산한 거리(바퀴 위치 차이 × 반지름, 네 바퀴 평균)
    wq = [k for k in c if k.startswith("q_Wheel_")]
    r = float(np.mean(list(s["model"]["wheel_tire_radius_m"].values())))
    roll_dist = float(np.mean([c[k][i_fwd] - c[k][i0] for k in wq])) * r
    slip = 1.0 - fwd / roll_dist if roll_dist else float("nan")
    ideal = 0.05 * 4.0
    cmd_mean = {k[4:]: float(np.mean(np.abs(c[k][(t >= SETTLE) & (t < SETTLE + 4.0)]))) for k in c if k.startswith("cmd_")}
    return {"M6": {"measured": {"forward_m": fwd, "backward_m": back, "side_max_m": side_max, "yaw_change_max_deg": yaw_max,
                                "definition": "전진 = 1 s~7 s의 처음 방향 성분, 후진 = 7 s~13 s"},
                   "checks": m6c, "verdict": verdict(all(m6c.values()))},
            "M8": {"measured": {"stop_time_after_backward_s": st2, "stop_time_after_forward_s(참고)": st1,
                                "speed_at_end_mps": float(speed[-1])},
                   "checks": m8c, "verdict": verdict(all(m8c.values()))},
            "K4": {"measured": {"forward_m": fwd, "바퀴_회전거리_m": roll_dist, "slip_ratio": slip,
                                "명령_이상값_m(0.05 m/s × 4 s)": ideal, "참고_명령_바퀴속도_평균_rad_s": cmd_mean},
                   "checks": {"전진 미끄럼률 |1 − 실제/바퀴 회전 거리| ≤ 2%": abs(slip) <= 0.02},
                   "verdict": verdict(abs(slip) <= 0.02)}}


def m7(s, c):
    i0 = at(c, SETTLE)
    run = c["t_s"] >= SETTLE
    yaw = np.unwrap(c["yaw_rad"][run])
    d = np.hypot(c["base_x_m"][run] - c["base_x_m"][i0], c["base_y_m"][run] - c["base_y_m"][i0])
    dyaw = math.degrees(float(yaw[-1] - yaw[0]))
    # 미끄럼 없는 이상 회전량(참고): 바퀴 ±1 rad/s × r ÷ (좌우 바퀴 간격/2) × 4 s
    xy = s["model"]["wheel_xy_at_start_m"]
    track = float(np.mean([abs(xy[k][1] - xy[k.replace("_Left", "_Right")][1]) for k in xy if k.endswith("_Left")]))
    r = float(np.mean(list(s["model"]["wheel_tire_radius_m"].values())))
    ideal = math.degrees(1.0 * r / (track / 2) * 4.0)
    checks = {"유한값·스텝 수": finite_ok(s), "yaw가 + 방향(반시계)으로 1° 이상 증가": dyaw >= 1.0,
              "중심 이동 ≤ 30 mm": float(np.max(d)) <= 0.030}
    return {"M7": {"measured": {"yaw_change_deg(기록만)": dyaw, "center_move_max_m": float(np.max(d)),
                                "ideal_no_slip_deg(참고)": ideal, "track_m": track},
                   "checks": checks, "verdict": verdict(all(checks.values()))}}


def main(folder):
    folder = Path(folder)
    s, c = load(folder)
    res = {"M1": m1, "M5": m5, "M6": m6_m8, "M7": m7}[s["test"]](s, c)
    out = {"test": s["test"], "dt_s": s["conditions"]["dt_s"], "mujoco_version": s["mujoco_version"], "results": res}
    (folder / "verdict.json").write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    for k, v in res.items():
        print(f"{folder.name} {k}: {v['verdict']}  " + ", ".join(f"{a}={'O' if b else 'X'}" for a, b in v["checks"].items()))
    return out


if __name__ == "__main__":
    for f in sys.argv[1:]:
        main(f)
