"""run_phys_test.py 결과 폴더(log.csv, summary.json)를 검증계획 v2 기준으로 판정한다.

판정 표기: 성공 / 실패 / 조건부 / 미실시 / 데이터 부족.
실행(PC, 일반 python3 가능): python3 261022_1243/phys_tests/analyze_phys_test.py <결과 폴더>
결과는 같은 폴더의 verdict.json에 쓴다.
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

SETTLE = 1.0


def load(folder):
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    with (folder / "log.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k not in ("segment", "contact_pairs_minsep_mm")}
    cols["segment"] = [r["segment"] for r in rows]
    cols["contact_pairs"] = [r["contact_pairs_minsep_mm"] for r in rows]
    return summary, cols


FD_WIN = 12  # 위치 차분 창(스텝): 12 × 1/120 s = 0.1 s


def fd_rate(x, t, win=FD_WIN):
    """위치 기록을 win 스텝 간격으로 차분한 속도(참고값). 보고 속도와 비교용."""
    v = np.full_like(x, np.nan)
    v[win:] = (x[win:] - x[:-win]) / (t[win:] - t[:-win])
    return v


def fd_link_speed(c):
    """링크 14개 위치 차분 속도의 스텝별 최댓값(m/s, 참고값)."""
    names = sorted({k[:-4] for k in c if k.endswith("_x_m") and not k.startswith("base_")})
    sp = np.stack([np.sqrt(fd_rate(c[n + "_x_m"], c["t_s"]) ** 2 + fd_rate(c[n + "_y_m"], c["t_s"]) ** 2
                           + fd_rate(c[n + "_z_m"], c["t_s"]) ** 2) for n in names], axis=1)
    return np.nanmax(sp, axis=1) if len(names) else None


def finite_ok(summary):
    return summary["nonfinite_first_t"] is None and summary["steps_recorded"] == summary["expected_steps"]


def verdict(ok):
    return "성공" if ok else "실패"


def p1(s, c):
    t = c["t_s"]
    i1 = int(np.argmin(np.abs(t - 1.0)))
    after = t >= 1.0
    dz = float(np.max(np.abs(c["base_z_m"][after] - c["base_z_m"][i1])))
    droll = float(np.degrees(np.max(np.abs(c["roll_rad"][after] - c["roll_rad"][i1]))))
    dpitch = float(np.degrees(np.max(np.abs(c["pitch_rad"][after] - c["pitch_rad"][i1]))))
    vmax = float(np.max(c["max_link_speed_mps"][after]))
    checks = {"유한값·스텝 수": finite_ok(s), "base 높이 변화 ≤ 3 mm": dz <= 0.003,
              "roll·pitch 변화 ≤ 1°": max(droll, dpitch) <= 1.0, "모든 링크 속도 ≤ 0.01 m/s": vmax <= 0.01}
    return {"measured": {"base_z_at_1s_m": float(c["base_z_m"][i1]), "base_z_change_max_m": dz,
                         "roll_change_max_deg": droll, "pitch_change_max_deg": dpitch,
                         "max_link_speed_after_1s_mps": vmax,
                         "max_link_speed_before_1s_mps": float(np.max(c["max_link_speed_mps"][~after])),
                         "참고_위치차분_링크속도_최대_1s이후_mps": (float(np.nanmax(fd_link_speed(c)[after]))
                                                          if fd_link_speed(c) is not None else None)},
            "checks": checks, "verdict": verdict(all(checks.values()))}


def p2(s, c):
    out, ok = {}, finite_ok(s)
    for start, end, tgt, label in s["schedule"]:
        if not tgt:
            continue
        (name, target), = tgt.items()
        rec = s["seg_end"][label]
        q = rec["q"][name]
        tol = 0.001 if name.startswith("gripper") else 0.01
        fk = s["urdf_fk"][label]
        err_fk = max(v["err_mm"] for v in fk.values())
        sign_ok = math.copysign(1, q) == math.copysign(1, target)
        item = {"target": target, "q_end": q, "abs_err": abs(q - target), "tol": tol,
                "fk_err_mm": {k: v["err_mm"] for k, v in fk.items()}, "sign_same": sign_ok,
                "checks": {"|q−목표| ≤ 기준": abs(q - target) <= tol, "FK 오차 ≤ 2 mm": err_fk <= 2.0,
                           "부호 같음": sign_ok}}
        item["verdict"] = verdict(all(item["checks"].values()))
        ok = ok and all(item["checks"].values())
        out[label] = item
    return {"measured": out, "checks": {"유한값·스텝 수": finite_ok(s)}, "verdict": verdict(ok)}


def p3(s, c):
    gl, gr = c["q_gripper_left_joint"], c["q_gripper_right_joint"]
    diff = np.abs(gr + gl)
    checks = {"유한값·스텝 수": finite_ok(s), "끝에서 |오른쪽 + 왼쪽| ≤ 1 mm": float(diff[-1]) <= 0.001}
    return {"measured": {"left_end_m": float(gl[-1]), "right_end_m": float(gr[-1]), "sum_end_m": float(gr[-1] + gl[-1]),
                         "sum_abs_max_m": float(np.max(diff)), "sum_abs_max_after_cmd_m": float(np.max(diff[c["t_s"] >= SETTLE]))},
            "checks": checks, "verdict": verdict(all(checks.values()))}


ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
POSE = [0.0, 1.5, 1.5, 1.7, 0.0]


def p4(s, c, on):
    pairs = s["contacts"]
    hit = {k: v for k, v in pairs.items() if k in ("arm_link_3|base_link", "arm_link_4|base_link")}
    touch = {k: v for k, v in hit.items() if v.get("steps_sep_le_0", 0) > 0}
    err = {n: abs(float(c[f"q_{n}"][-1]) - p) for n, p in zip(ARM, POSE)}
    m = {"self_collision_newton_now": s["readback"]["self_collision_newton_now"], "base_arm34_contacts": hit,
         "참고_separation≤0_인_base_arm34_접촉": touch,
         "all_contact_pairs": pairs, "reach_err_end_rad": err, "reach_err_max_rad": max(err.values())}
    if on:
        checks = {"유한값·스텝 수": finite_ok(s), "base_link–arm_link_3/4 접촉 보고": bool(hit)}
    else:
        checks = {"유한값·스텝 수": finite_ok(s), "base_link–arm_link_3/4 접촉 없음": not hit,
                  "도달 오차 ≤ 0.05 rad": max(err.values()) <= 0.05}
    return {"measured": m, "checks": checks, "verdict": verdict(all(checks.values()))}


def p5(s, c):
    w = [k for k in c if k.startswith("dq_Wheel_")]
    vmax = {k[3:]: float(np.max(np.abs(c[k]))) for k in w}
    vfd = {k[3:]: float(np.nanmax(np.abs(fd_rate(c["q_" + k[3:]], c["t_s"])))) for k in w}
    checks = {"유한값·스텝 수": finite_ok(s), "모든 스텝 |ω| ≤ 2.02 rad/s": max(vmax.values()) <= 2.0 * 1.01}
    return {"measured": {"wheel_abs_max_rad_s": vmax, "참고_위치차분_바퀴속도_최대_rad_s": vfd,
                         "physx_max_velocities_readback": {k: v for k, v in s["readback"]["max_velocities"].items()
                                                           if k.startswith("Wheel_")}},
            "checks": checks, "verdict": verdict(all(checks.values()))}


def at(c, t):
    return int(np.argmin(np.abs(c["t_s"] - t)))


def p6_p8(s, c):
    i0 = at(c, SETTLE)
    x0, y0, yaw0 = c["base_x_m"][i0], c["base_y_m"][i0], c["yaw_rad"][i0]
    h = np.array([math.cos(yaw0), math.sin(yaw0)])
    n = np.array([-h[1], h[0]])
    rel = np.stack([c["base_x_m"] - x0, c["base_y_m"] - y0], axis=1)
    along, side = rel @ h, rel @ n
    i_fwd_end, i_back_end = at(c, SETTLE + 6.0), at(c, SETTLE + 12.0)
    fwd = float(along[i_fwd_end] - along[i0])
    back = float(along[i_fwd_end] - along[i_back_end])
    run = c["t_s"] >= SETTLE
    side_max = float(np.max(np.abs(side[run])))
    yaw_max = float(np.degrees(np.max(np.abs(np.unwrap(c["yaw_rad"][run]) - yaw0))))
    speed_rep = np.hypot(c["base_vx"], c["base_vy"])
    speed_fd = np.hypot(fd_rate(c["base_x_m"], c["t_s"]), fd_rate(c["base_y_m"], c["t_s"]))
    speed = speed_rep

    def stop_time(t_cmd, t_end, speed):
        idx = np.where((c["t_s"] > t_cmd) & (c["t_s"] <= t_end))[0]
        above = idx[speed[idx] > 0.005]
        if len(above) == 0:
            return 0.0
        last = above[-1]
        if last == idx[-1]:
            return None  # 끝까지 0.005 m/s 아래로 내려가지 않음
        return float(c["t_s"][last + 1] - t_cmd)

    st1, st2 = stop_time(SETTLE + 4.0, SETTLE + 6.0, speed_rep), stop_time(SETTLE + 10.0, SETTLE + 12.0, speed_rep)
    st2_fd = stop_time(SETTLE + 10.0, SETTLE + 12.0, speed_fd)
    p6c = {"유한값·스텝 수": finite_ok(s), "전진 0.18~0.22 m": 0.18 <= fwd <= 0.22, "후진 0.18~0.22 m": 0.18 <= back <= 0.22,
           "옆 이탈 ≤ 10 mm": side_max <= 0.010, "yaw 변화 ≤ 2°": yaw_max <= 2.0}
    p8c = {"정지 명령 후 0.5 s 안에 ≤ 0.005 m/s(P8, 후진 끝)": st2 is not None and st2 <= 0.5}
    return {"P6": {"measured": {"forward_m": fwd, "backward_m": back, "side_max_m": side_max, "yaw_change_max_deg": yaw_max,
                                "definition": "전진 = 명령 시작(1 s)~첫 정지 구간 끝(7 s)의 처음 방향 성분, 후진 = 7 s~13 s"},
                   "checks": p6c, "verdict": verdict(all(p6c.values()))},
            "P8": {"measured": {"stop_time_after_backward_s": st2, "stop_time_after_forward_s(참고)": st1,
                                "speed_at_end_mps": float(speed[-1]),
                                "참고_위치차분_정지시간_s": st2_fd, "참고_위치차분_끝속도_mps": float(speed_fd[-1])},
                   "checks": p8c, "verdict": verdict(all(p8c.values()))}}


def p7(s, c):
    i0 = at(c, SETTLE)
    run = c["t_s"] >= SETTLE
    yaw = np.unwrap(c["yaw_rad"][run])
    d = np.hypot(c["base_x_m"][run] - c["base_x_m"][i0], c["base_y_m"][run] - c["base_y_m"][i0])
    dyaw = float(yaw[-1] - yaw[0])
    checks = {"유한값·스텝 수": finite_ok(s), "yaw가 + 방향(반시계)으로 증가": dyaw > 0, "중심 이동 ≤ 30 mm": float(np.max(d)) <= 0.030}
    return {"measured": {"yaw_change_deg(기록만)": math.degrees(dyaw), "center_move_max_m": float(np.max(d)),
                         "ideal_no_slip_deg(참고)": None},
            "checks": checks, "verdict": verdict(all(checks.values()))}


def main(folder):
    folder = Path(folder)
    s, c = load(folder)
    test = s["test"]
    res = {"P1": p1, "P2": p2, "P3": p3, "P5": p5, "P7": p7}.get(test)
    if res:
        result = res(s, c)
    elif test == "P4on":
        result = p4(s, c, True)
    elif test == "P4off":
        result = p4(s, c, False)
    elif test == "P6":
        result = p6_p8(s, c)
    else:
        raise ValueError(test)
    (folder / "verdict.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main(sys.argv[1])
