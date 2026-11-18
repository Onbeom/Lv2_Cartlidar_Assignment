"""run_phys_test.py 결과 폴더(log.csv, summary.json)를 검증계획 v3 기준으로 판정한다.

v3(사용자 승인 2026-10-22): 속도는 위치 차분(0.1 s 창)으로 판정하고 엔진 보고 속도는 참고로 남긴다.
접촉은 separation ≤ 0인 것만 센다. P7은 yaw 변화 ≥ 1°를 더한다. D1·D2 진단 판정을 더한다.
v4(2026-10-22): P 시험은 제어기 속도 한계로 실행된다. P2·P4에 팔·집게 실제 속도(위치 차분) 최대를 참고로 기록한다.
v5(2026-10-22): 속도 한계를 ROBOTIS 공식 무부하 속도(바퀴 7.3304, 팔 4.5029 rad/s)로. P5 기준 7.3304 × 1.01.
P4 켬 = base_link와 arm_link_2~5·집게 좌우 중 어느 쌍이든 실제 접촉(separation ≤ 0), 끔 = 그 쌍들의 실제 접촉 없음 + 도달 오차 ≤ 0.05 rad.
v6(2026-10-22): P2·P4에 팔 관절 실제 속도(위치 차분) ≤ 4.5029 × 1.1 = 4.9532 rad/s 기준을 더한다.

판정 표기: 성공 / 실패 / 조건부 / 미실시 / 데이터 부족.
실행(PC, 일반 python3 가능): python3 261022_1243/phys_tests/analyze_phys_test.py <결과 폴더>
결과는 같은 폴더의 verdict.json에 쓴다.
"""
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mujoco" / "stow"))
import maneuver  # noqa: E402  (바퀴 정의 파일 값)

import numpy as np

SETTLE = 1.0


def load(folder):
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    with (folder / "log.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k not in ("segment", "contact_pairs_minsep_mm")}
    # 연속 회전 바퀴의 위치는 엔진이 ±2π 범위로 감아(4π 주기) 돌려준다. 차분 전에 풀어 둔다(P5에서 확인).
    for k in cols:
        if k.startswith("q_Wheel_"):
            cols[k] = np.unwrap(cols[k], period=4 * np.pi)
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
    vmax_fd = float(np.nanmax(fd_link_speed(c)[after]))
    checks = {"유한값·스텝 수": finite_ok(s), "base 높이 변화 ≤ 3 mm": dz <= 0.003,
              "roll·pitch 변화 ≤ 1°": max(droll, dpitch) <= 1.0, "모든 링크 속도(위치 차분) ≤ 0.01 m/s": vmax_fd <= 0.01}
    return {"measured": {"base_z_at_1s_m": float(c["base_z_m"][i1]), "base_z_change_max_m": dz,
                         "roll_change_max_deg": droll, "pitch_change_max_deg": dpitch,
                         "링크속도_위치차분_최대_1s이후_mps": vmax_fd,
                         "참고_보고속도_링크최대_1s이후_mps": float(np.max(c["max_link_speed_mps"][after])),
                         "max_link_speed_before_1s_mps": float(np.max(c["max_link_speed_mps"][~after])),
                         },
            "checks": checks, "verdict": verdict(all(checks.values()))}


def joint_fd_max(c):
    """팔·집게 관절 실제 속도(위치 차분) 절댓값 최대(참고)."""
    return {k[2:]: float(np.nanmax(np.abs(fd_rate(c[k], c["t_s"])))) for k in c
            if k.startswith("q_RMX52_") or k.startswith("q_gripper_")}


ARM_SPEED_MAX = 43 * 2 * math.pi / 60 * 1.1  # XM430-W350 무부하 속도(11.1 V) × 1.1


def arm_speed_ok(c):
    v = {k: x for k, x in joint_fd_max(c).items() if k.startswith("RMX52_")}
    return max(v.values()) <= ARM_SPEED_MAX, max(v.values())


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
    sp_ok, sp = arm_speed_ok(c)
    ok = ok and sp_ok
    return {"measured": out, "관절속도_위치차분_최대": joint_fd_max(c),
            "checks": {"유한값·스텝 수": finite_ok(s), f"팔 실제 속도 ≤ {ARM_SPEED_MAX:.4f} rad/s (최대 {sp:.3f})": sp_ok},
            "verdict": verdict(ok)}


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
    arm_links = ("arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link", "gripper_right_link")
    hit = {k: v for k, v in pairs.items() if k in {"|".join(sorted(["base_link", a])) for a in arm_links}}
    touch = {k: v for k, v in hit.items() if v.get("steps_sep_le_0", 0) > 0}
    err = {n: abs(float(c[f"q_{n}"][-1]) - p) for n, p in zip(ARM, POSE)}
    m = {"self_collision_newton_now": s["readback"]["self_collision_newton_now"], "base_arm_contacts_보고": hit,
         "base_arm_실제접촉_separation≤0": touch,
         "all_contact_pairs": pairs, "reach_err_end_rad": err, "reach_err_max_rad": max(err.values()),
         "참고_관절속도_위치차분_최대": joint_fd_max(c)}
    if on:
        checks = {"유한값·스텝 수": finite_ok(s), "base_link–팔·집게 링크 실제 접촉(separation ≤ 0)": bool(touch)}
    else:
        checks = {"유한값·스텝 수": finite_ok(s), "base_link–팔·집게 링크 실제 접촉(separation ≤ 0) 없음": not touch,
                  "도달 오차 ≤ 0.05 rad": max(err.values()) <= 0.05}
    sp_ok, sp = arm_speed_ok(c)
    checks[f"팔 실제 속도 ≤ {ARM_SPEED_MAX:.4f} rad/s (최대 {sp:.3f})"] = sp_ok
    m["침투(참고, D5)"] = penetration(c, arm_links)
    return {"measured": m, "checks": checks, "verdict": verdict(all(checks.values()))}


def penetration(c, arm_links):
    """base_link–팔·집게 쌍의 침투 깊이(mm, = −separation). 전체 최대·마지막 2 s 최대·침투 스텝 수와 팔 drive 힘 최대."""
    t = c["t_s"]
    keys = {"|".join(sorted(["base_link", a])) for a in arm_links}
    out = {}
    for i, txt in enumerate(c["contact_pairs"]):
        for item in filter(None, txt.split(";")):
            k, v = item.rsplit(":", 1)
            if k in keys and float(v) < 0:
                d = out.setdefault(k, {"max_mm": 0.0, "last2s_max_mm": 0.0, "steps": 0})
                d["max_mm"] = max(d["max_mm"], -float(v))
                d["steps"] += 1
                if t[i] >= t[-1] - 2.0:
                    d["last2s_max_mm"] = max(d["last2s_max_mm"], -float(v))
    force = {k[6:]: float(np.max(np.abs(c[k]))) for k in c if k.startswith("force_RMX52_")}
    return {"pairs": out, "arm_force_abs_max_Nm": force}


def p5(s, c):
    w = [k for k in c if k.startswith("dq_Wheel_")]
    vmax = {k[3:]: float(np.max(np.abs(c[k]))) for k in w}
    vfd = {k[3:]: float(np.nanmax(np.abs(fd_rate(c["q_" + k[3:]], c["t_s"])))) for k in w}
    lim = maneuver.WHEEL_RATE  # XM430-W210 무부하 속도(11.1 V) × 증속 1.5, 바퀴 정의 파일 값
    checks = {"유한값·스텝 수": finite_ok(s), f"모든 스텝 |ω|(위치 차분) ≤ {lim * 1.01:.4f} rad/s": max(vfd.values()) <= lim * 1.01}
    return {"measured": {"바퀴속도_위치차분_최대_rad_s": vfd, "참고_보고속도_최대_rad_s": vmax,
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
    p8c = {"정지 명령 후 0.5 s 안에 ≤ 0.005 m/s(위치 차분, P8, 후진 끝)": st2_fd is not None and st2_fd <= 0.5}
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
    checks = {"유한값·스텝 수": finite_ok(s), "yaw가 + 방향(반시계)으로 1° 이상 증가": math.degrees(dyaw) >= 1.0,
              "중심 이동 ≤ 30 mm": float(np.max(d)) <= 0.030}
    return {"measured": {"yaw_change_deg(기록만)": math.degrees(dyaw), "center_move_max_m": float(np.max(d)),
                         "ideal_no_slip_deg(참고)": None},
            "checks": checks, "verdict": verdict(all(checks.values()))}


def d1(s, c):
    out, ok = {}, finite_ok(s)
    for start, end, tgt, label in s["schedule"]:
        i = at(c, end - 1e-6)
        target = tgt.get("RMX52_Elbow", 0.0)
        q = float(c["q_RMX52_Elbow"][i])
        good = abs(q - target) <= 0.01 and (target == 0.0 or math.copysign(1, q) == math.copysign(1, target))
        out[label] = {"target_rad": target, "q_end_rad": q, "ok": good}
        ok = ok and good
    others = {n: float(np.max(np.abs(c[f"q_{n}"]))) for n in ARM if n != "RMX52_Elbow"}
    return {"measured": {"segments": out, "다른 팔 관절 |q| 최대": others,
                         "max_velocities": s["readback"].get("max_velocities_after_override", s["readback"]["max_velocities"])},
            "checks": {"구간 끝마다 부호 같고 오차 ≤ 0.01 rad": ok}, "verdict": verdict(ok)}


def d3(s, c):
    r = d1(s, c)
    t = c["t_s"]
    win = (t > 0.5) & (t < 1.0)  # 명령 전 정지 구간
    fd = fd_rate(c["q_RMX52_Elbow"], t)
    gap = float(np.nanmean(np.abs(c["dq_RMX52_Elbow"][win] - fd[win])))
    ok2 = gap <= 0.005
    r["measured"]["정지구간_팔꿈치_|보고−위치차분|_평균_rad_s"] = gap
    r["measured"]["solver"] = [s["readback"].get("solver_type"), s["readback"].get("solver_iterations_usd(None=기본값)")]
    r["measured"]["gravity"] = s["conditions"]["gravity"]
    r["checks"]["정지 구간 |보고 − 위치 차분| ≤ 0.005 rad/s"] = ok2
    r["verdict"] = verdict(all(r["checks"].values()))
    return r


def d2(s, c):
    r = p7(s, c)
    dyaw = r["measured"]["yaw_change_deg(기록만)"]
    w = {k[2:]: float((c[k][-1] - c[k][at(c, SETTLE)]) / (c["t_s"][-1] - SETTLE)) for k in c if k.startswith("q_Wheel_")}
    ok = finite_ok(s) and dyaw >= 1.0
    return {"measured": {"yaw_change_deg": dyaw, "바퀴_평균속도_위치차분_rad_s": w, "conditions": s["conditions"]},
            "checks": {"4 s 동안 yaw 변화 ≥ 1°": ok}, "verdict": verdict(ok)}


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
    elif test in ("D1A", "D1B"):
        result = d1(s, c)
    elif test in ("D3A", "D3B", "D3C", "D3D", "D3E"):
        result = d3(s, c)
    elif test in ("D2A", "D2B", "D2C"):
        result = d2(s, c)
    elif test == "P6":
        result = p6_p8(s, c)
    else:
        raise ValueError(test)
    (folder / "verdict.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main(sys.argv[1])
