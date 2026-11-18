"""팔 접은 자세 확인 시험 V1~V3(검증계획 17장) 판정. Isaac(run_phys_test.py)과 MuJoCo(mujoco_drive.py) 결과 폴더를 같은 기준으로 본다.

V1 접은 채 제자리 회전: 회전(2~6 s) ≥ 1°, R_full ≤ 307 mm, 팔 관절 이탈(2 s 이후) ≤ 0.02 rad, 팔–다른 형상 실제 접촉 없음
V2 접은 채 직진·후진·정지: 전진·후진 0.18~0.22 m, 옆 이탈 ≤ 10 mm, yaw ≤ 2°, 후진 정지 명령 뒤 0.5 s 안에 ≤ 0.005 m/s,
   팔 이탈(2 s 이후) ≤ 0.02 rad, 접촉 없음
V1b 회전 중심 오프셋 보정 회전(18장): 이동 거리 d ± 2 mm·이동 중 yaw 변화 ≤ 0.5°(시험 성립), 회전 ≥ 1°,
   R_cell = |c_w − O| + R_pivot ≤ 307 mm(O = 2 s의 차체 원점 = 칸 가운데, c_w = 회전 중심의 월드 좌표), 팔 이탈·접촉은 V1과 같음
V8(23장)은 속도만 다른 V4라 V4 기준으로 판정
V4 모퉁이 호 회전(19장): 미로 조각과 외곽 최소 간격 ≥ 30 mm(시작부터 끝까지), 호 구간이 90°에 도달해 끝남, 팔 이탈·접촉은 V1과 같음
V5 막다른 길 후진(19장): 후진 거리 칸 길이 ± 10 %, 옆 이탈 ≤ 10 mm, yaw 변화 ≤ 2°, 정지 명령 뒤 0.5 s 안에 ≤ 0.005 m/s,
   최소 간격 ≥ 30 mm, 팔 이탈·접촉은 V1과 같음
V6 출발 칸 회전 + 정상화(20장): 최소 간격 ≥ 30 mm(회전·정상화 전 구간), 정지했을 때 옆 오차 ≤ 10 mm·헤딩 오차 ≤ 1°·
   다음 칸 가운데에서 앞뒤 ≤ 20 mm, 정지 명령 뒤 0.5 s 안에 ≤ 0.005 m/s, 팔 이탈·접촉은 V1과 같음
V7 위치 오차가 있을 때의 정상화(21장): V6과 같은 기준을 실제 자세로 판정
V6c·V7c(V6·V7 + 정지 후 헤딩 보정, 22장)는 V6 기준, V6cb(V6b + 보정)는 V6b 기준. 보정 뒤 마지막 정지 자세로 판정
V6b 호 회전 + 정상화(20장): V4 간격 기준 + V6 정상화 기준(나가는 통로 중심선·나가는 칸 가운데 기준)
   (V4·V5 간격: 시작 자세(t = 0)를 칸 가운데로 두고 maze_clearance.py의 미로 조각과 로봇 평면 볼록 껍질 사이 거리)
V3 0 → 접은 자세 → 0: 접촉 없음, 팔 실제 속도(위치 차분 0.1 s) ≤ 4.9532 rad/s, 도달 오차(4 s·7 s) ≤ 0.05 rad

R_full = |회전 중심 c| + max|외곽점 − c|. c는 회전 구간 궤적을 차체 좌표 고정점 둘레 강체 회전으로 맞춘 값,
외곽점은 MuJoCo 충돌 형상(로봇 전체, 기록된 팔 관절각)의 평면 투영이다.
실행(PC): MuJoCo venv python으로 python stow/analyze_stow.py <결과 폴더> ... → 폴더마다 verdict_stow.json
"""
import csv
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import maze_clearance as mz  # noqa: E402

# 261027_1530 재검사: 외곽 계산용 모델 = 도구 확인 때 mujoco_drive.py가 저장한 model.xml(형상은 URDF에서만 옴)
MODEL = Path(__file__).resolve().parents[1] / "drive/results/ref_model/model.xml"
ARM = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]
STOW = np.array([0.0, 1.4, -1.0, -1.3, 0.0])
ARM_BODIES = ("arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link", "gripper_right_link")
R_LIMIT, DEV_LIMIT, ARM_SPEED_LIMIT = 0.307, 0.02, 4.5029 * 1.1

_m = mujoco.MjModel.from_xml_path(str(MODEL))
_d = mujoco.MjData(_m)
_adr = {n: _m.jnt_qposadr[_m.joint(n).id] for n in ARM}


def footprint_xy(q):
    """차체 좌표(원점 = base_footprint)에서 로봇 전체 충돌 형상 점의 평면 좌표."""
    _d.qpos[:] = 0
    _d.qpos[3] = 1
    for n, v in zip(ARM, q):
        _d.qpos[_adr[n]] = v
    mujoco.mj_kinematics(_m, _d)
    pts = []
    for g in range(_m.ngeom):
        if _m.geom_bodyid[g] == 0:
            continue
        R, c, s, t = _d.geom_xmat[g].reshape(3, 3), _d.geom_xpos[g], _m.geom_size[g], _m.geom_type[g]
        if t == mujoco.mjtGeom.mjGEOM_MESH:
            mid = _m.geom_dataid[g]
            a, k = _m.mesh_vertadr[mid], _m.mesh_vertnum[mid]
            pts.append(_m.mesh_vert[a:a + k] @ R.T + c)
        elif t == mujoco.mjtGeom.mjGEOM_BOX:
            pts.append(np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * s @ R.T + c)
        else:
            ang = np.linspace(0, 2 * np.pi, 36)
            ring = np.stack([np.cos(ang) * s[0], np.sin(ang) * s[0], np.zeros_like(ang)], 1)
            pts.append(np.vstack([ring + [0, 0, s[1]], ring - [0, 0, s[1]]]) @ R.T + c)
    return np.vstack(pts)[:, :2]


def load(folder):
    with (folder / "log.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    c = {k: np.array([float(r[k]) if r[k] not in ("", "True", "False") else (float(r[k] == "True") if r[k] else np.nan)
                      for r in rows]) for k in rows[0] if k not in ("segment", "contact_pairs_minsep_mm")}
    if "contact_pairs_minsep_mm" in rows[0]:  # Isaac: 쌍별 최소 separation(mm)
        arm_min = []
        for r in rows:
            vals = []
            for item in filter(None, r["contact_pairs_minsep_mm"].split(";")):
                pair, v = item.rsplit(":", 1)
                a, b = pair.split("|")
                if (a in ARM_BODIES) != (b in ARM_BODIES):
                    vals.append(float(v) / 1000)
            arm_min.append(min(vals) if vals else np.nan)
        c["arm_contact_min_dist_m"] = np.array(arm_min)
    return c


def at(c, t):
    return int(np.argmin(np.abs(c["t_s"] - t)))


def fd(x, t, win):
    v = np.full_like(x, np.nan)
    v[win:] = (x[win:] - x[:-win]) / (t[win:] - t[:-win])
    return v


def contact_check(c):
    real = c["arm_contact_min_dist_m"]
    n = int(np.sum(real <= 0))
    return n == 0, {"arm_real_contact_steps": n, "arm_contact_min_dist_mm": None if np.all(np.isnan(real)) else float(np.nanmin(real) * 1000)}


def dev_check(c, t0):
    q = np.stack([c["q_" + n] for n in ARM], axis=1)
    sel = c["t_s"] >= t0
    dev = np.max(np.abs(q[sel] - STOW), axis=0)
    return float(dev.max()) <= DEV_LIMIT, {"arm_dev_max_rad": dict(zip(ARM, [round(float(v), 5) for v in dev]))}


def rot(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def pivot_fit(c, t_spin):
    """회전 구간(t ≥ t_spin) 궤적을 차체 좌표 고정점 둘레 강체 회전으로 맞춘다. 반환: 차체 좌표 회전 중심, 맞춤 RMS, R_pivot, 회전량(°)."""
    t = c["t_s"]
    i0 = at(c, t_spin)
    yaw = np.unwrap(c["yaw_rad"])
    idx = np.where(t >= t_spin)[0][::5]
    A = np.vstack([rot(yaw[j]) - rot(yaw[i0]) for j in idx])
    b = np.concatenate([-np.array([c["base_x_m"][j] - c["base_x_m"][i0], c["base_y_m"][j] - c["base_y_m"][i0]]) for j in idx])
    cen = np.linalg.lstsq(A, b, rcond=None)[0]
    rms = float(np.sqrt(np.mean((A @ cen - b) ** 2)))
    r_piv = 0.0
    for j in idx[::4]:
        xy = footprint_xy([c["q_" + n][j] for n in ARM])
        r_piv = max(r_piv, float(np.max(np.linalg.norm(xy - cen, axis=1))))
    return cen, rms, r_piv, math.degrees(yaw[-1] - yaw[i0])


def v1(c):
    t = c["t_s"]
    i0 = at(c, 2.0)
    cen, rms, r_piv, dyaw = pivot_fit(c, 2.0)
    r_full = float(np.linalg.norm(cen)) + r_piv
    ok_c, mc = contact_check(c)
    ok_d, md = dev_check(c, 2.0)
    checks = {"회전 ≥ 1°": dyaw >= 1.0, "R_full ≤ 307 mm": r_full <= R_LIMIT, "팔 이탈 ≤ 0.02 rad (2 s 이후)": ok_d,
              "팔–다른 형상 실제 접촉 없음": ok_c}
    return {"measured": {"yaw_deg": dyaw, "pivot_mm": [float(v * 1000) for v in cen], "pivot_fit_rms_mm": rms * 1000,
                         "R_pivot_mm": r_piv * 1000, "R_full_mm": r_full * 1000,
                         "center_move_max_mm": float(np.max(np.hypot(c["base_x_m"][t >= 2] - c["base_x_m"][i0],
                                                                       c["base_y_m"][t >= 2] - c["base_y_m"][i0])) * 1000),
                         **mc, **md}, "checks": checks}


def v1b(c, s):
    t = c["t_s"]
    d = float(s["conditions"]["pivot_offset_m"])
    t_spin = next(seg[0] for seg in s["schedule"] if seg[3].startswith("spin"))
    i0, i1 = at(c, 2.0), at(c, t_spin)
    o = np.array([c["base_x_m"][i0], c["base_y_m"][i0]])  # 칸 가운데 O
    yaw = np.unwrap(c["yaw_rad"])
    h = np.array([math.cos(yaw[i0]), math.sin(yaw[i0])])
    p1 = np.array([c["base_x_m"][i1], c["base_y_m"][i1]])
    move = float((p1 - o) @ h)
    side = float((p1 - o) @ np.array([-h[1], h[0]]))
    yaw_move = math.degrees(float(np.max(np.abs(yaw[i0:i1 + 1] - yaw[i0]))))
    cen, rms, r_piv, dyaw = pivot_fit(c, t_spin)
    c_w = p1 + rot(yaw[i1]) @ cen
    e = float(np.linalg.norm(c_w - o))
    r_cell = e + r_piv
    sel = t >= t_spin
    ok_c, mc = contact_check(c)
    ok_d, md = dev_check(c, 2.0)
    checks = {"이동 거리 d ± 2 mm": abs(move - d) <= 0.002, "이동 중 yaw 변화 ≤ 0.5°": yaw_move <= 0.5,
              "회전 ≥ 1°": dyaw >= 1.0, "R_cell ≤ 307 mm": r_cell <= R_LIMIT, "팔 이탈 ≤ 0.02 rad (2 s 이후)": ok_d,
              "팔–다른 형상 실제 접촉 없음": ok_c}
    return {"measured": {"pivot_offset_mm": d * 1000, "move_mm": move * 1000, "move_side_mm": side * 1000,
                         "move_yaw_max_deg": yaw_move, "t_spin_s": t_spin, "yaw_deg": dyaw,
                         "pivot_mm": [float(v * 1000) for v in cen], "pivot_fit_rms_mm": rms * 1000,
                         "pivot_world_minus_O_mm": [float(v * 1000) for v in c_w - o], "e_mm": e * 1000,
                         "R_pivot_mm": r_piv * 1000, "R_cell_mm": r_cell * 1000,
                         "center_move_max_mm": float(np.max(np.hypot(c["base_x_m"][sel] - o[0], c["base_y_m"][sel] - o[1])) * 1000),
                         **mc, **md}, "checks": checks}


def _start_frame(c):
    """시작 자세(t = 0) 기준 좌표로 바꾼 차체 x, y, yaw(이어 붙인 값)."""
    x0, y0 = c["base_x_m"][0], c["base_y_m"][0]
    yaw = np.unwrap(c["yaw_rad"])
    a = -yaw[0]
    dx, dy = c["base_x_m"] - x0, c["base_y_m"] - y0
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a), yaw - yaw[0]


def _robot_hull(c):
    t = c["t_s"]
    q = [float(np.median(c["q_" + n][t >= 2.0])) for n in ARM]
    return mz.hull(footprint_xy(q))


def _clear(c, x, y, yaw, obs, hull_pts, every=2):
    idx = np.arange(0, len(x), every)
    d = np.array([mz.clearance(hull_pts, x[i], y[i], yaw[i], obs) for i in idx])
    k = int(np.argmin(d))
    return float(d[k]), int(idx[k])


def _phase(s, name):
    return next(p for p in s["maneuver"]["phase_log"] if p[0] == name)


def v4(c, s):
    t = c["t_s"]
    mp = s["maneuver"]["params"]
    side = 1 if mp["side"] == "L" else -1
    R = mp["radius_m"]
    obs, info = mz.piece("corner", mp["corner"], side)
    x, y, yaw = _start_frame(c)
    hp = _robot_hull(c)
    dmin, k = _clear(c, x, y, yaw, obs, hp)
    arc = _phase(s, "arc")
    arc_done = arc[2] is not None
    sel = (t >= arc[1]) & (t <= (arc[2] if arc_done else t[-1]))
    xa, ya = x[sel], y[sel]
    A = np.column_stack([2 * xa, 2 * ya, np.ones_like(xa)])  # 원 맞춤(Kasa): x² + y² = 2ax + 2by + c
    sol = np.linalg.lstsq(A, xa ** 2 + ya ** 2, rcond=None)[0]
    r_eff = float(math.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2))
    cx, ey = info["corner_center"][0], info["exit_center"][1]
    i_arc_end = int(np.argmin(np.abs(t - arc[2]))) if arc_done else len(t) - 1
    after = t >= (arc[2] if arc_done else t[-1])
    over = float(np.degrees(np.max(side * yaw[after]) - math.pi / 2)) if np.any(after) else None
    ok_c, mc_ = contact_check(c)
    ok_d, md = dev_check(c, 2.0)
    checks = {"최소 간격 ≥ 30 mm": dmin >= 0.030, "호 회전 90° 도달": arc_done,
              "팔 이탈 ≤ 0.02 rad (2 s 이후)": ok_d, "팔–다른 형상 실제 접촉 없음": ok_c}
    return {"measured": {"corner": mp["corner"], "side": mp["side"], "radius_cmd_m": R,
                         "min_clearance_mm": dmin * 1000, "min_clearance_t_s": float(t[k]),
                         "min_clearance_phase": next((ph[0] for ph in s["maneuver"]["phase_log"]
                                                     if ph[1] <= t[k] and (ph[2] is None or t[k] < ph[2])), "hold"),
                         "min_clearance_pose": [float(x[k]), float(y[k]), math.degrees(float(yaw[k]))],
                         "arc_time_s": (arc[2] - arc[1]) if arc_done else None, "arc_time_nominal_s": (math.pi / 2) * R / mp["v_mps"],
                         "arc_radius_fit_m": r_eff,
                         "arc_end_outward_mm": float(x[i_arc_end] - cx) * 1000, "arc_end_side_mm": float(side * y[i_arc_end]) * 1000,
                         "yaw_overshoot_deg": over,
                         "end_outward_mm": float(x[-1] - cx) * 1000, "end_along_mm": float(side * y[-1] - abs(ey)) * 1000,
                         "end_heading_err_deg": math.degrees(float(side * yaw[-1] - math.pi / 2)),
                         **mc_, **md}, "checks": checks}


def v5(c, s):
    t = c["t_s"]
    mp = s["maneuver"]["params"]
    obs, info = mz.piece("deadend", mp["deadend"])
    L = info["cell_len"]
    x, y, yaw = _start_frame(c)
    hp = _robot_hull(c)
    dmin, k = _clear(c, x, y, yaw, obs, hp)
    b = _phase(s, "backward")
    st = _phase(s, "stop")
    i0 = at(c, b[1])
    back = float(x[i0] - x[-1])
    run = t >= b[1]
    side = float(np.max(np.abs(y[run] - y[i0])))
    yaw_max = math.degrees(float(np.max(np.abs(yaw[run] - yaw[i0]))))
    win = max(1, int(round(0.1 / float(np.median(np.diff(t))))))
    speed = np.hypot(fd(c["base_x_m"], t, win), fd(c["base_y_m"], t, win))
    idx = np.where(t > st[1])[0]
    above = idx[speed[idx] > 0.005]
    stop = 0.0 if len(above) == 0 else (None if above[-1] == idx[-1] else float(t[above[-1] + 1] - st[1]))
    ok_c, mc_ = contact_check(c)
    ok_d, md = dev_check(c, 2.0)
    checks = {"후진 거리 칸 길이 ± 10 %": abs(back - L) <= 0.1 * L, "옆 이탈 ≤ 10 mm": side <= 0.010,
              "yaw 변화 ≤ 2°": yaw_max <= 2.0, "정지 명령 뒤 0.5 s 안에 ≤ 0.005 m/s": stop is not None and stop <= 0.5,
              "최소 간격 ≥ 30 mm": dmin >= 0.030, "팔 이탈 ≤ 0.02 rad (2 s 이후)": ok_d, "팔–다른 형상 실제 접촉 없음": ok_c}
    return {"measured": {"deadend": mp["deadend"], "cell_len_m": L, "backward_m": back, "side_max_mm": side * 1000,
                         "yaw_max_deg": yaw_max, "stop_time_s": stop, "min_clearance_mm": dmin * 1000,
                         "min_clearance_t_s": float(t[k]), "end_pose": [float(x[-1]), float(y[-1]), math.degrees(float(yaw[-1]))],
                         **mc_, **md}, "checks": checks}


def _stop_time(c, t_stop):
    t = c["t_s"]
    win = max(1, int(round(0.1 / float(np.median(np.diff(t))))))
    speed = np.hypot(fd(c["base_x_m"], t, win), fd(c["base_y_m"], t, win))
    idx = np.where(t > t_stop)[0]
    above = idx[speed[idx] > 0.005]
    return 0.0 if len(above) == 0 else (None if above[-1] == idx[-1] else float(t[above[-1] + 1] - t_stop))


def _w_cmd(c, s):
    """기록된 바퀴 명령에서 차체 회전 속도 명령(rad/s)."""
    mp = s["maneuver"]["params"]
    left = [k for k in c if k.startswith("cmd_Wheel") and k.endswith("_Left")]
    right = [k for k in c if k.startswith("cmd_Wheel") and k.endswith("_Right")]
    vl = np.mean([c[k] for k in left], axis=0)
    vr = np.mean([c[k] for k in right], axis=0)
    return (vr - vl) * mp["wheel_r_m"] / mp["track_m"]


def _normalize_metrics(c, s, along, lat, head, t_norm0, t_stop):
    """정상화 구간 기록 값: 정착(옆 ≤ 10 mm·헤딩 ≤ 1°가 끝까지 유지)한 진행 위치, ω 상한에 걸린 비율."""
    t = c["t_s"]
    sel = (t >= t_norm0) & (t < t_stop)
    ok = (np.abs(lat) <= 0.010) & (np.abs(head) <= math.radians(1.0))
    idx = np.where(sel)[0]
    settle = None
    for j in idx:
        if ok[j:idx[-1] + 1].all():
            settle = float(along[j])
            break
    w = _w_cmd(c, s)[sel]
    return {"settle_along_m": settle, "w_sat_fraction": float(np.mean(np.abs(w) >= s["maneuver"]["params"]["w_max"] - 1e-4)),
            "lat_at_normalize_start_mm": float(lat[idx[0]] * 1000), "head_at_normalize_start_deg": math.degrees(float(head[idx[0]]))}


def _norm_checks(c, dmin, lat_end, head_end, along_err, stop):
    ok_c, mc_ = contact_check(c)
    ok_d, md = dev_check(c, 2.0)
    checks = {"최소 간격 ≥ 30 mm": dmin >= 0.030, "옆 오차 ≤ 10 mm": abs(lat_end) <= 0.010,
              "헤딩 오차 ≤ 1°": abs(head_end) <= 1.0, "정지 위치 앞뒤 ≤ 20 mm": abs(along_err) <= 0.020,
              "정지 명령 뒤 0.5 s 안에 ≤ 0.005 m/s": stop is not None and stop <= 0.5,
              "팔 이탈 ≤ 0.02 rad (2 s 이후)": ok_d, "팔–다른 형상 실제 접촉 없음": ok_c}
    return {k: bool(v) for k, v in checks.items()}, {**mc_, **md}


def _align_metrics(c, s, lat, along):
    """정지 후 헤딩 보정(22장) 기록: 평균한 추정 오차, 돌린 각도·시간, 보정 중 원점 이동(옆·진행)."""
    ph = next((p for p in s["maneuver"]["phase_log"] if p[0] == "align"), None)
    if ph is None:
        return {}
    i0 = at(c, ph[1])
    yaw = np.unwrap(c["yaw_rad"])
    return {"align": s["maneuver"]["align"], "align_time_s": ph[2] - ph[1],
            "align_turned_deg": math.degrees(float(yaw[-1] - yaw[i0])),
            "align_lat_shift_mm": float((lat[-1] - lat[i0]) * 1000), "align_along_shift_mm": float((along[-1] - along[i0]) * 1000)}


def v6(c, s):
    t = c["t_s"]
    mp = s["maneuver"]["params"]
    th0 = math.radians(mp["start_heading_deg"])
    xs, ys, yaws = _start_frame(c)
    x = xs * math.cos(th0) - ys * math.sin(th0)   # 칸 프레임(열린 쪽 +x, 칸 가운데 원점)
    y = xs * math.sin(th0) + ys * math.cos(th0)
    yaw = yaws + th0
    head = (yaw + math.pi) % (2 * math.pi) - math.pi
    obs, info = mz.piece("deadend", "A")
    obs = [o * [-1, 1] for o in obs]                # 막힌 끝벽을 −x 쪽으로
    dmin, k = _clear(c, x, y, yaw, obs, _robot_hull(c))
    spin, norm, stp = _phase(s, "spin"), _phase(s, "normalize"), _phase(s, "stop")
    i_spin_end = at(c, spin[2])
    turn = math.degrees(float(yaws[i_spin_end]))
    goal = mz.piece("deadend", "A")[1]["cell_len"]
    stop = _stop_time(c, stp[1])
    checks, extra = _norm_checks(c, dmin, y[-1], math.degrees(float(head[-1])), x[-1] - goal, stop)
    after = t >= spin[2]
    spin_target = {90: -90.0, -90: 90.0, 180: 180.0}[mp["start_heading_deg"]]
    extra_v7 = {}
    if s["test"] in ("V7", "V7c"):  # 정상화 중 흔들림(실제 옆 오차·헤딩·ω 명령의 RMS)
        sel = (t >= norm[1]) & (t < stp[1])
        w = _w_cmd(c, s)[sel]
        extra_v7 = {"loc_bias": mp["loc_bias"], "loc_noise": mp["loc_noise"], "seed": mp["seed"],
                    "norm_lat_rms_mm": float(np.sqrt(np.mean(y[sel] ** 2)) * 1000),
                    "norm_head_rms_deg": math.degrees(float(np.sqrt(np.mean(head[sel] ** 2)))),
                    "norm_w_cmd_rms": float(np.sqrt(np.mean((w - np.mean(w)) ** 2)))}
    extra_v7.update(_align_metrics(c, s, y, x))
    return {"measured": {"start_heading_deg": mp["start_heading_deg"], **extra_v7, "min_clearance_mm": dmin * 1000,
                         "min_clearance_t_s": float(t[k]), "min_clearance_phase": next(
                             (ph[0] for ph in s["maneuver"]["phase_log"] if ph[1] <= t[k] and (ph[2] is None or t[k] < ph[2])), "hold"),
                         "spin_time_s": spin[2] - spin[1], "spin_deg_at_end": turn,
                         "spin_overshoot_deg": float(np.max(np.sign(spin_target) * np.degrees(yaws[after])) - abs(spin_target)),
                         "origin_after_spin_mm": [float(x[i_spin_end] * 1000), float(y[i_spin_end] * 1000)],
                         **_normalize_metrics(c, s, x, y, head, norm[1], stp[1]),
                         "end_lat_mm": float(y[-1] * 1000), "end_heading_deg": math.degrees(float(head[-1])),
                         "end_along_err_mm": float((x[-1] - goal) * 1000), "stop_time_s": stop, **extra}, "checks": checks}


def v6b(c, s):
    t = c["t_s"]
    mp = s["maneuver"]["params"]
    side = 1 if mp["side"] == "L" else -1
    obs, info = mz.piece("corner", mp["corner"], side)
    x, y, yaw = _start_frame(c)
    dmin, k = _clear(c, x, y, yaw, obs, _robot_hull(c))
    cx, ey = info["corner_center"][0], abs(info["exit_center"][1])
    along, lat = side * y, -side * (x - cx)          # 나가는 통로 기준: 진행, 옆(왼쪽 +)
    head = (side * yaw - math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
    head = side * head
    arc, norm, stp = _phase(s, "arc"), _phase(s, "normalize"), _phase(s, "stop")
    i_arc_end = at(c, arc[2])
    stop = _stop_time(c, stp[1])
    checks, extra = _norm_checks(c, dmin, lat[-1], math.degrees(float(head[-1])), along[-1] - ey, stop)
    checks = {"호 회전 90° 도달": arc[2] is not None, **checks}
    return {"measured": {"corner": mp["corner"], "side": mp["side"], "radius_cmd_m": mp["radius_m"],
                         **_align_metrics(c, s, lat, along),
                         "min_clearance_mm": dmin * 1000, "min_clearance_t_s": float(t[k]),
                         "arc_end_outward_mm": float(x[i_arc_end] - cx) * 1000, "arc_end_along_m": float(along[i_arc_end]),
                         **_normalize_metrics(c, s, along, lat, head, norm[1], stp[1]),
                         "end_outward_mm": float(x[-1] - cx) * 1000, "end_heading_deg": math.degrees(float(head[-1])),
                         "end_along_err_mm": float((along[-1] - ey) * 1000), "stop_time_s": stop, **extra}, "checks": checks}


def v2(c):
    t = c["t_s"]
    win = max(1, int(round(0.1 / float(np.median(np.diff(t))))))
    i0 = at(c, 2.0)
    yaw0 = c["yaw_rad"][i0]
    h = np.array([math.cos(yaw0), math.sin(yaw0)])
    n = np.array([-h[1], h[0]])
    rel = np.stack([c["base_x_m"] - c["base_x_m"][i0], c["base_y_m"] - c["base_y_m"][i0]], axis=1)
    along, side = rel @ h, rel @ n
    fwd = float(along[at(c, 8.0)] - along[i0])
    back = float(along[at(c, 8.0)] - along[at(c, 14.0) - 1])
    run = t >= 2.0
    side_max = float(np.max(np.abs(side[run])))
    yaw_max = math.degrees(float(np.max(np.abs(np.unwrap(c["yaw_rad"][run]) - yaw0))))
    speed = np.hypot(fd(c["base_x_m"], t, win), fd(c["base_y_m"], t, win))
    idx = np.where((t > 12.0) & (t <= 14.0))[0]
    above = idx[speed[idx] > 0.005]
    stop = 0.0 if len(above) == 0 else (None if above[-1] == idx[-1] else float(t[above[-1] + 1] - 12.0))
    ok_c, mc = contact_check(c)
    ok_d, md = dev_check(c, 2.0)
    checks = {"전진 0.18~0.22 m": 0.18 <= fwd <= 0.22, "후진 0.18~0.22 m": 0.18 <= back <= 0.22, "옆 이탈 ≤ 10 mm": side_max <= 0.010,
              "yaw 변화 ≤ 2°": yaw_max <= 2.0, "정지 명령 뒤 0.5 s 안에 ≤ 0.005 m/s": stop is not None and stop <= 0.5,
              "팔 이탈 ≤ 0.02 rad (2 s 이후)": ok_d, "팔–다른 형상 실제 접촉 없음": ok_c}
    return {"measured": {"forward_m": fwd, "backward_m": back, "side_max_mm": side_max * 1000, "yaw_max_deg": yaw_max,
                         "stop_time_s": stop, **mc, **md}, "checks": checks}


def v3(c):
    t = c["t_s"]
    win = max(1, int(round(0.1 / float(np.median(np.diff(t))))))
    sp = {n: float(np.nanmax(np.abs(fd(c["q_" + n], t, win)))) for n in ARM}
    q = np.stack([c["q_" + n] for n in ARM], axis=1)
    e_stow = float(np.max(np.abs(q[at(c, 4.0) - 1] - STOW)))
    e_zero = float(np.max(np.abs(q[-1])))
    ok_c, mc = contact_check(c)
    checks = {"팔–다른 형상 실제 접촉 없음": ok_c, "팔 실제 속도 ≤ 4.9532 rad/s": max(sp.values()) <= ARM_SPEED_LIMIT,
              "도달 오차 ≤ 0.05 rad (4 s 접힘, 7 s 0 자세)": max(e_stow, e_zero) <= 0.05}
    return {"measured": {"arm_speed_max_rad_s": sp, "reach_err_stow_rad": e_stow, "reach_err_zero_rad": e_zero, **mc}, "checks": checks}


def main(folder):
    folder = Path(folder)
    s = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    c = load(folder)
    if s["test"] in ("V1b", "V4", "V5", "V6", "V6b", "V7", "V6c", "V6cb", "V7c", "V8"):
        res = {"V1b": v1b, "V4": v4, "V5": v5, "V6": v6, "V6b": v6b, "V7": v6,
               "V6c": v6, "V7c": v6, "V6cb": v6b, "V8": v4}[s["test"]](c, s)
    else:
        res = {"V1": v1, "V2": v2, "V3": v3}[s["test"]](c)
    res["verdict"] = "성공" if all(res["checks"].values()) else "실패"
    res["engine"] = s.get("engine", "Isaac Sim")
    res["test"] = s["test"]
    (folder / "verdict_stow.json").write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
    bad = [k for k, v in res["checks"].items() if not v]
    print(f"{res['engine']} {s['test']} {res['verdict']} {('실패: ' + ', '.join(bad)) if bad else ''} {folder.name}")
    return res


if __name__ == "__main__":
    for f in sys.argv[1:]:
        main(f)
