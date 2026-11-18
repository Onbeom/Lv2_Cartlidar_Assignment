"""L2 판정 → summary.md (검증계획 9-3, 사용자 승인 2026-10-25). 시스템 python3 + numpy.

사용: python3 analyze_l2.py <결과 폴더>
기대값은 Isaac 참값(truth.csv: 차체 자세)과 관찰 노드가 받은 TF(base_footprint→센서 frame)로 계산한다.
"""

import csv
import json
import math
import os
import statistics
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from wheel_kinematics import DiffDrive, load_config  # noqa: E402

D = sys.argv[1]
RATIO_MIN = 0.95
LAT_MAX = 0.3               # [s] 영상 지연 (인지 frame_max_age_s)
DIST_TOL = 0.02             # [m] 라이다·깊이 거리
JS_TOL = 1e-3               # [rad]
K_REL = 1e-4
WHEEL_TOL = 1e-6            # [rad/s]
DEPTH_PAIR = 0.02           # [s] 깊이·컬러 stamp 차 (인지 짝짓기 기준)
GAP_FLAG = 0.5              # [s] 이보다 긴 간격은 표시 (인지 신호등 max_gap)
BASE_Z = 0.0067499816754    # [m] URDF base_footprint → base_link
SELF_M = 0.30               # [m] 이보다 가까운 라이다 반사 = 로봇 자기 몸체(확인용 물체는 모두 0.6 m 이상)


def p(n):
    return os.path.join(D, n)


def jload(n, default=None):
    try:
        with open(p(n)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def rows(n):
    try:
        with open(p(n)) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def text(n):
    try:
        with open(p(n), errors="replace") as f:
            return f.read()
    except OSError:
        return ""


meta = jload("meta.json", {})
obs = jload("observer_info.json", {})
counts = jload("scene_counts.json", {})
truth = [{k: float(v) for k, v in r.items()} for r in rows("truth.csv")]
results = []


def add(stage, item, crit, measured, ok):
    if isinstance(ok, (bool, np.bool_)):
        ok = bool(ok)
    results.append((stage, item, crit, measured, "통과" if ok is True else ("실패" if ok is False else str(ok))))


rx = {}
for r in rows("rx.csv"):
    rx.setdefault(r["topic"], []).append(r)


# ---------- 기하 ----------
def q_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(r):
    return math.atan2(2 * (r["qw"] * r["qz"] + r["qx"] * r["qy"]), 1 - 2 * (r["qy"] ** 2 + r["qz"] ** 2))


def truth_at(t):
    return min(truth, key=lambda r: abs(r["sim_s"] - t)) if truth else None


def world_T_base(t):
    """world ← base_footprint (4x4), Isaac 참값의 base_link 자세에서 z 오프셋을 뺌"""
    r = truth_at(t)
    T = np.eye(4)
    T[:3, :3] = q_to_R(r["qx"], r["qy"], r["qz"], r["qw"])
    T[:3, 3] = [r["x"], r["y"], r["z"]]
    off = np.eye(4)
    off[2, 3] = -BASE_Z
    return T @ off


def base_T_frame(fr):
    e = obs["snapshot"]["tf_base"]["frames"][fr]
    T = np.eye(4)
    T[:3, :3] = q_to_R(*e["q_xyzw"])
    T[:3, 3] = e["t"]
    return T


def ray_box(o, d, c, s):
    """축 정렬 상자와 광선의 첫 교차 거리(없으면 inf). 광선 시작점이 상자 안이면 inf"""
    lo, hi = np.asarray(c) - np.asarray(s) / 2, np.asarray(c) + np.asarray(s) / 2
    tmin, tmax = -math.inf, math.inf
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if o[i] < lo[i] or o[i] > hi[i]:
                return math.inf
            continue
        t1, t2 = (lo[i] - o[i]) / d[i], (hi[i] - o[i]) / d[i]
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmax >= tmin > 0 else math.inf


def cast(o, d, collide_only, floor):
    best, who = math.inf, None
    for ob in meta.get("objects", []):
        if collide_only and not ob["collide"]:
            continue
        t = ray_box(o, d, ob["center"], ob["size"])
        if t < best:
            best, who = t, ob["name"]
    if floor and d[2] < -1e-9:
        t = -o[2] / d[2]
        if 0 < t < best:
            best, who = t, "floor"
    return best, who


# ---------- C1 ----------
clock = [float(r["stamp_s"]) for r in rx.get("/clock", [])]
back = sum(1 for i in range(1, len(clock)) if clock[i] < clock[i - 1])
add("C1", "C1-1 /clock 진행", "받음, 거꾸로 0회", f"{len(clock)}개, 거꾸로 {back}회", bool(clock) and back == 0)
ust = text("use_sim_time.txt").strip().splitlines()
ok_ust = len(ust) == 2 and all(ln.rstrip().endswith("True") for ln in ust)
add("C1", "C1-2 바깥 노드 use_sim_time", "robot_state_publisher·관찰 노드 모두 True",
    " / ".join(ln.strip() for ln in ust) or "기록 없음", ok_ust)

# ---------- C2 ----------
sim_s = counts.get("sim_s", 0.0)
rd = meta.get("render_dt", 1 / 30)
rates = {"/clock": 1 / rd, "/odom": 1 / rd, "/joint_states": meta.get("joint_state_hz", 50.0),
         "/scan_front": meta.get("lidar", {}).get("hz", 10.0), "/scan_rear": meta.get("lidar", {}).get("hz", 10.0)}
cams = meta.get("cameras", {})
img_topics, info_topics, depth_topics = {}, {}, {}
for c, v in cams.items():
    base = v["topic"]
    img_topics[c] = f"{base}/color/image_raw" if c != "arm" else f"{base}/image_raw"
    info_topics[c] = f"{base}/color/camera_info" if c != "arm" else f"{base}/camera_info"
    if v["depth"]:
        depth_topics[c] = f"{base}/aligned_depth_to_color/image_raw"
for t in list(img_topics.values()) + list(info_topics.values()) + list(depth_topics.values()):
    rates[t] = 1 / rd
odom_tf = [r for r in rx.get("/tf", [])]
for t, hz in rates.items():
    got = len(rx.get(t, []))
    e = sim_s * hz
    ratio = got / e if e else 0.0
    add("C2", f"C2-1 {t} 받은 개수", f"설정 {hz:.0f} Hz × {sim_s:.0f} s = {e:.0f}의 95 % 이상", f"{got} ({ratio:.1%})",
        ratio >= RATIO_MIN)
got = len(odom_tf)
e = sim_s / rd
add("C2", "C2-1 /tf odom→base_footprint 받은 개수", f"{e:.0f}의 95 % 이상", f"{got} ({got / e if e else 0:.1%})",
    e > 0 and got / e >= RATIO_MIN)

for t in list(img_topics.values()) + list(depth_topics.values()):
    lat = [float(r["rx_clock_s"]) - float(r["stamp_s"]) for r in rx.get(t, [])]
    if not lat:
        add("C2", f"C2-2 {t} 지연", f"100 % ≤ {LAT_MAX} s", "데이터 없음", False)
        continue
    p95 = sorted(lat)[int(0.95 * (len(lat) - 1))]
    add("C2", f"C2-2 {t} 지연", f"100 % ≤ {LAT_MAX} s (받은 순간 /clock − stamp)",
        f"최대 {max(lat):.3f} s, p95 {p95:.3f} s, 초과 {sum(x > LAT_MAX for x in lat)}개", max(lat) <= LAT_MAX)

snap = obs.get("snapshot", {})
have_geo = "tf_base" in snap and truth
# C2-3 라이다 방향
for topic, frame in meta.get("lidar", {}).get("topics", {}).items():
    sc = snap.get(topic)
    if not (sc and have_geo):
        add("C2", f"C2-3 {topic} 방향", "정면 벽 ± 0.02 m, 상자 광선 80 % ± 0.02 m", "snapshot 없음", False)
        continue
    T = world_T_base(sc["stamp"]) @ base_T_frame(frame)
    Rb = world_T_base(sc["stamp"])[:3, :3]
    o = T[:3, 3]
    want_dir = Rb @ np.array([1.0 if "front" in topic else -1.0, 0, 0])
    best_i, best_dot, exp, hit = None, -2, [], []
    for i, r in enumerate(sc["ranges"]):
        a = sc["angle_min"] + i * sc["angle_increment"]
        d = T[:3, :3] @ np.array([math.cos(a), math.sin(a), 0.0])
        t_hit, who = cast(o, d, True, False)
        exp.append(t_hit)
        hit.append(who)
        if float(d @ want_dir) > best_dot:
            best_dot, best_i = float(d @ want_dir), i
    m_front = sc["ranges"][best_i]
    ok_front = m_front is not None and abs(m_front - exp[best_i]) <= DIST_TOL
    box_all = [i for i, w in enumerate(hit) if w == "box_left"]
    # 로봇 자기 몸체(바퀴·기둥 등)에 가린 빔은 상자를 볼 수 없으므로 뺀다: 측정 거리 < SELF_M
    selfhit = [i for i in box_all if sc["ranges"][i] is not None and sc["ranges"][i] < SELF_M]
    box_i = [i for i in box_all if i not in selfhit]
    box_ok = [i for i in box_i if sc["ranges"][i] is not None and abs(sc["ranges"][i] - exp[i]) <= DIST_TOL]
    frac = len(box_ok) / len(box_i) if box_i else 0.0
    n_self_all = sum(1 for x in sc["ranges"] if x is not None and x < SELF_M)
    add("C2", f"C2-3 {topic} 정면 벽 거리", f"계산값 ± {DIST_TOL} m",
        f"측정 {m_front if m_front is None else round(m_front, 4)} / 계산 {exp[best_i]:.4f} m (빔 {best_i})", ok_front)
    add("C2", f"C2-3 {topic} 상자 방향", f"상자에 닿아야 하는 빔(몸체 가림 < {SELF_M} m 제외)의 80 % 이상이 계산값 ± {DIST_TOL} m",
        f"{len(box_ok)}/{len(box_i)} ({frac:.0%}), 몸체 가림 제외 {len(selfhit)}/{len(box_all)}, "
        f"계산 각도 {[box_all[0], box_all[-1]] if box_all else '-'}번 빔, 전체 스캔 중 < {SELF_M} m 빔 {n_self_all}개",
        bool(box_i) and frac >= 0.8)


# C2-4 카메라 방향·깊이
def marker_side(cam, marker, color_idx):
    fr = cams[cam]["frame"]
    img_t = img_topics[cam]
    s = snap.get(img_t)
    if not (s and have_geo):
        return None, "snapshot 없음"
    img = np.load(p(os.path.join("snapshot", s["file"]))).astype(np.int32)
    ch = [img[..., i] for i in range(3)]
    main = ch[color_idx]
    others = [ch[i] for i in range(3) if i != color_idx]
    mask = (main > 80) & (main > 2 * others[0]) & (main > 2 * others[1])
    fx, fy, cx, cy = cams[cam]["K"]
    ob = next(o for o in meta["objects"] if o["name"] == marker)
    T = world_T_base(s["stamp"]) @ base_T_frame(fr)
    pc = np.linalg.inv(T) @ np.array(list(ob["center"]) + [1.0])
    u_exp = fx * pc[0] / pc[2] + cx
    if mask.sum() < 20:
        return False, f"표식 화소 {int(mask.sum())}개(못 찾음), 계산 u {u_exp:.1f}"
    u_meas = float(np.nonzero(mask)[1].mean())
    ok = (u_meas - cx) * (u_exp - cx) > 0
    return ok, f"측정 u {u_meas:.1f} / 계산 u {u_exp:.1f} (cx {cx:.1f}, 차 {u_meas - u_exp:+.1f} px, 화소 {int(mask.sum())})"


for cam, marker, ci in (("front", "marker_front_red", 0), ("rear", "marker_rear_green", 1)):
    if cam in cams:
        ok, txt = marker_side(cam, marker, ci)
        add("C2", f"C2-4 {cam} 카메라 표식 방향", "표식이 계산한 쪽 절반에 보임", txt, ok if ok is not None else False)
if "front" in depth_topics and have_geo and snap.get(depth_topics["front"]):
    s = snap[depth_topics["front"]]
    dimg = np.load(p(os.path.join("snapshot", s["file"])))[..., 0]
    fx, fy, cx, cy = cams["front"]["K"]
    T = world_T_base(s["stamp"]) @ base_T_frame(cams["front"]["frame"])
    t_hit, who = cast(T[:3, 3], T[:3, :3] @ np.array([0, 0, 1.0]), False, True)
    d_meas = float(dimg[int(round(cy)), int(round(cx))])
    add("C2", "C2-4 앞 카메라 중앙 깊이", f"계산값 ± {DIST_TOL} m", f"측정 {d_meas:.4f} / 계산 {t_hit:.4f} m ({who})",
        abs(d_meas - t_hit) <= DIST_TOL)
else:
    add("C2", "C2-4 앞 카메라 중앙 깊이", f"계산값 ± {DIST_TOL} m", "snapshot 없음", False)

# C2-5 JointState
dof = meta.get("dof_names", [])
tq = {round(r["sim_s"], 4): r for r in truth}
js = rx.get("/joint_states", [])
bad_names, worst, n_cmp = 0, 0.0, 0
for r in js:
    e = json.loads(r["extra"])
    if e["name"] != dof:
        bad_names += 1
        continue
    tr = tq.get(round(float(r["stamp_s"]), 4))
    if tr is None:
        continue
    n_cmp += 1
    worst = max(worst, max(abs(v - tr[f"q_{n}"]) for n, v in zip(e["name"], e["position"])))
add("C2", "C2-5 /joint_states 이름·위치", f"이름 {len(dof)}개 순서 일치, 위치 = 참값 ± {JS_TOL} rad",
    f"{len(js)}개 중 이름 불일치 {bad_names}, 참값과 비교 {n_cmp}개, 최대 차 {worst:.2e}",
    bool(js) and bad_names == 0 and n_cmp > 0 and worst <= JS_TOL)

# C2-6 깊이 형식
for c, t in depth_topics.items():
    drows, crows = rx.get(t, []), rx.get(img_topics[c], [])
    bad_fmt = sum(1 for r in drows if json.loads(r["extra"])[:3] != [cams[c]["width"], cams[c]["height"], "32FC1"])
    cst = sorted(float(r["stamp_s"]) for r in crows)
    paired = 0
    for r in drows:
        s = float(r["stamp_s"])
        j = np.searchsorted(cst, s)
        near = [abs(cst[k] - s) for k in (j - 1, j) if 0 <= k < len(cst)]
        paired += bool(near) and min(near) <= DEPTH_PAIR
    frac = paired / len(drows) if drows else 0.0
    add("C2", f"★C2-6 {c} 깊이 형식·짝", f"32FC1 {cams[c]['width']}x{cams[c]['height']}, 컬러와 stamp 차 ≤ {DEPTH_PAIR} s ≥ 95 %",
        f"{len(drows)}개 중 형식 불일치 {bad_fmt}, 짝 {frac:.1%}", bool(drows) and bad_fmt == 0 and frac >= RATIO_MIN)

# ---------- C3 TF ----------
lk = rows("tf_lookup.csv")
fails = [r for r in lk if r["ok"] != "1"]
add("C3", "C3-1 odom → 인지 frame TF 조회", "10 s마다 6개 frame, 실패 0회",
    f"{len(lk)}회 중 실패 {len(fails)}" + (f" (예: {fails[0]['source']} {fails[0]['error'][:80]})" if fails else ""),
    len(lk) > 0 and not fails)
parents = {}
for topic, par, child, n in obs.get("tf_pairs", []):
    parents.setdefault(child, set()).add(par)
multi = {c: sorted(ps) for c, ps in parents.items() if len(ps) > 1}
pubs = obs.get("tf_publishers", {})
tfp, tsp = pubs.get("/tf", []), pubs.get("/tf_static", [])
ok_pub = len(tfp) == 2 and "robot_state_publisher" in tfp and tsp == ["robot_state_publisher"]
add("C3", "C3-2 TF 발행자 하나", "자식 frame마다 부모 1개, /tf = Isaac + robot_state_publisher, /tf_static = robot_state_publisher, "
    "base_footprint의 부모 = odom",
    f"부모 여럿 {multi or '없음'}, /tf {tfp}, /tf_static {tsp}, base_footprint 부모 {sorted(parents.get('base_footprint', []))}",
    not multi and ok_pub and parents.get("base_footprint") == {"odom"})

# ---------- C4 stamp·CameraInfo ----------
for c in cams:
    cst = [r["stamp_s"] for r in rx.get(img_topics[c], [])]
    ist = {r["stamp_s"] for r in rx.get(info_topics[c], [])}
    m = sum(1 for s in cst if s in ist)
    frac = m / len(cst) if cst else 0.0
    add("C4", f"C4-1 {c} 컬러·CameraInfo stamp", "≥ 95 % 같음", f"{m}/{len(cst)} ({frac:.1%})", bool(cst) and frac >= RATIO_MIN)
    fx, fy, cx, cy = cams[c]["K"]
    k_exp = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    p_exp = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
    bad, ex = 0, None
    infos = rx.get(info_topics[c], [])
    for r in infos:
        e = json.loads(r["extra"])
        k, rr, pp = e["k"], e["r"], e["p"]
        why = []
        if any(abs(a - b) > K_REL * max(1.0, abs(b)) for a, b in zip(k, k_exp)):
            why.append(f"K {k}")
        if not (k[8] == 1.0 and k[1] == 0.0 and k[3] == 0.0 and k[0] > 0 and k[4] > 0):
            why.append("k[8]·k[1]·k[3]·fx·fy")
        if rr != [1, 0, 0, 0, 1, 0, 0, 0, 1]:
            why.append(f"R {rr}")
        if any(abs(a - b) > K_REL * max(1.0, abs(b)) for a, b in zip(pp, p_exp)):
            why.append(f"P {pp}")
        if r["frame_id"] != cams[c]["frame"] or (e["w"], e["h"]) != (cams[c]["width"], cams[c]["height"]):
            why.append(f"frame {r['frame_id']} 크기 {e['w']}x{e['h']}")
        if why:
            bad += 1
            ex = ex or "; ".join(why)
    add("C4", f"C4-2 {c} CameraInfo 값", f"K = {[round(x, 3) for x in (fx, fy, cx, cy)]}(상대 {K_REL:g}), k[8]=1·k[1]=k[3]=0·fx,fy>0, "
        "R 단위, P=[K|0], frame·크기", f"{len(infos)}개 중 불일치 {bad}" + (f" (예: {ex[:160]})" if ex else ""),
        bool(infos) and bad == 0)

# ---------- C5 주행 ----------
dd = DiffDrive(load_config(meta.get("wheels_config", os.path.join(HERE, "config/wheels.yaml"))))
pdt = meta.get("phys_dt", 1 / 120)
worst_w, n_w, n_edge = 0.0, 0, 0
for r in truth:
    age = r["cmd_age_s"]
    if age >= 0 and abs((age - pdt) - dd.timeout) < 1e-5:   # 끊김 경계(정확히 timeout): 기록 반올림 때문에 어느 쪽이든 인정
        n_edge += 1
        continue
    t_pre = r["sim_s"] - pdt                       # 바퀴 목표는 이 기록 직전 pre_step에서 정함
    want = dd.rates_at(t_pre, (t_pre - (age - pdt)) if age >= 0 else None, r["cmd_v"], r["cmd_w"]) \
        if age >= 0 else [0.0] * len(dd.joints)
    got = [r[f"tgt_{j}"] for j in dd.joints]
    worst_w = max(worst_w, max(abs(a - b) for a, b in zip(want, got)))
    n_w += 1
crx, ctx = rows("cmd_rx.csv"), rows("cmd_tx.csv")
sent = {(float(r["v"]), float(r["w"])) for r in ctx}
bad_rx = [r for r in crx if (float(r["v"]), float(r["w"])) not in sent or r["frame_id"] != obs.get("cmd_frame")]
add("C5", "C5-1 명령 → 바퀴 목표", f"기구학 계산값과 차이 ≤ {WHEEL_TOL:g} rad/s, 받은 명령 = 보낸 값",
    f"{n_w}행 최대 차 {worst_w:.2e}(끊김 경계 {n_edge}행 제외), 보냄 {len(ctx)}·받음 {len(crx)}, 값·frame 불일치 {len(bad_rx)}",
    n_w > 0 and worst_w <= WHEEL_TOL and bool(crx) and not bad_rx)

sched = obs.get("schedule", [])
if truth and sched and truth[-1]["sim_s"] > 27:
    r0, r1 = truth_at(10.0), truth_at(17.0)
    y0 = yaw_of(r0)
    dx, dy = r1["x"] - r0["x"], r1["y"] - r0["y"]
    along = dx * math.cos(y0) + dy * math.sin(y0)
    lat = -dx * math.sin(y0) + dy * math.cos(y0)
    dyaw = math.degrees(math.remainder(yaw_of(r1) - y0, 2 * math.pi))
    add("C5", "C5-2 직진 0.10 m/s 5 s", "이동 0.50 ± 0.025 m, 옆 ≤ 0.02 m, yaw ≤ 2°",
        f"이동 {along:.4f} m, 옆 {lat:+.4f} m, yaw {dyaw:+.2f}°", abs(along - 0.5) <= 0.025 and abs(lat) <= 0.02 and abs(dyaw) <= 2)
    ya, yb = yaw_of(truth_at(17.0)), yaw_of(truth_at(23.0))
    turn = math.remainder(yb - ya, 2 * math.pi)
    add("C5", "C5-3 회전 +0.5 rad/s 4 s", "yaw 증가(반시계), 회전량은 기록",
        f"{math.degrees(turn):+.1f}° (명령 {math.degrees(2.0):.1f}°, 비율 {turn / 2.0:.2f})", turn > 0)

    def first_stop(t0):
        for r in truth:
            if r["sim_s"] >= t0 and math.hypot(r["vx"], r["vy"]) < 0.01:
                return r
        return None
    def first_zero(t_from):
        """장면이 0 명령을 실제로 처음 받은 시각(관찰 노드 /clock과 장면 시간은 한 프레임쯤 다를 수 있음)"""
        return next((float(r["sim_s"]) for r in crx if float(r["sim_s"]) >= t_from
                     and float(r["v"]) == 0.0 and float(r["w"]) == 0.0), None)
    z1, z2 = first_zero(14.0), first_zero(20.0)
    s1 = first_stop(z1) if z1 is not None else None
    s2 = first_stop(z2) if z2 is not None else None
    t_last = max((float(r["sim_s"]) for r in crx), default=None)
    t_zero = next((r["sim_s"] for r in truth if t_last is not None and r["sim_s"] > t_last
                   and all(r[f"tgt_{j}"] == 0.0 for j in dd.joints)), None)
    s3 = first_stop(t_zero) if t_zero is not None else None
    ok4 = (s1 is not None and s1["sim_s"] - z1 <= 0.5 and s2 is not None and s2["sim_s"] - z2 <= 0.5
           and t_zero is not None and t_zero - t_last <= dd.timeout + 0.02 and s3 is not None and s3["sim_s"] - t_zero <= 0.5)
    w21 = truth_at(z2 + 0.5)["wz"] if z2 is not None else float("nan")
    def dt_txt(s, z):
        return f"{s['sim_s'] - z:.3f}" if (s and z is not None) else "없음"
    add("C5", "C5-4 정지", "0 명령을 받은 뒤 0.5 s 안에 < 0.01 m/s(직진·회전 뒤), 명령이 끊기면 0.5 s 뒤 바퀴 목표 0, 그 뒤 0.5 s 안에 정지",
        f"직진 뒤 0 명령 {z1:.3f} s → {dt_txt(s1, z1)} s, 회전 뒤 0 명령 {z2:.3f} s → {dt_txt(s2, z2)} s "
        f"(그 0.5 s 뒤 ωz {w21:+.4f} rad/s), 마지막 명령 {t_last:.3f} s → 목표 0 {t_zero:.3f} s → 정지 "
        f"{s3['sim_s']:.3f} s" if (z1 is not None and z2 is not None and s3) else "기록 부족", ok4)
else:
    add("C5", "C5-2~4 주행", "명령 시간표 27 s 이상 기록", "기록 부족", False)

# ---------- 간격·전체 ----------
gap_flags = []
for t in sorted(rx):
    if t in ("/tf",):
        continue
    st = sorted(float(r["stamp_s"]) for r in rx[t])
    if len(st) < 3:
        continue
    g = max(b - a for a, b in zip(st, st[1:]))
    w = sorted(float(r["rx_wall_s"]) for r in rx[t])
    gw = max(b - a for a, b in zip(w, w[1:]))
    if g > GAP_FLAG:
        gap_flags.append(t)
    add("간격", f"★{t} 최대 간격", f"기록, stamp 간격 {GAP_FLAG} s 초과 시 표시",
        f"stamp {g * 1000:.0f} ms, 받은 시각 {gw * 1000:.0f} ms", "표시" if g > GAP_FLAG else "기록")
code = text("isaac_exit_code.txt").strip()
dur = meta.get("duration_s", 600.0)
add("전체", "시뮬레이션 시간", f"≥ {dur:.0f} s", f"{sim_s:.1f} s (실제 {counts.get('wall_s', 0):.0f} s)", sim_s >= dur - 0.05)
add("전체", "Isaac Sim 종료", "정상 종료(0)", code or "기록 없음", code == "0")

lines = ["# L2 로봇 장면 검사 결과 (방식 (a), 조건 A′, 모의 시험)", "", f"결과 폴더: `{os.path.basename(os.path.abspath(D))}`", ""]
for stage in ("C1", "C2", "C3", "C4", "C5", "간격", "전체"):
    rs = [r for r in results if r[0] == stage]
    judged = [r for r in rs if r[4] in ("통과", "실패")]
    verdict = ("통과" if all(r[4] == "통과" for r in judged) else "실패") if judged else "기록"
    lines += [f"## {stage} — {verdict}", "", "| 항목 | 기준 | 측정 | 판정 |", "|---|---|---|---|"]
    lines += [f"| {r[1]} | {r[2]} | {str(r[3]).replace('|', '/')} | {r[4]} |" for r in rs]
    lines.append("")
with open(p("summary.md"), "w") as f:
    f.write("\n".join(lines))
print("\n".join(lines))
