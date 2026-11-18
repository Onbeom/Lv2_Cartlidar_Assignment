"""브리지 검증 결과 판정 → summary.md (표준 라이브러리만 사용).

사용: python3 analyze_bridge.py <결과 폴더> --cond A|B
판정 기준은 시험 전에 사용자가 확인한 값이다(받은 개수 95 %, 간격 3배, LiDAR 2.00 ± 0.02 m 등).
★ 항목(CameraInfo d·r·p, TF 2개, LaserScan intensities, TwistStamped, 시스템 rclpy)은 2026-10-25 승인
(261025_0307/검증계획_ROS연동_261025.md 2장).
"""

import argparse
import csv
import json
import math
import os
import re
import statistics

RATIO_MIN = 0.95          # 받은 개수 / 발행한 개수
GAP_FACTOR = 3.0          # 평소 간격의 몇 배를 넘으면 끊김으로 셀지
WALL_TOL = 0.02           # [m]
POS_TOL_TF = 1e-4         # [m]
ANG_TOL = 1e-4            # [rad]
POS_TOL_ODOM = 1e-3       # [m]
K_REL_TOL = 1e-4
TWIST_TOL = 1e-6
MEM_GROWTH_FLAG = 0.05    # 1분 시점 대비 증가율, 표시만 함

ap = argparse.ArgumentParser()
ap.add_argument("dir")
ap.add_argument("--cond", choices=["A", "A2", "B"], required=True)
a = ap.parse_args()
D = a.dir


def p(name):
    return os.path.join(D, name)


def load_json(name, default=None):
    try:
        with open(p(name)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def read_text(name):
    try:
        with open(p(name), errors="replace") as f:
            return f.read()
    except OSError:
        return ""


meta = load_json("meta.json", {})
env = load_json("isaac_env.json", {})
first = load_json("first_msgs.json", {})
obs = load_json("observer_info.json", {})
pubinfo = load_json("observer_pubinfo.json", {})
results = []      # (단계, 항목, 기준, 측정, 판정)


def add(stage, item, crit, measured, ok):
    results.append((stage, item, crit, measured, "통과" if ok is True else ("실패" if ok is False else str(ok))))


# ---------- 기록 읽기 ----------
def read_rows(name):
    """CSV 행 목록. Isaac Sim이 장면을 만들기 전에 끝나 파일이 없으면 빈 목록"""
    try:
        with open(p(name)) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


ticks, pause_begin, pause_end = [], None, None
if True:
    for r in read_rows("ticks.csv"):
        if r["state"] == "play":
            ticks.append((float(r["wall_s"]), float(r["sim_s"])))
        elif r["state"] == "pause_begin":
            pause_begin = float(r["wall_s"])
        elif r["state"] == "pause_end":
            pause_end = float(r["wall_s"])

rx = {}
if os.path.exists(p("rx.csv")):
    with open(p("rx.csv")) as f:
        rd = csv.reader(f)
        next(rd, None)
        for r in rd:
            rx.setdefault(r[0], []).append(r)

# ---------- B0 기동 ----------
LYR = a.cond in ("A", "A2")
expect_distro = "lyrical" if LYR else "jazzy"
add("B0", "ROS_DISTRO 인식", f"= {expect_distro}", env.get("ROS_DISTRO"), env.get("ROS_DISTRO") == expect_distro)

logs = read_text("isaac_stdout.txt") + "\n" + read_text("kit_log.txt")
err_pat = re.compile(r"undefined symbol|cannot open shared object|symbol lookup error|"
                     r"\[Error\].*(ros2|rcl|rmw|isaacsim\.ros2)", re.I)
# Lyrical 전용 백엔드가 없어 기존 jazzy 백엔드로 넘어간다는 안내 줄은 오류로 세지 않고 기록한다(사용자 승인 2026-10-25)
FALLBACK = "libisaacsim.ros2.core.lyrical.so"
all_err = {ln.strip()[:200] for ln in logs.splitlines() if err_pat.search(ln)}
fallback_lines = sorted(x for x in all_err if FALLBACK in x)
err_lines = sorted(x for x in all_err if FALLBACK not in x)
add("B0", "ROS 관련 오류·심볼 미해결", f"0줄 ({FALLBACK} 안내 줄 제외)", f"{len(err_lines)}줄", len(err_lines) == 0)
add("B0", f"{FALLBACK} 없음 안내(jazzy 백엔드 사용)", "기록", f"{len(fallback_lines)}줄", "기록")

maps = read_text("proc_maps.txt")
libs = sorted({ln.split()[-1] for ln in maps.splitlines() if ln.split() and ln.split()[-1].startswith("/")})
expect_prefix = "/opt/ros/lyrical/" if LYR else "/exts/isaacsim.ros2.core/jazzy/lib/"
core_ok = True
for want in ("librcl.so", "librmw.so", "librmw_cyclonedds_cpp.so"):
    hits = [x for x in libs if os.path.basename(x) == want]
    ok = bool(hits) and all(expect_prefix in x for x in hits)
    core_ok &= ok
    add("B0", f"{want} 위치", f"'{expect_prefix}' 안", ", ".join(hits) or "불러오지 않음", ok)
internal = [x for x in libs if "/exts/isaacsim.ros2.core/" in x and ("/jazzy/lib/" in x or "/humble/lib/" in x)]
backend = [os.path.basename(x) for x in libs if re.search(r"libisaacsim\.ros2\.core\.(jazzy|humble)\.so", x)]
add("B0", "Isaac 백엔드", "기록", ", ".join(backend) or "없음", "기록")
if LYR:
    add("B0", "내장 jazzy 라이브러리 혼입", "기록(실패 아님)", f"{len(internal)}개", "기록")
if "System rclpy loaded" in logs:
    rclpy_state = "시스템 rclpy 불러옴"
else:
    m = re.search(r"Could not import system rclpy[^\n]*", logs)
    rclpy_state = m.group(0)[:160] if m else ("내장 rclpy" if a.cond == "B" else "기록 없음")
add("B0", "★ Isaac 프로세스 안 rclpy", "기록(실패 아님)", rclpy_state, "기록")

# ---------- B1 /clock ----------
clock = [(float(r[1]), float(r[2])) for r in rx.get("/clock", [])]
add("B1", "/clock 수신", "1개 이상", f"{len(clock)}개", len(clock) > 0)
back = sum(1 for i in range(1, len(clock)) if clock[i][1] < clock[i - 1][1])
add("B1", "시각이 거꾸로 간 횟수", "0", back, back == 0 if clock else False)
if pause_begin and pause_end and clock:
    pre = [c for t, c in clock if t <= pause_begin + 0.2]
    during = [c for t, c in clock if pause_begin + 0.2 < t < pause_end]
    after = [c for t, c in clock if pause_end < t < pause_end + 5.0]
    pre_max = max(pre) if pre else float("nan")
    grew = any(c > pre_max + 1e-9 for c in during)
    add("B1", "일시정지 중 시각 증가", "없음", f"메시지 {len(during)}개, 증가 {'있음' if grew else '없음'}", not grew)
    resumed = any(c > pre_max + 1e-9 for c in after)
    add("B1", "재생 재개 후 시각 증가", "있음", "있음" if resumed else "없음", resumed)
else:
    add("B1", "일시정지 시험", "기록 필요", "pause 기록 또는 /clock 없음", False)

# ---------- B2 값 ----------
cam = meta.get("camera", {})
for side in ("left", "right"):
    t = f"/{side}/image_raw"
    rows = rx.get(t, [])
    fid = cam.get(f"frame_{side}")
    bad = [r for r in rows if not (r[3] == fid and int(r[4]) == cam["width"] and int(r[5]) == cam["height"]
                                   and r[6] == cam["encoding"] and int(r[7]) == cam["width"] * 3
                                   and int(r[8]) == cam["width"] * 3 * cam["height"])]
    add("B2", f"{t} 크기·encoding·step·길이·frame_id",
        f"{cam['width']}x{cam['height']} {cam['encoding']} step {cam['width'] * 3}, {fid}",
        f"{len(rows)}개 중 불일치 {len(bad)}" + (f" (예: {bad[0][3:9]})" if bad else ""), bool(rows) and not bad)
    t = f"/{side}/camera_info"
    rows = rx.get(t, [])
    exp = {"fx": cam["fx"], "cx": cam["cx"], "fy": cam["fy"], "cy": cam["cy"]}

    def k_ok(r):
        got = {"fx": float(r[6]), "cx": float(r[7]), "fy": float(r[8]), "cy": float(r[9])}
        return all(abs(got[k] - v) <= K_REL_TOL * abs(v) for k, v in exp.items())

    bad = [r for r in rows if not (r[3] == fid and int(r[4]) == cam["width"] and int(r[5]) == cam["height"]
                                   and k_ok(r))]
    ex = rows[0] if rows else None
    add("B2", f"{t} 크기·fx·fy·cx·cy·frame_id",
        f"fx=fy={cam['fx']:.4f} cx={cam['cx']} cy={cam['cy']} (상대 {K_REL_TOL:g})",
        f"{len(rows)}개 중 불일치 {len(bad)}" + (f" (첫 값 fx {ex[6]} cx {ex[7]} fy {ex[8]} cy {ex[9]})" if ex else ""),
        bool(rows) and not bad)
    img_st = {r[2] for r in rx.get(f"/{side}/image_raw", [])}
    info_st = {r[2] for r in rows}
    if img_st and info_st:
        lo, hi = max(min(img_st), min(info_st)), min(max(img_st), max(info_st))
        common_img = [s for s in img_st if lo <= s <= hi]
        matched = sum(1 for s in common_img if s in info_st)
        frac = matched / len(common_img) if common_img else 0.0
        add("B2", f"{side} Image와 CameraInfo stamp 일치", f"겹치는 구간 영상의 {RATIO_MIN:.0%} 이상",
            f"{matched}/{len(common_img)} ({frac:.1%})", frac >= RATIO_MIN)
    else:
        add("B2", f"{side} Image와 CameraInfo stamp 일치", f"{RATIO_MIN:.0%} 이상", "데이터 없음", False)

D_LEN = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4, "": 0}
fx_m, fy_m, cx_m, cy_m = cam["fx"], cam["fy"], cam["cx"], cam["cy"]
R_EXP = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
for side in ("left", "right"):
    t = f"/{side}/camera_info"
    tx = -fx_m * cam["baseline_m"] if side == "right" else 0.0
    p_exp = [fx_m, 0.0, cx_m, tx, 0.0, fy_m, cy_m, 0.0, 0.0, 0.0, 1.0, 0.0]
    n_bad, ex_bad, n_all = 0, None, 0
    for r in rx.get(t, []):
        if len(r) < 12:
            continue
        n_all += 1
        c = json.loads(r[11])
        why = []
        d_want = D_LEN.get(c["model"])
        if d_want is not None and len(c["d"]) != d_want:
            why.append(f"d 길이 {len(c['d'])}≠{d_want}({c['model']})")
        if not all(math.isfinite(x) for x in c["d"] + c["k"] + c["r"] + c["p"]):
            why.append("유한하지 않은 값")
        for name, got, want in (("r", c["r"], R_EXP), ("p", c["p"], p_exp)):
            if len(got) != len(want) or any(abs(g - w) > K_REL_TOL * max(1.0, abs(w)) for g, w in zip(got, want)):
                why.append(f"{name} {[round(x, 4) for x in got]}")
        if why:
            n_bad += 1
            ex_bad = ex_bad or "; ".join(why)
    add("B2", f"★ {t} d 길이·r 단위행렬·p", f"P[3] = {tx:.4f}, 상대 {K_REL_TOL:g}, 모든 메시지",
        f"{n_all}개 중 불일치 {n_bad}" + (f" (예: {ex_bad[:220]})" if ex_bad else ""), n_all > 0 and n_bad == 0)

tfm = meta.get("tf", {})
tf_rows = [r for r in rx.get("/tf", []) if r[4] == tfm.get("child")]


def quat_angle(q1, q2):
    """두 쿼터니언(xyzw) 사이 회전각. acos 대신 atan2로 계산한다(acos는 1 근처에서 float32 반올림을 크게 부풀림)"""
    n1, n2 = math.sqrt(sum(x * x for x in q1)), math.sqrt(sum(x * x for x in q2))
    a = [x / n1 for x in q1]
    b = [x / n2 for x in q2]
    # q = a * conj(b)
    w = a[3] * b[3] + a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    x = -a[3] * b[0] + a[0] * b[3] - a[1] * b[2] + a[2] * b[1]
    y = -a[3] * b[1] + a[1] * b[3] - a[2] * b[0] + a[0] * b[2]
    z = -a[3] * b[2] + a[2] * b[3] - a[0] * b[1] + a[1] * b[0]
    return 2 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))


bad = []
for r in tf_rows:
    tr = [float(x) for x in r[5:8]]
    q = [float(x) for x in r[8:12]]
    if r[3] != tfm["parent"] or max(abs(x - y) for x, y in zip(tr, tfm["translation"])) > POS_TOL_TF \
            or quat_angle(q, tfm["rotation_xyzw"]) > ANG_TOL:
        bad.append(r)
add("B2", "/tf world→base_link", f"부모 {tfm.get('parent')}, 위치 ±{POS_TOL_TF:g} m, 각도 ±{ANG_TOL:g} rad",
    f"{len(tf_rows)}개 중 불일치 {len(bad)}" + (f" (예: 부모 {bad[0][3]} 값 {bad[0][5:12]})" if bad else ""),
    bool(tf_rows) and not bad)
tf2m = meta.get("tf2", {})
tf2_rows = [r for r in rx.get("/tf", []) if r[4] == tf2m.get("child")]
bad2 = [r for r in tf2_rows if r[3] != tf2m["parent"]
        or max(abs(float(x) - y) for x, y in zip(r[5:8], tf2m["translation"])) > POS_TOL_TF
        or quat_angle([float(x) for x in r[8:12]], tf2m["rotation_xyzw"]) > ANG_TOL]
# 같은 메시지(같은 받은 시각)에 두 변환이 함께 실렸는지
same_msg = len({r[1] for r in tf_rows} & {r[1] for r in tf2_rows})
add("B2", "★ /tf 변환 2개(base_link·marker_link)", f"둘 다 위치 ±{POS_TOL_TF:g} m, 각도 ±{ANG_TOL:g} rad",
    f"marker {len(tf2_rows)}개 중 불일치 {len(bad2)}, 같은 메시지에 함께 실림 {same_msg}회"
    + (f" (예: {bad2[0][3:12]})" if bad2 else ""), bool(tf_rows) and bool(tf2_rows) and not bad and not bad2)

lid = meta.get("lidar", {})
scan = rx.get("/scan", [])
bad_n, bad_r, nan_r, bad_f = 0, 0, 0, 0
for r in scan:
    amin, amax, inc, n = float(r[4]), float(r[5]), float(r[6]), int(r[7])
    n_exp = round((amax - amin) / inc) if inc else -1
    if n not in (n_exp, n_exp + 1):
        bad_n += 1
    fr = float(r[8])
    if not math.isfinite(fr):
        nan_r += 1
    elif abs(fr - lid["wall_dist_m"]) > WALL_TOL:
        bad_r += 1
    if r[3] != lid["frame"]:
        bad_f += 1
fr_vals = [float(r[8]) for r in scan if math.isfinite(float(r[8]))]
add("B2", "/scan ranges 개수가 각도 설정과 맞음", "모든 메시지", f"{len(scan)}개 중 불일치 {bad_n}", bool(scan) and bad_n == 0)
add("B2", "/scan 정면 거리", f"{lid.get('wall_dist_m')} ± {WALL_TOL} m, 모든 메시지",
    f"{len(scan)}개 중 범위 밖 {bad_r}, 값 없음 {nan_r}"
    + (f", 중앙값 {statistics.median(fr_vals):.4f} m" if fr_vals else ""),
    bool(scan) and bad_r == 0 and nan_r == 0)
add("B2", "/scan frame_id", lid.get("frame"), f"불일치 {bad_f}", bool(scan) and bad_f == 0)
bad_i, ex_i = 0, None
for r in scan:
    if len(r) < 14:
        bad_i += 1
        continue
    n, ni, nf = int(r[7]), int(r[11]), int(r[12])
    if ni not in (0, n) or nf != ni:
        bad_i += 1
        ex_i = ex_i or f"ranges {n}, intensities {ni}, 유한 {nf}, 범위 {r[13]}~{r[14] if len(r) > 14 else ''}"
add("B2", "★ /scan intensities 길이 = ranges 길이(또는 0), 모두 유한", "모든 메시지",
    f"{len(scan)}개 중 불일치 {bad_i}" + (f" (예: {ex_i})" if ex_i else ""), bool(scan) and bad_i == 0)

om = meta.get("odom", {})
odom = rx.get("/odom", [])
bad = []
for r in odom:
    pos = [float(x) for x in r[5:8]]
    q = [float(x) for x in r[8:12]]
    vals = pos + q
    if r[3] != om["frame"] or r[4] != om["child"] or not all(math.isfinite(v) for v in vals) \
            or max(abs(x - y) for x, y in zip(pos, om["position"])) > POS_TOL_ODOM \
            or quat_angle(q, om["orientation_xyzw"]) > ANG_TOL:
        bad.append(r)
fo = first.get("/odom", {})
vel_ok = bool(fo) and all(abs(x - y) <= 1e-9 for x, y in zip(fo.get("linear", []) + fo.get("angular", []),
                                                               list(om["linear"]) + list(om["angular"])))
add("B2", "/odom frame·위치·자세", f"{om.get('frame')}→{om.get('child')}, 위치 ±{POS_TOL_ODOM:g} m, 각도 ±{ANG_TOL:g} rad",
    f"{len(odom)}개 중 불일치 {len(bad)}" + (f" (예: {bad[0][3:12]})" if bad else ""), bool(odom) and not bad)
add("B2", "/odom 속도(첫 메시지)", "넣은 값과 같음(기록)",
    f"{fo.get('linear')} {fo.get('angular')}" if fo else "없음", "기록" if vel_ok else "기록(다름)")
cov_ok = bool(fo) and fo.get("pose_cov_len") == 36 and fo.get("twist_cov_len") == 36
add("B2", "/odom covariance 길이", "36, 36", f"{fo.get('pose_cov_len')}, {fo.get('twist_cov_len')}", cov_ok)

want = list(obs.get("twist_lin", [])) + list(obs.get("twist_ang", []))
tw_rows = [[float(r[k]) for k in ("lx", "ly", "lz", "ax", "ay", "az")] for r in read_rows("twist_rx.csv")]
nonzero = [v for v in tw_rows if any(x != 0.0 for x in v)]
bad = [v for v in nonzero if max(abs(x - y) for x, y in zip(v, want)) > TWIST_TOL]
add("B2", "/cmd_vel 수신값", f"보낸 값 {want}과 차이 ≤ {TWIST_TOL:g}",
    f"바뀐 값 {len(nonzero)}종, 불일치 {len(bad)}" + (f" (예: {bad[0]})" if bad else ""),
    bool(nonzero) and not bad and bool(want))

last = {}
try:
    with open(p("twist_stamped_rx.csv")) as f:
        for r in csv.DictReader(f):
            last[r["attr"]] = json.loads(r["value"])
except (OSError, ValueError):
    pass


def vec3(prefix):
    """범용 구독 노드 출력에서 3성분 벡터 찾기: '...twist:linear'(배열) 또는 '...twist:linear:x/y/z'(숫자 3개)"""
    for k, v in last.items():
        if k.endswith(prefix) and isinstance(v, list) and len(v) == 3:
            return v
    comp = [next((v for k, v in last.items() if k.endswith(f"{prefix}:{c}") and isinstance(v, float)), None)
            for c in "xyz"]
    return comp if None not in comp else None


s_want = list(obs.get("stamped_lin", [])) + list(obs.get("stamped_ang", []))
lin, ang = vec3("twist:linear"), vec3("twist:angular")
frame = next((v for k, v in last.items() if k.endswith("frame_id")), None)
got = (lin or []) + (ang or [])
s_ok = bool(s_want) and len(got) == 6 and max(abs(x - y) for x, y in zip(got, s_want)) <= TWIST_TOL \
    and frame == obs.get("stamped_frame")
skipped = set(meta.get("skipped", []))
if "stamped" not in skipped:
    add("B2", "★ /cmd_vel_stamped(TwistStamped, 범용 구독) 수신값", f"보낸 값 {s_want}·frame {obs.get('stamped_frame')}과 차이 ≤ {TWIST_TOL:g}",
        f"마지막 값 {got or '없음'}, frame {frame!r}, 출력 속성 {len(last)}개", s_ok)
else:
    add("B2", "★ /cmd_vel_stamped(TwistStamped, 범용 구독) 수신값", "이 실행에서는 뺌(--skip, 별도 실행으로 판정)",
        "-", "기록")

# ---------- B3 지속 ----------
n_ticks = len(ticks)
sim_dur = (ticks[-1][1] - ticks[0][1]) if ticks else 0.0
play_wall = 0.0
if ticks:
    play_wall = ticks[-1][0] - ticks[0][0] - ((pause_end - pause_begin) if pause_begin and pause_end else 0.0)
skip = cam.get("frame_skip", 0) + 1
expected = {"/clock": n_ticks, "/tf": n_ticks, "/odom": n_ticks,
            "/left/image_raw": n_ticks / skip, "/right/image_raw": n_ticks / skip,
            "/left/camera_info": n_ticks / skip, "/right/camera_info": n_ticks / skip,
            "/scan": sim_dur * lid.get("scan_rate_hz", lid.get("hz", 10.0))}
add("B3", "재생 시간(실제 시간)", "≥ 600 s", f"{play_wall:.1f} s (sim {sim_dur:.1f} s, 단계 {n_ticks})",
    play_wall >= 600.0 - 1.0)
for t, e in expected.items():
    got = len(tf_rows) if t == "/tf" else len(rx.get(t, []))
    ratio = got / e if e else 0.0
    add("B3", f"{t} 받은 개수", f"발행 추정 {e:.0f}의 {RATIO_MIN:.0%} 이상", f"{got} ({ratio:.1%})", ratio >= RATIO_MIN)
for t in expected:
    rows = tf_rows if t == "/tf" else rx.get(t, [])
    ts = sorted(float(r[1]) for r in rows)
    if len(ts) < 10:
        add("B3", f"{t} 끊김", f"평소 간격 {GAP_FACTOR:g}배 초과 0회", "데이터 부족", False)
        continue
    t0 = ts[0] + 2.0
    keep = [x for x in ts if x >= t0 and not (pause_begin and pause_end and pause_begin - 0.1 <= x <= pause_end + 1.0)]
    dts = [keep[i] - keep[i - 1] for i in range(1, len(keep))
           if not (pause_begin and pause_end and keep[i - 1] < pause_begin <= keep[i])]
    med = statistics.median(dts) if dts else float("nan")
    gaps = [d for d in dts if d > GAP_FACTOR * med]
    hz = (len(keep) - 1) / (keep[-1] - keep[0]) if len(keep) > 1 and keep[-1] > keep[0] else float("nan")
    add("B3", f"{t} 끊김", f"평소 간격 {GAP_FACTOR:g}배 초과 0회 (일시정지 구간 제외)",
        f"{len(gaps)}회 (평소 {med * 1000:.1f} ms, 최대 {max(dts) * 1000 if dts else float('nan'):.1f} ms, 평균 {hz:.1f} Hz)",
        len(gaps) == 0)
code = read_text("isaac_exit_code.txt").strip()
add("B3", "Isaac Sim 종료", "정상 종료(0)", code or "기록 없음", code == "0")


def growth(name, col):
    vals = []
    try:
        with open(p(name)) as f:
            for r in csv.DictReader(f):
                vals.append((float(r["wall_s"]), float(r[col])))
    except (OSError, KeyError, ValueError):
        return None
    if len(vals) < 3 or not ticks:
        return None
    ref_t = ticks[0][0] + 60.0
    ref = min(vals, key=lambda v: abs(v[0] - ref_t))[1]
    end = vals[-1][1] if vals[-1][0] <= ticks[-1][0] + 1 else min(vals, key=lambda v: abs(v[0] - ticks[-1][0]))[1]
    return ref, end


for name, col, unit, label in (("gpu_mem.csv", "used_mib", "MiB", "GPU 메모리"), ("isaac_rss.csv", "rss_mib", "MiB", "Isaac Sim 프로세스 메모리")):
    g = growth(name, col)
    if g is None:
        add("B3", f"{label} 증가", f"1분 대비 {MEM_GROWTH_FLAG:.0%} 초과 시 표시", "기록 없음", "기록")
    else:
        ref, end = g
        rate = (end - ref) / ref if ref else float("nan")
        add("B3", f"{label} 증가", f"1분 대비 {MEM_GROWTH_FLAG:.0%} 초과 시 표시",
            f"{ref:.0f} → {end:.0f} {unit} ({rate:+.1%})", "표시" if rate > MEM_GROWTH_FLAG else "기록")

# ---------- 출력 ----------
cond_name = {"A": "A: Lyrical source (native Lyrical)", "A2": "A′: Lyrical source + empy·lark 링크 (native Lyrical)",
             "B": "B: source 없음 (Isaac Sim 내장 Jazzy)"}[a.cond]
lines = [f"# 브리지 검증 결과 — {cond_name}", "", f"결과 폴더: `{os.path.basename(os.path.abspath(D))}`", ""]
SKIP_KEY = {"camera_info": "camera_info", "scan": "/scan"}
for i, r in enumerate(results):
    if any(k in skipped and key in r[1] for k, key in SKIP_KEY.items()):
        results[i] = (r[0], r[1], r[2], r[3], "기록(이 실행에서 뺌)")
if skipped:
    lines += [f"이 실행에서 뺀 것: {', '.join(sorted(skipped))}", ""]
for stage in ("B0", "B1", "B2", "B3"):
    rs = [r for r in results if r[0] == stage]
    judged = [r for r in rs if r[4] in ("통과", "실패")]
    verdict = "통과" if judged and all(r[4] == "통과" for r in judged) else "실패"
    lines += [f"## {stage} — {verdict}", "", "| 항목 | 기준 | 측정 | 판정 |", "|---|---|---|---|"]
    lines += [f"| {r[1]} | {r[2]} | {str(r[3]).replace('|', '/')} | {r[4]} |" for r in rs]
    lines.append("")
if err_lines:
    lines += ["## ROS 관련 오류 줄 (최대 20줄)", "", "```"] + err_lines[:20] + ["```", ""]
if LYR and internal:
    lines += ["## 함께 불러온 Isaac Sim 내장 ROS 라이브러리", ""] + [f"- `{x}`" for x in internal] + [""]
if pubinfo:
    lines += ["## 발행자 QoS (관찰 노드가 본 값)", "", "| 토픽 | 노드 | reliability | durability | depth |", "|---|---|---|---|---|"]
    for t, lst in pubinfo.items():
        for i in lst:
            lines.append(f"| {t} | {i['node']} | {i['reliability']} | {i['durability']} | {i['depth']} |")
    lines.append("")
with open(p("summary.md"), "w") as f:
    f.write("\n".join(lines))
print("\n".join(lines))
