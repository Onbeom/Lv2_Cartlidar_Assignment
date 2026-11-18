"""MuJoCo 검사·주행 시험과 Isaac 비교 전체를 HTML 보고서 하나로 만든다(프레임·데이터 내장).

구성: 요약 → 작업 흐름 → 로딩 검사 → 주행 장면 → 첫 계획과 준비 확인에서 찾은 문제 → 진단 → armature 출처 →
영상 비교(armature 0 vs 0.0163) → 공식 결과(K1~K4·dt·민감도·그래프) → Isaac v10·v11·v12 → G3·D6 진단 →
P7 기준 결정 자료 → 8~15절 주행 방식 시험(v13·S0·V1~V9, 표는 build_report_nav.py가 verdict_stow.json에서 채움) →
가설 기록 → 바로잡은 것 → 한계·출처.
수치는 결과 폴더(verdict.json·log.csv·summary.json)에서 읽는다. 준비 확인 진단 값(공식 아님)은 실행기록.md 3-1절과 같은 값을 넣는다.

실행(PC): python3 drive/build_report.py drive/results/261023_1851 --isaac ../phys_tests/results --out drive/results/261023_1851/report.html
"""
import argparse
import base64
import csv
import glob
import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np

import build_report_nav

ap = argparse.ArgumentParser()
ap.add_argument("results", type=Path)
ap.add_argument("--isaac", type=Path, required=True, help="Isaac phys_tests/results 폴더")
ap.add_argument("--out", type=Path, required=True)
args = ap.parse_args()
R = args.results.resolve()
I = args.isaac.resolve()
FPS = 10


# ---------- 읽기 ----------
def frames(mp4, width=400):
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(mp4), "-vf", f"fps={FPS},scale={width}:-2",
                        "-q:v", "7", f"{tmp}/%04d.jpg"], check=True)
        return [base64.b64encode(p.read_bytes()).decode() for p in sorted(Path(tmp).glob("*.jpg"))]


def load_log(path):
    with Path(path).open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    skip = ("segment", "contact_pairs_minsep_mm")
    return {k: np.array([float(r[k]) if r[k] not in ("True", "False", "") else float(r[k] == "True") for r in rows])
            for k in rows[0] if k not in skip}


def readouts(c, n_frames):
    t = c["t_s"]
    dq = np.abs(np.stack([c[k] for k in c if k.startswith("dq_Wheel_")], axis=1)).max(axis=1)
    yaw = np.degrees(np.unwrap(c["yaw_rad"]))
    out = []
    for i in range(n_frames):
        j = int(np.argmin(np.abs(t - i / FPS)))
        out.append([round(float(dq[j]), 2), round(float(math.hypot(c["base_x_m"][j] - c["base_x_m"][0],
                                                                       c["base_y_m"][j] - c["base_y_m"][0])), 4),
                    round(float(yaw[j] - yaw[0]), 2)])
    return out


def series(c, fn, win=0.02):
    t, v = c["t_s"], fn(c)
    out = []
    for a in np.arange(0.0, t[-1] + 1e-9, win):
        m = (t >= a) & (t < a + win)
        if m.any():
            k = np.argmax(np.abs(v[m]))
            out.append([round(float(a + win), 3), round(float(v[m][k]), 3)])
    return out


def pivot(path):
    """회전 구간(1 s~)을 차체 좌표의 고정점 둘레 강체 회전으로 맞춘 회전 중심(mm)과 RMS(mm)."""
    c = load_log(path)
    t = c["t_s"]
    i0 = int(np.argmin(np.abs(t - 1.0)))
    yaw = np.unwrap(c["yaw_rad"])
    rot = lambda a: np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    idx = np.where(t >= 1.0)[0][::5]
    A = np.vstack([rot(yaw[j]) - rot(yaw[i0]) for j in idx])
    b = np.concatenate([-np.array([c["base_x_m"][j] - c["base_x_m"][i0], c["base_y_m"][j] - c["base_y_m"][i0]]) for j in idx])
    r = np.linalg.lstsq(A, b, rcond=None)[0]
    return 1000 * r, 1000 * float(np.sqrt(np.mean((A @ r - b) ** 2)))


def one(pattern):
    hits = sorted(glob.glob(str(I / pattern)))
    if not hits:
        raise FileNotFoundError(pattern)
    return Path(hits[-1])


def jv(folder):
    return json.loads((Path(folder) / "verdict.json").read_text(encoding="utf-8"))


def mv(name):
    return json.loads((R / name / "verdict.json").read_text(encoding="utf-8"))["results"]


# ---------- MuJoCo 영상·그래프 ----------
summary = json.loads((R / "summary.json").read_text(encoding="utf-8"))
by_dt, by_wa = summary["by_dt"], summary["by_wheel_armature_dtB"]
scen = {}
for key, test, mp4 in (("M5", "M5", "videos/raw_M5_fixed_wa{}.mp4"), ("M7", "M7", "M7_dtB_wa{}/video.mp4"),
                       ("M6", "M6", "M6_dtB_wa{}/video.mp4")):
    scen[key] = {}
    for side, wa in (("a0", "0"), ("a1", "0.0163")):
        f = frames(R / mp4.format(wa))
        scen[key][side] = {"f": f, "m": readouts(load_log(R / f"{test}_dtB_wa{wa}" / "log.csv"), len(f))}
wheel_speed = lambda c: np.abs(np.stack([c[k] for k in c if k.startswith("dq_Wheel_")], axis=1)).max(axis=1)
yaw_deg = lambda c: np.degrees(np.unwrap(c["yaw_rad"]) - c["yaw_rad"][0])
charts = {"speed": {s: series(load_log(R / f"M5_dtB_wa{w}" / "log.csv"), wheel_speed) for s, w in (("a0", "0"), ("a1", "0.0163"))},
          "yaw": {s: series(load_log(R / f"M7_dtB_wa{w}" / "log.csv"), yaw_deg) for s, w in (("a0", "0"), ("a1", "0.0163"))}}

# ---------- Isaac ----------
def isaac_row(tag):
    """tag: "" (v10 = 2026-10-23 13:00~13:04 run_all_v10, 태그 없음) 또는 'v11_'·'v12_'. 시험별 판정과 대표 값."""
    pat = (lambda t: f"{t}_261023_130*") if tag == "" else (lambda t: f"{t}_{tag}*")
    out = {}
    for t in ("P1", "P2", "P3", "P4on", "P4off", "P5", "P6", "P7"):
        v = jv(one(pat(t)))
        if t == "P6":
            out["P6"] = (v["P6"]["verdict"], v["P6"]["measured"]["forward_m"])
            out["P8"] = (v["P8"]["verdict"], v["P8"]["measured"]["참고_위치차분_정지시간_s"])
            continue
        m = v["measured"]
        val = None
        if t == "P2":
            val = max(x["abs_err"] for k, x in m.items() if isinstance(x, dict) and "abs_err" in x and "gripper" not in k)
        elif t in ("P4on", "P4off"):
            val = max(list(m["참고_관절속도_위치차분_최대"].values())[:5])
        elif t == "P5":
            val = max(m["바퀴속도_위치차분_최대_rad_s"].values())
        elif t == "P7":  # 기준별로 판정을 나눠 보인다(회전 ≥ 1°, 중심 이동 ≤ 30 mm)
            ck = v["checks"]
            ok = lambda key: "성공" if next(b for a, b in ck.items() if key in a) else "실패"
            out["P7"] = (ok("1°"), m["yaw_change_deg(기록만)"])
            out["P7c"] = (ok("중심 이동"), m["center_move_max_m"] * 1000)
            continue
        out[t] = (v["verdict"], val)
    return out


ISA = {"v10": isaac_row(""), "v11": isaac_row("v11_"), "v12": isaac_row("v12_")}
ISA_LABEL = [("P1", "P1 정지", None, ""), ("P2", "P2 관절 하나씩 (최대 오차)", 4, "rad"), ("P3", "P3 집게 mimic", None, ""),
             ("P4on", "P4 켬 자기충돌 (팔 최대 속도)", 3, "rad/s"), ("P4off", "P4 끔 (팔 최대 속도)", 3, "rad/s"),
             ("P5", "P5 바퀴 속도 한계", 3, "rad/s"), ("P6", "P6 직진 전진", 4, "m"), ("P7", "P7 회전량 (기준 ≥ 1°)", 2, "°"),
             ("P7c", "P7 중심 이동 (기준 ≤ 30 mm)", 1, "mm"), ("P8", "P8 정지 시간", 3, "s")]


def cell(vv, d):
    verdict, val = vv
    txt = "" if val is None or d is None else f"{val:.{d}f}"
    cls = "ok" if verdict == "성공" else "ng"
    return f'<td class="n {cls}">{verdict}{" · " + txt if txt else ""}</td>'


isaac_rows = "".join(f"<tr><td>{lab}{' (' + u + ')' if u else ''}</td>" + "".join(cell(ISA[v][k], d) for v in ("v10", "v11", "v12")) + "</tr>"
                     for k, lab, d, u in ISA_LABEL)

g3 = []
for lab, pat2, pat4 in (("G3-1 피드포워드, kp 400·kd 24", "P2_G3_1_*", "P4off_G3_1_*"),) + tuple(
        (f"G3-2 kp {kp}·kd {kd}", f"P2_G3_2_kp{kp}_kd{kd}_*", f"P4off_G3_2_kp{kp}_kd{kd}_*")
        for kp in (400, 800) for kd in (24, 36, 48)):
    v4 = jv(one(pat4))
    s4 = v4["measured"]["참고_관절속도_위치차분_최대"]["RMX52_Shoulder"]
    hits2 = sorted(glob.glob(str(I / pat2)))
    p2 = jv(hits2[-1])["verdict"] if hits2 else "안 돌림"
    g3.append(f'<tr><td>{lab}</td><td class="n {"ok" if v4["verdict"] == "성공" else "ng"}">{v4["verdict"]} · {s4:.2f}</td>'
              f'<td class="n {"ok" if p2 == "성공" else ("ng" if p2 == "실패" else "")}">{p2}</td></tr>')
g3_rows = "".join(g3)

isaac_static = load_log(one("P1_D6_1_*") / "log.csv")
tI = isaac_static["t_s"] >= 1.0
fI = {k: float(np.mean(isaac_static[f"Fn_wheel_{k}_N"][tI])) for k in ("front_left", "front_right", "rear_left", "rear_right")}
fM = {k.lower().replace("wheel_revolute_", ""): v for k, v in mv("M1_dtB_wa0.0163")["K1"]["measured"]["mean_Fn_each_wheel_N"].items()}
load_rows = "".join(f'<tr><td>{n}</td><td class="n">{fI[k]:.2f}</td><td class="n">{fM[k]:.2f}</td></tr>' for k, n in (
    ("front_left", "앞 왼쪽"), ("front_right", "앞 오른쪽"), ("rear_left", "뒤 왼쪽"), ("rear_right", "뒤 오른쪽")))
load_rows += (f'<tr><td>앞 합 : 뒤 합</td><td class="n">{fI["front_left"] + fI["front_right"]:.2f} : {fI["rear_left"] + fI["rear_right"]:.2f}</td>'
              f'<td class="n">{fM["front_left"] + fM["front_right"]:.2f} : {fM["rear_left"] + fM["rear_right"]:.2f}</td></tr>')
pv = {"Isaac v11": pivot(one("P7_v11_*") / "log.csv"), "Isaac v11, 팔 armature 0 (D6-2)": pivot(one("P7_D6_2_*") / "log.csv"),
      "Isaac v12": pivot(one("P7_v12_*") / "log.csv"), "MuJoCo dt 1/120": pivot(R / "M7_dtA_wa0.0163" / "log.csv"),
      "MuJoCo dt 0.002": pivot(R / "M7_dtB_wa0.0163" / "log.csv")}
pivot_rows = "".join(f'<tr><td>{k}</td><td class="n">{p[0]:.1f}</td><td class="n">{p[1]:.1f}</td><td class="n">{rms:.2f}</td></tr>'
                     for k, (p, rms) in pv.items())

# ---------- 숫자 채우기 ----------
v_m1, v_m6 = mv("M1_dtB_wa0.0163"), mv("M6_dtB_wa0.0163")
K = {"K_WEIGHT": f"{v_m1['K1']['measured']['weight_N(질량×g)']:.2f}", "K_FN": f"{v_m1['K1']['measured']['mean_Fn_total_N']:.2f}",
     "K1": f"{v_m1['K1']['measured']['relative_error'] * 100:.4f}", "K2": f"{v_m1['K2']['measured']['mean_|CoP−CoM|_xy_m'] * 1000:.4f}",
     "K3M6": f"{summary['K3']['measured']['M6 전진 차이 (%)']:.3f}", "K3M7": f"{summary['K3']['measured']['M7 회전 차이 (%)']:.2f}",
     "K4": f"{v_m6['K4']['measured']['slip_ratio'] * 100:.3f}"}


def f(v, d=3):
    return "—" if v is None else f"{v:.{d}f}"


def row(label, vals, d):
    return "<tr><td>" + label + "</td>" + "".join(f'<td class="n">{f(v, d)}</td>' for v in vals) + "</tr>"


dt_rows = "".join(row(lab, [by_dt[x].get(lab) for x in "ABC"], d) for lab, d in (
    ("M5 최대 바퀴 속도 (rad/s)", 3), ("M6 전진 (m)", 4), ("M6 후진 (m)", 4), ("M7 회전 (°)", 2), ("M7 중심 이동 (mm)", 1),
    ("M8 정지 시간 (s)", 3), ("K4 미끄럼률 (%)", 3)))
wa_rows = "".join(row(lab, [by_wa[w].get(lab) for w in ("0", "0.00163", "0.0163", "0.163")], d) for lab, d in (
    ("M5 최대 바퀴 속도 (rad/s)", 2), ("M6 전진 (m)", 4), ("M7 회전 (°)", 2), ("M7 중심 이동 (mm)", 1), ("M8 정지 시간 (s)", 3)))
n_runs = len(summary["verdicts"])
n_items = sum(len(r) for r in summary["verdicts"].values())
n_fail = [f"{n} {k}" for n, r in summary["verdicts"].items() for k, v in r.items() if v != "성공"]

SC = {"M5": {"cond": "바퀴 목표 14.66 rad/s(한계의 2배)를 명령, 제어기가 7.33 rad/s로 자름. 3 s, dt 0.002 s, 마찰 0.3. 고정 카메라",
             "a0": (f"바퀴 속도가 위치 차분으로 최대 {by_wa['0']['M5 최대 바퀴 속도 (rad/s)']:.1f} rad/s까지 튐. 직진 명령인데 차체가 "
                    f"{scen['M5']['a0']['m'][-1][2]:.0f}° 돌고 {scen['M5']['a0']['m'][-1][1]:.2f} m만 감 — 실패"),
             "a1": f"최대 {by_wa['0.0163']['M5 최대 바퀴 속도 (rad/s)']:.3f} rad/s, 한계 7.40 안 — 성공"},
      "M7": {"cond": "왼쪽 바퀴 −1, 오른쪽 +1 rad/s, 4 s, dt 0.002 s, 마찰 0.3. 차체를 따라가는 카메라",
             "a0": f"4 s에 {by_wa['0']['M7 회전 (°)']:.1f}° — dt를 바꾸면 값이 달라짐(수렴 안 함)",
             "a1": f"4 s에 {by_wa['0.0163']['M7 회전 (°)']:.1f}° — dt 1/120~0.001에서 같은 값"},
      "M6": {"cond": "0.05 m/s 4 s → 정지 2 s → −0.05 m/s 4 s, dt 0.002 s. 바퀴 토크가 상한 아래라 둘 다 정상",
             "a0": f"전진 {by_wa['0']['M6 전진 (m)']:.4f} m, 정지 {by_wa['0']['M8 정지 시간 (s)']:.3f} s",
             "a1": f"전진 {by_wa['0.0163']['M6 전진 (m)']:.4f} m, 정지 {by_wa['0.0163']['M8 정지 시간 (s)']:.3f} s"}}

NAV = build_report_nav.build(R.parent, I, Path(__file__).resolve().parents[1] / "stow" / "S0_yaw0.png")

html = Path(__file__).with_name("report_template.html").read_text(encoding="utf-8")
for k, v in {"/*DATA*/": json.dumps(scen, separators=(",", ":")), "/*SC*/": json.dumps(SC, ensure_ascii=False),
             "/*CHARTS*/": json.dumps(charts, separators=(",", ":")), "{{DT_ROWS}}": dt_rows, "{{WA_ROWS}}": wa_rows,
             "{{ISAAC_ROWS}}": isaac_rows, "{{G3_ROWS}}": g3_rows, "{{LOAD_ROWS}}": load_rows, "{{PIVOT_ROWS}}": pivot_rows,
             "{{RUN}}": R.name, "{{N_RUNS}}": str(n_runs), "{{N_ITEMS}}": str(n_items),
             "{{N_FAIL}}": ", ".join(n_fail) if n_fail else "없음", **{"{{" + k + "}}": v for k, v in K.items()},
             **{"{{" + k + "}}": v for k, v in NAV.items()}}.items():
    html = html.replace(k, v)
assert "{{" not in html, html[html.index("{{"):html.index("{{") + 40]
args.out.write_text(html, encoding="utf-8")
print(f"{args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
