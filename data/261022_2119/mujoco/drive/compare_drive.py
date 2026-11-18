"""run_all.sh 결과 폴더 하나를 모아 summary.md·summary.json을 쓴다.

- 시험별 판정 표(dt A·B·C, 바퀴 armature별)
- K3: dt 수렴(기준 armature에서 B 0.002와 C 0.001 비교). M6 전진 거리 차이 ≤ 1%, M7 회전량 차이 ≤ 5%(2026-10-23 사용자 승인)
- 바퀴 armature 민감도(dt B): M5 최대 바퀴 속도, M6 전진·M8 정지 시간, M7 회전량·중심 이동
- Isaac v10 결과(검증계획 14장, 같은 시험 정의)를 참고로 나란히 적는다. 엔진 비교는 판정하지 않는다.

실행(PC): python3 drive/compare_drive.py drive/results/<YYMMDD_HHMM>
"""
import json
import sys
from pathlib import Path

REF_WA = "0.0163"
ISAAC_V10 = {"M6 전진 (m)": 0.1993, "M6 후진 (m)": 0.2006, "M7 회전 (°)": 1.521, "M7 중심 이동 (mm)": 5.8,
             "M5 최대 바퀴 속도 (rad/s)": 7.327, "M8 정지 시간 (s)": 0.117}


def load(folder):
    runs = {}
    for v in sorted(Path(folder).glob("*/verdict.json")):
        runs[v.parent.name] = json.loads(v.read_text(encoding="utf-8"))
    return runs


def key(test, dt, wa):
    return f"{test}_dt{dt}_wa{wa}"


def metrics(runs, dt, wa):
    out = {}
    r = runs.get(key("M5", dt, wa))
    if r:
        out["M5 최대 바퀴 속도 (rad/s)"] = max(r["results"]["M5"]["measured"]["바퀴속도_위치차분_최대_rad_s"].values())
    r = runs.get(key("M6", dt, wa))
    if r:
        m6, m8 = r["results"]["M6"]["measured"], r["results"]["M8"]["measured"]
        out.update({"M6 전진 (m)": m6["forward_m"], "M6 후진 (m)": m6["backward_m"], "M6 옆 이탈 (mm)": m6["side_max_m"] * 1000,
                    "M6 yaw 변화 (°)": m6["yaw_change_max_deg"], "M8 정지 시간 (s)": m8["stop_time_after_backward_s"],
                    "K4 미끄럼률 (%)": r["results"]["K4"]["measured"]["slip_ratio"] * 100})
    r = runs.get(key("M7", dt, wa))
    if r:
        m7 = r["results"]["M7"]["measured"]
        out.update({"M7 회전 (°)": m7["yaw_change_deg(기록만)"], "M7 중심 이동 (mm)": m7["center_move_max_m"] * 1000,
                    "M7 미끄럼 없는 이상 회전 (°, 참고)": m7["ideal_no_slip_deg(참고)"]})
    return out


def fmt(v):
    if v is None:
        return "없음"
    if isinstance(v, float):
        return f"{v:.4g}"
    return str(v)


def main(folder):
    folder = Path(folder)
    runs = load(folder)
    lines = [f"# MuJoCo 주행 시험 요약 ({folder.name}, 모의)", ""]

    lines += ["## 1. 판정", "", "| 실행 | 항목 | 판정 | 실패한 기준 |", "|---|---|---|---|"]
    for name, r in runs.items():
        for k, v in r["results"].items():
            bad = "; ".join(a for a, b in v["checks"].items() if not b)
            lines.append(f"| {name} | {k} | {v['verdict']} | {bad} |")

    b, c = metrics(runs, "B", REF_WA), metrics(runs, "C", REF_WA)
    k3 = {}
    if "M6 전진 (m)" in b and "M6 전진 (m)" in c:
        k3["M6 전진 차이 (%)"] = abs(b["M6 전진 (m)"] - c["M6 전진 (m)"]) / abs(c["M6 전진 (m)"]) * 100
    if "M7 회전 (°)" in b and "M7 회전 (°)" in c:
        k3["M7 회전 차이 (%)"] = abs(b["M7 회전 (°)"] - c["M7 회전 (°)"]) / abs(c["M7 회전 (°)"]) * 100
    k3_ok = bool(k3) and k3.get("M6 전진 차이 (%)", 99) <= 1.0 and k3.get("M7 회전 차이 (%)", 99) <= 5.0
    lines += ["", "## 2. K3 dt 수렴 (바퀴 armature 0.0163, B 0.002 vs C 0.001)", "",
              "| 항목 | 값 | 기준 |", "|---|---|---|"]
    lines += [f"| {k} | {fmt(v)} | {'≤ 1%' if 'M6' in k else '≤ 5%'} |" for k, v in k3.items()]
    lines += ["", f"K3 판정: **{'성공' if k3_ok else '실패'}**"]

    cols = [("A", REF_WA), ("B", REF_WA), ("C", REF_WA)]
    names = sorted({k for col in cols for k in metrics(runs, *col)})
    lines += ["", "## 3. dt별 값 (바퀴 armature 0.0163) + Isaac v10 참고", "",
              "| 항목 | A 1/120 | B 0.002 | C 0.001 | Isaac v10 (dt 1/120, 참고) |", "|---|---|---|---|---|"]
    for n in names:
        lines.append(f"| {n} | " + " | ".join(fmt(metrics(runs, *col).get(n)) for col in cols) + f" | {fmt(ISAAC_V10.get(n))} |")

    was = ["0", "0.00163", REF_WA, "0.163"]
    names = sorted({k for w in was for k in metrics(runs, "B", w)})
    lines += ["", "## 4. 바퀴 armature 민감도 (dt B 0.002)", "",
              "| 항목 | 0 (대조) | 0.00163 (×0.1) | 0.0163 (기준) | 0.163 (×10) |", "|---|---|---|---|---|"]
    for n in names:
        lines.append(f"| {n} | " + " | ".join(fmt(metrics(runs, "B", w).get(n)) for w in was) + " |")

    (folder / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (folder / "summary.json").write_text(json.dumps(
        {"K3": {"measured": k3, "verdict": "성공" if k3_ok else "실패"},
         "by_dt": {d: metrics(runs, d, REF_WA) for d in "ABC"},
         "by_wheel_armature_dtB": {w: metrics(runs, "B", w) for w in was},
         "isaac_v10_reference": ISAAC_V10,
         "verdicts": {n: {k: v["verdict"] for k, v in r["results"].items()} for n, r in runs.items()}},
        indent=2, ensure_ascii=False), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main(sys.argv[1])
