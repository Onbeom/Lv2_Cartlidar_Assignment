"""진단 결과 정리 → diag_summary.md (검증계획 12장, 사용자 승인 2026-10-25). 표준 라이브러리만.

사용: python3 analyze_diag.py <diag 폴더> [다른 diag 폴더 ...]   (여러 개면 조건 비교표도 쓴다)
손실 사건 = 한 토픽에서 stamp 간격 > 0.1 s(3프레임). 사건 시각 = 간격 뒤 첫 메시지를 받은 실제 시각.
"""

import csv
import glob
import os
import statistics
import sys

GAP = 0.1          # [s] 손실 사건 기준
FRAME = 1 / 30     # [s] 영상 주기
WIN = 1.0          # [s] 사건 전후로 RcvbufErrors 증가를 보는 폭
TOPICS = ["/top_camera_front/color/image_raw", "/top_camera_rear/color/image_raw", "/arm_camera/image_raw",
          "/top_camera_front/aligned_depth_to_color/image_raw", "/top_camera_rear/aligned_depth_to_color/image_raw"]


def read(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def receivers(d):
    l2 = sorted(glob.glob(os.path.join(d, "L2_*")))
    return {"R1 Python 관찰 노드": read(os.path.join(l2[-1], "rx.csv")) if l2 else [],
            "R2 최소 Python": read(os.path.join(d, "rx_min.csv")),
            "R3 최소 C++": read(os.path.join(d, "rx_cpp.csv"))}


def events(rows, topic):
    m = sorted((float(r["stamp_s"]), float(r["rx_wall_s"])) for r in rows if r["topic"] == topic)
    ev = [(b[1], b[0] - a[0]) for a, b in zip(m, m[1:]) if b[0] - a[0] > GAP]
    lost = sum(round(g / FRAME) - 1 for _, g in ev)
    return len(m), ev, lost, max((b[0] - a[0] for a, b in zip(m, m[1:])), default=0.0)


def analyze(d):
    mon = [{k: (float(v) if v else None) for k, v in r.items()} for r in read(os.path.join(d, "sysmon.csv"))]
    rcv = [(r["wall_s"], r["rcvbuf_errors"]) for r in mon]

    def rcv_delta(t0, t1):
        a = [v for t, v in rcv if t <= t0]
        b = [v for t, v in rcv if t >= t1]
        return (b[0] - a[-1]) if (a and b) else None

    rx = receivers(d)
    res = {"dir": os.path.basename(d), "rmem": open(os.path.join(d, "rmem_max.txt")).read().strip()
           if os.path.exists(os.path.join(d, "rmem_max.txt")) else "?",
           "rcv_total": (rcv[-1][1] - rcv[0][1]) if len(rcv) > 1 else None, "recv": {}}
    for name, rows in rx.items():
        per, all_ev = {}, []
        for t in TOPICS:
            n, ev, lost, mg = events(rows, t)
            per[t] = (n, len(ev), lost, mg)
            all_ev += [(w, g, t) for w, g in ev]
        with_rcv = sum(1 for w, _, _ in all_ev if (rcv_delta(w - WIN, w + WIN) or 0) > 0)
        res["recv"][name] = {"per": per, "events": sorted(all_ev), "with_rcv": with_rcv}
    for key, col in (("R1 Python 관찰 노드", "cpu_R1_observer"), ("R2 최소 Python", "cpu_R2_rx_min"),
                     ("R3 최소 C++", "cpu_R3_rx_cpp")):
        v = [r[col] for r in mon if r.get(col) is not None]
        res["recv"][key]["cpu"] = (statistics.mean(v), max(v)) if v else None
    return res


def verdicts(r):
    ev = {k: len(v["events"]) for k, v in r["recv"].items()}
    r1, r2, r3 = ev["R1 Python 관찰 노드"], ev["R2 최소 Python"], ev["R3 최소 C++"]
    total = r1 + r2 + r3
    rc = sum(v["with_rcv"] for v in r["recv"].values())
    return [
        ("E2 Python 처리 원인", "R1만 손실, R2·R3 0", f"R1 {r1}, R2 {r2}, R3 {r3}",
         "해당" if (r1 > 0 and r2 == 0 and r3 == 0) else ("해당 없음" if total else "판단 불가(손실 없음)")),
        ("E3 C++ 수신 영향", "R3 손실 사건 수·최대 간격", f"R3 사건 {r3}, 최대 간격 "
         f"{max(p[3] for p in r['recv']['R3 최소 C++']['per'].values()) * 1000:.0f} ms", "기록"),
        ("E4 송신 쪽 원인", "세 수신기가 같은 시점에 잃고 RcvbufErrors 증가 0",
         f"사건 중 RcvbufErrors 증가와 겹친 것 {rc}/{total}",
         "해당" if (total and r1 and r2 and r3 and rc == 0) else ("해당 없음" if total else "판단 불가(손실 없음)")),
        ("E1 수신 버퍼(이 조건)", "손실 시점 ±1 s에 RcvbufErrors 증가", f"{rc}/{total}, 실행 중 증가 합 {r['rcv_total']}",
         "D2와 비교해 판정" if total else "판단 불가(손실 없음)"),
    ]


def render(rs):
    lines = ["# 받는 쪽 영상 손실 진단 결과 (검증계획 12장, 모의)", ""]
    for r in rs:
        lines += [f"## {r['dir']} (rmem_max {r['rmem']}, 실행 중 RcvbufErrors 증가 {r['rcv_total']})", "",
                  "| 수신기 | 토픽 | 받은 개수 | 손실 사건(> 0.1 s) | 잃은 장 수(추정) | 최대 간격 |", "|---|---|---|---|---|---|"]
        for name, v in r["recv"].items():
            for t, (n, ne, lost, mg) in v["per"].items():
                lines.append(f"| {name} | {t} | {n} | {ne} | {lost} | {mg * 1000:.0f} ms |")
        lines += ["", "| 수신기 | CPU 평균·최대 (%) | 사건 중 RcvbufErrors 증가와 겹친 것 |", "|---|---|---|"]
        for name, v in r["recv"].items():
            c = v.get("cpu")
            lines.append(f"| {name} | {f'{c[0]:.1f} / {c[1]:.1f}' if c else '기록 없음'} | {v['with_rcv']}/{len(v['events'])} |")
        lines += ["", "| 판정 | 기준 | 측정 | 결과 |", "|---|---|---|---|"]
        lines += [f"| {a} | {b} | {c} | {e} |" for a, b, c, e in verdicts(r)]
        lines.append("")
    if len(rs) >= 2:
        lines += ["## 조건 비교 (E1 수신 버퍼 원인: D1에서 RcvbufErrors와 함께 잃고, D2에서 손실·증가 0)", "",
                  "| 폴더 | rmem_max | 손실 사건 R1/R2/R3 | RcvbufErrors 증가 |", "|---|---|---|---|"]
        for r in rs:
            e = [len(v["events"]) for v in r["recv"].values()]
            lines.append(f"| {r['dir']} | {r['rmem']} | {e[0]}/{e[1]}/{e[2]} | {r['rcv_total']} |")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    rs = [analyze(d) for d in sys.argv[1:]]
    txt = render(rs)
    with open(os.path.join(sys.argv[1], "diag_summary.md"), "w") as f:
        f.write(txt)
    print(txt)
