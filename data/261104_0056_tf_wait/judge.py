"""TF 기다림 판정 J1~J4 (PC). python3 judge.py <h 폴더>  → judge.json, 표 출력
처리율 = /perception/pucks 서로 다른 stamp 수(퍽 수 0 포함) / 시간. 결과율 = 퍽 있는(n>0) 것만.
J1 켬 정확 TF 비율 ≥ 0.99 (위상 6 개 모두) · J2 처리율 켬 ≥ 끔 × 0.95 · J3 지연 중앙 (켬 − 끔) ≤ 34 ms, 켬 p95 ≤ 80 ms
J4 (켬, 15 s 에 TF 멈춤) 멈춘 뒤 1 s 빼고 처리율 ≥ 멈추기 전 × 0.90
"""
import json
import sys
from pathlib import Path

import numpy as np

H = Path(sys.argv[1])
PH = ["0.003", "0.0086", "0.0142", "0.0198", "0.0254", "0.031"]


def load(name, stop=None):
    d = H / name
    a = json.loads((d / "analysis.json").read_text())
    h = json.loads((d / "harness.json").read_text())
    t0, dur = h["t0"], h["duration"]
    L = [json.loads(x) for x in (d / "pucks.jsonl").read_text().split("\n") if x]
    st = np.array(sorted({p["stamp"] for p in L}))
    a["proc_hz"] = round(len(st) / dur, 2)
    if stop is not None:
        pos = np.array(sorted({p["stamp"] for p in L if p["n"] > 0}))
        b, c = st[st - t0 < stop], st[st - t0 >= stop + 1.0]
        a["stop"] = {"proc_before": round(len(b) / stop, 2), "proc_after": round(len(c) / (dur - stop - 1.0), 2),
                     "result_before": round(len(pos[pos - t0 < stop]) / stop, 2),
                     "result_after": round(len(pos[pos - t0 >= stop + 1.0]) / (dur - stop - 1.0), 2)}
    return a


rows, ok = [], {"J1": True, "J2": True, "J3": True}
for p in PH:
    on, off = load(f"j_p{p}_w0.05"), load(f"j_p{p}_w0.0")
    j1 = on["exact_tf_ratio"] >= 0.99
    j2 = on["proc_hz"] >= off["proc_hz"] * 0.95
    dl = on["latency_ms"]["median"] - off["latency_ms"]["median"]
    j3 = dl <= 34 and on["latency_ms"]["p95"] <= 80
    ok["J1"] &= j1; ok["J2"] &= j2; ok["J3"] &= j3
    rows.append({"phase": p, "exact_on": on["exact_tf_ratio"], "exact_off": off["exact_tf_ratio"], "proc_on": on["proc_hz"],
                 "proc_off": off["proc_hz"], "lat_on": on["latency_ms"], "lat_off": off["latency_ms"], "dlat_med": round(dl, 1),
                 "picks_on": on["picks_last"], "J1": j1, "J2": j2, "J3": j3})
s_on, s_off = load("j_stop_w0.05", 15.0)["stop"], load("j_stop_w0.0", 15.0)["stop"]
ok["J4"] = s_on["proc_after"] >= s_on["proc_before"] * 0.90
out = {"rows": rows, "stop_on": s_on, "stop_off": s_off, "pass": ok}
(H / "judge.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
print("| 위상 [s] | 정확 TF 켬 · 끔 | 처리율 켬 · 끔 [Hz] | 지연 중앙 켬 · 끔 [ms] | 켬 p95 | J1 · J2 · J3 |")
for r in rows:
    print(f"| {r['phase']} | {r['exact_on']} · {r['exact_off']} | {r['proc_on']} · {r['proc_off']} | {r['lat_on']['median']} · {r['lat_off']['median']} | "
          f"{r['lat_on']['p95']} | {'O' if r['J1'] else 'X'} {'O' if r['J2'] else 'X'} {'O' if r['J3'] else 'X'} |")
print("J4 켬", s_on, "· 끔(참고)", s_off)
print("통과", ok)
