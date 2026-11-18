"""tf_lag_harness 결과 세기 (PC). python3 analyze.py <run 폴더> [--tf-stop 초]
- 도구: 영상 낸 순간 최신 TF 가 영상 stamp 보다 뒤진 시간(중앙 · 최대)
- 노드 로그 마지막 '촬영 시각 TF 로 고른 장' 줄: 가장 새 장 · 한 장 앞 · 기다림 · 최근 TF 로 대신
- 새 결과(퍽이 있는 /perception/pucks 의 서로 다른 stamp): 개수 · 초당, 지연(처음 받은 모의 시각 − stamp) 중앙 · p95
"""
import json
import re
import sys
from pathlib import Path

import numpy as np

d = Path(sys.argv[1])
stop = float(sys.argv[sys.argv.index("--tf-stop") + 1]) if "--tf-stop" in sys.argv else None
h = json.loads((d / "harness.json").read_text())
t0, dur = h["t0"], h["duration"]
lag = [x["t"] - x["tf_latest"] for x in map(json.loads, (d / "img_lag.jsonl").read_text().split("\n")[:-1]) if x["tf_latest"] is not None]
first = {}
for p in map(json.loads, (d / "pucks.jsonl").read_text().split("\n")[:-1]):
    if p["n"] > 0 and p["stamp"] not in first:
        first[p["stamp"]] = p["recv"]
st = np.array(sorted(first))
lat = np.array([first[s] - s for s in st]) if len(st) else np.array([np.nan])
picks = re.findall(r"front 촬영 시각 TF 로 고른 장: 가장 새 장 (\d+) · 한 장 앞 (\d+) · 기다림 (\d+) · 최근 TF 로 대신 (\d+)",
                   (d / "perception_launch.log").read_text())
out = {
    "tool_img_tf_lag_ms": {"median": round(1e3 * float(np.median(lag)), 1), "max": round(1e3 * float(np.max(lag)), 1)},
    "picks_last": dict(zip(["newest", "older", "wait", "fallback"], map(int, picks[-1]))) if picks else None,
    "new_results": int(len(st)), "rate_hz": round(len(st) / dur, 2),
    "latency_ms": {"median": round(1e3 * float(np.median(lat)), 1), "p95": round(1e3 * float(np.percentile(lat, 95)), 1)},
}
if out["picks_last"]:
    p = out["picks_last"]
    used = p["newest"] + p["older"] + p["fallback"]
    out["exact_tf_ratio"] = round((p["newest"] + p["older"]) / used, 4) if used else None
if stop is not None:
    a, b = st[st - t0 < stop], st[st - t0 >= stop + 1.0]   # 멈춘 뒤 1 s 는 빼고
    out["tf_stop"] = {"rate_before": round(len(a) / stop, 2), "rate_after": round(len(b) / max(1e-9, dur - stop - 1.0), 2)}
print(json.dumps(out, ensure_ascii=False))
(d / "analysis.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
