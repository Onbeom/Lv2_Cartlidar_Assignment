"""R3′ 집계: 렌더(render_r3.json) + C++ 판독(read_T*.jsonl) → summary.json. 기준: 보임 ≥ 0.95 장면에서 정답 100 %, 틀린 문자열 0, 144장."""
import json
from pathlib import Path

D = Path(__file__).resolve().parent
rows = []
for T in (3000, 6500):
    rep = json.loads((D / f"render_T{T}_awb/render_r3.json").read_text())
    read = {r["name"]: r for r in map(json.loads, (D / f"read_T{T}.jsonl").read_text().splitlines())}
    for p in rep["poses"]:
        r = read[p["name"]]
        got = r.get("text") or ""
        rows.append({"temp_k": T, "name": p["name"], "seed": p["seed"], "light": p["light_level"], "tilt": p["tilt_deg"],
                     "extra": p["extra_m"], "d_min": p["d_min"], "dist": p["dist_nominal"], "module_px": p["module_px"],
                     "roll_actual_deg": p["roll_actual_deg"], "visible": p["label_visible"], "truth": p["text"], "read": got,
                     "correct": got == p["text"], "wrong": bool(got) and got != p["text"], "ms": r.get("ms")})
elig = [r for r in rows if r["visible"] >= 0.95]


def cnt(sel):
    return {"n": len(sel), "correct": sum(r["correct"] for r in sel), "wrong": sum(r["wrong"] for r in sel)}


s = {"frames": len(rows), "eligible": cnt(elig), "not_visible": len(rows) - len(elig),
     "by_temp": {T: cnt([r for r in elig if r["temp_k"] == T]) for T in (3000, 6500)},
     "by_light": {L: cnt([r for r in elig if r["light"] == L]) for L in (0.0, 1.0)},
     "by_tilt": {t: cnt([r for r in elig if r["tilt"] == t]) for t in (0.0, 15.0)},
     "by_text": {x: cnt([r for r in elig if r["truth"] == x]) for x in sorted({r["truth"] for r in rows})},
     "module_px": [min(r["module_px"] for r in rows), max(r["module_px"] for r in rows)],
     "d_min": sorted({(r["tilt"], r["d_min"]) for r in rows}),
     "max_ms": max(r["ms"] or 0 for r in rows)}
s["pass"] = len(rows) == 144 and s["eligible"]["n"] == 144 and s["eligible"]["correct"] == 144 and s["eligible"]["wrong"] == 0
(D / "summary.json").write_text(json.dumps(s, indent=1, ensure_ascii=False))
(D / "rows.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False))
print(json.dumps(s, ensure_ascii=False))
