"""설계 자료 dump 에서 진짜(판정 대상, 라벨 30 mm 안) · 헛(50 mm 밖) 후보의 특징 분포."""
import json, sys, glob
from pathlib import Path
import numpy as np
rows = []
for f in sorted(glob.glob(sys.argv[1] + "/*.json")):
    for r in json.loads(Path(f).read_text()):
        r["src"] = Path(f).stem; rows.append(r)
T = [r for r in rows if r["dcur"] <= 0.03 and r["judged"]]
F = [r for r in rows if r["dmin"] > 0.05]
bw = lambda r: r["band_mean"] / max(r["white_mean"], 1)
for kind in ("wall", "top"):
    t = [r for r in T if r["kind"] == kind]
    print(f"[{kind}] 진짜 {len(t)}: bw 최대 {max(map(bw, t)):.3f}  상위5 {sorted(map(bw, t))[-5:]}")
    print(f"   점수 최소 {min(r['score'] for r in t):.3f} 하위5 {sorted(round(r['score'],3) for r in t)[:5]}")
    if kind == "wall":
        print(f"   pl_w 최소 {min(r['pl_w'] for r in t)*1000:.0f} mm 하위5 {sorted(round(r['pl_w']*1000) for r in t)[:5]}")
for r in sorted(F, key=lambda r: -r["score"]):
    print("헛", r["src"], r["scene"], r["kind"], "score %.3f bw %.3f pl_w %.0f mm side %.2f sharp %.3f" % (r["score"], bw(r), r["pl_w"]*1000, r["side_lo"], r["sharp"]))
# 진짜 bw 상위 장면
for r in sorted([r for r in T if r["kind"] == "top"], key=lambda r: -bw(r))[:6]:
    print("진짜 bw 높음", r["src"], r["scene"], "bw %.3f score %.3f hdist %.2f" % (bw(r), r["score"], r["hdist"]))
