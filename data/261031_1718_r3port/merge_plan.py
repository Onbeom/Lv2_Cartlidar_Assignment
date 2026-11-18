"""plan/seed_*.json 합치기 → plan/r3_plan_T3000.json (실패 조건은 그대로 모음)."""
import json
import sys
from pathlib import Path

D = Path(__file__).resolve().parent / "plan"
out = {"plan": [], "labels": [], "unreachable": []}
for f in sorted(D.glob("seed_*.json")):
    d = json.loads(f.read_text())
    for k in out:
        out[k] += d[k]
(D / "r3_plan_T3000.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
arm = [p for p in out["plan"] if p["kind"] == "arm"]
print("라벨", len(out["labels"]), "팔 장면", len(arm), "자세 없음", len(out["unreachable"]),
      "문자열", sorted({l["text"] for l in out["labels"]}))
sys.exit(0 if len(arm) == 36 and not out["unreachable"] else 1)
