"""A1 재생 행(a1_rows.jsonl)과 노드 ARM_LOCK 진단 줄을 맞춘다 (PC). python3 join_diag.py <재생 폴더>
행 = 팔 영상 한 장(재생 순서). ARM_LOCK 줄은 같은 stamp 를 여러 번 처리할 수 있어 stamp 마다 마지막 줄을 쓰고, stamp 순서로 행과 짝짓는다.
실패 장(perception_summary.json failures)의 진단과 거부 이유별 개수를 낸다."""
import json, re, sys
from collections import Counter
from pathlib import Path
d = Path(sys.argv[1])
rows = [json.loads(l) for l in (d / "a1_rows.jsonl").read_text().splitlines()]
pat = re.compile(r"ARM_LOCK stamp=([\d.]+) pick=(-?\d+) boxes=(\d+) cls=(\d+) size=(\d+) seed_gate=(\d+) follow=(\d+) ambiguous=(\d) locked=(\d) seen=(\d) seed=(\d)")
by = {}
for m in pat.finditer((d / "perception_launch.log").read_text()):
    by[m.group(1)] = dict(zip(["pick", "boxes", "cls", "size", "seed_gate", "follow", "ambiguous", "locked", "seen", "seed"], map(int, m.groups()[1:])))
st = sorted(by, key=float)
print(f"행 {len(rows)} · 진단 stamp {len(st)}")
fails = {(f["scene"], f["k"]) for f in json.loads((d / "perception_summary.json").read_text())["failures"] if f.get("k") is not None}
if len(st) != len(rows):
    print("개수가 달라 짝짓지 않음"); sys.exit(1)
reasons = Counter()
for r, s in zip(rows, st):
    g = by[s]
    if (r["scene"], r["k"]) in fails:
        why = [k for k in ("cls", "size", "seed_gate", "follow") if g[k]] + (["ambiguous"] if g["ambiguous"] else []) or ["no_box" if g["boxes"] == 0 else "?"]
        reasons.update(why)
        print(r["scene"], r["k"], r["stage"], "pick", g["pick"], "boxes", g["boxes"], "빠짐", {k: g[k] for k in ("cls", "size", "seed_gate", "follow") if g[k]},
              "모호" if g["ambiguous"] else "", "locked", g["locked"], "seen", g["seen"], "seed", g["seed"])
print("실패 장 이유:", dict(reasons))
