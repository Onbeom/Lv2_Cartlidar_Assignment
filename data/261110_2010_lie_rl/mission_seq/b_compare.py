"""2026-11-11 (RL 261111 1) B단계 결과 비교: 칸에 앉음, 손가락-칸 벽 > 1 N (단계별 · 칸별), 손 안 appr · side 중앙, 놓기 직전 기울기, 최대 힘.
  python3 b_compare.py <B json> [...]"""
import json
import sys

for f in sys.argv[1:]:
    R = json.load(open(f))["results"]
    S = [r for r in R if r.get("seated")]
    pk = [r for r in R if r.get("picked")]
    hit = [r for r in S if max(r["max_force_N"]["finger"].values()) > 1.0]
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")  # noqa: E731
    print(f"== {f}: 대 {len(R)}, 집기 {len(pk)}, 칸에 앉음 {len(S)}, 그중 손가락-칸 벽 > 1 N {len(hit)} ({100 * len(hit) / max(1, len(S)):.0f} %), M1 (앉음 + 몸체 1 N 이하) {sum(1 for r in R if r.get('m1'))}")
    ph = {}
    for r in pk:
        for k, v in r["max_force_N"]["finger"].items():
            ph.setdefault(k, []).append(v)
    print("  단계별 손가락-몸체 > 1 N 대 / 최대 N: " + ", ".join(f"{k} {sum(1 for x in v if x > 1)} / {max(v):.0f}" for k, v in ph.items()))
    sl = {}
    for r in S:
        a, b = sl.get(r["slot"], (0, 0))
        sl[r["slot"]] = (a + (max(r["max_force_N"]["finger"].values()) > 1.0), b + 1)
    print("  칸별 닿음/앉음: " + ", ".join(f"{k} {a}/{b}" for k, (a, b) in sorted(sl.items())))
    h = [r["in_hand"]["handover"] for r in pk]
    pl = [r["in_hand"]["place"]["tilt_deg"] for r in pk]
    print(f"  넘겨받을 때 appr 중앙 {med([x['appr_mm'] for x in h]):.2f} · side 중앙 {med([x['side_mm'] for x in h]):.2f} mm, 놓기 직전 기울기 중앙 {med(pl):.2f}° | 단계 {dict((k, sum(1 for r in R if r.get('stage') == k)) for k in ('pick_fail', 'transit_drop', 'carry_slip', 'not_seated', 'seated'))}")
