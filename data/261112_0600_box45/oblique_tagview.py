"""비스듬히 서기 두 번 보기 안: 같은 자리 (틈 0.35) 에서 태그 읽기용 box_look (9 칸 > 30 px · 태그 1280 · 관절 > 0.1 · 상자 안 0 · 차체 · 실은 퍽 ≥ 15 mm),
β 0 · ±15 · ±30 · ±45 · 보는 점 오차 18 mm 대각 네 방향 + 0. place 는 0.44 m · 50° (렌더 P1 224/224) 그대로.
    ~/isaacsim-venv/bin/python -u oblique_tagview.py > oblique_tagview.txt
"""
import itertools, math, os, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
exec(open(HERE / "oblique_view.py").read().split('print("== 보기 후보')[0])
print("== 틈 0.35 태그 보기 후보 (9 칸 · 태그 · 관절 · 상자 안 · 차체 · 실은 퍽 통과), 태그 큰 순")
rows = []
for dist, ang in itertools.product((0.34, 0.36, 0.38, 0.40), (45, 50, 55, 60, 65)):
    v = evaluate(0.35, dist, ang, (0, 15, -15, 30, -30, 45, -45))
    if v is None:
        continue
    w, inbox, qs = v
    if w["cells"] > 30 and w["tag"] >= 16 and w["jm"] > 0.1 and inbox == 0:
        bd = [CE.body(q, True) for q in qs]
        b, pk = min(x[0] for x in bd), min(x[1] for x in bd)
        if b >= 15 and pk >= 15:
            rows.append((w["tag"], dist, ang, w, b, pk, qs[0]))
for t, dist, ang, w, b, pk, q in sorted(rows, key=lambda r: -r[0]):
    print(f"  {dist:.2f} m · {ang}°: 태그 {w['tag']:.1f} px · 9 칸 {w['cells']:.1f} px · 관절 {w['jm']:.2f} · 차체 {b:.1f} · 실은 퍽 {pk:.1f} mm · q {np.round(q, 4).tolist()}")
