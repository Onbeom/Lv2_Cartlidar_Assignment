"""기록용 (판정 아님, 계산): 두 번째 퍽은 같은 정차에서 turn ±17° 뒤 target_look L · R 자세에서 찾음.
그 자세에서 앞 D435 컬러 광학 원점 → 두 번째 퍽(윗면 · 중간 높이) 시선을 팔 충돌 상자가 가리는지 (면 0.127, 옆 배치 ±0.08 · inline ±0.04).
  python3 tl_d435_occl.py <dev urdf>"""
import math, os, sys
import numpy as np
dev = sys.argv[1]
sys.argv = ["align_search.py", dev, "--pose", "-1.35", "-0.02", "1.88"]
ns = {"__name__": "x"}; _o = sys.stdout; sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "align_search.py")).read(), ns); sys.stdout = _o
fk, boxes, seg_obb, D435 = ns["fk"], ns["_boxes"], ns["seg_obb"], ns["D435"]
FRONT, C_X, HALF, ZTOP = ns["FRONT"], ns["C_X"], ns["HALF"], ns["ZTOP"]
TL = {"TL_L": [-0.2976, -0.5214, 0.2755, 1.2058, 0], "TL_R": [0.2976, -0.5214, 0.2755, 1.2058, 0]}
xc = FRONT + 0.127 + HALF
for name, q5 in TL.items():
    q = {f"joint{i + 1}": q5[i] for i in range(5)}
    bx = [(n, fk(n, q) @ To, h) for n, To, h in boxes]
    for yaw_deg in (17, -17):
        yaw = math.radians(yaw_deg)
        for lay, (dx, dy) in (("옆+", (0, 0.08)), ("옆-", (0, -0.08)), ("inline 앞", (-0.04, 0)), ("inline 뒤", (0.04, 0))):
            hits = []
            for zf in (ZTOP + 0.06, ZTOP + 0.03):
                x, y = xc + dx - C_X, dy
                c, s = math.cos(-yaw), math.sin(-yaw)          # 로봇이 yaw 돌면 장면은 −yaw
                p = np.array([C_X + c * x - s * y, s * x + c * y, zf])
                h = [n for n, M, hh in bx if seg_obb(D435, p, M, hh)]
                if h: hits.append((round(zf, 3), h[0]))
            print(f"{name} 회전 {yaw_deg:+d}° 퍽 {lay}: {'가림 ' + str(hits) if hits else '안 가림'}")
