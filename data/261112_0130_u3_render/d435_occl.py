"""앞 D435 → 상자 윗면 시선을 팔이 가리는지 (DRIVE 자세, 대기 자리 면 0.45, 상자 ±0.05 · ±15°, Nav2 ±0.05 · ±0.1). 팔 충돌 상자 = dev URDF.
    python3 d435_occl.py"""
import itertools, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
AV = os.path.expanduser("~/git/lv2_next_module/261111_1700_align_view")
sys.argv = ["align_search.py", os.path.join(HERE, "waffle_4wd_dev_c689a38.urdf"), "--pose", "-1.37", "-0.01", "1.85"]
ns = {"__name__": "x"}; _o = sys.stdout; sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(AV, "align_search.py")).read(), ns); sys.stdout = _o
fk, seg_obb, boxes = ns["fk"], ns["seg_obb"], ns["_boxes"]
D435 = fk("top_camera_front_color_optical_frame", {})[:3, 3]
FRONT, HALF, ZTOP = 0.1328, 0.1025, 0.07
NOM = np.array([FRONT + 0.45 + HALF, 0.0])
def R2(t): return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
for name, q5 in (("DRIVE", [-0.0245, -1.6122, 1.2901, 1.1888, 0.0]), ("STOW", [1.554, -2.088, 1.549, -1.868, 0.043])):
    q = {f"joint{i + 1}": q5[i] for i in range(5)}
    bx = [(n, fk(n, q) @ To, h) for n, To, h in boxes]
    blocked, total, links = 0, 0, set()
    for dx, dy, b, nx, ny, nyw in itertools.product((-0.05, 0, 0.05), (-0.05, 0, 0.05), (-15, 0, 15), (-0.05, 0, 0.05), (-0.05, 0, 0.05), (-0.1, 0, 0.1)):
        bc = R2(-nyw) @ (NOM + np.array([dx, dy]) - np.array([nx, ny])); by = math.radians(b) - nyw
        for sx, sy in itertools.product(np.linspace(-1, 1, 5), repeat=2):
            if abs(sx) < 1 and abs(sy) < 1: continue   # 테두리 점만
            p = np.r_[bc + R2(by) @ np.array([sx, sy]) * HALF, ZTOP]
            hit = [n for n, M, h in bx if seg_obb(D435, p, M, h)]
            total += 1
            if hit: blocked += 1; links.update(hit)
    print(f"{name}: 상자 윗면 테두리 시선 {total} 중 팔에 가림 {blocked}  ({sorted(links)})")
