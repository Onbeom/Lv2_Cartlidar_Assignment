"""위에서 본 팔레트(스토리지) 칸 이름 그림 (계산 그림, 사진 아님). 칸 자리 = pallet_slots.yaml(= loaded URDF 퍽), 팔꿈치 = link3 충돌 상자 15 를 위에서 본 것.
  python draw_pallet_top.py <isaac urdf 폴더> <출력 png>"""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Circle, Rectangle
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "rl_probe"))
from align_geom import Geom
for f in font_manager.findSystemFonts():
    if "NotoSansCJK" in f or "NotoSerifCJK" in f:
        font_manager.fontManager.addfont(f)
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "Noto Sans CJK KR", "Noto Serif CJK KR", "DejaVu Sans"]
I, out = sys.argv[1], sys.argv[2]
G = Geom(os.path.join(I, "waffle_4wd_loaded.urdf"))
SL = [("R_1", "cyl", 0.0489, 0.0767, 121.6), ("R_2", "box", 0.0111, 0.0300, 160.5), ("R_3", "cyl", 0.0111, -0.0300, 199.5), ("R_4", "box", 0.0489, -0.0767, 238.4),
      ("G_1", "box", -0.0232, 0.0771, 147.1), ("G_2", "cyl", -0.0434, 0.0270, 169.0), ("G_3", "box", -0.0434, -0.0270, 191.0), ("G_4", "cyl", -0.0232, -0.0771, 212.9),
      ("B_1", "cyl", -0.0780, 0.0788, 155.6), ("B_2", "box", -0.0931, 0.0270, 171.9), ("B_3", "cyl", -0.0931, -0.0270, 188.1), ("B_4", "box", -0.0780, -0.0788, 204.4)]
COL = {"R": "#d62728", "G": "#2ca02c", "B": "#1f77b4"}
FLOOR = {"R": 0.114, "G": 0.124, "B": 0.136}
# 그림 좌표: 위 = 로봇 앞(+x), 가로축 = y 를 뒤집어 그림 왼쪽 = 로봇 왼쪽(+y) (위에서 내려다본 방향)
def uv(x, y): return (y, x)
fig, ax = plt.subplots(figsize=(8, 9.5))
FRONT, REAR, HW = 0.1328, -0.2135, 0.1325
ax.add_patch(Polygon([uv(FRONT, HW), uv(FRONT, -HW), uv(REAR, -HW), uv(REAR, HW)], closed=True, fill=False, lw=2, ec="k"))
ax.text(*uv(FRONT + 0.008, 0), "차체 앞 (범퍼)", ha="center", va="bottom", fontsize=11)
ax.plot(*uv(0.096, 0), "k+", ms=14, mew=2); ax.text(*uv(0.104, 0.012), "팔 관절 1 축\n(x 0.096)", fontsize=9, ha="right")
ax.plot(*uv(-0.0658, 0), "mx", ms=10, mew=2); ax.text(*uv(-0.0658, -0.03), "제자리 회전 중심 icr_x −0.066", fontsize=8, color="m", va="center", ha="center")
for n, sh, x, y, th in SL:
    c = COL[n[0]]
    if sh == "cyl":
        ax.add_patch(Circle(uv(x, y), 0.015, fc=c, alpha=0.45, ec=c, lw=1.5))
    else:
        t = math.radians(th); R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        P = [R @ np.array(p) + np.array([x, y]) for p in ((-.015, -.015), (.015, -.015), (.015, .015), (-.015, .015))]
        ax.add_patch(Polygon([uv(*p) for p in P], closed=True, fc=c, alpha=0.45, ec=c, lw=1.5))
    ax.text(*uv(x, y), n, ha="center", va="center", fontsize=10, weight="bold")
for name, q, col in (("DRIVE", [-0.0245, -1.6122, 1.2901, 1.1888, 0], "#ff7f0e"), ("E2", [0, -1.37, -0.01, 1.85, 0], "#9467bd")):
    for ln, an, A in G.arm_pts(q, 0.010):
        if an.endswith("box15") and ln == "link3":
            lo, hi = A.min(0), A.max(0)
            u0, v0 = uv(hi[0], hi[1]); u1, v1 = uv(lo[0], lo[1])
            ax.add_patch(Rectangle((min(u0, u1), min(v0, v1)), abs(u1 - u0), abs(v1 - v0), fill=False, ec=col, lw=2.2, ls="--"))
            side = 0.125 if name == "DRIVE" else -0.125
            ax.annotate(f"{name} 자세 팔꿈치\n(관절 3 모터, 위에서 본 자리)\n아랫면 높이 z {lo[2]:.3f} m", xy=((u0 + u1) / 2, (v0 + v1) / 2), xytext=(side, -0.005),
                        color=col, fontsize=9, ha="center", arrowprops=dict(arrowstyle="->", color=col))
ax.text(*uv(REAR + 0.01, 0), "앞줄 R 칸 바닥 0.114 · 가운데 G 0.124 · 뒷줄 B 0.136 m\n퍽 높이 60 mm → 퍽 윗면 R 0.174 · G 0.184 · B 0.196 m\nDRIVE 팔꿈치 아랫면 0.173 < G 퍽 윗면 0.184 → G_2 · G_3 와 겹침\n원 = 원기둥 퍽, 네모 = 직육면체 퍽 (loaded URDF 배치)",
        ha="center", va="bottom", fontsize=9)
ax.set_aspect("equal"); ax.set_xlim(-0.2, 0.2); ax.set_ylim(REAR - 0.01, FRONT + 0.03)
ax.set_xlabel("← 로봇 왼쪽 (+y)      base_link y [m]      로봇 오른쪽 (−y) →"); ax.set_ylabel("base_link x [m] (위 = 앞)")
ax.invert_xaxis()
ax.set_title("팔레트(스토리지) 칸 이름 — 위에서 본 그림 (계산, 사진 아님)\npallet_slots.yaml · loaded URDF, 팔꿈치 = link3 충돌 상자", fontsize=11)
fig.tight_layout(); fig.savefig(out, dpi=130)
print(out)
