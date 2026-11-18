"""원래 브라켓(왼쪽)과 위상최적화 브라켓(오른쪽)을 조립 위치에서 나란히 그림(왼쪽 Main + Carrier)."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
H = Path(__file__).resolve().parents[1]
R = Path(sys.argv[1])
sets = {"원래": [H / "placed/Floor_Bracket_Left_Main_assembled_mm.stl", H / "placed/Floor_Bracket_Left_Profile_Carrier_assembled_mm.stl"],
        "위상최적화": [R / "Floor_Bracket_Left_Main_TO_assembled_mm.stl", R / "Floor_Bracket_Left_Profile_Carrier_TO_assembled_mm.stl"]}
col = ["#2f6fb0", "#e08a1e"]
fig = plt.figure(figsize=(15, 10))
k = 0
for e, a in [(15, -75), (15, 105)]:
    for title, files in sets.items():
        k += 1
        ax = fig.add_subplot(2, 2, k, projection="3d")
        vol = 0
        for c, f in zip(col, files):
            m = trimesh.load(f)
            vol += m.volume
            light = np.clip(0.35 + 0.65 * np.abs(m.face_normals @ np.array([0.3, -0.5, 0.8])), 0, 1)
            ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=np.outer(light, matplotlib.colors.to_rgb(c)), edgecolor="none"))
        ax.set_xlim(-160, 100); ax.set_ylim(80, 150); ax.set_zlim(5, 125); ax.set_box_aspect((260, 70, 120))
        ax.view_init(elev=e, azim=a); ax.set_xlabel("x 앞(+)"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_title(f"{title} — Main + Carrier {vol / 1000:.1f} cm³", fontsize=10)
fig.suptitle("층간 긴 브라켓(왼쪽) 위상최적화 후보 A — 증속 벨트 배치 하중, 파랑 Main · 주황 Carrier(오른쪽은 거울상)", fontsize=11)
plt.tight_layout(rect=(0, 0, 1, 0.96))
plt.savefig(sys.argv[2], dpi=100)
print("saved", sys.argv[2])
