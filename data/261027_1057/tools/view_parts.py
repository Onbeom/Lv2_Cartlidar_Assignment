"""조립 위치 브라켓(왼쪽 Main + Carrier) 여러 방향 그림(형상 파악용)."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK KR", "DejaVu Sans"]
P = Path(__file__).resolve().parents[1] / "placed"
ms = {"Main": trimesh.load(P / "Floor_Bracket_Left_Main_assembled_mm.stl"),
      "Carrier": trimesh.load(P / "Floor_Bracket_Left_Profile_Carrier_assembled_mm.stl")}
col = {"Main": "#2f6fb0", "Carrier": "#e08a1e"}
views = [(20, -70), (20, 110), (85, -90), (-60, -60), (5, 180), (5, 0)]
fig = plt.figure(figsize=(16, 10))
for k, (e, a) in enumerate(views):
    ax = fig.add_subplot(2, 3, k + 1, projection="3d")
    for n, m in ms.items():
        light = np.clip(0.3 + 0.7 * np.abs(m.face_normals @ np.array([0.3, -0.5, 0.8])), 0, 1)
        ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=np.outer(light, matplotlib.colors.to_rgb(col[n])),
                                             edgecolor="k", linewidths=0.05))
    ax.set_xlim(-160, 100); ax.set_ylim(80, 150); ax.set_zlim(5, 125)
    ax.set_box_aspect((260, 70, 120))
    ax.view_init(elev=e, azim=a)
    ax.set_title(f"elev {e}, azim {a}", fontsize=9)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
plt.tight_layout()
plt.savefig(sys.argv[1], dpi=100)
