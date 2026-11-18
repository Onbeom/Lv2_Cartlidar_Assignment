"""조립 위치 브라켓 4개만 그린 그림(나머지 부품은 그리지 않음). base_link 좌표 mm, 앞 = +x.
사용(PC): ~/pa-course/IsaacLab-6.1/.venv/bin/python tools/render_brackets.py <출력 png>
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK KR", "NanumGothic", "DejaVu Sans"]
P = Path(__file__).resolve().parents[1] / "placed"
COL = {"Main": "#2f6fb0", "Profile_Carrier": "#e08a1e"}     # 파랑·주황(색약에서도 구분)
fig = plt.figure(figsize=(13, 6.2))
views = [(25, -60, "앞 왼쪽 위에서"), (20, 120, "뒤 오른쪽 위에서")]
for k, (elev, azim, title) in enumerate(views):
    ax = fig.add_subplot(1, 2, k + 1, projection="3d")
    for f in sorted(P.glob("*_assembled_mm.stl")):
        m = trimesh.load(f)
        kind = "Main" if f.name.endswith("Main_assembled_mm.stl") else "Profile_Carrier"
        light = np.clip(0.35 + 0.65 * np.abs(m.face_normals @ np.array([0.3, -0.5, 0.8]) / 1.0), 0, 1)
        base = np.array(matplotlib.colors.to_rgb(COL[kind]))
        pc = Poly3DCollection(m.triangles, facecolors=np.outer(light, base), edgecolor="none")
        ax.add_collection3d(pc)
        c = m.bounds.mean(0)
        side = "왼쪽" if c[1] > 0 else "오른쪽"
        ax.text(c[0], c[1], m.bounds[1, 2] + 12, f"{side} {'Main' if kind == 'Main' else 'Carrier'}", fontsize=8,
                ha="center")
    ax.set_xlim(-170, 110); ax.set_ylim(-150, 150); ax.set_zlim(0, 140)
    ax.set_box_aspect((280, 300, 140))
    ax.set_xlabel("x 앞(+) [mm]"); ax.set_ylabel("y 왼쪽(+) [mm]"); ax.set_zlabel("z [mm]")
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontsize=10)
fig.suptitle("토폴로지·모폴로지 설계 대상 후보: 층간 긴 브라켓(좌·우 각 Main + Profile Carrier, 조립 위치, 다른 부품은 숨김)", fontsize=11)
fig.text(0.5, 0.02, "파랑 = Floor_Bracket_*_Main(출력물 217.6×90×42 mm, 142.0 cm³)   주황 = Floor_Bracket_*_Profile_Carrier(52.7×110×58 mm, 66.4 cm³)",
         ha="center", fontsize=9)
plt.tight_layout(rect=(0, 0.04, 1, 0.95))
plt.savefig(sys.argv[1], dpi=130)
print("saved", sys.argv[1])
