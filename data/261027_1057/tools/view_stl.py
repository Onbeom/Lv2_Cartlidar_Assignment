"""STL 하나를 여러 방향에서 그린 그림(형상 파악용). 사용: view_stl.py <stl> <png>"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

m = trimesh.load(sys.argv[1])
b = m.bounds
fig = plt.figure(figsize=(15, 9))
for k, (e, a) in enumerate([(20, -60), (20, 30), (20, 120), (90, -90), (0, 0), (0, -90)]):
    ax = fig.add_subplot(2, 3, k + 1, projection="3d")
    light = np.clip(0.3 + 0.7 * np.abs(m.face_normals @ np.array([0.3, -0.5, 0.8])), 0, 1)
    ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=np.outer(light, [0.18, 0.44, 0.69]), edgecolor="k", linewidths=0.03))
    c = b.mean(0); r = (b[1] - b[0]).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=e, azim=a)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z"); ax.set_title(f"elev {e} azim {a}", fontsize=9)
plt.tight_layout(); plt.savefig(sys.argv[2], dpi=90)
