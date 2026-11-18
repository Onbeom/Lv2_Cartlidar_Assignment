"""render_tilt.py 결과를 장면별 한 줄(왼쪽 35.2° · 오른쪽 10°)로 모은 그림."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.image as mpimg  # noqa: E402

plt.rcParams["font.family"] = ["Noto Sans CJK KR", "DejaVu Sans"]
d = Path(sys.argv[1])
meta = json.loads((d / "meta.json").read_text())
names = list(dict.fromkeys(s["name"] for s in meta["scenes"]))
nt = len(meta["tilts_deg"])
fig, axes = plt.subplots(len(names), nt, figsize=(5 * nt, 3.9 * len(names)))
for i, n in enumerate(names):
    for j, tilt in enumerate(meta["tilts_deg"]):
        s = next(s for s in meta["scenes"] if s["name"] == n and s["tilt_deg"] == tilt)
        ax = axes[i, j]
        ax.imshow(mpimg.imread(d / s["image"]))
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(f"{s['desc']} — 기울기 {tilt:g}°", fontsize=9)
arm = {"s0": "팔 접은 자세 S0", "zero": "팔 기본 자세(관절 모두 0)"}.get(meta.get("arm", "s0"))
tl = " · ".join(f"{t:g}°" for t in meta["tilts_deg"])
fig.suptitle(f"RealSense 컬러 640×480, 아래 기울기 {tl}(왼쪽부터, 35.2° = 지금 URDF), 같은 로봇 자리 · {arm} · 조명 {meta['light']}\n"
             "과제 미로 seed_1, 모의 렌더(판정 아님)", fontsize=11)
plt.tight_layout(rect=(0, 0, 1, 0.97))
plt.savefig(sys.argv[2], dpi=110)
print("saved", sys.argv[2])
