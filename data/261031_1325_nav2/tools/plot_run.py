#!/home/pa4/yolo-venv/bin/python
"""모의 주행 그림: 벽 지도(map_mission) + 고정 물체 + 목표(화살표) + 참값 경로(footprint 여유로 색) + 목표별 결과."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

HERE = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser()
ap.add_argument('--run', required=True)
ap.add_argument('--out', required=True)
a = ap.parse_args()
run = Path(a.run)


def load_pgm(yaml_path):
    my = yaml.safe_load(open(yaml_path))
    parts = (Path(yaml_path).parent / my['image']).read_bytes().split(maxsplit=4)
    W, H = int(parts[1]), int(parts[2])
    img = np.frombuffer(parts[4][:W * H], np.uint8).reshape(H, W)
    ox, oy = my['origin'][:2]
    return img, [ox, ox + W * my['resolution'], oy, oy + H * my['resolution']]


plt.rcParams['font.family'] = ['Noto Sans CJK KR', 'DejaVu Sans']
fig, ax = plt.subplots(figsize=(8, 10))
img, ext = load_pgm(HERE / 'config' / 'map_mission.yaml')
ax.imshow(img, cmap='gray', extent=ext, vmin=0, vmax=255)
obj, ext2 = load_pgm(HERE / 'config' / 'map_objects.yaml')
ax.imshow(np.ma.masked_where(obj != 0, obj), cmap='autumn', extent=ext2, alpha=0.8)
rows = list(csv.DictReader(open(run / 'truth.csv')))
x = np.array([float(r['x']) for r in rows])
y = np.array([float(r['y']) for r in rows])
c = np.array([float(r['clear_body']) for r in rows])
sc = ax.scatter(x, y, c=np.clip(c, 0, 0.2), cmap='viridis', s=2, vmin=0, vmax=0.2)
fig.colorbar(sc, ax=ax, shrink=0.6, label='footprint 여유 m(참값)')
rg = json.loads((run / 'run_goals.json').read_text())
ev = json.loads((run / 'evaluate.json').read_text()) if (run / 'evaluate.json').exists() else {}
for i, g in enumerate(rg['goals']):
    gx, gy, gyaw = g['goal']
    ok = g.get('status') == 'SUCCEEDED'
    ax.arrow(gx, gy, 0.15 * math.cos(gyaw), 0.15 * math.sin(gyaw), width=0.02, color='tab:green' if ok else 'tab:red')
    ax.text(gx + 0.05, gy + 0.08, f"{i + 1} {g['name'].split('(')[0]} {g.get('status', '?')} {g.get('pos_err', '')}", fontsize=7,
            color='tab:green' if ok else 'tab:red', bbox=dict(fc='white', alpha=0.7, lw=0))
ax.set_title(f"Nav2 모의(2D, Isaac 아님) {run.name}\n최소 여유 {ev.get('G3_min_clearance_m')} m · 위치 추정 RMSE {ev.get('G4_loc_rmse_m')} m · 통과 {ev.get('pass')}",
             fontsize=9)
ax.set_xlabel('x m (발제 좌표)')
ax.set_ylabel('y m')
ax.set_xlim(ext[0], ext[1])
ax.set_ylim(ext[2], ext[3])
fig.tight_layout()
fig.savefig(a.out, dpi=130)
print(a.out)
