#!/usr/bin/env python3
"""place_box 만 빠르게 채점: pb_run.py <렌더 폴더> <도구> [--only 영상 ...] [--save 결과.json]
render_u3.json 의 장면마다 도구를 돌려 중심 · 요 오차를 낸다 (eval_u3.py P1 과 같은 계산, 태그 읽기 없음)."""
import argparse, json, math, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
ap = argparse.ArgumentParser(); ap.add_argument('dir', type=Path); ap.add_argument('tool'); ap.add_argument('--only', nargs='*'); ap.add_argument('--save')
a = ap.parse_args()
meta = json.load(open(a.dir / 'render_u3.json')); K = meta['K']
scs = [s for s in meta['scenes'] if not a.only or s['image'] in a.only]
def run(sc):
    T = [v for row in sc['T_bf_cam'] for v in row]
    out = subprocess.run([a.tool, str(a.dir / sc['image']), *map(str, (K['fx'], K['fy'], K['cx'], K['cy'])), *map(str, T),
                          str(sc['prior'][0]), str(sc['prior'][1]), str(sc['prior_yaw'])], capture_output=True, text=True).stdout.split()
    x, y, yaw, score, ok = float(out[0]), float(out[1]), float(out[2]), float(out[3]), int(out[4])
    dc = 1000 * math.hypot(x - sc['center'][0], y - sc['center'][1])
    dy = abs(math.degrees((yaw - sc['yaw'] + math.pi / 4) % (math.pi / 2) - math.pi / 4))
    return dict(image=sc['image'], id=sc['id'], cam_dz=sc.get('cam_dz', 0.0), ok=bool(ok), c=dc, y=dy, score=score, extra=out[5:])
with ThreadPoolExecutor(4) as ex:
    rows = list(ex.map(run, scs))
if a.save:
    json.dump(rows, open(a.save, 'w'), indent=0)
for r in rows:
    if a.only or not r['ok'] or r['c'] > 3:
        print(f"{r['image']} dz {r['cam_dz']:+.3f} ok {int(r['ok'])} 중심 {r['c']:.2f} mm 요 {r['y']:.2f}° score {r['score']:.1f} {' '.join(r['extra'])}")
ok = [r for r in rows if r['ok']]
print(f"합계 {len(rows)} 찾음 {len(ok)} | 중심 ≤3 mm·요 ≤1° {sum(r['c'] <= 3 and r['y'] <= 1 for r in ok)} | 중심 >15 mm {sum(r['c'] > 15 for r in ok)} | 최대 {max([r['c'] for r in ok] or [0]):.2f} mm")
