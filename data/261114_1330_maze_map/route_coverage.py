#!/usr/bin/env python3
"""미로 재녹화 경로의 라이다 시야 확인 (PC, 11-16). map_amcl 위에서 경로를 따라 앞 · 뒤 라이다가 실제로 보는 각(11-14 bag 에서 잰 값)으로
광선을 쏘아, 한 번도 안 맞는 벽 칸을 그린다.
  python3 route_coverage.py <free_scan_front.npy> <free_scan_rear.npy> <출력 png>
- 쓸 만한 각: 11-14 bag 4000 장에서 0.35 m 넘게 반사가 돌아온 비율이 반 이상인 1° 칸 (로봇 기준, 0 = 앞, 반시계 +) — 앞 87° · 뒤 196°
- 광선 3.0 m, 1° 간격. 경로는 0.05 m 간격, 회전은 지나가는 각을 5° 간격으로 (360° 자리는 한 바퀴)
"""
import math
import sys

import numpy as np
from PIL import Image, ImageDraw

MAP = '/home/pa4/git/lv2_next_module/261114_1330_maze_map/sdf_map/map_amcl.pgm'
RES, OX, OY = 0.025, -0.15, -0.15
LIDAR = {'front': (0.1318, 0.0), 'rear': (-0.1318, 0.0)}
FREE = {'front': np.load(sys.argv[1]), 'rear': np.load(sys.argv[2])}
RMAX = 3.0
E, N, W, S = 0.0, 90.0, 180.0, 270.0
P = {'P0': (0.69, 0.95), 'A': (0.69, 1.68), 'P1': (0.69, 3.15), 'P2': (2.2, 3.15), 'B': (2.2, 4.45), 'P3': (2.2, 4.6), 'P4': (0.54, 4.6),
     'P6': (2.9, 4.6), 'Q': (3.8, 4.6), 'P7': (2.9, 3.875), 'P8': (3.65, 3.875), 'P9': (2.2, 3.875), 'P10': (2.2, 1.7), 'P11': (3.6, 1.7),
     'P12': (2.2, 0.65), 'P13': (3.85, 0.65)}
# (동작, …): ('rot', 자리, 시작 방향, 끝 방향, 회전 방향 +1 반시계 / −1 시계) · ('360', 자리) · ('go', 시작, 끝, 방향) — 뒤로 나옴도 같은 방향 그대로
ROUTE = [('rot', 'P0', W, N, -1), ('go', 'P0', 'A', N), ('360', 'A'), ('go', 'A', 'P1', N), ('rot', 'P1', N, E, -1), ('go', 'P1', 'P2', E),
         ('360', 'P2'), ('rot', 'P2', E, N, 1), ('go', 'P2', 'B', N), ('360', 'B'), ('go', 'B', 'P3', N), ('rot', 'P3', N, E, -1),
         ('go', 'P3', 'P4', E), ('go', 'P4', 'P3', E), ('go', 'P3', 'P6', E),   # v3: P4 방은 E 를 본 채 뒤로 들어감 (뒤 라이다가 방 왼쪽 위 벽을 봄) ('360', 'P6'), ('go', 'P6', 'Q', E),
         ('go', 'Q', 'P6', E), ('rot', 'P6', E, S, -1), ('go', 'P6', 'P7', S), ('rot', 'P7', S, E, 1), ('go', 'P7', 'P8', E), ('go', 'P8', 'P7', E),
         ('rot', 'P7', E, W, 1), ('go', 'P7', 'P9', W), ('rot', 'P9', W, S, 1), ('go', 'P9', 'P10', S), ('rot', 'P10', S, E, 1),
         ('go', 'P10', 'P11', E), ('go', 'P11', 'P10', E), ('rot', 'P10', E, S, -1), ('go', 'P10', 'P12', S), ('rot', 'P12', S, E, 1),
         ('go', 'P12', 'P13', E), ('rot', 'P13', E, W, 1), ('go', 'P13', 'P12', W), ('rot', 'P12', W, N, -1), ('go', 'P12', 'P2', N),
         ('rot', 'P2', N, W, 1), ('go', 'P2', 'P1', W), ('rot', 'P1', W, S, 1), ('go', 'P1', 'P0', S), ('rot', 'P0', S, W, -1)]

im = np.array(Image.open(MAP))
H, Wd = im.shape
occ = im < 128


def cell(x, y):
    return int(round(H - 1 - (y - OY) / RES)), int(round((x - OX) / RES))


poses = []
for st in ROUTE:
    if st[0] == 'go':
        (x0, y0), (x1, y1) = P[st[1]], P[st[2]]
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
        poses += [(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, st[3]) for t in np.linspace(0, 1, n + 1)]
    elif st[0] == 'rot':
        x, y = P[st[1]]
        a0, a1, sgn = st[2], st[3], st[4]
        sweep = ((a1 - a0) * sgn) % 360 or 360
        poses += [(x, y, a0 + sgn * k) for k in np.arange(0, sweep + 0.1, 5)]
    else:
        x, y = P[st[1]]
        poses += [(x, y, h) for h in np.arange(0, 360, 5)]

seen = np.zeros_like(occ)
steps = np.arange(0.0, RMAX, RES / 2)
for x, y, h in poses:
    hr = math.radians(h)
    for name, (lx, ly) in LIDAR.items():
        ox = x + math.cos(hr) * lx - math.sin(hr) * ly
        oy = y + math.sin(hr) * lx + math.cos(hr) * ly
        for d in np.where(FREE[name])[0]:
            a = hr + math.radians(d + 0.5)
            for r in steps[steps > 0.12]:
                rr, cc = cell(ox + r * math.cos(a), oy + r * math.sin(a))
                if not (0 <= rr < H and 0 <= cc < Wd):
                    break
                if occ[rr, cc]:
                    seen[rr, cc] = True
                    break

# 겉면 벽 칸만 셈: 출발 자리에서 이어진 빈 칸(미로 안)과 맞닿은 벽 칸 (두꺼운 벽 속 · 바깥 면은 원래 안 보임)
from scipy import ndimage
lab, _ = ndimage.label(~occ)
free_in = lab == lab[cell(*P['P0'])]
nb = np.zeros_like(occ)
nb[1:, :] |= free_in[:-1, :]
nb[:-1, :] |= free_in[1:, :]
nb[:, 1:] |= free_in[:, :-1]
nb[:, :-1] |= free_in[:, 1:]
wall = occ & nb
total, hit = int(wall.sum()), int((wall & seen).sum())
ys, xs = np.where(wall)
wx, wy = OX + xs * RES, OY + (H - 1 - ys) * RES
outer = (wx < 0.1) | (wx > 4.25) | (wy < 0.1) | (wy > 6.1)
print(f'바깥 벽 안쪽 면 {int(outer.sum())} 칸 중 본 칸 {int(seen[ys[outer], xs[outer]].sum())} ({100 * seen[ys[outer], xs[outer]].mean():.1f} %) · 안쪽 벽 {int((~outer).sum())} 칸 중 {100 * seen[ys[~outer], xs[~outer]].mean():.1f} %')
print(f'자세 {len(poses)} 개 · 벽 칸 {total} · 본 칸 {hit} ({100 * hit / total:.1f} %) · 안 본 칸 {total - hit} ({100 * (total - hit) / total:.1f} %)')
S_ = 3
img = Image.fromarray(np.where(occ, 0, 255).astype(np.uint8)).convert('RGB').resize((Wd * S_, H * S_), Image.NEAREST)
dr = ImageDraw.Draw(img)
for r, c in zip(*np.where(wall & ~seen)):
    dr.rectangle([c * S_, r * S_, c * S_ + S_ - 1, r * S_ + S_ - 1], fill=(230, 0, 0))
for r, c in zip(*np.where(wall & seen)):
    dr.rectangle([c * S_, r * S_, c * S_ + S_ - 1, r * S_ + S_ - 1], fill=(0, 150, 0))
for k, (x, y) in P.items():
    rr, cc = cell(x, y)
    dr.ellipse([cc * S_ - 5, rr * S_ - 5, cc * S_ + 5, rr * S_ + 5], outline=(0, 0, 200), width=2)
    dr.text((cc * S_ + 7, rr * S_ - 12), k, fill=(0, 0, 200))
dr.text((8, H * S_ - 20), f'green = seen wall, red = never seen ({100 * (total - hit) / total:.1f} % of wall surface cells)', fill=(0, 0, 0))
img.save(sys.argv[3])
