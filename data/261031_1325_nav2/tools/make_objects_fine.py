"""설계 11 고정 물체 지도: map_objects 와 같은 물체 · 같은 발제 좌표 범위를 더 고운 격자(기본 0.01 m)로, 칸 중심이 물체 안일 때만 점유(0) — 바깥으로 반 칸 덧대지 않음.
설계 9 v9_c: 참값 회전 여유 0.029 m 인데 costmap 상 충돌 — 물체 지도 덧댐(반 칸 0.0125) · local costmap 재표본(±0.0125) · footprint 윤곽 격자화(±0.0125)가 여유를 먹음.
--posts(설계 26): 미로 충돌 원기둥(261030_1943/maze_posts.json, 반지름 0.0303) 도 점유로 그림 — SLAM 지도엔 칸 크기만큼 깎여 들어가 있고
geom 단면 자료엔 빠져 있던 것(설계 21 · 22 · 25 출발 통로 막힘 원인).
--walls(설계 27): 도면(geom) 벽 선분도 점유로 그림 — 두께 3 mm 판은 SLAM 지도에 구멍이 날 수 있음(tools/check_world_consistency.py: (1.175, 1.275) 5.5 cm ·
(3.392, 5.12) 3.8 cm). 선분에서 반 칸(0.005) 안의 칸을 점유. 위치 추정은 SLAM 지도 그대로, costmap 만 도면 구조물을 함께 봄."""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--ref', default=str(HERE / 'config' / 'map_objects.yaml'), help='범위(origin · 크기)를 맞출 지도')
ap.add_argument('--res', type=float, default=0.01)
ap.add_argument('--name', default='map_objects_fine')
ap.add_argument('--posts', action='store_true')
ap.add_argument('--walls', action='store_true')
a = ap.parse_args()
ref = yaml.safe_load(open(a.ref))
parts = (Path(a.ref).parent / ref['image']).read_bytes().split(maxsplit=4)
W0, H0 = int(parts[1]), int(parts[2])
ox, oy = ref['origin'][:2]
W, H = int(round(W0 * ref['resolution'] / a.res)), int(round(H0 * ref['resolution'] / a.res))
X, Y = np.meshgrid(ox + (np.arange(W) + 0.5) * a.res, (oy + (np.arange(H) + 0.5) * a.res)[::-1])
img = np.full((H, W), 128, np.uint8)
for o in geom.load_world()['objects']:
    c, s = math.cos(o['yaw']), math.sin(o['yaw'])
    lx = c * (X - o['cx']) + s * (Y - o['cy'])
    ly = -s * (X - o['cx']) + c * (Y - o['cy'])
    img[(np.abs(lx) <= o['hx']) & (np.abs(ly) <= o['hy'])] = 0
n_post = 0
if a.posts:
    import json
    seen = set()
    for q in json.loads(geom.POSTS.read_text())['posts']:
        cx, cy = q['center'][0] + geom.USD_OFFSET[0], q['center'][1] + geom.USD_OFFSET[1]
        if (round(cx, 4), round(cy, 4)) in seen:
            continue
        seen.add((round(cx, 4), round(cy, 4)))
        img[np.hypot(X - cx, Y - cy) <= q['radius']] = 0
        n_post += 1
n_wall = 0
if a.walls:
    w_ = geom.load_world(posts=None)
    segs = w_['body'][:len(w_['body']) - 4 * len(w_['objects'])]          # 벽 선분만(layout 물체 제외, 기둥은 --posts 로)
    for x0, y0, x1, y1 in segs:
        L = math.hypot(x1 - x0, y1 - y0)
        for t in np.linspace(0, 1, max(2, int(L / (a.res / 4)) + 1)):
            px, py = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            i, j = int((px - ox) / a.res), H - 1 - int((py - oy) / a.res)
            if 0 <= i < W and 0 <= j < H:
                img[j, i] = 0
        n_wall += 1
out = HERE / 'config'
(out / f'{a.name}.pgm').write_bytes(b'P5\n%d %d\n255\n' % (W, H) + img.tobytes())
(out / f'{a.name}.yaml').write_text(yaml.safe_dump({'image': f'{a.name}.pgm', 'resolution': a.res, 'origin': ref['origin'], 'negate': 0,
                                                     'occupied_thresh': 0.65, 'free_thresh': 0.25, 'mode': 'trinary'}, sort_keys=False))
print(f'{W}x{H} 칸 {a.res} m, 점유 {int((img == 0).sum())}, 기둥 {n_post}, 벽 선분 {n_wall}')
