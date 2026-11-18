"""Nav2 용 벽 지도 복사본: SLAM 지도(map_mission)의 출발 자리에 찍힌 자기 차체(마스크가 고정 칸을 배우기 전 첫 스캔)를 지운다.
출발 자세의 footprint + pad 안 칸을 빈칸(254)으로 바꿔 map_mission_nav.pgm/yaml 로 쓴다. 원본은 그대로. 지울 근거: 지도 만들 때 로봇이 그 자리에 있었으므로 다른 물체가 있을 수 없음."""
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
ap.add_argument('--map_yaml', default=str(HERE / 'config' / 'map_mission.yaml'))
ap.add_argument('--start', type=float, nargs=3, default=[0.734, 0.95, math.pi])
ap.add_argument('--pad', type=float, default=0.02)
a = ap.parse_args()
my = yaml.safe_load(open(a.map_yaml))
src = Path(a.map_yaml).parent / my['image']
parts = src.read_bytes().split(maxsplit=4)
W, H = int(parts[1]), int(parts[2])
img = np.frombuffer(parts[4][:W * H], np.uint8).reshape(H, W).copy()
res, (ox, oy, _) = my['resolution'], my['origin']
X, Y = np.meshgrid(ox + (np.arange(W) + 0.5) * res, (oy + (np.arange(H) + 0.5) * res)[::-1])
x, y, yaw = a.start
c, s = math.cos(yaw), math.sin(yaw)
lx, ly = c * (X - x) + s * (Y - y), -s * (X - x) + c * (Y - y)
fp = geom.FOOTPRINT
inside = (lx >= fp[0] - a.pad) & (lx <= fp[1] + a.pad) & (np.abs(ly) <= fp[2] + a.pad)
occ_before = int(((img < 255 * (1 - my['occupied_thresh'])) & inside).sum())
img[inside] = 254
out = Path(a.map_yaml).parent
(out / 'map_mission_nav.pgm').write_bytes(b'P5\n%d %d\n255\n' % (W, H) + img.tobytes())
my2 = dict(my, image='map_mission_nav.pgm')
(out / 'map_mission_nav.yaml').write_text(yaml.safe_dump(my2, sort_keys=False))
print(f'출발 footprint 칸 {int(inside.sum())}개 지움(그중 점유였던 칸 {occ_before})')
