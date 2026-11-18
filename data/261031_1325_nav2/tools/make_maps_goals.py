"""고정 물체 지도와 모의 목표 만들기.
① map_objects.pgm/yaml: map_mission(SLAM 지도, 발제 좌표)과 같은 격자에 발제 배치(layout.json)의 고정 물체(상자 · 받침대 · 신호등 · 퍽)만 점유(0),
   나머지는 모름(128). costmap 둘째 정적 층(use_maximum)으로 쓰므로 모름 칸은 벽 지도를 덮지 않는다.
② goals.json: 출발 → 시작 상자 1 → L1 → L2 → 시작 상자 3 → L3 → 출발. 목표 자세 = 면 앞에서 범퍼(footprint 앞 0.263 m)가 면에서 standoff 만큼 떨어져 면을 봄.
   standoff 는 0.45 m(align.coarse_distance)부터 0.10 m 까지 줄이며 footprint 여유 ≥ min_clear 인 첫 값(0.30 m = drive coarse_min 미만이면 notes 에 남김). 장소 태그는 그 seed 의 태그 면,
   시작 상자는 네 면 중 가장 큰 standoff 가 되는 면."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--map_yaml', default='/home/pa4/git/lv2_next_module/261031_0335_slam_launch/test/results/bright/map/map_mission.yaml')
ap.add_argument('--out', default=str(HERE / 'config'))
ap.add_argument('--min_clear', type=float, default=0.05)
ap.add_argument('--rot_clear', type=float, default=None, help='주면: 그 자리 제자리 회전 원(외접 반경)과 장애물 사이 여유도 이 값 이상(설계 2)')
ap.add_argument('--min_standoff', type=float, default=0.10)
ap.add_argument('--goals_name', default='goals_seed1.json')
a = ap.parse_args()
w = geom.load_world()
lay = w['layout']
FRONT = geom.FOOTPRINT[1]

# ① 물체 지도
import yaml  # noqa: E402
my = yaml.safe_load(open(a.map_yaml))
pgm = Path(a.map_yaml).parent / my['image']
raw = pgm.read_bytes()
parts = raw.split(maxsplit=4)        # P5 W H MAX data
assert parts[0] == b'P5'
W, H = int(parts[1]), int(parts[2])
res, (ox, oy, _) = my['resolution'], my['origin']
gx = ox + (np.arange(W) + 0.5) * res
gy = oy + (np.arange(H) + 0.5) * res
X, Y = np.meshgrid(gx, gy[::-1])     # pgm 첫 줄 = 위(y 큼)
img = np.full((H, W), 128, np.uint8)
for o in w['objects']:
    c, s = math.cos(o['yaw']), math.sin(o['yaw'])
    lx = c * (X - o['cx']) + s * (Y - o['cy'])
    ly = -s * (X - o['cx']) + c * (Y - o['cy'])
    img[(np.abs(lx) <= o['hx'] + res / 2) & (np.abs(ly) <= o['hy'] + res / 2)] = 0
out = Path(a.out)
(out / 'map_objects.pgm').write_bytes(b'P5\n%d %d\n255\n' % (W, H) + img.tobytes())
(out / 'map_objects.yaml').write_text(yaml.safe_dump({'image': 'map_objects.pgm', 'resolution': res, 'origin': my['origin'], 'negate': 0,
                                                      'occupied_thresh': 0.65, 'free_thresh': 0.25, 'mode': 'trinary'}, sort_keys=False))


# ② 목표
R_CIRC = math.hypot(max(-geom.FOOTPRINT[0], geom.FOOTPRINT[1]), geom.FOOTPRINT[2])   # 제자리 회전 반경 0.314 m


def rot_clearance(x, y):
    return float(geom._point_seg_dist(x, y, w['body']).min() - R_CIRC)


def approach(name, fx, fy, nx, ny):
    for d in np.arange(0.45, a.min_standoff - 0.0001, -0.01):
        bx, by = fx + nx * (d + FRONT), fy + ny * (d + FRONT)
        yaw = math.atan2(-ny, -nx)
        c = geom.clearance(bx, by, yaw, w['body'])
        rc = rot_clearance(bx, by)
        if c >= a.min_clear and (a.rot_clear is None or rc >= a.rot_clear):
            return dict(name=name, x=round(bx, 4), y=round(by, 4), yaw=round(yaw, 5), standoff=round(float(d), 2), clearance=round(c, 4),
                        face=[round(fx, 4), round(fy, 4)], normal=[round(nx, 4), round(ny, 4)], rot_clearance=round(rc, 4))
    return None


boxes = {b['name']: b for b in lay['boxes']}
goals, notes = [], []


def box_goal(name, label):
    b = boxes[name]
    best = None
    for k in range(4):
        th = b['yaw'] + k * math.pi / 2
        nx, ny = math.cos(th), math.sin(th)
        half = b['size'][0 if k % 2 == 0 else 1] / 2
        g = approach(label, b['center'][0] + nx * half, b['center'][1] + ny * half, nx, ny)
        if g and (best is None or (g['standoff'], g['clearance']) > (best['standoff'], best['clearance'])):
            best = g
    return best


def tag_goal(place):
    t = next(t for t in lay['place_tags'] if t['place'] == place)
    if t['face'] == 'top':
        notes.append(f'{place} 태그가 윗면 → 상자 면 중 가장 큰 standoff')
        return box_goal(t['box'], place) if t['box'] in boxes else None
    nx, ny = t['normal'][:2]
    return approach(f'{place}(tag {t["id"]} {t["face"]})', t['center'][0], t['center'][1], nx, ny)


r = lay['robot']
start = dict(name='start', x=r['x'], y=r['y'], yaw=round(r['yaw'], 5))
for g in [box_goal('start_box_1', 'start_box_1'), tag_goal('L1'), tag_goal('L2'), box_goal('start_box_3', 'start_box_3'), tag_goal('L3'), start]:
    if g is None:
        notes.append('목표 못 만듦(목록에서 뺌)')
        continue
    elif g.get('standoff', 1) < 0.30:
        notes.append(f"{g['name']}: 범퍼-면 {g['standoff']} m < coarse_min 0.30 m (면 앞 칸이 차체 길이 0.418 m + 0.30 m 보다 좁음)")
    goals.append(g)
(out / a.goals_name).write_text(json.dumps({'layout': str(geom.LAYOUT), 'start': start, 'min_clear': a.min_clear, 'rot_clear': a.rot_clear, 'goals': goals, 'notes': notes},
                                                 ensure_ascii=False, indent=1))
for g in goals:
    print(g)
print(notes)
