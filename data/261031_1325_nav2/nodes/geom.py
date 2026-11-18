"""모의 참값 형상(발제 좌표, m): 미로 벽 선분 · 고정 물체 · 퍽, footprint 와의 여유 계산. 모의기 · 목표 도구 · 채점이 같이 쓴다.
벽 = 261030_1943/maze_slice_z0.08.json(USD 좌표, 라이다 높이 단면) + usd_offset.
기둥 = 261030_1943/maze_posts.json(충돌 원기둥 반지름 0.0303 m, 높이 0~0.39, 단면 자료엔 Mesh 만 있어 빠졌던 것, 261101 추가) — 외접 16각형 선분, 라이다 · 몸체 둘 다. 물체 = 발제 배치 layout.json(boxes · signal_light · pucks).
라이다(높이 약 0.082 m)에 보이는 것 = 벽 · 신호등(0.177 m) · 퍽(윗면 0.13 m). 상자 · 받침대(0.07 m)는 라이다에 안 보이고 몸체 충돌에만 넣는다."""
import json
import math
from pathlib import Path

import numpy as np

WALLS = Path('/home/pa4/git/lv2_next_module/261030_1943/maze_slice_z0.08.json')
POSTS = Path('/home/pa4/git/lv2_next_module/261030_1943/maze_posts.json')
LAYOUT = Path('/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json')
USD_OFFSET = (2.2345, 3.2136)
FOOTPRINT = (-0.155, 0.263, 0.171)     # base_link 기준 x 뒤 · 앞, |y| (drive 그리퍼 포함, 261030_2202 S3)
SIGNAL_HALF = 0.03                      # 신호등 기둥 반폭 가정(도면에 폭 없음, 머리 지름 0.06 과 같게)


def rect_segments(cx, cy, hx, hy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    return [(pts[i], pts[(i + 1) % 4]) for i in range(4)]


def post_segments(cx, cy, r, n=16):
    R = r / math.cos(math.pi / n)                    # 외접 다각형(원보다 작게 잡지 않음)
    pts = [(cx + R * math.cos(2 * math.pi * (k + 0.5) / n), cy + R * math.sin(2 * math.pi * (k + 0.5) / n)) for k in range(n)]
    return [(pts[k][0], pts[k][1], pts[(k + 1) % n][0], pts[(k + 1) % n][1]) for k in range(n)]


def load_world(layout=LAYOUT, walls=WALLS, posts=POSTS):
    """반환: dict(lidar=선분 배열 (N,4) 라이다에 보이는 것, body=선분 배열 몸체 충돌용, objects=물체 목록(지도 칸 그리기용))."""
    w = json.loads(Path(walls).read_text())
    ox, oy = USD_OFFSET
    wall = [(a[0] + ox, a[1] + oy, b[0] + ox, b[1] + oy) for _, a, b in w['segments']]
    if posts:
        seen = set()
        for q in json.loads(Path(posts).read_text())['posts']:
            k = (round(q['center'][0], 4), round(q['center'][1], 4))
            if k not in seen:
                seen.add(k)
                wall += post_segments(q['center'][0] + ox, q['center'][1] + oy, q['radius'])
    lay = json.loads(Path(layout).read_text())
    objects, low, high = [], [], []
    for b in lay['boxes']:
        cx, cy, _ = b['center']
        o = dict(name=b['name'], cx=cx, cy=cy, hx=b['size'][0] / 2, hy=b['size'][1] / 2, yaw=b.get('yaw', 0.0), height=b['size'][2])
        objects.append(o)
    sl = lay['signal_light']
    objects.append(dict(name='signal_light', cx=sl['x'], cy=sl['y'], hx=SIGNAL_HALF, hy=SIGNAL_HALF, yaw=sl.get('yaw', 0.0), height=sl['height']))
    for p in lay['pucks']:
        (x0, y0, z0), (x1, y1, z1) = p['aabb']
        objects.append(dict(name=p['name'], cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, hx=(x1 - x0) / 2, hy=(y1 - y0) / 2, yaw=0.0, height=z1, puck=True))
    for o in objects:
        segs = [(a[0], a[1], b[0], b[1]) for a, b in rect_segments(o['cx'], o['cy'], o['hx'], o['hy'], o['yaw'])]
        (high if o['height'] > 0.082 else low).extend(segs)
    return dict(lidar=np.array(wall + high), body=np.array(wall + high + low), objects=objects, layout=lay)


def footprint_polygon(x, y, yaw, fp=FOOTPRINT):
    return [(a[0], a[1]) for a, _ in rect_segments(x + math.cos(yaw) * (fp[0] + fp[1]) / 2, y + math.sin(yaw) * (fp[0] + fp[1]) / 2,
                                                    (fp[1] - fp[0]) / 2, fp[2], yaw)]


def _point_seg_dist(px, py, segs):
    ax, ay, bx, by = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    dx, dy = bx - ax, by - ay
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / np.maximum(dx * dx + dy * dy, 1e-12), 0, 1)
    return np.hypot(ax + t * dx - px, ay + t * dy - py)


def _inside(poly, px, py):
    """점들(px, py 배열)이 반시계 볼록 다각형 poly(rect_segments 순서) 안이면 True."""
    res = np.ones(len(px), bool)
    for i in range(len(poly)):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % len(poly)]
        res &= (x1 - x0) * (py - y0) - (y1 - y0) * (px - x0) >= 0
    return res


def clearance(x, y, yaw, segs, fp=FOOTPRINT):
    """footprint 사각형과 선분들 사이 최소 거리(m). 겹치면 음수(−0.001 고정)."""
    poly = footprint_polygon(x, y, yaw, fp)
    # 근처 선분만
    r = math.hypot(max(abs(fp[0]), fp[1]), fp[2]) + 1.0
    near = segs[(np.minimum(segs[:, 0], segs[:, 2]) < x + r) & (np.maximum(segs[:, 0], segs[:, 2]) > x - r) &
                (np.minimum(segs[:, 1], segs[:, 3]) < y + r) & (np.maximum(segs[:, 1], segs[:, 3]) > y - r)]
    if len(near) == 0:
        return 1.0
    if _inside(poly, near[:, 0], near[:, 1]).any() or _inside(poly, near[:, 2], near[:, 3]).any():
        return -0.001
    d = min(_point_seg_dist(px, py, near).min() for px, py in poly)
    # 선분 끝점 → 다각형 변 거리
    edges = np.array([(poly[i][0], poly[i][1], poly[(i + 1) % 4][0], poly[(i + 1) % 4][1]) for i in range(4)])
    for qx, qy in np.concatenate([near[:, :2], near[:, 2:]]):
        d = min(d, _point_seg_dist(qx, qy, edges).min())
    # 변이 서로 교차(끝점이 밖에 있어도)
    for e in edges:
        if _intersects(e, near).any():
            return -0.001
    return float(d)


def _intersects(e, segs):
    def cross(ax, ay, bx, by, cx, cy):
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    x0, y0, x1, y1 = e
    d1 = cross(x0, y0, x1, y1, segs[:, 0], segs[:, 1])
    d2 = cross(x0, y0, x1, y1, segs[:, 2], segs[:, 3])
    d3 = cross(segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3], x0, y0)
    d4 = cross(segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3], x1, y1)
    return (d1 * d2 < 0) & (d3 * d4 < 0)


def raycast(ox, oy, dirs, segs, rmax):
    """원점 (ox, oy) 에서 방향 dirs (N,2) 로 선분들까지 거리. 안 맞으면 inf."""
    ax, ay = segs[:, 0] - ox, segs[:, 1] - oy
    ex, ey = segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]
    dx, dy = dirs[:, 0:1], dirs[:, 1:2]
    den = dx * ey - dy * ex                       # (N, M)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * ey - ay * ex) / den             # 광선 거리
        u = (ax * dy - ay * dx) / den             # 선분 위치
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t <= rmax)
    t = np.where(hit, t, np.inf)
    return t.min(axis=1)
