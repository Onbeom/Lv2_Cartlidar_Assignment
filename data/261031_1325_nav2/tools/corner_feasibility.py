"""출발 통로 ㄱ자 모퉁이(남향 통로 → wall_d0 끝 기둥 아래 동쪽 틈)를 차체 footprint 그대로 지나는 길이 있는지, 있으면 가장 좁은 곳 여유가
가장 큰 길(병목 최대 경로)을 찾는다(기하 계산, 모의 참값 형상 = geom, 기둥 포함).
동작: 전진 · 후진 step m, 회전 중심(x_icr)을 축으로 ±dth 회전. 상태는 (x, y, θ) 를 칸(xy_cell, dth)으로 반올림해 방문 표시(Hybrid-A* 와 같은 방식).
사용: python3 corner_feasibility.py [--start x y yaw] [--goal_xmin 1.25] [--out corner_feasibility.json]"""
import argparse
import heapq
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--start', type=float, nargs=3, default=[0.81, 0.75, -math.pi / 2])
ap.add_argument('--goal_xmin', type=float, default=1.25)
ap.add_argument('--goal_yaw_tol', type=float, default=math.radians(15))
ap.add_argument('--box', type=float, nargs=4, default=[0.50, 1.45, -0.02, 0.95], help='탐색 범위 x0 x1 y0 y1(base_link)')
ap.add_argument('--step', type=float, default=0.02)
ap.add_argument('--dth', type=float, default=math.radians(5))
ap.add_argument('--xy_cell', type=float, default=0.01)
ap.add_argument('--x_icr', type=float, default=-0.0786)
ap.add_argument('--out', default=str(HERE / 'tools' / 'corner_feasibility.json'))
a = ap.parse_args()
B = geom.load_world()['body']
XI = a.x_icr
cache = {}


def key(x, y, t):
    return (round(x / a.xy_cell), round(y / a.xy_cell), round((t % (2 * math.pi)) / a.dth) % round(2 * math.pi / a.dth))


def clear(x, y, t):
    k = key(x, y, t)
    if k not in cache:
        cache[k] = geom.clearance(x, y, t, B)
    return cache[k]


def moves(x, y, t):
    c, s = math.cos(t), math.sin(t)
    for d in (a.step, -a.step):
        yield x + d * c, y + d * s, t, 'fwd' if d > 0 else 'back'
    rx, ry = x + XI * c, y + XI * s
    for d in (a.dth, -a.dth):
        t2 = t + d
        yield rx - XI * math.cos(t2), ry - XI * math.sin(t2), t2, 'rot+' if d > 0 else 'rot-'


x0, y0, t0 = a.start
c0 = clear(x0, y0, t0)
best = {key(x0, y0, t0): c0}
heap = [(-c0, x0, y0, t0)]
parent = {key(x0, y0, t0): None}
found = None
n = 0
while heap:
    nc, x, y, t = heapq.heappop(heap)
    bott = -nc
    k = key(x, y, t)
    if best.get(k, -9) > bott + 1e-12:
        continue
    n += 1
    if x >= a.goal_xmin and abs(math.atan2(math.sin(t), math.cos(t))) <= a.goal_yaw_tol:
        found = (bott, x, y, t)
        break
    for x2, y2, t2, m in moves(x, y, t):
        if not (a.box[0] <= x2 <= a.box[1] and a.box[2] <= y2 <= a.box[3]):
            continue
        c2 = clear(x2, y2, t2)
        if c2 < 0:
            continue
        b2 = min(bott, c2)
        k2 = key(x2, y2, t2)
        if b2 > best.get(k2, -9) + 1e-12:
            best[k2] = b2
            parent[k2] = (k, m, (round(x, 4), round(y, 4), round(t, 4)))
            heapq.heappush(heap, (-b2, x2, y2, t2))
res = {'start': a.start, 'start_clearance': round(c0, 4), 'expanded': n, 'clearance_evals': len(cache)}
if found:
    bott, x, y, t = found
    path, k = [(round(x, 4), round(y, 4), round(t, 4), 'goal')], key(x, y, t)
    while parent.get(k):
        pk, m, pose = parent[k]
        path.append((*pose, m))
        k = pk
    path.reverse()
    worst = min(path, key=lambda p: clear(p[0], p[1], p[2]))
    res.update(feasible=True, bottleneck_clearance_m=round(bott, 4), n_steps=len(path), worst_pose=worst, path=path)
else:
    res.update(feasible=False)
Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps({k: v for k, v in res.items() if k != 'path'}, ensure_ascii=False))
