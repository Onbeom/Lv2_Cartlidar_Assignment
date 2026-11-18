"""새 미로(maze.zip) 벽 단면으로 녹화 경로를 만든다. 격자 마디(가로 0.734 · 세로 0.80 간격)를 벽에서 떨어진 곳으로 옮기고,
선분이 벽에서 충분히 떨어진 이웃끼리만 잇는다. 출발 마디에서 깊이 우선으로 모두 돌고 돌아온다(이웃 순서 고정 → 늘 같은 경로).
사용: python route_maze.py  → route_maze.json (경유점, 길이, 여유)"""
import json
import numpy as np
from scipy.spatial import cKDTree

import os
HERE = os.path.dirname(os.path.abspath(__file__))
seg = json.load(open(f'{HERE}/maze_slice_z0.20.json'))['segments']
pts = []
for _, a, b in seg:
    a, b = np.array(a), np.array(b)
    n = max(2, int(np.linalg.norm(b - a) / 0.01) + 1)
    pts += list(a + np.linspace(0, 1, n)[:, None] * (b - a))
tree = cKDTree(np.array(pts))
NODE_CLR, EDGE_CLR = 0.25, 0.22        # [m] 마디(제자리 회전) · 선분(직진) 벽 여유
START = (-1.468, -2.202)               # 이전 발제 출발 (734, 950) mm 를 가운데 맞춤으로 옮긴 값(가정)
xs = [-1.835 + 0.734 * i for i in range(6)]
ys = [-2.80 + 0.80 * j for j in range(8)]


def clr(p):
    return float(tree.query(p)[0])


def seg_clr(a, b):
    a, b = np.array(a), np.array(b)
    n = max(2, int(np.linalg.norm(b - a) / 0.02) + 1)
    return float(tree.query(a + np.linspace(0, 1, n)[:, None] * (b - a))[0].min())


nodes = {}
for i, x in enumerate(xs):
    for j, y in enumerate(ys):
        best = None
        for dx in np.arange(-0.24, 0.241, 0.04):
            for dy in np.arange(-0.24, 0.241, 0.04):
                c = clr((x + dx, y + dy))
                key = (c >= NODE_CLR, -np.hypot(dx, dy), c)
                if best is None or key > best[0]:
                    best = (key, (round(x + dx, 3), round(y + dy, 3)), c)
        if best[0][0]:
            nodes[(i, j)] = best[1]
start = min(nodes, key=lambda k: np.hypot(nodes[k][0] - START[0], nodes[k][1] - START[1]))
STEP = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def nbrs(k):
    return [(k[0] + di, k[1] + dj) for di, dj in STEP if (k[0] + di, k[1] + dj) in nodes
            and seg_clr(nodes[k], nodes[(k[0] + di, k[1] + dj)]) >= EDGE_CLR]


seen, order = {start}, [start]


def visit(k):
    for nb in nbrs(k):
        if nb not in seen:
            seen.add(nb); order.append(nb); visit(nb); order.append(k)


visit(start)
wps = [list(nodes[k]) for k in order]
length = float(sum(np.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(wps, wps[1:])))
out = {'start_assumed': START, 'start_node': nodes[start], 'node_clearance_m': NODE_CLR, 'edge_clearance_m': EDGE_CLR,
       'nodes_total': len(xs) * len(ys), 'nodes_kept': len(nodes), 'visited': len(seen), 'waypoints': wps, 'length_m': length,
       'min_node_clearance': min(clr(p) for p in wps), 'min_edge_clearance': min(seg_clr(a, b) for a, b in zip(wps, wps[1:])),
       'unvisited_kept': [nodes[k] for k in nodes if k not in seen]}
json.dump(out, open(f'{HERE}/route_maze.json', 'w'), indent=1)
print({k: v for k, v in out.items() if k != 'waypoints'})
