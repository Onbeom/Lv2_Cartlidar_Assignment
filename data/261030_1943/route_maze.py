"""새 미로(maze.zip) 벽 단면으로 녹화 경로를 만든다(v2). v1(격자 마디 이웃만 잇기)은 마디 6개만 이어져 실패 → route_maze_v1_grid_fail.*.
방법: 2 cm 격자 벽 여유 지도 → 덮을 마디(가로 0.734 · 세로 0.80 격자를 벽에서 떨어진 곳으로 옮김) → 출발 마디에서 가장 가까운(경로 거리)
안 간 마디를 차례로 방문 → 출발로 복귀. 마디 사이는 여유 ≥ EDGE_CLR 칸만 지나는 최단 경로를 직선 시야로 줄인 경유점.
로봇 크기(새 모델 sensors.json base_boxes + 바퀴): 반폭 약 0.171 m, 제자리 회전 반경 약 0.21 m."""
import heapq
import json
import os
import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
seg = json.load(open(f'{HERE}/maze_slice_z0.20.json'))['segments']
NODE_CLR, EDGE_CLR, RES = 0.30, 0.30, 0.02   # v4: v3(0.24, 0.20) 은 꺾이는 점 여유 0.20 m 에서 제자리 회전 중 벽 끝에 걸림(본 녹화 경유점 47)
# v5: 회전하는 점(마디 · 꺾이는 점) 여유 ≥ 0.30 m. drive 자세 충돌 상자 회전 반경 0.263 m(그리퍼, URDF FK) + 회전 중심 밀림 약 4 cm.
#     v4 는 꺾이는 점 여유 0.25 m 에서 180° 회전 중 그리퍼가 e0 벽에 걸림(경유점 71)
TURN_CLR = NODE_CLR   # v5 최종: 통로도 0.30(0.25 로는 벽 끝 둘레 꺾이는 점 45개가 0.30 미만)
# 발제 원점(왼쪽 아래 기둥 중심) = USD (-2.2345, -3.2136) (바깥 모서리 profile 기둥 중심, 회전 0.05° 이하 무시). 발제 = USD + (2.2345, 3.2136)
MISSION_ORIGIN_USD = (-2.2345, -3.2136)
START = (0.734 + MISSION_ORIGIN_USD[0], 0.950 + MISSION_ORIGIN_USD[1])   # 발제 출발 (734, 950) mm (사용자 확인 2026-10-30: 벽 외 좌표는 발제 원점 기준 그대로)
x0, y0, x1, y1 = -2.4, -3.4, 2.4, 3.4
nx, ny = int((x1 - x0) / RES) + 1, int((y1 - y0) / RES) + 1
occ = np.zeros((nx, ny), bool)
for _, a, b in seg:
    a, b = np.array(a), np.array(b)
    for p in a + np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.005) + 1))[:, None] * (b - a):
        occ[int(round((p[0] - x0) / RES)), int(round((p[1] - y0) / RES))] = True
clr = distance_transform_edt(~occ) * RES
# 바깥 벽 밖은 제외: 안쪽 한 점에서 여유>0 으로 이어진 칸만
inside = np.zeros_like(occ)
from scipy.ndimage import label
lab, _ = label(clr > RES)
inside = lab == lab[int((0 - x0) / RES), int((0 - y0) / RES)]
ok = inside & (clr >= EDGE_CLR)


def ij(p):
    return int(round((p[0] - x0) / RES)), int(round((p[1] - y0) / RES))


def xy(c):
    return (round(x0 + c[0] * RES, 3), round(y0 + c[1] * RES, 3))


def dijkstra(src):
    dist = np.full(ok.shape, np.inf); prev = {}
    dist[src] = 0; h = [(0.0, src)]
    while h:
        d, c = heapq.heappop(h)
        if d > dist[c]:
            continue
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == dj == 0:
                    continue
                n = (c[0] + di, c[1] + dj)
                if 0 <= n[0] < nx and 0 <= n[1] < ny and ok[n]:
                    nd = d + RES * (1.4142 if di and dj else 1.0)
                    if nd < dist[n]:
                        dist[n] = nd; prev[n] = c; heapq.heappush(h, (nd, n))
    return dist, prev


def visible(a, b):
    n = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1])) + 1)
    return all(ok[int(round(a[0] + t * (b[0] - a[0]))), int(round(a[1] + t * (b[1] - a[1])))] for t in np.linspace(0, 1, n))


def path_to(prev, src, dst):
    p = [dst]
    while p[-1] != src:
        p.append(prev[p[-1]])
    p = p[::-1]
    out, k = [p[0]], 0
    while k < len(p) - 1:
        j = len(p) - 1
        while j > k + 1 and not (visible(p[k], p[j]) and (j == len(p) - 1 or clr[p[j]] >= TURN_CLR)):
            j -= 1
        if j == k + 1 and not (j == len(p) - 1 or clr[p[j]] >= TURN_CLR):   # 회전 여유 칸이 안 보이면 보이는 가장 먼 칸
            j = max(i for i in range(k + 1, len(p)) if visible(p[k], p[i]))
        out.append(p[j]); k = j
    return out


nodes = []
for x in [-1.835 + 0.734 * i for i in range(6)]:
    for y in [-2.80 + 0.80 * j for j in range(8)]:
        best = None
        for dx in np.arange(-0.30, 0.301, RES * 2):
            for dy in np.arange(-0.30, 0.301, RES * 2):
                c = ij((x + dx, y + dy))
                if 0 <= c[0] < nx and 0 <= c[1] < ny and inside[c] and clr[c] >= NODE_CLR:
                    k = np.hypot(dx, dy)
                    if best is None or k < best[0]:
                        best = (k, c)
        if best:
            nodes.append(best[1])
s = ij(START)                          # v3: 실제 출발 위치를 첫 마디로(v2 는 가장 가까운 격자 마디)
assert ok[s], '출발 위치 여유 부족'
nodes = [n for n in nodes if n != s] + [s]
todo = set(nodes) - {s}
cur, wps, unreachable = s, [xy(s)], []
while todo:
    dist, prev = dijkstra(cur)
    reach = [c for c in todo if np.isfinite(dist[c])]
    unreachable += [xy(c) for c in todo if not np.isfinite(dist[c])]
    todo = set(reach)
    if not todo:
        break
    nxt = min(todo, key=lambda c: (dist[c], c))
    wps += [xy(c) for c in path_to(prev, cur, nxt)[1:]]
    todo.discard(nxt); cur = nxt
dist, prev = dijkstra(cur)
wps += [xy(c) for c in path_to(prev, cur, s)[1:]]
length = float(sum(np.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(wps, wps[1:])))
out = {'version': 5, 'turn_clearance_m': TURN_CLR, 'mission_origin_usd': MISSION_ORIGIN_USD, 'start_clearance_m': float(clr[s]), 'start_assumed': START, 'start_node': xy(s), 'node_clearance_m': NODE_CLR, 'edge_clearance_m': EDGE_CLR,
       'nodes': len(nodes), 'unreachable_nodes': unreachable, 'waypoints': wps, 'n_waypoints': len(wps), 'length_m': length,
       'min_waypoint_clearance': float(min(clr[ij(p)] for p in wps)), 'waypoints_below_turn_clr': [p for p in wps if clr[ij(p)] < TURN_CLR]}
json.dump(out, open(f'{HERE}/route_maze.json', 'w'), indent=1)
print({k: v for k, v in out.items() if k != 'waypoints'})
