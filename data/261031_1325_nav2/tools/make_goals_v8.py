"""설계 8 목표: L1 자세를 '실제로 도는 구간'의 footprint 여유로 다시 고른다(설계 7 반복 r3 실패 대응).
설계 4 규칙(카메라 0.35~0.55 m, 법선 ≤ 15°, footprint 여유 ≥ 0.05)은 그대로, 360° 회전 여유 ≥ 0.03 대신
들어올 때(+y → 태그 방향, 반시계) · 나갈 때(태그 방향 → −y, 반시계) 두 1/4 회전의 최소 footprint 여유 ≥ 0.07 m
(= 반복 3회에서 본 위치 추정 최대 오차 0.066 m 이상, L1 상자는 라이다에 안 보여 추정 오차가 그대로 여유를 먹음).
그중 |거리 − 0.45| + 각/90 최소. 나머지 목표는 --base 그대로.
설계 9: 같은 규칙을 L3 에도(--place L3 --base goals_seed1_v8.json --out goals_seed1_v9.json, L3 도 +y 로 들어와 −y 로 나감 — v8 참값 경로로 확인)."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / 'nodes'))
import geom  # noqa: E402

CAM_X, D_RANGE, D_BEST, MAX_OFF, MIN_CLEAR, SWEEP_CLEAR = -0.0606, (0.35, 0.55), 0.45, 15.0, 0.05, 0.07
ap = argparse.ArgumentParser()
ap.add_argument('--place', default='L1')
ap.add_argument('--base', default='goals_seed1_v6.json')
ap.add_argument('--out', default='goals_seed1_v8.json')
ap.add_argument('--yaw_in', type=float, default=math.pi / 2, help='들어올 때 차체 방향')
ap.add_argument('--yaw_out', type=float, default=1.5 * math.pi, help='나갈 때 차체 방향(반시계로 이어지게)')
a = ap.parse_args()
w = geom.load_world()
B = w['body']


def sweep(x, y, a, b, n=60):
    return min(geom.clearance(x, y, a + (b - a) * i / n, B) for i in range(n + 1))


t = next(t for t in w['layout']['place_tags'] if t['place'] == a.place)
tx, ty = t['center'][:2]
nx, ny = t['normal'][:2]
best = None
cx, cy = tx + 0.4 * nx, ty + 0.4 * ny                      # 태그 앞 0.4 m 둘레만 찾음
for x in np.arange(cx - 0.15, cx + 0.15, 0.005):
    for y in np.arange(cy - 0.15, cy + 0.15, 0.005):
        r = math.hypot(tx - x, ty - y)
        d = r - CAM_X
        if not D_RANGE[0] <= d <= D_RANGE[1]:
            continue
        off = math.degrees(math.acos(max(-1.0, min(1.0, ((x - tx) * nx + (y - ty) * ny) / r))))
        if off > MAX_OFF:
            continue
        yaw = math.atan2(ty - y, tx - x)
        c = geom.clearance(x, y, yaw, B)
        if c < MIN_CLEAR:
            continue
        s_in, s_out = sweep(x, y, a.yaw_in, yaw), sweep(x, y, yaw, a.yaw_out)
        if min(s_in, s_out) < SWEEP_CLEAR:
            continue
        score = abs(d - D_BEST) + off / 90
        if best is None or score < best[0]:
            best = (score, dict(name=f"{a.place}(tag {t['id']} {t['face']}, 회전 구간 규칙)", x=round(float(x), 4), y=round(float(y), 4), yaw=round(yaw, 5),
                                camera_dist=round(d, 3), off_normal_deg=round(off, 1), clearance=round(c, 4),
                                sweep_in=round(s_in, 4), sweep_out=round(s_out, 4)))
goals = json.loads((HERE / 'config' / a.base).read_text())['goals']
i = next(i for i, g in enumerate(goals) if g['name'].startswith(f'{a.place}(tag'))
old = goals[i]
goals[i] = best[1]
print('old', old)
print('new', best[1])
(HERE / 'config' / a.out).write_text(json.dumps({'layout': str(geom.LAYOUT), 'rule': __doc__, 'goals': goals}, ensure_ascii=False, indent=1))
