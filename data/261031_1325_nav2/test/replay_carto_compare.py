"""도구 확인(판정 아님): /tracked_pose(Cartographer map 좌표)를 발제 좌표로 옮겨 녹화 참값 /sim/truth 와 비교한다.
원래 실시간 녹화(<결과>/out_bag)와 replay_carto.sh 재생 결과(<재생>/replay_bag)를 같은 방식으로 셈.
값: 위치 오차 RMSE · 최대 · p99, 구간 [t0, t1](sim 초) 안 최대, 오차가 2 s 안에 0.05 m 넘게 바뀐 '튐' 횟수와 시각.
사용: python3 replay_carto_compare.py <결과 폴더> <재생 폴더...> [--win t0 t1] → 각 재생 폴더/compare.json, 표준 출력 표"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/pa4/git/lv2_next_module/261101_0016_isaac_live')
import bagutil as m  # noqa: E402

ALIGN = (0.729, 0.960, 3.143774)   # mission_from_carto_map(live_bright/map/align.json), launch 기본값과 같음
ap = argparse.ArgumentParser()
ap.add_argument('src')
ap.add_argument('replays', nargs='*')
ap.add_argument('--win', type=float, nargs=2, default=[400.0, 430.0])
a = ap.parse_args()
SRC = Path(a.src)
truth = np.array([(m.stamp(x.header), x.pose.position.x, x.pose.position.y) for _, x, _ in m.read_bag(str(SRC / 'out_bag'), ['/sim/truth'])])


def stats(bag):
    rows = []
    for _, x, _ in m.read_bag(bag, ['/tracked_pose']):
        t = m.stamp(x.header)
        p = m.compose(ALIGN, (x.pose.position.x, x.pose.position.y, m.yaw_of(x.pose.orientation)))
        rows.append((t, p[0], p[1]))
    e = np.array(rows)
    e = e[(e[:, 0] >= truth[0, 0]) & (e[:, 0] <= truth[-1, 0])]
    tx, ty = np.interp(e[:, 0], truth[:, 0], truth[:, 1]), np.interp(e[:, 0], truth[:, 0], truth[:, 2])
    d = np.hypot(e[:, 1] - tx, e[:, 2] - ty)
    w = (e[:, 0] >= a.win[0]) & (e[:, 0] <= a.win[1])
    jumps = []
    j = 0
    for i in range(len(e)):
        while e[i, 0] - e[j, 0] > 2.0:
            j += 1
        if abs(d[i] - d[j]) > 0.05 and (not jumps or e[i, 0] - jumps[-1] > 5.0):
            jumps.append(round(float(e[i, 0]), 2))
    k = int(np.argmax(d))
    return {'n': int(len(e)), 't_range': [round(float(e[0, 0]), 2), round(float(e[-1, 0]), 2)],
            'rmse': round(float(np.sqrt(np.mean(d ** 2))), 4), 'max': round(float(d[k]), 4), 't_max': round(float(e[k, 0]), 2),
            'p99': round(float(np.percentile(d, 99)), 4), f'win_{a.win[0]:g}_{a.win[1]:g}_max': round(float(d[w].max()), 4) if w.any() else None,
            'jumps_005_in_2s': jumps}


res = {'live': stats(str(SRC / 'out_bag'))}
for r in a.replays:
    s = stats(str(Path(r) / 'replay_bag'))
    res[Path(r).name] = s
    json.dump({'src': str(SRC), 'align': ALIGN, 'live': res['live'], 'replay': s}, open(Path(r) / 'compare.json', 'w'), indent=1, ensure_ascii=False)
for k, v in res.items():
    print(k, json.dumps(v, ensure_ascii=False))
