#!/usr/bin/env python3
"""loc_cpp 비교 결과 (PC, 11-16). out/cmp bag 에서 Python 판(원래 토픽) · C++ 판(/cpp/…) 출력을 견준다.
  python3 compare.py [out/cmp]
- 상태(/localization/status) · lock: 값이 바뀐 시각 목록, 같은 순서인지, 바뀐 시각 차
- 다시 찾기(/initialpose): 횟수 · 시각 (C++ 은 AMCL 에 안 닿게 /cpp 로 돌렸음)
- 스캔(filtered 앞 · 뒤, merged): 같은 stamp 끼리 범위 배열이 같은지 (inf 끼리 같음, 1 mm 안)
"""
import math
import sys
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

URI = sys.argv[1] if len(sys.argv) > 1 else 'out/cmp'
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=URI, storage_id=''), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
msgs = defaultdict(list)
t0 = None
while rd.has_next():
    t, data, stamp = rd.read_next()
    t0 = stamp if t0 is None else t0
    msgs[t].append(((stamp - t0) * 1e-9, deserialize_message(data, get_message(types[t]))))


def changes(topic, f):
    out, last = [], object()
    for ts, m in msgs.get(topic, []):
        v = f(m)
        if v != last:
            out.append((ts, v))
            last = v
    return out


print(f'bag {URI}: ' + ', '.join(f'{k} {len(v)}' for k, v in sorted(msgs.items())))
for name, topic, f in [('상태', '/localization/status', lambda m: m.data), ('lock', '/localization/lock', lambda m: m.data)]:
    a, b = changes(topic, f), changes('/cpp' + topic, f)
    sa, sb = [v for _, v in a], [v for _, v in b]
    print(f'\n== {name}: Python {len(a)} 번 바뀜 · C++ {len(b)} 번')
    print('  값 순서 같음' if sa == sb else '  값 순서 다름')
    for i in range(max(len(a), len(b))):
        pa = f'{a[i][0]:7.1f} s {a[i][1]}' if i < len(a) else '-'
        pb = f'{b[i][0]:7.1f} s {b[i][1]}' if i < len(b) else '-'
        d = f'{b[i][0] - a[i][0]:+.2f} s' if i < len(a) and i < len(b) and a[i][1] == b[i][1] else ''
        print(f'  {i:2d} | Py {pa:<28} | C++ {pb:<28} | {d}')
for topic in ['/initialpose']:
    a, b = msgs.get(topic, []), msgs.get('/cpp' + topic, [])
    print(f'\n== 다시 찾기 {topic}: Python {len(a)} 번 {[round(t, 1) for t, _ in a]} · C++ {len(b)} 번 {[round(t, 1) for t, _ in b]}')
for topic in ['/scan_front_filtered', '/scan_rear_filtered', '/scan_merged']:
    A = {(m.header.stamp.sec, m.header.stamp.nanosec): m for _, m in msgs.get(topic, [])}
    Bm = {(m.header.stamp.sec, m.header.stamp.nanosec): m for _, m in msgs.get('/cpp' + topic, [])}
    keys = sorted(set(A) & set(Bm))
    same = 0
    worst = 0.0
    for k in keys:
        ra, rb = np.array(A[k].ranges, float), np.array(Bm[k].ranges, float)
        if len(ra) != len(rb):
            continue
        fa, fb = np.isfinite(ra), np.isfinite(rb)
        if (fa != fb).any():
            worst = max(worst, float((fa != fb).mean()))
            continue
        diff = np.abs(ra[fa] - rb[fb]).max() if fa.any() else 0.0
        if diff <= 0.001:
            same += 1
        else:
            worst = max(worst, diff)
    print(f'\n== {topic}: Py {len(A)} · C++ {len(Bm)} · 같은 stamp {len(keys)} · 범위 같음 {same} ({100 * same / max(1, len(keys)):.1f} %)')
