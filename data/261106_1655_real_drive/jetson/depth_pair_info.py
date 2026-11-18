#!/usr/bin/env python3
"""D435 컬러 · 정렬 깊이 stamp 짝 (읽기만, camera_info 만 받아 가벼움). depth_pair_info.py <카메라 이름> <초> [허용 s]

컬러 stamp 마다 같은 stamp(허용 안)의 정렬 깊이가 몇 장인지 센다 (0 장 = 인지 노드에서 '깊이 짝 없음').
같은 컬러 stamp 를 받은 정렬 깊이가 2 장 이상이면 정렬이 직전 컬러를 다시 쓴 것.
1 s 칸마다 짝 없음 비율도 내서 시간에 따라 바뀌는지 본다.
"""
import collections
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

cam, secs = sys.argv[1], float(sys.argv[2])
tol = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02
rclpy.init()
node = rclpy.create_node('depth_pair_info')
col, dep = [], []


def ns(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


node.create_subscription(CameraInfo, f'/{cam}/color/camera_info', lambda m: col.append(ns(m)), qos_profile_sensor_data)
node.create_subscription(CameraInfo, f'/{cam}/aligned_depth_to_color/camera_info', lambda m: dep.append(ns(m)), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()

col = sorted(set(col))[3:-3]
dcount = collections.Counter(dep)
tol_ns = int(tol * 1e9)
dsorted = sorted(dcount)
import bisect
n_by = collections.Counter()
per_sec = collections.defaultdict(lambda: [0, 0])
for c in col:
    i = bisect.bisect_left(dsorted, c - tol_ns)
    n = 0
    while i < len(dsorted) and dsorted[i] <= c + tol_ns:
        n += dcount[dsorted[i]]
        i += 1
    n_by[min(n, 3)] += 1
    b = per_sec[(c - col[0]) // 10**9]
    b[0] += 1
    b[1] += n == 0
print(f'{cam}: 컬러 {len(col)} 장 ({len(col) / secs:.1f} Hz), 정렬 깊이 {len(dep)} 장 ({len(dep) / secs:.1f} Hz), {secs:.0f} s')
print(f'  정렬 깊이 stamp 중 컬러 stamp 와 정확히 같은 것: {100 * sum(v for k, v in dcount.items() if k in set(col)) / max(1, len(dep)):.1f} %')
print(f'  같은 stamp 정렬 깊이 2 장 이상: {sum(1 for v in dcount.values() if v >= 2)} 개 stamp')
for k in range(4):
    print(f'  컬러 1 장에 짝 깊이 {k}{"+" if k == 3 else ""} 장: {n_by[k]} ({100 * n_by[k] / max(1, len(col)):.1f} %)')
print('  1 s 칸별 짝 없음 %: ' + ' '.join(f'{100 * b[1] / b[0]:.0f}' for _, b in sorted(per_sec.items())))
