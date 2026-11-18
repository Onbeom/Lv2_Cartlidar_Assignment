#!/usr/bin/env python3
"""D435 컬러 · 정렬 깊이 짝 도착 지연 (읽기만). depth_pair_lag.py <카메라 이름> <초> [허용 s]

컬러 프레임마다 stamp 차이가 허용(기본 0.02 s) 안인 깊이를 찾아
  - 짝 없음: 끝까지 그런 깊이가 안 옴 (카메라가 안 보냄 · 놓침)
  - 늦은 짝: 깊이가 컬러보다 늦게 도착 → 도착 지연(깊이 받은 시각 − 컬러 받은 시각) 분포
를 낸다. 인지 노드는 처리 주기(약 15 Hz)마다 컬러 큐를 비우므로, 지연이 길면 최근 컬러가 깊이 없이 버려진다.
"""
import statistics
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

cam, secs = sys.argv[1], float(sys.argv[2])
tol = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02
rclpy.init()
node = rclpy.create_node('depth_pair_lag')
col, dep = [], []   # (stamp s, 받은 시각 s)


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


node.create_subscription(Image, f'/{cam}/color/image_raw', lambda m: col.append((st(m), time.monotonic())), qos_profile_sensor_data)
node.create_subscription(Image, f'/{cam}/aligned_depth_to_color/image_raw', lambda m: dep.append((st(m), time.monotonic())), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()

col = col[5:-5]   # 앞뒤 자르기 (구독 시작 · 끝 경계)
lags, none, diffs = [], 0, []
for cs, cr in col:
    cand = [(abs(ds - cs), ds, dr) for ds, dr in dep if abs(ds - cs) <= tol]
    if not cand:
        none += 1
        continue
    d, ds, dr = min(cand)
    diffs.append((ds - cs) * 1e3)
    lags.append((dr - cr) * 1e3)


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float('nan')


print(f'{cam}: 컬러 {len(col)} 장, 깊이 {len(dep)} 장, {secs:.0f} s')
print(f'  컬러 Hz {len(col) / secs:.1f}, 깊이 Hz {len(dep) / secs:.1f}')
print(f'  짝 없음(허용 {tol * 1e3:.0f} ms 안 깊이 없음) {none} 장 = {100 * none / max(1, len(col)):.1f} %')
if lags:
    print(f'  stamp 차(깊이 − 컬러) ms: 중앙 {statistics.median(diffs):.1f}, 최소 {min(diffs):.1f}, 최대 {max(diffs):.1f}')
    print(f'  도착 지연(깊이 받음 − 컬러 받음) ms: 중앙 {statistics.median(lags):.1f}, p10 {q(lags, .1):.1f}, '
          f'p90 {q(lags, .9):.1f}, p99 {q(lags, .99):.1f}, 최대 {max(lags):.1f}')
    for th in (0, 33, 67):
        print(f'  깊이가 컬러보다 {th} ms 넘게 늦게 옴: {100 * sum(l > th for l in lags) / len(lags):.1f} %')
