#!/usr/bin/env python3
"""두 D435 컬러 도착 순서로 인지 노드 신호등 처리(process_signal) 흉내 (읽기만, camera_info 만 받음).

signal_order_sim.py <초> [처리 Hz=15]

인지 노드는 처리 주기마다 앞 · 뒤 카메라에 쌓인 컬러를 모아 stamp 순으로 처리하고, 이미 처리한 stamp 보다 이른 프레임은 버린다.
뒤 카메라 프레임이 먼저 처리되면, 그보다 이른 stamp 의 앞 카메라 프레임이 늦게 도착했을 때 버려진다.
받은 시각 · stamp 기록으로 이를 다시 계산해 앞 카메라 프레임 버림 비율, 앞 카메라 프레임이 하나도 처리되지 않은 주기 비율(= 모름 메시지)을 낸다.
"""
import statistics
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

secs = float(sys.argv[1])
hz = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
rclpy.init()
node = rclpy.create_node('signal_order_sim')
rec = []   # (카메라, stamp s, 받은 시각 time.time())


def cb(cam):
    return lambda m: rec.append((cam, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, time.time()))


node.create_subscription(CameraInfo, '/top_camera_front/color/camera_info', cb('F'), qos_profile_sensor_data)
node.create_subscription(CameraInfo, '/top_camera_rear/color/camera_info', cb('R'), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.01)
node.destroy_node()
rclpy.shutdown()

for cam in 'FR':
    lat = [(a - s) * 1e3 for c, s, a in rec if c == cam]
    print(f'{cam}: {len(lat)} 장, 도착 − stamp ms 중앙 {statistics.median(lat):.1f} (p10 {sorted(lat)[len(lat) // 10]:.1f}, '
          f'p90 {sorted(lat)[9 * len(lat) // 10]:.1f})')
a0, a1 = rec[0][2] + 1, rec[-1][2] - 1
for phase in (0.0, 0.25, 0.5, 0.75):
    last, dropped, front_n, empty, batches = -1e18, 0, 0, 0, 0
    tb = a0 + phase / hz
    i = 0
    rs = sorted(rec, key=lambda r: r[2])
    while i < len(rs) and rs[i][2] <= tb - 1 / hz:
        i += 1
    while tb < a1:
        batch = []
        while i < len(rs) and rs[i][2] <= tb:
            batch.append(rs[i])
            i += 1
        ok_front = 0
        for c, s, a in sorted(batch, key=lambda r: r[1]):
            if s <= last:
                if c == 'F':
                    dropped += 1
                    front_n += 1
                continue
            last = s
            if c == 'F':
                ok_front += 1
                front_n += 1
        if batch:
            batches += 1
            empty += ok_front == 0
        tb += 1 / hz
    print(f'주기 위상 {phase:.2f}: 앞 프레임 버림 {dropped}/{front_n} = {100 * dropped / max(1, front_n):.1f} %, '
          f'앞 프레임 0 장 처리 주기 {empty}/{batches} = {100 * empty / max(1, batches):.1f} %')
