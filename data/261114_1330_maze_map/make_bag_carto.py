#!/usr/bin/env python3
"""녹화 bag → Cartographer 오프라인 입력 사본 (PC, 11-16 준비, 11-16 녹화 maze_1116_1116_0735 부터 씀).
  python3 make_bag_carto.py <원본 bag 폴더> <출력 폴더> [--raw] [--imu-bias auto|<rad/s>|0] [--odom <토픽>]
- --odom: /odom 자리에 넣을 토픽 (기본 /odom = 바퀴. 11-16: /odometry/filtered = EKF(바퀴 이동 + IMU 회전) 비교용)
- 스캔: 기본은 녹화 때 차체를 가린 /scan_front_filtered · /scan_rear_filtered (11-16 녹화는 #304 고친 TF 로 가림),
  --raw 면 원래 /scan_front · /scan_rear. 이름을 /scan_1 · /scan_2 로 (오프라인 노드는 --ros-args 이름 바꾸기를 bag 에 안 씀)
- /odom · /tf_static 그대로 (11-16 녹화는 TF 가 이미 맞음 — 11-14 bag 처럼 고치지 않음), /tf 는 넣지 않음 (Cartographer 가 odom 을 씀)
- /imu/data → /imu, z 각속도 영점: auto = 처음 정지 구간(첫 /cmd_vel 이 0 아닌 때 전, 최대 10 s) 평균을 뺌 · 숫자 = 그 값 · 0 = 안 뺌
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

src, out = sys.argv[1], sys.argv[2]
raw = '--raw' in sys.argv
bias_arg = sys.argv[sys.argv.index('--imu-bias') + 1] if '--imu-bias' in sys.argv else 'auto'
ODOM = sys.argv[sys.argv.index('--odom') + 1] if '--odom' in sys.argv else '/odom'
SCAN = {('/scan_front' if raw else '/scan_front_filtered'): '/scan_1', ('/scan_rear' if raw else '/scan_rear_filtered'): '/scan_2'}


def reader(topics=None):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=src, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


types = {t.name: t.type for t in reader().get_all_topics_and_types()}
bias = 0.0
if bias_arg == 'auto':
    r = reader(['/imu/data', '/cmd_vel'])
    t0, move, zs = None, None, []
    while r.has_next():
        t, d, s = r.read_next()
        t0 = s if t0 is None else t0
        m = deserialize_message(d, get_message(types[t]))
        if t == '/cmd_vel':
            tw = m.twist if hasattr(m, 'twist') else m
            if abs(tw.linear.x) > 1e-3 or abs(tw.angular.z) > 1e-3:
                move = s
                break
        elif (s - t0) * 1e-9 <= 10.0:
            zs.append(m.angular_velocity.z)
        elif move is None and (s - t0) * 1e-9 > 10.0:
            break
    bias = float(np.mean(zs)) if zs else 0.0
    print(f'IMU z 영점 (처음 정지 {len(zs)} 개 평균): {bias:+.4f} rad/s · 표준편차 {float(np.std(zs)) if zs else 0:.4f}')
else:
    bias = float(bias_arg)
wr = rosbag2_py.SequentialWriter()
wr.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
names = {**SCAN, ODOM: '/odom', '/tf_static': '/tf_static', '/imu/data': '/imu'}
for i, (a, b) in enumerate(names.items()):
    wr.create_topic(rosbag2_py.TopicMetadata(id=i, name=b, type=types[a], serialization_format='cdr'))
n = {}
r = reader(list(names))
while r.has_next():
    t, d, s = r.read_next()
    if t == '/imu/data' and bias:
        m = deserialize_message(d, get_message(types[t]))
        m.angular_velocity.z -= bias
        d = serialize_message(m)
    wr.write(names[t], d, s)
    n[names[t]] = n.get(names[t], 0) + 1
del wr
print('쓴 메시지:', n)
