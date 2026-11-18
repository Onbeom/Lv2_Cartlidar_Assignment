#!/usr/bin/env python3
"""11-14 미로 bag 을 고친 라이다 TF(PR #304: 실물 laser_front yaw π · laser_rear yaw 0) 로 다시 만드는 사본 (PC, 11-15).
  python3 make_bag_yaw.py <출력 폴더> [--imu-bias 0.0998]
- /scan_front · /scan_rear 원래 스캔(_filtered 는 녹화 때 틀린 TF 로 차체를 가린 것이라 안 씀) → 고친 TF 로 base_link 에 옮겨
  차체 상자(scan_body_mask 기본값: x −0.175 ~ 0.285, |y| ≤ 0.19) 안 점을 inf 로 → /scan_1 · /scan_2
- /tf_static 의 laser_front · laser_rear 회전을 고친 값으로 바꿈. /odom 그대로, /imu/data 는 z 각속도에서 영점(→ /imu 로 이름 바꿈)(기본 0.0998 rad/s, 11-14 정지 구간 10 곳 평균) 뺌
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

SRC = '/home/pa4/git/lv2_next_module/261114_1330_maze_map/bag/maze_1114_1317'
OUT = sys.argv[1]
BIAS = float(sys.argv[sys.argv.index('--imu-bias') + 1]) if '--imu-bias' in sys.argv else 0.0998
XYZ = {'laser_front': (0.1318, 0.0, 0.0675), 'laser_rear': (-0.1318, 0.0, 0.0675)}   # robot.urdf.xacro
YAW = {'laser_front': math.pi, 'laser_rear': 0.0}   # PR #304 실물 값 (roll π 는 둘 다)
BOX = (-0.175, 0.285, 0.19)
REN = {'/scan_front': ('/scan_1', 'laser_front'), '/scan_rear': ('/scan_2', 'laser_rear')}
KEEP = {'/odom', '/imu/data', '/tf_static'}


def quat_roll_pi_yaw(yaw):
    # R = Rz(yaw) · Rx(π) → 쿼터니언 (x, y, z, w) = qz ⊗ qx = (cos(y/2), sin(y/2), 0, 0)
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def mask(m, frame):
    r = np.array(m.ranges, dtype=float)
    a = m.angle_min + m.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0)
    # 센서 → base_link: roll π 로 y 부호 반전, 그다음 yaw
    xs, ys = r * np.cos(a), -r * np.sin(a)
    c, s = math.cos(YAW[frame]), math.sin(YAW[frame])
    bx = XYZ[frame][0] + c * xs - s * ys
    by = XYZ[frame][1] + s * xs + c * ys
    inside = ok & (bx >= BOX[0]) & (bx <= BOX[1]) & (np.abs(by) <= BOX[2])
    r[inside] = math.inf
    m.ranges = [float(v) for v in r]
    return int(inside.sum())


def main():
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=SRC, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    wr = rosbag2_py.SequentialWriter()
    wr.open(rosbag2_py.StorageOptions(uri=OUT, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    out_types = {}
    for t, ty in types.items():
        if t in REN:
            out_types[REN[t][0]] = ty
        elif t == '/imu/data':
            out_types['/imu'] = ty   # Cartographer 오프라인은 이름 바꾸기를 bag 에 안 씀 → /imu
        elif t in KEEP:
            out_types[t] = ty
    for i, (t, ty) in enumerate(out_types.items()):
        wr.create_topic(rosbag2_py.TopicMetadata(id=i, name=t, type=ty, serialization_format='cdr'))
    n = {}
    masked = {'laser_front': [], 'laser_rear': []}
    while rd.has_next():
        t, data, stamp = rd.read_next()
        if t in REN:
            m = deserialize_message(data, get_message(types[t]))
            masked[REN[t][1]].append(mask(m, REN[t][1]))
            wr.write(REN[t][0], serialize_message(m), stamp)
            n[REN[t][0]] = n.get(REN[t][0], 0) + 1
        elif t == '/imu/data':
            m = deserialize_message(data, get_message(types[t]))
            m.angular_velocity.z -= BIAS
            wr.write('/imu', serialize_message(m), stamp)
            n['/imu'] = n.get('/imu', 0) + 1
        elif t == '/tf_static':
            m = deserialize_message(data, get_message(types[t]))
            for tr in m.transforms:
                if tr.child_frame_id in YAW:
                    x, y, z, w = quat_roll_pi_yaw(YAW[tr.child_frame_id])
                    q = tr.transform.rotation
                    print(f'tf_static {tr.header.frame_id} → {tr.child_frame_id}: 옛 q ({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}) → ({x:.3f}, {y:.3f}, {z:.3f}, {w:.3f})')
                    q.x, q.y, q.z, q.w = x, y, z, w
            wr.write(t, serialize_message(m), stamp)
            n[t] = n.get(t, 0) + 1
        elif t in KEEP:
            wr.write(t, data, stamp)
            n[t] = n.get(t, 0) + 1
    del wr
    print('쓴 메시지:', n)
    for f, v in masked.items():
        print(f'{f}: 스캔마다 차체 상자 안 점 중앙값 {int(np.median(v))} (최대 {max(v)})')


if __name__ == '__main__':
    main()
