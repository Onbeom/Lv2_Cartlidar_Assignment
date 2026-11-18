#!/usr/bin/env python3
"""loc_cpp 비교용 재생 bag (PC, 11-16). 11-14 미로 bag 에서 위치 추정 입력만 뽑아, 고친 라이다 TF(#304) · odom → base_link TF 를 넣는다.
  python3 make_replay_bag.py <출력 폴더>
- /scan_front · /scan_rear 원래 그대로 (차체 가림은 재생 때 scan_body_mask 가 함), /odom · /imu/data 그대로
- /tf_static: laser_front · laser_rear 회전만 #304 값으로 (앞 yaw π · 뒤 0)
- /tf: /odom 마다 odom → base_link (11-14 bag 에는 /tf 가 녹화되지 않았음, 실물은 EKF 가 냄)
"""
import math
import sys

import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

SRC = '/home/pa4/git/lv2_next_module/261114_1330_maze_map/bag/maze_1114_1317'
YAW = {'laser_front': math.pi, 'laser_rear': 0.0}
KEEP = {'/scan_front', '/scan_rear', '/odom', '/imu/data', '/tf_static'}

rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=SRC, storage_id=''), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
wr = rosbag2_py.SequentialWriter()
wr.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
for i, t in enumerate(sorted(KEEP) + ['/tf']):
    ty = types[t] if t in types else 'tf2_msgs/msg/TFMessage'
    wr.create_topic(rosbag2_py.TopicMetadata(id=i, name=t, type=ty, serialization_format='cdr'))
n = {}
while rd.has_next():
    t, data, stamp = rd.read_next()
    if t not in KEEP:
        continue
    if t == '/tf_static':
        m = deserialize_message(data, get_message(types[t]))
        for tr in m.transforms:
            if tr.child_frame_id in YAW:
                y = YAW[tr.child_frame_id]
                q = tr.transform.rotation
                q.x, q.y, q.z, q.w = math.cos(y / 2), math.sin(y / 2), 0.0, 0.0   # Rz(yaw) · Rx(π)
        data = serialize_message(m)
    wr.write(t, data, stamp)
    n[t] = n.get(t, 0) + 1
    if t == '/odom':
        o = deserialize_message(data, get_message(types[t]))
        tr = TransformStamped()
        tr.header = o.header
        tr.child_frame_id = o.child_frame_id
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z
        tr.transform.rotation = o.pose.pose.orientation
        wr.write('/tf', serialize_message(TFMessage(transforms=[tr])), stamp)
        n['/tf'] = n.get('/tf', 0) + 1
del wr
print(n)
