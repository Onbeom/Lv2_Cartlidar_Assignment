"""launch 확인용 bag(모의): 녹화 원본에서 /clock · /tf_static · /tf(odom 부모 제외 = 관절) · /scan_front · /scan_rear(마스크 전 원본)
· /top_camera_front/infra1/camera_info(영상 stamp 비교용) · /odom(회전 배수 S 보정: 차체 기준 이동 그대로, 회전 증분 · 각속도 1/S)을 복사.
odom→base_link TF 는 넣지 않음(launch 의 EKF 가 냄). 원본 QoS 그대로. 사용: python make_test_bag.py <녹화 폴더> <이름> [S=1.15]"""
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

REC, NAME = Path(sys.argv[1]), sys.argv[2]
S = float(sys.argv[3]) if len(sys.argv) > 3 else 1.15
DST = Path(__file__).resolve().parent / 'bags' / NAME
assert not DST.exists(), DST
TOPICS = ['/clock', '/tf_static', '/tf', '/scan_front', '/scan_rear', '/odom', '/top_camera_front/infra1/camera_info']


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(REC / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    meta = {v.name: v for v in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r, meta


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


r, meta = reader(['/odom'])
raw = []
while r.has_next():
    _, d, _ = r.read_next()
    m = deserialize_message(d, get_message(meta['/odom'].type))
    raw.append((m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)))
raw.sort()
fixed = {raw[0][0]: raw[0][1:]}
cx, cy, cth = raw[0][1:]
for (_, x0, y0, t0), (ns, x1, y1, t1) in zip(raw, raw[1:]):
    c, s = math.cos(t0), math.sin(t0)
    bx, by = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)
    c2, s2 = math.cos(cth), math.sin(cth)
    cx, cy, cth = cx + c2 * bx - s2 * by, cy + s2 * bx + c2 * by, cth + math.atan2(math.sin(t1 - t0), math.cos(t1 - t0)) / S
    fixed[ns] = (cx, cy, cth)
r, meta = reader(TOPICS)
DST.parent.mkdir(exist_ok=True)
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=str(DST), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
made, n = set(), {}
while r.has_next():
    t, d, stamp = r.read_next()
    if t == '/tf':
        m = deserialize_message(d, get_message(meta[t].type))
        m.transforms = [x for x in m.transforms if x.header.frame_id not in ('map', 'odom')]
        if not m.transforms:
            continue
        d = serialize_message(m)
    elif t == '/odom':
        m = deserialize_message(d, get_message(meta[t].type))
        x, y, th = fixed[m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec]
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.x = m.pose.pose.orientation.y = 0.0
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        m.twist.twist.angular.z /= S
        d = serialize_message(m)
    if t not in made:
        w.create_topic(meta[t])
        made.add(t)
    w.write(t, d, stamp)
    n[t] = n.get(t, 0) + 1
del w
print(NAME, S, n)
