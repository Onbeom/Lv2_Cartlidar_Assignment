"""바퀴 odom 회전 배수 보정 bag(모의, 진단). bag_wheel_odom_<light> 의 /odom · /tf(odom→base_link) 를 다시 적분: 한 칸마다
차체 기준 이동은 그대로, 회전 증분만 1/S 배. 다른 메시지 · QoS 는 그대로 복사. 결과 bag_wheel_odom_scaled_<light>.
S = 1.15: 앞 녹화 확인에서 잰 회전 과대 1.149(이번 녹화 T3 1.152 를 그대로 쓰지 않음). 실물에서는 제자리 회전 실측으로 정할 값.
사용: python prepare_wheel_scaled.py <bright|dark> [S]"""
import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

LIGHT = sys.argv[1]
S = float(sys.argv[2]) if len(sys.argv) > 2 else 1.15
OUT = Path(__file__).resolve().parent / 'results'
SRC, DST = OUT / f'bag_wheel_odom_{LIGHT}', OUT / f'bag_wheel_odom_scaled_{LIGHT}'
assert not DST.exists(), DST


def reader():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(SRC), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return r


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# 1) /odom 자세를 stamp 순으로 다시 적분
r = reader()
meta = {v.name: v for v in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/odom']))
raw = []
while r.has_next():
    _, d, _ = r.read_next()
    m = deserialize_message(d, get_message(meta['/odom'].type))
    p = m.pose.pose
    raw.append((m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec, p.position.x, p.position.y, yaw_of(p.orientation)))
raw.sort()
fixed = {}
cx, cy, cth = raw[0][1], raw[0][2], raw[0][3]
fixed[raw[0][0]] = (cx, cy, cth)
for (_, x0, y0, t0), (ns, x1, y1, t1) in zip(raw, raw[1:]):
    c, s = math.cos(t0), math.sin(t0)
    bx, by = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)
    c2, s2 = math.cos(cth), math.sin(cth)
    cx, cy, cth = cx + c2 * bx - s2 * by, cy + s2 * bx + c2 * by, cth + wrap(t1 - t0) / S
    fixed[ns] = (cx, cy, cth)


def put(pose_xyz_q, ns):
    x, y, th = fixed[ns]
    return x, y, math.sin(th / 2), math.cos(th / 2)


# 2) 다시 쓰기
r = reader()
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=str(DST), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
made, n_odom, n_tf, miss = set(), 0, 0, 0
while r.has_next():
    t, d, stamp = r.read_next()
    if t not in made:
        w.create_topic(meta[t])
        made.add(t)
    if t == '/odom':
        m = deserialize_message(d, get_message(meta[t].type))
        ns = m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec
        x, y, qz, qw = put(None, ns)
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.x = m.pose.pose.orientation.y = 0.0
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        m.twist.twist.angular.z /= S
        d = serialize_message(m)
        n_odom += 1
    elif t == '/tf':
        m = deserialize_message(d, get_message(meta[t].type))
        hit = False
        for tr in m.transforms:
            if tr.header.frame_id == 'odom':
                ns = tr.header.stamp.sec * 10 ** 9 + tr.header.stamp.nanosec
                if ns not in fixed:
                    miss += 1
                    continue
                x, y, qz, qw = put(None, ns)
                tr.transform.translation.x, tr.transform.translation.y = x, y
                tr.transform.rotation.x = tr.transform.rotation.y = 0.0
                tr.transform.rotation.z, tr.transform.rotation.w = qz, qw
                hit = True
        if hit:
            n_tf += 1
            d = serialize_message(m)
    w.write(t, d, stamp)
del w
audit = {'source': str(SRC), 'scale_S': S, 'odom_msgs': n_odom, 'tf_odom_msgs': n_tf, 'tf_stamp_without_odom': miss,
         'final_yaw_raw_deg': math.degrees(raw[-1][3] - raw[0][3]), 'final_yaw_fixed_deg': math.degrees(cth - raw[0][3]),
         'note': '모의 · 진단. 실물 배수는 제자리 회전 실측으로 정함'}
(OUT / f'bag_audit_wheel_odom_scaled_{LIGHT}.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2))
print(json.dumps(audit, ensure_ascii=False))
assert miss == 0, 'odom TF stamp 가 /odom 에 없음'
