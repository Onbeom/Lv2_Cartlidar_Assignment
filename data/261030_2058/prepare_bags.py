"""새 모델 · 새 미로 녹화(261030_1943/results/n261028_*)에서 Cartographer 대조용 bag 을 만든다. 원본 녹화는 읽기만 한다.
공통: /scan_front · /scan_rear(차체 가림 칸 마스크 적용) · /tf_static · /tf(odom 부모 제외 = 관절 TF) · /clock, 원본 QoS 그대로.
  lidar_only : odom 없음
  wheel_odom : 녹화의 바퀴 /odom + TF odom→base_link (보정 배수 1.0)
  truth_odom : 정답 궤적 odom(/odom_truth, TF odom→base_link) — 진단용 상한, 후보 아님
차체 가림 마스크: 출발 전 정지 구간(sim 0.5~1.9 s) 스캔에서 거리 < 0.32 m 가 한 번이라도 나온 칸을 그 라이다의 차체 칸으로 보고 모든 스캔에서 inf.
  (로봇이 출발 전 벽에서 0.34 m 이상 떨어져 있어 벽이 아닌 차체만 걸림. 실물에서도 정지 상태 한 번 재서 정하는 방식)
사용: python prepare_bags.py <녹화 폴더> <bright|dark>"""
import csv
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

REC, LIGHT = Path(sys.argv[1]), sys.argv[2]
OUT = Path(__file__).resolve().parent / 'results'
OUT.mkdir(exist_ok=True)
BODY_R, STATIC = 0.32, (0.5, 1.9)


def read(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(REC / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    meta = {v.name: v for v in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while r.has_next():
        t, d, s = r.read_next()
        yield meta[t], d, s


def to_time(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


# 1) 차체 칸 마스크 (v2): 정지 스캔의 점을 base_link 로 옮겨 로봇 몸체 상자 안이면 그 칸을 가림.
#    v1(거리 < 0.32 m 이면 가림)은 출발 지점 뒤 벽(뒤 라이다에서 약 0.22 m)까지 가려 뒤 라이다 289° 를 지움 → results_v1_tool_error
BODY_BOX = (-0.175, 0.285, 0.19)   # base_link 기준 x 최소 · 최대, |y| 최대 [m]: 차체 상자 x −0.155~0.263(그리퍼), y ±0.171(바퀴) + 2 cm


def qmat(r):
    x, y, z, w = r.x, r.y, r.z, r.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


static = {}
for meta, data, _ in read(['/tf_static']):
    for t in deserialize_message(data, get_message(meta.type)).transforms:
        M = np.eye(4)
        M[:3, :3] = qmat(t.transform.rotation)
        M[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        static[t.child_frame_id] = (t.header.frame_id, M)


def to_base(frame):
    M, f = np.eye(4), frame
    while f != 'base_link':
        p, T = static[f]
        M, f = T @ M, p
    return M


mask = {}
for meta, data, _ in read(['/scan_front', '/scan_rear']):
    m = deserialize_message(data, get_message(meta.type))
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if STATIC[0] <= ts <= STATIC[1]:
        r = np.array(m.ranges)
        a = m.angle_min + m.angle_increment * (np.arange(len(r)) + 0.5)
        ok = np.isfinite(r)
        P = to_base(m.header.frame_id) @ np.stack([np.where(ok, r, 0) * np.cos(a), np.where(ok, r, 0) * np.sin(a), np.zeros(len(r)), np.ones(len(r))])
        inside = ok & (P[0] >= BODY_BOX[0]) & (P[0] <= BODY_BOX[1]) & (np.abs(P[1]) <= BODY_BOX[2])
        mask[meta.name] = mask.get(meta.name, np.zeros(len(r), bool)) | inside
# 2) 공통 메시지
base, counts, src_meta = [], Counter(), {}
for meta, data, stamp in read(['/scan_front', '/scan_rear', '/tf_static', '/tf', '/clock', '/odom']):
    src_meta.setdefault(meta.name, meta)
    if meta.name == '/odom':
        continue
    if meta.name == '/tf':
        msg = deserialize_message(data, get_message(meta.type))
        msg.transforms = [t for t in msg.transforms if t.header.frame_id not in ('map', 'odom')]
        if not msg.transforms:
            continue
        data = serialize_message(msg)
    elif meta.name.startswith('/scan'):
        msg = deserialize_message(data, get_message(meta.type))
        r = np.array(msg.ranges)
        r[mask[meta.name]] = np.inf
        msg.ranges = [float(x) for x in r]
        data = serialize_message(msg)
    base.append((stamp, meta, data))
    counts[meta.name] += 1
wheel = []
for meta, data, stamp in read(['/odom', '/tf']):
    if meta.name == '/odom':
        wheel.append((stamp, meta, data))
    else:
        msg = deserialize_message(data, get_message(meta.type))
        msg.transforms = [t for t in msg.transforms if t.header.frame_id == 'odom']
        if msg.transforms:
            wheel.append((stamp, meta, serialize_message(msg)))
# 3) 정답 odom (시작 자세 기준 상대값, child base_link)
rows = [(float(r['sim_s']), float(r['x']), float(r['y']), float(r['yaw'])) for r in csv.DictReader(open(REC / 'truth.csv'))]
t0, x0, y0, th0 = rows[0]
c, s = math.cos(-th0), math.sin(-th0)
odom_meta = rosbag2_py.TopicMetadata(id=0, name='/odom_truth', type='nav_msgs/msg/Odometry', serialization_format='cdr')
truth = []
for ts, x, y, th in rows:
    dx, dy = x - x0, y - y0
    px, py, pth = c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(th - th0), math.cos(th - th0))
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = to_time(ts), 'odom', 'base_link'
    o.pose.pose.position.x, o.pose.pose.position.y = px, py
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pth / 2), math.cos(pth / 2)
    tf = TransformStamped()
    tf.header, tf.child_frame_id = o.header, 'base_link'
    tf.transform.translation.x, tf.transform.translation.y = px, py
    tf.transform.rotation.z, tf.transform.rotation.w = o.pose.pose.orientation.z, o.pose.pose.orientation.w
    ns = int(round(ts * 1e9))
    truth += [(ns, odom_meta, serialize_message(o)), (ns, src_meta['/tf'], serialize_message(TFMessage(transforms=[tf])))]
for kind, extra in [('lidar_only', []), ('wheel_odom', wheel), ('truth_odom', truth)]:
    target = OUT / f'bag_{kind}_{LIGHT}'
    assert not target.exists(), target
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(target), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    made = set()
    for stamp, meta, data in sorted(base + extra, key=lambda v: v[0]):
        if meta.name not in made:
            w.create_topic(meta)
            made.add(meta.name)
        w.write(meta.name, data, stamp)
    del w
audit = {'recording': str(REC), 'light': LIGHT, 'counts': dict(counts), 'wheel_msgs': len(wheel), 'truth_odom_msgs': len(truth) // 2,
         'body_mask_bins': {k: int(v.sum()) for k, v in mask.items()}, 'body_mask_deg': {k: float(v.sum() * 0.9) for k, v in mask.items()},
         'body_mask_rule': f'v2: 정지 {STATIC} s 스캔 점이 base_link 몸체 상자 {BODY_BOX} 안인 칸', 'qos_copied': {k: len(v.offered_qos_profiles) for k, v in src_meta.items()},
         'truth_in': {'lidar_only': False, 'wheel_odom': False, 'truth_odom': 'diagnostic_upper_bound_only'},
         'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (REC / 'bag').glob('*.mcap')}}
(OUT / f'bag_audit_{LIGHT}.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2))
print(json.dumps({k: v for k, v in audit.items() if k != 'source_sha256'}, ensure_ascii=False), flush=True)
