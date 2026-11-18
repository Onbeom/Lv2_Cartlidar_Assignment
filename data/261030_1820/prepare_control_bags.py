"""대조용 bag 두 종류를 만든다. 원본 녹화는 읽기만 한다.
lidar_only: 라이다·정적TF·관절TF·clock (바퀴odom·map/odom TF 제외)
truth_odom: lidar_only + 정답 궤적 odom(/odom_truth, TF odom→base_footprint) — 진단용 상한, 후보 아님"""
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
import rosbag2_py
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

ROOT = Path('/home/pa4/git/lv2_next_module')
HERE = Path(__file__).resolve().parent
OUT = HERE / 'results_v2'
REC = {'bright': 'm1711_bright_1_261027_203413', 'dark': 'm1711_dark_1_261027_203822'}


def read(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    meta = {v.name: v for v in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        yield meta[topic], data, stamp


def to_time(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def quat_z(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def truth_rows(rec):
    rows = list(csv.DictReader(open(rec / 'truth.csv')))
    t = [(float(r['sim_s']), float(r['x']), float(r['y']), float(r['yaw'])) for r in rows]
    t0, x0, y0, th0 = t[0]
    c, s = math.cos(-th0), math.sin(-th0)
    out = []
    for ts, x, y, th in t:
        dx, dy = x - x0, y - y0
        out.append((ts, c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(th - th0), math.cos(th - th0))))
    return out


OUT.mkdir(exist_ok=True)
for light, name in REC.items():
    rec = ROOT / '261027_0953/results' / name
    src = rec / 'bag'
    base = []
    src_meta = {}
    counts = Counter()
    edges = Counter()
    for meta, data, stamp in read(src, ['/scan_front', '/scan_rear', '/tf_static', '/tf', '/clock']):
        if meta.name == '/tf':
            msg = deserialize_message(data, get_message(meta.type))
            msg.transforms = [t for t in msg.transforms if t.header.frame_id not in ['map', 'odom']]
            if not msg.transforms:
                continue
            for t in msg.transforms:
                edges[t.header.frame_id + '->' + t.child_frame_id] += 1
            data = serialize_message(msg)
        base.append((stamp, meta, data))
        src_meta.setdefault(meta.name, meta)
        counts[meta.name] += 1
    assert not any(k.startswith(('map->', 'odom->')) for k in edges)
    extra = []
    # v2: 원본 토픽 QoS(특히 /tf_static transient_local)를 그대로 쓴다. v1은 빈 QoS로 써서 tf_static이 전달되지 않음
    odom_meta = rosbag2_py.TopicMetadata(id=0, name='/odom_truth', type='nav_msgs/msg/Odometry', serialization_format='cdr')
    for ts, x, y, th in truth_rows(rec):
        sz, cw = quat_z(th)
        odo = Odometry()
        odo.header.stamp = to_time(ts)
        odo.header.frame_id = 'odom'
        odo.child_frame_id = 'base_footprint'
        odo.pose.pose.position.x, odo.pose.pose.position.y = x, y
        odo.pose.pose.orientation.z, odo.pose.pose.orientation.w = sz, cw
        tf = TransformStamped()
        tf.header = odo.header
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = sz, cw
        ns = int(round(ts * 1e9))
        extra.append((ns, odom_meta, serialize_message(odo)))
        extra.append((ns, src_meta['/tf'], serialize_message(TFMessage(transforms=[tf]))))
    for kind, rows in [('lidar_only', base), ('truth_odom', base + extra)]:
        target = OUT / f'bag_{kind}_{light}'
        assert not target.exists()
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(target), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        made = set()
        for stamp, meta, data in sorted(rows, key=lambda v: v[0]):
            if meta.name not in made:
                writer.create_topic(meta)
                made.add(meta.name)
            writer.write(meta.name, data, stamp)
        del writer
    audit = {'light': light, 'source_bag': str(src), 'old_model': '261027_1711', 'counts': dict(counts),
             'tf_edges_kept': dict(edges), 'truth_odom_messages': len(extra) // 2,
             'qos_copied_from_source': {k: len(v.offered_qos_profiles) for k, v in src_meta.items()},
             'wheel_odom_excluded': True, 'map_odom_tf_excluded': True,
             'truth_in_lidar_only': False, 'truth_in_truth_odom': 'diagnostic_upper_bound_only',
             'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in src.glob('*.mcap')}}
    (OUT / f'bag_audit_{light}.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2))
    print(light, dict(counts), 'truth', len(extra) // 2, flush=True)
