"""녹화 도구 확인 T1~T4 (판정 아님). 사용: python check_record_lab.py <결과 폴더>
T1 토픽 수(발행 대비) · 좌우 영상 stamp · 수신 버퍼 오류, T2 라이다 거리 vs 정답 자세 + 벽 단면 계산, T3 odom 직진 · 회전 비,
T4 TF 나무(부모 하나씩, odom→base_link→laser/infra 이어짐)."""
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

D = Path(sys.argv[1])
HERE = Path(__file__).resolve().parent
segs = np.array([a + b for _, a, b in json.load(open(HERE / 'maze_slice_z0.20.json'))['segments']])


def quat_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tmat(tr):
    t, r = tr.translation, tr.rotation
    m = np.eye(4)
    m[:3, :3] = quat_mat(r.x, r.y, r.z, r.w)
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(D / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
count, static, parents, scans, ir_st = Counter(), {}, defaultdict(set), defaultdict(list), defaultdict(list)
while reader.has_next():
    topic, data, _ = reader.read_next()
    count[topic] += 1
    if topic in ('/tf', '/tf_static', '/scan_front', '/scan_rear') or topic.endswith('image_rect_raw'):
        msg = deserialize_message(data, get_message(types[topic]))
        if topic in ('/tf', '/tf_static'):
            for tr in msg.transforms:
                parents[tr.child_frame_id].add(tr.header.frame_id)
                if topic == '/tf_static':
                    static[tr.child_frame_id] = (tr.header.frame_id, tmat(tr.transform))
        elif topic.startswith('/scan'):
            scans[topic].append((stamp(msg.header), msg.header.frame_id, msg.angle_min, msg.angle_increment, np.array(msg.ranges)))
        else:
            ir_st[topic.split('/')[2]].append(stamp(msg.header))
sent = json.load(open(D / 'scene_counts.json'))['sent']
rcv = [l.split(',') for l in open(D / 'rcvbuf_errors.csv').read().split()]
t1 = {'counts': dict(count), 'ratio_scan': {k: count[k] / sent['scan'][k] for k in sent['scan']},
      'ratio_ir': {c: count[f'/top_camera_front/{c}/image_rect_raw'] / sent['ir'][c] for c in sent['ir']},
      'ratio_js': count['/joint_states'] / sent['js'], 'ratio_odom': count['/odom'] / sent['odom'],
      'ir_stamp_pairs_equal': len(set(ir_st['infra1']) & set(ir_st['infra2'])) / max(1, len(ir_st['infra1'])),
      'rcvbuf_increase': int(rcv[1][1]) - int(rcv[0][1])}


def chain(frame):
    """base_link 에서 frame 까지 정적 변환(행렬)"""
    m = np.eye(4)
    f = frame
    while f != 'base_link':
        p, t = static[f]
        m = t @ m
        f = p
    return m


multi = {c: sorted(p) for c, p in parents.items() if len(p) > 1}
need = ['base_footprint', 'laser_front', 'laser_rear', 'top_camera_front_infra1_optical_frame', 'top_camera_front_infra2_optical_frame']
reach = {}
for f in need:
    try:
        chain(f)
        reach[f] = True
    except KeyError:
        reach[f] = False
t4 = {'multi_parent': multi, 'odom_child': sorted(c for c, p in parents.items() if 'odom' in p), 'static_reach_from_base_link': reach}

tr = np.array([[float(r['sim_s']), float(r['x']), float(r['y']), float(r['z']), float(r['qw']), float(r['qx']), float(r['qy']), float(r['qz']),
                float(r['yaw']), float(r['odom_x']), float(r['odom_y']), float(r['odom_yaw'])] for r in csv.DictReader(open(D / 'truth.csv'))])
phase = [r['phase'] for r in csv.DictReader(open(D / 'truth.csv'))]


def base_at(t):
    i = min(np.searchsorted(tr[:, 0], t), len(tr) - 1)
    r = tr[i]
    m = np.eye(4)
    m[:3, :3] = quat_mat(r[5], r[6], r[7], r[4])
    m[:3, 3] = r[1:4]
    return m


def cast(o, d):
    """2D 광선 o + s d 와 벽 선분의 가장 가까운 교차 거리"""
    a, b = segs[:, :2], segs[:, 2:]
    e = b - a
    den = d[0] * e[:, 1] - d[1] * e[:, 0]
    w = a - o
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (w[:, 0] * e[:, 1] - w[:, 1] * e[:, 0]) / den
        u = (w[:, 0] * d[1] - w[:, 1] * d[0]) / den
    ok = (np.abs(den) > 1e-12) & (s > 0) & (u >= 0) & (u <= 1)
    return float(s[ok].min()) if ok.any() else np.inf


t2 = {}
for topic, rows in scans.items():
    diffs, blocked = [], []
    for k, (ts, frame, a0, inc, rng) in enumerate(rows):
        if k % 10:
            continue
        L = base_at(ts) @ chain(frame)
        o = L[:2, 3]
        own = 0
        for i, r in enumerate(rng):
            a = a0 + inc * (i + 0.5)
            dw = (L[:3, :3] @ np.array([math.cos(a), math.sin(a), 0.0]))[:2]
            dw /= np.linalg.norm(dw)
            exp = cast(o, dw)
            if np.isfinite(r) and r < 0.25:
                own += 1
                continue
            if np.isfinite(r) and np.isfinite(exp) and exp < 3.5:
                diffs.append(r - exp)
        blocked.append(own * inc * 180 / math.pi)
    diffs = np.array(diffs)
    t2[topic] = {'n_compared': len(diffs), 'median_abs_cm': float(np.median(np.abs(diffs)) * 100) if len(diffs) else None,
                 'p95_abs_cm': float(np.percentile(np.abs(diffs), 95) * 100) if len(diffs) else None,
                 'median_signed_cm': float(np.median(diffs) * 100) if len(diffs) else None,
                 'own_body_deg_median': float(np.median(blocked)) if blocked else None}

go = [i for i in range(1, len(tr)) if phase[i] == 'go' and phase[i - 1] == 'go']
turn = [i for i in range(1, len(tr)) if phase[i] == 'turn' and phase[i - 1] == 'turn']


def dsum(idx, cols):
    return sum(float(np.hypot(*(tr[i, cols] - tr[i - 1, cols]))) for i in idx)


def ysum(idx, col):
    return sum(abs(math.remainder(tr[i, col] - tr[i - 1, col], 2 * math.pi)) for i in idx)


t3 = {'go_truth_m': dsum(go, [1, 2]), 'go_odom_m': dsum(go, [9, 10]), 'turn_truth_rad': ysum(turn, 8), 'turn_odom_rad': ysum(turn, 11)}
t3['go_ratio'] = t3['go_odom_m'] / max(t3['go_truth_m'], 1e-9)
t3['turn_ratio'] = t3['turn_odom_rad'] / max(t3['turn_truth_rad'], 1e-9)
out = {'T1': t1, 'T2_lidar': t2, 'T3_odom': t3, 'T4_tf': t4}
(D / 'check.json').write_text(json.dumps(out, ensure_ascii=False, indent=2))
print(json.dumps(out, ensure_ascii=False, indent=1))
