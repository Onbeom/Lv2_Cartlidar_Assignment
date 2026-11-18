"""스캔-정답 정합 진단(모의, Cartographer 없이). 정답 자세로 스캔 점을 월드에 놓고 벽 단면까지 거리를 잰다.
시각 어긋남 dt, 라이다별 장착 yaw, 스캔 안 칸별 시각 펼침(왜곡)을 바꿔 가며 거리가 가장 작아지는 값을 찾는다.
사용: python scan_consistency.py <녹화 폴더> [스캔 간격=4]"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

REC, STEP = Path(sys.argv[1]), int(sys.argv[2]) if len(sys.argv) > 2 else 4
ROOT = Path('/home/pa4/git/lv2_next_module')
CLIP, BODY_BOX = 0.10, (-0.175, 0.285, 0.19)

segs = json.load(open(ROOT / '261030_1943/maze_slice_z0.08.json'))['segments']
walls = []
for _, a, b in segs:
    a, b = np.array(a), np.array(b)
    walls += list(a + np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.005) + 1))[:, None] * (b - a))
tree = cKDTree(np.array(walls))

rows = list(csv.DictReader(open(REC / 'truth.csv')))
T = np.array([[float(r['sim_s']), float(r['x']), float(r['y']), float(r['yaw']), float(r['cmd_w']), float(r['cmd_v'])] for r in rows])
yaw_u = np.unwrap(T[:, 3])


def pose(t):
    return np.interp(t, T[:, 0], T[:, 1]), np.interp(t, T[:, 0], T[:, 2]), np.interp(t, T[:, 0], yaw_u)


def qmat(r):
    x, y, z, w = r.x, r.y, r.z, r.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(REC / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    meta = {v.name: v for v in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while r.has_next():
        t, d, _ = r.read_next()
        yield t, deserialize_message(d, get_message(meta[t].type))


static = {}
for _, m in read(['/tf_static']):
    for t in m.transforms:
        M = np.eye(4)
        M[:3, :3] = qmat(t.transform.rotation)
        M[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        static[t.child_frame_id] = (t.header.frame_id, M)


def to_base(f):
    M = np.eye(4)
    while f != 'base_link':
        p, A = static[f]
        M, f = A @ M, p
    return M


# 라이다별 스캔: (stamp, 칸 번호, base_link 기준 점 xy, 라이다 원점 xy)
data = {}
count = {}
for topic, m in read(['/scan_front', '/scan_rear']):
    count[topic] = count.get(topic, 0) + 1
    if count[topic] % STEP:
        continue
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    r = np.array(m.ranges)
    i = np.arange(len(r))
    a = m.angle_min + m.angle_increment * i
    ok = np.isfinite(r) & (r > 0.3) & (r < 6)
    M = to_base(m.header.frame_id)
    P = (M @ np.stack([r * np.cos(a), r * np.sin(a), 0 * r, 1 + 0 * r]))[:2]
    inside = (P[0] >= BODY_BOX[0]) & (P[0] <= BODY_BOX[1]) & (np.abs(P[1]) <= BODY_BOX[2])
    ok &= ~inside
    data.setdefault(topic, []).append((ts, i[ok] / len(r), P[:, ok], M[:2, 3]))


def score(topic, dt=0.0, dyaw=0.0, spread=0.0, sel=None):
    """평균 거리[cm](CLIP 에서 자름)와 3 cm 안 비율. spread: 칸 0→끝 시각 펼침[s](칸 시각 = stamp + dt + spread·(칸/전체 − 0.5))"""
    out = []
    c, s = math.cos(dyaw), math.sin(dyaw)
    for k, (ts, frac, P, o) in enumerate(data[topic]):
        if sel is not None and not sel[topic][k]:
            continue
        Q = P - o[:, None]
        Q = np.stack([c * Q[0] - s * Q[1], s * Q[0] + c * Q[1]]) + o[:, None]
        tb = ts + dt + spread * (frac - 0.5)
        x, y, th = pose(tb)
        W = np.stack([x + np.cos(th) * Q[0] - np.sin(th) * Q[1], y + np.sin(th) * Q[0] + np.cos(th) * Q[1]], 1)
        out.append(tree.query(W)[0])
    d = np.concatenate(out)
    return float(np.minimum(d, CLIP).mean() * 100), float((d < 0.03).mean())


# 구간: 정답 각속도로 회전(|w| > 0.3 rad/s) · 직진(|w| < 0.05, v > 0.05)
def wz(t):
    return np.interp(t + 0.05, T[:, 0][1:], np.diff(yaw_u) / np.diff(T[:, 0]))


def vx(t):
    return np.interp(t, T[:, 0][1:], np.hypot(np.diff(T[:, 1]), np.diff(T[:, 2])) / np.diff(T[:, 0]))


sel_turn = {tp: np.array([abs(wz(d[0])) > 0.3 for d in v]) for tp, v in data.items()}
sel_str = {tp: np.array([abs(wz(d[0])) < 0.05 and vx(d[0]) > 0.05 for d in v]) for tp, v in data.items()}
res = {'recording': str(REC), 'step': STEP, 'clip_m': CLIP, 'n_scans': {k: len(v) for k, v in data.items()},
       'n_turn': {k: int(v.sum()) for k, v in sel_turn.items()}, 'n_straight': {k: int(v.sum()) for k, v in sel_str.items()}}
print(json.dumps(res, ensure_ascii=False), flush=True)
for tp in data:
    r = res[tp] = {}
    r['base'] = {'all': score(tp), 'turn': score(tp, sel=sel_turn), 'straight': score(tp, sel=sel_str)}
    dts = np.round(np.arange(-0.2, 0.2001, 0.01), 3)
    r['dt_turn'] = {float(d): score(tp, dt=d, sel=sel_turn)[0] for d in dts}
    best_dt = min(r['dt_turn'], key=r['dt_turn'].get)
    r['best_dt_turn'] = best_dt
    r['dt_straight_at_best'] = score(tp, dt=best_dt, sel=sel_str)
    yaws = np.round(np.arange(-1.5, 1.5001, 0.1), 2)
    r['dyaw_straight_deg'] = {float(y): score(tp, dyaw=math.radians(y), sel=sel_str)[0] for y in yaws}
    r['best_dyaw_straight_deg'] = min(r['dyaw_straight_deg'], key=r['dyaw_straight_deg'].get)
    r['spread_turn'] = {f'{sp:+.2f}': score(tp, dt=dt0, spread=sp, sel=sel_turn)[0]
                        for sp in (-0.1, -0.05, 0.05, 0.1) for dt0 in (0.0,)}
    print(tp, json.dumps(r, ensure_ascii=False), flush=True)
out = Path(__file__).resolve().parent / 'results' / f'scan_consistency_{REC.name}.json'
out.write_text(json.dumps(res, ensure_ascii=False, indent=1))
print('saved', out)
