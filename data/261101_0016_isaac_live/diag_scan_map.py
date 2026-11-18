"""진단(판정 아님): 실시간 Nav2 녹화의 라이다 스캔 끝점을 ① 참값 자세 ② 추정(map→odom · odom→base_link TF, Cartographer 구성이면 mission_map→map 정적 TF 를 앞에 붙임) 자세로 Nav2 지도(map_mission_nav)에 놓고
가장 가까운 점유 칸까지 거리(10 cm 에서 자름)를 잰다. 참값에서도 크면 지도 · 센서 쪽, 참값에선 작은데 AMCL 자세에서도 작으면 AMCL 이 틀린 자세를 맞다고 보는 것.
사용: python3 diag_scan_map.py <결과 폴더> [시작 s] [끝 s]"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bagutil as m  # noqa: E402

OUT = Path(sys.argv[1])
T0, T1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0.0, 1e9)
MAP = Path('/home/pa4/git/lv2_next_module/261031_1325_nav2/config/map_mission_nav.yaml')
meta = yaml.safe_load(MAP.read_text())
img = cv2.imread(str(MAP.parent / meta['image']), cv2.IMREAD_GRAYSCALE)
occ = ((255 - img.astype(float)) / 255.0) > meta['occupied_thresh']
res, (ox, oy) = meta['resolution'], meta['origin'][:2]
H, W = img.shape
dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res


def map_dist(pts):
    c = np.floor((pts[:, 0] - ox) / res).astype(int)
    r = H - 1 - np.floor((pts[:, 1] - oy) / res).astype(int)
    ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    d = np.full(len(pts), 0.10)
    d[ok] = np.minimum(dist[r[ok], c[ok]], 0.10)
    return d


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


bag = str(OUT / 'out_bag')
static = {}
for _, msg, _ in m.read_bag(bag, ['/tf_static']):
    for tr in msg.transforms:
        T = np.eye(4)
        T[:3, :3] = quat_mat(tr.transform.rotation)
        T[:3, 3] = [tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z]
        static[tr.child_frame_id] = (tr.header.frame_id, T)


def to_base(frame):
    T = np.eye(4)
    while frame != 'base_link':
        p, Tp = static[frame]
        T = Tp @ T
        frame = p
    return T


truth = np.array([(m.stamp(x.header), x.pose.position.x, x.pose.position.y, m.yaw_of(x.pose.orientation)) for _, x, _ in m.read_bag(bag, ['/sim/truth'])])
tf = m.read_tf(bag)
mo, ob = tf[('map', 'odom')], tf[('odom', 'base_link')]
if static.get('map', ('',))[0] == 'mission_map':        # 설계 18(Cartographer): map = Cartographer, 지도 = mission_map 좌표 → 정적 TF 를 앞에 붙임
    Tm = static['map'][1]
    mm = (Tm[0, 3], Tm[1, 3], math.atan2(Tm[1, 0], Tm[0, 0]))
    c0, s0 = math.cos(mm[2]), math.sin(mm[2])
    mo = np.column_stack([mo[:, 0], mm[0] + c0 * mo[:, 1] - s0 * mo[:, 2], mm[1] + s0 * mo[:, 1] + c0 * mo[:, 2], mo[:, 3] + mm[2]])


def at(s, t):
    i = np.clip(np.searchsorted(s[:, 0], t, side='right') - 1, 0, len(s) - 1)
    return s[i]


def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


rows = []
Tl = {}
for topic, msg, _ in m.read_bag(bag, ['/scan_front', '/scan_rear']):
    t = m.stamp(msg.header)
    if not T0 <= t <= T1:
        continue
    if msg.header.frame_id not in Tl:
        Tl[msg.header.frame_id] = to_base(msg.header.frame_id)
    r = np.array(msg.ranges)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.32) & (r < 8.0)          # 차체 가림(≤ 0.32 m) 빼고
    p = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum()), np.ones(ok.sum())])
    pb = (Tl[msg.header.frame_id] @ p)[:2].T             # base_link 기준
    out = []
    for pose in (at(truth, t)[1:4], comp(at(mo, t)[1:4], at(ob, t)[1:4])):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pw = np.stack([pose[0] + c * pb[:, 0] - s * pb[:, 1], pose[1] + s * pb[:, 0] + c * pb[:, 1]], 1)
        out.append(float(np.mean(map_dist(pw))))
    q, e = at(truth, t), comp(at(mo, t)[1:4], at(ob, t)[1:4])
    rows.append((t, topic, out[0], out[1], math.hypot(q[1] - e[0], q[2] - e[1])))
rows = np.array([(r[0], 0 if r[1] == '/scan_front' else 1, r[2], r[3], r[4]) for r in rows])
for k, name in ((0, '앞'), (1, '뒤')):
    s = rows[rows[:, 1] == k]
    print(f'{name}: 스캔 {len(s)}, 지도 거리 평균 — 참값 자세 중앙 {np.median(s[:, 2]) * 100:.2f} cm · p95 {np.percentile(s[:, 2], 95) * 100:.2f}, '
          f'추정 자세 중앙 {np.median(s[:, 3]) * 100:.2f} cm · p95 {np.percentile(s[:, 3], 95) * 100:.2f}')
for t in np.arange(max(T0, rows[0, 0]), min(T1, rows[-1, 0]), 4.0):
    s = rows[(rows[:, 0] >= t) & (rows[:, 0] < t + 4.0)]
    if len(s):
        print(f'{t:6.0f}~{t + 4:4.0f} s 참값 {np.mean(s[:, 2]) * 100:5.2f} cm | 추정 {np.mean(s[:, 3]) * 100:5.2f} cm | 위치 오차 {np.mean(s[:, 4]):.3f} m')
