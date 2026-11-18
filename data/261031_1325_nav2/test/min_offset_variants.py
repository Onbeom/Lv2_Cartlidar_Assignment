"""오프라인 비교(판정 아님, GPU 없음, 11-02 '5 cm 어긋남 감지 개선' ①): 잔차 계산 방식별로 정상 잔차와 작은 어긋남 감지 비율을 같은 스캔으로 비교.
방식: A 지금(칸 값 · 중앙값) · B 쌍선형 보간 · 중앙값 · C 보간 · p75 · D 보간 · 0.05 m 에서 자른 평균 · E 양쪽 거리장(벽 안쪽도 경계까지 거리) 보간 · 중앙값 · F E + p75.
정상 잔차 = 추정 자세(mission_map 정적 TF · map→odom · odom→base_link) · 참값 자세. 어긋남 = 참값 자세를 이동만(방향 무작위) · 회전만.
문턱 = 방식마다 '추정 자세 정상 최대 + 0.005'(지금 0.025 → 0.03 과 같은 여유) → 그 문턱을 넘는 스캔 비율.
사용: python3 min_offset_variants.py <결과 폴더> [<결과 폴더> ...] → 첫 폴더/min_offset_variants.json (정상 분포는 모든 폴더, 어긋남은 각 폴더 스캔 1/5)"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, '/home/pa4/git/lv2_next_module/261101_0016_isaac_live')
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import bagutil as m  # noqa: E402
import loc_core as lc  # noqa: E402

MAP = Path(__file__).resolve().parent.parent / 'config' / 'map_mission_nav.yaml'
field = lc.DistField(MAP)
res = field.res
occ = field.dist <= 0.0
d_out = np.minimum(cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res, field.clip)
d_in = cv2.distanceTransform(occ.astype(np.uint8), cv2.DIST_L2, 5) * res
two = np.where(occ, np.maximum(d_in - res / 2, 0.0), np.maximum(d_out - res / 2, 0.0)).astype(np.float32)   # 점유 경계까지 거리(양쪽)


def bilinear(grid, xy):
    """칸 중심 기준 쌍선형 보간(지도 밖은 clip)"""
    fx = (xy[..., 0] - field.ox) / res - 0.5
    fy = (field.H - 1) - ((xy[..., 1] - field.oy) / res - 0.5)
    c0, r0 = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
    ax, ay = fx - c0, fy - r0
    ok = (c0 >= 0) & (c0 + 1 < field.W) & (r0 >= 0) & (r0 + 1 < field.H)
    out = np.full(xy.shape[:-1], field.clip, np.float32)
    c, r, x, y = c0[ok], r0[ok], ax[ok], ay[ok]
    out[ok] = (grid[r, c] * (1 - x) * (1 - y) + grid[r, c + 1] * x * (1 - y) + grid[r + 1, c] * (1 - x) * y + grid[r + 1, c + 1] * x * y)
    return out


def dists(kind, pose, pts):
    xy = lc.place(np.asarray([pose], float), pts)[0]
    if kind == 'nearest':
        return field.lookup(xy)
    return bilinear(field.dist if kind == 'bilinear' else two, xy)


VARIANTS = {'A_cell_median': ('nearest', lambda d: np.median(d)), 'B_interp_median': ('bilinear', lambda d: np.median(d)),
            'C_interp_p75': ('bilinear', lambda d: np.percentile(d, 75)), 'D_interp_mean_clip05': ('bilinear', lambda d: np.minimum(d, 0.05).mean()),
            'E_two_median': ('two', lambda d: np.median(d)), 'F_two_p75': ('two', lambda d: np.percentile(d, 75))}


def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def load(out):
    bag = str(out / 'out_bag')
    static, static3 = {}, {}
    for _, msg, _ in m.read_bag(bag, ['/tf_static']):
        for tr in msg.transforms:
            t = tr.transform
            static[tr.child_frame_id] = (tr.header.frame_id, (t.translation.x, t.translation.y, m.yaw_of(t.rotation)))
            static3[tr.child_frame_id] = (tr.header.frame_id, lc.T2d_from_tf(t.translation.x, t.translation.y, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def to_base(frame):
        T = np.column_stack([np.eye(2), [0.0, 0.0]])
        while frame != 'base_link':
            p, Tp = static3[frame]
            T = lc.T2d_compose(Tp, T)
            frame = p
        return T
    truth = np.array([(m.stamp(x.header), x.pose.position.x, x.pose.position.y, m.yaw_of(x.pose.orientation)) for _, x, _ in m.read_bag(bag, ['/sim/truth'])])
    tf = m.read_tf(bag)
    mm = static['map'][1] if static.get('map', ('',))[0] == 'mission_map' else (0.0, 0.0, 0.0)
    scans = {}
    for topic, msg, _ in m.read_bag(bag, ['/scan_front', '/scan_rear']):
        st = round(m.stamp(msg.header), 3)
        scans.setdefault(st, {})[topic] = lc.scan_points(msg.ranges, msg.angle_min, msg.angle_increment, to_base(msg.header.frame_id), r_min=0.32, n_max=90)
    pairs = sorted((t, np.concatenate([v['/scan_front'], v['/scan_rear']])) for t, v in scans.items() if len(v) == 2)
    pairs = [p for p in pairs if p[0] > truth[0, 0] + 15.0]
    return truth, tf[('map', 'odom')], tf[('odom', 'base_link')], mm, pairs


def at(s, t):
    return s[min(max(np.searchsorted(s[:, 0], t, side='right') - 1, 0), len(s) - 1)]


pct = lambda a: {k: round(float(np.percentile(a, p)), 4) for k, p in (('p50', 50), ('p95', 95), ('p99', 99), ('max', 100))}
outs = [Path(p) for p in sys.argv[1:]]
rng = np.random.default_rng(20261102)
cases = [('xy', d) for d in (0.025, 0.035, 0.05, 0.075, 0.10)] + [('yaw', a) for a in (1, 2, 3, 5)]
norm = {k: {'est': [], 'truth': []} for k in VARIANTS}
off = {k: {f'{c[0]}_{c[1]}': [] for c in cases} for k in VARIANTS}
n_pairs = {}
for out in outs:
    truth, mo, ob, mm, pairs = load(out)
    n_pairs[out.name] = len(pairs)
    print(out.name, '스캔 쌍', len(pairs), flush=True)
    for i, (t, pts) in enumerate(pairs[::5]):
        q = at(truth, t)[1:4]
        e = comp(mm, comp(at(mo, t)[1:4], at(ob, t)[1:4]))
        offp = []
        for kind, v in cases:
            if kind == 'xy':
                ang = rng.uniform(0, 2 * math.pi)
                offp.append((f'{kind}_{v}', (q[0] + v * math.cos(ang), q[1] + v * math.sin(ang), q[2])))
            else:
                offp.append((f'{kind}_{v}', (q[0], q[1], q[2] + rng.choice([-1, 1]) * math.radians(v))))
        for k, (dk, agg) in VARIANTS.items():
            norm[k]['est'].append(float(agg(dists(dk, e, pts))))
            norm[k]['truth'].append(float(agg(dists(dk, q, pts))))
            for name, p in offp:
                off[k][name].append(float(agg(dists(dk, p, pts))))
rep = {'folders': n_pairs, 'variants': {}}
for k in VARIANTS:
    est = np.array(norm[k]['est'])
    thr = round(float(est.max()) + 0.005, 4)
    row = {'normal_est': pct(est), 'normal_truth': pct(norm[k]['truth']), 'thr': thr,
           'over_thr_frac': {n: round(float(np.mean(np.array(v) > thr)), 3) for n, v in off[k].items()},
           'p50': {n: round(float(np.median(v)), 4) for n, v in off[k].items()}}
    rep['variants'][k] = row
    print(k, 'est', row['normal_est'], 'thr', thr, json.dumps(row['over_thr_frac']), flush=True)
json.dump(rep, open(outs[0] / 'min_offset_variants.json', 'w'), ensure_ascii=False, indent=1)
