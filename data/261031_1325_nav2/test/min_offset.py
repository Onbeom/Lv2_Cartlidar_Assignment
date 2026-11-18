"""(min_offset.py 의 앞부분은 check_reloc_offline.py 와 같음)
오프라인 확인(판정 아님, GPU 없음): Isaac 실시간 Nav2 녹화(out_bag)의 라이다 스캔으로 loc_core 를 시험한다.
① 정상 잔차: 추정 자세(mission_map 정적 TF · map→odom · odom→base_link)와 참값 자세에서 스캔 끝점 거리 중앙값 분포 → 감지 문턱값 근거.
② 근처 탐색(K1 꼴): 참값에서 0.20 m · 10° 어긋난 자세를 중심으로 ±0.5 m · ±30° 탐색 → 참값과 오차.
③ 지도 전체 탐색(K2 · K3 꼴): 중심 없이 탐색 → 참값과 오차, 1 · 2등 점수 차(모호성).
앞 · 뒤 스캔은 같은 stamp 끼리 합침. 차체 가림은 r > 0.32 m 로 뺌(diag_scan_map.py 와 같음).
사용: python3 min_offset.py <결과 폴더> 0 [res_thr, 기본 0.03] → <결과 폴더>/min_offset.json"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/pa4/git/lv2_next_module/261101_0016_isaac_live')
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
import bagutil as m  # noqa: E402
import loc_core as lc  # noqa: E402

OUT = Path(sys.argv[1])
N = int(sys.argv[2]) if len(sys.argv) > 2 else 30
MAP = Path(__file__).resolve().parent.parent / 'config' / 'map_mission_nav.yaml'
field = lc.DistField(MAP)
bag = str(OUT / 'out_bag')

static, static3 = {}, {}
for _, msg, _ in m.read_bag(bag, ['/tf_static']):
    for tr in msg.transforms:
        t = tr.transform
        static[tr.child_frame_id] = (tr.header.frame_id, (t.translation.x, t.translation.y, m.yaw_of(t.rotation)))
        static3[tr.child_frame_id] = (tr.header.frame_id, lc.T2d_from_tf(t.translation.x, t.translation.y, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))


def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def to_base(frame):
    """라이다 frame → base_link 2×3(roll π 장착 반영)"""
    T = np.column_stack([np.eye(2), [0.0, 0.0]])
    while frame != 'base_link':
        p, Tp = static3[frame]
        T = lc.T2d_compose(Tp, T)
        frame = p
    return T


truth = np.array([(m.stamp(x.header), x.pose.position.x, x.pose.position.y, m.yaw_of(x.pose.orientation)) for _, x, _ in m.read_bag(bag, ['/sim/truth'])])
tf = m.read_tf(bag)
mo, ob = tf[('map', 'odom')], tf[('odom', 'base_link')]
MM = static['map'][1] if static.get('map', ('',))[0] == 'mission_map' else (0.0, 0.0, 0.0)


def at(s, t):
    return s[min(max(np.searchsorted(s[:, 0], t, side='right') - 1, 0), len(s) - 1)]


scans = {}
for topic, msg, _ in m.read_bag(bag, ['/scan_front', '/scan_rear']):
    st = round(m.stamp(msg.header), 3)
    pts = lc.scan_points(msg.ranges, msg.angle_min, msg.angle_increment, to_base(msg.header.frame_id), r_min=0.32, n_max=90)
    scans.setdefault(st, {})[topic] = pts
pairs = sorted((t, np.concatenate([v['/scan_front'], v['/scan_rear']])) for t, v in scans.items() if len(v) == 2)
t_first = truth[0, 0] + 15.0
pairs = [p for p in pairs if p[0] > t_first]
print('스캔 쌍', len(pairs))

pct = lambda a: {k: round(float(np.percentile(a, p)), 4) for k, p in (('p50', 50), ('p95', 95), ('p99', 99), ('max', 100))}

# 잡을 수 있는 가장 작은 어긋남(인지 세션 권함 2026-11-02): 참값 자세를 이동만 · 회전만 어긋나게 해 잔차가 문턱(res_thr)을 넘는 비율.
THR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.03
rng = np.random.default_rng(20261102)
sub = pairs[::5]
cases = [('xy', d) for d in (0.025, 0.035, 0.05, 0.075, 0.10, 0.15, 0.20)] + [('yaw', a) for a in (1, 2, 3, 5, 7, 10)]
out = {'n': len(sub), 'res_thr': THR, 'rows': []}
for kind, v in cases:
    rr = []
    for t, pts in sub:
        q = at(truth, t)[1:4]
        if kind == 'xy':
            ang = rng.uniform(0, 2 * math.pi); p = (q[0] + v * math.cos(ang), q[1] + v * math.sin(ang), q[2])
        else:
            p = (q[0], q[1], q[2] + rng.choice([-1, 1]) * math.radians(v))
        rr.append(lc.residual(field, p, pts))
    rr = np.array(rr)
    row = {'kind': kind, 'value': v, 'over_thr_frac': round(float(np.mean(rr > THR)), 3), **pct(rr)}
    out['rows'].append(row); print(json.dumps(row, ensure_ascii=False))
json.dump(out, open(OUT / 'min_offset.json', 'w'), indent=1)
