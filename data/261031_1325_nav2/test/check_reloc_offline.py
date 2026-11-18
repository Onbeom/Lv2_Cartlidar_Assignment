"""오프라인 확인(판정 아님, GPU 없음): Isaac 실시간 Nav2 녹화(out_bag)의 라이다 스캔으로 loc_core 를 시험한다.
① 정상 잔차: 추정 자세(mission_map 정적 TF · map→odom · odom→base_link)와 참값 자세에서 스캔 끝점 거리 중앙값 분포 → 감지 문턱값 근거.
② 근처 탐색(K1 꼴): 참값에서 0.20 m · 10° 어긋난 자세를 중심으로 ±0.5 m · ±30° 탐색 → 참값과 오차.
③ 지도 전체 탐색(K2 · K3 꼴): 중심 없이 탐색 → 참값과 오차, 1 · 2등 점수 차(모호성).
앞 · 뒤 스캔은 같은 stamp 끼리 합침. 차체 가림은 r > 0.32 m 로 뺌(diag_scan_map.py 와 같음).
사용: python3 check_reloc_offline.py <결과 폴더> [표본 수, 기본 30] → <결과 폴더>/reloc_offline.json"""
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

res_est, res_truth = [], []
for t, pts in pairs[::5]:
    q = at(truth, t)[1:4]
    e = comp(MM, comp(at(mo, t)[1:4], at(ob, t)[1:4]))
    res_truth.append(lc.residual(field, q, pts))
    res_est.append(lc.residual(field, e, pts))
pct = lambda a: {k: round(float(np.percentile(a, p)), 4) for k, p in (('p50', 50), ('p95', 95), ('p99', 99), ('max', 100))}
rep = {'n_scan_pairs': len(pairs), 'residual_est_m': pct(res_est), 'residual_truth_m': pct(res_truth)}
print('정상 잔차(추정 자세)', rep['residual_est_m'], '참값 자세', rep['residual_truth_m'])

rng = np.random.default_rng(20261101)
idx = rng.choice(len(pairs), size=min(N, len(pairs)), replace=False)
loc_rows, glob_rows = [], []
for k in idx:
    t, pts = pairs[k]
    q = at(truth, t)[1:4]
    ang = rng.uniform(0, 2 * math.pi)
    sgn = rng.choice([-1, 1])
    off = (q[0] + 0.20 * math.cos(ang), q[1] + 0.20 * math.sin(ang), q[2] + sgn * math.radians(10))
    r_off = lc.residual(field, off, pts)
    L = lc.search(field, pts, center=off)
    G = lc.search(field, pts, center=None)
    err = lambda p: (math.hypot(p[0] - q[0], p[1] - q[1]), abs(math.degrees(math.atan2(math.sin(p[2] - q[2]), math.cos(p[2] - q[2])))))
    le, ge = err(L['best']), err(G['best'])
    loc_rows.append({'t': t, 'residual_off': round(r_off, 4), 'err_m': round(le[0], 4), 'err_deg': round(le[1], 2), 'score': round(L['score'], 4),
                     'second_score': None if L['second_score'] is None else round(L['second_score'], 4)})
    glob_rows.append({'t': t, 'err_m': round(ge[0], 4), 'err_deg': round(ge[1], 2), 'score': round(G['score'], 4),
                      'second_score': None if G['second_score'] is None else round(G['second_score'], 4), 'second': G['second'], 'n_cand': G['n_cand']})
    print(f"t {t:7.2f} 어긋난 자세 잔차 {r_off:.3f} | 근처 {le[0]:.3f} m {le[1]:.1f}° 점수 {L['score']:.4f} | "
          f"전체 {ge[0]:.3f} m {ge[1]:.1f}° 점수 {G['score']:.4f} 2등 {G['second_score'] if G['second_score'] is None else round(G['second_score'], 4)}", flush=True)
ok = lambda rows: sum(1 for r in rows if r['err_m'] <= 0.05 and r['err_deg'] <= 3.0)
rep |= {'local': {'n': len(loc_rows), 'ok_0.05m_3deg': ok(loc_rows), 'residual_off': pct([r['residual_off'] for r in loc_rows]), 'rows': loc_rows},
        'global': {'n': len(glob_rows), 'ok_0.05m_3deg': ok(glob_rows),
                   'margin_second_minus_best': pct([r['second_score'] - r['score'] for r in glob_rows if r['second_score'] is not None]), 'rows': glob_rows}}
json.dump(rep, open(OUT / 'reloc_offline.json', 'w'), ensure_ascii=False, indent=1)
print('근처 탐색 성공', rep['local']['ok_0.05m_3deg'], '/', len(loc_rows), '· 전체 탐색 성공', rep['global']['ok_0.05m_3deg'], '/', len(glob_rows))
print('전체 탐색 2등 − 1등 점수', rep['global']['margin_second_minus_best'])
