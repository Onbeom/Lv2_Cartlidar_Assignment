"""A-2 영상 시각 채점(모의, 11-03). 사용: python3 evaluate_a2.py <결과 폴더>  → <결과 폴더>/evaluate_a2.json
261101_0016_isaac_live/evaluate_live.py 의 S3 · S4 와 같은 계산에 영상 구독 탐침(img_lag.csv)을 더함. 프레임은 팀 규약 map(이전 mission_map).
  T1 영상 stamp − 그때 받은 EKF odom → base_link 최신 stamp ≤ 0.1 s(넘는 것 0) — ① bag camera_info 수신 시각(이전 방식) ② 영상 구독 탐침 둘 다
  T2 map → 앞 카메라(infra1 optical) TimePointZero 조회 ≥ 99 %, 나이(지금 − 돌려받은 stamp) p95 ≤ 0.05 s — tf_probe_cam.csv
채점 구간 = map → odom 이 처음 들어온 뒤 0.5 s ~ TF 기록 끝."""
import csv
import json
import sys
from pathlib import Path

import numpy as np

OUT = Path(sys.argv[1]).resolve()
sys.path.insert(0, '/home/pa4/git/lv2_next_module/261101_0016_isaac_live')
import bagutil as m  # noqa: E402

CAMS = ['infra1', 'infra2']


def by_rx(series):
    return series[np.argsort(series[:, 4], kind='stable')]


def at_rx(s, t):
    i = np.searchsorted(s[:, 4], t, side='right') - 1
    return None if i < 0 else s[i]


def dist(v):
    v = np.asarray(v, dtype=float)
    if not len(v):
        return {'n': 0}
    return {'n': int(len(v)), 'median': float(np.median(v)), 'p95': float(np.percentile(v, 95)), 'p99': float(np.percentile(v, 99)),
            'max': float(v.max()), 'min': float(v.min()), 'over_0.1s': int((v > 0.1).sum())}


def probe(path, t0, t1):
    rows = [r for r in csv.DictReader(open(path)) if t0 < float(r['now_s']) <= t1]
    age = np.array([float(r['zero_age_s']) for r in rows if r['zero_ok'] == '1'])
    return {'n': len(rows), 'zero_ok_ratio': float(np.mean([r['zero_ok'] == '1' for r in rows])) if rows else 0.0,
            'zero_age_s': dist(age), 'now_ok_ratio': float(np.mean([r['now_ok'] == '1' for r in rows])) if rows else 0.0}


bag = str(OUT / 'out_bag')
tf = m.read_tf(bag)
mo, ob = by_rx(tf[('map', 'odom')]), by_rx(tf[('odom', 'base_link')])
T0, T_END = max(mo[0, 4], ob[0, 4]) + 0.5, min(mo[:, 4].max(), ob[:, 4].max())
res = {'simulation_only': True, 'out': str(OUT), 'eval_start_s': float(T0), 'eval_end_s': float(T_END)}

# T1 ① bag camera_info(이전 방식)
sc = json.loads((OUT / 'scene' / 'scene_counts.json').read_text())
for c in CAMS:
    lag, st = [], []
    for _, msg, rx in m.read_bag(bag, [f'/top_camera_front/{c}/camera_info']):
        st.append(m.stamp(msg.header))
        k = at_rx(ob, rx)
        if k is not None and T0 < rx <= T_END:
            lag.append(m.stamp(msg.header) - k[0])
    st = np.sort(np.array(st))
    res[f'T1_bag_{c}_image_minus_ekf_s'] = dist(lag)
    res[f'bag_{c}_camera_info'] = {'received': int(len(st)), 'scene_sent': int(sc['sent']['ir'][c]),
                                   'max_stamp_gap_s': float(np.diff(st).max()) if len(st) > 1 else None}
# T1 ② 영상 구독 탐침
rows = [r for r in csv.DictReader(open(OUT / 'img_lag.csv')) if T0 < float(r['rx_s']) <= T_END]
lag2 = [float(r['lag_s']) for r in rows if r['lag_s'] != 'nan']
img = np.sort(np.array([float(r['img_s']) for r in rows]))
res['T1_probe_image_minus_ekf_s'] = dist(lag2)
res['probe_image'] = {'rows': len(rows), 'ekf_missing': int(sum(r['lag_s'] == 'nan' for r in rows)),
                      'rx_minus_img_s': dist([float(r['rx_s']) - float(r['img_s']) for r in rows]),
                      'max_stamp_gap_s': float(np.diff(img).max()) if len(img) > 1 else None,
                      'map_ok_ratio': float(np.mean([r['map_ok'] == '1' for r in rows])) if rows else 0.0,
                      'map_age_now_s': dist([float(r['map_age_now_s']) for r in rows if r['map_ok'] == '1']),
                      'map_age_img_s': dist([float(r['map_age_img_s']) for r in rows if r['map_ok'] == '1']),
                      'near_ok_ratio': float(np.mean([r['near_ok'] == '1' for r in rows])) if rows else 0.0}
lags = [res[f'T1_bag_{c}_image_minus_ekf_s'] for c in CAMS] + [res['T1_probe_image_minus_ekf_s']]
res['T1_pass'] = bool(all(d['n'] > 0 and d['over_0.1s'] == 0 for d in lags))

# T2 TF 탐침
res['T2_probe_map_to_camera'] = probe(OUT / 'tf_probe_cam.csv', T0, T_END)
res['D_probe_map_to_base_link'] = probe(OUT / 'tf_probe.csv', T0, T_END)
p = res['T2_probe_map_to_camera']
res['T2_pass'] = bool(p['zero_ok_ratio'] >= 0.99 and p['zero_age_s'].get('p95', 1e9) <= 0.05)

# 기록: TF 주기 · map → odom stamp 앞당김(AMCL transform_tolerance)
res['ekf_tf_period_s_median'] = float(np.median(np.diff(np.sort(ob[:, 0]))))
res['map_odom_tf_period_s_median'] = float(np.median(np.diff(np.sort(mo[:, 4]))))
res['map_odom_stamp_minus_rx_s_median'] = float(np.median(mo[:, 0] - mo[:, 4]))
res['ekf_stamp_minus_rx_s'] = dist(ob[:, 0] - ob[:, 4])
res['pass'] = bool(res['T1_pass'] and res['T2_pass'])
(OUT / 'evaluate_a2.json').write_text(json.dumps(res, ensure_ascii=False, indent=2))
print(json.dumps(res, ensure_ascii=False))
