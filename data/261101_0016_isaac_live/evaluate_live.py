"""1단계 SLAM 실시간 연결 채점(모의). 사용: python3 evaluate_live.py <결과 폴더>  → <결과 폴더>/evaluate.json
261031_0335_slam_launch/test/evaluate_launch.py(bag 재생 확인)와 같은 A~E 에 실시간 연결 판정 S1~S4 를 더함(기준 사용자 승인 2026-11-01):
  S1 /clock 이 앞으로만 움직이고 모든 노드 use_sim_time true
  S2 /scan_front · /scan_rear · /odom 끊김 없음 = 받은 수 ≥ 장면이 낸 수의 99 % 이고 stamp 간격 최대 ≤ 주기의 2 배
  S3 mission_map → 앞 카메라(infra1 optical) TimePointZero 조회 ≥ 99 %, 나이 p95 ≤ 0.05 s
  S4 영상 stamp − 받은 EKF odom→base_link 최신 stamp ≤ 0.1 s(넘는 것 0)
A · B(위치 정확도)는 판정이 아니라 bag 재생 결과(밝음 A 0.0424 · B 0.0089 m)와 비교하려는 기록."""
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

OUT = Path(sys.argv[1]).resolve()
REC = OUT / 'scene'
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bagutil as m  # noqa: E402
USD_OFFSET = np.array([2.2345, 3.2136])


def by_rx(series):
    return series[np.argsort(series[:, 4], kind='stable')]


def at_rx(s, t):
    i = np.searchsorted(s[:, 4], t, side='right') - 1
    return None if i < 0 else s[i]


def probe(path, t0, t1):
    rows = [r for r in csv.DictReader(open(path)) if t0 < float(r['now_s']) <= t1]
    age = np.array([float(r['zero_age_s']) for r in rows if r['zero_ok'] == '1'])
    return {'n': len(rows), 'zero_ok_ratio': float(np.mean([r['zero_ok'] == '1' for r in rows])) if rows else 0.0,
            'zero_age_s_median': float(np.median(age)) if len(age) else None, 'zero_age_s_p95': float(np.percentile(age, 95)) if len(age) else None,
            'zero_age_s_max': float(age.max()) if len(age) else None, 'now_ok_ratio': float(np.mean([r['now_ok'] == '1' for r in rows])) if rows else 0.0}


bag = str(OUT / 'out_bag')
tf = m.read_tf(bag)
mo, ob = by_rx(tf[('map', 'odom')]), by_rx(tf[('odom', 'base_link')])
tr = m.truth(str(REC))
st = tr[0, 1:4]
al = json.loads((OUT / 'map' / 'align.json').read_text())['mission_from_carto_map']
ca, sa = math.cos(al['yaw']), math.sin(al['yaw'])
eA, hA, eB, hB = [], [], [], []
T_END = min(tr[-1, 0], mo[:, 4].max(), ob[:, 4].max())
T0 = max(mo[0, 4], ob[0, 4]) + 0.5
for t in np.arange(T0, T_END, 0.05):
    a, b = at_rx(mo, t), at_rx(ob, t)
    p = m.compose(a[1:4], b[1:4])
    j = min(np.searchsorted(tr[:, 0], t), len(tr) - 1)
    w = m.to_world(np.array([p[:2]]), st)[0]
    eA.append(np.linalg.norm(w - tr[j, 1:3]))
    hA.append(math.degrees(abs(m.wrap(p[2] + st[2] - tr[j, 3]))))
    q = np.array([al['x'] + ca * p[0] - sa * p[1], al['y'] + sa * p[0] + ca * p[1]]) - USD_OFFSET
    eB.append(np.linalg.norm(q - tr[j, 1:3]))
    hB.append(math.degrees(abs(m.wrap(p[2] + al['yaw'] - tr[j, 3]))))
stat = lambda v: {'rmse': float(np.sqrt((np.array(v) ** 2).mean())), 'p95': float(np.percentile(v, 95)), 'max': float(np.max(v))}
res = {'simulation_only': True, 'live_isaac': True, 'out': str(OUT), 'eval_start_s': float(T0), 'eval_end_s': float(T_END), 'truth_end_s': float(tr[-1, 0]),
       'A_realtime_pos_m': stat(eA), 'A_realtime_yaw_deg': stat(hA), 'B_after_map_align_pos_m': stat(eB), 'B_after_map_align_yaw_deg': stat(hB)}

# S1 /clock · use_sim_time
clk = np.array([msg.clock.sec + msg.clock.nanosec * 1e-9 for _, msg, _ in m.read_bag(bag, ['/clock'])])
ust = (OUT / 'use_sim_time.txt').read_text().splitlines() if (OUT / 'use_sim_time.txt').exists() else []
ust = [ln for ln in ust if not ln.startswith('/transform_listener_impl_')]   # tf2 가 탐침 안에 만드는 내부 노드(매개변수 서비스 없음)
bad_nodes = [ln for ln in ust if 'True' not in ln]
res['S1'] = {'clock_n': int(len(clk)), 'clock_first_s': float(clk[0]), 'clock_last_s': float(clk[-1]),
             'clock_backwards': int((np.diff(clk) < 0).sum()), 'clock_repeats': int((np.diff(clk) == 0).sum()),
             'nodes': len(ust), 'nodes_not_sim_time': bad_nodes}
res['S1']['pass'] = bool(len(clk) > 0 and res['S1']['clock_backwards'] == 0 and clk[-1] > clk[0] and len(ust) > 0 and not bad_nodes)

# S2 입력 끊김
sc = json.loads((REC / 'scene_counts.json').read_text())
sent = {'/scan_front': sc['sent']['scan']['/scan_front'], '/scan_rear': sc['sent']['scan']['/scan_rear'], '/odom': sc['sent']['odom']}
period = {'/scan_front': 0.1, '/scan_rear': 0.1, '/odom': 0.02}
s2 = {}
for topic in sent:
    stm = np.sort(np.array([m.stamp(msg.header) for _, msg, _ in m.read_bag(bag, [topic])]))
    gap = float(np.diff(stm).max()) if len(stm) > 1 else float('inf')
    s2[topic] = {'received': int(len(stm)), 'sent': int(sent[topic]), 'ratio': float(len(stm) / max(sent[topic], 1)),
                 'max_stamp_gap_s': gap, 'period_s': period[topic]}
    s2[topic]['pass'] = bool(s2[topic]['ratio'] >= 0.99 and gap <= 2 * period[topic] + 1e-6)
res['S2'] = s2
res['S2_pass'] = all(v['pass'] for v in s2.values())

# S3 TF 탐침(지도 · 중계가 생긴 뒤부터 채점 끝까지)
res['S3_probe_mission_map_to_camera'] = probe(OUT / 'tf_probe_cam.csv', T0, T_END)
res['D_probe_map_to_base_link'] = probe(OUT / 'tf_probe.csv', T0, T_END)
p3 = res['S3_probe_mission_map_to_camera']
res['S3_pass'] = bool(p3['zero_ok_ratio'] >= 0.99 and p3['zero_age_s_p95'] is not None and p3['zero_age_s_p95'] <= 0.05)

# S4 영상 stamp − EKF 최신 stamp
lag = []
for _, msg, rx in m.read_bag(bag, ['/top_camera_front/infra1/camera_info']):
    k = at_rx(ob, rx)
    if k is not None:
        lag.append(m.stamp(msg.header) - k[0])
lag = np.array(lag)
res['S4_image_minus_ekf_stamp_s'] = {'n': int(len(lag)), 'median': float(np.median(lag)), 'p95': float(np.percentile(lag, 95)),
                                     'max': float(lag.max()), 'over_0.1s': int((lag > 0.1).sum())}
res['S4_pass'] = bool(len(lag) > 0 and res['S4_image_minus_ekf_stamp_s']['over_0.1s'] == 0)
res['ekf_tf_period_s_median'] = float(np.median(np.diff(np.sort(ob[:, 0]))))
res['relay_tf_period_s_median'] = float(np.median(np.diff(np.sort(mo[:, 4]))))
res['relay_stamp_minus_rx_s_median'] = float(np.median(mo[:, 0] - mo[:, 4]))

# E 마스크 · 장면
last = {}
for ln in (OUT / 'launch.log').read_text().splitlines():
    g = re.search(r'(/scan_\w+): 스캔 (\d+), 평균 가린 칸 ([\d.]+)', ln)
    if g:
        last[g.group(1)] = {'scans': int(g.group(2)), 'mean_masked_bins': float(g.group(3)), 'deg': float(g.group(3)) * 0.9}
res['E_mask'] = last
res['scene_counts'] = sc
res['map_align'] = json.loads((OUT / 'map' / 'align.json').read_text())
res['stage1_pass'] = bool(res['S1']['pass'] and res['S2_pass'] and res['S3_pass'] and res['S4_pass'])
(OUT / 'evaluate.json').write_text(json.dumps(res, ensure_ascii=False, indent=2))
print(json.dumps({k: v for k, v in res.items() if k not in ('map_align', 'scene_counts')}, ensure_ascii=False))
