"""launch 확인 채점(모의). 사용: python evaluate_launch.py <bright|dark> <녹화 폴더>  → test/results/<이름>/evaluate.json
A 실시간 자세: 시각 t 에 이미 받은 map→odom(중계) · odom→base_link(EKF)를 합성(수신 시각으로 고름) vs 정답, 출발 자세 기준(261030_2058 와 같은 채점).
B 도면 맞춤 뒤: save_and_align_map 의 변환(발제 ← Cartographer map)으로 자세를 발제 → USD 좌표로 옮겨 정답과 바로 비교(정답 · 출발 자세를 쓰지 않는 독립 확인).
C 조건 1(인지 세션): 영상(camera_info) 수신 때 그 영상 stamp − 이미 받은 EKF odom→base_link 의 최신 stamp(> 0.1 s 면 인지가 base 기준으로 바뀜).
D TF 탐침: map→base_link TimePointZero 조회 나이 · 지금 조회 성공 비율. E 마스크: launch 로그의 평균 가린 칸."""
import csv
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

NAME, REC = sys.argv[1], sys.argv[2]
OUT = Path(__file__).resolve().parent / 'results' / NAME
spec = importlib.util.spec_from_file_location('old_eval', '/home/pa4/git/lv2_next_module/261027_0953/slam/eval_slam.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
USD_OFFSET = np.array([2.2345, 3.2136])


def by_rx(series):
    return series[np.argsort(series[:, 4], kind='stable')]


def at_rx(s, t):
    i = np.searchsorted(s[:, 4], t, side='right') - 1
    return None if i < 0 else s[i]


tf = m.read_tf(str(OUT / 'out_bag'))
mo, ob = by_rx(tf[('map', 'odom')]), by_rx(tf[('odom', 'base_link')])
tr = m.truth(REC)
st = tr[0, 1:4]
al = json.loads((OUT / 'map' / 'align.json').read_text())['mission_from_carto_map']
ca, sa = math.cos(al['yaw']), math.sin(al['yaw'])
eA, hA, eB, hB = [], [], [], []
T_END = min(tr[-1, 0], mo[:, 4].max(), ob[:, 4].max())   # 기록이 끝난 뒤는 채점하지 않음(녹화기 종료 때 끝 몇 초가 빠질 수 있음)
for t in np.arange(max(mo[0, 4], ob[0, 4]) + 0.5, T_END, 0.05):
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
res = {'simulation_only': True, 'name': NAME, 'recording': REC, 'eval_end_s': float(T_END), 'truth_end_s': float(tr[-1, 0]),
       'A_realtime_pos_m': stat(eA), 'A_realtime_yaw_deg': stat(hA), 'B_after_map_align_pos_m': stat(eB), 'B_after_map_align_yaw_deg': stat(hB)}
# C 조건 1
cam = []
for _, msg, rx in m.read_bag(str(OUT / 'out_bag'), ['/top_camera_front/infra1/camera_info']):
    cam.append((m.stamp(msg.header), rx))
lag = []
for s, rx in cam:
    k = at_rx(ob, rx)
    if k is not None:
        lag.append(s - k[0])
lag = np.array(lag)
res['C_image_minus_ekf_stamp_s'] = {'n': int(len(lag)), 'median': float(np.median(lag)), 'p95': float(np.percentile(lag, 95)),
                                    'max': float(lag.max()), 'over_0.1s': int((lag > 0.1).sum())}
res['ekf_tf_period_s_median'] = float(np.median(np.diff(np.sort(ob[:, 0]))))
res['relay_tf_period_s_median'] = float(np.median(np.diff(np.sort(mo[:, 4]))))
res['relay_stamp_minus_rx_s_median'] = float(np.median(mo[:, 0] - mo[:, 4]))
# D 탐침
rows = list(csv.DictReader(open(OUT / 'tf_probe.csv')))
rows = [r for r in rows if mo[0, 4] + 0.5 < float(r['now_s']) <= T_END]
age = np.array([float(r['zero_age_s']) for r in rows if r['zero_ok'] == '1'])
res['D_probe'] = {'n': len(rows), 'zero_ok_ratio': float(np.mean([r['zero_ok'] == '1' for r in rows])),
                  'zero_age_s_median': float(np.median(age)), 'zero_age_s_p95': float(np.percentile(age, 95)), 'zero_age_s_max': float(age.max()),
                  'now_ok_ratio': float(np.mean([r['now_ok'] == '1' for r in rows]))}
# E 마스크
last = {}
for ln in (OUT / 'launch.log').read_text().splitlines():
    g = re.search(r'(/scan_\w+): 스캔 (\d+), 평균 가린 칸 ([\d.]+)', ln)
    if g:
        last[g.group(1)] = {'scans': int(g.group(2)), 'mean_masked_bins': float(g.group(3)), 'deg': float(g.group(3)) * 0.9}
res['E_mask'] = last
res['map_align'] = json.loads((OUT / 'map' / 'align.json').read_text())
(OUT / 'evaluate.json').write_text(json.dumps(res, ensure_ascii=False, indent=2))
print(json.dumps({k: v for k, v in res.items() if k != 'map_align'}, ensure_ascii=False))
