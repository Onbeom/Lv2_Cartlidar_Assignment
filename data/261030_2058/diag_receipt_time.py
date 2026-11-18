"""실시간 관점 채점(모의, 진단): 시각 t 에 '이미 받은' TF 만 쓴다(stamp 대신 수신 시각으로 고름). 합성 자세 = map→odom(t 까지 받은 최신) ∘ odom→base_link(t 까지 받은 최신).
사용: python diag_receipt_time.py <이름표>=<녹화 폴더> ... (results/carto_<kind>_<이름표>[_V…] 중 lidar_only · lidar_tfodom_scaled 의 c6 · V4)"""
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('old_eval', '/home/pa4/git/lv2_next_module/261027_0953/slam/eval_slam.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def at_rx(series, t):
    s = series[np.argsort(series[:, 4])]
    i = np.searchsorted(s[:, 4], t, side='right') - 1
    return None if i < 0 else s[i, 1:4]


rows = []
for label, rec in (a.split('=', 1) for a in sys.argv[1:]):
    tr = m.truth(rec)
    st = tr[0, 1:4]
    for kind in ('lidar_only', 'lidar_tfodom_scaled'):
        for tag in ('', '_V4_no_pose_extrapolator'):
            d = HERE / 'results' / f'carto_{kind}_{label}{tag}'
            tf = m.read_tf(str(d / 'out_bag'))
            mo, ob = tf[('map', 'odom')], tf[('odom', 'base_link')]
            e, eh = [], []
            for t in np.arange(max(mo[:, 4].min(), ob[:, 4].min()) + 0.5, tr[-1, 0], 0.05):
                a, b = at_rx(mo, t), at_rx(ob, t)
                p = m.compose(a, b)
                j = min(np.searchsorted(tr[:, 0], t), len(tr) - 1)
                w = m.to_world(np.array([p[:2]]), st)[0]
                e.append(np.linalg.norm(w - tr[j, 1:3]))
                eh.append(math.degrees(abs(m.wrap(p[2] + st[2] - tr[j, 3]))))
            e, eh = np.array(e), np.array(eh)
            rows.append({'label': label, 'kind': kind, 'variant': tag.lstrip('_') or 'c6', 'rx_rmse_m': float(np.sqrt((e ** 2).mean())),
                         'rx_p95_m': float(np.percentile(e, 95)), 'rx_yaw_rmse_deg': float(np.sqrt((eh ** 2).mean())),
                         'rx_yaw_p95_deg': float(np.percentile(eh, 95)),
                         'odom_base_rx_minus_stamp_ms_median': float(np.median(ob[:, 4] - ob[:, 0]) * 1e3),
                         'map_odom_rx_minus_stamp_ms_median': float(np.median(mo[:, 4] - mo[:, 0]) * 1e3)})
            print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
(HERE / 'results' / 'diag_receipt_time.json').write_text(json.dumps({'simulation_only': True, 'rows': rows}, ensure_ascii=False, indent=1))
