"""설정 복사본 시험 채점(모의). evaluate.py 와 같은 지표 + 지도 전체 회전(시작점 둘레, |w| < 0.05) · 회전 중 yaw 오차 기울기(s × w).
기준 c6 r1(carto_<kind>_<light>) 도 같은 방식으로 다시 재서 표에 같이 넣는다. 결과 results/summary_variants.json(summary.json 은 건드리지 않음).
사용: python evaluate_variants.py <녹화 폴더 밝음> <녹화 폴더 어두움>"""
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path('/home/pa4/git/lv2_next_module')
OUT = Path(__file__).resolve().parent / 'results'
spec = importlib.util.spec_from_file_location('old_eval', ROOT / '261027_0953/slam/eval_slam.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
REC = {'bright': Path(sys.argv[1]), 'dark': Path(sys.argv[2])}
segs = json.load(open(ROOT / '261030_1943/maze_slice_z0.08.json'))['segments']
walls = []
for _, a, b in segs:
    a, b = np.array(a), np.array(b)
    walls += list(a + np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.01) + 1))[:, None] * (b - a))
walls = np.array(walls)
tree = cKDTree(walls)


def criteria(row):
    return {'position_rmse_0.05m': row.get('A1_rmse_m', np.inf) <= .05, 'yaw_rmse_3deg': row.get('A3_yaw_rmse_deg', np.inf) <= 3,
            'aligned_wall_error_3cm': row.get('M1_aligned_mean_err_cm', np.inf) <= 3,
            'aligned_ghost_ratio_0.1': row.get('M1_aligned_ghost_ratio', 1) <= .1, 'jumps_zero': row.get('jumps', -1) == 0}


def rotation_metrics(tf, tr, start):
    """지도 전체 회전[°](시작점 둘레 강체 회전, |w| < 0.05 구간)과 회전 중(|w| > 0.3) yaw 오차 = k[s]·w + c 의 k"""
    mo, ob = tf[('map', 'odom')], tf[('odom', 'base_footprint')]
    w = np.gradient(np.unwrap(tr[:, 3]), tr[:, 0])
    E, Q, W, EY = [], [], [], []
    for t in np.arange(1, tr[-1, 0], 0.05):
        e = m.compose(m.at(mo, t), m.at(ob, t))
        j = min(np.searchsorted(tr[:, 0], t), len(tr) - 1)
        E.append(m.to_world(np.array([e[:2]]), start)[0])
        Q.append(tr[j, 1:3])
        W.append(w[j])
        EY.append(m.wrap(e[2] + start[2] - tr[j, 3]))
    E, Q, W, EY = map(np.array, (E, Q, W, EY))
    s = np.abs(W) < 0.05
    p, q = E[s] - start[:2], Q[s] - start[:2]
    h = p.T @ q
    ang = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, sn = math.cos(ang), math.sin(ang)
    pr = np.stack([c * p[:, 0] - sn * p[:, 1], sn * p[:, 0] + c * p[:, 1]], 1)
    k = np.polyfit(W[np.abs(W) > 0.3], EY[np.abs(W) > 0.3], 1)
    return {'R_global_rot_deg': math.degrees(ang),
            'R_rmse_still_m': float(np.sqrt((np.linalg.norm(p - q, axis=1) ** 2).mean())),
            'R_rmse_still_derot_m': float(np.sqrt((np.linalg.norm(pr - q, axis=1) ** 2).mean())),
            'R_turn_yaw_slope_s': float(k[0])}


rows = []
for light, rec in REC.items():
    truth = m.truth(str(rec))
    start = truth[0, 1:4]
    for kind in ('lidar_only', 'truth_odom'):
        for dest in [OUT / f'carto_{kind}_{light}'] + sorted(OUT.glob(f'carto_{kind}_{light}_V*')):
            variant = dest.name.split(f'_{light}')[1].lstrip('_') or 'c6_r1'
            if not (dest / 'run_summary.json').exists():
                rows.append({'light': light, 'kind': kind, 'variant': variant, 'tool_error': 'run_summary 없음'})
                continue
            run = json.loads((dest / 'run_summary.json').read_text())
            if 'error' in run:
                rows.append({'light': light, 'kind': kind, 'variant': variant, 'tool_error': run['error']})
                continue
            tf = m.read_tf(str(dest / 'out_bag'))
            if ('odom', 'base_link') in tf:
                tf[('odom', 'base_footprint')] = tf[('odom', 'base_link')]
            row = m.pose_metrics(tf, truth, start, 'carto')
            row.pop('traj', None)
            row.update(m.map_metrics(str(dest), start, tree, walls))
            row.update(rotation_metrics(tf, truth, start))
            row.update({'light': light, 'kind': kind, 'variant': variant, 'recording': str(rec), 'truth_as_input': kind == 'truth_odom'})
            row['criteria_reference'] = criteria(row)
            row['pass_all'] = all(row['criteria_reference'].values())
            rows.append(row)
            print(json.dumps({k: row[k] for k in ('light', 'kind', 'variant', 'A1_rmse_m', 'A1_fit_rmse_m', 'A3_yaw_rmse_deg', 'jumps',
                                                  'M1_aligned_mean_err_cm', 'R_global_rot_deg', 'R_rmse_still_derot_m', 'R_turn_yaw_slope_s',
                                                  'pass_all')}, ensure_ascii=False), flush=True)
(OUT / 'summary_variants.json').write_text(json.dumps({'simulation_only': True, 'model': '261029_1030/urdf_261028', 'maze': 'maze.zip',
                                                       'rows': rows}, ensure_ascii=False, indent=2))
