"""T0 채점 자체 확인 + 대조 재생 채점. 기존 eval_slam.py 를 그대로 불러 쓴다."""
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path('/home/pa4/git/lv2_next_module')
HERE = Path(__file__).resolve().parent
OUT = HERE / 'results_v2'
spec = importlib.util.spec_from_file_location('old_eval', ROOT / '261027_0953/slam/eval_slam.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
REC = {'bright': 'm1711_bright_1_261027_203413', 'dark': 'm1711_dark_1_261027_203822'}
only_t0 = '--t0' in sys.argv


def criteria(row):
    return {'position_rmse_0.05m': row.get('A1_rmse_m', np.inf) <= .05,
            'yaw_rmse_3deg': row.get('A3_yaw_rmse_deg', np.inf) <= 3,
            'aligned_wall_error_3cm': row.get('M1_aligned_mean_err_cm', np.inf) <= 3,
            'aligned_ghost_ratio_0.1': row.get('M1_aligned_ghost_ratio', 1) <= .1,
            'jumps_zero': row.get('jumps', -1) == 0}


rows = []
for light, name in REC.items():
    rec = ROOT / '261027_0953/results' / name
    truth = m.truth(str(rec))
    start = truth[0, 1:4]
    # T0: 정답을 시작 자세 기준 상대값으로 바꿔 추정값처럼 넣고 같은 식으로 채점
    c, s = np.cos(-start[2]), np.sin(-start[2])
    d = truth[:, 1:3] - start[:2]
    rel = np.stack([truth[:, 0], c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1], m.wrap(truth[:, 3] - start[2]),
                    truth[:, 0]], 1)
    zero = np.stack([truth[:, 0]] + [np.zeros(len(truth))] * 3 + [truth[:, 0]], 1)
    t0 = m.pose_metrics({('map', 'odom'): zero, ('odom', 'base_footprint'): rel}, truth, start, 'carto')
    t0 = {k: v for k, v in t0.items() if k != 'traj'}
    t0.update({'light': light, 'kind': 'T0_scorer_self_check', 'pass_expectation': t0['A1_rmse_m'] <= .005 and t0['jumps'] == 0})
    rows.append(t0)
    print(json.dumps(t0, ensure_ascii=False), flush=True)
    if only_t0:
        continue
    meta = json.loads((rec / 'meta.json').read_text())
    walls = m.gt_walls(str(Path(meta['world']) / 'layout.json'))
    tree = cKDTree(walls)
    for kind in ['lidar_only', 'truth_odom']:
        dest = OUT / f'carto_{kind}_{light}'
        if not (dest / 'run_summary.json').exists():
            continue
        run = json.loads((dest / 'run_summary.json').read_text())
        if 'error' in run:
            rows.append({'light': light, 'kind': kind, 'tool_error': run['error']})
            continue
        tf = m.read_tf(str(dest / 'out_bag'))
        row = m.pose_metrics(tf, truth, start, 'carto')
        row.update(m.map_metrics(str(dest), start, tree, walls))
        row.update({'light': light, 'kind': kind, 'truth_as_input': kind == 'truth_odom'})
        row['criteria_reference_only'] = criteria(row)
        (dest / 'accuracy.json').write_text(json.dumps(row, ensure_ascii=False, indent=2))
        rows.append({k: v for k, v in row.items() if k != 'traj'})
        print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
OUT.mkdir(exist_ok=True)
(OUT / ('t0_only.json' if only_t0 else 'control_summary.json')).write_text(
    json.dumps({'simulation_only': True, 'old_model': '261027_1711', 'diagnostic_only': True, 'rows': rows}, ensure_ascii=False, indent=2))
