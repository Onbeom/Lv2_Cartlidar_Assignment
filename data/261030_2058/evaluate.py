"""새 모델 · 새 미로 대조 채점. 261027_0953/slam/eval_slam.py 함수를 그대로 쓰고, 정답 벽은 새 미로 라이다 높이(z 0.0823) 단면,
odom 자식 frame 은 base_link(새 URDF 루트, base_footprint 와 xy 같음).
사용: python evaluate.py <녹화 폴더 밝음> <녹화 폴더 어두움>"""
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path('/home/pa4/git/lv2_next_module')
HERE = Path(__file__).resolve().parent
OUT = HERE / 'results'
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


rows = []
for light, rec in REC.items():
    truth = m.truth(str(rec))
    start = truth[0, 1:4]
    for dest in sorted(OUT.glob(f'carto_*_{light}*')):
        kind = dest.name[len('carto_'):].split(f'_{light}')[0]
        rep = dest.name.split(f'_{light}')[1] or '_r1'
        if not (dest / 'run_summary.json').exists():
            continue
        run = json.loads((dest / 'run_summary.json').read_text())
        if 'error' in run:
            rows.append({'light': light, 'kind': kind, 'rep': rep, 'tool_error': run['error']})
            continue
        tf = m.read_tf(str(dest / 'out_bag'))
        if ('odom', 'base_link') in tf:
            tf[('odom', 'base_footprint')] = tf[('odom', 'base_link')]
        row = m.pose_metrics(tf, truth, start, 'carto')
        row.update(m.map_metrics(str(dest), start, tree, walls))
        row.update({'light': light, 'kind': kind, 'rep': rep, 'recording': str(rec), 'truth_as_input': kind == 'truth_odom'})
        row['criteria_reference'] = criteria(row)
        row['pass_all'] = all(row['criteria_reference'].values())
        (dest / 'accuracy.json').write_text(json.dumps(row, ensure_ascii=False, indent=2))
        rows.append({k: v for k, v in row.items() if k != 'traj'})
        print(json.dumps({k: v for k, v in rows[-1].items() if k not in ('latency_components',)}, ensure_ascii=False), flush=True)
(OUT / 'summary.json').write_text(json.dumps({'simulation_only': True, 'model': '261029_1030/urdf_261028', 'maze': 'maze.zip', 'rows': rows},
                                             ensure_ascii=False, indent=2))
