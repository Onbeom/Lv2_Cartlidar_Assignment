"""범용 채점(모의): evaluate_variants.py 지표 + 지도 도면 맞춤 보정 뒤 위치 RMSE(diag_map_align_correction.py 와 같은 방식).
사용: python evaluate_runs.py <출력 이름.json> <이름표>=<녹화 폴더> ... — results/carto_<kind>_<이름표>[_<복사본>] 폴더를 모두 잰다."""
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results'
spec = importlib.util.spec_from_file_location('ev', HERE / 'evaluate_variants.py')
src = (HERE / 'evaluate_variants.py').read_text()
# evaluate_variants.py 의 함수 · 벽만 가져옴(아래 실행부는 빼고)
ns = {'__file__': str(HERE / 'evaluate_variants.py')}
exec(compile(src.split('\nrows = []\n')[0].replace("REC = {'bright': Path(sys.argv[1]), 'dark': Path(sys.argv[2])}", ''), 'evaluate_variants.py', 'exec'), ns)
m, tree, walls, criteria, rotation_metrics = ns['m'], ns['tree'], ns['walls'], ns['criteria'], ns['rotation_metrics']
KINDS = ['lidar_tfodom_scaled', 'lidar_tfodom_raw', 'wheel_odom_scaled', 'lidar_only', 'truth_odom', 'wheel_odom']
name_out = sys.argv[1]
recs = dict(a.split('=', 1) for a in sys.argv[2:])
rows = []
for label, rec in recs.items():
    truth = m.truth(rec)
    start = truth[0, 1:4]
    for dest in sorted(OUT.glob('carto_*')):
        rest = dest.name[len('carto_'):]
        kind = next((k for k in KINDS if rest.startswith(k + '_')), None)
        if kind is None:
            continue
        tail = rest[len(kind) + 1:]
        if tail != label and not tail.startswith(label + '_'):
            continue
        variant = tail[len(label):].lstrip('_') or 'c6_r1'
        if variant.startswith('r') and variant[1:].isdigit():
            continue                                  # 재생 반복(_r2 · _r3)은 summary.json 에서 봄
        run = json.loads((dest / 'run_summary.json').read_text()) if (dest / 'run_summary.json').exists() else {'error': 'run_summary 없음'}
        if 'error' in run:
            rows.append({'label': label, 'kind': kind, 'variant': variant, 'tool_error': run['error']})
            continue
        tf = m.read_tf(str(dest / 'out_bag'))
        if ('odom', 'base_link') in tf:
            tf[('odom', 'base_footprint')] = tf[('odom', 'base_link')]
        row = m.pose_metrics(tf, truth, start, 'carto')
        traj = row.pop('traj')
        row.update(m.map_metrics(str(dest), start, tree, walls))
        row.update(rotation_metrics(tf, truth, start))
        dx, dy, deg = row['M1_align']
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        p = np.array(traj['est']) - start[:2]
        q = np.stack([c * p[:, 0] - s * p[:, 1] + dx, s * p[:, 0] + c * p[:, 1] + dy], 1) + start[:2]
        e = np.linalg.norm(q - np.array(traj['ref']), axis=1)
        row.update({'A1_rmse_after_map_align_m': float(np.sqrt((e ** 2).mean())), 'A1_max_after_map_align_m': float(e.max()),
                    'label': label, 'kind': kind, 'variant': variant, 'recording': rec, 'truth_as_input': kind == 'truth_odom'})
        row['criteria_reference'] = criteria(row)
        row['pass_all'] = all(row['criteria_reference'].values())
        rows.append(row)
        print(json.dumps({k: row[k] for k in ('label', 'kind', 'variant', 'A1_rmse_m', 'A1_rmse_after_map_align_m', 'A3_yaw_rmse_deg', 'jumps',
                                              'M1_aligned_mean_err_cm', 'R_global_rot_deg', 'pass_all')}, ensure_ascii=False), flush=True)
(OUT / name_out).write_text(json.dumps({'simulation_only': True, 'rows': rows}, ensure_ascii=False, indent=2))
