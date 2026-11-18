"""지도 도면 맞춤 보정(모의, 진단): 저장 지도를 미로 도면(발제 maze.zip 벽 단면)에 맞춘 변환 M1_align(dx, dy, dθ, 시작점 둘레)을
궤적에 그대로 적용했을 때 위치 RMSE. 정답 궤적은 채점에만 쓰고 보정값은 지도 · 도면에서만 구함.
사용: python diag_map_align_correction.py <녹화 폴더 밝음> <녹화 폴더 어두움>"""
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path('/home/pa4/git/lv2_next_module')
OUT = Path(__file__).resolve().parent / 'results'
spec = importlib.util.spec_from_file_location('old_eval', ROOT / '261027_0953/slam/eval_slam.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
REC = {'bright': Path(sys.argv[1]), 'dark': Path(sys.argv[2])}
rows = json.load(open(OUT / 'summary_variants.json'))['rows']
res = []
for r in rows:
    if 'tool_error' in r:
        continue
    tr = m.truth(str(REC[r['light']]))
    start = tr[0, 1:4]
    name = f"carto_{r['kind']}_{r['light']}" + ('' if r['variant'] == 'c6_r1' else '_' + r['variant'])
    tf = m.read_tf(str(OUT / name / 'out_bag'))
    if ('odom', 'base_link') in tf:
        tf[('odom', 'base_footprint')] = tf[('odom', 'base_link')]
    pm = m.pose_metrics(tf, tr, start, 'carto')
    est, ref = np.array(pm['traj']['est']), np.array(pm['traj']['ref'])
    dx, dy, deg = r['M1_align']
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    p = est - start[:2]
    q = np.stack([c * p[:, 0] - s * p[:, 1] + dx, s * p[:, 0] + c * p[:, 1] + dy], 1) + start[:2]
    e = np.linalg.norm(q - ref, axis=1)
    res.append({'kind': r['kind'], 'light': r['light'], 'variant': r['variant'], 'A1_rmse_m': r['A1_rmse_m'], 'map_align': r['M1_align'],
                'A1_rmse_after_map_align_m': float(np.sqrt((e ** 2).mean())), 'A1_max_after_map_align_m': float(e.max())})
    print(json.dumps(res[-1], ensure_ascii=False), flush=True)
(OUT / 'diag_map_align_correction.json').write_text(json.dumps({'simulation_only': True, 'rows': res}, ensure_ascii=False, indent=1))
