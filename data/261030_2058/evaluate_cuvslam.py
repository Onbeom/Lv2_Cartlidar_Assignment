"""cuVSLAM 재생 채점(모의): map→odom · odom→base_link(cuVSLAM 이 냄)로 evaluate_variants.py 와 같은 자세 지표 + 지도 전체 회전.
점유 지도가 없어 벽 지표는 없음. 사용: python evaluate_cuvslam.py <이름표>=<녹화 폴더> ..."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results'
src = (HERE / 'evaluate_variants.py').read_text()
ns = {'__file__': str(HERE / 'evaluate_variants.py')}
exec(compile(src.split('\nrows = []\n')[0].replace("REC = {'bright': Path(sys.argv[1]), 'dark': Path(sys.argv[2])}", ''), 'evaluate_variants.py', 'exec'), ns)
m, rotation_metrics = ns['m'], ns['rotation_metrics']
rows = []
for label, rec in (a.split('=', 1) for a in sys.argv[1:]):
    truth = m.truth(rec)
    start = truth[0, 1:4]
    for dest in sorted(OUT.glob(f'cuvslam_{label}_V*')):
        tf = m.read_tf(str(dest / 'out_bag'))
        if ('odom', 'base_link') in tf:
            tf[('odom', 'base_footprint')] = tf[('odom', 'base_link')]
        row = {'label': label, 'variant': dest.name.split(f'{label}_')[1], 'recording': rec}
        try:
            r = m.pose_metrics(tf, truth, start, 'cuvslam')
            r.pop('traj', None)
            row.update(r)
            if r.get('ok'):
                row.update(rotation_metrics(tf, truth, start))
                row['criteria_reference'] = {'position_rmse_0.05m': row['A1_rmse_m'] <= .05, 'yaw_rmse_3deg': row['A3_yaw_rmse_deg'] <= 3,
                                             'jumps_zero': row['jumps'] == 0}
                row['pass_all'] = all(row['criteria_reference'].values())
        except Exception as error:      # 출력 TF 가 모자라면 도구 오류로 남김
            row['tool_error'] = repr(error)
        rows.append(row)
        print(json.dumps({k: v for k, v in row.items() if k != 'criteria_reference'}, ensure_ascii=False), flush=True)
(OUT / 'summary_cuvslam.json').write_text(json.dumps({'simulation_only': True, 'rows': rows}, ensure_ascii=False, indent=2))
