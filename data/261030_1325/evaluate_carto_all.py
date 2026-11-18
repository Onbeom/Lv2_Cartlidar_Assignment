"""완료한 고정VO Cartographer 비교만 채점. 입력 구성/실패를 지우지 않는다."""
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

ROOT=Path('/home/pa4/git/lv2_next_module')
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
OUT=BASE/'improvement_261030_1325'
spec=importlib.util.spec_from_file_location('old_eval',ROOT/'261027_0953/slam/eval_slam.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
old={'bright':'m1711_bright_1_261027_203413','dark':'m1711_dark_1_261027_203822'}
reports=[]
for light,recname in old.items():
    rec=ROOT/'261027_0953/results'/recname
    truth=m.truth(str(rec));start=truth[0,1:4]
    meta=json.loads((rec/'meta.json').read_text());walls=m.gt_walls(str(Path(meta['world'])/'layout.json'));tree=cKDTree(walls)
    for variant,weight in [('w100000',100000),('w1000',1000),('w0',0),('local1_4',0),('local1_1',0)]:
        dest=OUT/f'carto_{light}_{variant}'
        if not (dest/'run_summary.json').exists():continue
        accuracy=dest/'accuracy.json'
        if accuracy.exists():
            reports.append(json.loads(accuracy.read_text()));continue
        run=json.loads((dest/'run_summary.json').read_text())
        if 'error' in run:
            reports.append({'light':light,'weight':weight,'tool_error':run['error']});continue
        tf=m.read_tf(str(dest/'out_bag'));row=m.pose_metrics(tf,truth,start,'carto')
        row.update(m.map_metrics(str(dest),start,tree,walls))
        row.update({'light':light,'variant':variant,'weight':weight,'source_vo':str(BASE/f'slam_coupled_{light}'/'out_bag'),
                    'fixed_vo_state':m.vo_state(str(BASE/f'slam_coupled_{light}'))})
        row['latency_components']={}
        for edge,series in tf.items():
            if edge not in [('map','odom'),('odom','base_footprint')]:continue
            lag=(series[:,4]-series[:,0])*1000
            row['latency_components']['->'.join(edge)]={'median_ms':float(np.median(lag)),'p95_ms':float(np.percentile(lag,95))}
        row['criteria']={'position_rmse_0.05m':row.get('A1_rmse_m',np.inf)<=.05,
            'yaw_rmse_3deg':row.get('A3_yaw_rmse_deg',np.inf)<=3,
            'aligned_wall_error_3cm':row.get('M1_aligned_mean_err_cm',np.inf)<=3,
            'aligned_ghost_ratio_0.1':row.get('M1_aligned_ghost_ratio',1)<=.1,
            'carto_latency_p95_100ms':row.get('C2_latency_ms_p95',np.inf)<=100,
            'fixed_vo_tracking_lost_zero':row['fixed_vo_state'].get('A4_lost_events',-1)==0,
            'jumps_zero':row.get('jumps',-1)==0}
        row['pass']=all(row['criteria'].values())
        accuracy.write_text(json.dumps(row,ensure_ascii=False,indent=2));reports.append(row)
        print(json.dumps({k:v for k,v in row.items() if k not in ['traj','latency_components']},ensure_ascii=False),flush=True)
(OUT/'carto_comparison_all.json').write_text(json.dumps({'simulation_only':True,'old_model':'261027_1711','reused_design_bags':True,
    'latency_note':'CPU재생Cartographer출력지연. VO추론자체의지연/Jetson/실물아님. 성분별별도기록.', 'rows':reports},ensure_ascii=False,indent=2))
