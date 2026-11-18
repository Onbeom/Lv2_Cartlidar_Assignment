"""기존모의bag의결합/기준결과비교. 사전기록기준을바꾸지않는다."""
import importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('old_eval',root/'261027_0953/slam/eval_slam.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
base=root/'Cartlidar/sim_render/results/followup_261028_1457';reports=[]
for light,folder in [('bright','m1711_bright_1_261027_203413'),('dark','m1711_dark_1_261027_203822')]:
 rec=root/'261027_0953/results'/folder;out=base/('slam_coupled_'+light);tr=m.truth(str(rec));start=tr[0,1:4];meta=json.loads((rec/'meta.json').read_text());walls=m.gt_walls(str(Path(meta['world'])/'layout.json'));tree=cKDTree(walls)
 results={}
 for name,path,kind in [('coupled',out,'cuvslam'),('baseline_carto',next(rec.glob('carto_*')),'carto')]:
  tf=m.read_tf(str(path/'out_bag'));row=m.pose_metrics(tf,tr,start,kind);row.update(m.map_metrics(str(path),start,tree,walls));row['latency_components']={}
  for edge,series in tf.items():
   lag=(series[:,4]-series[:,0])*1000;row['latency_components']['->'.join(edge)]={'median_ms':float(np.median(lag)),'p95_ms':float(np.percentile(lag,95))}
  if name=='coupled':
   row.update(m.vo_state(str(path)));row['criteria']={'position_rmse_0.05m':row['A1_rmse_m']<=.05,'yaw_rmse_3deg':row['A3_yaw_rmse_deg']<=3.,'aligned_wall_error_3cm':row.get('M1_aligned_mean_err_cm',float('inf'))<=3.,'aligned_ghost_ratio_0.1':row.get('M1_aligned_ghost_ratio',1.)<=.1,'latency_p95_100ms':row['C2_latency_ms_p95']<=100.+1e-6,'tracking_lost_zero':row['A4_lost_events']==0,'jumps_zero':row['jumps']==0};row['pass']=all(row['criteria'].values())
  results[name]=row
 (out/'accuracy_compare.json').write_text(json.dumps(results,ensure_ascii=False,indent=2));reports.append({'light':light,**results})
(base/'slam_coupled_comparison.json').write_text(json.dumps({'simulation_only':True,'old_model':'261027_1711','old_world':'b_dice seed1','independent_repetitions':False,'latency_note':'결합 합계는 VO와 Carto, 기준 합계는 Carto만 포함하므로 성분별 지연을 참고','rows':reports},ensure_ascii=False,indent=2))
print(json.dumps([{k:({a:b for a,b in v.items() if a not in ('traj','M1_align','latency_components')} if isinstance(v,dict) else v) for k,v in r.items()} for r in reports],ensure_ascii=False,indent=2))
