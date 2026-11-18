"""18시야 회귀 및 FK/전송 포함 GPU wall 시간. 정답은 출력 계산 뒤 채점에서만 읽음."""
import hashlib
import json
import sys
import time
from pathlib import Path
import numpy as np
import torch
from gpu_filter import filter_depth
from gpu_geometry import load_local,prepare

ROOT=Path('/home/pa4/git/lv2_next_module')
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'))
from robot_geom import Robot
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
OUT=BASE/'improvement_261030_1325/filter_gpu_v1';OUT.mkdir(exist_ok=False)
manifest=json.loads((OUT.parent/'filter_inputs/manifest.json').read_text())
start=time.perf_counter();urdf=Path(manifest['urdf']);robot=Robot(urdf,urdf.parent.parent);local=load_local(robot)
load_s=time.perf_counter()-start
results=[]
for entry in manifest['rows']:
    raw=np.load(entry['input']);depth=raw['depth']
    meta=json.loads((BASE/entry['source']/'summary.json').read_text())
    row=next(r for r in meta['rows'] if r['visible'] and r['frame']==entry['frame'])
    start=time.perf_counter();geometry=prepare(robot,local,entry['arm_q'],entry['frame'],row['K'])
    mask=filter_depth(depth,*geometry);torch.cuda.synchronize();cold_s=time.perf_counter()-start
    times=[]
    for _ in range(20):
        torch.cuda.synchronize();start=time.perf_counter()
        geometry=prepare(robot,local,entry['arm_q'],entry['frame'],row['K'])
        mask=filter_depth(depth,*geometry);torch.cuda.synchronize();times.append(time.perf_counter()-start)
    reference=Path(entry['reference'])
    ref=np.load(reference/(entry['frame']+'_mask.npy'));truth=np.load(reference/(entry['frame']+'_truth.npy'))
    fp=int((mask&~truth).sum());fn=int((truth&~mask).sum());recall=float((truth&mask).sum()/truth.sum()) if truth.any() else None
    filtered=depth.copy();filtered[mask]=0
    same=bool(np.array_equal(mask,ref));unchanged=bool(np.array_equal(filtered[~mask],depth[~mask],equal_nan=True))
    p95=float(np.percentile(times,95));quality=same and unchanged and (recall is None or recall>=.95) and fp/max(1,int((~truth).sum()))<=.001
    result={'source':entry['source'],'frame':entry['frame'],'cold_s':cold_s,'full_fk_transfer_apply_p95_s':p95,
            'full_fk_transfer_apply_median_s':float(np.median(times)),'reference_identical':same,'changed_mask_pixels':int((mask!=ref).sum()),
            'fp':fp,'fn':fn,'recall':recall,'unchanged_elsewhere':unchanged,'quality_pass':quality,'realtime_30hz':p95<=1/30}
    results.append(result);np.save(OUT/(entry['source']+'__'+entry['frame']+'_mask.npy'),mask)
    print(json.dumps(result,ensure_ascii=False),flush=True)
summary={'simulation_only':True,'device':torch.cuda.get_device_name(),'torch_version':torch.__version__,'load_local_hulls_s':load_s,
         'rows':results,'all_quality_pass':all(r['quality_pass'] for r in results),'all_30hz_pass':all(r['realtime_30hz'] for r in results),
         'source_sha256':{n:hashlib.sha256((Path(__file__).parent/n).read_bytes()).hexdigest() for n in ['gpu_filter.py','gpu_geometry.py','check_gpu_filter.py']},
         'limitations':'기존18시야회귀·같은3정지자세반복. 매회FK/형상변환·CPU/GPU전송포함. ROS수신/시각TF대기/nvblox/실물/Jetson미포함.'}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
