"""URDF 충돌형상으로 깊이의 로봇 점 제거. 렌더 정답마스크는 평가에만 쓴다."""
import json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from PIL import Image
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'Cartlidar/sim_world'));from robot_geom import Robot
meta_path=root/'Cartlidar/sim_render/results/mission_261028/render_r12_gpu/render_r12.json';meta=json.loads(meta_path.read_text());pose=meta['poses'][0];out=root/'Cartlidar/sim_render/results/followup_261028_1457/nvblox_self_filter';out.mkdir(exist_ok=True)
depth=np.load(meta_path.parent/pose['images']['off']['depth']);K=meta['K'];T=np.array(pose['T_world_cam']);h,w=depth.shape;v,u=np.indices(depth.shape);valid=np.isfinite(depth)&(depth>0);points=np.stack([(u-K['cx'])*depth/K['fx'],(v-K['cy'])*depth/K['fy'],depth],-1);points=points@T[:3,:3].T+T[:3,3];flat=points.reshape(-1,3);keep=valid.ravel();mask=np.zeros(h*w,bool);margin=.005
urdf=Path(meta['urdf']);rb=Robot(urdf,urdf.parent.parent);shapes=rb.world_shapes(meta['arm_q'],np.array(pose['T_world_base_footprint']));t0=time.perf_counter();used=0
for link,V in shapes:
 candidates=np.flatnonzero(keep & ~mask & np.all(flat>=V.min(0)-margin,axis=1)&np.all(flat<=V.max(0)+margin,axis=1))
 if not len(candidates):continue
 hull=ConvexHull(V);eq=hull.equations;inside=np.all(flat[candidates]@eq[:,:3].T+eq[:,3]<=margin,axis=1);mask[candidates[inside]]=True;used+=1
elapsed=time.perf_counter()-t0;mask=mask.reshape(h,w);filtered=depth.copy();filtered[mask]=0;np.save(out/'filtered_depth_m.npy',filtered);np.save(out/'predicted_mask.npy',mask)
# 여기부터만 평가 정답 사용.
truth=np.asarray(Image.open(meta_path.parent/pose['name']/'robot_mask.png'))>0;tp=int((truth&mask).sum());fp=int((~truth&mask).sum());fn=int((truth&~mask).sum());recall=tp/max(1,int(truth.sum()));false_rate=fp/max(1,int((~truth).sum()))
report={'mode':'offline prototype; old model; no realtime claim','source':str(meta_path),'margin_m':margin,'used_shapes':used,'total_shapes':len(shapes),'mask_compute_s':elapsed,'truth_robot_pixels':int(truth.sum()),'removed_pixels':int(mask.sum()),'true_positive':tp,'false_positive':fp,'false_negative':fn,'robot_recall':recall,'non_robot_false_removal_rate':false_rate,'unchanged_elsewhere':bool(np.array_equal(filtered[~mask],depth[~mask],equal_nan=True)),'pass':recall>=.95 and false_rate<=.001}
(out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False),flush=True)
