"""공식 TAO 데이터로더로 실제 생성자료를 읽는 CPU 도구 확인."""
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'tao_source'))
from nvidia_tao_pytorch.cv.centerpose.dataloader.data_feeder import ObjectPoseDataset
ROOT=HERE.parent;OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/centerpose_data'
config=dict(num_classes=1,num_joints=8,max_objs=12,
 mean=[.40789654,.44719302,.47026115],std=[.28863828,.27408164,.27809835],
 _eig_val=[.2141788,.01817699,.00341571],
 _eig_vec=[[-.58752847,-.69563484,.41340352],[-.5832747,.00994535,-.81221408],[-.56089297,.71832671,.41158938]],
 num_symmetry=1,mse_loss=False,center_3D=False,obj_scale=True,use_absolute_scale=True,
 dense_hp=False,hps_uncertainty=False,reg_bbox=True,reg_offset=True,hm_hp=True,reg_hp_offset=True,
 flip_idx=[[1,5],[3,7],[2,6],[4,8]],no_color_aug=False,not_rand_crop=False,aug_rot=0,
 flip=.5,input_res=512,output_res=128,scale=.4,shift=.1,rotate=0,hm_gauss=2)
rows=[]
for category in ['cylinder','cuboid']:
    for split in ['train','val','eval']:
        # split val를 쓰는 도구확인에서는 무작위crop을 끄고 모든정답을 유지한다.
        dataset=ObjectPoseDataset(str(OUT/category/split),SimpleNamespace(**config,category=category),'val')
        indices=[0,len(dataset)//2,len(dataset)-1]
        for i in indices:
            sample=dataset[i]
            arrays={k:v for k,v in sample.items() if isinstance(v,np.ndarray)}
            finite=all(np.isfinite(v).all() for v in arrays.values())
            objects=int(sample['reg_mask'].sum())
            rows.append({'category':category,'split':split,'index':i,'dataset_images':len(dataset),
                         'input_shape':list(sample['input'].shape),'hm_shape':list(sample['hm'].shape),
                         'objects':objects,'all_finite':bool(finite),'pass':bool(finite and objects>0)})
summary={'official_source_sha':'99403b7dd941d1276e4e23ed3efd6794e207a75d','rows':rows,
         'pass':all(r['pass'] for r in rows),'sampled_only':True,'model_trained':False,
         'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(OUT/'loader_check.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False),flush=True)
