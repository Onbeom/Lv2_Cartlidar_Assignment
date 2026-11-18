"""공식 TAO DLA34 클래스만 불러오는 독립 PyTorch 실험 어댑터.

원본 소스는 변경하지 않는다. FAN/Lightning/NGC CLI는 실행하지 않는다.
네트워크·로스·데이터로더 출처는 tao_source/의 Apache-2.0 NVIDIA 소스다.
"""
import ast
import hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from torch import nn
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'tao_deps'))
from safetensors.torch import load_file
sys.path.insert(0,str(HERE/'tao_source'))
from nvidia_tao_pytorch.cv.centerpose.model.model_utils import ConvGRU,DLAUp,IDAUp,fill_fc_weights,get_dla_base,group_norm
from nvidia_tao_pytorch.cv.centerpose.model.criterion import ObjectPoseLoss
from nvidia_tao_pytorch.cv.centerpose.dataloader.data_feeder import ObjectPoseDataset

def make_model():
    path=HERE/'tao_source/nvidia_tao_pytorch/cv/centerpose/model/centerpose.py'
    tree=ast.parse(path.read_text())
    node=next(v for v in tree.body if isinstance(v,ast.ClassDef) and v.name=='CenterPose_DLA34')
    namespace=dict(globals())
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
    cfg=SimpleNamespace(down_ratio=4,final_kernel=1,last_level=5,head_conv=256,out_channel=0,use_convGRU=False,use_pretrained=False)
    model=namespace['CenterPose_DLA34'](cfg)
    pretrained=load_file(str(HERE/'dla34_imagenet_timm.safetensors'))
    actual=model.base.state_dict()
    weights={k:v for k,v in pretrained.items() if k in actual and actual[k].shape==v.shape}
    matched=sum(v.numel() for v in weights.values());total=sum(v.numel() for v in actual.values())
    assert matched/total>.99,(matched,total)
    model.base.load_state_dict(weights,strict=False)
    metadata={'adapter':'TAO공식DLA34클래스/공식로스/공식데이터로더+별도PyTorch루프,TAO CLI미실행',
              'backbone_matched_elements':matched,'backbone_total_elements':total,
              'missing_backbone_keys':[k for k in actual if k not in weights],
              'pretrained_source':'https://huggingface.co/timm/dla34.in1k',
              'pretrained_sha256':hashlib.sha256((HERE/'dla34_imagenet_timm.safetensors').read_bytes()).hexdigest(),
              'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'model_config':vars(cfg)}
    return model,metadata

def dataset_config(category,symmetry=1):
    return SimpleNamespace(num_classes=1,num_joints=8,max_objs=12,
        mean=[.40789654,.44719302,.47026115],std=[.28863828,.27408164,.27809835],
        _eig_val=[.2141788,.01817699,.00341571],
        _eig_vec=[[-.58752847,-.69563484,.41340352],[-.5832747,.00994535,-.81221408],[-.56089297,.71832671,.41158938]],
        category=category,num_symmetry=symmetry,mse_loss=False,center_3D=False,obj_scale=True,use_absolute_scale=True,
        dense_hp=False,hps_uncertainty=False,reg_bbox=True,reg_offset=True,hm_hp=True,reg_hp_offset=True,
        flip_idx=[[1,5],[3,7],[2,6],[4,8]],no_color_aug=False,not_rand_crop=False,aug_rot=0,
        flip=.5,input_res=512,output_res=128,scale=.4,shift=.1,rotate=0,hm_gauss=2)

def loss_config():
    return SimpleNamespace(num_stacks=1,hm_hp=True,mse_loss=False,dense_hp=False,hps_uncertainty=False,
        reg_bbox=True,wh_weight=.1,obj_scale=True,obj_scale_weight=1.,obj_scale_uncertainty=False,
        use_residual=False,dimension_ref=[.03,.06,.03],reg_offset=True,off_weight=1.,reg_hp_offset=True,
        hm_hp_weight=1.,hm_weight=1.,hp_weight=1.,KL_kps_uncertainty=1.,KL_scale_uncertainty=1.,gpus=[0])

if __name__=='__main__':
    import json
    torch.set_num_threads(4)
    model,metadata=make_model()
    model.eval()
    with torch.no_grad():out=model(torch.zeros(1,3,512,512))
    metadata['output_shapes']={k:list(v.shape) for k,v in out[0].items()}
    target=HERE.parent/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/centerpose_data/model_cpu_check.json'
    target.write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
    print(json.dumps(metadata,ensure_ascii=False))
