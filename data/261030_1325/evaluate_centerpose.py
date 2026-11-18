"""고정 CenterPose last.pt의 합성평가. GT는 추론 후 채점에만 읽는다."""
import hashlib
import json
import time
from pathlib import Path
import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from tao_dla_adapter import make_model,dataset_config
from nvidia_tao_pytorch.cv.centerpose.dataloader.augmentation import get_affine_transform
from nvidia_tao_pytorch.cv.centerpose.model.post_processing import HeatmapDecoder,TransformOutputs,MergeOutput
from nvidia_tao_pytorch.cv.centerpose.model.post_processing_utils import pnp_shell

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=BASE/'centerpose_evaluation';OUT.mkdir(exist_ok=False)
torch.set_num_threads(4);D=np.diag([1.,-1.,-1.]);reports=[]
def overlap(a,b):
    a=np.asarray(a);b=np.asarray(b);wh=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]))
    i=float(wh.prod());return i/(float(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2]))-i+1e-9)
for category in ['cylinder','cuboid']:
    checkpoint=BASE/f'centerpose_train_{category}/last.pt'
    assert (checkpoint.parent/'finished.json').is_file(),'30epoch 미완료 모델 평가 금지'
    config=dataset_config(category);model,_=make_model()
    payload=torch.load(checkpoint,map_location='cpu',weights_only=True);model.load_state_dict(payload['state_dict']);model.cuda().eval()
    torch.cuda.reset_peak_memory_stats()
    decode=HeatmapDecoder(num_select=12).eval();transform=TransformOutputs().eval();merge=MergeOutput(vis_threshold=.3).eval()
    # K는 기존 합성카메라의 공개된 고정 보정값이며 정답 pose와 독립이다.
    K=np.array([[605.850769,0,324.365814],[0,605.67987,245.139175],[0,0,1.]])
    paths=sorted((BASE/'centerpose_data'/category/'eval').glob('*.png'))
    for _ in range(5):
        with torch.inference_mode():model(torch.zeros(1,3,512,512,device='cuda'))
    rows=[];durations=[];total=matched=success=fp=0;errors=[];angles=[]
    for path in paths:
        image=cv2.imread(str(path));h,w=image.shape[:2]
        c=np.array([w/2,h/2],np.float32);s=float(max(h,w))
        affine=get_affine_transform(c,s,0,[512,512])
        pixels=cv2.warpAffine(image,affine,(512,512),flags=cv2.INTER_LINEAR).astype(np.float32)/255.
        inp=((pixels-np.array(config.mean,np.float32))/np.array(config.std,np.float32)).transpose(2,0,1)[None].copy()
        torch.cuda.synchronize();start=time.perf_counter()
        with torch.inference_mode():outputs=model(torch.from_numpy(inp).cuda())
        torch.cuda.synchronize()
        decoded=decode([{k:v.cpu() for k,v in outputs[0].items()}])
        items=merge(transform(decoded,torch.from_numpy(c[None]),torch.tensor([s])))[0]
        predictions=[]
        for item in items:
            points1=np.asarray(item['kps_displacement_mean']).reshape(8,2);points2=np.asarray(item['kps_heatmap_mean']).reshape(8,2)
            points=np.hstack((points1,points2)).reshape(-1,2)
            pose=pnp_shell(points,np.array([.03,.06,.03]),K,opencv_return=True)
            box=np.asarray(item['bbox'],float);box[[0,2]]=np.clip(box[[0,2]],0,w);box[[1,3]]=np.clip(box[[1,3]],0,h)
            pred={'score':float(item['score']),'box':box.tolist(),'pose_valid':pose is not None}
            if pose is not None:
                projected,points3d,location,quaternion=pose
                pred.update({'center':list(map(float,location)),'quaternion_xyzw':list(map(float,quaternion)),
                             'projected_cuboid':np.asarray(projected).tolist()})
            predictions.append(pred)
        torch.cuda.synchronize();durations.append(time.perf_counter()-start)
        # 이 시점부터만 정답 pose를 읽는다.
        gt=json.loads(path.with_suffix('.json').read_text())['objects'];total+=len(gt);used=set()
        for pred in sorted(predictions,key=lambda v:-v['score']):
            choices=[]
            for j,obj in enumerate(gt):
                if j in used:continue
                points=np.array(obj['projected_cuboid']);box=np.r_[points.min(0),points.max(0)]
                box[[0,2]]=np.clip(box[[0,2]],0,w);box[[1,3]]=np.clip(box[[1,3]],0,h)
                choices.append((overlap(pred['box'],box),j))
            best,j=max(choices,default=(0,-1))
            if best<.5 or not pred['pose_valid']:fp+=1;continue
            used.add(j);matched+=1;obj=gt[j]
            error=float(np.linalg.norm(np.array(pred['center'])-D@np.array(obj['location'])))
            estimated=Rotation.from_quat(pred['quaternion_xyzw']).as_matrix()[:,1]
            target=(D@Rotation.from_quat(obj['quaternion_xyzw']).as_matrix())[:,1]
            angle=float(np.degrees(np.arccos(np.clip(abs(estimated@target),0,1))))
            good=error<=.01 and angle<=10;success+=good;errors.append(error);angles.append(angle)
            pred.update({'matched_gt_name':obj['name'],'center_error_m':error,'long_axis_error_deg':angle,'success':bool(good)})
        rows.append({'image':str(path),'gt_objects':len(gt),'predictions':predictions})
    report={'category':category,'images':len(paths),'gt_objects':total,'matched_valid_pose':matched,'fp_or_invalid_pose':fp,
            'success_1cm_axis10deg':success,'success_ratio':success/total if total else None,
            'pass':bool(total and success/total>=.95),
            'center_median_m':float(np.median(errors)) if errors else None,'center_p95_m':float(np.percentile(errors,95)) if errors else None,
            'axis_median_deg':float(np.median(angles)) if angles else None,'axis_p95_deg':float(np.percentile(angles,95)) if angles else None,
            'inference_decode_pnp_p95_ms':float(np.percentile(durations,95)*1000),'peak_allocated_bytes':torch.cuda.max_memory_allocated(),
            'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest()}
    reports.append(report);(OUT/(category+'_rows.json')).write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    (OUT/'summary.json').write_text(json.dumps({'synthetic_only':True,'rows':reports,'limit':'TAO구성요소+독립PyTorch루프·30epoch/480장초기비교. 현행RGB-D와동일입력직접비교/실물/Jetson아님.'},ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False),flush=True)
    del model;torch.cuda.empty_cache()
