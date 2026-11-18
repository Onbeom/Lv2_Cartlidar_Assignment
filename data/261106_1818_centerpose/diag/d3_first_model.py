"""D3 첫 시도 last.pt 의 2D 키포인트 오차 (평가 · 학습 일부), D4 loss 항목별 값(train/val phase). GPU, 짧음."""
import json, sys, time
from pathlib import Path
import cv2, numpy as np, torch
from scipy.spatial.transform import Rotation
ROOT=Path('/home/pa4/git/lv2_next_module');HERE=ROOT/'261030_1325';sys.path.insert(0,str(HERE))
from tao_dla_adapter import make_model,dataset_config,loss_config,ObjectPoseDataset,ObjectPoseLoss
from nvidia_tao_pytorch.cv.centerpose.dataloader.augmentation import get_affine_transform
from nvidia_tao_pytorch.cv.centerpose.model.post_processing import HeatmapDecoder,TransformOutputs,MergeOutput
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=Path(__file__).resolve().parent;D=np.diag([1.,-1.,-1.]);torch.set_num_threads(4)
K=np.array([[605.850769,0,324.365814],[0,605.67987,245.139175],[0,0,1.]])
SIGNS=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float);LOCAL=SIGNS*np.array([.03,.06,.03])/2
def gt_variants(o,n):
    R=D@Rotation.from_quat(o['quaternion_xyzw']).as_matrix();t=D@np.array(o['location']);V=[]
    for k in range(n):
        a=2*np.pi*k/n;Ry=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
        uv=(LOCAL@(R@Ry).T+t)@K.T;V.append(uv[:,:2]/uv[:,2,None])
    return np.array(V),np.array(o['projected_cuboid'])
def run(model,config,paths,n):
    dec=HeatmapDecoder(num_select=12).eval();tr=TransformOutputs().eval();mg=MergeOutput(vis_threshold=.1).eval()
    rows=[]
    for path in paths:
        im=cv2.imread(str(path));h,w=im.shape[:2];c=np.array([w/2,h/2],np.float32);s=float(max(h,w))
        A=get_affine_transform(c,s,0,[512,512]);px=cv2.warpAffine(im,A,(512,512),flags=cv2.INTER_LINEAR).astype(np.float32)/255.
        inp=((px-np.array(config.mean,np.float32))/np.array(config.std,np.float32)).transpose(2,0,1)[None].copy()
        with torch.inference_mode():o=model(torch.from_numpy(inp).cuda())
        items=mg(tr(dec([{k:v.cpu() for k,v in o[0].items()}]),torch.from_numpy(c[None]),torch.tensor([s])))[0]
        P=[]
        for it in items:
            P.append(dict(score=float(it['score']),ct=np.asarray(it['ct'],float) if 'ct' in it else None,box=np.asarray(it['bbox'],float),
                          kd=np.asarray(it['kps_displacement_mean']).reshape(8,2),kh=np.asarray(it['kps_heatmap_mean']).reshape(8,2)))
        for g in json.loads(path.with_suffix('.json').read_text())['objects']:
            V,pc=gt_variants(g,n);gc=pc[0];size=(pc[1:].max(0)-pc[1:].min(0)).max()
            if not P:rows.append(dict(size=float(size),det=False));continue
            d=[np.linalg.norm((p['box'][:2]+p['box'][2:])/2-gc) for p in P];i=int(np.argmin(d));p=P[i]
            rk=min(np.sqrt(((p['kd']-v)**2).sum(1).mean()) for v in V);rh=min(np.sqrt(((p['kh']-v)**2).sum(1).mean()) for v in V)
            rows.append(dict(size=float(size),det=bool(d[i]<size),center_px=float(d[i]),disp_rms_px=float(rk),heat_rms_px=float(rh),score=p['score']))
    return rows
def summ(rows):
    D_=[r for r in rows if r.get('det')];f=lambda k,q:float(np.percentile([r[k] for r in D_],q)) if D_ else None
    return dict(n=len(rows),near_detected=len(D_),size_med=float(np.median([r['size'] for r in rows])),center_px_med=f('center_px',50),disp_rms_med=f('disp_rms_px',50),disp_rms_p95=f('disp_rms_px',95),heat_rms_med=f('heat_rms_px',50))
res={}
for cat,n in [('cylinder',12),('cuboid',4)]:
    cfg=dataset_config(cat,n);model,_=make_model();ck=torch.load(BASE/f'centerpose_train_{cat}/last.pt',map_location='cpu',weights_only=True);model.load_state_dict(ck['state_dict']);model.cuda().eval()
    ev=sorted((BASE/'centerpose_data'/cat/'eval').glob('*.png'))
    man=json.loads((BASE/f'centerpose_train_{cat}/manifest.json').read_text());tr=[Path(p) for p in man['train_files'][:120]]
    res[cat]={'eval':summ(run(model,cfg,ev,n)),'train120':summ(run(model,cfg,tr,n))}
    # D4 loss 항목: 같은 val 80장을 phase train / val 두 방식으로 계산
    val=ObjectPoseDataset(str(BASE/'centerpose_data'/cat/'val'),cfg,'val');crit=ObjectPoseLoss(loss_config()).cuda()
    dl=torch.utils.data.DataLoader(torch.utils.data.Subset(val,man['val_indices']),batch_size=2,num_workers=0)
    T={}
    for ph in ['train','val']:
        S=[]
        for b in dl:
            b={k:v.cuda() for k,v in b.items() if isinstance(v,torch.Tensor)}
            with torch.inference_mode():
                l=crit(model(b['input']),b,ph)
            if isinstance(l,tuple):l=l[0]
            S.append(float(l))
        T[ph]=float(np.mean(S))
    res[cat]['same_val80_loss_by_phase']=T
    print(cat,json.dumps(res[cat],ensure_ascii=False),flush=True)
    del model,crit;torch.cuda.empty_cache()
(OUT/'d3.json').write_text(json.dumps(res,ensure_ascii=False,indent=1))
