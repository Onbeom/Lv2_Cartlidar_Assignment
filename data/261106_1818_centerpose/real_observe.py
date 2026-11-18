"""실물 팔 영상 참고 관찰 (채점 아님). 내부값 미측정 → f 는 가로 113° 가정, 왜곡 보정 없음. 3D 정답 없음.
같은 가정 K 에서 CenterPose 와 태그 방식의 중심 차이(일치도), 재투영 오차, 유효율만 낸다."""
import json, sys, glob
from pathlib import Path
import cv2, numpy as np, torch, subprocess, tempfile
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from train_crop import CORNERS,crop_matrix,make_model,to_tensor
from ultralytics import YOLO
ROOT=HERE.parent;fx=640/np.tan(1.9721/2);K=np.array([[fx,0,640],[0,fx,360],[0,0,1.]])
y=YOLO(str(ROOT/'261104_1618_E3/runs/E3/weights/last.pt'));ck=torch.load(HERE/'runs/d2/best.pt',map_location='cpu',weights_only=False);size=ck['size'];g=size//4
m=make_model();m.load_state_dict(ck['model']);m.cuda().eval()
sets={'final_arm':sorted(glob.glob(str(ROOT/'261105_2047_final_eval/final_arm/images/*.png'))),
 'judge2_tag':[str(ROOT/f'261106_0917_real_judge2/clips/{n}/{k:05d}.png') for n in sorted(p.name for p in (ROOT/'261106_0917_real_judge2/clips').glob('tag0*')) for k in (0,30)],
 'barcode_real_L':[str(p) for d in sorted((ROOT/'261104_1754_tag_barcode_real/clips').glob('L*')) for p in sorted(d.glob('*.png'))[:2]]}
@torch.inference_mode()
def pose(im,box):
    M=crop_matrix(box,size);c=cv2.warpAffine(im,M,(size,size),flags=cv2.INTER_LINEAR,borderValue=(114,114,114));o=m(to_tensor(c)[None].cuda())[0];h=o['hm'][0,0].sigmoid();yy,xx=divmod(int(h.argmax()),g);s=float(h[yy,xx])
    ij=np.array([xx,yy],np.float32);p=(o['hps'][0,:,yy,xx].cpu().numpy().reshape(8,2)+ij)*4;inv=cv2.invertAffineTransform(M);pix=p@inv[:,:2].T+inv[:,2]
    ok,rv,tv=cv2.solvePnP(CORNERS,pix.astype(np.float64),K,np.zeros(5),flags=cv2.SOLVEPNP_EPNP);r=dict(score=s,corners=pix.tolist(),valid=False)
    if ok:
        rv,tv=cv2.solvePnPRefineLM(CORNERS,pix.astype(np.float64),K,np.zeros(5),rv,tv);pr,_=cv2.projectPoints(CORNERS,rv,tv,K,np.zeros(5));e=float(np.linalg.norm(pr.reshape(-1,2)-pix,axis=1).mean());r.update(center=tv.ravel().tolist(),reproj=e,z=cv2.Rodrigues(rv)[0][:,2].tolist(),valid=bool(tv[2,0]>0 and e<10 and s>=.25))
    return r
out={};Rows={}
for name,fs in sets.items():
    fs=[f for f in fs if Path(f).exists()];rows=[]
    with tempfile.NamedTemporaryFile('w',suffix='.txt',delete=False) as L:
        for i,f in enumerate(fs): L.write(f"{i} {f} {fx} {fx} 640 360 1 0 0 0 0 1 0 0 0 0 1 0\n")
    tag={json.loads(x)['name']:json.loads(x) for x in subprocess.run([str(HERE/'tagtool/puck_tool'),L.name],capture_output=True,text=True).stdout.splitlines()}
    n=v=0;rep=[];score=[];diff=[];axd=[]
    for i,f in enumerate(fs):
        im=cv2.imread(f);D=[]
        for b in y.predict(im,imgsz=(384,640),conf=.25,iou=.45,agnostic_nms=True,device=0,verbose=False)[0].boxes:
            box=b.xyxy[0].cpu().tolist();r=pose(im,box);r.update(box=box,cls=int(b.cls),conf=float(b.conf));D.append(r);n+=1
            if r['valid']:v+=1;rep.append(r['reproj']);score.append(r['score'])
        for t in tag[str(i)]['pucks']:
            # 태그 중심이 상자 안인 CenterPose 와 비교 (같은 가정 K, 정답 아님)
            uv=(K@np.array(t['center']));uv=uv[:2]/uv[2]
            C=[r for r in D if r['valid'] and r['box'][0]<=uv[0]<=r['box'][2] and r['box'][1]<=uv[1]<=r['box'][3]]
            if len(C)>=1:
                r=max(C,key=lambda r:r['conf']);diff.append(np.linalg.norm(np.array(r['center'])-t['center'])*1e3)
                if t['orientation_valid']:axd.append(np.degrees(np.arccos(min(1,abs(np.dot(r['z'],t['z']))))))
        rows.append(dict(file=f,dets=D,tags=tag[str(i)]['pucks']))
    q=lambda a,p:round(float(np.percentile(a,p)),2) if len(a) else None
    out[name]=dict(images=len(fs),e3_boxes=n,valid=v,reproj_p50_p95=[q(rep,50),q(rep,95)],score_p50=q(score,50),tag_pucks=sum(len(r['tags']) for r in rows),compared=len(diff),center_diff_mm_p50_p95=[q(diff,50),q(diff,95)],axis_diff_deg_p50_p95=[q(axd,50),q(axd,95)]);Rows[name]=rows;print(name,out[name],flush=True)
(HERE/'results_real').mkdir(exist_ok=True);(HERE/'results_real/summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=1));(HERE/'results_real/rows.json').write_text(json.dumps(Rows))
