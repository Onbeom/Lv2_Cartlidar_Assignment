"""개인 PC 후보 B의 FP32 ONNX 변환 및 CPU 동등성. Jetson/TensorRT 미실행."""
import hashlib
import json
import os
import shutil
from pathlib import Path
os.environ['YOLO_OFFLINE']='true'
import cv2
import numpy as np
import onnx
import onnxruntime as ort
import torch
from ultralytics import YOLO

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=BASE/'yolo_export';OUT.mkdir(exist_ok=False)
pt=OUT/'yolo11n_task4_B.pt';shutil.copy2(BASE/'runs/B_negative/weights/last.pt',pt)
torch.set_num_threads(2)
YOLO(str(pt)).export(format='onnx',imgsz=640,batch=1,dynamic=False,simplify=False,opset=17,nms=False,device='cpu',half=False)
op=pt.with_suffix('.onnx');graph=onnx.load(str(op));onnx.checker.check_model(graph)
model=YOLO(str(pt)).model.float().fuse().eval()
options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
session=ort.InferenceSession(str(op),sess_options=options,providers=['CPUExecutionProvider'])
assert session.get_outputs()[0].shape==[1,8,8400]
def decode(pred):
    cls=pred[4:].argmax(0);score=pred[4:].max(0);boxes=[];scores=[];classes=[]
    for i in np.flatnonzero(score>=.25):
        cx,cy,w,h=pred[:4,i];boxes.append([int(cx-w/2),int(cy-h/2),int(w),int(h)]);scores.append(float(score[i]));classes.append(int(cls[i]))
    keep=np.asarray(cv2.dnn.NMSBoxes(boxes,scores,.25,.45)).ravel()
    return [{'class':classes[int(k)],'box':boxes[int(k)],'score':scores[int(k)]} for k in keep]
def iou(a,b):
    ax,ay,aw,ah=a;bx,by,bw,bh=b
    inter=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
    union=aw*ah+bw*bh-inter
    return inter/union if union else 1.
rows=[]
for path in sorted((HERE/'B_negative/images/eval').glob('*.png')):
    bgr=cv2.imread(str(path));h,w=bgr.shape[:2];scale=min(640/w,640/h);nw,nh=int(w*scale),int(h*scale);px,py=(640-nw)//2,(640-nh)//2
    canvas=np.full((640,640,3),114,np.uint8);canvas[py:py+nh,px:px+nw]=cv2.resize(bgr,(nw,nh))
    x=np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
    with torch.inference_mode():y=model(torch.from_numpy(x))[0].numpy()
    z=session.run(None,{session.get_inputs()[0].name:x})[0]
    a,b=decode(y[0]),decode(z[0]);same=[v['class'] for v in a]==[v['class'] for v in b]
    overlap=min((iou(v['box'],w['box']) for v,w in zip(a,b)),default=1.)
    box=float(np.abs(y[:,:4]-z[:,:4]).max());score=float(np.abs(y[:,4:]-z[:,4:]).max())
    rows.append({'image':str(path),'box_max_abs_px':box,'score_max_abs':score,'count_pt':len(a),'count_onnx':len(b),
                 'same_class_order':same,'min_iou':overlap,'pass':box<=.01 and score<=.0001 and len(a)==len(b) and same and overlap>=.999})
summary={'purpose':'FP32변환동등성,인식정답률/Jetson/TensorRT아님','input':[(s.name,s.shape,s.type) for s in session.get_inputs()],
         'output':[(s.name,s.shape,s.type) for s in session.get_outputs()],
         'classes':['red_cylinder','green_cylinder','red_cuboid','green_cuboid'],
         'checkpoint_sha256':hashlib.sha256(pt.read_bytes()).hexdigest(),'onnx_sha256':hashlib.sha256(op.read_bytes()).hexdigest(),
         'rows':rows,'pass':all(r['pass'] for r in rows),'onnx_version':onnx.__version__,'onnxruntime_version':ort.__version__,
         'nms_note':'같은C++식정수상자/클래스전체NMS대조. Ultralytics기본클래스별NMS전체동등성은별도.'}
(OUT/'manifest.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps({'images':len(rows),'pass':summary['pass'],'failed':[r for r in rows if not r['pass']],
                  'max_box_px':max(r['box_max_abs_px'] for r in rows),'max_score_abs':max(r['score_max_abs'] for r in rows)},ensure_ascii=False),flush=True)
