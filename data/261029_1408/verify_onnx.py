"""고정 영상의 PyTorch/ONNX 원시 출력과 동일 C++형 후처리 결과 비교. 검출 정확도 평가는 아님."""
import json,hashlib
from pathlib import Path
import cv2,numpy as np,torch,onnx,onnxruntime as ort
from ultralytics import YOLO
root=Path(__file__).resolve().parent
out=root.parent/'Cartlidar/sim_render/results/followup_261028_1457/YOLO_변환_261029';out.mkdir(exist_ok=True)
pt=root/'models/ft1b_legacy6.pt';op=root/'models/ft1b_legacy6.onnx'
model=YOLO(str(pt)).model.float().fuse().eval()
graph=onnx.load(str(op));onnx.checker.check_model(graph)
session=ort.InferenceSession(str(op),providers=['CPUExecutionProvider'])
examples=json.loads((out.parent/'yolo_examples_cpu.json').read_text())['images']
def decode(pred):
    cls=pred[4:].argmax(0);score=pred[4:].max(0);boxes=[];scores=[];classes=[]
    for i in np.flatnonzero(score>=.3):
        cx,cy,w,h=pred[:4,i];boxes.append([int(cx-w/2),int(cy-h/2),int(w),int(h)]);scores.append(float(score[i]));classes.append(int(cls[i]))
    keep=cv2.dnn.NMSBoxes(boxes,scores,.3,.45)
    return [{'class':classes[int(k)],'box':boxes[int(k)],'score':scores[int(k)]} for k in np.asarray(keep).ravel()]
def iou(a,b):
    ax,ay,aw,ah=a;bx,by,bw,bh=b;i=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by));return i/(aw*ah+bw*bh-i) if aw*ah+bw*bh>i else 1.
rows=[]
for e in examples:
    path=Path(e['image']);bgr=cv2.imread(str(path));h,w=bgr.shape[:2];scale=min(640/w,640/h);nw,nh=int(w*scale),int(h*scale);px,py=(640-nw)//2,(640-nh)//2
    canvas=np.full((640,640,3),114,np.uint8);canvas[py:py+nh,px:px+nw]=cv2.resize(bgr,(nw,nh))
    x=np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
    with torch.inference_mode():y=model(torch.from_numpy(x))[0].numpy()
    z=session.run(None,{session.get_inputs()[0].name:x})[0]
    a,b=decode(y[0]),decode(z[0]);pairs=[iou(i['box'],j['box']) for i,j in zip(a,b)];classes=[i['class'] for i in a]==[i['class'] for i in b]
    box_error=float(np.max(np.abs(y[:,:4]-z[:,:4])));score_error=float(np.max(np.abs(y[:,4:]-z[:,4:])))
    row={'image':str(path),'shape':list(z.shape),'box_max_abs_px':box_error,'score_max_abs':score_error,'count_pt':len(a),'count_onnx':len(b),'class_order_equal':classes,'min_iou':min(pairs,default=1.),'pass':box_error<=.01 and score_error<=.0001 and len(a)==len(b) and classes and min(pairs,default=1.)>=.999,'pt':a,'onnx':b};rows.append(row)
report={'test':'변환 동등성, 검출 정답률 아님','source_sha256':hashlib.sha256(pt.read_bytes()).hexdigest(),'onnx_sha256':hashlib.sha256(op.read_bytes()).hexdigest(),'input':[(s.name,s.shape,s.type) for s in session.get_inputs()],'output':[(s.name,s.shape,s.type) for s in session.get_outputs()],'metadata':dict((v.key,v.value) for v in graph.metadata_props),'rows':rows,'pass':all(r['pass'] for r in rows),'torch':torch.__version__,'onnx':onnx.__version__,'onnxruntime':ort.__version__}
(out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('metadata','rows')},ensure_ascii=False));print([{k:v for k,v in r.items() if k not in ('pt','onnx','image')} for r in rows])
