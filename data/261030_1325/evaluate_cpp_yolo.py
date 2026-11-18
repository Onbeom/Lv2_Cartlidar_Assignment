"""정적 ONNX→실제 C++ 클래스별 NMS의 CPU 인식 평가. TensorRT는 실행하지 않음."""
import ast
import hashlib
import json
import subprocess
from pathlib import Path
import cv2
import numpy as np
import onnxruntime as ort

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
OUT=BASE/'yolo_cpp_evaluation';OUT.mkdir(exist_ok=False)
NAMES=['red_cylinder','green_cylinder','red_cuboid','green_cuboid']
tree=ast.parse((HERE/'evaluate_yolo.py').read_text())
functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['iou','pairs','summarize']]
exec(compile(ast.Module(body=functions,type_ignores=[]),str(HERE/'evaluate_yolo.py'),'exec'),globals())
options=ort.SessionOptions();options.intra_op_num_threads=2;options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_DISABLE_ALL
model=BASE/'yolo_export/yolo11n_task4_B.onnx'
session=ort.InferenceSession(str(model),sess_options=options,providers=['CPUExecutionProvider'])
rows=[]
for p in sorted((HERE/'B_negative/images/eval').glob('*.png')):
    image=cv2.imread(str(p));h,w=image.shape[:2];s=min(640/w,640/h);nw,nh=int(w*s),int(h*s);px,py=(640-nw)//2,(640-nh)//2
    canvas=np.full((640,640,3),114,np.uint8);canvas[py:py+nh,px:px+nw]=cv2.resize(image,(nw,nh))
    x=np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255
    raw=session.run(None,{session.get_inputs()[0].name:x})[0]
    result=subprocess.run([str(HERE/'yolo_cpp/decode_cli'),str(raw.shape[1]),str(raw.shape[2]),str(w),str(h),str(s),str(px),str(py)],input=raw.tobytes(),capture_output=True,check=True)
    detections=json.loads(result.stdout)
    for d in detections:d['name']=NAMES[d['class']]
    gt=[]
    for line in (HERE/'B_negative/labels/eval'/(p.stem+'.txt')).read_text().splitlines():
        if not line.strip():continue
        c,cx,cy,bw,bh=map(float,line.split());cx*=w;cy*=h;bw*=w;bh*=h
        gt.append({'class':int(c),'box':[cx-bw/2,cy-bh/2,cx+bw/2,cy+bh/2]})
    rows.append({'image':str(p),'gt':gt,'detections':detections})
metrics=summarize(rows)
metrics.update({'source':'ONNX정적640square→실제C++float/classwiseNMS,CPU','tensorrt_verified':False,
                'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
                'cpp_header_sha256':hashlib.sha256((HERE/'yolo_cpp/yolo_postprocess.hpp').read_bytes()).hexdigest()})
(OUT/'summary.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))
(OUT/'rows.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
print(json.dumps(metrics,ensure_ascii=False),flush=True)
