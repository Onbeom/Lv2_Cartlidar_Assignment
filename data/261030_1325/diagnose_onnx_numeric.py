"""전체 원시 출력 기준 실패를 CPU 실행기 차이로 분리. 기존 결과 보존."""
import json
from pathlib import Path
import cv2
import numpy as np
import torch
import onnxruntime as ort
from ultralytics import YOLO

ROOT=Path('/home/pa4/git/lv2_next_module');BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/yolo_export'
first=json.loads((BASE/'manifest.json').read_text());worst=sorted(first['rows'],key=lambda v:-v['box_max_abs_px'])[:8]
torch.set_num_threads(2);results=[]
for mkldnn in [True,False]:
    torch.backends.mkldnn.enabled=mkldnn
    model=YOLO(str(BASE/'yolo11n_task4_B.pt')).model.float().fuse().eval()
    for optimization in ['all','disabled']:
        options=ort.SessionOptions();options.intra_op_num_threads=2
        options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_ENABLE_ALL if optimization=='all' else ort.GraphOptimizationLevel.ORT_DISABLE_ALL
        session=ort.InferenceSession(str(BASE/'yolo11n_task4_B.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
        for row in worst:
            img=cv2.imread(row['image']);h,w=img.shape[:2];scale=min(640/w,640/h);nw,nh=int(w*scale),int(h*scale);px,py=(640-nw)//2,(640-nh)//2
            canvas=np.full((640,640,3),114,np.uint8);canvas[py:py+nh,px:px+nw]=cv2.resize(img,(nw,nh))
            x=np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255
            with torch.inference_mode():a=model(torch.from_numpy(x))[0].numpy()
            b=session.run(None,{session.get_inputs()[0].name:x})[0]
            diff=np.abs(a[:,:4]-b[:,:4]);index=np.unravel_index(diff.argmax(),diff.shape)
            anchor=index[-1];score=max(float(a[:,4:,anchor].max()),float(b[:,4:,anchor].max()))
            results.append({'image':row['image'],'mkldnn':mkldnn,'ort_optimization':optimization,
                'box_max_abs_px':float(diff.max()),'worst_anchor_score':score})
(BASE/'numeric_diagnosis.json').write_text(json.dumps({'design_diagnosis':True,'rows':results},ensure_ascii=False,indent=2))
print(json.dumps(results,ensure_ascii=False),flush=True)
