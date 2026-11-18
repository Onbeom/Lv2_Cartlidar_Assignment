"""같은 고정 평가입력에 6/4클래스 모델 비교. 실제사진/합성 입력 결과를 분리한다."""
import hashlib
import json
import os
import time
from pathlib import Path
os.environ['YOLO_OFFLINE']='true'
os.environ['MPLCONFIGDIR']='/tmp/lv2_plot_config'
import cv2
import numpy as np
import torch
from ultralytics import YOLO

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
TARGET=OUT/'yolo_evaluation';TARGET.mkdir(exist_ok=False)
NAMES=['red_cylinder','green_cylinder','red_cuboid','green_cuboid']
models={'legacy6':ROOT/'261029_1408/models/ft1b_legacy6.pt',
        'A_positive':OUT/'runs/A_positive/weights/last.pt','B_negative':OUT/'runs/B_negative/weights/last.pt'}

def iou(a,b):
    a=np.asarray(a);b=np.asarray(b)
    wh=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]))
    intersection=float(wh.prod());return intersection/(float(np.prod(a[2:]-a[:2]))+float(np.prod(b[2:]-b[:2]))-intersection+1e-9)

def pairs(dets,gts,threshold=.5,same_class=False):
    used=set();got=[]
    for d in sorted(dets,key=lambda v:-v['score']):
        candidates=[(iou(d['box'],g['box']),j) for j,g in enumerate(gts)
                    if j not in used and (not same_class or d['class']==g['class'])]
        overlap,j=max(candidates,default=(0,-1))
        good=overlap>=threshold
        if good:used.add(j)
        got.append((d,j if good else None))
    return got

def summarize(rows):
    stats=[];aps=[]
    for cid,name in enumerate(NAMES):
        n=sum(g['class']==cid for r in rows for g in r['gt'])
        fixed=[];ap_by_iou=[]
        for threshold in np.linspace(.5,.95,10):
            predictions=[]
            for row in rows:
                detections=[d for d in row['detections'] if d['class']==cid]
                gt=[g for g in row['gt'] if g['class']==cid]
                predictions.extend((d['score'],j is not None) for d,j in pairs(detections,gt,float(threshold),True))
            predictions.sort(reverse=True)
            if not predictions or not n:ap_by_iou.append(None if not n else 0.);continue
            tp=np.cumsum([v[1] for v in predictions]);fp=np.arange(1,len(predictions)+1)-tp
            recall=tp/n;precision=tp/(tp+fp)
            envelope=np.maximum.accumulate(precision[::-1])[::-1]
            ap_by_iou.append(float(np.mean([envelope[recall>=r].max() if np.any(recall>=r) else 0. for r in np.linspace(0,1,101)])))
            if abs(threshold-.5)<1e-8:fixed=[(s,ok) for s,ok in predictions if s>=.25]
        tp=sum(ok for _,ok in fixed);fp=len(fixed)-tp
        stats.append({'class':name,'targets':n,'tp':tp,'fp':fp,'fn':n-tp,
                      'precision':tp/(tp+fp) if tp+fp else None,'recall':tp/n if n else None,
                      'AP50_101point':ap_by_iou[0],
                      'AP50_95_101point':float(np.mean(ap_by_iou)) if n else None})
        if n:aps.append(ap_by_iou)
    unknown=sum(d['class']<0 and d['score']>=.25 for row in rows for d in row['detections'])
    shape=matched=color=0
    for row in rows:
        for d,j in pairs([d for d in row['detections'] if d['score']>=.25],row['gt']):
            if j is None:continue
            matched+=1
            dn=d['name'].split('_',1);gn=NAMES[row['gt'][j]['class']].split('_',1)
            color+=dn[0]==gn[0];shape+=dn[1]==gn[1]
    return {'per_class':stats,'tp':sum(v['tp'] for v in stats),'fp':sum(v['fp'] for v in stats)+unknown,
            'unknown_class_fp':unknown,'images':len(rows),'targets':sum(v['targets'] for v in stats),
            'matched_class_agnostic':matched,'shape_correct':shape,'color_correct':color,
            'conditional_shape_ratio':shape/matched if matched else None,
            'mAP50_101point':float(np.mean([a[0] for a in aps])) if aps else None,
            'mAP50_95_101point':float(np.mean(aps)) if aps else None,
            'note':'직접구현101점보간AP. 공식COCO evaluator나Ultralytics val의완전동등성주장아님'}

reports={}
for name,path in models.items():
    assert path.is_file(),str(path)
    model=YOLO(str(path));torch.cuda.reset_peak_memory_stats()
    dummy=np.zeros((480,640,3),np.uint8)
    for _ in range(5):model.predict(dummy,conf=.001,iou=.45,max_det=300,device=0,verbose=False)
    reports[name]={'checkpoint':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'splits':{}}
    for split in ['eval','sim_eval']:
        rows=[];durations=[]
        for p in sorted((HERE/'B_negative/images'/split).glob('*.png')):
            image=cv2.imread(str(p));h,w=image.shape[:2];gt=[]
            for line in (HERE/'B_negative/labels'/split/(p.stem+'.txt')).read_text().splitlines():
                if not line.strip():continue
                cid,x,y,bw,bh=map(float,line.split());x*=w;y*=h;bw*=w;bh*=h
                gt.append({'class':int(cid),'box':[x-bw/2,y-bh/2,x+bw/2,y+bh/2]})
            torch.cuda.synchronize();start=time.perf_counter()
            result=model.predict(image,imgsz=640,conf=.001,iou=.45,max_det=300,device=0,verbose=False)[0]
            boxes=result.boxes.xyxy.cpu().tolist();classes=result.boxes.cls.cpu().tolist();scores=result.boxes.conf.cpu().tolist()
            torch.cuda.synchronize();durations.append(time.perf_counter()-start)
            detections=[]
            for box,cid,score in zip(boxes,classes,scores):
                cname=model.names[int(cid)]
                detections.append({'class':NAMES.index(cname) if cname in NAMES else -1,'name':cname,'score':score,'box':box})
            rows.append({'image':str(p),'gt':gt,'detections':detections})
        metrics=summarize(rows);metrics['p95_in_memory_predict_ms']=float(np.percentile(durations,95)*1000)
        metrics['peak_allocated_bytes']=torch.cuda.max_memory_allocated()
        reports[name]['splits'][split]=metrics
        (TARGET/f'{name}_{split}_rows.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
        print(name,split,json.dumps(metrics,ensure_ascii=False),flush=True)
    del model;torch.cuda.empty_cache()
a=reports['A_positive']['splits']['eval'];b=reports['B_negative']['splits']['eval']
criteria={'fp_strictly_lower':b['fp']<a['fp'],'class_tp_no_drop':all(y['tp']>=x['tp'] for x,y in zip(a['per_class'],b['per_class'])),
          'conditional_shape_95pct':b['conditional_shape_ratio'] is not None and b['conditional_shape_ratio']>=.95}
(TARGET/'summary.json').write_text(json.dumps({'models':reports,'B_vs_A_criteria':criteria,'B_improvement_candidate':all(criteria.values()),
    'purpose':'PC 제한비교. 최종실물3분할검증/Jetson미실행. 평가결과에따라체크포인트/문턱선택하지않음.'},ensure_ascii=False,indent=2))
