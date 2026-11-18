"""검증 자료만: 헛 출력 내용, 태그 방식 놓침 원인."""
import json
import numpy as np, cv2
J=json.load(open('data/val/truth.json'));K=np.array(J['K'])
cp={json.loads(x)['id']:json.loads(x) for x in open('results_val/d2_e3.jsonl')}
tag={json.loads(x)['name']:json.loads(x) for x in open('results_val/tag.jsonl')}
def iou(a,b):
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:]);i=np.prod(np.maximum(hi-lo,0));return float(i/((a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-i+1e-9))
sc=[];fp=[]
for im in J['images']:
    boxes=[np.array([*np.clip(np.array(q['corners_px']),[0,0],[1280,720]).min(0),*np.clip(np.array(q['corners_px']),[0,0],[1280,720]).max(0)]) for q in im['pucks']];used={}
    for d in sorted([d for d in cp[im['id']]['dets'] if d['valid']],key=lambda d:-d['score']):
        ov=[iou(np.array(d['box']),b) for b in boxes];j=int(np.argmax(ov))
        if ov[j]<.3 or j in used: fp.append(dict(id=im['id'],box=[round(x) for x in d['box']],cls=d['cls'],conf=round(d['conf'],2),score=round(d['score'],2),reproj=round(d['reproj_px'],2),iou=round(ov[j],2),dup=j in used))
        else: used[j]=d;sc.append((d['score'],d['conf'],d['reproj_px']))
print('헛',fp);sc=np.array(sc);print('맞은 것 score/conf/reproj 분위 1 5 50',np.percentile(sc,[1,5,50],axis=0).round(2).tolist())
R=[]
for im in J['images']:
    T=np.array(im['T_bf_cam']);ids={q['id'] for q in tag[im['id']]['pucks']}
    for q in im['pucks']:
        if q['visible']<.5 or q['tag_visible_max']<.9:continue
        Rc=np.array(q['R_cam']);c=np.array(q['center_cam']);F={'top':(0,0,.03),'bottom':(0,0,-.03),'px':(.015,0,0),'nx':(-.015,0,0),'py':(0,.015,0),'ny':(0,-.015,0)}
        best=None
        for f,v in q['tags_visible'].items():
            if v<.9:continue
            p=Rc@np.array(F[f])+c;n=Rc@(np.array(F[f])/np.linalg.norm(F[f]));ang=np.degrees(np.arccos(np.clip(-(n@p)/np.linalg.norm(p),-1,1)));px=K[0,0]*.016/p[2]
            if best is None or px*np.cos(np.radians(ang))>best[0]*np.cos(np.radians(best[1])):best=(px,ang)
        uvc=(K@c)[:2]/c[2]
        R.append(dict(hit=q['tag_id'] in ids,px=best[0],ang=best[1],dome=im['dome'],sun=im['sun'],floor=im['floor'],edge=np.hypot(uvc[0]-640,uvc[1]-360),color=q['color'],shape=q['shape']))
def tb(n,k,bins):
    print(n,[(a,b,sum(1 for r in R if a<=r[k]<b),round(float(np.mean([r['hit'] for r in R if a<=r[k]<b] or [0])),3)) for a,b in bins])
tb('태그 한 변 px(1280 기준)','px',[(0,24,),(24,28),(28,32),(32,36),(36,42),(42,60),(60,400)][0:0]+[(0,24),(24,28),(28,32),(32,36),(36,42),(42,60),(60,400)])
tb('시선-법선 각','ang',[(0,20),(20,30),(30,40),(40,50),(50,60),(60,90)])
tb('돔 밝기','dome',[(250,500),(500,800),(800,1101)])
tb('태양','sun',[(0,1),(1,2),(2,3.1)])
print('색',{c:round(float(np.mean([r['hit'] for r in R if r['color']==c])),3) for c in ('red','green')},'바닥',{f:round(float(np.mean([r['hit'] for r in R if r['floor']==f])),2) for f in range(8)})
