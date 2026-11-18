"""보존된 RGB 렌더에서 JPEG 변형을 만들고 동일 회전 판독기로 비교. 정답은 판독 뒤 채점에만 사용."""
import argparse,json,time
from pathlib import Path
import cv2
from barcode_rotation import read_rotations
ap=argparse.ArgumentParser();ap.add_argument('--render',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--qualities',type=int,nargs='+',default=[0,95,85,70]);a=ap.parse_args()
cv2.setNumThreads(1);a.out.mkdir(parents=True,exist_ok=True)
meta=json.loads((a.render/'render_r3.json').read_text());rows=[]
for q in a.qualities:
    count=wrong=eligible=0;times=[]
    for p in meta['poses']:
        if p['kind']!='arm':continue
        image=cv2.imread(str(a.render/p['images']['color']))
        if image is None:raise RuntimeError(p['images']['color'])
        if q:
            ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,q])
            if not ok:raise RuntimeError('JPEG 인코딩 실패')
            path=a.out/f'jpeg_q{q}'/p['name']/'frame.jpg';path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data.tobytes())
            image=cv2.imdecode(data,cv2.IMREAD_COLOR)
        started=time.monotonic();r=read_rotations(image);elapsed=time.monotonic()-started;times.append(elapsed)
        in_scope=p['label_visible']>=0.95 and p['extra_m']<=0.040001 and p['tilt_deg']<=15
        good=r['text']==p['text'];bad=r['text'] is not None and not good
        eligible+=in_scope;count+=in_scope and good;wrong+=bad
        row={'quality':q,'name':p['name'],'expected':p['text'],'text':r['text'],'symbols':r['symbols'],'eligible':in_scope,'correct':good,'wrong':bad,'seconds':elapsed,'hits':r['hits']};rows.append(row)
        with (a.out/'progress.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
    summary={'quality':q,'eligible':eligible,'correct':count,'wrong':wrong,'pass':eligible>0 and count==eligible and wrong==0,'mean_s':sum(times)/len(times),'max_s':max(times)}
    (a.out/f'summary_q{q}.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
(a.out/'rows.json').write_text(json.dumps(rows,indent=1,ensure_ascii=False))
