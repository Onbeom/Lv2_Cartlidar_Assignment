"""평가 자료 실패 나누기: 거리 · 보이는 비율 · 태그 화소별. ."""
import json,sys
import numpy as np
J=json.load(open('data/eval/truth.json'));K=np.array(J['K'])
tag={json.loads(x)['name']:json.loads(x) for x in open('results/tag.jsonl')}
cp={json.loads(x)['id']:json.loads(x) for x in open(f'results/{sys.argv[1]}.jsonl')}
def iou(a,b):
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:]);i=np.prod(np.maximum(hi-lo,0));return float(i/((a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-i+1e-9))
R=[]
for im in J['images']:
    boxes=[np.array([*np.clip(np.array(q['corners_px']),[0,0],[1280,720]).min(0),*np.clip(np.array(q['corners_px']),[0,0],[1280,720]).max(0)]) for q in im['pucks']]
    used={}
    for d in sorted([d for d in cp[im['id']]['dets'] if d['valid']],key=lambda d:-d['score']):
        ov=[iou(np.array(d['box']),b) for b in boxes];j=int(np.argmax(ov))
        if ov[j]>=.3 and j not in used: used[j]=d
    tg={q['id']:q for q in tag[im['id']]['pucks']}
    for j,q in enumerate(im['pucks']):
        if q['visible']<.5:continue
        c=np.array(q['center_cam']);z=np.array(q['R_cam'])[:,2];uv=np.array(q['corners_px']);ct=(K@c)[:2]/c[2]
        ok=lambda e,a:e<=.01 and a<=10
        r=dict(id=im['id'],j=j,dist=q['dist_m'],vis=q['visible'],tv=q['tag_visible_max'],edge=min(ct[0],1280-ct[0],ct[1],720-ct[1]),cut=not((uv>=0).all() and (uv[:,0]<=1280).all() and (uv[:,1]<=720).all()),size=float((uv.max(0)-uv.min(0)).max()),shape=q['shape'],lying=q['lying'])
        t=tg.get(q['tag_id']);r['tag_out']=t is not None;r['tag_ok']=False
        if t and t['orientation_valid']:
            e=np.linalg.norm(np.array(t['center'])-c);a=np.degrees(np.arccos(min(1,abs(np.dot(t['z'],z)))));r['tag_ok']=bool(ok(e,a))
        d=used.get(j);r['cp_out']=d is not None;r['cp_ok']=False
        if d:
            e=np.linalg.norm(np.array(d['center'])-c);a=np.degrees(np.arccos(min(1,abs(np.dot(d['z'],z)))));r['cp_ok']=bool(ok(e,a));r['cp_e']=float(e*1e3);r['cp_a']=float(a)
        R.append(r)
def show(name,sel):
    S=[r for r in R if sel(r)]
    if S:print(f"{name:22s} n{len(S):4d} tag출력{np.mean([r['tag_out'] for r in S]):.3f} 태그성공{np.mean([r['tag_ok'] for r in S]):.3f} cp출력{np.mean([r['cp_out'] for r in S]):.3f} cp성공{np.mean([r['cp_ok'] for r in S]):.3f}")
print('거리 m');[show(f'{a}-{b}',lambda r,a=a,b=b:a<=r['dist']<b) for a,b in [(0,.18),(.18,.22),(.22,.26),(.26,.3),(.3,.5)]]
print('긴변 px');[show(f'{a}-{b}',lambda r,a=a,b=b:a<=r['size']<b) for a,b in [(0,70),(70,90),(90,110),(110,140),(140,400)]]
print('보임');[show(f'{a}-{b}',lambda r,a=a,b=b:a<=r['vis']<b) for a,b in [(.5,.7),(.7,.85),(.85,1.01)]]
show('잘림',lambda r:r['cut']);show('안 잘림',lambda r:not r['cut'])
print('태그 보임 비율');[show(f'{a}-{b}',lambda r,a=a,b=b:a<=r['tv']<b) for a,b in [(0,.5),(.5,.9),(.9,1.01)]]
F=[r for r in R if r['cp_out'] and not r['cp_ok']];print('cp 실패',len(F),'미출력',sum(not r['cp_out'] for r in R))
for r in sorted(F,key=lambda r:-r['cp_e'])[:14]:print({k:(round(v,2) if isinstance(v,float) else v) for k,v in r.items() if k in('id','dist','vis','cut','shape','lying','cp_e','cp_a','size','edge')})
print('태그 보임인데 태그 방식 출력 없음',sum(r['tv']>=.9 and not r['tag_out'] for r in R))
print('태그 출력했지만 실패',sum(r['tag_out'] and not r['tag_ok'] for r in R))
