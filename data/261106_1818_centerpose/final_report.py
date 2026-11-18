"""설계 3 평가 자료 요약 · 그림. 모의."""
import json
from pathlib import Path
import numpy as np
J=json.load(open('data/eval/truth.json'));K=np.array(J['K'])
tag={json.loads(x)['name']:json.loads(x) for x in open('results/tag.jsonl')};cp={json.loads(x)['id']:json.loads(x) for x in open('results_d3/cp_e3_agnostic.jsonl')}
F={'top':(0,0,.03),'bottom':(0,0,-.03),'px':(.015,0,0),'nx':(-.015,0,0),'py':(0,.015,0),'ny':(0,-.015,0)}
def iou(a,b):
    lo=np.maximum(a[:2],b[:2]);hi=np.minimum(a[2:],b[2:]);i=np.prod(np.maximum(hi-lo,0));return float(i/((a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-i+1e-9))
R=[]
for im in J['images']:
    boxes=[np.array([*np.clip(np.array(q['corners_px']),[0,0],[1280,720]).min(0),*np.clip(np.array(q['corners_px']),[0,0],[1280,720]).max(0)]) for q in im['pucks']];used={}
    for d in sorted([d for d in cp[im['id']]['dets'] if d['valid']],key=lambda d:-d['score']):
        ov=[iou(np.array(d['box']),b) for b in boxes];j=int(np.argmax(ov))
        if ov[j]>=.3 and j not in used:used[j]=d
    tg={q['id']:q for q in tag[im['id']]['pucks']}
    for j,q in enumerate(im['pucks']):
        if q['visible']<.5:continue
        c=np.array(q['center_cam']);z=np.array(q['R_cam'])[:,2];Rc=np.array(q['R_cam']);best=0;ba=None
        for f,v in q['tags_visible'].items():
            if v<.9:continue
            p=Rc@np.array(F[f])+c;n=Rc@(np.array(F[f])/np.linalg.norm(F[f]));a=np.degrees(np.arccos(np.clip(-(n@p)/np.linalg.norm(p),-1,1)));px=K[0,0]*.016/p[2]
            if px>best:best,ba=px,a
        r=dict(dist=q['dist_m'],seen=q['tag_visible_max']>=.9,px=best,ang=ba,cond=('lying_tag_up' if q['tag6'] else 'lying_no_up_tag') if q['lying'] else 'standing_'+q['shape'])
        def es(cc,zz):
            e=float(np.linalg.norm(np.array(cc)-c));a=float(np.degrees(np.arccos(min(1,abs(np.dot(zz,z)/np.linalg.norm(zz))))));return e,a
        t=tg.get(q['tag_id']);r['t']=es(t['center'],t['z']) if t and t['orientation_valid'] else None
        d=used.get(j);r['c']=es(d['center'],d['z']) if d else None
        R.append(r)
ok=lambda v:v is not None and v[0]<=.01 and v[1]<=10
def wilson(k,n,z=1.96):
    if not n:return None
    p=k/n;d=1+z*z/n;c=(p+z*z/2/n)/d;h=z*np.sqrt(p*(1-p)/n+z*z/4/n/n)/d;return [round(c-h,3),round(c+h,3)]
def row(sel):
    S=[r for r in R if sel(r)];T=[r['t'] for r in S if r['t']];C=[r['c'] for r in S if r['c']];kt=sum(ok(r['t']) for r in S);kc=sum(ok(r['c']) for r in S)
    q=lambda V,i:[round(float(np.percentile([v[i]*(1e3 if i==0 else 1) for v in V],p)),2) for p in (50,95)] if V else None
    return dict(n=len(S),tag_out=round(len(T)/max(1,len(S)),3),tag_ok=round(kt/max(1,len(S)),3),tag_ci=wilson(kt,len(S)),tag_c=q(T,0),tag_a=q(T,1),cp_out=round(len(C)/max(1,len(S)),3),cp_ok=round(kc/max(1,len(S)),3),cp_ci=wilson(kc,len(S)),cp_c=q(C,0),cp_a=q(C,1))
O={'전체':row(lambda r:True),'태그 보임':row(lambda r:r['seen']),'태그 안 보임':row(lambda r:not r['seen']),
   '거리 0.22 m 이하':row(lambda r:r['dist']<=.22),'거리 0.22 m 초과':row(lambda r:r['dist']>.22),
   '태그 변 36 px 이상 · 각 40° 미만':row(lambda r:r['seen'] and r['px']>=36 and r['ang'] is not None and r['ang']<40),
   '서 있는 원기둥':row(lambda r:r['cond']=='standing_cylinder'),'서 있는 직육면체':row(lambda r:r['cond']=='standing_cuboid'),'누움, 위 면 태그 있음':row(lambda r:r['cond']=='lying_tag_up'),'누움, 위 면 태그 없음':row(lambda r:r['cond']=='lying_no_up_tag')}
P=[r for r in R if r['t'] and r['c']];O['둘 다 출력한 퍽']=dict(n=len(P),tag_c=[round(float(np.percentile([r['t'][0]*1e3 for r in P],p)),2) for p in (50,95)],cp_c=[round(float(np.percentile([r['c'][0]*1e3 for r in P],p)),2) for p in (50,95)],tag_a=[round(float(np.percentile([r['t'][1] for r in P],p)),2) for p in (50,95)],cp_a=[round(float(np.percentile([r['c'][1] for r in P],p)),2) for p in (50,95)],cp_better_center=round(float(np.mean([r['c'][0]<r['t'][0] for r in P])),3),tag_ok=round(float(np.mean([ok(r['t']) for r in P])),3),cp_ok=round(float(np.mean([ok(r['c']) for r in P])),3))
Path('results_d3/report.json').write_text(json.dumps(O,ensure_ascii=False,indent=1))
for k,v in O.items():print(k,v)
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for n in ('Noto Sans CJK KR','NanumGothic','Noto Sans KR'):
    if any(n==f.name for f in fm.fontManager.ttflist):plt.rcParams['font.family']=n;break
INK='#0b0b0b';SEC='#52514e';GRID='#e4e3df';BG='#fcfcfb';T='#2a78d6';C='#eb6834'
fig,ax=plt.subplots(1,2,figsize=(12.5,5.2),gridspec_kw=dict(width_ratios=[1.15,1]),facecolor=BG)
names=['서 있는 원기둥','서 있는 직육면체','누움, 위 면 태그 있음','누움, 위 면 태그 없음','태그 안 보임'];y=np.arange(len(names));h=.36;a=ax[0]
for off,key,col,lab in ((-h/2-.01,'tag_ok',T,'태그 + estimate_puck (한 장)'),(h/2+.01,'cp_ok',C,'E3 + CenterPose crop 256')):
    v=[O[n][key]*100 for n in names];a.barh(y+off,v,h,color=col,label=lab)
    for yy,x in zip(y+off,v):a.text(x+1,yy,f'{x:.1f}',va='center',fontsize=10,color=INK)
a.set_yticks(y,[f"{n} (n {O[n]['n']})" for n in names],color=INK,fontsize=10.5);a.invert_yaxis();a.set_xlim(0,112);a.set_xlabel('성공률 % (중심 ≤ 10 mm + 긴 축 ≤ 10°)',color=SEC);a.axvline(95,color=SEC,lw=1,ls=':');a.text(95,-.68,'전체 기준 95',ha='center',fontsize=9,color=SEC)
a.legend(loc='lower center',bbox_to_anchor=(.42,-.3),ncol=2,frameon=False,fontsize=10);a.set_title('조건별',loc='left',color=INK,fontsize=12.5)
a=ax[1];bins=[(0,.18),(.18,.22),(.22,.26),(.26,.30),(.30,.5)];lab=['~0.18','0.18~0.22','0.22~0.26','0.26~0.30','0.30~'];X=np.arange(5)
for key,col,mk in (('t',T,'o'),('c',C,'s')):
    V=[100*np.mean([ok(r[key]) for r in R if lo<=r['dist']<hi]) for lo,hi in bins];a.plot(X,V,color=col,lw=2,marker=mk,ms=8,mec=BG,mew=2)
    for x,v in zip(X,V):a.annotate(f'{v:.0f}',(x,v),xytext=(0,9 if key=='c' else -16),textcoords='offset points',ha='center',fontsize=10,color=INK)
a.set_xticks(X,[f"{l}\n(n {sum(lo<=r['dist']<hi for r in R)})" for l,(lo,hi) in zip(lab,bins)],color=SEC,fontsize=9.5);a.set_ylim(0,112);a.set_xlabel('카메라에서 퍽 중심까지 거리 m (관찰 자세 약 0.20)',color=SEC);a.set_ylabel('성공률 %',color=SEC);a.set_title('거리별',loc='left',color=INK,fontsize=12.5)
for a in ax:
    a.set_facecolor(BG);a.spines[['top','right','left']].set_visible(False);a.spines['bottom'].set_color(GRID);a.tick_params(length=0);a.grid(axis='x' if a is ax[0] else 'y',color=GRID,lw=.8);a.set_axisbelow(True)
fig.suptitle('퍽 자세: 모의 합성 평가 500장면, 대상 726개 (실물 아님)',x=.01,ha='left',color=INK,fontsize=14)
fig.tight_layout(rect=(0,0,1,.95));fig.savefig('../photo/261107_centerpose_비교.png',dpi=130,facecolor=BG)
