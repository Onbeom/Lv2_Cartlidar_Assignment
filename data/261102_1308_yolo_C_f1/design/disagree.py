"""칸마다 팔(태그) 표와 D435 표를 정답과 견줘 누가 틀렸는지 센다(문턱은 등록부와 같음: 팔 0.4, D435 0.6)."""
import json,sys,collections,math
BOX=((1.468,.950),(3.670,3.152),(2.936,5.354),(.734,5.354),(2.202,5.354),(3.670,3.886))
C=collections.Counter(); ex=[]
for rp in sys.argv[1:]:
    runs=[json.loads(l) for l in open(rp+'/b0_runs.jsonl')]; ob=collections.defaultdict(list)
    for l in open(rp+'/survey.observations.jsonl'):
        o=json.loads(l); ob[o['survey_id']].append(o['observation_m'])
    for r in runs:
        cells={}
        for c in r['cells']:
            x,y=BOX[c['box']-1]; cells[(c['box'],c['side'])]=c
        for m in ob[r['survey_id']]:
            best=min(cells.values(),key=lambda c:math.hypot(m['x']-(r['snapshot']['slots'][[ (s['box'],s['side']) for s in r['snapshot']['slots']].index((c['box'],c['side']))]['x_mm']/1000),m['y']-(r['snapshot']['slots'][[ (s['box'],s['side']) for s in r['snapshot']['slots']].index((c['box'],c['side']))]['y_mm']/1000)))
            arm=m['camera']=='arm' and m['id']>=0
            th=.4 if arm else .6
            if m['confidence']<th: continue
            for f in ('color','shape'):
                if m[f] is None: continue
                k=('arm' if arm else m['camera'],f,'ok' if m[f]==best['want'][f] else 'wrong'); C[k]+=1
                if k[2]=='wrong': ex.append((rp.split('/')[-2]+'/'+rp.split('/')[-1],best['name'],k,m[f],round(m['confidence'],3)))
for k in sorted(C): print(k,C[k])
for e in ex[:20]: print(' ',e)
