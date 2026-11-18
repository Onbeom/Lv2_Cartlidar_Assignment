import json,random
import cv2,numpy as np
R=json.load(open('results_real/rows.json'));ROWS=R['final_arm'];size=[]
print('상자 긴 변별 유효율 (final_arm)')
A=[(max(d['box'][2]-d['box'][0],d['box'][3]-d['box'][1]),d['valid'],d['score']) for r in ROWS for d in r['dets']]
for lo,hi in [(0,40),(40,70),(70,110),(110,160),(160,300),(300,2000)]:
    S=[a for a in A if lo<=a[0]<hi];print(lo,hi,len(S),round(np.mean([a[1] for a in S]),3) if S else None,round(np.median([a[2] for a in S]),2) if S else None)
E=[(i,len(r['dets'])) for i,r in enumerate(ROWS) if r['dets']]
Bigs=sorted(((max(d['box'][2]-d['box'][0],d['box'][3]-d['box'][1]),i) for i,r in enumerate(ROWS) for d in r['dets']),reverse=True)
pick=[];seen=set()
for s,i in Bigs:
    if i not in seen and (not pick or min(abs(i-p) for p in pick)>6):pick.append(i);seen.add(i)
    if len(pick)==6:break
J2=R['judge2_tag'][:0]+[r for r in R['judge2_tag'] if r['dets']][::3][:3]
cells=[]
E2=[(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(5,7),(4,6),(6,7)]
for r in [ROWS[i] for i in pick]+J2:
    im=cv2.imread(r['file'])
    for d in r['dets']:
        x1,y1,x2,y2=map(int,d['box']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,255,255),2)
        P=np.array(d['corners']).astype(int)
        for a,b in E2:cv2.line(im,tuple(P[a]),tuple(P[b]),(255,0,255) if d['valid'] else (120,120,120),2)
        cv2.putText(im,f"{d['score']:.2f} {'ok' if d['valid'] else 'NO'} {d.get('reproj',-1):.1f}px",(x1,max(20,y1-6)),0,.7,(255,255,255),2)
    cells.append(cv2.resize(im,(640,360)))
while len(cells)%3:cells.append(np.zeros_like(cells[0]))
cv2.imwrite('../photo/261107_centerpose_실물관찰.png',np.vstack([np.hstack(cells[i:i+3]) for i in range(0,len(cells),3)]))
print('pick',[ROWS[i]['file'].split('/')[-1] for i in pick])
