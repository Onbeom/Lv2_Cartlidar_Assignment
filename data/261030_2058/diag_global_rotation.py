import numpy as np, importlib.util, math
spec=importlib.util.spec_from_file_location('e','/home/pa4/git/lv2_next_module/261027_0953/slam/eval_slam.py'); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
B='/home/pa4/git/lv2_next_module/261030_2058/results/'; R='/home/pa4/git/lv2_next_module/261030_1943/results/'
rec={'bright':R+'n261028_bright_261030_211830','dark':R+'n261028_dark_261030_213001'}
def fit_angle(p,q,c):
    p=p-c;q=q-c; h=p.T@q; return math.degrees(math.atan2(h[0,1]-h[1,0],h[0,0]+h[1,1]))
for kind in ['truth_odom','lidar_only','wheel_odom']:
  for light in ['bright','dark']:
    tf=m.read_tf(B+f'carto_{kind}_{light}/out_bag'); mo=tf[('map','odom')]; ob=tf[('odom','base_link')]
    tr=m.truth(rec[light]); st=tr[0,1:4]; yu=np.unwrap(tr[:,3]); w=np.gradient(yu,tr[:,0])
    g=np.arange(1,tr[-1,0],0.05); E=[];Q=[];W=[];T=[]
    for t in g:
        e=m.compose(m.at(mo,t),m.at(ob,t)); j=min(np.searchsorted(tr[:,0],t),len(tr)-1)
        E.append(m.to_world(np.array([e[:2]]),st)[0]); Q.append(tr[j,1:3]); W.append(w[j]); T.append(t)
    E,Q,W,T=map(np.array,(E,Q,W,T)); s=np.abs(W)<0.05
    ang=fit_angle(E[s],Q[s],st[:2])   # 시작점 둘레 회전만
    c,sn=math.cos(math.radians(ang)),math.sin(math.radians(ang)); P=E[s]-st[:2]; P=np.stack([c*P[:,0]-sn*P[:,1],sn*P[:,0]+c*P[:,1]],1)+st[:2]
    before=np.sqrt((np.linalg.norm(E[s]-Q[s],axis=1)**2).mean()); after=np.sqrt((np.linalg.norm(P-Q[s],axis=1)**2).mean())
    # 시간별 회전 형성: 구간별 시작점 둘레 각
    seg=[]
    for a,b in [(0,30),(30,100),(100,250),(250,520)]:
        k=s&(T>=a)&(T<b)&(np.linalg.norm(Q-st[:2],axis=1)>0.5)
        seg.append(round(fit_angle(E[k],Q[k],st[:2]),2) if k.sum()>10 else None)
    print(f'{kind:10s} {light:6s} 시작점둘레 회전 {ang:+.3f}° | 위치 RMS(|w|<0.05) {before:.4f} → 회전 뺀 뒤 {after:.4f} | 구간별 회전 0-30/30-100/100-250/250-520 s {seg}')
