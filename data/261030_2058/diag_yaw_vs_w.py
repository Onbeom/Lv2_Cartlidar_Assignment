import numpy as np, importlib.util, math
spec=importlib.util.spec_from_file_location('e','/home/pa4/git/lv2_next_module/261027_0953/slam/eval_slam.py'); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
B='/home/pa4/git/lv2_next_module/261030_2058/results/'; R='/home/pa4/git/lv2_next_module/261030_1943/results/'
rec={'bright':R+'n261028_bright_261030_211830','dark':R+'n261028_dark_261030_213001'}
for name in ['carto_truth_odom_dark','carto_lidar_only_dark','carto_lidar_only_bright','carto_wheel_odom_dark']:
    light=name.split('_')[-1]; tf=m.read_tf(B+name+'/out_bag'); mo=tf[('map','odom')]; ob=tf[('odom','base_link')]
    tr=m.truth(rec[light]); st=tr[0,1:4]; yu=np.unwrap(tr[:,3]); w=np.gradient(yu,tr[:,0])
    # stamp 간격
    print(name,'map->odom 주기 중앙',np.median(np.diff(mo[:,0])),'odom->base 주기',np.median(np.diff(ob[:,0])),'수신-stamp 중앙 m2o',np.median(mo[:,4]-mo[:,0]))
    g=np.arange(1,tr[-1,0],0.05); ey=[];ww=[];ep=[]
    for t in g:
        a,b=m.at(mo,t),m.at(ob,t); e=m.compose(a,b); j=min(np.searchsorted(tr[:,0],t),len(tr)-1)
        ey.append(math.degrees(m.wrap(e[2]+st[2]-tr[j,3]))); ww.append(w[j]); ep.append(np.linalg.norm(m.to_world(np.array([e[:2]]),st)[0]-tr[j,1:3]))
    ey,ww,ep=map(np.array,(ey,ww,ep))
    for lab,s in [('|w|<0.05',np.abs(ww)<0.05),('|w|>0.3',np.abs(ww)>0.3)]:
        print(f'  {lab}: n={s.sum()} yaw오차 RMS={np.sqrt((ey[s]**2).mean()):.2f}° 위치 RMS={np.sqrt((ep[s]**2).mean()):.4f}')
    s=np.abs(ww)>0.3; k=np.polyfit(np.radians(ww[s]),np.radians(ey[s]),1); print(f'  회전구간 yaw오차 = {k[0]:.3f} s × w + {math.degrees(k[1]):.2f}°  상관 {np.corrcoef(ww[s],ey[s])[0,1]:.2f}')
