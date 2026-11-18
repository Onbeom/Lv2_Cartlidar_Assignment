import sys;sys.path.insert(0,'.')
import numpy as np, auto_label3d as A
from multiprocessing import Pool
with Pool(16) as po:
    for s in ('train3','final'):
        print(s)
        for f in (350,424,500,566,640,720,800,900,1000,1200):
            K=np.array([[f,0,640],[0,f,360],[0,0,1.]]);R=A.run(s,K,None,po);u=A.summary(R)
            T=[r for r in R if r.get('n_tags',0)>0];T2=[r for r in R if r.get('n_tags',0)>1]
            print(f" f {f:5d} 상자 {u['n']} 태그 {u['tag_n']} 둘 {u['two_tags']} 자동 {u.get('auto',0)} | 윤곽 IoU 중앙 {u['sil_med']:.3f} 태그 px 중앙 {u['tag_px_med']:.2f} 상자 IoU {u['box_med']:.3f} | 태그 둘 태그 px 중앙 {np.median([r['tag_px'] for r in T2]) if T2 else float('nan'):.2f} 윤곽 {np.median([r['sil_iou'] for r in T2]) if T2 else float('nan'):.3f}",flush=True)
