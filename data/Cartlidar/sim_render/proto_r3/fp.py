import json, numpy as np, cv2, sys, collections
sys.path.insert(0,'.')
from locate_proto import locate
from pathlib import Path
root=Path(sys.argv[1])
rep=json.loads((root/'render_r3.json').read_text()); K=rep['d435']['K']
fp=[]
for p in rep['poses']:
    if p['kind']!='d435': continue
    bgr=cv2.imread(str(root/p['images']['color'])); dep=np.load(root/p['images']['depth'])
    T=np.array(p['T_world_cam']); tr=(np.linalg.inv(T)@np.append(p['label_center'],1))[:3]
    for f in locate(bgr,dep,K,T_base_cam=T):
        if np.linalg.norm(f['center']-tr)>0.05:
            w=T@np.append(f['center'],1)
            fp.append((p['name'],f['kind'],[round(x) for x in f['px']],round(f['L']*1000),round(f['S']*1000),round(f['rms']*1000,1),round(f['stripe'],2),[round(x,3) for x in w[:3]]))
print('false',len(fp))
for x in fp[:30]: print(x)
