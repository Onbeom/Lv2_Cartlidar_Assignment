"""첫 시도 D435 자료(TAO 형식)를 train_crop 형식으로. 첫 시도와 같은 480장 · 같은 평가 자료."""
import json, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/home/pa4/git/lv2_next_module');BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
HERE=Path(__file__).resolve().parent;OUT=HERE/'data_d435';D=np.diag([1.,-1.,-1.])
K=[[605.850769,0,324.365814],[0,605.67987,245.139175],[0,0,1.]];KA=np.array(K)
C=np.array([[x,y,z] for x in (-.015,.015) for y in (-.015,.015) for z in (-.03,.03)])
S=np.array([[1,0,0],[0,0,1],[0,-1,0.]])  # 내 몸체 → 옛 몸체 (내 z = 옛 y, 내 y = -옛 z)
def conv(cat,files,name):
    rows=[]
    for f in files:
        L=json.loads(Path(f).with_suffix('.json').read_text());P=[]
        for i,o in enumerate(L['objects']):
            R=D@Rotation.from_quat(o['quaternion_xyzw']).as_matrix();c=D@np.array(o['location']);Rm=R@S
            uv=(C@Rm.T+c)@KA.T;uv=uv[:,:2]/uv[:,2,None]
            assert np.linalg.det(Rm)>0
            P.append(dict(tag_id=i,shape=cat,color='x',lying=False,tag6=False,center_cam=c.tolist(),R_cam=Rm.tolist(),visible=o['visibility'],tags_visible={},tag_visible_max=0,corners_px=uv.tolist(),dist_m=float(np.linalg.norm(c))))
        rows.append(dict(id=Path(f).stem,image=str(Path(f).with_suffix('.png')),T_bf_cam=np.eye(4).tolist(),pucks=P))
    d=OUT/f'{cat}_{name}';d.mkdir(parents=True,exist_ok=True);(d/'truth.json').write_text(json.dumps(dict(K=K,width=640,height=480,images=rows,synthetic_only=True)))
    print(d,len(rows),sum(len(r['pucks']) for r in rows))
for cat in ['cylinder','cuboid']:
    m=json.loads((BASE/f'centerpose_train_{cat}/manifest.json').read_text())
    conv(cat,m['train_files'],'train480');conv(cat,m['validation_files'],'val80');conv(cat,sorted((BASE/'centerpose_data'/cat/'eval').glob('*.png')),'eval')
