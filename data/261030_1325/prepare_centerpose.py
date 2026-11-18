"""기존 합성 렌더의 실제 포즈에서 TAO 형식 3D정답을 만든다. 추론 입력으로 정답 사용 금지."""
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/centerpose_data'
OUT.mkdir(exist_ok=False)
BASE=ROOT/'261026_1414/data'
SIGNS=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
S=np.array([[1,0,0],[0,0,-1],[0,1,0]])  # CenterPose y축=물체원래z축
D=np.diag([1.,-1.,-1.])  # OpenCV→OpenGL 광학좌표
counts=Counter();max_projection=0.;max_pnp_error=0.;checked=0
world_cache={}
for name in ['truth_train_a.json','truth_train_b.json','truth_val_0.json','truth_eval_0.json']:
    data=json.loads((BASE/name).read_text());k=data['K'];W=data['width'];H=data['height']
    K=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
    # 공식 loader는 viewport축을 [height,width]로 적용한뒤 [y,x]로돌려쓴다.
    # 그계약에맞춰 OpenGL→정규화viewport행렬을만들고원래K투영과왕복대조한다.
    P=np.array([[0,-2*k['fy']/H,1-2*k['cy']/H,0],
                [2*k['fx']/W,0,1-2*k['cx']/W,0],[0,0,-1,0],[0,0,-1,0.]])
    for row in data['images']:
        if row['world'] not in world_cache:
            world_cache[row['world']]=json.loads((ROOT/row['world']/'layout.json').read_text())
        layout=world_cache[row['world']]
        objects={p['name']:p for p in layout['pucks']}
        C=np.linalg.inv(np.array(row['T_world_cam']))
        for category in ['cylinder','cuboid']:
            annotations=[]
            for pk in row['pucks']:
                if pk['shape']!=category or not pk['labeled'] or pk['visible']<.5:continue
                obj=objects[pk['name']]
                # 물체 몸체치수는 렌더생성기의 명시값. tag/sticker돌출은몸체box에포함하지않음.
                dims=np.array([.03,.06,.03])
                Rc=C[:3,:3]@np.array(obj['R'])@S
                center=C[:3,:3]@np.array(obj['center'])+C[:3,3]
                local=SIGNS*dims/2
                points_cv=np.vstack((center,local@Rc.T+center))
                if np.any(points_cv[:,2]<=0):continue
                uv=points_cv@K.T;uv=uv[:,:2]/uv[:,2,None]
                points_gl=points_cv@D.T
                q=np.c_[points_gl,np.ones(9)]@P.T;q=q[:,:3]/q[:,3,None]
                viewport=(q+1)/2*[H,W,1.]
                projected=viewport[:,[1,0]]
                error=float(np.max(np.abs(projected-uv)));max_projection=max(max_projection,error)
                ok,rvec,tvec=cv2.solvePnP(local.astype(np.float64),uv[1:].astype(np.float64),K,np.zeros(5),flags=cv2.SOLVEPNP_ITERATIVE)
                assert ok
                max_pnp_error=max(max_pnp_error,float(np.linalg.norm(tvec.ravel()-center)))
                annotations.append({'name':pk['name'],'category':category,'scale':dims.tolist(),
                    'location':(D@center).tolist(),'quaternion_xyzw':Rotation.from_matrix(D@Rc).as_quat().tolist(),
                    'keypoints_3d':points_gl.tolist(),'projected_cuboid':projected.tolist(),
                    'visibility':pk['visible'],'units':'m'})
                checked+=1
            if not annotations:continue
            dest=OUT/category/data['split'];dest.mkdir(parents=True,exist_ok=True)
            src=BASE/row['image'];(dest/src.name).symlink_to(src.resolve())
            label={'camera_data':{'intrinsics':k,'camera_projection_matrix':P.tolist(),'width':W,'height':H},
                   'objects':annotations,'provenance':{'truth':str(BASE/name),'source_world':row['world'],
                       'old_robot_model':data['urdf'],'T_world_cam':row['T_world_cam'],'units':'m'}}
            (dest/(src.stem+'.json')).write_text(json.dumps(label,ensure_ascii=False,indent=2))
            counts[f'{category}_{data["split"]}']+=1
summary={'synthetic_only':True,'counts':dict(counts),'objects':checked,
         'projection_roundtrip_max_px':max_projection,'pnp_oracle_roundtrip_max_center_error_m':max_pnp_error,
         'roundtrip_pass':max_projection<=1e-6 and max_pnp_error<=1e-6,
         'metric_is_model_accuracy':False,'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(OUT/'conversion_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps(summary,ensure_ascii=False),flush=True)
