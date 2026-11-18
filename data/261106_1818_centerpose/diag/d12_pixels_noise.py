"""D1 물체 화소 크기, D2 키포인트 잡음 → PnP 오차 (CPU, 모델 없음). 첫 시도 D435 평가 자료 + 팔 카메라 자리."""
import json, math, sys
from pathlib import Path
import cv2, numpy as np
ROOT=Path('/home/pa4/git/lv2_next_module')
BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/centerpose_data'
OUT=Path(__file__).resolve().parent
SIGNS=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)
LOCAL=SIGNS*np.array([.03,.06,.03])/2
K_D=np.array([[605.850769,0,324.365814],[0,605.67987,245.139175],[0,0,1.]])
D=np.diag([1.,-1.,-1.])
rng=np.random.default_rng(0)
def noise(rows,K,sigmas,label):
    out=[]
    for s in sigmas:
        err=[];ang=[];fail=0
        for R,t in rows:
            uv=(LOCAL@R.T+t)@K.T;uv=uv[:,:2]/uv[:,2,None]
            pts=uv+rng.normal(0,s,uv.shape)
            ok,rv,tv=cv2.solvePnP(LOCAL,pts,K,np.zeros(5),flags=cv2.SOLVEPNP_EPNP)
            if not ok: fail+=1;continue
            rv,tv=cv2.solvePnPRefineLM(LOCAL,pts,K,np.zeros(5),rv,tv)
            err.append(np.linalg.norm(tv.ravel()-t))
            Rh=cv2.Rodrigues(rv)[0];ang.append(math.degrees(math.acos(min(1,abs(Rh[:,1]@R[:,1])))))
        err=np.array(err);ang=np.array(ang)
        out.append(dict(input=label,sigma_px=s,n=len(err),center_med_mm=1e3*float(np.median(err)),center_p95_mm=1e3*float(np.percentile(err,95)),
                        axis_med_deg=float(np.median(ang)),axis_p95_deg=float(np.percentile(ang,95)),ok_1cm_10deg=float(np.mean((err<=.01)&(ang<=10)))))
        print(out[-1],flush=True)
    return out
res={'pixels':[],'noise':[]}
SIG=[0.1,0.25,0.5,1,2,4]
for cat in ['cylinder','cuboid']:
    w=[];z=[];rows=[]
    for f in sorted((BASE/cat/'eval').glob('*.json')):
        for o in json.loads(f.read_text())['objects']:
            p=np.array(o['projected_cuboid']);b=p[1:].max(0)-p[1:].min(0);w.append(max(b))
            c=D@np.array(o['location']);z.append(np.linalg.norm(c))
            Rq=np.array(cv2.Rodrigues(np.zeros(3))[0])
            from scipy.spatial.transform import Rotation
            rows.append((D@Rotation.from_quat(o['quaternion_xyzw']).as_matrix(),c))
    w=np.array(w);z=np.array(z)
    res['pixels'].append(dict(input='D435_640x480_'+cat,n=len(w),long_side_px_med=float(np.median(w)),long_side_px_p05=float(np.percentile(w,5)),long_side_px_p95=float(np.percentile(w,95)),
      at_512_px_med=float(np.median(w))*512/640,heatmap_cells_med=float(np.median(w))*512/640/4,dist_m_med=float(np.median(z)),dist_m_p95=float(np.percentile(z,95))))
    print(res['pixels'][-1],flush=True)
    res['noise']+=noise(rows[::3],K_D,SIG,'D435_'+cat)
# 팔 카메라: 면 요 판정 자리(모의, 태그 39 px 관찰 자세)의 카메라 자세 + 서 있는 퍽
J=json.loads((ROOT/'261103_1845_face_yaw/judge3/render_face_yaw.json').read_text());k=J['K']
K_A=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
rows=[];w=[]
for sc in J['scenes']:
    T=np.linalg.inv(np.array(sc['T_bf_cam']));tg=sc['target'];cy=math.cos(tg['yaw']);sy=math.sin(tg['yaw'])
    # 몸체축: x,z 짧은 변 · y 긴 변 (LOCAL 약속). 세계 z = 긴 축
    Rw=np.array([[cy,0,sy],[sy,0,-cy],[0,1,0]])
    Rc=T[:3,:3]@Rw;c=T[:3,:3]@np.array([*tg['xy'],J['box_h']+.03])+T[:3,3];rows.append((Rc,c))
    uv=(LOCAL@Rc.T+c)@K_A.T;uv=uv[:,:2]/uv[:,2,None];b=uv.max(0)-uv.min(0);w.append(max(b))
w=np.array(w)
res['pixels'].append(dict(input='arm_1280x720_observe_cuboid',n=len(w),long_side_px_med=float(np.median(w)),long_side_px_p05=float(np.percentile(w,5)),long_side_px_p95=float(np.percentile(w,95)),
   dist_m_med=float(np.median([np.linalg.norm(c) for _,c in rows]))))
print(res['pixels'][-1],flush=True)
res['noise']+=noise(rows*5,K_A,SIG,'arm_observe')
(OUT/'d12.json').write_text(json.dumps(res,ensure_ascii=False,indent=1))
