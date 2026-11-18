"""A1 개인 보기 경로 진단: 기존 차체·카메라 위치/기울기 유지, 목표가 집게 사이로 복귀하도록 광축 방위 조절."""
import sys,json,math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
ROOT=Path('/home/pa4/git/lv2_next_module');sys.path.insert(0,str(ROOT/'Cartlidar/sim_render'))
from arm_pose import ArmRealizer,ARM_J
from r12_poses import Scene
O=ROOT/'Cartlidar/sim_render/results/a1_code_261028';U=ROOT/'261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf';draw=json.loads((ROOT/'Cartlidar/sim_world/data/maze_dice_mission.json').read_text())
plans=json.loads((O/'mission_inout_plan.json').read_text())['plan'];done=[];report=[]
for s in plans:
 if s['kind']!='out':continue
 if len(sys.argv)>1 and s['seed']!=int(sys.argv[1]):continue
 lay=json.loads((Path(s['world'])/'layout.json').read_text());scene=Scene(lay,draw,U,U.parent.parent,shelf_top=.07);ar=ArmRealizer(U,U.parent.parent,scene,lay,gripper=.02);base=np.array(s['T_world_base_footprint_arm']);target=np.array(next(p for p in lay['pucks'] if p['name']==s['target'])['center']);lo,hi=np.array(ar.lim).T
 prev=None;frames=[];fail=None
 for k,f in enumerate(s['arm']):
  original=np.array([f['arm_q'][j] for j in ARM_J]);T0=np.array(f['T_world_cam']);theta=math.acos(np.clip(-T0[2,2],-1,1));best=None
  if f['stage']=='approach' or theta < math.radians(12):
   frames.append(f);prev=original;continue
  for back in (0.,.005,.01,.015,.02,.025,.03,.035,.04,.045,.05):
   pos=T0[:3,3]-back*T0[:3,2]
   def residual(q):
    T=ar.cam(base,q);p=T[:3,:3].T@(target-T[:3,3]);
    return np.r_[T[:3,3]-pos,.1*(T[2,2]+math.cos(theta)),.05*p[0]/max(p[2],.02)]
   for start in ([prev] if prev is not None else [])+[original]:
    r=least_squares(residual,np.clip(start,lo,hi),bounds=(lo,hi),xtol=1e-10,ftol=1e-10,gtol=1e-10,max_nfev=200)
    T=ar.cam(base,r.x);p=T[:3,:3].T@(target-T[:3,3]);ae=abs(math.acos(np.clip(-T[2,2],-1,1))-theta)
    if np.linalg.norm(T[:3,3]-pos)>.001 or ae>math.radians(.5) or p[2]<=.01 or abs(283*p[0]/p[2])>3:continue
    if ar.arm_hits(base,r.x):continue
    delta=np.max(abs(r.x-prev))*15 if prev is not None else 0
    cost=delta+back*10
    if best is None or cost<best[0]:best=(cost,r.x,T,back,delta)
   if best and best[4]<=4.502949:break
  if best is None:fail=k;break
  _,q,T,back,delta=best
  frames.append(dict(f,T_world_cam=T.tolist(),arm_q=ar.q(q),gap_adjust_backoff_m=back,gap_target_x_px=float(283*(T[:3,:3].T@(target-T[:3,3]))[0]/(T[:3,:3].T@(target-T[:3,3]))[2]+320)))
  prev=q
 if fail is None:
  qs=np.array([[f['arm_q'][j] for j in ARM_J] for f in frames]);speed=float(np.max(abs(np.diff(qs,axis=0)))*15);s=dict(s,arm=frames,id=s['id']+'_gap');done.append(s)
 else:speed=None
 rec=dict(seed=s['seed'],frames=len(frames),failed_frame=fail,max_joint_speed=speed);report.append(rec);print(rec,flush=True)
(O/'gap_plan_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n');(O/'mission_gap_plan.json').write_text(json.dumps({'plan':done},indent=1)+'\n')
