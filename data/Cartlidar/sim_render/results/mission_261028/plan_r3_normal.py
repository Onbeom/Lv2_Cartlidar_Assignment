"""수평 벽 라벨 도구 확인: B0의 벽 정면 차체 격자와 손목 π 해를 실제 발제 월드 장애물에서 검색."""
import sys,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
TOP=ROOT.parents[3]
sys.path.insert(0,str(TOP/'Cartlidar/sim_render'));sys.path.insert(0,str(TOP/'Cartlidar/sim_world'))
from arm_pose import ArmRealizer,ARM_J
from r12_poses import Scene
from urdf_chain import planar_pose
import code128
U=TOP/'261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf'
drawing=json.loads((TOP/'Cartlidar/sim_world/data/maze_dice_mission.json').read_text())
old=json.loads((ROOT/'r3_final_plan.json').read_text())
miss={r['label'] for r in old['unreachable']}
plan=[];unreachable=[];diagnostics=[]
for lb in old['labels']:
 if lb['id'] not in miss:continue
 w=Path(lb['world']);w=w if w.is_absolute() else TOP/w
 lay=json.loads((w/'layout.json').read_text());sc=Scene(lay,drawing,U,U.parent.parent,shelf_top=.07);ar=ArmRealizer(U,U.parent.parent,sc,lay,gripper=0.,backoff_max=0.)
 c=np.array(lb['center']);n=np.array(lb['normal']);u=np.array(lb['u']);v=np.cross(n,u)
 # 닫힌 집게와 카메라는 같은 강체. 광축 정면에서 집게 꼭짓점이 벽 안에 들어가면 반드시 충돌.
 qq=ar.q([0.,.3,.3,0.,math.pi]);cam=ar.rb.link_poses(qq,None)['camera_optical_frame'];ci=np.linalg.inv(cam)
 depth=max(float((V@ci[:3,:3].T+ci[:3,3])[:,2].max()) for name,V in ar.rb.world_shapes(qq) if 'gripper' in name)
 lower=depth+.005-.0005
 T=np.eye(4);T[:3,0]=u;T[:3,1]=-v;T[:3,2]=-n
 # B0에서 이미 얻은 평면 팔 해를 초기값으로 사용. 관절·충돌 기준은 동일.
 seeds=[[0.,-.51,-.96,1.21,math.pi],[0.,1.42,-1.15,-.54,math.pi],[0.,1.3,0.,-1.56,math.pi],[0.,.76,.65,-1.41,math.pi]]
 ref=.42 if c[2]<.16 else .32 if c[2]<.25 else .24
 bases=[planar_pose(float(bx),float(c[1]),math.pi) for bx in sorted(np.arange(.20,.621,.02),key=lambda x:abs(x-ref))]
 bases=[b for b in bases if not ar.body_hits(b)]
 def solve(d):
  des=T.copy();des[:3,3]=c+d*n
  for b in bases:
   for q,pe,ae in ar.solve_arm(b,des,seeds):
    if ar.arm_hits(b,q):continue
    actual=ar.cam(b,q)
    if sc.los_blocked(actual[:3,3],c+n*.002,end_skip=0.):continue
    return b,q,actual
  return None
 found=None
 for d in np.round(np.arange(.04,.2001,.005),4):
  if d<=lower:continue
  r=solve(float(d))
  if r is not None:found=(float(d),r);break
 if not found:
  unreachable.append({'label':lb['id'],'tilt_deg':0.,'reason':'벽 정면 차체 격자에서도 자세 없음'});print(lb['id'],'없음',flush=True);continue
 dmin,r0=found
 for extra in (0.,.02,.04):
  r=r0 if extra==0 else solve(dmin+extra)
  if r is None:
   unreachable.append({'label':lb['id'],'tilt_deg':0.,'extra_m':extra,'reason':'추가 거리 자세 없음'});continue
  b,q,cam=r;ci=np.linalg.inv(cam)
  a=ci@np.r_[c-u*lb['size'][0]/2,1.];z=ci@np.r_[c+u*lb['size'][0]/2,1.]
  mp=636*np.linalg.norm(z[:2]/z[2]-a[:2]/a[2])/len(code128.modules(lb['text']))
  plan.append({'kind':'arm','id':f"{lb['id']}_a_e{int(extra*100):02d}_t00",'label':lb['id'],'world':lb['world'],'seed':lay['seed'],
    'dist_nominal':round(dmin+extra,4),'d_min':dmin,'extra_m':extra,'backoff_m':0.,'tilt_deg':0.,'azimuth_deg':0.,'roll_deg':0.,'flip':False,
    'roll_actual_deg':float(math.degrees(math.atan2(cam[:3,0]@v,cam[:3,0]@u))), 'module_px':round(float(mp),3),
    'T_world_cam':cam.tolist(),'T_world_base_footprint':b.tolist(),'arm_q':{**dict(zip(ARM_J,map(float,q))),'gripper_left_joint':0.},'planner':'B0 벽 정면 격자·명시 초기값'})
 diagnostics.append({'label':lb['id'],'gripper_optical_depth_max_m':depth,'proven_wall_collision_below_m':lower,'d_min_grid_m':dmin,'base_candidates':len(bases)})
 print(lb['id'],'d_min',dmin,'만든 장면',sum(p['label']==lb['id'] for p in plan),flush=True)
(ROOT/'r3_normal_plan.json').write_text(json.dumps({'plan':plan,'labels':old['labels'],'unreachable':unreachable,'diagnostics':diagnostics},indent=1,ensure_ascii=False))
