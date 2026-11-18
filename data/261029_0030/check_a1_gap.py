import sys,ast,json,math
from pathlib import Path
import numpy as np
ROOT=Path('/home/pa4/git/lv2_next_module');sys.path.insert(0,str(ROOT/'Cartlidar/sim_render'))
from arm_pose import ArmRealizer,ARM_J
from r12_poses import Scene
O=ROOT/'Cartlidar/sim_render/results/a1_code_261028';U=ROOT/'261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf';draw=json.loads((ROOT/'Cartlidar/sim_world/data/maze_dice_mission.json').read_text());plans=json.loads((O/'mission_gap_plan.json').read_text())['plan']
tree=ast.parse((ROOT/'261028_1459/render_a1_view.py').read_text());funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('puck_surface','cam_points','in_frac','project')];ns=dict(np=np,math=math,HALF_XY=.015,HALF_Z=.03,W=640,H=480);exec(compile(ast.Module(body=funcs,type_ignores=[]),'<geometry>','exec'),ns);K=dict(fx=283,fy=283,cx=320,cy=240)
report=[];okplans=[]
for s in plans:
 lay=json.loads((Path(s['world'])/'layout.json').read_text());scene=Scene(lay,draw,U,U.parent.parent,shelf_top=.07);ar=ArmRealizer(U,U.parent.parent,scene,lay,gripper=.02);base=np.array(s['T_world_base_footprint_arm']);pucks={p['name']:p for p in lay['pucks']};surf=ns['puck_surface'](pucks[s['target']]);frames=[s['arm'][0]];hits=[]
 for k,f in enumerate(s['arm'][1:],1):
  prev=frames[-1];a=np.array([prev['arm_q'][j] for j in ARM_J]);b=np.array([f['arm_q'][j] for j in ARM_J]);n=int(np.ceil(np.max(abs(b-a))*15/4.502949));n=max(1,n)
  for t in range(1,n):
   q=a+(b-a)*t/n;h=ar.arm_hits(base,q)
   if h:hits.append((k,h))
   frames.append(dict(f,arm_q=ar.q(q),T_world_cam=ar.cam(base,q).tolist(),interpolated=True))
  frames.append(f)
 holds=[f for f in frames if f['stage']=='hold_out'];outside=[ns['in_frac'](*surf,np.array(f['T_world_cam']),K)==0 for f in holds];nb=[ns['project'](np.array(f['T_world_cam']),K,pucks[s['neighbor']]['center']) for f in holds];neighbor=all(z>0 and 10<=x<630 and 10<=y<470 for x,y,z in nb);qs=np.array([[f['arm_q'][j] for j in ARM_J] for f in frames]);speed=float(np.max(abs(np.diff(qs,axis=0)))*15)
 rec=dict(seed=s['seed'],frames=len(frames),max_joint_speed=speed,interpolation_collisions=hits,target_out=all(outside),neighbor_in=neighbor);report.append(rec);print(rec)
 if all(outside) and neighbor and not hits and speed<=4.50295:okplans.append(dict(s,arm=frames))
(O/'gap_plan_check.json').write_text(json.dumps(report,indent=2)+'\n');(O/'mission_gap_plan_checked.json').write_text(json.dumps({'plan':okplans},indent=1)+'\n')
assert len(okplans)==6,(len(okplans),'불완전한 계획은 렌더하지 않음')
