import sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'Cartlidar/sim_render')
from arm_pose import ArmRealizer
from urdf_chain import planar_pose
class Scene:boxes=[]
u=Path('261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf');ar=ArmRealizer(u,u.parent.parent,Scene(),{'pucks':[]},gripper=.02)
rng=np.random.default_rng(0);seeds=[rng.uniform([a for a,b in ar.lim],[b for a,b in ar.lim]) for _ in range(12)]
rows=[]
for height in (.13,.145,.16,.18,.20):
 for radius in (.24,.28,.30,.32,.34,.36):
  b=planar_pose(-radius,0,0);q=next(ar.solve_tcp(b,np.array([0.,0.,height]),0,seeds),None)
  row={'height':height,'radius':radius,'ik':q is not None,'q':q.tolist() if q is not None else None};rows.append(row);print(height,radius,q is not None,flush=True)
Path('Cartlidar/sim_render/results/a1_code_261028/tcp_workspace_limits_only.json').write_text(json.dumps(rows,indent=1))
