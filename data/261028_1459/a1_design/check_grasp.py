import sys,json,math,collections
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
sys.path.insert(0,'Cartlidar/sim_render');from arm_pose import ArmRealizer
from r12_poses import Scene
U=Path('261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf')
lay=json.loads(Path('Cartlidar/sim_world/worlds/dice_mission/seed_2/layout.json').read_text());draw=json.loads(Path('Cartlidar/sim_world/data/maze_dice_mission.json').read_text());sc=Scene(lay,draw,U,U.parent.parent,shelf_top=.07)
ar=ArmRealizer(U,U.parent.parent,sc,lay,gripper=.02);pts=np.vstack([v[:,:2] for _,v in ar.body]);sc.footprint=pts[ConvexHull(pts).vertices]
counts=collections.Counter();original=ar.arm_hits

def hits(t,q):
 out=original(t,q)
 if out:counts[str(out[0])]+=1
 return out
ar.arm_hits=hits
for tid in (10,6):
 p=next(p for p in lay['pucks'] if p['tag_id']==tid);top=p['aabb'][1][2]
 for roll in (0.,math.pi):
  counts.clear();r=ar.realize_tcp([np.array([*p['center'][:2],top+.05])],[],roll=roll,max_bases=48)
  print(tid,p['shape'],p['standing'],'roll',roll,'result',r,'collisions',counts.most_common(4),flush=True)
