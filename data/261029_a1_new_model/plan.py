from common import *
from scipy.spatial import ConvexHull
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'));from robot_geom import Robot,box_depth,hull_depth
sys.path.insert(0,str(ROOT/'Cartlidar/sim_render'));from r12_poses import Scene
rb=Robot(URDF,PKG);draw=json.loads((ROOT/'Cartlidar/sim_world/data/maze_dice_mission.json').read_text());seed=int(sys.argv[1]) if len(sys.argv)>1 else 20261029;rng=np.random.default_rng(seed);plan=[];diag=[]
# 모델의 fixed연결은 같은몸체, moving 인접몸체는 조립 접촉을 허용한다.
component={l:l for l in rb.links}
def comp(l):
 while component[l]!=l:l=component[l]
 return l
for j in rb.joints.values():
 if j['type']=='fixed':component[comp(j['child'])]=comp(j['parent'])
adj={frozenset((comp(j['child']),comp(j['parent']))) for j in rb.joints.values() if j['type']!='fixed'}
shape_cache={};self_cache={}
def shapes(q):
 key=tuple(sorted(q.items()))
 if key not in shape_cache:shape_cache[key]=rb.world_shapes(q)
 return shape_cache[key]
def self_ok(q):
 key=tuple(sorted(q.items()))
 if key in self_cache:return self_cache[key]
 items=shapes(q)
 for i,(a,A) in enumerate(items):
  for b,B in items[i+1:]:
   if comp(a)==comp(b) or frozenset((comp(a),comp(b))) in adj:continue
   if np.all(A.min(0)<B.max(0)) and np.all(B.min(0)<A.max(0)) and hull_depth(A,B)>1e-6:self_cache[key]=False;return False
 self_cache[key]=True;return True

def obstacles(lay):
 s=object.__new__(Scene);s.boxes=[]
 for p in draw['panels']:
  hx,hy=(p['length']/2,.0015) if p['axis']=='x' else (.0015,p['length']/2);s.boxes.append((*p['center'],hx,hy,.38,'wall'))
 for p in draw['posts']:s.boxes.append((p['x'],p['y'],.018,.018,.4,'post'))
 for p in lay['shelves']:s.boxes.append((*p['center'][:2],p['size'][0]/2,p['size'][1]/2,p['size'][2],'shelf'))
 s.box_arr=np.array([b[:5] for b in s.boxes]);boxes=[(np.array([x,y,h/2]),np.array([hx+.005,hy+.005,h/2+.005]),n) for x,y,hx,hy,h,n in s.boxes]
 for p in lay['pucks']:
  lo,hi=np.array(p['aabb']);boxes.append(((lo+hi)/2,(hi-lo)/2+.005,p['name']))
 return s,boxes

def hit_world(T,q,boxes):
 C=np.array([b[0] for b in boxes]);H=np.array([b[1] for b in boxes])
 for link,v in shapes(q):
  V=v@T[:3,:3].T+T[:3,3];near=np.all((V.min(0)<C+H)&(V.max(0)>C-H),axis=1)
  for i in np.flatnonzero(near):
   if box_depth(V,C[i],H[i])>=0:return True
 return False
for world_seed in range(1,7):
 w=ROOT/f'Cartlidar/sim_world/worlds/dice_mission/seed_{world_seed}';lay=json.loads((w/'layout.json').read_text());scene,boxes=obstacles(lay);surfaces={p['name']:CAM['arm']['geometry']['puck_surface'](p) for p in lay['pucks']};chosen=None
 for attempt in range(2500):
  sh=lay['shelves'][int(rng.integers(len(lay['shelves'])))];phi=float(rng.uniform(0,2*math.pi));dist=float(rng.uniform(.48,.73));xy=np.array(sh['center'][:2])+dist*np.array([math.cos(phi),math.sin(phi)]);T=base_pose(*xy,phi+math.pi+float(rng.uniform(-.15,.15)));poses=FRAMES.poses(Q);front=T@poses[CAM['d435']['frame']];arm=T@poses[CAM['arm']['frame']];possible=[]
  for p in lay['pucks']:
   b=(np.linalg.inv(T)@np.r_[p['center'],1])[:3]
   if np.all(b>=[-.3,-.25,-.1]) and np.all(b<=[.35,.25,.6]):continue
   u,v,z=CAM['d435']['geometry']['project'](front,CAM['d435']['K'],p['center'])
   if z>.15 and 20<u<620 and 20<v<460 and not scene.los_blocked(front[:3,3],p['center'],end_skip=.05):possible.append((np.linalg.norm(b[:2]),p))
  if not possible:continue
  p=min(possible,key=lambda x:x[0])[1]
  if p['location']!=sh['name']:continue
  if CAM['arm']['geometry']['in_frac'](*surfaces[p['name']],arm,CAM['arm']['K'])<1:continue
  gx,gy=draw['grid']['x'],draw['grid']['y'];V=np.vstack([v for l,v in shapes(Q)])@T[:3,:3].T+T[:3,3]
  if V[:,0].min()<gx[0] or V[:,0].max()>gx[-1] or V[:,1].min()<gy[0] or V[:,1].max()>gy[-1] or hit_world(T,Q,boxes):continue
  for sign in [1,-1]:
   qs=[];end=None
   for angle in np.arange(0,1.66,.15):
    q=dict(Q,joint1=float(sign*angle))
    if not self_ok(q) or hit_world(T,q,boxes):break
    poses=FRAMES.poses(q);Tc=T@poses[CAM['arm']['frame']];qs.append(q)
    if CAM['arm']['geometry']['in_frac'](*surfaces[p['name']],Tc,CAM['arm']['K'])==0:end=len(qs)-1;break
   if end is None:continue
   seq=[(Q,'visible')]*4+[(q,'pan_out') for q in qs[1:]]+[(qs[-1],'hold_out')]*3+[(q,'pan_back') for q in qs[-2::-1]]+[(Q,'visible')]*4
   near=[q_ for q_ in lay['pucks'] if q_['name']!=p['name']];nb=min(near,key=lambda n:(n['color']!=p['color'],np.linalg.norm(np.array(n['center'])-p['center'])))
   chosen=dict(id=f'w{world_seed}_out00',kind='out',world=str(w),seed=world_seed,shelf=sh['name'],target=p['name'],neighbor=nb['name'],neighbor_same_color=nb['color']==p['color'],T_world_base_link=T.tolist(),T_world_base_footprint=(T@np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-.00675],[0,0,0,1]])).tolist(),T_world_d435=front.tolist(),initial_q=Q,arm=[dict(stage=stage,arm_q=q,T_world_cam=(T@FRAMES.poses(q)[CAM['arm']['frame']]).tolist(),backoff_m=0.) for q,stage in seq]);break
  if chosen:break
 if not chosen:diag.append(dict(seed=world_seed,planned=False,attempts=attempt+1));print(diag[-1],flush=True);continue
 plan.append(chosen);diag.append(dict(seed=world_seed,planned=True,attempts=attempt+1,frames=len(chosen['arm']),target=chosen['target']));print(diag[-1],flush=True)
(OUT/f'plan_{seed}.json').write_text(json.dumps(dict(plan=plan,diagnostic=diag),indent=1)+'\n');assert len(plan)==6,diag
