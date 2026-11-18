import sys, json, math, numpy as np
from pathlib import Path
B=Path('/home/pa4/git/lv2_next_module')
sys.path.insert(0,str(B/'Cartlidar/sim_world')); sys.path.insert(0,str(B/'Cartlidar/sim_render'))
from robot_geom import Robot
from urdf_chain import fixed_transform, planar_pose
from scipy.spatial import ConvexHull
pkg=B/'261022_2119/urdf/waffle_4wd_description_261022_2119'; U=str(pkg/'urdf/waffle_4wd.urdf')
r=Robot(U,pkg)
q={'RMX52_Base_Yaw':0,'RMX52_Shoulder':1.4,'RMX52_Elbow':-1.0,'RMX52_Wrist_Pitch':-1.3,'RMX52_Wrist_Roll_Reversed':0,'gripper_left_joint':0.0}
P=np.vstack([V[:,:2] for _,V in r.world_shapes(q)]); hull=P[ConvexHull(P).vertices]
print('footprint x',hull[:,0].min().round(3),hull[:,0].max().round(3),'y',hull[:,1].min().round(3),hull[:,1].max().round(3))
Tc=fixed_transform(U,'top_camera_front_color_optical_frame')
K=dict(fx=605.85,fy=605.68,cx=324.37,cy=245.14)
d=json.load(open(B/'Cartlidar/sim_world/data/maze_upper_drawing.json'))
boxes=[]  # (cx,cy,hx,hy,top)
for pn in d['panels']:
    hx,hy=(pn['length']/2,0.0015) if pn['axis']=='x' else (0.0015,pn['length']/2)
    boxes.append((pn['center'][0],pn['center'][1],hx,hy,0.38))
for p in d['posts']: boxes.append((p['x'],p['y'],0.018,0.018,0.40))
def pt_box_dist(x,y,b):
    dx=max(abs(x-b[0])-b[2],0); dy=max(abs(y-b[1])-b[3],0); return math.hypot(dx,dy)
def seg_hits(p0,p1,bx):
    d=p1-p0
    for b in bx:
        lo=np.array([b[0]-b[2],b[1]-b[3],-1.0]); hi=np.array([b[0]+b[2],b[1]+b[3],b[4]])
        t0,t1=0.0,0.97   # 신호등 몸통 자체는 빼려고 끝 3 % 제외
        okk=True
        for k in range(3):
            if abs(d[k])<1e-12:
                if p0[k]<lo[k] or p0[k]>hi[k]: okk=False;break
            else:
                a=(lo[k]-p0[k])/d[k]; c=(hi[k]-p0[k])/d[k]
                if a>c: a,c=c,a
                t0=max(t0,a); t1=min(t1,c)
                if t0>t1: okk=False;break
        if okk: return True
    return False
for sd in (1,2,3):
    l=json.load(open(B/f'Cartlidar/sim_world/worlds/b_dice/seed_{sd}/layout.json'))
    sl=l['signal_light']; A=l['assumptions']['signal_light']
    bx=boxes+[(s['center'][0],s['center'][1],0.08,0.08,0.16) for s in l['shelves']]
    segz=[A['first_led_z']+(i+0.5)*A['segment_length'] for i in range(3)]
    L=np.array([sl['x'],sl['y']])
    for dist in (0.5,0.9,1.3):
        ok=[]
        for phi in range(0,360,5):
            ph=math.radians(phi); pos=L+dist*np.array([math.cos(ph),math.sin(ph)]); yaw=ph+math.pi
            T=planar_pose(pos[0],pos[1],yaw); Rz=T[:2,:2]
            fp=hull@Rz.T+pos
            # inside maze bounds
            if not (-2.202<pos[0]<2.202 and 0<pos[1]<3.152): continue
            if min(pt_box_dist(x,y,b) for x,y in fp for b in bx[:len(boxes)])<0.02: continue
            # footprint edges crossing panels: sample edges
            bad=False
            for i in range(len(fp)):
                a,bb=fp[i],fp[(i+1)%len(fp)]
                for s in np.linspace(0,1,12):
                    x,y=a+(bb-a)*s
                    if any(pt_box_dist(x,y,b)<0.01 for b in bx): bad=True;break
                if bad:break
            if bad: continue
            # robot centre region inside: also check panels strictly inside hull -> approximate by grid points
            Tw=T@Tc; cam=Tw[:3,3]; Ti=np.linalg.inv(Tw)
            vis=True
            for z in segz:
                p=np.array([sl['x'],sl['y'],z]); qq=Ti@np.append(p,1); v=K['fy']*qq[1]/qq[2]+K['cy']; u=K['fx']*qq[0]/qq[2]+K['cx']
                if not(0<=u<640 and 0<=v<480): vis=False;break
                if seg_hits(cam,p,bx): vis=False;break
            if vis: ok.append(phi)
        print(f'seed{sd} cell {sl["cell"]} wall {sl["wall_side"]} d={dist}: {len(ok)}/72 ok', ok)
