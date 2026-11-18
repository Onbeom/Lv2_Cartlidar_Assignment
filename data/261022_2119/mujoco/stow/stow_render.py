import os; os.environ.setdefault('MUJOCO_GL','egl')
import mujoco, numpy as np, math, json, sys
m=mujoco.MjModel.from_xml_path('/home/pa4/git/lv2_next_module/261022_2119/mujoco/drive/results/261023_1851/M7_dtB_wa0.0163/model.xml'); d=mujoco.MjData(m)
ARM=['RMX52_Base_Yaw','RMX52_Shoulder','RMX52_Elbow','RMX52_Wrist_Pitch','RMX52_Wrist_Roll_Reversed']
adr={n:m.jnt_qposadr[m.joint(n).id] for n in ARM}
armb=set(b for b in range(m.nbody) if m.body(b).name.startswith(('arm_link','gripper')))
W,H=480,270
for g in range(m.ngeom):
    if m.geom_bodyid[g] in armb: m.geom_group[g]=4
r=mujoco.Renderer(m,H,W); r.enable_depth_rendering()
opt_all=mujoco.MjvOption(); opt_noarm=mujoco.MjvOption(); opt_all.geomgroup[4]=1; opt_noarm.geomgroup[4]=0
def cam_for(name):
    mujoco.mj_forward(m,d); b=m.body(name).id; p=d.xpos[b].copy(); R=d.xmat[b].reshape(3,3).copy()
    return p,R
def frac(q, camname):
    d.qpos[:]=0; d.qpos[3]=1
    for n,v in zip(ARM,q): d.qpos[adr[n]]=v
    mujoco.mj_forward(m,d)
    b=m.body(camname).id; p=d.xpos[b]; R=d.xmat[b].reshape(3,3)
    cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    fwd=R[:,2]; cam.lookat[:]=p+fwd; cam.distance=1.0
    cam.azimuth=math.degrees(math.atan2(fwd[1],fwd[0])); cam.elevation=math.degrees(math.asin(fwd[2]))
    m.vis.global_.fovy=42.5
    r.update_scene(d,camera=cam,scene_option=opt_all); d1=r.render().copy()
    r.update_scene(d,camera=cam,scene_option=opt_noarm); d2=r.render().copy()
    return float((d1<d2-1e-4).mean())
res=json.load(open(sys.argv[1]))
lim={'sh':(-1.5,1.5),'el':(-1.4,1.5),'pi':(-1.97,1.7)}
def margin(q): return min(q[1]-lim['sh'][0],lim['sh'][1]-q[1],q[2]-lim['el'][0],lim['el'][1]-q[2],q[3]-lim['pi'][0],lim['pi'][1]-q[3])
cand=[x for x in res if margin(x['q'])>=0.05-1e-9]
out=[]
for x in cand:
    ff=frac(x['q'],'top_camera_front_color_optical_frame'); fr=frac(x['q'],'top_camera_rear_color_optical_frame')
    x['front_frac']=round(ff,4); x['rear_frac']=round(fr,4); out.append(x)
ref={k:(round(frac(q,'top_camera_front_color_optical_frame'),4),round(frac(q,'top_camera_rear_color_optical_frame'),4)) for k,q in {'q0':(0,0,0,0,0),'P4pose':(0,1.5,1.5,1.7,0)}.items()}
print('reference (front,rear):',ref)
json.dump(out,open(sys.argv[2],'w'))
for y in (0.0,round(math.pi,3)):
    a=[x for x in out if abs(x['q'][0]-y)<1e-3]
    print('yaw',y,len(a))
    for x in sorted(a,key=lambda x:(x['front_frac']+x['rear_frac'],x['taumax']))[:8]: print('  ',x['q'],'front',x['front_frac'],'rear',x['rear_frac'],'tau',x['taumax'],'zc',x['zc'],'zmax',x['zmax'],'clear',x['clear_mm'])
r.close()
