import os, math, subprocess; os.environ.setdefault('MUJOCO_GL','egl')
import mujoco, numpy as np
OUT='/home/pa4/git/lv2_next_module/261022_2119/mujoco/stow'
m=mujoco.MjModel.from_xml_path('/home/pa4/git/lv2_next_module/261022_2119/mujoco/drive/results/261023_1851/M7_dtB_wa0.0163/model.xml'); d=mujoco.MjData(m)
ARM=['RMX52_Base_Yaw','RMX52_Shoulder','RMX52_Elbow','RMX52_Wrist_Pitch','RMX52_Wrist_Roll_Reversed']
adr={n:m.jnt_qposadr[m.joint(n).id] for n in ARM}
r=mujoco.Renderer(m,360,480)
def shot(q, cam_kind):
    d.qpos[:]=0; d.qpos[3]=1
    for n,v in zip(ARM,q): d.qpos[adr[n]]=v
    mujoco.mj_forward(m,d); cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    if cam_kind=='side':
        cam.lookat[:]=[0,0,0.2]; cam.distance=1.2; cam.azimuth=90; cam.elevation=-10
    elif cam_kind=='iso':
        cam.lookat[:]=[0,0,0.2]; cam.distance=1.2; cam.azimuth=135; cam.elevation=-30
    else:
        b=m.body('top_camera_front_color_optical_frame').id; p=d.xpos[b]; f=d.xmat[b].reshape(3,3)[:,2]
        cam.lookat[:]=p+f; cam.distance=1.0; cam.azimuth=math.degrees(math.atan2(f[1],f[0])); cam.elevation=math.degrees(math.asin(f[2])); m.vis.global_.fovy=42.5
    r.update_scene(d,camera=cam); return r.render()
for tag,q in {'S0_yaw0':(0.0,1.4,-1.0,-1.3,0.0),'S1_yawm3.092':(-(math.pi-0.05),1.0,-0.8,-1.4,0.0),'ref_q0':(0,0,0,0,0)}.items():
    imgs=[shot(q,k) for k in ('side','iso','front')]
    row=np.concatenate(imgs,axis=1)
    p=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{row.shape[1]}x{row.shape[0]}','-i','-','-frames:v','1',f'{OUT}/{tag}.png'],stdin=subprocess.PIPE); p.communicate(row.tobytes())
r.close(); print('ok')
