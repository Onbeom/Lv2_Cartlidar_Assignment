import mujoco, numpy as np, math, itertools, time, json, sys
m=mujoco.MjModel.from_xml_path('/home/pa4/git/lv2_next_module/261022_2119/mujoco/drive/results/261023_1851/M7_dtB_wa0.0163/model.xml'); d=mujoco.MjData(m)
ARM=['RMX52_Base_Yaw','RMX52_Shoulder','RMX52_Elbow','RMX52_Wrist_Pitch','RMX52_Wrist_Roll_Reversed']
adr={n:m.jnt_qposadr[m.joint(n).id] for n in ARM}
armb=set(b for b in range(m.nbody) if m.body(b).name.startswith(('arm_link_2','arm_link_3','arm_link_4','arm_link_5','gripper')))
armg=[g for g in range(m.ngeom) if m.geom_bodyid[g] in armb]
for g in armg: m.geom_margin[g]=0.01  # 10 mm 안 근접도 접촉으로 보고
# 팔 형상 점(지역 좌표, 메시 꼭짓점 5개 중 1개)
local={}
for g in armg:
    t=m.geom_type[g]; s=m.geom_size[g]
    if t==mujoco.mjtGeom.mjGEOM_MESH:
        mid=m.geom_dataid[g]; a=m.mesh_vertadr[mid]; n=m.mesh_vertnum[mid]; local[g]=m.mesh_vert[a:a+n][::3]
    elif t==mujoco.mjtGeom.mjGEOM_BOX: local[g]=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*s
    else:
        ang=np.linspace(0,2*np.pi,16); ring=np.stack([np.cos(ang)*s[0],np.sin(ang)*s[0],np.zeros_like(ang)],1); local[g]=np.vstack([ring+[0,0,s[1]],ring-[0,0,s[1]]])
BOX=(-0.226,0.218,-0.153,0.153)
cams={}
for nm in ['top_camera_front_color_optical_frame','top_camera_rear_color_optical_frame']:
    mujoco.mj_forward(m,d); b=m.body(nm).id; cams[nm]=(d.xpos[b].copy(), d.xmat[b].reshape(3,3).copy())
HF,VF=math.radians(69.4)/2, math.radians(42.5)/2  # D435 컬러 화각(가로 69.4°, 세로 42.5°)
def eval_pose(q):
    d.qpos[:]=0; d.qpos[3]=1
    for n,v in zip(ARM,q): d.qpos[adr[n]]=v
    mujoco.mj_fwdPosition(m,d)
    P=np.vstack([local[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g] for g in armg])
    out=max(BOX[0]-P[:,0].min(),P[:,0].max()-BOX[1],BOX[2]-P[:,1].min(),P[:,1].max()-BOX[3],0)
    clear=1.0
    for i in range(d.ncon):
        c=d.contact[i]; b1,b2=m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]
        if (b1 in armb)!=(b2 in armb) or (b1 in armb and b2 in armb and abs(b1-b2)>1 and not ({m.body(b1).name[:7],m.body(b2).name[:7]}=={'gripper'})):
            if (b1 in armb) and (b2 in armb): continue
            clear=min(clear,c.dist)
    occl={}
    for nm,(p,R) in cams.items():
        v=(P-p)@R   # 광학 좌표: x 오른쪽, y 아래, z 앞
        inside=(v[:,2]>0.02)&(np.abs(np.arctan2(v[:,0],v[:,2]))<HF)&(np.abs(np.arctan2(v[:,1],v[:,2]))<VF)
        occl[nm[11:16]]=int(inside.sum())
    mujoco.mj_fwdVelocity(m,d) if False else None
    d.qvel[:]=0; d.qacc[:]=0; mujoco.mj_inverse(m,d)
    tau=[float(d.qfrc_inverse[m.jnt_dofadr[m.joint(n).id]]) for n in ARM[:4]]
    zc=float(sum(m.body_mass[b]*d.xipos[b][2] for b in armb)/sum(m.body_mass[b] for b in armb))
    return out,clear,occl,tau,zc,float(P[:,2].max()),float(P[:,2].min())
t0=time.time(); res=[]
yaws=[0.0,math.pi]
for y in yaws:
  for sh in np.arange(-1.5,1.501,0.1):
    for el in np.arange(-1.4,1.501,0.1):
      for pi_ in np.arange(-1.9,1.701,0.1):
        q=(y,sh,el,pi_,0.0); out,clear,occl,tau,zc,zmax,zmin=eval_pose(q)
        if out==0 and clear>=0.01-1e-9:
            res.append(dict(q=[round(v,3) for v in q],clear_mm=round(1000*min(clear,1),1),occl=occl,tau=[round(t,3) for t in tau],taumax=round(max(abs(t) for t in tau),3),zc=round(zc,3),zmax=round(zmax,3),zmin=round(zmin,3)))
print('evaluated in',round(time.time()-t0,1),'s; feasible',len(res))
json.dump(res,open(sys.argv[1],'w'))
