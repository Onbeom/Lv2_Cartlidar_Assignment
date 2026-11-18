"""쥔 퍽을 앞 D435 로 볼 팔 자세 후보 (기하 계산). 집게 45° · 관절 한계 안. 빠른 FK."""
import math,sys,json,numpy as np
sys.path.insert(0,'../Cartlidar/sim_render'); from urdf_chain import fixed_transform
U='/home/pa4/git/Lv2_Cartlidar_Assignment_armcam/workspace/urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf'
T_c=fixed_transform(U,'top_camera_front_color_optical_frame',root='base_footprint');Ti=np.linalg.inv(T_c)
T_a=fixed_transform(U,'arm_base_link',root='base_footprint')
def Rz(a):c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1.]])
def Ry(a):c,s=math.cos(a),math.sin(a);return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1.]])
def Tr(x,y,z):m=np.eye(4);m[:3,3]=[x,y,z];return m
def fk(q1,q2,q3,q4):return T_a@Rz(q1)@Tr(0,0,.0595)@Ry(q2)@Tr(.024,0,.128)@Ry(q3)@Tr(.124,0,0)@Ry(q4)@Tr(.028,0,0)
assert np.allclose(fk(.3,-.5,.7,.5),fixed_transform(U,'gripper_roll_link',root='base_footprint',q=dict(joint1=.3,joint2=-.5,joint3=.7,joint4=.5)),atol=1e-5)
S=math.sqrt(.5);L=np.array([.139,0,0])-.015*np.array([-S,0,S]);zp=np.array([-S,0,S]);f=606;W,H=640,480
res=[]
for y in range(0,91,10):
 for q2 in np.arange(-2.1,1.6,.1):
  for q3 in np.arange(-1.6,1.6,.1):
    q4=.7854-q2-q3
    if not -1.9<q4<1.98:continue
    q1=math.radians(y);G=fk(q1,q2,q3,q4)
    c=(G@np.append(L,1))[:3]
    if c[2]<.17:continue   # 팔레트 위
    ring=[]
    for z in(-.03,.03):
        for a in np.linspace(0,2*np.pi,8,endpoint=False):
            ring.append(L+z*zp+.015*np.array([math.cos(a)*S,math.sin(a),math.cos(a)*S]))
    P=(G@np.column_stack([np.array(ring),np.ones(len(ring))]).T).T
    p=(Ti@P.T).T
    if (p[:,2]<.12).any():continue
    u,v=f*p[:,0]/p[:,2]+W/2,f*p[:,1]/p[:,2]+H/2
    if u.min()<25 or u.max()>W-25 or v.min()<25 or v.max()>H-25:continue
    ray=T_c[:3,3]-c;ray/=np.linalg.norm(ray);ang=math.degrees(math.acos(-G[:3,0]@ray))
    res.append((y,round(q2,2),round(q3,2),round(q4,2),round(ang,1),round(float(np.linalg.norm(T_c[:3,3]-c)),3),round(float(c[0]),3),round(float(c[1]),3),round(float(c[2]),3)))
print(len(res))
for y in range(0,91,10):
    a=[r for r in res if r[0]==y]
    if a:
        a.sort(key=lambda r:r[5]); print('j1',y,'n',len(a),'가장 가까움',a[0]);a.sort(key=lambda r:-r[4]);print('  각 큼',a[0])
json.dump(res,open('view_poses.json','w'))
