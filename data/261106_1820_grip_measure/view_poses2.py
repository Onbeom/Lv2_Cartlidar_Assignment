"""집게를 들어 올려 쥔 퍽을 앞 D435 로 볼 자세 후보 (기하 계산). pitch = q2+q3+q4 (위 − , 0.785 = 45° 아래)."""
import math,sys,json,numpy as np
sys.path.insert(0,'../Cartlidar/sim_render'); from urdf_chain import fixed_transform
U='/home/pa4/git/Lv2_Cartlidar_Assignment_armcam/workspace/urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf'
T_c=fixed_transform(U,'top_camera_front_color_optical_frame',root='base_footprint');Ti=np.linalg.inv(T_c);T_a=fixed_transform(U,'arm_base_link',root='base_footprint')
def Ry(a):c,s=math.cos(a),math.sin(a);return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1.]])
def Tr(x,y,z):m=np.eye(4);m[:3,3]=[x,y,z];return m
def fk(q2,q3,q4):return T_a@Tr(0,0,.0595)@Ry(q2)@Tr(.024,0,.128)@Ry(q3)@Tr(.124,0,0)@Ry(q4)@Tr(.028,0,0)
S=math.sqrt(.5);zp=np.array([-S,0,S]);L=np.array([.139,0,0])-.015*zp;K=(605.85,605.68,324.37,245.14);W,H=640,480
Q5=[-.7854]  # 자세 pitch [rad], 아래 반복에서 씀
res=[]
for pd in (45,30,15,0,-15,-30,-45):
 p=math.radians(pd)
 for q2 in np.arange(-2.1,1.6,.05):
  for q3 in np.arange(-1.6,1.6,.05):
    q4=p-q2-q3
    if not -1.9<q4<1.98:continue
    G=fk(q2,q3,q4);c=(G@np.append(L,1))[:3]
    if c[0]<.1:continue
    ring=[L+z*zp+.015*np.array([math.cos(a)*S,math.sin(a),math.cos(a)*S]) for z in(-.03,.03) for a in np.linspace(0,2*np.pi,8,endpoint=False)]
    P=(G@np.column_stack([np.array(ring),np.ones(len(ring))]).T).T;x=(Ti@P.T).T
    if(x[:,2]<.12).any():continue
    u,v=K[0]*x[:,0]/x[:,2]+K[2],K[1]*x[:,1]/x[:,2]+K[3]
    if u.min()<25 or u.max()>W-25 or v.min()<25 or v.max()>H-25:continue
    ray=T_c[:3,3]-c;d=np.linalg.norm(ray);ray/=d
    ang=math.degrees(math.acos(np.clip(-G[:3,0]@ray,-1,1)))   # 집게 뒤쪽에서 본 각: 90 = 링크 z 쪽에서 바로
    nrm=G[:3,:3]@zp; tag=math.degrees(math.acos(np.clip(nrm@ray,-1,1)))
    # 시작 자세 q_front_tilt 와 관절 거리
    dq=abs(q2+1.32032)+abs(q3-.67766)+abs(q4-1.42805)
    res.append((pd,round(float(q2),2),round(float(q3),2),round(float(q4),2),round(ang,1),round(tag,1),round(float(d),3),round(float(c[0]),3),round(float(c[2]),3),round(float(dq),2)))
print(len(res))
for pd in (45,30,15,0,-15,-30,-45):
    a=[r for r in res if r[0]==pd]
    if a:
        a=sorted(a,key=lambda r:-r[4]);print('pitch',pd,'n',len(a),'(pitch,q2,q3,q4,시선각,태그각,거리,퍽x,퍽z,관절거리)');print('  각 최대',a[0])
        b=sorted(a,key=lambda r:r[9])[0];print('  q_front 가까움',b)
print('--- q2 >= -1.35 (몸체에서 떨어진 쪽)')
for pd in (15,0,-15,-30,-45):
    a=[r for r in res if r[0]==pd and r[1]>=-1.35 and r[3]<1.9 and r[8]>0.2]
    a=sorted(a,key=lambda r:-r[4])[:3]
    for r in a: print(r)
