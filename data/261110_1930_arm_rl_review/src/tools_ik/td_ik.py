"""위에서 (집게 아래로) 퍽을 컵에 넣는 자세 IK - 12 칸, 입구 기준 높이별. base_link 좌표."""
import numpy as np, math
from scipy.optimize import least_squares
def Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
LIM=np.array([[-3.14159,3.14159],[-2.14,1.6],[-1.6,1.6],[-1.92,2.0],[-2.8,2.8]])
def fk(q):
    p=np.array([0.096,0,0.131]); R=Rz(q[0])
    p=p+R@[0,0,0.0595]; R=R@Ry(q[1])
    p=p+R@[0.024,0,0.128]; R=R@Ry(q[2])
    p=p+R@[0.124,0,0]; R=R@Ry(q[3])
    elbow=None
    p=p+R@[0.028,0,0]; R=R@Rx(q[4])
    return p+R@[0.14,0,0], R
CUPS={"R_1":(0.0488595,0.0766667),"R_2":(0.0111472,0.03),"R_3":(0.0111472,-0.03),"R_4":(0.0488595,-0.0766667),
      "G_1":(-0.023249,0.0770954),"G_2":(-0.043409,0.027),"G_3":(-0.043409,-0.027),"G_4":(-0.023249,-0.0770954),
      "B_1":(-0.077968,0.0788418),"B_2":(-0.093082,0.027),"B_3":(-0.093082,-0.027),"B_4":(-0.077968,-0.0788418)}
RIM=0.150
def solve(c, h, roll_dir=None):
    p=np.array([c[0],c[1],RIM+h])
    th=math.atan2(c[1],c[0]-0.096)
    def res(q):
        P,R=fk(q); r=list((P-p)*100)+list((R[:,0]-np.array([0,0,-1.0]))*3)
        if roll_dir is not None: r+=list((R[:,1]-roll_dir)*1)
        return r
    best=None
    for a in (-0.6,-0.2,0.2,0.6,1.0):
        for b in (-1.2,-0.6,0.0,0.6,1.2):
            for cc in (-1.5,-0.5,0.5,1.5):
                s=least_squares(res,np.array([th,a,b,cc,0.0]),bounds=(LIM[:,0],LIM[:,1]))
                if best is None or s.cost<best.cost-1e-9: best=s
    q=best.x; P,R=fk(q); e=np.linalg.norm(P-p)*1000; tilt=math.degrees(math.acos(np.clip(-R[2,0],-1,1)))
    marg=min(min(q[i]-LIM[i,0],LIM[i,1]-q[i]) for i in range(1,4))
    return q,e,tilt,marg
if __name__=="__main__":
    # 잡는 점 높이 (입구 위): 넣기 끝 = 퍽 위 15 mm 아래를 쥠 -> 퍽 바닥 = 턱 (입구 - 15), 퍽 위 = 입구 + 45 -> 잡는 점 입구 + 30
    # 넣기 시작 (퍽 바닥 입구 + 5) = 잡는 점 입구 + 50 / 꽂힌 옆 퍽 위로 지나갈 높이 (퍽 바닥 입구 + 60) = 입구 + 105
    for k,c in CUPS.items():
        d=math.hypot(c[0]-0.096,c[1]); out=[]
        for h in (0.030,0.050,0.105):
            q,e,tilt,m=solve(c,h)
            ok= e<2 and tilt<3 and m>0.05
            out.append(f"+{h*1000:.0f}:{'OK' if ok else 'NG'} e{e:.0f} 기울{tilt:.0f}° 여유{m:.2f} q[{', '.join(f'{v:.2f}' for v in q)}]")
        print(f"{k} r{d*1000:.0f}mm | "+" | ".join(out))
