"""쥔 퍽은 세운 채 (손가락 닫는 축 수평), 집게 피치 p (0 = 수평 옆 쥠, 90 = 위에서) 별로 12 칸 닿는지. 관절1 |q1| <= 160°.
두 갈래: 정면 (관절1 을 컵 쪽으로) / 어깨 너머 (관절1 은 앞쪽 근처, 팔을 뒤로 젖힘). 높이 = 퍽 바닥이 벽 위 + 5 mm (살짝 위에서 놓기)."""
from td_ik import fk, LIM, np, math, least_squares
SL=[("3-1",-0.0931,0.027,0.166),("3-2",-0.0931,-0.027,0.166),("3-0",-0.078,0.0788,0.166),("3-3",-0.078,-0.0788,0.166),
    ("2-1",-0.0434,0.027,0.154),("2-2",-0.0434,-0.027,0.154),("2-0",-0.0232,0.0771,0.154),("2-3",-0.0232,-0.0771,0.154),
    ("1-1",0.0111,0.03,0.144),("1-2",0.0111,-0.03,0.144),("1-0",0.0489,0.0767,0.144),("1-3",0.0489,-0.0767,0.144)]
J1=math.radians(160); L=LIM.copy(); L[0]=[-J1,J1]
def solve(x,y,wt,pitch):
    p=np.array([x,y,wt+0.005+0.045])
    def res(q):
        P,R=fk(q); a=R[:,0]; az=-a[2]                       # 집게 아래쪽 성분 = sin(피치)
        return list((P-p)*100)+[(az-math.sin(math.radians(pitch)))*3, R[2,1]*3]  # 손가락 닫는 축 (y) 수평
    out=[]
    th=math.atan2(y,x-0.096)
    for j1 in (max(-J1,min(J1,th)), th-math.pi if th>0 else th+math.pi):
        for a in (-2.0,-1.2,-0.4,0.4,1.2):
            for b in (-1.2,0.0,1.2):
                for c in (-1.5,0.0,1.5):
                    s=least_squares(res,np.array([j1,a,b,c,0.0]),bounds=(L[:,0],L[:,1]))
                    q=s.x; P,R=fk(q); e=np.linalg.norm(P-p)*1000; pe=abs(math.degrees(math.asin(np.clip(-R[2,0],-1,1)))-pitch)
                    m=min(min(q[i]-L[i,0],L[i,1]-q[i]) for i in range(4))
                    if e<1.5 and pe<2 and m>0.05: out.append((m,q))
    return max(out,key=lambda t:t[0]) if out else None
print("칸   | "+" | ".join(f"피치 {p}°" for p in (0,30,45,60,90)))
for n,x,y,wt in SL:
    row=[]
    for p in (0,30,45,60,90):
        r=solve(x,y,wt,p)
        row.append("  -  " if r is None else f"{'정면' if abs(abs(r[1][0])-abs(math.atan2(y,x-0.096)))<0.3 else '어깨너머'} q1 {math.degrees(r[1][0]):.0f}°")
    print(n," | ".join(row))
