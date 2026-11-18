from td_ik import *
# (1) 위에서 쥔 퍽: 집게를 수직에서 몇 도까지 기울여야 B 줄에 닿나
def solve_tilt(c,h,maxtilt):
    p=np.array([c[0],c[1],RIM+h]); th=math.atan2(c[1],c[0]-0.096)
    def res(q):
        P,R=fk(q); t=math.degrees(math.acos(np.clip(-R[2,0],-1,1)))
        return list((P-p)*100)+[max(0,t-maxtilt)*0.2]
    best=None
    for a in (-0.4,0.0,0.4,0.8):
        for b in (-1.2,-0.6,0.0,0.6):
            for cc in (0.5,1.0,1.5,2.0):
                s=least_squares(res,np.array([th,a,b,cc,0.0]),bounds=(LIM[:,0],LIM[:,1]))
                if best is None or s.cost<best.cost: best=s
    q=best.x; P,R=fk(q); return q, np.linalg.norm(P-p)*1000, math.degrees(math.acos(np.clip(-R[2,0],-1,1)))
for mt in (5,10,15,20):
    q,e,t=solve_tilt(CUPS["B_2"],0.030,mt); print(f"B_2 위에서, 기울기 ≤{mt}°: 오차 {e:.0f} mm 기울 {t:.0f}° q {np.round(q,2)}")
# (2) 옆에서 쥔 퍽 (집게 수평, 바깥 방향): 높이별 관절 - 지금 정책의 '완전히 굽힘' 비교
def solve_side(c,h):
    p=np.array([c[0],c[1],RIM+h]); th=math.atan2(c[1],c[0]-0.096); rad=np.array([math.cos(th),math.sin(th),0])
    def res(q):
        P,R=fk(q); return list((P-p)*100)+list((R[:,0]-rad)*3)
    best=None
    for a in (-1.2,-0.6,0.0,0.6):
        for b in (-0.6,0.4,1.0,1.5):
            for cc in (-1.5,-0.8,0.0,0.8):
                s=least_squares(res,np.array([th,a,b,cc,0.0]),bounds=(LIM[:,0],LIM[:,1]))
                if best is None or s.cost<best.cost: best=s
    q=best.x; P,R=fk(q); return q,np.linalg.norm(P-p)*1000
for h in (0.060,0.090,0.120,0.150):
    q,e=solve_side(CUPS["B_2"],h); print(f"B_2 옆에서 쥠, 잡는 점 입구+{h*1000:.0f} (퍽 바닥 +{h*1000-45:.0f}): 오차 {e:.0f} mm q {np.round(q,2)}")
