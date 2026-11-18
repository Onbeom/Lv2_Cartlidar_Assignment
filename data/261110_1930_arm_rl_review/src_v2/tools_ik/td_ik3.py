from td_ik import *
J1=math.radians(160)
LIM2=LIM.copy(); LIM2[0]=[-J1,J1]
def solve_mode(c,h,mode):
    p=np.array([c[0],c[1],RIM+h]); th=math.atan2(c[1],c[0]-0.096); rad=np.array([math.cos(th),math.sin(th),0])
    def res(q):
        P,R=fk(q); r=list((P-p)*100)
        if mode=="top": r+=list((R[:,0]-np.array([0,0,-1.0]))*3)
        else: r+=list((R[:,0]-rad)*3)
        return r
    sols=[]
    for j1 in (max(-J1,min(J1,th)), th-math.pi if th>0 else th+math.pi):
        for a in (-1.8,-1.2,-0.6,0.0,0.6):
            for b in (-1.2,-0.4,0.4,1.2):
                for cc in (-1.5,-0.5,0.5,1.5):
                    s=least_squares(res,np.array([j1,a,b,cc,0.0]),bounds=(LIM2[:,0],LIM2[:,1]))
                    q=s.x; P,R=fk(q); e=np.linalg.norm(P-p)*1000
                    marg=min(min(q[i]-LIM2[i,0],LIM2[i,1]-q[i]) for i in range(0,4))
                    # 몸·판 충돌 대충: 팔꿈치·손목 점이 판 (z<0.17) 아래로 내려가면 NG
                    sols.append((e,-marg,q))
    ok=[s for s in sols if s[0]<1.5 and -s[1]>0.03]
    if not ok: b=min(sols,key=lambda s:s[0]); return None,b
    return min(ok,key=lambda s:s[1]),None
for mode in ("side","top"):
    print("==",{"side":"옆에서 쥠 (집게 수평, 컵 바깥 방향)","top":"위에서 쥠 (집게 아래)"}[mode])
    for k,c in CUPS.items():
        h=0.060 if mode=="side" else 0.030
        best,bad=solve_mode(c,h,mode)
        if best: print(f"  {k}: 됨 q {np.round(best[2],2)} 한계 여유 {-best[1]:.2f}")
        else: print(f"  {k}: 안 됨 (가장 가까운 오차 {bad[0]:.0f} mm, q {np.round(bad[2],2)})")
