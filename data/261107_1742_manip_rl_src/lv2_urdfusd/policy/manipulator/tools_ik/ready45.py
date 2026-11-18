from td_ik import fk, LIM, np, math, least_squares
q0=np.array([0.0,1.07,0.60,-1.66,0.0]); P0,R0=fk(q0); print("READY 잡는 점", np.round(P0,4), "집게 x", np.round(R0[:,0],3))
for pitch in (45,):
    for dz in (0.0,0.02,0.04):
        p=P0+np.array([0,0,dz]); s=math.sin(math.radians(pitch))
        def res(q):
            P,R=fk(q); return list((P-p)*100)+[(-R[2,0]-s)*3, R[2,1]*3, R[1,0]*3]
        best=min((least_squares(res,np.array([0,a,b,c,0]),bounds=(LIM[:,0],LIM[:,1])) for a in (0.3,0.8,1.2) for b in (-0.5,0.3,1.0) for c in (-1.5,-0.5,0.5)),key=lambda r:r.cost)
        q=best.x; P,R=fk(q); m=min(min(q[i]-LIM[i,0],LIM[i,1]-q[i]) for i in range(1,4))
        print(f"피치 {pitch} 높이 +{dz*1000:.0f}: q {np.round(q,3)} 오차 {np.linalg.norm(P-p)*1000:.1f} mm 여유 {m:.2f}")
