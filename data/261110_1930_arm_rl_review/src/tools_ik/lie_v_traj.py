"""누운 직육면체 해석 2 (2026-11-04 사용자 제안): 반지름 방향으로 누운 퍽의 안쪽 끝을 수평 · 바깥 집게로 쥐고, 들어 손목을 아래로 90° 숙여
퍽을 집게 아래로 세워 매단 채 (사진 자세, 집게 수직) 칸에 넣는 경로. 집게 수직은 1 · 2열 8 칸만 (3열은 손목 관절 4 한계).

쥔 퍽 (집게 수직 뒤): 퍽 바닥 = 잡는 점 − 45 mm (아래) − 15 mm (관절1 쪽), 카메라 z = 칸 반지름 바깥.
경로 (칸마다): 피벗 (관절1 0, 잡는 점 반지름 = 칸 반지름 + 15 mm, 퍽 바닥 Z_T) -> 관절1 을 칸 방향으로 (칸 바로 위) -> 내림은 chain45_play 손 IK.
결과 ../lie_v_traj.json. 팀 URDF FK (td_ik)."""
import json, math, numpy as np
from td_ik import fk, LIM, least_squares

T = json.load(open("../pallet_place_traj.json"))
J1 = np.array([0.096, 0.0])
OFF_IN = 0.015    # 퍽 축이 잡는 점보다 관절1 쪽으로
DROP = 0.045      # 잡는 점 - 퍽 바닥 (세로)
Z_T = 0.215       # 회전 · 운반 때 퍽 바닥 높이 (1 · 2열 꽂힌 퍽 윗면 0.174 · 0.184 위)
LIMM = LIM.copy(); LIMM[0] = [-3.0, 3.0]
z = np.array([0, 0, 1.0])

def ik(p, th, q0):
    h = np.array([math.cos(th), math.sin(th), 0.0])
    def res(q):
        P, R = fk(q)
        return list((P - p) * 100) + list((R[:, 0] + z) * 3) + list((R[:, 2] - h) * 3)
    best = None
    for sd in [q0] + [np.array([q0[0], a, b, c, 0.0]) for a in (-1.0, -0.4, 0.2, 0.8) for b in (-1.2, -0.4, 0.4) for c in (1.0, 1.6, 1.9)]:
        s = least_squares(res, np.clip(sd, LIMM[:, 0] + 1e-4, LIMM[:, 1] - 1e-4), bounds=(LIMM[:, 0], LIMM[:, 1]))
        if best is None or s.cost < best.cost: best = s
        if s.cost < 1e-9: break
    q = best.x; P, R = fk(q)
    m = min(min(q[i] - LIM[i, 0], LIM[i, 1] - q[i]) for i in range(1, 4))
    return q, np.linalg.norm(P - p) * 1000, math.degrees(math.acos(np.clip(-R[2, 0], -1, 1))), m

out = []
for s in T["slots"]:
    if s["row"] == 3: continue
    c = np.array([s["x"], s["y"]]); rs = float(np.linalg.norm(c - J1)); th = math.atan2(c[1] - J1[1], c[0] - J1[0])
    rt = rs + OFF_IN
    q_piv, e0, a0, m0 = ik(np.array([J1[0] + rt, 0.0, Z_T + DROP]), 0.0, np.array([0.0, 0.0, -0.3, 1.9, 0.0]))
    n_ = int(abs(th) / 0.05) + 1
    rot = []
    for k in range(n_ + 1):
        q = q_piv.copy(); q[0] = th * k / n_; rot.append([round(float(v), 5) for v in q])
    # 칸 위 (관절1 = th) 자세 확인 + 놓는 높이 (벽 위 −10 mm) 자세
    q_hov, e1, a1, m1 = ik(np.array([J1[0] + rt * math.cos(th), rt * math.sin(th), Z_T + DROP]), th, np.array(rot[-1]))
    q_pl, e2, a2, m2 = ik(np.array([J1[0] + rt * math.cos(th), rt * math.sin(th), s["wall_top_z"] - 0.010 + DROP]), th, q_hov)
    print(f"{s['row']}열 {s['index']} 관절1 {math.degrees(th):+.1f}° | 피벗 오차 {e0:.1f}mm 기울기 {a0:.1f}° 여유 {m0:+.2f} q {np.round(q_piv, 2)}"
          f" | 칸 위 여유 {m1:+.2f} | 놓는 자세 오차 {e2:.1f}mm {a2:.1f}° 여유 {m2:+.2f} q {np.round(q_pl, 2)}")
    out.append(dict(row=s["row"], index=s["index"], x=s["x"], y=s["y"], floor_z=s["floor_z"], wall_top_z=s["wall_top_z"], j1_yaw=th,
                    q_pivot=[round(float(v), 5) for v in q_piv], rotate=rot, carry=[[round(float(v), 5) for v in q_hov]],
                    q_place=[round(float(v), 5) for v in q_pl], joint_margin=round(float(min(m0, m1, m2)), 4)))
json.dump(dict(note="누운 직육면체 해석 2: 안쪽 끝을 수평 집게로 쥐고 손목을 숙여 세워 매단 퍽 (집게 수직, 퍽 바닥 = 잡는 점 −45 mm 아래 · −15 mm 관절1 쪽) 을 1 · 2열 칸 위 (퍽 바닥 z_transit) 로. tools_ik/lie_v_traj.py",
               off_in=OFF_IN, drop=DROP, z_transit=Z_T, slots=out), open("../lie_v_traj.json", "w"), ensure_ascii=False, indent=1)
print("-> ../lie_v_traj.json")
