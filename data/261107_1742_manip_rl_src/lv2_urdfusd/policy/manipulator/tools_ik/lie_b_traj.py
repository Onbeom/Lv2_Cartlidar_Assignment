"""누운 직육면체 B안 경로 (2026-11-04): 위에서 쥐고 손목으로 세운 퍽 (집게 수평 · 바깥, 쥐는 점 = 퍽 중심 + 15 mm) 을 바깥 열 4 칸 위로 옮기는 관절 경유점.

앞 피벗 (관절1 0, 퍽 바닥 반지름 PIV_R · 높이 Z_T) -> 관절1 을 칸 방향으로 (왼쪽 +, 오른쪽 −) -> 같은 높이로 칸 반지름까지 (carry)
-> 끝 = 칸 바로 위 (퍽 바닥 Z_T). 내리기 · 놓기 · 빼기는 chainlie_play.py 의 손 IK. 결과 ../lie_b_traj.json
팀 URDF FK (td_ik.fk, TCP = gripper_roll_link + 0.14), 경유점 사이 앞 해를 시작값으로 (같은 가지)."""
import json, math, numpy as np
from td_ik import fk, LIM, least_squares

T = json.load(open("../pallet_place_traj.json"))
J1 = np.array([0.096, 0.0])
GDZ = 0.020          # 잡는 점 = 퍽 중심 + 20 mm (2026-11-04 t1: +15 면 손가락 밑이 벽 위 3 mm 라 처지면 벽에 걸림)
BOT = 0.030 + GDZ    # TCP - 퍽 바닥
PIV_R = 0.18         # 관절1 을 돌리는 반지름. t1 의 0.14 (관절 2 −1.76) 는 팔꿈치가 팔레트 벽을 스침 (5 N) -> 0.18 (관절 2 −1.17)
Z_T = 0.240          # 운반 퍽 바닥 높이: 바깥 열 꽂힌 퍽 윗면 0.196 + 44 mm
LIMM = LIM.copy(); LIMM[0] = [-3.0, 3.0]

def ik(p, d, q0):
    def res(q):
        P, R = fk(q)
        return list((P - p) * 100) + list((R[:, 0] - d) * 3) + [R[2, 1] * 3]
    s = least_squares(res, q0, bounds=(LIMM[:, 0], LIMM[:, 1]))
    q = s.x; P, R = fk(q)
    return q, np.linalg.norm(P - p) * 1000, math.degrees(math.acos(np.clip(R[:, 0] @ d, -1, 1)))

def pt(r, th, zb):
    return np.array([J1[0] + r * math.cos(th), r * math.sin(th), zb + BOT]), np.array([math.cos(th), math.sin(th), 0.0])

# 앞 피벗 (여러 시작값 중 가장 여유 큰 해)
p, d = pt(PIV_R, 0.0, Z_T)
best = None
for a in (-1.9, -1.5, -1.0, -0.6):
    for b in (0.8, 1.2, 1.5):
        for c in (0.3, 0.7, 1.1):
            q, e, ang = ik(p, d, np.array([0, a, b, c, 0.0]))
            m = min(min(q[i] - LIM[i, 0], LIM[i, 1] - q[i]) for i in range(1, 4))
            if e < 0.5 and ang < 0.5 and (best is None or q[1] > best[0][1]): best = (q, m)  # 어깨를 가장 덜 젖힌 해
q_piv, m_piv = best
print(f"앞 피벗 q {np.round(q_piv, 3)} 여유 {m_piv:+.3f}")
out = []
for s in T["slots"]:
    if s["row"] != 3: continue
    c = np.array([s["x"], s["y"]]); rs = float(np.linalg.norm(c - J1)); th = math.atan2(c[1] - J1[1], c[0] - J1[0])
    rot = [[th * k / max(1, int(abs(th) / 0.05)), *q_piv[1:4], 0.0] for k in range(int(abs(th) / 0.05) + 1)]
    rot[-1][0] = th
    q = np.array(rot[-1]); carry = []; worst = (0.0, 0.0, 9.0)
    for r in np.linspace(PIV_R, rs, max(2, int((rs - PIV_R) / 0.005) + 1)):
        p, d = pt(r, th, Z_T)
        q, e, ang = ik(p, d, q)
        m = min(min(q[i] - LIM[i, 0], LIM[i, 1] - q[i]) for i in range(1, 4))
        worst = (max(worst[0], e), max(worst[1], ang), min(worst[2], m))
        carry.append([round(float(v), 5) for v in q])
    print(f"{s['row']}열 {s['index']} 반지름 {rs * 1000:.0f} mm 관절1 {math.degrees(th):+.1f}° | carry {len(carry)} 점 오차 최대 {worst[0]:.2f} mm {worst[1]:.2f}° 관절 여유 최소 {worst[2]:+.3f} | 칸 위 q {carry[-1]}")
    out.append(dict(row=s["row"], index=s["index"], x=s["x"], y=s["y"], floor_z=s["floor_z"], wall_top_z=s["wall_top_z"], j1_yaw=th,
                    q_pivot=[round(float(v), 5) for v in q_piv], rotate=[[round(float(v), 5) for v in x] for x in rot], carry=carry,
                    max_err_mm=round(worst[0], 3), max_ang_deg=round(worst[1], 3), joint_margin=round(worst[2], 4)))
json.dump(dict(note="누운 직육면체 B안: 세운 퍽 (집게 수평 바깥, 잡는 점 = 퍽 중심 + 15 mm) 을 바깥 열 칸 위 (퍽 바닥 z_transit) 로. tools_ik/lie_b_traj.py 가 만듦",
               grasp_dz=GDZ, pivot_r=PIV_R, z_transit=Z_T, slots=out), open("../lie_b_traj.json", "w"), ensure_ascii=False, indent=1)
print("-> ../lie_b_traj.json")
