#!/usr/bin/env python3
"""P2b 손-눈 보정 모의 확인: URDF 설계 카메라에서 알려진 만큼 어긋난 카메라로 보드 코너를 만들고(잡음 포함) 풀이가 되찾는지.
handeye_selftest.py <robot.urdf.xacro> [자세 수=12] [화소 잡음 px=0.3] [관절 잡음 rad=0.002] [관절 범위 ±rad=0.35] [보드=10x7]"""
import sys
import numpy as np, cv2, yaml
from handeye_lib import Chain, urdf_from_xacro, T_of, rpy_to_R, RZ_PI, board_pose, solve, consistency, rot_deg
rng = np.random.default_rng(1)
xa = sys.argv[1]; N = int(sys.argv[2]) if len(sys.argv) > 2 else 12; RANGE = float(sys.argv[5]) if len(sys.argv) > 5 else 0.35
pxn = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3; qn = float(sys.argv[4]) if len(sys.argv) > 4 else 0.002
ch = Chain(urdf_from_xacro(xa))
Xd = ch.design('link5', 'arm_camera_optical_frame')
Xtrue = Xd @ T_of(rpy_to_R(0.02, -0.03, 0.01), [0.004, -0.003, 0.005])   # 설계에서 약 5~7 mm · 2° 어긋남
info = yaml.safe_load(open(__import__('os').path.join(__import__('os').path.dirname(__file__), 'arm_camera_1280x720.yaml')))
K = np.array(info['camera_matrix']['data']).reshape(3, 3); D = np.array(info['distortion_coefficients']['data'])
BX, BY = (int(v) for v in (sys.argv[6] if len(sys.argv) > 6 else '10x7').split('x'))
NC = (BX - 1) * (BY - 1); MINC = 20 if NC >= 40 else 10
sq = 0.025 if BX >= 10 else 0.046
q0 = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0}
# 보드 자리: 설계 카메라가 0 자세에서 보는 곳 근처 바닥(z 0)
Tc0 = ch.fk(q0) @ Xd
ray = Tc0[:3, 2]; o = Tc0[:3, 3]; s = -o[2] / ray[2] if ray[2] < 0 else 0.3
ctr = o + s * ray; ctr[2] = 0.0
Tbb = T_of(rpy_to_R(0, 0, rng.uniform(-0.3, 0.3)), ctr - np.array([BX / 2 * sq, BY / 2 * sq, 0]))
obj = np.array([[((i % (BX - 1)) + 1) * sq, ((i // (BX - 1)) + 1) * sq, 0] for i in range(NC)])
TL, TC, used = [], [], 0
tries = 0
while used < N and tries < 2000:
    tries += 1
    q = {k: rng.uniform(-RANGE, RANGE) for k in q0}
    TbL = ch.fk(q)
    Topt = np.linalg.inv(TbL @ Xtrue) @ Tbb
    Traw = RZ_PI @ Topt
    pc = (Traw[:3, :3] @ obj.T).T + Traw[:3, 3]
    if np.any(pc[:, 2] < 0.08): continue
    img, _ = cv2.projectPoints(obj, cv2.Rodrigues(Traw[:3, :3])[0], Traw[:3, 3], K, D)
    img = img.reshape(-1, 2)
    inb = (img[:, 0] > 5) & (img[:, 0] < 1275) & (img[:, 1] > 5) & (img[:, 1] < 715)
    if inb.sum() < MINC: continue
    ids = np.nonzero(inb)[0]
    T_est, rms = board_pose(img[ids] + rng.normal(0, pxn, (len(ids), 2)), ids, K, D, sq, squares_x=BX)
    if T_est is None: continue
    qm = {k: v + rng.normal(0, qn) for k, v in q.items()}
    TL.append(ch.fk(qm)); TC.append(RZ_PI @ T_est); used += 1
print(f'보드 {BX}x{BY} 칸 {sq * 1e3:.1f} mm, 자세 {used} (시도 {tries}), 화소 잡음 {pxn} px, 관절 잡음 {qn} rad')
for m in ('TSAI', 'PARK', 'DANIILIDIS'):
    X = solve(TL, TC, m)
    dp, dr, _ = consistency(TL, TC, X)
    e_t = np.linalg.norm(X[:3, 3] - Xtrue[:3, 3]) * 1e3; e_r = rot_deg(X[:3, :3].T @ Xtrue[:3, :3])
    print(f'  {m:10s} 참값과 차 {e_t:5.2f} mm · {e_r:4.2f}° | 보드 흩어짐 위치 최대 {max(dp):5.2f} mm · 회전 최대 {max(dr):4.2f}°')
d = np.linalg.inv(Xd) @ Xtrue
print(f'(설계와 참값 차 {np.linalg.norm(d[:3,3])*1e3:.1f} mm · {rot_deg(d[:3,:3]):.2f}°)')
