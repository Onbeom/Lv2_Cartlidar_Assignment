#!/usr/bin/env python3
"""팔 카메라 자세별 실측 보정표 (PC, 11-15). 팔 길이 · 영점이 정해지기 전, 찍어 둔 보드 사진으로 '찍은 관절각에서만' 맞는 보정을 낸다.
  cal_pose_table.py <control URDF> <intr yaml> <출력 yaml> <기록 폴더:보드x_mm> …
- 장마다 보드 PnP 로 실제 카메라 자세(base_link 기준)를 내고, 자세(caps 의 pose 이름)마다 link5 → 카메라 장착 보정
  (피치 · 롤 · x · z, link5 기준 — cal_fit3 M1b 에서 요를 뺀 것) 을 따로 맞춘다. 보드 y · 요는 기록 폴더마다 풂(노트북을 다시 놓음),
  보드 가운데 x 는 폴더마다 손 측정값, 높이 = 책상 위 75 mm 가정. 요 · y 보정은 보드 y · 요와 섞여 낼 수 없어 0 으로 둠(한계).
- 출력: 자세마다 관절각 평균 q, 보정(피치 · 롤 rad, x · z m), 장 수, 잔차(위치 · 높이 mm, 방향 °), 설계 FK 대비 카메라 높이 차(mm).
"""
import json
import math
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares

import handeye_lib as L
from handeye_lib import Chain, T_of, rpy_to_R

urdf, intr, out = sys.argv[1], sys.argv[2], sys.argv[3]
specs = [(a.rsplit(':', 1)[0], float(a.rsplit(':', 1)[1]) / 1000.0) for a in sys.argv[4:]]
ch5 = Chain(open(urdf).read(), base='base_link', tip='link5')
X0 = ch5.design('link5', 'arm_camera_optical_frame')
y = yaml.safe_load(open(intr))
K = np.array(y['camera_matrix']['data']).reshape(3, 3)
D = np.array(y['distortion_coefficients']['data'])
RZ = np.diag([-1.0, -1.0, 1.0, 1.0])
SQ, SY = 0.04917, 0.04875
FLOOR, SCREEN_H = -0.00675, 0.075

data = []
for si, (d, bx) in enumerate(specs):
    for l in open(d + '/caps.jsonl'):
        c = json.loads(l)
        T, rms = L.board_pose(np.array(c['corners']), c['ids'], K, D, SQ, squares_x=6, square_y_m=SY)
        if T is None or rms > 1.0:
            continue
        data.append(dict(sess=si, bx=bx, pose=c['pose'], q=np.array(c['q'][:5]), Tcb=RZ @ T))
poses = sorted({d['pose'] for d in data})
NS = len(specs)
NPP = 4   # 자세마다 피치 · 롤 · x · z


def board_T(bx, b):
    R = rpy_to_R(math.pi, 0.0, b[1])
    ctr = np.array([bx, b[0], FLOOR + SCREEN_H])
    return T_of(R, ctr - R @ np.array([3 * SQ, 2 * SY, 0.0]))


def corr_T(p):
    return T_of(rpy_to_R(p[1], p[0], 0.0), [p[2], 0.0, p[3]])


def fk_cam(q, p=None):
    Xc = X0 if p is None else corr_T(p) @ X0
    return ch5.fk({f'joint{i + 1}': q[i] for i in range(5)}) @ Xc


def obs(d, x):
    b = x[NPP * len(poses) + 2 * d['sess']:NPP * len(poses) + 2 * d['sess'] + 2]
    return board_T(d['bx'], b) @ np.linalg.inv(d['Tcb'])


def resid(x):
    r = []
    for d in data:
        k = poses.index(d['pose'])
        Tp, To = fk_cam(d['q'], x[NPP * k:NPP * k + NPP]), obs(d, x)
        r += list((Tp[:3, 3] - To[:3, 3]) * 1000.0)
        dR = Tp[:3, :3].T @ To[:3, :3]
        r += list(np.degrees(np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2) * 10.0)
    return np.array(r)


x0 = np.zeros(NPP * len(poses) + 2 * NS)
for s in range(NS):
    x0[NPP * len(poses) + 2 * s:NPP * len(poses) + 2 * s + 2] = [-0.03, math.pi / 2]
sol = least_squares(resid, x0)
r = sol.fun.reshape(-1, 6)
table = {}
print(f'장 {len(data)} · 자세 {poses} · 기록 {NS}')
for k, pz in enumerate(poses):
    idx = [i for i, d in enumerate(data) if d['pose'] == pz]
    p = sol.x[NPP * k:NPP * k + NPP]
    qm = np.mean([data[i]['q'] for i in idx], 0)
    dz_fk = np.mean([(obs(data[i], sol.x)[2, 3] - fk_cam(data[i]['q'])[2, 3]) * 1000 for i in idx])
    pos = np.linalg.norm(r[idx, :3], axis=1); hz = np.abs(r[idx, 2]); ang = np.linalg.norm(r[idx, 3:], axis=1) / 10
    T5 = ch5.fk({f'joint{i + 1}': qm[i] for i in range(5)})   # base_link → link5 (설계 FK, 실행 중 가까운 자세 찾기용)
    rpy5 = [math.atan2(T5[2, 1], T5[2, 2]), math.asin(max(-1.0, min(1.0, -T5[2, 0]))), math.atan2(T5[1, 0], T5[0, 0])]
    table[pz] = dict(q=[round(float(v), 4) for v in qm], n=len(idx),
                     link5_xyz=[round(float(v), 4) for v in T5[:3, 3]], link5_rpy=[round(float(v), 4) for v in rpy5],
                     pitch_rad=round(float(p[0]), 5), roll_rad=round(float(p[1]), 5), x_m=round(float(p[2]), 4), z_m=round(float(p[3]), 4),
                     resid_pos_mm_max=round(float(pos.max()), 2), resid_z_mm_max=round(float(hz.max()), 2), resid_ang_deg_max=round(float(ang.max()), 3),
                     cam_z_minus_fk_mm=round(float(dz_fk), 1))
    print(f'{pz}: {len(idx)} 장, 보정 피치 {math.degrees(p[0]):+.2f}° 롤 {math.degrees(p[1]):+.2f}° x {p[2]*1000:+.1f} z {p[3]*1000:+.1f} mm | '
          f'잔차 최대 위치 {pos.max():.1f} · 높이 {hz.max():.1f} mm · 방향 {ang.max():.2f}° | 실제 카메라 높이 − FK {dz_fk:+.1f} mm')
for s, (d, bx) in enumerate(specs):
    b = sol.x[NPP * len(poses) + 2 * s:NPP * len(poses) + 2 * s + 2]
    print(f'  보드 {d.split("/")[-1]}: x {bx*1000:.1f} (실측) y {b[0]*1000:+.1f} mm 요 {math.degrees(b[1]):.1f}°')
yaml.safe_dump(dict(note='팔 카메라 자세별 실측 보정 (cal_pose_table.py, link5 → 카메라 장착 보정 = 피치 · 롤 · x · z, 요 · y 는 0). '
                         '찍은 관절각 q 근처에서만 유효.', urdf=urdf, poses=table), open(out, 'w'), allow_unicode=True, sort_keys=False)
print('→', out)
