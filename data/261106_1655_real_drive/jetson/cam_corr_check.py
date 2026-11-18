#!/usr/bin/env python3
"""팔 카메라 자세별 보정 확인 (PC, 11-15). 맞춤에 안 쓴 새 보드 사진으로 '보정 전 · 뒤' 카메라 높이 차를 낸다 (PR #296 켜기 조건).
  cam_corr_check.py <control URDF> <intr yaml> <보정표 yaml(cal_pose_table.py 출력)> <기록 폴더:보드x_mm> …
- 장마다 보드 PnP 로 실제 카메라 자세를 내고(보드 y · 요는 폴더마다 풂, x 는 손 측정, 높이 75 mm 가정), 설계 FK 와 보정 FK 의 높이 차를 비교.
- 보정 항목 고르기는 perception 과 같게: base_link ← link5 가 표와 위치 0.03 m · 각 0.10 rad 안이면 그 항목, 아니면 '없음'(보정 안 함).
- 판정(켜기 조건, README): 보정이 걸린 장의 보정 뒤 |높이 차| ≤ 5 mm.
"""
import json
import math
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares

import handeye_lib as L
from handeye_lib import Chain, T_of, rpy_to_R

urdf, intr, tab = sys.argv[1], sys.argv[2], sys.argv[3]
specs = [(a.rsplit(':', 1)[0], float(a.rsplit(':', 1)[1]) / 1000.0) for a in sys.argv[4:]]
ch5 = Chain(open(urdf).read(), base='base_link', tip='link5')
X0 = ch5.design('link5', 'arm_camera_optical_frame')
y = yaml.safe_load(open(intr))
K = np.array(y['camera_matrix']['data']).reshape(3, 3)
D = np.array(y['distortion_coefficients']['data'])
RZ = np.diag([-1.0, -1.0, 1.0, 1.0])
SQ, SY = 0.04917, 0.04875
FLOOR, SCREEN_H = -0.00675, 0.075
TOL_M, TOL_RAD = 0.03, 0.10
table = yaml.safe_load(open(tab))['poses']

data = []
for si, (d, bx) in enumerate(specs):
    for l in open(d + '/caps.jsonl'):
        c = json.loads(l)
        T, rms = L.board_pose(np.array(c['corners']), c['ids'], K, D, SQ, squares_x=6, square_y_m=SY)
        if T is None or rms > 1.0:
            continue
        data.append(dict(sess=si, bx=bx, pose=c.get('pose', '?'), img=c.get('image', ''), q=np.array(c['q'][:5]), Tcb=RZ @ T))


def board_T(bx, b):
    R = rpy_to_R(math.pi, 0.0, b[1])
    ctr = np.array([bx, b[0], FLOOR + SCREEN_H])
    return T_of(R, ctr - R @ np.array([3 * SQ, 2 * SY, 0.0]))


def select(T5):
    best, bd = None, 1e9
    for k, v in table.items():
        dp = np.linalg.norm(T5[:3, 3] - np.array(v['link5_xyz']))
        Rt = rpy_to_R(*v['link5_rpy'])
        da = math.acos(max(-1.0, min(1.0, (np.trace(Rt.T @ T5[:3, :3]) - 1) / 2)))
        if dp <= TOL_M and da <= TOL_RAD and dp < bd:
            best, bd = k, dp
    return best


def cam(q, name):
    T5 = ch5.fk({f'joint{i + 1}': q[i] for i in range(5)})
    if name is None:
        return T5 @ X0
    v = table[name]
    return T5 @ T_of(rpy_to_R(v['roll_rad'], v['pitch_rad'], 0.0), [v['x_m'], 0.0, v['z_m']]) @ X0


for d in data:
    d['sel'] = select(ch5.fk({f'joint{i + 1}': d['q'][i] for i in range(5)}))


def resid(x):   # 보드 y · 요만 풂: 보정 FK(없으면 설계)와 관측의 위치 · 방향 차
    r = []
    for d in data:
        b = x[2 * d['sess']:2 * d['sess'] + 2]
        To = board_T(d['bx'], b) @ np.linalg.inv(d['Tcb'])
        Tp = cam(d['q'], d['sel'])
        r += list((Tp[:3, 3] - To[:3, 3]) * 1000.0)
    return np.array(r)


x0 = np.tile([-0.03, math.pi / 2], len(specs))
sol = least_squares(resid, x0)
ok_all = True
print('장 | 자세(기록) | 고른 보정 | 높이 차(실제 − 설계 FK) | 보정 뒤(실제 − 보정 FK) mm')
for d in data:
    b = sol.x[2 * d['sess']:2 * d['sess'] + 2]
    To = board_T(d['bx'], b) @ np.linalg.inv(d['Tcb'])
    dz0 = (To[2, 3] - cam(d['q'], None)[2, 3]) * 1000
    dz1 = (To[2, 3] - cam(d['q'], d['sel'])[2, 3]) * 1000
    ok = d['sel'] is not None and abs(dz1) <= 5.0
    ok_all &= ok
    print(f"{d['img']} | {d['pose']} | {d['sel'] or '없음'} | {dz0:+.1f} | {dz1:+.1f} {'통과' if ok else '불통과'}")
print('판정:', '통과(모든 장 보정 걸림 · |보정 뒤 높이 차| ≤ 5 mm)' if ok_all else '불통과')
