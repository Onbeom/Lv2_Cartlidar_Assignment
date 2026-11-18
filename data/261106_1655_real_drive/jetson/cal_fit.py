#!/usr/bin/env python3
"""TAG 높이 오차 원인 가르기 풀이 (PC, 11-14 검토 승인 판정).
장마다 보드 PnP(게시 K, 거꾸로 장착 = 광학 z 둘레 180°)로 '실제' 카메라 자세(base 기준)를 내고, 모델 M1 · M2 · M3 로 FK 를 고쳐 맞춤.
  보드: 책상 위 높이 75 mm · 수평 고정, x · y · 요만 풂 — 기록 폴더(촬영 묶음)마다 따로 (노트북을 다시 놓음) (기울기 방향을 안 적어 수평으로 둠 — 한계).
  M1 카메라 장착: link5 → 카메라 피치 · x · z 보정 / M2 관절 2 · 3 · 4 영점 / M3 둘 다.
판정: 모든 장 위치 잔차 ≤ 5 mm · 높이 ≤ 3 mm · 방향 ≤ 0.5°, 한 자세 빼고 맞춘 뒤 그 자세 높이 예측 ≤ 5 mm, |ρ| ≤ 0.9 · 조건수 ≤ 100.
cal_fit.py <control URDF> <intr yaml> <기록 폴더 …>
"""
import json
import math
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares

import handeye_lib as L
from handeye_lib import Chain, T_of, rpy_to_R

urdf, intr, dirs = sys.argv[1], sys.argv[2], sys.argv[3:]
ch5 = Chain(open(urdf).read(), base='base_link', tip='link5')
X0 = ch5.design('link5', 'arm_camera_optical_frame')   # link5 → 광학 (설계값)
y = yaml.safe_load(open(intr))
K = np.array(y['camera_matrix']['data']).reshape(3, 3)
D = np.array(y['distortion_coefficients']['data'])
RZ = np.diag([-1.0, -1.0, 1.0, 1.0])
SQ, SY = 0.04917, 0.04875
FLOOR, SCREEN_H = -0.00675, 0.075

data = []
for si, d in enumerate(dirs):
    for l in open(d + '/caps.jsonl'):
        c = json.loads(l)
        T, rms = L.board_pose(np.array(c['corners']), c['ids'], K, D, SQ, squares_x=6, square_y_m=SY)
        if T is None or rms > 1.0:
            continue
        data.append(dict(sess=si, pose=c['pose'], q=np.array(c['q'][:5]), Tcb=RZ @ T, rms=rms))   # 광학(게시) ← 보드
poses = sorted({d['pose'] for d in data})
print(f'장 {len(data)}, 자세 {len(poses)}: ' + ', '.join(f"{p}({sum(d['pose'] == p for d in data)})" for p in poses))


def board_T(b):
    """보드(모서리 원점, x 오른쪽 · y 아래 · z 화면 안) → base. b = (x, y, yaw) 는 보드 가운데."""
    R = rpy_to_R(math.pi, 0.0, b[2])   # 화면이 위를 봄 (보드 z = 아래)
    ctr = np.array([b[0], b[1], FLOOR + SCREEN_H])
    return T_of(R, ctr - R @ np.array([3 * SQ, 2 * SY, 0.0]))


def cam_pred(q, p, model):
    dq = np.zeros(5)
    Xc = X0
    if model in ('M2', 'M3'):
        dq[1:4] = p[0:3]
    if model in ('M1', 'M3'):
        k = 3 if model == 'M3' else 0
        Xc = T_of(rpy_to_R(0.0, p[k], 0.0), [p[k + 1], 0.0, p[k + 2]]) @ X0
    return ch5.fk({f'joint{i + 1}': q[i] + dq[i] for i in range(5)}) @ Xc


NP = {'M1': 3, 'M2': 3, 'M3': 6}


def resid(x, model, sel):
    p = x[:NP[model]]
    r = []
    for d in sel:
        Tb = board_T(x[NP[model] + 3 * d['sess']:NP[model] + 3 * d['sess'] + 3])   # 판(촬영 묶음)마다 노트북을 다시 놓으므로 보드 자리는 판마다 따로 (11-14)
        Tobs = Tb @ np.linalg.inv(d['Tcb'])            # base ← 광학 (관측)
        Tp = cam_pred(d['q'], p, model)
        r += list((Tp[:3, 3] - Tobs[:3, 3]) * 1000.0)   # mm
        dR = Tp[:3, :3].T @ Tobs[:3, :3]
        r += list(np.degrees(np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2) * 10.0)   # 0.1° = 1 mm 무게
    return np.array(r)


def fit(model, sel):
    x0 = np.zeros(NP[model] + 3 * len(dirs))
    for k in range(len(dirs)):
        x0[NP[model] + 3 * k:NP[model] + 3 * k + 3] = [0.52, -0.03, math.pi / 2]
    return least_squares(resid, x0, args=(model, sel))


def report(model):
    s = fit(model, data)
    p = s.x[:NP[model]]
    r = s.fun.reshape(-1, 6)
    pos = np.linalg.norm(r[:, :3], axis=1)
    hz = np.abs(r[:, 2])
    ang = np.linalg.norm(r[:, 3:], axis=1) / 10.0
    J = s.jac
    cov = np.linalg.pinv(J.T @ J)
    sd = np.sqrt(np.diag(cov))
    corr = cov / np.outer(sd, sd)
    np_ = NP[model]
    cpar = corr[:np_, :np_]
    rho = np.max(np.abs(cpar - np.eye(np_))) if np_ > 1 else 0.0
    cond = np.linalg.cond(J[:, :np_])
    # 한 자세 빼고 맞춘 뒤 그 자세 높이 예측
    loo = []
    for pz in poses:
        tr = [d for d in data if d['pose'] != pz]
        te = [d for d in data if d['pose'] == pz]
        s2 = fit(model, tr)
        rr = resid(s2.x, model, te).reshape(-1, 6)
        loo.append((pz, float(np.max(np.abs(rr[:, 2])))))
    names = {'M1': ['카메라 피치 °', '카메라 x mm', '카메라 z mm'], 'M2': ['관절 2 영점 °', '관절 3 영점 °', '관절 4 영점 °'],
             'M3': ['관절 2 영점 °', '관절 3 영점 °', '관절 4 영점 °', '카메라 피치 °', '카메라 x mm', '카메라 z mm']}[model]
    vals = [math.degrees(v) if '°' in n else v * 1000 for n, v in zip(names, p)]
    ok = pos.max() <= 5 and hz.max() <= 3 and ang.max() <= 0.5 and max(v for _, v in loo) <= 5
    sep = rho <= 0.9 and cond <= 100
    print(f'\n== {model}: ' + ' · '.join(f'{n} {v:+.2f}' for n, v in zip(names, vals)) +
          ' | 보드 ' + ' / '.join(f'x {s.x[np_ + 3 * k] * 1000:.0f} y {s.x[np_ + 3 * k + 1] * 1000:.0f} 요 {math.degrees(s.x[np_ + 3 * k + 2]):.1f}°' for k in range(len(dirs))))
    print(f'   잔차 최대: 위치 {pos.max():.1f} mm · 높이 {hz.max():.1f} mm · 방향 {ang.max():.2f}° (기준 5 · 3 · 0.5)')
    print('   한 자세 빼고 높이 예측 최대 mm: ' + ' · '.join(f'{p_} {v:.1f}' for p_, v in loo) + ' (기준 5)')
    print(f'   파라미터 상관 최대 |ρ| {rho:.2f} (≤ 0.9) · 조건수 {cond:.0f} (≤ 100)')
    print(f'   → 설명 {"함" if ok else "못 함"}, 갈라짐 {"됨" if sep else "안 됨"}')
    return ok, sep


res = {m: report(m) for m in ('M1', 'M2', 'M3')}
pick = next((m for m in ('M1', 'M2', 'M3') if res[m][0]), None)
print('\n판정: ' + (f'{pick} (기준을 만족하는 가장 단순한 모델)' + ('' if res[pick][1] else ' — 단 파라미터가 갈라지지 않음(합만 정해짐)') if pick
                   else 'M1~M3 모두 만족 못 함 → 아무것도 고치지 않고 모델별 잔차 · 상관만 보고'))
