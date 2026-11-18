#!/usr/bin/env python3
"""관절 3 중력 토크 계산 (검토 조건 4): URDF 질량 · 질량중심으로 자세 · 이동 경로에서 관절 3 축 둘레 중력 토크 [N·m].
j3_torque.py <urdf(질량 있는 것)> — 모델 관절각(펌웨어 − ik.joint_offsets)으로 계산한다.
"""
import sys, itertools, math, xml.etree.ElementTree as ET
import numpy as np
from handeye_lib import rpy_to_R, axis_angle_R, T_of
r = ET.parse(sys.argv[1]).getroot()
J = {j.get('name'): j for j in r.findall('joint')}
kids = {}
for j in r.findall('joint'):
    kids.setdefault(j.find('parent').get('link'), []).append(j)
mass = {}
for l in r.findall('link'):
    i = l.find('inertial')
    if i is not None:
        o = i.find('origin')
        mass[l.get('name')] = (float(i.find('mass').get('value')), np.array([float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]))
OFF = [-0.0283, -0.4788, 0.1641, 0.0955, 0.0]   # dev manipulator.yaml ik.joint_offsets (펌웨어 = 모델 + offset)


def fk_all(q):
    """q: {'joint1'..'joint5': 모델 rad}. base_link 기준 링크별 변환."""
    T = {'base_link': np.eye(4)}
    stack = ['base_link']
    while stack:
        p = stack.pop()
        for j in kids.get(p, []):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            Tj = T[p] @ T_of(rpy_to_R(*rpy), xyz)
            if j.get('type') in ('revolute', 'continuous') and j.get('name') in q:
                ax = [float(v) for v in j.find('axis').get('xyz').split()]
                Tj = Tj @ T_of(axis_angle_R(ax, q[j.get('name')]), [0, 0, 0])
            c = j.find('child').get('link')
            T[c] = Tj
            stack.append(c)
    return T


def subtree(link):
    out = [link]
    for j in kids.get(link, []):
        out += subtree(j.find('child').get('link'))
    return out


DIST3 = subtree(J['joint3'].find('child').get('link'))
M3 = sum(mass[l][0] for l in DIST3 if l in mass)


def tau3(qm):
    q = {f'joint{k + 1}': qm[k] for k in range(5)}
    T = fk_all(q)
    T3 = T[J['joint3'].find('child').get('link')]
    o3, ax3 = T3[:3, 3], T3[:3, :3] @ np.array([float(v) for v in J['joint3'].find('axis').get('xyz').split()])
    tau = 0.0
    for l in DIST3:
        if l in mass:
            m, c = mass[l]
            p = T[l][:3, :3] @ c + T[l][:3, 3]
            tau += np.dot(np.cross(p - o3, np.array([0, 0, -9.81 * m])), ax3)
    return tau


def model(fw):
    return [fw[k] - OFF[k] for k in range(5)]


P = {'STOW': [1.554, -2.088, 1.549, -1.868, 0.043], 'DRIVE': [-0.0245, -1.6122, 1.2901, 1.1888, 0.0],
     'ALIGN': [0.0, -1.37, -0.01, 1.85, 0.0], 'BOXLOOK_PLACE': [-0.0283, -0.9699, -0.0276, 1.3892, 0.0],
     'BOXLOOK_TAG': [-0.0283, -0.5540, -0.4264, 1.4594, 0.0]}
print(f'관절 3 뒤 질량 합 {M3:.3f} kg (링크 {", ".join(l for l in DIST3 if l in mass)})')
print('자세별 관절 3 중력 토크 (펌웨어각 → 모델각 = 펌웨어 − offset, 참고로 offset 없이도):')
for n, fw in P.items():
    print(f'  {n:14s} {tau3(model(fw)):+.3f} N·m   (offset 없이 {tau3(fw):+.3f})')


def path_max(a, b, order, n=20):
    q = list(a); worst = abs(tau3(model(q)))
    for j in order:
        for t in np.linspace(0, 1, n):
            q2 = list(q); q2[j] = a[j] + t * (b[j] - a[j])
            worst = max(worst, abs(tau3(model(q2))))
        q[j] = b[j]
    return worst


def path_lin(a, b, n=40):
    return max(abs(tau3(model([a[k] + t * (b[k] - a[k]) for k in range(5)]))) for t in np.linspace(0, 1, n))


for a, b in (('ALIGN', 'BOXLOOK_PLACE'), ('BOXLOOK_PLACE', 'BOXLOOK_TAG'), ('BOXLOOK_TAG', 'DRIVE')):
    res = sorted((path_max(P[a], P[b], list(o)), ''.join(str(k + 1) for k in o)) for o in itertools.permutations([0, 1, 2, 3]))
    print(f'{a} → {b}: 한 관절씩 지금 순서(3,2,1,4) 최대 {path_max(P[a], P[b], [2, 1, 0, 3]):.3f} · 가장 작은 순서 {res[0][1]} {res[0][0]:.3f} · 함께 보간 {path_lin(P[a], P[b]):.3f} N·m')
