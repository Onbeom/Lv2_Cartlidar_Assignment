#!/usr/bin/env python3
"""TAG 원인 가르기 자세 (fk_poses_tagfit.yaml) 사전 확인 (PC, 11-14 검토 요청):
  ① 닿음 여유: 자세마다 · ALIGN ↔ 자세 길 (올 때 관절 3,2,1,4,5 · 갈 때 5,4,1,2,3 순서로 한 관절씩 보간) 에서
     팔 (link2 → link5 → 집게 끝 tool_link, 카메라) 과 노트북 · 상자 · 바닥 · 팔레트 칸 사이 최소 거리.
  ② 관절 2 · 3 중력 토크 최대: 오늘 실물로 다닌 자세 · 길의 최대와 비교.
tagfit_check.py <control URDF (xacro 펼친 것)> <질량 URDF waffle_4wd_loaded.urdf> <자세 yaml> [--laptop_x 0.27]
가정 (값은 아래 상수): 팔레트 칸 거리는 어깨(관절 2 축) 6 cm 안 점을 뺌 (어깨는 늘 칸 옆, ALIGN 도 0.8 cm 로 나옴), 팔 링크 반지름 2.5 cm · 집게 · 카메라 반지름 3 cm (중심선에서 뺌), 노트북 + 7 cm 상자 = 직육면체
  x 앞 끝 + laptop_x ~ + 0.80 m · y ±0.20 m · 바닥 ~ 9 cm, 팔레트 칸 = 퍽 자리 (질량 URDF puck_*) 둘레 반지름 4 cm · 퍽 위 3 cm 까지 원기둥.
관절각: 닿음은 펌웨어각 그대로 (robot_state_publisher 와 같음, PLACE 높이 실측과 +8 mm). --with_offset 이면 펌웨어 − ik.joint_offsets 도.
토크는 j3_torque.py 와 같은 방식 (질량 URDF, 펌웨어 − offset).
"""
import argparse
import io
import contextlib
import runpy
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from handeye_lib import Chain

ap = argparse.ArgumentParser()
ap.add_argument('ctrl_urdf')
ap.add_argument('mass_urdf')
ap.add_argument('poses')
ap.add_argument('--with_offset', action='store_true', help='펌웨어 − ik.joint_offsets 도 같이 계산 (기본: 펌웨어각만 — PLACE 실측 높이와 +8 mm 로 맞음, offset 을 빼면 −19 mm)')
ap.add_argument('--laptop_x', type=float, default=0.27, help='노트북 화면 가까운 끝 ← 앞 끝 [m] (가장 가까운 경우)')
a = ap.parse_args()

FRONT = 0.1328          # robot_geometry base.front_offset
FLOOR = -0.00675        # base_link 기준 바닥
R_LINK, R_TIP = 0.025, 0.03
OFF = [-0.0283, -0.4788, 0.1641, 0.0955, 0.0]
MODES = (False, True) if a.with_offset else (False,)
LAP = (FRONT + a.laptop_x, FRONT + 0.80, -0.20, 0.20, FLOOR, FLOOR + 0.09)

ctrl = open(a.ctrl_urdf).read()
tips = ['link2', 'link3', 'link4', 'link5', 'tool_link', 'arm_camera_optical_frame']
chains = {t: Chain(ctrl, base='base_link', tip=t) for t in tips}

# 팔레트 칸: 질량 URDF 의 퍽 자리 (base_link 기준)
mr = ET.parse(a.mass_urdf).getroot()
sys.argv = ['j3_torque.py', a.mass_urdf]
with contextlib.redirect_stdout(io.StringIO()):
    G = runpy.run_path('j3_torque.py', run_name='__main__')
T0 = G['fk_all']({})
cups = [T0[l.get('name')][:3, 3] for l in mr.findall('link') if l.get('name').startswith('puck_')]


def pts(fw):
    """팔 중심선 점 (점, 반지름)"""
    q = {f'joint{k + 1}': fw[k] for k in range(5)}
    P = [chains[t].fk(q)[:3, 3] for t in tips]
    out = []
    for i in range(len(P) - 2):   # link2 → link3 → link4 → link5 → tool_link
        for s in np.linspace(0, 1, 8):
            out.append((P[i] + s * (P[i + 1] - P[i]), R_LINK))
    for s in np.linspace(0, 1, 6):
        out.append((P[3] + s * (P[4] - P[3]), R_TIP))
    out.append((P[5], R_TIP))
    return out


def box_dist(p, b):
    d = np.maximum(np.maximum([b[0] - p[0], b[2] - p[1], b[4] - p[2]], 0), [p[0] - b[1], p[1] - b[3], p[2] - b[5]])
    return float(np.linalg.norm(d))


def clear(fw):
    r = {}
    for model_off in MODES:
        q = [fw[k] - OFF[k] for k in range(5)] if model_off else fw
        shoulder = chains['link2'].fk({f'joint{k + 1}': q[k] for k in range(5)})[:3, 3]
        for p, rad in pts(q):
            near_shoulder = np.linalg.norm(p - shoulder) < 0.06   # 어깨 (관절 2 축) 둘레는 자세와 상관없이 칸 옆에 붙어 있음 → 칸 거리에서 뺌
            r['노트북·상자'] = min(r.get('노트북·상자', 9), box_dist(p, LAP) - rad)
            r['바닥'] = min(r.get('바닥', 9), p[2] - FLOOR - rad)
            for c in ([] if near_shoulder else cups):
                dxy = np.hypot(p[0] - c[0], p[1] - c[1]) - 0.04
                dz = p[2] - (c[2] + 0.03)
                d = max(dxy, dz) if (dxy > 0 or dz > 0) else max(dxy, dz)
                r['팔레트 칸'] = min(r.get('팔레트 칸', 9), (np.hypot(max(dxy, 0), max(dz, 0)) if (dxy > 0 and dz > 0) else d) - rad)
    return r


def path(a_, b_, order, n=12):
    q = list(a_)
    seq = [list(q)]
    for j in order:
        for t in np.linspace(0, 1, n):
            q2 = list(q)
            q2[j] = a_[j] + t * (b_[j] - a_[j])
            seq.append(q2)
        q[j] = b_[j]
    return seq


def tau(jn, fw):
    J, fk_all, mass, subtree = G['J'], G['fk_all'], G['mass'], G['subtree']
    qm = {f'joint{k + 1}': fw[k] - OFF[k] for k in range(5)}
    T = fk_all(qm)
    ch = J[jn].find('child').get('link')
    o, ax = T[ch][:3, 3], T[ch][:3, :3] @ np.array([float(v) for v in J[jn].find('axis').get('xyz').split()])
    t = 0.0
    for l in subtree(ch):
        if l in mass:
            m, c = mass[l]
            p = T[l][:3, :3] @ c + T[l][:3, 3]
            t += np.dot(np.cross(p - o, np.array([0, 0, -9.81 * m])), ax)
    return abs(t)


P = G['P']
AL = P['ALIGN']
TAG = [-0.0283, -0.5540, -0.4264, 1.4594, 0.0]
PLACE = [-0.0283, -0.9699, -0.0276, 1.3892, 0.0]
# 오늘 실물로 다닌 자세 · 길 (토크 비교 기준)
seen = []
for nm, b in (('DRIVE', P['DRIVE']), ('PLACE', PLACE), ('TAG', TAG)):
    seen += path(AL, b, [2, 1, 0, 3, 4]) + path(b, AL, [4, 3, 0, 1, 2])
seen += path(P['STOW'], P['DRIVE'], [2, 1, 0, 3, 4]) + path(P['DRIVE'], AL, [2, 1, 0, 3, 4])
ref2 = max(tau('joint2', q) for q in seen)
ref3 = max(tau('joint3', q) for q in seen)
print(f'오늘 다닌 자세 · 길의 최대 중력 토크: 관절 2 {ref2:.3f} · 관절 3 {ref3:.3f} N·m (DRIVE 관절 3 {tau("joint3", P["DRIVE"]):.3f})')
print(f'ALIGN 자체의 팔레트 칸 여유 {clear(AL)["팔레트 칸"] * 100:.1f} cm (오늘 ALIGN 을 실물로 10 번 넘게 지남, 닿음 없음 → 칸 모델이 보수적)')
print(f'노트북 · 상자 가까운 끝 = 앞 끝 + {a.laptop_x:.2f} m, 팔레트 칸 {len(cups)} 개')
print('| 자세 | 노트북·상자 (자세 / 길 최소) | 바닥 | 팔레트 칸 | 관절 2 토크 최대 | 관절 3 토크 최대 | 판정 |')
print('|---|---|---|---|---|---|---|')
EX = yaml.safe_load(open(a.poses))['poses']
for nm in list(EX) + ['PLACE', 'TAG']:
    fw = EX.get(nm) or (PLACE if nm == 'PLACE' else TAG)
    at = clear(fw)
    seq = path(AL, fw, [2, 1, 0, 3, 4]) + path(fw, AL, [4, 3, 0, 1, 2])
    pth = {}
    for q in seq:
        near_al = max(abs(q[k] - AL[k]) for k in range(5)) < 0.05   # ALIGN 둘레는 오늘 실물로 여러 번 다닌 곳 (팔레트 칸 거리는 공통 값이라 뺌)
        for k, v in clear(q).items():
            if k == '팔레트 칸' and near_al:
                continue
            pth[k] = min(pth.get(k, 9), v)
    t2 = max(tau('joint2', q) for q in seq)
    t3 = max(tau('joint3', q) for q in seq)
    ok = min(pth.values()) >= 0.03 and t2 <= ref2 + 1e-6 and t3 <= ref3 + 1e-6
    print(f'| {nm} | {at["노트북·상자"] * 100:.1f} / {pth["노트북·상자"] * 100:.1f} cm | {pth["바닥"] * 100:.1f} cm | '
          f'{pth["팔레트 칸"] * 100:.1f} cm | {t2:.3f} | {t3:.3f} | {"씀" if ok else "뺌"} |')
