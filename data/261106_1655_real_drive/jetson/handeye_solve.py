#!/usr/bin/env python3
"""P2b 손-눈 보정 풀이 (PC · Jetson): arm_handeye.py 기록 → link5 → 팔 카메라 변환, URDF 설계값과 차, 반복성.

handeye_solve.py <기록 폴더> <robot.urdf.xacro> <보드 가로 칸 m> [--square_y 세로 칸 m] [--intr arm_camera_1280x720.yaml] [--max_rms 1.0]

판정 기준(11-11 '실물 주행 1109 2' 조건부 승인): ① 장 ≥ 20 · 재투영 RMS 중앙 ≤ 0.5 px · 상대 회전 > 0.3 rad 인 장 쌍이 있고 그 회전축이 서로 30° 넘게
  벌어진 묶음 ≥ 2 ② 반복성 위치 SD ≤ 3 mm · 회전 최대 ≤ 0.5° ③ 세 방법 서로 차 ≤ 3 mm · 0.5° ④ URDF 와 차 기록(5 mm · 1° 넘으면 새 리비전 제안).
  기록 지표(문턱은 결과 뒤): base → 보드 흩어짐(보드 고정이라 영점 · 링크 · 처짐 같은 계통 오차가 드러남).
  결과는 URDF 설계 사슬 + 실제 관절각 기준 카메라 위치다(관절 영점 오차 · 링크 길이 차가 섞임).

출력:
  - 장마다 보드 재투영 RMS(px), 쓴 장 수
  - 방법 3 가지(TSAI · PARK · DANIILIDIS) 결과와 서로 차
  - 반복성: 장을 무작위로 반씩 나눠 50 번 다시 풀었을 때 PARK 결과 흩어짐(위치 · 회전 표준편차)
  - URDF 설계(link5 → arm_camera_optical_frame)와 차: link5 기준 xyz [mm] · 회전 [°]
  - URDF 에 넣을 값: arm_camera_link 의 parent=link5 xyz · rpy (광학 프레임 회전은 그대로)
"""
import argparse
import json
import os

import numpy as np
import yaml

from handeye_lib import Chain, urdf_from_xacro, RZ_PI, board_pose, solve, consistency, rot_deg, R_to_rpy, diversity

ap = argparse.ArgumentParser()
ap.add_argument('dir')
ap.add_argument('xacro')
ap.add_argument('square_m', type=float)
ap.add_argument('--square_y', type=float, default=None, help='세로 칸 크기 m (재 보니 가로와 다르면)')
ap.add_argument('--intr', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'arm_camera_1280x720.yaml'))
ap.add_argument('--max_rms', type=float, default=1.0)
a = ap.parse_args()

info = yaml.safe_load(open(a.intr))
K = np.array(info['camera_matrix']['data']).reshape(3, 3)
D = np.array(info['distortion_coefficients']['data'])
ch = Chain(urdf_from_xacro(a.xacro))
caps = [json.loads(l) for l in open(os.path.join(a.dir, 'caps.jsonl'))]
TL, TC, rms_all, QS = [], [], [], []
for c in caps:
    if len(c.get('ids', [])) < 6:   # 보드가 안 보인 장(--any 로 저장한 자세 사진 등)은 건너뜀
        continue
    sx = int(c.get('board', '10x7').lower().split('x')[0])   # 찍을 때 쓴 보드 (caps.jsonl 에 기록, 옛 기록은 10x7)
    T, rms = board_pose(np.array(c['corners']), c['ids'], K, D, a.square_m, squares_x=sx, square_y_m=a.square_y)
    rms_all.append(rms)
    if T is None or rms > a.max_rms:
        continue
    TL.append(ch.fk({f'joint{k + 1}': c['q'][k] for k in range(5)}))
    QS.append(c['q'][:5])
    TC.append(RZ_PI @ T)   # 돌리기 전 광학 → 발행 광학(arm_camera_optical_frame)
print(f'장 {len(caps)}, 씀 {len(TL)} (재투영 RMS ≤ {a.max_rms} px), RMS 중앙 {np.median(rms_all):.3f} 최대 {max(rms_all):.3f} px')
if len(TL) < 3:
    raise SystemExit('장이 너무 적음')
QS = np.array(QS)
print('관절 범위 [rad]: ' + ' · '.join(f'J{k + 1} {QS[:, k].min():+.2f}~{QS[:, k].max():+.2f} ({np.ptp(QS[:, k]):.2f})' for k in range(5)))
npair, nall, ngroup, _ = diversity(TL)
div_ok = ngroup >= 2 and npair > 0
print(f'자세 다양성: 상대 회전 > 0.3 rad 장 쌍 {npair}/{nall}, 회전축 묶음(서로 30° 넘게) {ngroup} 개 → {"충분" if div_ok else "부족"} (기준: 묶음 ≥ 2)')
Xd = ch.design('link5', 'arm_camera_optical_frame')
res = {m: solve(TL, TC, m) for m in ('TSAI', 'PARK', 'DANIILIDIS')}
X = res['PARK']
for m, Xm in res.items():
    d = np.linalg.inv(X) @ Xm
    dp, dr, _ = consistency(TL, TC, Xm)
    print(f'  {m:10s} PARK 과 차 {np.linalg.norm(d[:3, 3]) * 1e3:5.2f} mm · {rot_deg(d[:3, :3]):4.2f}° | 보드 흩어짐 위치 중앙 {np.median(dp):5.1f} · 최대 {max(dp):5.1f} mm, 회전 최대 {max(dr):4.2f}°')
rng = np.random.default_rng(0)
xs, rs = [], []
n = len(TL)
for _ in range(50):
    idx = rng.permutation(n)[: max(3, n // 2)]
    Xh = solve([TL[i] for i in idx], [TC[i] for i in idx], 'PARK')
    xs.append(Xh[:3, 3])
    rs.append(rot_deg(X[:3, :3].T @ Xh[:3, :3]))
xs = np.array(xs)
rep_sd = float(np.linalg.norm(xs.std(0)) * 1e3); rep_rot = float(max(rs))
print(f'반복성(반씩 50 번): 위치 표준편차 {np.linalg.norm(xs.std(0)) * 1e3:.2f} mm (축별 {", ".join(f"{v * 1e3:.2f}" for v in xs.std(0))}), 회전 평균 차 {np.mean(rs):.2f}° · 최대 {max(rs):.2f}°')
dp, dr, _ = consistency(TL, TC, X)
print(f'[기록 지표] base → 보드 흩어짐(PARK): 위치 중앙 {np.median(dp):.1f} · p95 {np.percentile(dp, 95):.1f} · 최대 {max(dp):.1f} mm, 회전 중앙 {np.median(dr):.2f} · 최대 {max(dr):.2f}° (문턱은 결과 뒤)')
mdiff = max(max(np.linalg.norm((np.linalg.inv(X) @ Xm)[:3, 3]) * 1e3 for Xm in res.values()), 0)
rdiff = max(rot_deg((np.linalg.inv(X) @ Xm)[:3, :3]) for Xm in res.values())
c1 = len(TL) >= 20 and float(np.median([r for r in rms_all if r is not None])) <= 0.5 and div_ok
c2 = rep_sd <= 3.0 and rep_rot <= 0.5
c3 = mdiff <= 3.0 and rdiff <= 0.5
print(f'판정: ① {"통과" if c1 else "불통과"} (장 {len(TL)}, RMS 중앙, 다양성) ② {"통과" if c2 else "불통과"} (SD {rep_sd:.2f} mm · 회전 최대 {rep_rot:.2f}°) ③ {"통과" if c3 else "불통과"} (방법 차 최대 {mdiff:.2f} mm · {rdiff:.2f}°) → {"통과" if c1 and c2 and c3 else "불통과"}')
d = np.linalg.inv(Xd) @ X
dl = X[:3, 3] - Xd[:3, 3]
print('※ 아래는 URDF 설계 사슬 + 실제 관절각 기준 카메라 위치 (관절 영점 오차 · 링크 길이 · 처짐이 섞임)')
print(f'URDF 설계와 차 (link5 기준): 위치 {np.linalg.norm(dl) * 1e3:.1f} mm (x {dl[0] * 1e3:+.1f} · y {dl[1] * 1e3:+.1f} · z {dl[2] * 1e3:+.1f}), 회전 {rot_deg(d[:3, :3]):.2f}°')
T_cl_opt = ch.design('arm_camera_link', 'arm_camera_optical_frame')
Xl = X @ np.linalg.inv(T_cl_opt)
r, p, y = R_to_rpy(Xl[:3, :3])
Xdl = Xd @ np.linalg.inv(T_cl_opt)
r0, p0, y0 = R_to_rpy(Xdl[:3, :3])
print(f'arm_camera_link (parent link5) 설계: xyz {Xdl[0, 3]:.6f} {Xdl[1, 3]:.6f} {Xdl[2, 3]:.6f} rpy {r0:.6f} {p0:.6f} {y0:.6f}')
print(f'arm_camera_link (parent link5) 실측: xyz {Xl[0, 3]:.6f} {Xl[1, 3]:.6f} {Xl[2, 3]:.6f} rpy {r:.6f} {p:.6f} {y:.6f}')
