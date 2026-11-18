#!/usr/bin/env python3
"""촬영 자세의 카메라 렌즈 자리 예상 (PC, 11-14 직접 측정 전 '예상' 열용). 자세마다 ① 설계 FK ② 사진(PnP, 보드 높이만 씀 — 높이는 보드 x 와 무관)
③ 사진 + 실측 보드 x 로 낸 렌즈 앞뒤 자리. 높이 = 책상(바닥 z −6.75 mm) 위, 앞뒤 = 1 층 앞 끝(132.8 mm) 에서.
사람은 렌즈 앞면 가운데를 재므로 광학 중심을 광축(보는 방향)으로 d 만큼 옮긴 점과 비교 — d 를 모르므로 0 과 상한 20 mm 둘 다 (11-14 검토).
  cam_pos_pred.py <URDF> <intr> <보드 x mm> <기록 폴더 …>"""
import json, math, sys
import numpy as np, yaml
import handeye_lib as L
from handeye_lib import Chain, T_of, rpy_to_R
urdf, intr, BX, dirs = sys.argv[1], sys.argv[2], float(sys.argv[3]) / 1000, sys.argv[4:]
ch = Chain(open(urdf).read(), base='base_link', tip='link5')
X0 = ch.design('link5', 'arm_camera_optical_frame')
y = yaml.safe_load(open(intr)); K = np.array(y['camera_matrix']['data']).reshape(3, 3); D = np.array(y['distortion_coefficients']['data'])
RZ = np.diag([-1.0, -1.0, 1.0, 1.0]); SQ, SY = 0.04917, 0.04875; FLOOR, SCREEN_H, FRONT = -0.00675, 0.075, 0.1328
rows = {}
for d in dirs:
    for l in open(d + '/caps.jsonl'):
        c = json.loads(l)
        if not c['pose'].startswith('BOXLOOK'):
            continue
        T, rms = L.board_pose(np.array(c['corners']), c['ids'], K, D, SQ, squares_x=6, square_y_m=SY)
        if T is None or rms > 1.0:
            continue
        Tbc = np.linalg.inv(RZ @ T)            # 보드 ← 광학
        Rb = rpy_to_R(math.pi, 0.0, math.pi / 2)   # 보드 방향(수평, 요 90°) — 높이에는 롤 · 피치만 영향
        cam_in_b = Rb @ Tbc[:3, 3] + Rb @ (-np.array([3 * SQ, 2 * SY, 0.0]))   # 보드 가운데 기준, base 축
        ax_pnp = (Rb @ Tbc[:3, :3])[:, 2]   # 광축(보는 방향), base 축
        Tf = ch.fk({f'joint{i + 1}': c['q'][i] for i in range(5)}) @ X0
        for dd in (0.0, 0.020):
            z_pnp = FLOOR + SCREEN_H + cam_in_b[2] + dd * ax_pnp[2]
            x_pnp = BX + cam_in_b[0] + dd * ax_pnp[0]
            fk = Tf[:3, 3] + dd * Tf[:3, 2]
            rows.setdefault((c['pose'], dd), []).append((fk, z_pnp, x_pnp))
for (p, dd), v in sorted(rows.items()):
    fk = np.mean([r[0] for r in v], 0); z = np.mean([r[1] for r in v]); x = np.mean([r[2] for r in v])
    print(f'{p} d {dd * 1000:.0f} mm ({len(v)} 장): 렌즈 높이(책상 위) 설계 FK {(fk[2] - FLOOR) * 1000:.0f} mm · 사진 {(z - FLOOR) * 1000:.0f} mm | '
          f'앞 끝에서 앞뒤 설계 FK {(fk[0] - FRONT) * 1000:.0f} mm · 사진 + 실측 보드 {(x - FRONT) * 1000:.0f} mm')
