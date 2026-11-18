#!/usr/bin/env python3
"""R2: 앞 D435 실물 장소 상자 재기 (measure_front 와 같은 place_box, 배율 1.0, 장 중앙값) 를 줄자 정답과 견줌 (PC).
r2_box.py <폴더 R2_*> ... [--yaw_deg 앞 카메라 요 보정 °] [--tire_x 0.1255]
정답 (사람이 놓음, 상자 205 x 205 x 70):
  R2_0a · R2_0b  : β 0°, 가까운 변이 앞바퀴 앞 끝 선과 나란히 d = 0.30 · 0.60 m, 좌우 가운데 → 중심 x = tire_x + d + 0.1025, y 0
  R2_45a · R2_45b: β 45°, 가까운 꼭짓점이 가운데 줄 위 d = 0.30 · 0.55 m, 옆 꼭짓점 둘이 앞바퀴 선에서 같은 거리 → 중심 x = tire_x + d + 0.1450, y 0
사전값 = 정답 + (+0.020, −0.020) m, 사전 요 0 (지도 축). 판정: 중심 ≤ 15 mm · 요(90° 주기) ≤ 4°."""
import argparse, glob, json, math, os, subprocess
import numpy as np
import front_cal as F
HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, 'pb_dev', 'place_box_check')
TRUTH = {'R2_0a': (0.30, 0.0, 0.1025), 'R2_0b': (0.60, 0.0, 0.1025), 'R2_45a': (0.30, 45.0, 0.1450), 'R2_45b': (0.55, 45.0, 0.1450)}
XYZ = [-0.0480903, 0.0250244, 0.4836226]; RPY = [0.0, 0.5692410, 0.0]     # PR #240 고친 피치

ap = argparse.ArgumentParser(); ap.add_argument('folders', nargs='+')
ap.add_argument('--yaw_deg', type=float, default=0.0, help='Y1 로 잰 앞 카메라 요(+ = 실제가 왼쪽). 넣으면 TF 에 반영')
ap.add_argument('--tire_x', type=float, default=0.1255, help='앞바퀴 앞 끝 x (base_link, 축 0.0925 + 반지름 0.033)')
a = ap.parse_args()
rpy = [RPY[0], RPY[1], RPY[2] + math.radians(a.yaw_deg)]
allok = True
for f in a.folders:
    name = os.path.basename(f.rstrip('/'))
    none = name == 'R2_none'
    d, beta, half = TRUTH['R2_0a'] if none else TRUTH[name]; cx, cy = a.tire_x + d + half, 0.0   # 빈 책상: 가까운 배치 자리를 사전값으로
    meta = json.load(open(os.path.join(f, 'meta.json')))
    K = np.array(meta['K']).reshape(3, 3)
    Tn, _ = F.T_bf_opt(XYZ, rpy, meta.get('T_link_optical') or F.DEF_LINK_OPT)
    env = dict(os.environ, PLACE_BOX_SCALES='1.0')
    got = []
    for img in sorted(glob.glob(os.path.join(f, '*.png'))):
        o = subprocess.run([TOOL, img, *map(str, (K[0, 0], K[1, 1], K[0, 2], K[1, 2])), *map(str, Tn.ravel()),
                            str(cx + 0.020), str(cy - 0.020), '0'], capture_output=True, text=True, env=env).stdout.split()
        if len(o) > 4 and o[4] == '1':
            got.append([float(o[0]), float(o[1]), float(o[2])])
    if none:
        okn = len(got) < 3; allok &= okn
        print(f'R2_none: 빈 책상에서 찾음 {len(got)} 장 → {"통과" if okn else "불통과 (틀린 찾음)"} (기준 < 3/10)'); continue
    if len(got) < 3:
        print(f'{name}: 찾음 {len(got)}/10 → 불통과'); allok = False; continue
    g = np.array(got); x, y = np.median(g[:, 0]), np.median(g[:, 1])
    yaw = math.degrees(np.median(np.mod(g[:, 2] + math.pi / 4, math.pi / 2) - math.pi / 4))
    dc = math.hypot(x - cx, y - cy) * 1e3; dyaw = abs((yaw - beta + 45) % 90 - 45)
    ok = dc <= 15 and dyaw <= 4; allok &= ok
    print(f'{name}: 찾음 {len(got)}/{len(list(glob.glob(os.path.join(f, "*.png"))))}, 잰 중심 ({x:.3f}, {y:+.3f}) 정답 ({cx:.3f}, 0) → 중심 {dc:.1f} mm '
          f'(앞뒤 {(x - cx) * 1e3:+.1f} · 옆 {(y - cy) * 1e3:+.1f}), 요 {yaw:+.1f}° 정답 {beta:.0f}° → {dyaw:.1f}° | {"통과" if ok else "불통과"}')
print('R2 판정:', '통과' if allok else '불통과', '(중심 ≤ 15 mm · 요 ≤ 4°, 앞 카메라 요 보정', f'{a.yaw_deg:+.2f}°)')
