#!/usr/bin/env python3
"""모니터용 보정 화면 (바닥에 눕힌 모니터에 전체 화면으로 띄움). gen_screen.py <가로 px> <세로 px> [출력 접두어]
tag36h11 ID 20 ~ 27 을 2 줄 x 4 장(세로가 짧으면 2 x 3)으로 화면 꽉 차게 놓는다. 태그 자리(px)는 <접두어>.json 에 적는다.
실측: 화면에 뜬 태그 하나의 검은 변 [mm] 와 그 태그의 검은 변 px(json 'black_px') 로 mm/px 를 낸다."""
import json, sys
import cv2, numpy as np

W, H = int(sys.argv[1]), int(sys.argv[2])
pre = sys.argv[3] if len(sys.argv) > 3 else f'screen_{W}x{H}'
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
ncol = 4 if W / H >= 1.5 else 3
nrow = 2
cell_w, cell_h = W / ncol, H / nrow
black = int(min(cell_w, cell_h) * 0.72) // 8 * 8          # 흰 틈 최소 14 % 씩
img = np.full((H, W), 255, np.uint8)
tags = []
for r in range(nrow):
    for c in range(ncol):
        tid = 20 + r * ncol + c
        m = cv2.aruco.drawMarker(d, tid, 8) if not hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.generateImageMarker(d, tid, 8)
        m = cv2.resize(m, (black, black), interpolation=cv2.INTER_NEAREST)
        cx, cy = int(round((c + 0.5) * cell_w)), int(round((r + 0.5) * cell_h))
        x0, y0 = cx - black // 2, cy - black // 2
        img[y0:y0 + black, x0:x0 + black] = m
        # 꼭짓점 (aruco 순서: 왼위 · 오른위 · 오른아래 · 왼아래), 화소 경계 기준
        tags.append({'id': tid, 'corners_px': [[x0, y0], [x0 + black, y0], [x0 + black, y0 + black], [x0, y0 + black]]})
cv2.imwrite(pre + '.png', img)
json.dump({'width': W, 'height': H, 'black_px': black, 'tags': tags}, open(pre + '.json', 'w'), indent=1)
print(pre + '.png', f'{ncol}x{nrow}', 'black_px', black)
