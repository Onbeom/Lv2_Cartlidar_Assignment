#!/usr/bin/env python3
"""replay_track · analyze 도구 확인용 합성 녹화(모의). 태그가 가로로 일정 속도로 왕복하는 1280×720 영상과 frames.csv 를 만든다.

make_fake_run.py <폴더> <긴변 px> <장당 px> [태그 수]
"""
import json
import os
import sys

import cv2
import numpy as np

out, side, step = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
count = int(sys.argv[4]) if len(sys.argv) > 4 else 1
os.makedirs(out, exist_ok=True)
aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
span = 300.0
n = 90


def marker(i):
    # 태그 검은 칸 8 칸 + 흰 여백 1 칸씩 → 10 칸. side 는 검은 테두리의 긴 변
    cell = side / 8.0
    m = cv2.resize(cv2.aruco.drawMarker(aruco, i, 8 * 20, borderBits=1), (side, side), interpolation=cv2.INTER_AREA)
    pad = int(round(cell))
    return cv2.copyMakeBorder(m, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


tags = [marker(i) for i in range(count)]
csv = open(os.path.join(out, 'frames.csv'), 'w', encoding='utf-8')
csv.write('frame,t,q,cmd\n')
x, d = 0.0, 1.0
for k in range(n):
    img = np.full((720, 1280, 3), 150, np.uint8)
    for i, m in enumerate(tags):
        cx = int(640 + x + (i - (count - 1) / 2) * (side + 80))
        y0, x0 = 360 - m.shape[0] // 2, cx - m.shape[1] // 2
        img[y0:y0 + m.shape[0], x0:x0 + m.shape[1]] = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(os.path.join(out, f'{k:06d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    # 명령 각도 = 위치에 비례(왕복 끝 말고는 등속)
    csv.write(f'{k},{k / 30.0:.4f},{x / 1000.0:.4f},{x / 1000.0:.4f}\n')
    x += d * step
    if abs(x) > span:
        d = -d
        x += 2 * d * step
csv.close()
json.dump({'speed': step * 30.0 / 1000.0}, open(os.path.join(out, 'run.json'), 'w'))
