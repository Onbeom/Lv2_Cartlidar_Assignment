#!/usr/bin/env python3
"""tag05 · tag06 장에 같이 찍힌 바코드 라벨 자리를 바닥색으로 덮는다 (PC, 사용자 결정 11-06). 크기를 1280×720 그대로 둬 태그 검출의 축소 비율을 바꾸지 않는다.
실행: python3 mask_labels.py <출력 폴더>   → <출력>/images.txt (한 줄: 이름 경로)"""
import glob
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
out = sys.argv[1]
X0, Y1 = 930, 290      # 라벨이 들어온 범위 (최소 x 988, 최대 y 247) 보다 넉넉히
lines = []
for pre in ('tag05', 'tag06'):
    cl = sorted(glob.glob(f'{HERE}/clips/{pre}_10cm_*'))[0]
    n = os.path.basename(cl)
    os.makedirs(f'{out}/{n}', exist_ok=True)
    for p in sorted(glob.glob(f'{cl}/*.png')):
        im = cv2.imread(p)
        im[:Y1, X0:] = np.median(im[Y1:Y1 + 60, X0:], axis=(0, 1))
        q = f'{out}/{n}/{os.path.basename(p)}'
        cv2.imwrite(q, im)
        lines.append(f'{n}/{os.path.basename(p)[:-4]} {q}')
open(f'{out}/images.txt', 'w').write('\n'.join(lines) + '\n')
print(len(lines), '장')
