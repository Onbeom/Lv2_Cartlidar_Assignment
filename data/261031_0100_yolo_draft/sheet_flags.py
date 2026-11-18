"""check_labels_result.json 의 의심 상자를 확대해 한 장 그림으로(사용자 눈 확인용). 이웃 장과 클래스가 바뀐 경우 두 장을 나란히.
사용: ~/yolo-venv/bin/python sheet_flags.py <라벨 폴더> <출력 png>"""
import json
import re
import sys
from pathlib import Path

import cv2
import numpy as np

D, OUTP = Path(sys.argv[1]), Path(sys.argv[2])
NAMES = (D / 'classes.txt').read_text().split()
SHORT = {'red_cylinder': 'RC', 'green_cylinder': 'GC', 'red_cuboid': 'RB', 'green_cuboid': 'GB'}
COL = {'red_cylinder': (0, 0, 255), 'red_cuboid': (0, 140, 255), 'green_cylinder': (0, 255, 0), 'green_cuboid': (255, 255, 0)}
res = json.loads((D.parent / 'check_labels_result.json').read_text())
imgs = {p.name.split('__')[1][:3]: p for p in (D / 'images').glob('*.png')}


def load(num):
    p = imgs[num]
    im = cv2.imread(str(p))
    h, w = im.shape[:2]
    bs = []
    for ln in (D / 'labels' / (p.stem + '.txt')).read_text().split('\n'):
        if ln.strip():
            k, x, y, bw, bh = ln.split()
            bs.append((NAMES[int(k)], (float(x) - float(bw) / 2) * w, (float(y) - float(bh) / 2) * h, (float(x) + float(bw) / 2) * w, (float(y) + float(bh) / 2) * h))
    return im, bs


def crop(num, cx, cy, tag):
    im, bs = load(num)
    im = cv2.convertScaleAbs(im, alpha=2.2, beta=10)       # 어두운 사진이라 화면만 밝게(파일은 그대로)
    for j, (n, x0, y0, x1, y1) in enumerate(bs):
        cv2.rectangle(im, (int(x0), int(y0)), (int(x1), int(y1)), COL[n], 1)
        cv2.putText(im, f'{j + 1}{SHORT[n]}', (int(x0), int(y0) - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.32, COL[n], 1)
    h, w = im.shape[:2]
    r = 55
    x0, y0 = int(np.clip(cx - r, 0, w - 2 * r)), int(np.clip(cy - r, 0, h - 2 * r))
    c = cv2.resize(im[y0:y0 + 2 * r, x0:x0 + 2 * r], (330, 330), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(c, (0, 0), (329, 22), (0, 0, 0), -1)
    cv2.putText(c, tag, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return c


tiles = []
for name, whys in res['flags'].items():
    num = name.split('__')[1][:3]
    im, bs = load(num)
    for why in whys:
        m = re.match(r'줄(\d+)', why)
        if not m:
            continue
        b = bs[int(m.group(1)) - 1]
        cx, cy = (b[1] + b[3]) / 2, (b[2] + b[4]) / 2
        nm = re.search(r'이웃 (\d{3})', why)
        left = crop(num, cx, cy, f'{name.split("__")[0][:3]} {num} line{m.group(1)}')
        right = crop(nm.group(1), cx, cy, f'neighbor {nm.group(1)}') if nm else np.zeros_like(left)
        tiles.append(np.hstack([left, np.full((330, 4, 3), 255, np.uint8), right]))
cols = 3
rows = [np.hstack(tiles[i:i + cols] + [np.zeros_like(tiles[0])] * (cols - len(tiles[i:i + cols]))) for i in range(0, len(tiles), cols)]
sheet = np.vstack([np.vstack([r, np.full((6, r.shape[1], 3), 255, np.uint8)]) for r in rows])
cv2.imwrite(str(OUTP), sheet)
print(len(tiles), '칸 →', OUTP, sheet.shape)
