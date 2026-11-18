"""초안 상자 그린 묶음별 표본 모음(묶음마다 처음 · 가운데 · 끝 3장). 사용: sheet.py <라벨폴더> <출력png>"""
import json, sys
from collections import defaultdict
from pathlib import Path
import cv2, numpy as np
d = Path(sys.argv[1]); L = json.load(open(d / 'auto_label.json'))
COL = [(0, 0, 255), (0, 200, 0), (0, 0, 160), (0, 120, 0)]; AB = ['RC', 'GC', 'RB', 'GB']
g = defaultdict(list)
for e in L: g[e['batch']].append(e)
rows = []
for b, es in g.items():
    tiles = []
    for e in (es[0], es[len(es) // 2], es[-1]):
        im = cv2.imread(str(d / 'images' / e['image'])); h, w = im.shape[:2]
        for ln, s in zip((d / 'labels' / (Path(e['image']).stem + '.txt')).read_text().split('\n'), e['conf']):
            k, x, y, bw, bh = map(float, ln.split()); k = int(k)
            p0 = (int((x - bw / 2) * w), int((y - bh / 2) * h)); p1 = (int((x + bw / 2) * w), int((y + bh / 2) * h))
            cv2.rectangle(im, p0, p1, COL[k], max(2, w // 320)); cv2.putText(im, f'{AB[k]} {s:.2f}', (p0[0], max(15, p0[1] - 4)), 0, w / 900, COL[k], max(1, w // 640))
        im = cv2.resize(im, (480, int(480 * h / w)))
        cv2.putText(im, e['image'][:14], (5, 22), 0, .7, (255, 255, 255), 2)
        tiles.append(im)
    rows.append(np.hstack(tiles))
cv2.imwrite(sys.argv[2], np.vstack(rows))
