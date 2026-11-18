"""팔 영상 E3 @0.6 틀린 장 전부: 흰색 = 사람 라벨, 노랑 = E3 0.6 이상. 그림만 만든다."""
import json, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent.parent
N = ['Rcyl', 'Gcyl', 'Rcub', 'Gcub']
r = json.load(open(HERE / 'review/errors_E3.json'))['final_arm']; D = json.load(open(HERE / 'dets_E3.json'))
imgs = sorted({x['img'] for x in r['miss'] + r['fp']})
tiles = []
for im in imgs:
    a = cv2.imread(str(HERE / 'final_arm/images' / im)); H, W = a.shape[:2]
    for l in (HERE / 'final_arm/labels' / (im[:-4] + '.txt')).read_text().split('\n'):
        if l.strip():
            k, x, y, w, h = l.split(); x, y, w, h = float(x) * W, float(y) * H, float(w) * W, float(h) * H
            p = (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2))
            cv2.rectangle(a, *p, (255, 255, 255), 3); cv2.putText(a, N[int(k)], (p[0][0] + 4, max(30, p[0][1] + 34)), 0, 1.1, (255, 255, 255), 3, cv2.LINE_AA)
    for c, s, *b in D['final_arm/' + im]:
        if s >= .6:
            b = [int(v) for v in b]; cv2.rectangle(a, b[:2], b[2:], (0, 255, 255), 3); cv2.putText(a, f'{N[int(c)]} {s:.2f}', (b[0] + 4, min(H - 8, b[3] - 8)), 0, 1.1, (0, 255, 255), 3, cv2.LINE_AA)
    a = cv2.resize(a, (640, 360), interpolation=cv2.INTER_AREA)
    cv2.rectangle(a, (0, 0), (150, 22), (0, 0, 0), -1); cv2.putText(a, im[10:21], (3, 16), 0, .5, (255, 255, 255), 1, cv2.LINE_AA)
    tiles.append(a)
print(len(tiles), imgs)
n = 3; page = 12
for p in range(0, len(tiles), page):
    t = tiles[p:p + page]
    while len(t) % n: t.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(HERE / f'review/arm_err_{p // page}.jpg'), np.vstack([np.hstack(t[i:i + n]) for i in range(0, len(t), n)]), [cv2.IMWRITE_JPEG_QUALITY, 85])
