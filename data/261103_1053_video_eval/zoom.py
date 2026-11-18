"""zoom.py <폴더> <출력png> <이미지 앞부분>... : 상자 묶음 둘레를 원래 화소로 잘라 확대(상자는 1화소 선, 바깥에 표시). 검수용."""
import cv2, sys, numpy as np
from pathlib import Path
d = Path(sys.argv[1]); ts = []
COL = [(0, 0, 255), (0, 200, 0), (0, 0, 160), (0, 120, 0)]; AB = ['RC', 'GC', 'RB', 'GB']
for pre in sys.argv[3:]:
    f = [p for p in sorted((d / 'images').iterdir()) if p.name.startswith(pre)][0]
    im = cv2.imread(str(f)); h, w = im.shape[:2]
    B = [list(map(float, l.split())) for l in (d / 'labels' / (f.stem + '.txt')).read_text().split('\n') if l.strip()]
    R = B or [[0, .5, .5, .3, .3]]
    x0 = min(b[1] - b[3] / 2 for b in R) * w; x1 = max(b[1] + b[3] / 2 for b in R) * w; y0 = min(b[2] - b[4] / 2 for b in R) * h; y1 = max(b[2] + b[4] / 2 for b in R) * h
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2; s = max(x1 - x0, y1 - y0) * 1.3 + 20
    a, b0 = int(max(0, cx - s / 2)), int(max(0, cy - s * 0.375)); c = im[b0:b0 + int(s * .75), a:a + int(s)].copy()
    Z = 480 / c.shape[1]; c = cv2.resize(c, (480, int(c.shape[0] * Z)), interpolation=cv2.INTER_CUBIC)
    for k, x, y, bw, bh in B:
        p0 = (int(((x - bw / 2) * w - a) * Z) - 2, int(((y - bh / 2) * h - b0) * Z) - 2); p1 = (int(((x + bw / 2) * w - a) * Z) + 2, int(((y + bh / 2) * h - b0) * Z) + 2)
        cv2.rectangle(c, p0, p1, COL[int(k)], 1); cv2.putText(c, AB[int(k)], (p0[0], max(12, p0[1] - 3)), 0, .45, COL[int(k)], 1)
    c = cv2.resize(c, (480, 360)); cv2.putText(c, f.name.split('_0')[0] + ' ' + str(len(B)), (5, 18), 0, .55, (255, 255, 0), 2); ts.append(c)
ts += [np.zeros_like(ts[0])] * (-len(ts) % 4)
cv2.imwrite(sys.argv[2], np.vstack([np.hstack(ts[i:i + 4]) for i in range(0, len(ts), 4)]))
