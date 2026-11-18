"""D(0.6) 가 놓친 셈 안 정답을 잘라 보여 줌(정답 = 흰 상자 + 클래스, 0.25 이상 D 검출 = 색 상자 + 점수). 사용: miss_sheet.py <카메라> <출력> [최대 장]"""
import json, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent
cam, outp, nmax = sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 48
src = open(HERE / 'judge.py').read(); src = src[:src.index("side = lambda b")]
sys.argv = ['x']; ns = {'__file__': str(HERE / 'judge.py')}
import contextlib, io
with contextlib.redirect_stdout(io.StringIO()): exec(src, ns)
GT, META, iou = ns['GT'], ns['META'], ns['iou']; D = json.load(open(HERE / 'dets_all_D.json'))
import os
MS = float(os.environ.get('MIN_SHARP', '0'))
AB = ['RC', 'GC', 'RB', 'GB']; COL = [(0, 0, 255), (0, 200, 0), (0, 0, 160), (0, 120, 0)]; tiles = []
side = lambda b: max(b[2] - b[0], b[3] - b[1])
for key, g in sorted(GT.items()):
    if not key.startswith(cam): continue
    m = META[key]; im = None
    if m.get('sharp', 1e9) < MS: continue
    for x in g:
        if cam == 'video_arm':
            s0 = META['s0'][m['batch']]; inn = s0 and side(x['b']) >= s0; near = True
        else:
            inn = x['z'] is not None and x['z'] <= 3.0; near = x['z'] is not None and x['z'] <= 1.0
        ok = lambda d: d[0] == x['c'] if near else d[0] % 2 == x['c'] % 2
        if not inn or any(d[1] >= 0.6 and iou(d[2:], x['b']) >= 0.5 and ok(d) for d in D[key]): continue
        if im is None: im = cv2.imread(str(HERE / cam / 'images' / key.split('/')[1]))
        x0, y0, x1, y1 = x['b']; s = max(x1 - x0, y1 - y0) * 2.2; cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        a, b = int(max(0, cx - s / 2)), int(max(0, cy - s / 2)); c = im.copy()
        cv2.rectangle(c, (int(x0), int(y0)), (int(x1), int(y1)), (255, 255, 255), 1)
        for k, sc, *bb in D[key]:
            if sc >= 0.25 and iou(bb, x['b']) > 0.1:
                cv2.rectangle(c, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), COL[k], 1)
                cv2.putText(c, f'{AB[k]}{sc:.2f}', (int(bb[0]), int(bb[3]) + 10), 0, 0.35, COL[k], 1)
        c = cv2.resize(c[b:b + int(s), a:a + int(s)], (200, 200), interpolation=cv2.INTER_CUBIC)
        z = f" {x['z']:.2f}m" if x.get('z') else ''
        cv2.putText(c, key.split('/')[1].split('_0')[0][-9:] + ' ' + AB[x['c']] + z, (2, 12), 0, 0.38, (255, 255, 0), 1); tiles.append(c)
        if len(tiles) >= nmax: break
    if len(tiles) >= nmax: break
tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 8)
cv2.imwrite(outp, np.vstack([np.hstack(tiles[i:i + 8]) for i in range(0, len(tiles), 8)])); print(len(tiles))
