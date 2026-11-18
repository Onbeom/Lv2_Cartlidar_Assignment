"""사용자가 고친 라벨 다시 검수(도구 확인 · 판정 아님, 11-02 15:4x). 라벨 파일은 읽기만 한다.
검사: (1) 형식 · 범위 (2) 한 장 안 겹친 상자(IoU ≥ 0.5) (3) 앞뒤 장과 상자 수 · 클래스가 다른 장(같은 묶음, 촬영 순서)
      (4) 앞뒤 장의 같은 자리 상자(IoU ≥ 0.3)와 클래스가 다른 상자 (5) 초안에서 지운 신뢰도 ≥ 0.6 상자.
사용: review_labels.py <라벨폴더> ... → review/<폴더>.json + review/<폴더>_의심.png(의심 장만, 고친 상자 굵게 · 지운 0.6 이상 초안 얇게)."""
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
OUT = HERE / 'review'
OUT.mkdir(exist_ok=True)
COL = [(0, 0, 255), (0, 200, 0), (0, 0, 160), (0, 120, 0)]
AB = ['RC', 'GC', 'RB', 'GB']


def boxes(lines):
    out = []
    for ln in lines:
        p = ln.split()
        if p:
            k, x, y, w, h = int(p[0]), *map(float, p[1:])
            out.append((k, x - w / 2, y - h / 2, x + w / 2, y + h / 2))
    return out


def iou(a, b):
    ix = max(0.0, min(a[3], b[3]) - max(a[1], b[1])); iy = max(0.0, min(a[4], b[4]) - max(a[2], b[2]))
    u = (a[3] - a[1]) * (a[4] - a[2]) + (b[3] - b[1]) * (b[4] - b[2]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


for d in map(Path, sys.argv[1:]):
    L = json.load(open(d / 'auto_label.json'))
    rep = {'images': len(L), 'boxes': 0, 'class_count': Counter(), 'format': [], 'overlap': [], 'count_jump': [], 'class_flip': [], 'deleted_hi': []}
    final = {}
    for e in L:
        raw = (d / 'labels' / (Path(e['image']).stem + '.txt')).read_text().split('\n')
        for ln in raw:
            p = ln.split()
            if p and (len(p) != 5 or p[0] not in '0123' or not all(0 <= float(v) <= 1 for v in p[1:]) or float(p[3]) <= 0 or float(p[4]) <= 0):
                rep['format'].append([e['image'], ln])
        B = boxes(raw); final[e['image']] = B
        rep['boxes'] += len(B); rep['class_count'].update(AB[b[0]] for b in B)
        for i in range(len(B)):
            for j in range(i + 1, len(B)):
                if iou(B[i], B[j]) >= 0.5:
                    rep['overlap'].append([e['image'], AB[B[i][0]], AB[B[j][0]], round(iou(B[i], B[j]), 2)])
        D = boxes(e['draft'])
        for db, s in zip(D, e['conf']):
            if s >= 0.6 and not any(iou(db, b) >= 0.5 for b in B):
                rep['deleted_hi'].append([e['image'], AB[db[0]], s])
    g = defaultdict(list)
    for e in L:
        g[e['batch']].append(e['image'])
    for b, ims in g.items():
        ims.sort(key=lambda s: s.split('__')[1])
        for i, im in enumerate(ims):
            nb = [ims[j] for j in (i - 1, i + 1) if 0 <= j < len(ims)]
            sig = lambda x: sorted(k for k, *_ in final[x])
            if nb and all(sig(n) != sig(im) for n in nb):
                rep['count_jump'].append([im, ''.join(AB[k][0] + AB[k][1] + ' ' for k in sig(im)).strip(), [len(final[n]) for n in nb]])
            for bx in final[im]:
                for n in nb:
                    m = max(((iou(bx, c), c) for c in final[n]), default=(0, None))
                    if m[0] >= 0.3 and m[1][0] != bx[0]:
                        rep['class_flip'].append([im, AB[bx[0]], n, AB[m[1][0]]]); break
    rep['class_count'] = dict(rep['class_count'])
    sus = sorted({r[0] for k in ('format', 'overlap', 'count_jump', 'class_flip', 'deleted_hi') for r in rep[k]})
    rep['suspect_images'] = sus
    json.dump(rep, open(OUT / f'{d.name}.json', 'w'), ensure_ascii=False, indent=1)
    print(d.name, {k: (len(v) if isinstance(v, list) else v) for k, v in rep.items()})
    # 의심 장 그림(4열, 타일 폭 400)
    E = {e['image']: e for e in L}
    tiles = []
    for im in sus:
        img = cv2.imread(str(d / 'images' / im)); h, w = img.shape[:2]
        e = E[im]
        for db, s in zip(boxes(e['draft']), e['conf']):
            if s >= 0.6 and not any(iou(db, b) >= 0.5 for b in final[im]):
                cv2.rectangle(img, (int(db[1] * w), int(db[2] * h)), (int(db[3] * w), int(db[4] * h)), (255, 255, 255), 1)
                cv2.putText(img, f'del {AB[db[0]]} {s:.2f}', (int(db[1] * w), int(db[4] * h) + 9), 0, 0.3, (255, 255, 255), 1)
        for k, x0, y0, x1, y1 in final[im]:
            cv2.rectangle(img, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)), COL[k], 1)
            cv2.putText(img, AB[k], (int(x0 * w), max(15, int(y0 * h) - 2)), 0, 0.35, COL[k], 1)
        R = [b[1:] for b in final[im]] + [db[1:] for db, s in zip(boxes(e['draft']), e['conf']) if s >= 0.6]
        if R:  # 상자 둘레만 확대(가로 · 세로 3배 여유, 최소 160 화소)
            x0, y0 = min(r[0] for r in R) * w, min(r[1] for r in R) * h; x1, y1 = max(r[2] for r in R) * w, max(r[3] for r in R) * h
            cx, cy, sz = (x0 + x1) / 2, (y0 + y1) / 2, max(160, 1.6 * max(x1 - x0, y1 - y0))
            a, b = int(max(0, cx - sz / 2)), int(max(0, cy - sz * 0.375)); img = img[b:b + int(sz * 0.75), a:a + int(sz)]
        img = cv2.resize(img, (400, 300))
        cv2.putText(img, im.split('_2026')[0], (5, 20), 0, .6, (255, 255, 0), 2)  # 묶음__번호
        tiles.append(img)
    if tiles:
        th = tiles[0].shape[0]
        tiles = [cv2.resize(t, (400, th)) for t in tiles]
        tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 4)
        rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
        for p in range(0, len(rows), 6):  # 한 그림 6줄(24장)
            cv2.imwrite(str(OUT / f'{d.name}_의심_{p // 6 + 1}.png'), np.vstack(rows[p:p + 6]))
