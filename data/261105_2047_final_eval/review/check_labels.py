"""라벨 검수(판정 아님): 형식 오류, 겹친 상자(IoU >= 0.9), 안 본 장, 초안 대비 고침 수, 지운 0.6 이상 상자 목록 + 확대 그림."""
import json, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent.parent
NAMES = ['red_cyl', 'green_cyl', 'red_cub', 'green_cub']
def xyxy(p, W=1, H=1):
    k, x, y, w, h = p
    return [(x - w / 2) * W, (y - h / 2) * H, (x + w / 2) * W, (y + h / 2) * H]
def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0])); iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.
crops = []
for cam in ('final_front', 'final_rear', 'final_arm'):
    L = json.load(open(HERE / cam / 'auto_label.json'))
    fmt, dup, unchecked, deleted, changed, added, nb, empty = [], [], [], [], [], [], 0, 0
    for e in L:
        if not e.get('checked'): unchecked.append(e['image'])
        p = HERE / cam / 'labels' / (e['image'][:-4] + '.txt')
        cur = []
        for i, l in enumerate(p.read_text().split('\n')):
            if not l.strip(): continue
            f = l.split()
            try:
                v = [int(f[0])] + [float(x) for x in f[1:]]
                if len(f) != 5 or not 0 <= v[0] <= 3 or not all(0 <= x <= 1 for x in v[1:]) or v[3] <= 0 or v[4] <= 0: raise ValueError
                cur.append(v)
            except Exception:
                fmt.append(f'{e["image"]}:{i}:{l}')
        nb += len(cur); empty += not cur
        for i in range(len(cur)):
            for j in range(i):
                if iou(xyxy(cur[i]), xyxy(cur[j])) >= 0.9: dup.append(e['image'])
        dr = [[float(x) for x in s.split()] for s in e.get('draft', [])]
        conf = e.get('conf', [])
        mcur = set()
        for k, d in enumerate(dr):
            c = conf[k] if k < len(conf) else None
            best = max(((iou(xyxy(d), xyxy(x)), j) for j, x in enumerate(cur)), default=(0, -1))
            if best[0] >= 0.5:
                mcur.add(best[1])
                if int(cur[best[1]][0]) != int(d[0]) and c is not None and c >= 0.6:
                    changed.append((e['image'], c, int(d[0]), int(cur[best[1]][0])))
            elif c is not None and c >= 0.6:
                deleted.append((cam, e['image'], c, d, cur))
        added += [(e['image'], x) for j, x in enumerate(cur) if j not in mcur]
    ed = {k: sum(e.get('edits', {}).get(k, 0) for e in L) for k in ('deleted', 'added', 'class_changed', 'resized')}
    print(f'== {cam}: 장 {len(L)} 안 본 장 {len(unchecked)} 라벨 상자 {nb} 빈 장 {empty} | 형식 오류 {len(fmt)} 겹침 {len(dup)} | 도구 기록 {ed} | 지운 0.6+ {len(deleted)} 클래스 바꾼 0.6+ {len(changed)} 초안에 없던 상자 {len(added)}')
    for x in fmt[:10]: print('  형식', x)
    for x in dup[:10]: print('  겹침', x)
    for x in unchecked[:10]: print('  안 봄', x)
    for x in changed[:60]: print('  클래스', x[0], round(x[1], 3), NAMES[x[2]], '->', NAMES[x[3]])
    for cam_, im, c, d, cur in deleted: print('  지움', im, round(c, 3), NAMES[int(d[0])], [round(v, 3) for v in d[1:]])
    crops += deleted
json.dump([[a, b, c, d] for a, b, c, d, _ in crops], open(HERE / 'review' / 'deleted_06.json', 'w'))
tiles = []
for cam, im, c, d, cur in crops:
    img = cv2.imread(str(HERE / cam / 'images' / im)); H, W = img.shape[:2]
    b = [int(v) for v in xyxy(d, W, H)]
    s = max(b[2] - b[0], b[3] - b[1]); m = max(int(s * 0.9), 60); cx, cy = (b[0] + b[2]) // 2, (b[1] + b[3]) // 2
    x0, y0, x1, y1 = max(0, cx - s // 2 - m), max(0, cy - s // 2 - m), min(W, cx + s // 2 + m), min(H, cy + s // 2 + m)
    v = img.copy()
    for x in cur:
        q = [int(t) for t in xyxy(x, W, H)]
        cv2.rectangle(v, q[:2], q[2:], (0, 220, 0), 1)
    cv2.rectangle(v, b[:2], b[2:], (0, 0, 255), 1)
    t = cv2.resize(v[y0:y1, x0:x1], (240, 240), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(t, (0, 0), (240, 18), (0, 0, 0), -1)
    cv2.putText(t, f'{len(tiles)} {im[6:16]} {NAMES[int(d[0])]} {c:.2f}', (2, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    tiles.append(t)
if tiles:
    n = 8; rows = []
    while len(tiles) % n: tiles.append(np.zeros_like(tiles[0]))
    for i in range(0, len(tiles), n): rows.append(np.hstack(tiles[i:i + n]))
    out = np.vstack(rows)
    cv2.imwrite(str(HERE / 'review' / 'deleted_06_crops.jpg'), out, [cv2.IMWRITE_JPEG_QUALITY, 88]); print('그림', out.shape)
