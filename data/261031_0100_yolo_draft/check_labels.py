"""확정 라벨 자동 점검(도구 확인, 판정 아님 — 맞고 틀림은 사용자가 사진을 보고 정함). 의심 장 목록 · 이유를 낸다.
점검: ① 형식(클래스 0~3, 좌표 0~1) ② 같은 장 상자 겹침(IoU > 0.5) ③ 작은 상자(한 변 < 8 px) ④ 상자 안 색(채도 있는 화소의 빨강 · 초록 비율)이 클래스 색과 반대
⑤ 같은 묶음 앞뒤 장에서 같은 자리(IoU > 0.3) 퍽의 클래스가 바뀜(색 또는 모양) ⑥ 앞뒤 장 모두와 클래스별 개수가 다름.
사용: ~/yolo-venv/bin/python check_labels.py <라벨 폴더>"""
import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

D = Path(sys.argv[1])
NAMES = (D / 'classes.txt').read_text().split()
COLOR = {i: ('red' if n.startswith('red') else 'green') for i, n in enumerate(NAMES)}
log = json.loads((D / 'auto_label.json').read_text())


def boxes(e, w, h):
    out = []
    for ln in (D / 'labels' / (Path(e['image']).stem + '.txt')).read_text().split('\n'):
        if ln.strip():
            k, x, y, bw, bh = ln.split()
            out.append((int(k), float(x), float(y), float(bw), float(bh),
                        (float(x) - float(bw) / 2) * w, (float(y) - float(bh) / 2) * h, (float(x) + float(bw) / 2) * w, (float(y) + float(bh) / 2) * h))
    return out


def iou(a, b):
    ix = max(0, min(a[7], b[7]) - max(a[5], b[5]))
    iy = max(0, min(a[8], b[8]) - max(a[6], b[6]))
    i = ix * iy
    u = (a[7] - a[5]) * (a[8] - a[6]) + (b[7] - b[5]) * (b[8] - b[6]) - i
    return i / u if u > 0 else 0


items = []
for e in log:
    img = cv2.imread(str(D / 'images' / e['image']))
    h, w = img.shape[:2]
    items.append((e, img, boxes(e, w, h)))
flags = {}


def flag(e, why):
    flags.setdefault(e['image'], []).append(why)


hsv_all = {}
for e, img, bs in items:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for j, b in enumerate(bs):
        if not (0 <= b[0] < len(NAMES)) or not all(0 <= v <= 1 for v in b[1:5]):
            flag(e, f'줄{j + 1} 형식')
        if (b[7] - b[5]) < 8 or (b[8] - b[6]) < 8:
            flag(e, f'줄{j + 1} 작은 상자 {b[7] - b[5]:.0f}×{b[8] - b[6]:.0f}px')
        for k in range(j + 1, len(bs)):
            if iou(b, bs[k]) > 0.5:
                flag(e, f'줄{j + 1}·{k + 1} 겹침 IoU {iou(b, bs[k]):.2f}')
        x0, y0, x1, y1 = (max(0, int(round(v))) for v in b[5:9])
        p = hsv[y0:y1, x0:x1].reshape(-1, 3)
        p = p[(p[:, 1] > 60) & (p[:, 2] > 30)]
        if len(p) >= 10:
            red = ((p[:, 0] < 12) | (p[:, 0] > 165)).sum()
            green = ((p[:, 0] > 35) & (p[:, 0] < 90)).sum()
            if COLOR[b[0]] == 'red' and green > 2 * max(red, 1):
                flag(e, f'줄{j + 1} {NAMES[b[0]]} 인데 초록 화소 많음({green} vs 빨강 {red})')
            if COLOR[b[0]] == 'green' and red > 2 * max(green, 1):
                flag(e, f'줄{j + 1} {NAMES[b[0]]} 인데 빨강 화소 많음({red} vs 초록 {green})')
# 같은 묶음 앞뒤 장
for t in range(len(items)):
    e, _, bs = items[t]
    nb = [items[u] for u in (t - 1, t + 1) if 0 <= u < len(items) and items[u][0]['batch'] == e['batch']]
    for j, b in enumerate(bs):
        for e2, _, bs2 in nb:
            m = max(bs2, key=lambda c: iou(b, c), default=None)
            if m is not None and iou(b, m) > 0.3 and m[0] != b[0]:
                flag(e, f'줄{j + 1} {NAMES[b[0]]} ↔ 이웃 {e2["image"].split("__")[1][:3]} 같은 자리 {NAMES[m[0]]}')
    c = Counter(b[0] for b in bs)
    if nb and all(Counter(b[0] for b in bs2) != c for _, _, bs2 in nb):
        flag(e, '앞뒤 장 모두와 클래스별 개수 다름 ' + ' '.join(f'{NAMES[k][:1].upper()}{NAMES[k].split("_")[1][:3]}{v}' for k, v in sorted(c.items())))
res = {'folder': str(D), 'n_images': len(items), 'n_flagged': len(flags), 'flags': flags,
       'note': '도구 확인 · 판정 아님. 의심 장은 사용자가 사진을 보고 정함. 흔들린 퍽은 구분 안 되면 표시 안 함(사용자 규칙)'}
(D.parent / 'check_labels_result.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(len(items), '장 중 의심', len(flags))
for k, v in flags.items():
    print(k.split('__')[0], k.split('__')[1][:3], ' | '.join(v))
