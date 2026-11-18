"""팔 영상 참고 수치(판정 아님): 판정 코드와 같은 짝 맞춤에 조건만 바꿔 E3 · E2 재현율 · 정밀도. 라벨 · 규칙은 바꾸지 않는다."""
import json
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent.parent; S0 = 114.
def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0])); iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.
side = lambda b: max(b[2] - b[0], b[3] - b[1])
L = json.load(open(HERE / 'final_arm/auto_label.json')); GT = {}
for e in L:
    im = cv2.imread(str(HERE / 'final_arm/images' / e['image'])); H, W = im.shape[:2]; hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV); g = []
    for l in (HERE / 'final_arm/labels' / (e['image'][:-4] + '.txt')).read_text().split('\n'):
        if l.strip():
            k, x, y, w, h = l.split(); x, y, w, h = float(x) * W, float(y) * H, float(w) * W, float(h) * H; b = [x - w / 2, y - h / 2, x + w / 2, y + h / 2]
            q = hsv[max(0, int(b[1])):int(b[3]) + 1, max(0, int(b[0])):int(b[2]) + 1].reshape(-1, 3); m = ((q[:, 0] < 10) | (q[:, 0] > 170)) & (q[:, 1] > 80) & (q[:, 2] > 60)
            bright = bool(int(k) == 2 and m.mean() >= .5 and np.median(q[m, 2]) >= 200) if m.sum() else False
            g.append({'c': int(k), 'cs': {int(k)}, 'b': b, 'bright': bright})
    for i in range(len(g) - 1, 0, -1):
        j = next((j for j in range(i) if iou(g[i]['b'], g[j]['b']) >= .9), None)
        if j is not None: g[j]['cs'] |= g[i]['cs']; g.pop(i)
    GT[e['image']] = g
def run(name, th=.6, color_only=False, agnostic_nms=False, min_w=0, skip=lambda x: False, fix=None):
    D = json.load(open(HERE / f'dets_{name}.json')); tp = fp = tg = 0
    for im, g0 in GT.items():
        g = [dict(x) for x in g0]
        if fix and im in fix:
            for i, c in fix[im]: g[i]['cs'] = {c}; g[i]['c'] = c
        for x in g: x['in'] = side(x['b']) >= S0 - 1e-6 and min(x['b'][2] - x['b'][0], x['b'][3] - x['b'][1]) >= min_w and not skip(x)
        ds = sorted([d for d in D['final_arm/' + im] if d[1] >= th], key=lambda d: -d[1])
        if agnostic_nms:
            k = []
            for d in ds:
                if all(iou(d[2:], o[2:]) < .45 for o in k): k.append(d)
            ds = k
        used = set(); tg += sum(x['in'] for x in g)
        for c, s, *b in ds:
            ok = [(iou(b, x['b']), j) for j, x in enumerate(g) if j not in used and iou(b, x['b']) >= .5 and (not x['in'] or (c % 2 in {k % 2 for k in x['cs']} if color_only else c in x['cs']))]
            if ok:
                j = max(ok)[1]; used.add(j); tp += g[j]['in']; continue
            if side(b) < S0 - 1e-6 or any(not x['in'] and iou(b, x['b']) >= .5 for x in g): continue
            fp += 1
    return f'대상 {tg} 맞힘 {tp} 헛 {fp} 놓침 {tg - tp} | 재현율 {tp / tg:.3f} 정밀도 {tp / max(1, tp + fp):.3f}'
idx = next(i for i, x in enumerate(GT['final_arm_v1__049_001468.png']) if x['c'] == 3)
FIX = {'final_arm_v1__049_001468.png': [(idx, 1)]}
print('밝은 넓은 면 Rcub 대상 수', sum(x['bright'] and side(x['b']) >= S0 for g in GT.values() for x in g))
for n in ('E3', 'E2'):
    print(n, '판정 그대로        ', run(n))
    print(n, '049 라벨 하나 고침 ', run(n, fix=FIX))
    print(n, '폭 40 px 미만 가장자리 뺌', run(n, min_w=40))
    print(n, '밝은 넓은 면 Rcub 뺌', run(n, skip=lambda x: x['bright']))
    print(n, '밝은 면 뺌+폭 40 뺌', run(n, min_w=40, skip=lambda x: x['bright']))
    print(n, '색만 봄            ', run(n, color_only=True))
    print(n, '클래스 무관 NMS    ', run(n, agnostic_nms=True))
    print(n, '문턱 0.25          ', run(n, th=.25))
