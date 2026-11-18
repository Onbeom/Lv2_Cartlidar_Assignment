"""팔 기준 '색 + 자리'(사용자 2026-11-06 결정, 결과를 본 뒤 바꿈): 색만 맞으면 맞힘, 같은 색 · IoU >= 0.7 겹침은 점수 높은 하나만(arm.engine_merge_iou), 문턱은 0.6 과 엔진 하한 0.25."""
import json
from pathlib import Path
import cv2
HERE = Path(__file__).resolve().parent.parent; S0 = 114.
def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0])); iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.
side = lambda b: max(b[2] - b[0], b[3] - b[1])
def run(folder, dets, cam, th, min_w=0, batch=None):
    D = json.load(open(dets)); L = json.load(open(folder / 'auto_label.json')); tg = tp = fp = 0; wrong_color = []
    for e in L:
        if batch and e['batch'] != batch: continue
        H, W = cv2.imread(str(folder / 'images' / e['image'])).shape[:2]; g = []
        for l in (folder / 'labels' / (e['image'][:-4] + '.txt')).read_text().split('\n'):
            if l.strip():
                k, x, y, w, h = l.split(); x, y, w, h = float(x) * W, float(y) * H, float(w) * W, float(h) * H; g.append({'c': int(k), 'b': [x - w / 2, y - h / 2, x + w / 2, y + h / 2]})
        for i in range(len(g) - 1, 0, -1):
            if any(iou(g[i]['b'], g[j]['b']) >= .9 for j in range(i)): g.pop(i)
        for x in g: x['in'] = side(x['b']) >= S0 and min(x['b'][2] - x['b'][0], x['b'][3] - x['b'][1]) >= min_w
        ds = sorted([d for d in D[f'{cam}/{e["image"]}'] if d[1] >= th], key=lambda d: -d[1]); kept = []
        for d in ds:
            if all(int(d[0]) % 2 != int(o[0]) % 2 or iou(d[2:], o[2:]) < .7 for o in kept): kept.append(d)
        used = set(); tg += sum(x['in'] for x in g)
        for c, s, *b in kept:
            ok = [(iou(b, x['b']), j) for j, x in enumerate(g) if j not in used and iou(b, x['b']) >= .5 and (not x['in'] or int(c) % 2 == x['c'] % 2)]
            if ok:
                j = max(ok)[1]; used.add(j); tp += g[j]['in']; continue
            if side(b) < S0 or any(not x['in'] and iou(b, x['b']) >= .5 for x in g): continue
            fp += 1; wrong_color.append((e['image'][10:21], round(s, 2)))
    return f'대상 {tg} 맞힘 {tp} 헛 {fp} 놓침 {tg - tp} | 재현율 {tp / tg:.3f} 정밀도 {tp / max(1, tp + fp):.3f}', wrong_color
F, A = HERE / 'final_arm', HERE.parent / '261104_1211_arena_eval'
for th in (.6, .25):
    for mw in (0, 40):
        r, w = run(F, HERE / 'dets_E3.json', 'final_arm', th, mw); print(f'최종 E3 문턱 {th} 최소폭 {mw}:', r, w if th == .6 and mw == 0 else '')
    print(f'경기장 E3 문턱 {th}:', run(A / 'arena_arm', A / 'dets_E3.json', 'arena_arm', th)[0])
for n in ('E3', 'E2'):
    print(n, '최종 @0.6:', run(F, HERE / f'dets_{n}.json', 'final_arm', .6)[0])
    for b in ('final_arm_v1', 'final_arm_v2', 'final_arm_v3'):
        print(' ', b, run(F, HERE / f'dets_{n}.json', 'final_arm', .6, batch=b)[0])
