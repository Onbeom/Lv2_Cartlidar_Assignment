"""팔 영상 E3 @0.6 을 상자 긴 변(px) 구간별로(판정 아님). 짝 맞춤은 judge.py 와 같고, 구간만 나눈다. 인자: 폴더 dets 파일."""
import json, sys
from pathlib import Path
import cv2
HERE = Path(__file__).resolve().parent.parent
BINS = [(114, 150), (150, 200), (200, 300), (300, 500), (500, 9999)]
def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0])); iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.
side = lambda b: max(b[2] - b[0], b[3] - b[1])
def run(folder, dets, cam):
    D = json.load(open(dets)); L = json.load(open(folder / 'auto_label.json'))
    S = {b: dict(t=0, tp=0, fp=0, shape=0, color=0, low=0) for b in BINS}
    def bin_of(v): return next(b for b in BINS if b[0] <= v < b[1])
    for e in L:
        im = cv2.imread(str(folder / 'images' / e['image'])); H, W = im.shape[:2]; g = []
        for l in (folder / 'labels' / (e['image'][:-4] + '.txt')).read_text().split('\n'):
            if l.strip():
                k, x, y, w, h = l.split(); x, y, w, h = float(x) * W, float(y) * H, float(w) * W, float(h) * H
                g.append({'c': int(k), 'cs': {int(k)}, 'b': [x - w / 2, y - h / 2, x + w / 2, y + h / 2]})
        for i in range(len(g) - 1, 0, -1):
            j = next((j for j in range(i) if iou(g[i]['b'], g[j]['b']) >= .9), None)
            if j is not None: g[j]['cs'] |= g[i]['cs']; g.pop(i)
        for x in g:
            x['in'] = side(x['b']) >= 114
            if x['in']: S[bin_of(side(x['b']))]['t'] += 1
        used = set(); ds = sorted([d for d in D[f'{cam}/{e["image"]}'] if d[1] >= .6], key=lambda d: -d[1])
        for c, s, *b in ds:
            ok = [(iou(b, x['b']), j) for j, x in enumerate(g) if j not in used and iou(b, x['b']) >= .5 and (not x['in'] or c in x['cs'])]
            if ok:
                j = max(ok)[1]; used.add(j)
                if g[j]['in']: S[bin_of(side(g[j]['b']))]['tp'] += 1
                continue
            if side(b) < 114: continue
            S[bin_of(side(b))]['fp'] += 1
        for j, x in enumerate(g):
            if x['in'] and j not in used:
                hi = [d for d in ds if iou(d[2:], x['b']) >= .5]; s = S[bin_of(side(x['b']))]
                if hi: s['shape' if int(hi[0][0]) % 2 == x['c'] % 2 else 'color'] += 1
                else: s['low'] += 1
    return S
for name, folder, dets, cam in (('최종 E3', HERE / 'final_arm', HERE / 'dets_E3.json', 'final_arm'), ('최종 E2', HERE / 'final_arm', HERE / 'dets_E2.json', 'final_arm'), ('경기장 E3', HERE.parent / '261104_1211_arena_eval/arena_arm', HERE.parent / '261104_1211_arena_eval/dets_E3.json', 'arena_arm')):
    print('==', name); S = run(folder, dets, cam); T = TP = FP = 0
    for b, s in S.items():
        T += s['t']; TP += s['tp']; FP += s['fp']
        print(f"  {b[0]}~{b[1]} px: 대상 {s['t']} 맞힘 {s['tp']} 헛 {s['fp']} 놓침 {s['t'] - s['tp']}(모양 {s['shape']} · 색 {s['color']} · 점수/없음 {s['low']}) | 재현율 {s['tp'] / max(1, s['t']):.3f} 정밀도 {s['tp'] / max(1, s['tp'] + s['fp']):.3f}")
    print(f'  합: 대상 {T} 맞힘 {TP} 헛 {FP} | {TP / T:.3f} {TP / (TP + FP):.3f}')
    for lo in (150, 200):
        t = sum(s['t'] for b, s in S.items() if b[0] >= lo); tp = sum(s['tp'] for b, s in S.items() if b[0] >= lo); fp = sum(s['fp'] for b, s in S.items() if b[0] >= lo)
        print(f'  긴 변 {lo} px 이상만(참고): 대상 {t} | 재현율 {tp / t:.3f} 정밀도 {tp / max(1, tp + fp):.3f}')
