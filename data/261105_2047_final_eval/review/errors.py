"""judge.py 와 같은 짝 맞춤으로 E3 @0.6 놓침 · 헛상자를 종류별로 나눈다(기준은 바꾸지 않음). 사용: errors.py <모델> → review/errors_<모델>.json + 표."""
import json, sys
from collections import Counter, defaultdict
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent.parent
NAMES = ['red_cyl', 'green_cyl', 'red_cub', 'green_cub']; TH = .6; S0 = 114.; FAR, NEAR = 3., 1.; AGN = {'final_rear'}
name = sys.argv[1]; D = json.load(open(HERE / f'dets_{name}.json'))
def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0])); iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.
def bd(dep, b, unit):
    h, w = dep.shape; x0, y0, x1, y1 = b; cx, cy, bw, bh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 4, (y1 - y0) / 4
    p = dep[max(0, int(cy - bh)):min(h, int(cy + bh) + 1), max(0, int(cx - bw)):min(w, int(cx + bw) + 1)]; p = p[p > 0]
    return float(np.median(p)) * unit if p.size else None
side = lambda b: max(b[2] - b[0], b[3] - b[1])
out = {}
for cam in ('final_front', 'final_rear', 'final_arm'):
    arm = cam.endswith('_arm'); L = json.load(open(HERE / cam / 'auto_label.json')); miss, fp, tgt, tp = [], [], 0, 0
    for e in L:
        key = f'{cam}/{e["image"]}'; im = cv2.imread(str(HERE / cam / 'images' / e['image'])); H, W = im.shape[:2]; gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY); sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
        g = []
        for l in (HERE / cam / 'labels' / (e['image'][:-4] + '.txt')).read_text().split('\n'):
            if l.strip():
                k, x, y, w, h = l.split(); x, y, w, h = float(x) * W, float(y) * H, float(w) * W, float(h) * H
                g.append({'c': int(k), 'cs': {int(k)}, 'b': [x - w / 2, y - h / 2, x + w / 2, y + h / 2]})
        for i in range(len(g) - 1, 0, -1):
            j = next((j for j in range(i) if iou(g[i]['b'], g[j]['b']) >= .9), None)
            if j is not None: g[j]['cs'] |= g[i]['cs']; g.pop(i)
        dep = unit = None
        if e.get('depth'):
            dep = cv2.imread(e['depth'], cv2.IMREAD_UNCHANGED); unit = json.load(open(Path(e['depth']).parent.parent / 'capture.json'))['calibration']['depth_unit_m']
        for x in g:
            if arm: x['in'] = side(x['b']) >= S0 - 1e-6; x['near'] = True
            else: x['z'] = bd(dep, x['b'], unit); x['in'] = x['z'] is not None and x['z'] <= FAR; x['near'] = x['z'] is not None and x['z'] <= NEAR
        all_d = D[key]; dets = sorted([d for d in all_d if d[1] >= TH], key=lambda d: -d[1]); used = set(); tgt += sum(x['in'] for x in g)
        for c, s, *b in dets:
            ok = [(iou(b, x['b']), j) for j, x in enumerate(g) if j not in used and iou(b, x['b']) >= .5 and (not x['in'] or cam in AGN or (c in x['cs'] if x['near'] else c % 2 in {k % 2 for k in x['cs']}))]
            if ok:
                j = max(ok)[1]; used.add(j); tp += g[j]['in']; continue
            if arm: ign = side(b) < S0 - 1e-6
            else:
                z = bd(dep, b, unit); ign = z is None or z > FAR or any(not x['in'] and iou(b, x['b']) >= .5 for x in g)
            if ign: continue
            ov = max(((iou(b, x['b']), j) for j, x in enumerate(g)), default=(0, -1))
            kind = '배경' if ov[0] < .1 else ('클래스 틀림' if ov[0] >= .5 else '상자 어긋남')
            if kind == '클래스 틀림': kind += ':' + ('모양' if c % 2 == g[ov[1]]['c'] % 2 else '색')
            cover = (min(b[2], g[ov[1]]['b'][2]) - max(b[0], g[ov[1]]['b'][0])) * (min(b[3], g[ov[1]]['b'][3]) - max(b[1], g[ov[1]]['b'][1])) / ((b[2] - b[0]) * (b[3] - b[1])) if ov[1] >= 0 and ov[0] > 0 else 0
            fp.append({'img': e['image'], 'batch': e['batch'], 'c': int(c), 's': round(s, 3), 'b': [round(v, 1) for v in b], 'kind': kind, 'iou': round(ov[0], 2), 'in_gt': round(cover, 2), 'gt': g[ov[1]]['c'] if ov[1] >= 0 else None, 'side': round(side(b)), 'sharp': round(sharp)})
        for j, x in enumerate(g):
            if not x['in'] or j in used: continue
            b = x['b']; near = [d for d in all_d if iou(d[2:], b) >= .5]; hi = [d for d in near if d[1] >= TH]; good = [d for d in near if d[0] in x['cs']]
            if hi: kind = '클래스 틀림:' + ('모양' if hi[0][0] % 2 == x['c'] % 2 else '색')
            elif near: kind = f'점수 낮음(같은 클래스)' if good else '점수 낮음(다른 클래스)'
            elif any(iou(d[2:], b) >= .1 and d[1] >= TH for d in all_d): kind = '상자 어긋남(IoU<0.5)'
            else: kind = '검출 없음'
            edge = b[0] <= 2 or b[1] <= 2 or b[2] >= W - 2 or b[3] >= H - 2
            miss.append({'img': e['image'], 'batch': e['batch'], 'c': x['c'], 'b': [round(v, 1) for v in b], 'kind': kind, 'best': round(max((d[1] for d in near), default=0), 3), 'side': round(side(b)), 'edge': edge, 'z': round(x.get('z') or 0, 2), 'near': bool(x['near']), 'sharp': round(sharp)})
    out[cam] = {'miss': miss, 'fp': fp}
    print(f'== {cam} 대상 {tgt} 맞힘 {tp} 놓침 {len(miss)} 헛 {len(fp)}')
    print('  놓침:', dict(Counter(m['kind'] for m in miss)), '가장자리', sum(m['edge'] for m in miss), '클래스', dict(Counter(NAMES[m['c']] for m in miss)))
    print('  헛:', dict(Counter(f['kind'] for f in fp)), '퍽 안(상자의 80% 이상)', sum(f['in_gt'] >= .8 for f in fp))
    print('  자리 잇달음: 장당 놓침', Counter(Counter(m['img'] for m in miss).values()))
json.dump(out, open(HERE / 'review' / f'errors_{name}.json', 'w'), ensure_ascii=False, indent=0)
