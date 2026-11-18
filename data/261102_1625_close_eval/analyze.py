"""dets_*.json 과 라벨로 폴더(카메라)별 · 묶음별 재현율 · 정밀도 · 모양 · 색을 문턱 0.25 · 0.5 · 0.6 에서 셈. 짝 규칙은 zeroshot_b.py 와 같음(IoU ≥ 0.5, 같은 클래스, 점수 높은 순)."""
import json, sys
from collections import defaultdict
from pathlib import Path
from PIL import Image
SRC = Path('/home/pa4/git/lv2_next_module/261102_1330_yolo_close_draft')
AB = ['RC', 'GC', 'RB', 'GB']

def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0

def gts(key):
    d, n = key.split('/'); w, h = Image.open(SRC / d / 'images' / n).size; out = []
    for l in (SRC / d / 'labels' / (n[:-4] + '.txt')).read_text().split('\n'):
        if l.strip():
            k, x, y, bw, bh = l.split(); x, y, bw, bh = float(x) * w, float(y) * h, float(bw) * w, float(bh) * h
            out.append((int(k), [x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2]))
    return out

G = {}
rep = {}
for name in (sys.argv[1:] if __name__ == "__main__" else []):
    D = json.load(open(f'dets_{name}.json'))
    for th in (0.25, 0.5, 0.6):
        S = defaultdict(lambda: defaultdict(int)); confuse = defaultdict(int)
        for key, dets in D.items():
            if key not in G: G[key] = gts(key)
            g = G[key]; dd = sorted([x for x in dets if x[1] >= th], key=lambda x: -x[1])
            for grp in (key.split('/')[0], key.split('/')[0] + ':' + key.split('/')[1][:5]):
                s = S[grp]; s['images'] += 1; s['targets'] += len(g)
                used = set()
                for c, sc, *b in dd:
                    cand = [(iou(b, gb), j) for j, (gc, gb) in enumerate(g) if j not in used and gc == c]
                    bi = max(cand, default=(0, None))
                    if bi[0] >= 0.5: used.add(bi[1]); s['tp'] += 1
                    else: s['fp'] += 1
                used2 = set()
                for c, sc, *b in dd:
                    cand = [(iou(b, gb), j) for j, (gc, gb) in enumerate(g) if j not in used2]
                    bi = max(cand, default=(0, None))
                    if bi[0] >= 0.5:
                        used2.add(bi[1]); gc = g[bi[1]][0]; s['agn'] += 1
                        s['shape_ok'] += (c // 2) == (gc // 2); s['color_ok'] += (c % 2) == (gc % 2)
                        if ':' not in grp and c != gc: confuse[f'{AB[gc]}→{AB[c]}'] += 1
                s['fn'] += len(g) - len(used)
        for grp, s in S.items():
            r = dict(s); r.setdefault('fp', 0); r.setdefault('tp', 0); r['recall'] = round(s['tp'] / max(s['targets'], 1), 4); r['precision'] = round(s['tp'] / max(s['tp'] + s['fp'], 1), 4)
            r['shape'] = round(s['shape_ok'] / max(s['agn'], 1), 4); r['color'] = round(s['color_ok'] / max(s['agn'], 1), 4)
            if ':' not in grp: r['confusion'] = dict(confuse) if grp == 'x' else None
            rep.setdefault(name, {}).setdefault(str(th), {})[grp] = r
        # 카메라별 혼동(클래스 무관 짝 기준)
        for cam in {k.split('/')[0] for k in D}:
            cc = defaultdict(int)
            for key, dets in D.items():
                if not key.startswith(cam): continue
                g = G[key]; used2 = set()
                for c, sc, *b in sorted([x for x in dets if x[1] >= th], key=lambda x: -x[1]):
                    cand = [(iou(b, gb), j) for j, (gc, gb) in enumerate(g) if j not in used2]
                    bi = max(cand, default=(0, None))
                    if bi[0] >= 0.5:
                        used2.add(bi[1]); gc = g[bi[1]][0]
                        if c != gc: cc[f'{AB[gc]}→{AB[c]}'] += 1
            rep[name][str(th)][cam]['confusion'] = dict(cc)
if __name__ == '__main__':
    json.dump(rep, open('results.json', 'w'), ensure_ascii=False, indent=1)
for name in rep:
    for th in rep[name]:
        for grp, r in sorted(rep[name][th].items()):
            if ':' not in grp:
                print(name, th, grp, {k: r.get(k, 0) for k in ('images', 'targets', 'tp', 'fp', 'fn', 'recall', 'precision', 'shape', 'color', 'confusion')})
