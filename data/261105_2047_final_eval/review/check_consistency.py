"""같은 묶음에서 같은 자리(IoU >= 0.5)의 상자인데 라벨 클래스가 다른 경우를 찾는다(카메라가 서 있는 장면의 라벨 실수 찾기). 모델 결과는 보지 않는다."""
import json
from collections import defaultdict, Counter
from pathlib import Path
HERE = Path(__file__).resolve().parent.parent
NAMES = ['red_cyl', 'green_cyl', 'red_cub', 'green_cub']
def xy(v):
    k, x, y, w, h = v
    return (x - w / 2, y - h / 2, x + w / 2, y + h / 2)
def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0])); iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.
for cam in ('final_front', 'final_rear', 'final_arm'):
    L = json.load(open(HERE / cam / 'auto_label.json'))
    B = defaultdict(list)
    for e in L:
        cur = [[float(x) for x in l.split()] for l in (HERE / cam / 'labels' / (e['image'][:-4] + '.txt')).read_text().split('\n') if l.strip()]
        B[e['batch']].append((e['image'], cur))
    for b, fr in B.items():
        tr = []  # 자리별 묶음
        for im, cur in fr:
            for x in cur:
                t = next((t for t in tr if iou(t['box'], xy(x)) >= 0.5), None)
                if t is None:
                    t = {'box': xy(x), 'cnt': Counter(), 'items': []}; tr.append(t)
                t['cnt'][int(x[0])] += 1; t['items'].append((im, int(x[0])))
        big = [t for t in tr if sum(t['cnt'].values()) >= 5]
        print(f'== {cam} {b}: 장 {len(fr)} 자리 {len(tr)}(5장 이상 {len(big)}) 상자 {sum(len(c) for _, c in fr)}')
        for t in sorted(big, key=lambda t: -sum(t['cnt'].values())):
            if len(t['cnt']) > 1:
                mc = t['cnt'].most_common(1)[0][0]
                odd = [i for i, k in t['items'] if k != mc]
                print('   ', [round(v, 2) for v in t['box']], {NAMES[k]: n for k, n in t['cnt'].items()}, 'odd:', [o[-16:-4] for o in odd[:8]])
