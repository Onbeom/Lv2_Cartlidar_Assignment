"""C(또는 B) 문턱 th 에서 놓친 정답을 원인(클래스 틀림 · 아예 없음)과 상자 높이(화소)별로 나눔. 사용: fn_breakdown.py <B|C> <th>"""
import json, sys
from collections import Counter
from analyze import gts, iou, AB
D = json.load(open(f'dets_{sys.argv[1]}.json')); th = float(sys.argv[2])
bins = [0, 24, 48, 96, 10000]
for cam in sorted({k.split('/')[0] for k in D}):
    tot, wrong, none = Counter(), Counter(), Counter()
    for key, dets in D.items():
        if not key.startswith(cam): continue
        g = gts(key); dd = [x for x in dets if x[1] >= th]
        for gc, gb in g:
            h = gb[3] - gb[1]; b = next(i for i in range(4) if bins[i] <= h < bins[i + 1]); tot[b] += 1
            same = [d for d in dd if d[0] == gc and iou(d[2:], gb) >= 0.5]
            if same: continue
            if any(iou(d[2:], gb) >= 0.5 for d in dd): wrong[b] += 1
            else: none[b] += 1
    print(cam, [(f'{bins[i]}~{bins[i+1]}px', tot[i], f'틀림 {wrong[i]}', f'없음 {none[i]}', f'맞힘 {round(1 - (wrong[i] + none[i]) / max(tot[i], 1), 3)}') for i in range(4)])
