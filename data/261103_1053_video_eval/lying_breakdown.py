"""도구 확인: D(문턱 0.6) 앞 D435 재현율을 '누움(상자 가로/세로 ≥ 1.2) / 서 있음(< 1.2)' × 거리(깊이)로 나눔. 정답 클래스까지 맞아야 맞힘(1 m 밖은 색만).
누움 판단은 상자 모양으로 한 근사(위에서 본 장면은 틀릴 수 있음). 쌓인 퍽 · 흐린 장 처리 없음."""
import json, os, sys, contextlib, io
from pathlib import Path
from collections import defaultdict
HERE = Path(__file__).resolve().parent
os.environ.update(JUDGE_CAMS='judge_front,video_front')
src = open(HERE / 'judge.py').read(); src = src[:src.index("side = lambda b")]
ns = {'__file__': str(HERE / 'judge.py')}; sys.argv = ['x']
with contextlib.redirect_stdout(io.StringIO()): exec(src, ns)
GT, iou = ns['GT'], ns['iou']; DET = json.load(open(HERE / 'dets_all_D.json'))
bins = [(0, 0.5), (0.5, 1.0), (1.0, 3.0)]
for cam in ('judge_front', 'video_front'):
    S = defaultdict(lambda: [0, 0])
    for key, g in GT.items():
        if not key.startswith(cam + '/'): continue
        for x in g:
            z = x['z']
            if z is None or z > 3.0: continue
            w, h = x['b'][2] - x['b'][0], x['b'][3] - x['b'][1]; pose = '누움' if w / h >= 1.2 else '서 있음'
            b = next(f'{lo}~{hi} m' for lo, hi in bins if lo <= z < hi or (hi == 3.0 and z <= 3.0))
            near = z <= 1.0
            hit = any(d[1] >= 0.6 and iou(d[2:], x['b']) >= 0.5 and (d[0] in x['cs'] if near else d[0] % 2 in {k % 2 for k in x['cs']}) for d in DET[key])
            S[(pose, b)][0] += 1; S[(pose, b)][1] += hit
    print('==', cam)
    for k in sorted(S): print(f'  {k[0]:4s} {k[1]:9s} 정답 {S[k][0]:4d} 맞힘 {S[k][1] / S[k][0]:.3f}')
