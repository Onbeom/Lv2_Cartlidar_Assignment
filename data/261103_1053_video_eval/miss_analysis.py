"""D(문턱 0.6) 영상 판정에서 놓친 정답을 원인별로 나눔: 클래스 틀림(같은 자리 다른 클래스 검출) · 점수 낮음(0.25~0.6 에 맞는 검출) · 아예 없음.
장마다 흔들림(라플라시안 분산, 낮을수록 흐림)과 상자 높이도 함께 → 흐림 · 작은 상자가 원인인지 봄. 사용: miss_analysis.py [모델]"""
import json, sys
from collections import Counter
from pathlib import Path
import cv2, numpy as np
sys.argv = [sys.argv[0]] + (sys.argv[1:] or ['D'])
HERE = Path(__file__).resolve().parent
import importlib.util
src = open(HERE / 'judge.py').read(); src = src[:src.index("side = lambda b")]  # 정답 · 깊이 · s0 읽기만 재사용
ns = {'__file__': str(HERE / 'judge.py')}; exec(src, ns); GT, META, iou = ns['GT'], ns['META'], ns['iou']
D = json.load(open(HERE / f'dets_{sys.argv[1]}.json'))
side = lambda b: max(b[2] - b[0], b[3] - b[1])
for cam in ('video_front', 'video_rear', 'video_arm'):
    cause, blur_all, blur_miss, h_all, h_miss = Counter(), [], [], [], []
    for key, g in GT.items():
        if not key.startswith(cam): continue
        m = META[key]; im = cv2.imread(str(HERE / key.split('/')[0] / 'images' / key.split('/')[1]), cv2.IMREAD_GRAYSCALE)
        bl = cv2.Laplacian(im, cv2.CV_64F).var()
        for x in g:
            if cam == 'video_arm':
                s0 = META['s0'][m['batch']]; x['in'] = s0 is not None and side(x['b']) >= s0 - 1e-6; near = True
            else:
                x['in'] = x['z'] is not None and x['z'] <= 3.0; near = x['z'] is not None and x['z'] <= 1.0
            if not x['in']: continue
            ok = lambda d: d[0] == x['c'] if near else d[0] % 2 == x['c'] % 2
            hit = [d for d in D[key] if d[1] >= 0.6 and iou(d[2:], x['b']) >= 0.5 and ok(d)]
            blur_all.append(bl); h_all.append(x['b'][3] - x['b'][1])
            if hit: continue
            blur_miss.append(bl); h_miss.append(x['b'][3] - x['b'][1])
            if any(d[1] >= 0.6 and iou(d[2:], x['b']) >= 0.5 for d in D[key]): cause['클래스 틀림'] += 1
            elif any(iou(d[2:], x['b']) >= 0.5 and ok(d) for d in D[key]): cause['점수 0.6 미만'] += 1
            elif any(iou(d[2:], x['b']) >= 0.3 for d in D[key]): cause['상자 어긋남(IoU<0.5)'] += 1
            else: cause['검출 없음'] += 1
    q = lambda a: [round(float(np.percentile(a, p)), 1) for p in (10, 50)] if a else None
    print(cam, dict(cause), '| 흐림 분산 p10·p50 전체', q(blur_all), '놓침', q(blur_miss), '| 상자 높이 p10·p50 전체', q(h_all), '놓침', q(h_miss))
