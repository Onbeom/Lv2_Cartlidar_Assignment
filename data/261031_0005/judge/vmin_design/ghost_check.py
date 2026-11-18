#!/usr/bin/env python3
"""렌더 잔상 검사 (PC, 모의 렌더 도구 확인). 상태 영상마다 켜진 칸(꺼짐 영상과 칸 중심 7×7 의 채널 최대 |차| 평균 > 30)이
그 상태 칸 하나뿐인지 본다. 다르면(예: 노랑 상태에 빨강이 남음) 렌더 결함으로 보고 종료 코드 1.
  ghost_check.py <render_r12 결과 폴더> [...]"""
import json, sys
import cv2, numpy as np

bad_all = 0
for R in sys.argv[1:]:
    meta = json.load(open(f'{R}/render_r12.json'))
    bad = []
    for p in meta['poses']:
        off = cv2.imread(f"{R}/{p['name']}/off.png").astype(np.int16)
        for st in ('red', 'yellow', 'green'):
            im = cv2.imread(f"{R}/{p['name']}/{st}.png").astype(np.int16)
            lit = []
            for seg in ('red', 'yellow', 'green'):
                u, w = (int(round(x)) for x in p['segments'][seg]['px'])
                if np.abs(im[w - 3:w + 4, u - 3:u + 4] - off[w - 3:w + 4, u - 3:u + 4]).max(2).mean() > 30:
                    lit.append(seg)
            if lit != [st]:
                bad.append(f"{p['name']}/{st}: 켜진 칸 {lit}")
    print(R, '잔상 없음' if not bad else f'결함 {len(bad)}: {bad}')
    bad_all += len(bad)
sys.exit(1 if bad_all else 0)
