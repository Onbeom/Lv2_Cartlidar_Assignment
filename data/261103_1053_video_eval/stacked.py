"""(11-03 14:1x 고침: 깊이 차 < 0.06 m 일 때만 쌓임 — 화면에서 위아래로 보여도 앞뒤로 놓인 퍽은 깊이 차가 큼)
쌓인 퍽 정답 찾기(사용자 11-03: 과제에 쌓인 퍽 없음 → 판정에서 뺌). 규칙: 두 정답 상자가 가로로 겹치고(겹친 폭 ≥ 좁은 쪽 폭의 60 %)
위 상자의 아래 변이 아래 상자의 위쪽 40 % 안에 있으면 두 상자 모두 '쌓임'. → stacked.json {이미지 키: [정답 번호]} + 그림 review/stacked_check.png"""
import json, sys
from pathlib import Path
import cv2, numpy as np
src = open(Path(__file__).resolve().parent / 'judge.py').read(); BD = None
exec(src[src.index('def box_depth'):src.index('# 정답 읽기')])
HERE = Path(__file__).resolve().parent; out = {}; tiles = []
INFO = {}
for cam in ('judge_front', 'judge_rear', 'video_front', 'video_rear'):
    for e in json.load(open(HERE / cam / 'auto_label.json')): INFO[f'{cam}/{e["image"]}'] = e['depth']
for cam in ('judge_front', 'judge_rear', 'video_front', 'video_rear'):
    for lab in sorted((HERE / cam / 'labels').glob('*.txt')):
        img = HERE / cam / 'images' / (lab.stem + '.png'); h, w = cv2.imread(str(img)).shape[:2]
        B = []
        for l in lab.read_text().split('\n'):
            if l.strip():
                k, x, y, bw, bh = map(float, l.split()); B.append([(x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h])
        dep = cv2.imread(INFO[f'{cam}/{img.name}'], cv2.IMREAD_UNCHANGED); Z = [box_depth(dep, b, 0.001) for b in B]
        S = set()
        for i, a in enumerate(B):
            for j, b in enumerate(B):
                if i == j: continue
                ov = min(a[2], b[2]) - max(a[0], b[0]); narrow = min(a[2] - a[0], b[2] - b[0])
                if narrow <= 0 or ov < 0.6 * narrow: continue
                if a[3] <= b[3] and b[1] - 0.05 * (b[3] - b[1]) <= a[3] <= b[1] + 0.4 * (b[3] - b[1]) and a[1] < b[1]:  # a 가 b 위에
                    if Z[i] is not None and Z[j] is not None and abs(Z[i] - Z[j]) < 0.06: S |= {i, j}
        if S:
            out[f'{cam}/{img.name}'] = sorted(S)
            if len(tiles) < 40:
                im = cv2.imread(str(img))
                for i, b in enumerate(B):
                    cv2.rectangle(im, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255) if i in S else (0, 255, 0), 2)
                cv2.putText(im, img.name[:14], (5, 20), 0, .6, (0, 255, 255), 2); tiles.append(cv2.resize(im, (320, 240)))
json.dump(out, open(HERE / 'stacked.json', 'w'), ensure_ascii=False, indent=1)
tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 8)
cv2.imwrite(str(HERE / 'review/stacked_check.png'), np.vstack([np.hstack(tiles[i:i + 8]) for i in range(0, len(tiles), 8)]))
from collections import Counter
print(Counter(k.split('/')[0] for k in out), '상자', sum(len(v) for v in out.values()))
