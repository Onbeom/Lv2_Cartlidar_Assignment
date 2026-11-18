"""도구 확인(판정 아님, 탐색): 흐린 장(라플라시안 분산 < 문턱)을 아예 쓰지 않으면(정답 · 검출 모두 뺌) D 문턱 0.6 재현율 · 정밀도가 어떻게 되는지.
문턱을 결과를 보고 고르면 낙관적이다 → 채택하려면 새 영상으로 다시 판정해야 함."""
import json, sys
from collections import defaultdict
from pathlib import Path
import cv2
HERE = Path(__file__).resolve().parent
src = open(HERE / 'judge.py').read()
blur = {}
for cam in ('video_front', 'video_rear', 'video_arm'):
    for p in (HERE / cam / 'images').iterdir():
        blur[f'{cam}/{p.name}'] = cv2.Laplacian(cv2.imread(str(p), cv2.IMREAD_GRAYSCALE), cv2.CV_64F).var()
out = {}
for thr in (0, 30, 50, 80, 120):
    keep = {k for k, v in blur.items() if v >= thr}
    s = src.replace("for key, g in GT.items():\n            cam", "for key, g in GT.items():\n            if key not in KEEP: continue\n            cam")
    s = s.replace("json.dump(rep, open(HERE / 'judge.json', 'w'), ensure_ascii=False, indent=1)", "").split("for th in ('0.6',):")[0]
    ns = {'__file__': str(HERE / 'judge.py'), 'KEEP': keep}; sys.argv = ['x', 'B', 'C', 'D']
    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()): exec(s, ns)
    for cam in ('video_front', 'video_rear', 'video_arm'):
        n = sum(1 for k in keep if k.startswith(cam)); tot = sum(1 for k in blur if k.startswith(cam))
        row = {m: (ns['rep'][m]['0.6'][cam]['recall'], ns['rep'][m]['0.6'][cam]['precision']) for m in 'BCD'}
        out.setdefault(cam, []).append((thr, f'{n}/{tot}장', row))
for cam, rows in out.items():
    print(cam)
    for r in rows: print('  흐림 문턱', r[0], r[1], r[2])
json.dump(out, open(HERE / 'blur_gate.json', 'w'), ensure_ascii=False, indent=1)
