"""판정 영상의 모든 프레임(6 fps)에 모델을 돌려 검출 저장(창 판정용). 사용: window_dets.py <가중치> <출력 json> <영상 폴더>... (DETS_DEVICE)"""
import json, os, sys
from pathlib import Path
os.environ['YOLO_OFFLINE'] = 'true'
from ultralytics import YOLO
m = YOLO(sys.argv[1]); out = {}; dev = os.environ.get('DETS_DEVICE', 'cpu')
for vd in sys.argv[3:]:
    for d in sorted(p for p in Path(vd).iterdir() if (p / 'capture.json').exists()):
        imgs = sorted((d / 'color').glob('*.png'))
        for k in range(0, len(imgs), 32):
            rs = m.predict([str(p) for p in imgs[k:k + 32]], conf=0.10, iou=0.45, imgsz=640, device=dev, verbose=False)
            for p, r in zip(imgs[k:k + 32], rs):
                out[f'{d.name}/{int(p.stem)}'] = [[int(c), float(s), *map(float, b)] for c, s, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
        print(d.name, len(imgs), flush=True)
json.dump(out, open(sys.argv[2], 'w'))
