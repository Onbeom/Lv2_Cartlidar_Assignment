"""실물 근접 사진(261102_1330_yolo_close_draft, 사용자 고친 라벨)에 B · C 검출을 conf 0.10 · NMS 0.45 · imgsz 640 으로 모두 저장(평가는 analyze.py).
사용: dets.py <가중치 pt> <출력 json> <폴더>...  (GPU 0)"""
import json, os, sys
from pathlib import Path
os.environ['YOLO_OFFLINE'] = 'true'
from ultralytics import YOLO
m = YOLO(sys.argv[1]); out = {}
for d in sys.argv[3:]:
    for img in sorted(Path(d, 'images').glob('*.png')):
        r = m.predict(str(img), conf=0.10, iou=0.45, imgsz=640, device=0, verbose=False)[0]
        out[f'{Path(d).name}/{img.name}'] = [[int(c), float(s), *map(float, b)] for c, s, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
json.dump(out, open(sys.argv[2], 'w'))
print(sys.argv[2], len(out))
