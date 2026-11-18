"""E3 상자 캐시 (학습 입력용). 배포와 같은 384 x 640, conf .25"""
import json, sys
from pathlib import Path
from ultralytics import YOLO
d = Path(sys.argv[1]); J = json.loads((d / 'truth.json').read_text()); y = YOLO('/home/pa4/git/lv2_next_module/261104_1618_E3/runs/E3/weights/last.pt'); O = {}
for im in J['images']:
    O[im['id']] = [b.xyxy[0].cpu().tolist() for b in y.predict(str(d / im['image']), imgsz=(384, 640), conf=.25, iou=.45, device=0, verbose=False)[0].boxes]
(d / 'e3_dets.json').write_text(json.dumps(O)); print(len(O), sum(len(v) for v in O.values()))
