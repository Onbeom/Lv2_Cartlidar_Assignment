"""라벨 초안(후보 B, CPU). auto_label.py 와 같은 문턱 · 출력 형식, 사진 이름은 원래 이름 그대로(묶음 접두사 유지). 원본 폴더는 읽기만."""
import json
import shutil
from pathlib import Path

from ultralytics import YOLO

ROOT = Path('/home/pa4/git/lv2_next_module')
SRC = ROOT / '261026_1414/real_labels_261027_far_dark/images'
MODEL = ROOT / '261030_1325/prototype/yolo_models/yolo11n_task4_B.pt'
OUT = Path(__file__).resolve().parent / 'real_labels_far_dark_B'
CONF, NMS = 0.10, 0.45
NAMES4 = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']
model = YOLO(str(MODEL))
assert [model.names[i] for i in range(len(model.names))] == NAMES4, model.names
(OUT / 'images').mkdir(parents=True, exist_ok=False)
(OUT / 'labels').mkdir()
(OUT / 'classes.txt').write_text('\n'.join(NAMES4) + '\n')
log = []
for f in sorted(SRC.glob('*.png')):
    shutil.copy2(f, OUT / 'images' / f.name)
    r = model.predict(str(f), conf=CONF, iou=NMS, device='cpu', verbose=False)[0]
    lines, confs = [], []
    for (x, y, w, h), c, s in zip(r.boxes.xywhn.numpy(), r.boxes.cls.numpy(), r.boxes.conf.numpy()):
        lines.append(f'{int(c)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}')
        confs.append(round(float(s), 3))
    (OUT / 'labels' / f'{f.stem}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
    log.append({'image': f.name, 'source': str(f), 'batch': f.name.split('__')[0], 'dets': len(lines), 'conf': confs, 'checked': False})
(OUT / 'auto_label.json').write_text(json.dumps(log, indent=1, ensure_ascii=False))
print(f'{len(log)}장, 상자 {sum(e["dets"] for e in log)}개 (초안, 사람 확인 필요) → {OUT}')
