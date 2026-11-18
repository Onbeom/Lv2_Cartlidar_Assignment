"""새 실물 근접 사진(2026-11-02) 라벨 초안(후보 C, CPU). auto_label.py 와 같은 문턱 · 출력 형식(fix_labels.py 로 고침).
사진은 원본 폴더에서 하드 링크(디스크 절약, 원본은 읽기만). 이름 = <묶음>__<번호3>_<촬영시각>.png, 묶음 = 카메라 글자 + 촬영 분(f1216 · a1244).
사람이 찍힌 장도 그대로 씀(사람 = 배경 · 장애물, 상자 없음 — 사용자 2026-11-02). 얼굴 있는 장은 팀 저장소에 넣지 않음."""
import json
import os
import sys
from pathlib import Path

from ultralytics import YOLO

ROOT = Path('/home/pa4/git/lv2_next_module')
PHOTO = ROOT / 'Cartlidar/photo'
MODEL = ROOT / '261102_1304_yolo_C/yolo_export/yolo11n_task4_C.pt'
HERE = Path(__file__).resolve().parent
CONF, NMS = 0.10, 0.45
NAMES4 = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']
SETS = {  # 출력 폴더: (사진 폴더, 묶음 안 경로, 카메라 글자)
    'real_labels_close_front_C': (PHOTO / '20261102_close', 'front_261822072247/color.png', 'f'),
    'real_labels_arm720_C': (PHOTO / '20261102_arm720', 'arm/color.png', 'a'),
    'real_labels_close_rear_C': (PHOTO / '20261102_close', 'rear_261822079801/color.png', 'r'),  # 14:2x 추가(뒤 D435 도 다른 장면을 찍음)
}
ONLY = sys.argv[1:]  # 출력 폴더 이름을 주면 그것만(이미 만든 폴더는 건너뜀)
model = YOLO(str(MODEL))
assert [model.names[i] for i in range(len(model.names))] == NAMES4, model.names
for name, (src, rel, tag) in SETS.items():
    if ONLY and name not in ONLY:
        continue
    out = HERE / name
    (out / 'images').mkdir(parents=True, exist_ok=False)
    (out / 'labels').mkdir()
    (out / 'classes.txt').write_text('\n'.join(NAMES4) + '\n')
    log = []
    for n, d in enumerate(sorted(p for p in src.iterdir() if p.is_dir()), 1):
        f = d / rel
        assert f.exists(), f
        batch = tag + d.name[9:13]
        img = f'{batch}__{n:03d}_{d.name}.png'
        os.link(f, out / 'images' / img)
        r = model.predict(str(f), conf=CONF, iou=NMS, device='cpu', verbose=False)[0]
        lines, confs = [], []
        for (x, y, w, h), c, s in zip(r.boxes.xywhn.numpy(), r.boxes.cls.numpy(), r.boxes.conf.numpy()):
            lines.append(f'{int(c)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}')
            confs.append(round(float(s), 3))
        (out / 'labels' / f'{Path(img).stem}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
        log.append({'image': img, 'source': str(f), 'batch': batch, 'dets': len(lines), 'conf': confs,
                    'classes': [int(c) for c in r.boxes.cls.numpy()], 'checked': False})
    (out / 'auto_label.json').write_text(json.dumps(log, indent=1, ensure_ascii=False))
    print(f'{name}: {len(log)}장, 상자 {sum(e["dets"] for e in log)}개 (초안, 사람 확인 필요)', flush=True)
