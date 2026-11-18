"""도구 확인(판정 아님, 11-02): YOLO 후보 B 를 튜터 합성 자료(라벨 있음)에 그대로 적용해 맞히는 정도를 본다 → 추가 학습이 필요한지 판단 근거.
튜터 클래스 0 red_cylinder · 1 red_prism · 2 green_cylinder · 3 green_prism → 우리 0 red_cylinder · 2 red_cuboid · 1 green_cylinder · 3 green_cuboid.
짝 = IoU ≥ 0.5, conf ≥ 0.25, NMS 0.45(평가 묶음과 같음). CPU.
사용: yolo-venv/bin/python zeroshot_b.py [--weights <pt>] [--out <json>] <이미지 폴더>[@ours] ... → --out(기본 zeroshot_b.json, 폴더별 집계).
폴더 끝에 @ours 를 붙이면 라벨이 이미 우리 클래스 순서(평가 묶음 등), 아니면 튜터 순서로 보고 바꿈."""
import json
import os
import sys
from pathlib import Path

os.environ['YOLO_OFFLINE'] = 'true'
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('--weights', default='/home/pa4/git/lv2_next_module/261030_1325/prototype/yolo_models/yolo11n_task4_B.pt')
ap.add_argument('--out', default=str(Path(__file__).with_name('zeroshot_b.json')))
ap.add_argument('--device', default='cpu')
ap.add_argument('folders', nargs='+')
args = ap.parse_args()
torch.set_num_threads(2)
W = Path(args.weights)
TUTOR = {0: 0, 1: 2, 2: 1, 3: 3}
MAP = TUTOR
NAMES = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']
model = YOLO(str(W))


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


def gts(img, lab):
    w, h = Image.open(img).size
    out = []
    if lab.exists():
        for line in lab.read_text().split('\n'):
            p = line.split()
            if len(p) == 5:
                c, x, y, bw, bh = int(p[0]), *map(float, p[1:])
                out.append({'class': MAP[c], 'box': [(x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h], 'h': bh * h})
    return out


rep = {}
for spec in args.folders:
    folder = Path(spec.removesuffix('@ours'))
    MAP = {0: 0, 1: 1, 2: 2, 3: 3} if spec.endswith('@ours') else TUTOR
    imgs = sorted(folder.glob('*.png')) + sorted(folder.glob('*.jpg'))
    S = {'images': len(imgs), 'targets': 0, 'tp': 0, 'fp': 0, 'agn': 0, 'shape_ok': 0, 'color_ok': 0, 'fn_h': [], 'tp_conf': [], 'fp_conf': [],
         'per_class': {n: {'targets': 0, 'tp': 0, 'fp': 0} for n in NAMES}}
    for img in imgs:
        lab = Path(str(img.parent).replace('/images', '/labels')) / (img.stem + '.txt')
        G = gts(img, lab)
        r = model.predict(str(img), conf=0.25, iou=0.45, imgsz=640, device=args.device, verbose=False)[0]
        D = [{'class': int(c), 'score': float(s), 'box': b.tolist()} for c, s, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
        S['targets'] += len(G)
        for g in G:
            S['per_class'][NAMES[g['class']]]['targets'] += 1
        used = set()
        for d in sorted(D, key=lambda d: -d['score']):
            cand = [(iou(d['box'], g['box']), j) for j, g in enumerate(G) if j not in used and g['class'] == d['class']]
            best = max(cand, default=(0, None))
            if best[0] >= 0.5:
                used.add(best[1]); S['tp'] += 1; S['tp_conf'].append(d['score']); S['per_class'][NAMES[d['class']]]['tp'] += 1
            else:
                S['fp'] += 1; S['fp_conf'].append(d['score']); S['per_class'][NAMES[d['class']]]['fp'] += 1
        S['fn_h'] += [G[j]['h'] for j in range(len(G)) if j not in used]
        used2 = set()   # 클래스 무관 짝(검출) → 그 안에서 모양 · 색
        for d in sorted(D, key=lambda d: -d['score']):
            cand = [(iou(d['box'], g['box']), j) for j, g in enumerate(G) if j not in used2]
            best = max(cand, default=(0, None))
            if best[0] >= 0.5:
                used2.add(best[1]); g = G[best[1]]; S['agn'] += 1
                S['shape_ok'] += (d['class'] // 2) == (g['class'] // 2); S['color_ok'] += (d['class'] % 2) == (g['class'] % 2)
    pc = lambda a: {k: round(float(np.percentile(a, p)), 3) for k, p in (('min', 0), ('p5', 5), ('p50', 50))} if a else {}
    rep[str(folder)] = {'images': S['images'], 'targets': S['targets'], 'tp': S['tp'], 'fp': S['fp'],
                        'recall': round(S['tp'] / max(S['targets'], 1), 4), 'precision': round(S['tp'] / max(S['tp'] + S['fp'], 1), 4),
                        'detected_any_class': S['agn'], 'shape_ratio': round(S['shape_ok'] / max(S['agn'], 1), 4), 'color_ratio': round(S['color_ok'] / max(S['agn'], 1), 4),
                        'tp_conf': pc(S['tp_conf']), 'fp_conf': pc(S['fp_conf']), 'missed_box_height_px': pc(S['fn_h']), 'per_class': S['per_class']}
    print(json.dumps({folder.name if folder.name not in ('test', 'val', 'train', 'images') else str(folder)[-60:]: rep[str(folder)]}, ensure_ascii=False), flush=True)
json.dump({'weights': str(W), 'folders': rep}, open(args.out, 'w'), ensure_ascii=False, indent=1)
