"""도구 확인(판정 아님): 후보 D 상자 신뢰도를 문턱 0.5 · 0.6 으로 자를 때 맞힌 상자(TP) · 헛상자(FP, 클래스 틀림 포함) 수.
짝 규칙은 261102_1141_tutor_yolo/zeroshot_b.py 와 같음(IoU ≥ 0.5, 같은 클래스, conf ≥ 0.25 · NMS 0.45). CPU."""
import json
import os
from pathlib import Path
os.environ['YOLO_OFFLINE'] = 'true'
import torch
from PIL import Image
from ultralytics import YOLO

ROOT = Path('/home/pa4/git/lv2_next_module')
W = ROOT / '261102_1634_yolo_D/yolo_export/yolo11n_task4_D.pt'
FOLDERS = {'real_203': ROOT / '261030_1325/B_negative/images/eval', 'sim_240': ROOT / '261030_1325/B_negative/images/sim_eval'}
torch.set_num_threads(4); model = YOLO(str(W))


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0.0


out = {}
for name, folder in FOLDERS.items():
    tp, fp, n_gt = [], [], 0
    for img in sorted(folder.glob('*.png')):
        w, h = Image.open(img).size; G = []
        lab = Path(str(img.parent).replace('/images', '/labels')) / (img.stem + '.txt')
        for line in (lab.read_text().split('\n') if lab.exists() else []):
            p = line.split()
            if len(p) == 5:
                c, x, y, bw, bh = int(p[0]), *map(float, p[1:]); G.append((c, [(x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h]))
        n_gt += len(G)
        r = model.predict(str(img), conf=0.25, iou=0.45, imgsz=640, device='cpu', verbose=False)[0]
        used = set()
        for c, s, b in sorted(zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()), key=lambda t: -t[1]):
            best = max(((iou(b, g[1]), j) for j, g in enumerate(G) if j not in used and g[0] == int(c)), default=(0, None))
            if best[0] >= 0.5: used.add(best[1]); tp.append(round(s, 3))
            else: fp.append({'image': img.name, 'class': int(c), 'score': round(s, 3)})
    out[name] = {'targets': n_gt, 'tp_all': len(tp), 'fp_all': len(fp),
                 **{f'th{t}': {'tp': sum(s >= t for s in tp), 'fp': sum(f['score'] >= t for f in fp)} for t in (0.5, 0.6)},
                 'tp_scores_below_0.6': sorted(s for s in tp if s < 0.6), 'fp_scores_ge_0.5': sorted((f for f in fp if f['score'] >= 0.5), key=lambda f: -f['score'])}
(Path(__file__).with_name('threshold_counts.json')).write_text(json.dumps(out, ensure_ascii=False, indent=1))
print(json.dumps({k: {kk: v[kk] for kk in ('targets', 'tp_all', 'fp_all', 'th0.5', 'th0.6')} for k, v in out.items()}, ensure_ascii=False))
