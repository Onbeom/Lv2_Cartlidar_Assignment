"""(11-04 E3 팔 근접 학습 영상용 사본: 초안 모델 E2, 100장 미만 영상 뺌. 그 전 사본: 다시 찍은 앞 영상 _f2 도 앞으로 셈)
새 판정 영상(2026-11-03 촬영, 학습에 안 쓴 장면)에서 초당 1장을 뽑아 라벨 초안을 만든다(후보 D, CPU). 형식은 fix_labels.py 와 같음(conf 0.10, NMS 0.45).
- D435(record_signal.py 폴더: capture.json · frames.csv · color/ · depth_aligned/): color_timestamp_ms 기준 매 1 초 첫 장. 컬러는 하드 링크, 깊이 경로는 auto_label.json 'depth' 에 적음(판정 때 거리).
- Arducam(capture_arducam.py 폴더: <시각>_<label>/arm/frames/*.jpg · frames.csv): host_elapsed_s 기준 매 1 초 첫 장. JPEG 를 그대로 PNG 로 바꿔 저장(무손실, 화소 같음).
- 이름 = <label>__<번호3>_<원본 번호6>.png, 묶음(batch) = 영상 label(front_v1 …). 사람 = 배경(상자 없음), 얼굴 있는 장은 팀 저장소에 넣지 않음.
사용: video_draft.py <영상 폴더(video_2026110x)>"""
import csv, json, os, re, sys
from pathlib import Path
import cv2
from ultralytics import YOLO

ROOT = Path('/home/pa4/git/lv2_next_module'); HERE = Path(__file__).resolve().parent
MODEL = ROOT / '261103_1724_E2/runs/E2/weights/last.pt'  # 11-04 E3 팔 학습 영상 초안은 E2 로
NAMES4 = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']
SRC = Path(sys.argv[1])
PREFIX = sys.argv[2] if len(sys.argv) > 2 else 'video'  # 출력 폴더 앞부분(11-03 커튼 판정 영상은 judge)
model = YOLO(str(MODEL)); assert [model.names[i] for i in range(4)] == NAMES4


STEP = float(os.environ.get('SAMPLE_SEC', '1'))  # 몇 초마다 한 장(11-03 E2 학습 영상은 2초)


def pick(rows, key):
    t0 = float(rows[0][key]); out, nxt = [], 0.0
    for r in rows:
        t = float(r[key]) - t0
        if t >= nxt:
            out.append(r); nxt = (int(t / STEP) + 1) * STEP
    return out, t0


sets = {f'{PREFIX}_front': [], f'{PREFIX}_rear': [], f'{PREFIX}_arm': []}
for d in sorted(p for p in SRC.iterdir() if p.is_dir()):
    if (d / 'capture.json').exists():
        cap = json.load(open(d / 'capture.json')); lab = cap['label']
        rows = list(csv.DictReader(open(d / 'frames.csv')))
        rows = [r for r in rows if (d / 'color' / f'{int(r["index"]):06d}.png').exists()]
        rows = [dict(r, t_s=float(r['color_timestamp_ms']) / 1000.0) for r in rows]  # ms → s
        sel, _ = pick(rows, 't_s'); kind = f'{PREFIX}_front' if (lab.startswith('front') or re.search(r'_f\d*$', lab)) else f'{PREFIX}_rear'
        for r in sel:
            i = int(r['index']); sets[kind].append((lab, i, d / 'color' / f'{i:06d}.png', d / 'depth_aligned' / f'{i:06d}.png', float(r['color_timestamp_ms'])))
    elif (d / 'arm' / 'frames.csv').exists() and json.load(open(d / 'arm' / 'metadata.json'))['frames'] >= 100:  # 11-04: 실수로 다시 시작된 1초 미만 영상(19 · 22장) 뺌
        lab = d.name.split('_', 2)[2]; a = d / 'arm'
        rows = [r for r in csv.DictReader(open(a / 'frames.csv')) if (a / 'frames' / f'{int(r["index"]):06d}.jpg').exists()]
        sel, _ = pick(rows, 'host_elapsed_s')
        for r in sel:
            i = int(r['index']); sets[f'{PREFIX}_arm'].append((lab, i, a / 'frames' / f'{i:06d}.jpg', None, float(r['host_elapsed_s'])))
for name, items in sets.items():
    out = HERE / name
    if not items:
        continue
    if out.exists():
        print('있음, 건너뜀', name); continue
    (out / 'images').mkdir(parents=True); (out / 'labels').mkdir(); (out / 'classes.txt').write_text('\n'.join(NAMES4) + '\n')
    log = []
    for n, (lab, i, f, dep, t) in enumerate(items, 1):
        img = f'{lab}__{n:03d}_{i:06d}.png'
        if f.suffix == '.png':
            os.link(f, out / 'images' / img)
        else:
            cv2.imwrite(str(out / 'images' / img), cv2.imread(str(f)))
        r = model.predict(str(out / 'images' / img), conf=0.10, iou=0.45, device='cpu', verbose=False)[0]
        lines = [f'{int(c)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}' for (x, y, w, h), c in zip(r.boxes.xywhn.numpy(), r.boxes.cls.numpy())]
        (out / 'labels' / f'{Path(img).stem}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
        log.append({'image': img, 'source': str(f), 'depth': str(dep) if dep else None, 'time': t, 'batch': lab, 'dets': len(lines),
                    'conf': [round(float(s), 3) for s in r.boxes.conf.numpy()], 'classes': [int(c) for c in r.boxes.cls.numpy()], 'checked': False})
    (out / 'auto_label.json').write_text(json.dumps(log, indent=1, ensure_ascii=False))
    print(f'{name}: {len(log)}장, 상자 {sum(e["dets"] for e in log)}개, 묶음 {sorted(set(e["batch"] for e in log))}', flush=True)
