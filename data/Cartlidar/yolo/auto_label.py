"""실물 사진 자동 라벨(초안): 시뮬레이션으로 학습한 모델로 color.png 마다 YOLO 라벨 초안을 만든다. 사람이 고친 뒤 학습에 쓴다.

입력: capture_realsense.py 결과(<촬영 폴더>/<시각>/<카메라>/color.png) 또는 png · jpg 가 든 아무 폴더(하위 폴더까지 찾음).
출력: <out>/images/<묶음>__<상대 경로를 _ 로 이은 이름>.png(원본 복사), <out>/labels/<같은 이름>.txt(YOLO 형식, 초안),
      <out>/classes.txt(라벨 도구용 클래스 이름), <out>/auto_label.json(장마다 검출 수 · 신뢰도, 확인 안 됨 표시).
  '묶음'(--batch)은 촬영 묶음 이름. 학습 · 검증 · 시험을 나눌 때 같은 묶음이 섞이지 않게 이 이름으로 나눈다.
신뢰도 문턱은 일부러 낮게(기본 0.10) 두어 놓친 퍽보다 지울 상자가 많게 한다(지우기가 새로 그리기보다 빠름).

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/auto_label.py --model <best.pt> --src <촬영 폴더> --batch <묶음 이름> --out <라벨 폴더> [--to4]
클래스 번호는 모델을 따르고, 6클래스 모델로 4클래스 라벨을 만들 때는 --to4(번호 = 모양 × 2 + 색, puck_classes.py).
"""
import argparse
import json
import shutil
from pathlib import Path

from ultralytics import YOLO

import puck_classes as pc

ap = argparse.ArgumentParser()
ap.add_argument("--model", type=Path, required=True)
ap.add_argument("--src", type=Path, required=True)
ap.add_argument("--batch", required=True)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--conf", type=float, default=0.10)
ap.add_argument("--nms", type=float, default=0.45)
ap.add_argument("--only-color-png", action="store_true", help="capture_realsense.py 결과에서 color.png 만 쓴다")
ap.add_argument("--device", default="0")
ap.add_argument("--to4", action="store_true",
                help="6클래스 모델(sim_v1 · ft1)의 검출을 4클래스 번호로 바꿔 적음(파랑 검출은 버림, 사용자 결정 2026-10-26)")
args = ap.parse_args()

pats = ["color.png"] if args.only_color_png else ["*.png", "*.jpg", "*.jpeg"]
files = sorted({f for p in pats for f in args.src.rglob(p) if "depth" not in f.name})
if not files:
    raise SystemExit(f"{args.src} 에 영상이 없음")
(args.out / "images").mkdir(parents=True, exist_ok=True)
(args.out / "labels").mkdir(parents=True, exist_ok=True)
model = YOLO(str(args.model))
names = pc.names_of(model)
if args.to4:
    if names != pc.NAMES6:
        raise SystemExit(f"--to4 는 6클래스 모델에만: {names}")
    names = pc.NAMES4
if (args.out / "classes.txt").exists() and pc.read_names(args.out / "classes.txt") != names:
    raise SystemExit(f"{args.out}/classes.txt 와 클래스 번호가 다름 — 다른 라벨 폴더를 쓸 것")
(args.out / "classes.txt").write_text("\n".join(names) + "\n")
log = []
for f in files:
    name = f"{args.batch}__" + "_".join(f.relative_to(args.src).with_suffix("").parts)
    dst = args.out / "images" / f"{name}{f.suffix.lower()}"
    if dst.exists():
        raise SystemExit(f"{dst} 이 이미 있음 — 묶음 이름이 겹치지 않게")
    shutil.copy2(f, dst)
    r = model.predict(str(f), conf=args.conf, iou=args.nms, device=args.device, verbose=False)[0]
    lines, confs = [], []
    for (x, y, w, h), c, s in zip(r.boxes.xywhn.cpu().numpy(), r.boxes.cls.cpu().numpy(), r.boxes.conf.cpu().numpy()):
        if args.to4:
            if int(c) not in pc.REMAP_6_TO_4:
                continue
            c = pc.REMAP_6_TO_4[int(c)]
        lines.append(f"{int(c)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
        confs.append(round(float(s), 3))
    (args.out / "labels" / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
    log.append({"image": dst.name, "source": str(f), "batch": args.batch, "dets": len(lines), "conf": confs,
                "checked": False})
prev = json.loads((args.out / "auto_label.json").read_text()) if (args.out / "auto_label.json").exists() else []
(args.out / "auto_label.json").write_text(json.dumps(prev + log, indent=1, ensure_ascii=False))
print(f"{len(files)}장, 상자 {sum(e['dets'] for e in log)}개 (초안, 사람 확인 필요) → {args.out}")
