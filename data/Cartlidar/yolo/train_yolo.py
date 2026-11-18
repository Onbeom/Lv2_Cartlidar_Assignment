"""YOLO11n 퍽 학습: render_yolo.py 자료(images/{train,val}, labels/{train,val})로 학습한다. 평가 자료(eval)는 쓰지 않는다.

클래스 이름은 자료 폴더의 classes.txt 를 따르고, 없으면 6클래스(번호 = 모양 × 3 + 색, render_yolo.py 와 같음). 4클래스는 puck_classes.py. 증강은 ultralytics 기본값(색상 hsv_h 0.015 는 빨강 · 초록을 넘기지 않음).
실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/train_yolo.py --data <자료 폴더> --project <결과 폴더> --name <이름>
결과: <project>/<name>/weights/best.pt · last.pt, results.csv, args.yaml, data.yaml(자료 폴더 안)
"""
import argparse
from pathlib import Path

from ultralytics import YOLO

import puck_classes as pc

ap = argparse.ArgumentParser()
ap.add_argument("--data", type=Path, required=True)
ap.add_argument("--project", type=Path, required=True)
ap.add_argument("--name", required=True)
ap.add_argument("--model", default="yolo11n.pt")
ap.add_argument("--epochs", type=int, default=80)
ap.add_argument("--batch", type=int, default=16)
ap.add_argument("--imgsz", type=int, default=640)
ap.add_argument("--patience", type=int, default=20)
ap.add_argument("--workers", type=int, default=4)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--device", default="0")
args = ap.parse_args()

names = pc.read_names(args.data / "classes.txt")
yaml = args.data.resolve() / "data.yaml"
yaml.write_text(f"path: {args.data.resolve()}\ntrain: images/train\nval: images/val\nnames:\n"
                + "".join(f"  {i}: {n}\n" for i, n in enumerate(names)))
model = YOLO(args.model)
model.train(data=str(yaml), epochs=args.epochs, batch=args.batch, imgsz=args.imgsz, patience=args.patience,
            workers=args.workers, seed=args.seed, deterministic=True, device=args.device,
            project=str(args.project.resolve()), name=args.name, exist_ok=False, plots=True)
