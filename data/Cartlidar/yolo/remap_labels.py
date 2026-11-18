"""YOLO 라벨 6클래스 → 4클래스 번호 바꾸기(사용자 결정 2026-10-26). 원본은 그대로 두고 새 폴더에 쓴다.

0 → 0 red_cylinder · 1 → 1 green_cylinder · 3 → 2 red_cuboid · 4 → 3 green_cuboid. 파랑(2 · 5) 번호가 있으면 멈춘다.
출력 폴더에 같은 이름의 .txt 와 classes.txt(4클래스 이름)를 만든다.

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/remap_labels.py <6클래스 라벨 폴더> <새 4클래스 라벨 폴더>
"""
import argparse
from pathlib import Path

from puck_classes import NAMES4, remap_lines

ap = argparse.ArgumentParser()
ap.add_argument("src", type=Path)
ap.add_argument("dst", type=Path)
args = ap.parse_args()

files = sorted(p for p in args.src.glob("*.txt") if p.name != "classes.txt")
if not files:
    raise SystemExit(f"{args.src} 에 라벨이 없음")
args.dst.mkdir(parents=True, exist_ok=False)
for p in files:
    lines = remap_lines(p.read_text().split("\n"))
    (args.dst / p.name).write_text("\n".join(lines) + ("\n" if lines else ""))
(args.dst / "classes.txt").write_text("\n".join(NAMES4) + "\n")
print(f"{len(files)}개 → {args.dst} (4클래스)")
