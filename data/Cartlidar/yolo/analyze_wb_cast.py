"""WB 자동 / 고정 비교: 무채색 면(벽판 · 바닥)에서 채널 비율(B/G, R/G)을 WB 모드 · 노출별로 잰다.

무채색 면이면 WB 가 맞을 때 R/G · B/G 가 1 가까이 나온다. 신호등 · 퍽 영역이 화면에 없어도 WB 가 빨강 채널을 얼마나 줄이는지는 볼 수 있다.
입력: frames.csv(file, exposure, gain, wb_mode) 와 같은 폴더의 PNG. 영역은 --roi 이름=y0,y1,x0,x1 로 여러 개(기본은 wbtest_rear 화면 기준).

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/analyze_wb_cast.py <폴더> [--roi wall=5,50,10,120 --roi carpet=400,470,60,300]
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("folder", type=Path)
ap.add_argument("--roi", action="append", default=[], help="이름=y0,y1,x0,x1")
args = ap.parse_args()
rois = dict(r.split("=") for r in args.roi) or {"wall": "5,50,10,120", "carpet": "400,470,60,300"}
rois = {k: [int(v) for v in s.split(",")] for k, s in rois.items()}

per = defaultdict(list)  # (노출, gain, wb, 영역) → 장별 (B, G, R) 중앙값
for r in csv.DictReader(open(args.folder / "frames.csv")):
    img = cv2.imread(str(args.folder / r["file"])).astype(float)
    key = (float(r["exposure"]), float(r["gain"]), r["wb_mode"])
    for name, (y0, y1, x0, x1) in rois.items():
        per[key + (name,)].append(np.median(img[y0:y1, x0:x1].reshape(-1, 3), 0))
    per[key + ("whole",)].append(img.reshape(-1, 3).mean(0))

lines = ["| 노출 | gain | WB | 영역 | 장 | B · G · R 중앙값 | B/G | R/G | R/G 10~90 백분위 |", "|---|---|---|---|---|---|---|---|---|"]
for (e, g, wb, name), v in sorted(per.items()):
    v = np.array(v)
    m = np.median(v, 0)
    lo, hi = np.percentile(v[:, 2] / np.maximum(v[:, 1], 1e-6), [10, 90])
    lines.append(f"| {e:g} | {g:g} | {wb} | {name} | {len(v)} | {m[0]:.0f} · {m[1]:.0f} · {m[2]:.0f} | "
                 f"{m[0] / m[1]:.2f} | {m[2] / m[1]:.2f} | {lo:.2f}~{hi:.2f} |")
print("\n".join(lines))
