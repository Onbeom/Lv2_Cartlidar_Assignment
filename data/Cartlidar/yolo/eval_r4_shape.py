"""R4 모양 판정 (YOLO, 2D 영상에서): render_yolo.py 평가 자료(truth_eval_*.json)에 학습한 모델을 돌려 정답 퍽과 짝 짓는다.

대상 = R4 와 같게 렌더 깊이로 겉면이 --min-vis(0.5) 이상 보이고 base_footprint 에서 수평 --max-dist(1.0 m) 이내인 퍽. 선반 · 바닥 따로 셈.
짝 = 검출 상자와 정답 상자(인스턴스 분할로 본 보이는 화소의 상자) IoU ≥ --iou(0.5), IoU 큰 쌍부터 하나씩(대상이 아닌 퍽도 짝 후보에
  넣어 그 퍽에 붙은 검출이 헛검출로 세지지 않게 한다). 검출은 신뢰도 ≥ --conf, NMS IoU --nms.
셈: 검출 = 짝 지은 대상 / 대상, 모양 · 색 · 클래스 = 짝 지은 대상 중 맞은 수. 헛검출 = 어느 퍽과도 짝이 안 된 검출(기록).
클래스 번호 = 모양 × 3 + 색(색 0 red · 1 green · 2 blue, 모양 0 cylinder · 1 cuboid), render_yolo.py 와 같음.

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/eval_r4_shape.py --model <best.pt> --data <자료 폴더> --out <결과 폴더>
결과: <out>/eval_r4.json(장마다 짝 · 틀린 것), eval_r4.md(표)
"""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--model", type=Path, required=True)
ap.add_argument("--data", type=Path, required=True)
ap.add_argument("--split", default="eval")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--conf", type=float, default=0.25)
ap.add_argument("--nms", type=float, default=0.45)
ap.add_argument("--iou", type=float, default=0.5)
ap.add_argument("--min-vis", type=float, default=0.5)
ap.add_argument("--max-dist", type=float, default=1.0)
ap.add_argument("--imgsz", type=int, default=640)
ap.add_argument("--device", default="0")
args = ap.parse_args()

SHAPES, COLORS = ["cylinder", "cuboid"], ["red", "green", "blue"]


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


truths = sorted(args.data.glob(f"truth_{args.split}_*.json"))
if not truths:
    raise SystemExit(f"{args.data} 에 truth_{args.split}_*.json 이 없음")
images = [im for t in truths for im in json.loads(t.read_text())["images"]]
model = YOLO(str(args.model))
args.out.mkdir(parents=True, exist_ok=False)

cnt = defaultdict(Counter)    # 묶음 → 대상 · 짝 · 모양 · 색 · 클래스
conf_mat = defaultdict(Counter)
fp_total, rows, misses = 0, [], []
for im in images:
    r = model.predict(str(args.data / im["image"]), imgsz=args.imgsz, conf=args.conf, iou=args.nms, device=args.device,
                      verbose=False)[0]
    dets = [(b.tolist(), int(c), float(s)) for b, c, s in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.cls.cpu().numpy(),
                                                             r.boxes.conf.cpu().numpy())]
    gts = [p for p in im["pucks"] if p.get("bbox")]
    pairs = sorted(((iou(d[0], g["bbox"]), i, j) for i, d in enumerate(dets) for j, g in enumerate(gts)), reverse=True)
    used_d, used_g, match = set(), set(), {}
    for v, i, j in pairs:
        if v < args.iou:
            break
        if i in used_d or j in used_g:
            continue
        used_d.add(i)
        used_g.add(j)
        match[j] = i
    fp = len(dets) - len(used_d)
    fp_total += fp
    for j, g in enumerate(gts):
        inside = g["base_hdist_m"] <= args.max_dist
        if g["visible"] < args.min_vis:
            continue
        key = f"{g['location']}_{'le' if inside else 'gt'}{args.max_dist:g}m"
        c = cnt[key]
        c["target"] += 1
        if j not in match:
            misses.append({"image": im["name"], "puck": g["name"], "key": key, "visible": g["visible"],
                           "base_hdist_m": g["base_hdist_m"], "px": g["px"], "shape": g["shape"], "color": g["color"]})
            continue
        pc = dets[match[j]][1]
        c["matched"] += 1
        c["shape_ok"] += SHAPES[pc // 3] == g["shape"]
        c["color_ok"] += COLORS[pc % 3] == g["color"]
        c["class_ok"] += pc == g["cls"]
        if inside:
            conf_mat[g["location"]][(g["shape"], SHAPES[pc // 3])] += 1
        if SHAPES[pc // 3] != g["shape"] or COLORS[pc % 3] != g["color"]:
            misses.append({"image": im["name"], "puck": g["name"], "key": key, "wrong": True, "pred": pc, "gt": g["cls"],
                           "conf": dets[match[j]][2], "visible": g["visible"], "base_hdist_m": g["base_hdist_m"],
                           "px": g["px"], "standing": g["standing"]})
    rows.append({"image": im["name"], "dets": len(dets), "false_pos": fp})

res = {"model": str(args.model), "data": str(args.data), "truth_files": [t.name for t in truths], "images": len(images),
       "params": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
       "counts": {k: dict(v) for k, v in cnt.items()}, "false_pos": fp_total,
       "shape_confusion_le": {loc: {f"{a}->{b}": n for (a, b), n in m.items()} for loc, m in conf_mat.items()},
       "misses": misses, "per_image": rows}
(args.out / "eval_r4.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))


def pct(a, b):
    return f"{a}/{b} = {100 * a / b:.1f} %" if b else "-"


lines = [f"모델 `{args.model}`, 자료 `{args.data}` ({len(images)}장), conf ≥ {args.conf}, NMS {args.nms}, 짝 IoU ≥ {args.iou}, "
         f"대상 보임 ≥ {args.min_vis}", "", "| 묶음 | 대상 | 검출 | 모양(짝) | 색(짝) | 클래스(짝) |", "|---|---|---|---|---|---|"]
for k in sorted(cnt):
    c = cnt[k]
    lines.append(f"| {k} | {c['target']} | {pct(c['matched'], c['target'])} | {pct(c['shape_ok'], c['matched'])} | "
                 f"{pct(c['color_ok'], c['matched'])} | {pct(c['class_ok'], c['matched'])} |")
lines += ["", f"헛검출(어느 퍽과도 짝 안 됨): {fp_total}", "",
          f"모양 혼동(1.0 m 이내, 정답->예측): {json.dumps(res['shape_confusion_le'], ensure_ascii=False)}"]
(args.out / "eval_r4.md").write_text("\n".join(lines) + "\n")
print("\n".join(lines))
