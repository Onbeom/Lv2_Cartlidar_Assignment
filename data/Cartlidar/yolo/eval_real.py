"""실물 사진 모양 평가 (YOLO, 2D): 사람이 고친 YOLO 라벨(<자료>/labels/<split>)과 모델 검출을 짝 지어 모양 · 색 · 검출 · 헛검출을 센다.

짝 = 검출 상자와 라벨 상자 IoU ≥ --iou(0.5), IoU 큰 쌍부터 하나씩. 검출은 신뢰도 ≥ --conf(0.25), NMS IoU --nms(0.45).
셈: 검출 = 짝 지은 퍽 / 라벨 퍽, 모양 · 색 = 짝 지은 퍽 중 맞은 수, 헛검출 = 짝 없는 검출. 묶음(파일 이름의 rNN)별 · 전체.
상자 위치(참고, 판정에 안 씀): 짝 지은 퍽의 IoU · 상자 중심 차이(px) · 아래 변 차이(px, 바닥 접점) 중앙값 · 90 %.
실물은 base_footprint 거리를 모르므로 거리로 거르지 않고 라벨이 있는 퍽을 모두 대상으로 한다(eval_r4_shape.py 와 다른 점).
색 · 모양은 번호가 아니라 이름('<색>_<모양>')으로 비교: 라벨은 자료 폴더 classes.txt(없으면 6클래스), 검출은 모델 names.
그래서 6클래스 모델을 4클래스 라벨로(또는 반대로) 평가해도 된다.

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/eval_real.py --model <best.pt> --data <자료 폴더> --split eval --out <결과 폴더>
결과: <out>/eval_real.json(장마다 짝 · 틀린 것 · 헛검출), eval_real.md(표)
"""
import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import cv2
from ultralytics import YOLO

import puck_classes as pc

ap = argparse.ArgumentParser()
ap.add_argument("--model", type=Path, required=True)
ap.add_argument("--data", type=Path, required=True)
ap.add_argument("--split", default="eval")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--conf", type=float, default=0.25)
ap.add_argument("--nms", type=float, default=0.45)
ap.add_argument("--iou", type=float, default=0.5)
ap.add_argument("--pass-shape", type=float, default=0.95, help="통과: 짝 지은 퍽 중 모양 비율 하한(사용자 승인 2026-10-26)")
ap.add_argument("--imgsz", type=int, default=640)
ap.add_argument("--device", default="0")
args = ap.parse_args()



def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


images = sorted((args.data / "images" / args.split).glob("*.png"))
if not images:
    raise SystemExit(f"{args.data}/images/{args.split} 에 사진이 없음")
model = YOLO(str(args.model))
DET = [pc.split(n) for n in pc.names_of(model)]  # 번호 → (색, 모양)
GT = [pc.split(n) for n in pc.read_names(args.data / "classes.txt")]
args.out.mkdir(parents=True, exist_ok=False)

cnt = defaultdict(Counter)
conf_mat = Counter()
loc = defaultdict(lambda: defaultdict(list))  # 묶음 → {iou, center, bottom} 값 목록
rows = []
for p in images:
    h, w = cv2.imread(str(p)).shape[:2]
    gts = []
    for ln in (args.data / "labels" / args.split / (p.stem + ".txt")).read_text().split("\n"):
        if ln.strip():
            k, x, y, bw, bh = ln.split()
            x, y, bw, bh = float(x) * w, float(y) * h, float(bw) * w, float(bh) * h
            gts.append((int(k), [x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2]))
    r = model.predict(str(p), imgsz=args.imgsz, conf=args.conf, iou=args.nms, device=args.device, verbose=False)[0]
    dets = [(b.tolist(), int(c), float(s)) for b, c, s in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.cls.cpu().numpy(),
                                                             r.boxes.conf.cpu().numpy())]
    pairs = sorted(((iou(d[0], g[1]), i, j) for i, d in enumerate(dets) for j, g in enumerate(gts)), reverse=True)
    used_d, used_g, matched = set(), set(), []
    for v, i, j in pairs:
        if v < args.iou:
            break
        if i in used_d or j in used_g:
            continue
        used_d.add(i)
        used_g.add(j)
        matched.append((i, j, v))
    m = re.search(r"(r\d\d)_", p.name)
    batch = m.group(1) if m else "?"
    wrong = []
    for i, j, v in matched:
        dk, gk = dets[i][1], gts[j][0]
        (dc, ds), (gc, gs) = DET[dk], GT[gk]
        d, g = dets[i][0], gts[j][1]
        for key in (batch, "all"):
            loc[key]["iou"].append(v)
            loc[key]["center"].append(((d[0] + d[2] - g[0] - g[2]) ** 2 + (d[1] + d[3] - g[1] - g[3]) ** 2) ** 0.5 / 2)
            loc[key]["bottom"].append(abs(d[3] - g[3]))
        conf_mat[(gs, ds)] += 1
        for c in (cnt[batch], cnt["all"], cnt[f"{batch}_{gc}"], cnt[f"all_{gc}"]):
            c["matched"] += 1
            c["shape_ok"] += ds == gs
            c["color_ok"] += dc == gc
        if (dc, ds) != (gc, gs):
            wrong.append({"truth": f"{gc}_{gs}", "det": f"{dc}_{ds}", "conf": round(dets[i][2], 3), "iou": round(v, 3), "box": gts[j][1]})
    fps = [{"cls": dets[i][1], "conf": round(dets[i][2], 3), "box": [round(x) for x in dets[i][0]]}
           for i in range(len(dets)) if i not in used_d]
    for c in (cnt[batch], cnt["all"]):
        c["targets"] += len(gts)
        c["images"] += 1
        c["fp"] += len(fps)
    for k, _ in gts:  # 색별 셈(빨강 · 초록 따로, ft2 승인 조건). 장 · 헛검출은 묶음 줄에만.
        for c in (cnt[f"{batch}_{GT[k][0]}"], cnt[f"all_{GT[k][0]}"]):
            c["targets"] += 1
    rows.append({"image": p.name, "batch": batch, "targets": len(gts), "dets": len(dets), "matched": len(matched),
                 "missed": [gts[j] for j in range(len(gts)) if j not in used_g], "wrong": wrong, "fp": fps})


def pct(a, b):
    return f"{a}/{b} ({100 * a / b:.1f} %)" if b else "-"


a = cnt["all"]
ok = a["matched"] > 0 and a["shape_ok"] / a["matched"] >= args.pass_shape
lines = [f"# 실물 모양 평가 — {args.model}", "",
         f"자료 `{args.data}` split `{args.split}`, conf ≥ {args.conf}, NMS {args.nms}, 짝 IoU ≥ {args.iou}. 거리로 거르지 않음.",
         f"**판정: {'통과' if ok else '실패'}** (짝 지은 퍽 중 모양 ≥ {args.pass_shape * 100:.0f} %)", "",
         "| 묶음(_색 = 그 색 라벨 퍽만) | 장 | 라벨 퍽 | 검출(짝) | 모양 | 색 | 헛검출(장당) |", "|---|---|---|---|---|---|---|"]
for k in sorted(cnt):
    c = cnt[k]
    lines.append(f"| {k} | {c['images'] or '-'} | {c['targets']} | {pct(c['matched'], c['targets'])} | {pct(c['shape_ok'], c['matched'])} | "
                 f"{pct(c['color_ok'], c['matched'])} | {f"{c['fp']} ({c['fp'] / c['images']:.2f})" if c['images'] else '-'} |")
lines += ["", "상자 위치(짝 지은 퍽, 참고 · 판정 아님): 중앙값 / 90 %", "",
          "| 묶음 | IoU | 중심 차이 px | 아래 변 차이 px |", "|---|---|---|---|"]
q = lambda xs, f: sorted(xs)[min(len(xs) - 1, int(f * len(xs)))]  # noqa: E731
for k in sorted(loc):
    L = loc[k]
    lines.append(f"| {k} | {q(L['iou'], 0.5):.3f} / {q(L['iou'], 0.1):.3f}(하위 10 %) | {q(L['center'], 0.5):.1f} / {q(L['center'], 0.9):.1f} | "
                 f"{q(L['bottom'], 0.5):.1f} / {q(L['bottom'], 0.9):.1f} |")
lines += ["", "모양 혼동(정답 → 검출): " + ", ".join(f"{t} → {d} {n}" for (t, d), n in sorted(conf_mat.items()))]
(args.out / "eval_real.md").write_text("\n".join(lines) + "\n")
(args.out / "eval_real.json").write_text(json.dumps({"args": {k: str(v) for k, v in vars(args).items()},
                                                     "counts": {k: dict(v) for k, v in cnt.items()},
                                                     "loc": {k: dict(v) for k, v in loc.items()},
                                                     "pass": ok, "images": rows}, ensure_ascii=False, indent=1))
print("\n".join(lines))
