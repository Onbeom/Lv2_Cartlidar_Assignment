"""노출 · gain 훑기 분석: exposure_sweep.py 결과에서 조합마다 (1) 켜진 신호등 칸이 제 색으로 잡히는지 (2) 퍽이 배경보다 밝고 색이 남는지 잰다.

관심 영역(rois.json)은 처음 한 번 마우스로 고른다(신호등 빨강 · 노랑 · 초록 칸 각 1개, 빨강 퍽들, 초록 퍽들). 카메라를 옮기지 않은 다른 훑기
(예: 낮조명 off)에는 --rois 로 같은 파일을 쓴다. 이미 rois.json 이 있으면 그대로 쓴다.

신호등: 칸마다 평균 V 를 재고, 같은 조합 안에서 그 칸의 최솟값보다 --lit-delta 이상 밝은 칸 중 가장 밝은 칸을 '켜짐'으로 본다.
  켜진 칸 안에서 팀 perception 신호등 규칙(signal_light.hpp: S ≥ s_min, V ≥ v_min, 색상 범위)으로 제 색 화소 · 다른 색 화소를 센다.
  하얗게 날아간 화소 = V ≥ v_min 이고 S < s_min.
퍽: 영역 가운데 60 % 와 둘레 띠(영역을 50 % 넓힌 것 − 10 % 넓힌 것)의 Lab 중앙값 거리(dE, 배경과 얼마나 다른가),
  가운데의 색 신호 크기(chroma = 화소마다 BGR 최대 − 최소의 중앙값, 0~255 — 어두우면 작아져 잡음에 묻힘), 제 색상 범위 화소 비율.
  둘레에 다른 퍽 · 상자가 들어가지 않게 퍽 사이를 퍽 폭 이상 띄워 둔다.
출력: <훑기 폴더>/summary.csv · summary.md(조합별 표), contact.jpg(조합마다 1장, 보기용).

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/analyze_sweep.py <훑기 폴더> [--rois <다른 폴더>/rois.json]
"""
import argparse
import csv
import json
import shutil
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("sweep", type=Path)
ap.add_argument("--rois", type=Path, help="다른 훑기에서 고른 rois.json(카메라 · 장면이 같을 때)")
ap.add_argument("--v-min", type=int, default=200, help="신호등 V 하한(팀 perception.yaml signal.v_min)")
ap.add_argument("--s-min", type=int, default=90, help="신호등 S 하한(signal.s_min)")
ap.add_argument("--lit-delta", type=float, default=15.0, help="켜짐으로 볼 칸 평균 V 증가")
ap.add_argument("--min-correct", type=int, default=30, help="(제안값, 확인 전) 신호등 통과: 켜진 칸 제 색 화소 중앙값 하한")
ap.add_argument("--max-wrong", type=float, default=0.1, help="(제안값, 확인 전) 신호등 통과: 다른 색 화소 / 제 색 화소 상한")
args = ap.parse_args()

# 팀 perception 신호등 색상 범위(OpenCV H 0~179) — 퍽 색 판단에도 같은 범위를 씀
HUE = {"red": lambda h: (h <= 10) | (h >= 170), "yellow": lambda h: (h >= 15) & (h <= 35),
       "green": lambda h: (h >= 40) & (h <= 90)}
LAMPS = ["red", "yellow", "green"]

rows = list(csv.DictReader(open(args.sweep / "frames.csv")))
if not rows:
    raise SystemExit("frames.csv 가 비어 있음")
roi_path = args.sweep / "rois.json"
if args.rois and not roi_path.exists():
    shutil.copy(args.rois, roi_path)
if not roi_path.exists():
    # 보기 좋은 중간 밝기 장으로 고름(화면만 밝힘)
    ref = sorted(rows, key=lambda r: abs(np.log(float(r["exposure"]) * float(r["gain"]) / 166 / 64)))[0]
    img = cv2.imread(str(args.sweep / "frames" / ref["file"]))
    show = cv2.resize(np.clip((img / 255.0) ** 0.5 * 255, 0, 255).astype(np.uint8), None, fx=2, fy=2)
    rois = {}
    for lamp in LAMPS:
        print(f"신호등 {lamp} 칸 하나를 끌어서 고르고 Enter")
        x, y, w, h = cv2.selectROI(f"signal {lamp} segment", show, showCrosshair=False)
        cv2.destroyAllWindows()
        rois[f"lamp_{lamp}"] = [x // 2, y // 2, w // 2, h // 2]
    for col in ("red", "green"):
        print(f"{col} 퍽을 하나씩 끌어 고르고 Enter(퍽마다), 다 고르면 Esc. 없으면 바로 Esc")
        sel = cv2.selectROIs(f"{col} pucks", show, showCrosshair=False)
        cv2.destroyAllWindows()
        rois[f"puck_{col}"] = [[int(v) // 2 for v in r] for r in sel] if len(sel) else []
    json.dump(rois, open(roi_path, "w"), indent=1)
rois = json.load(open(roi_path))


def crop(a, r, pad=0.0):
    x, y, w, h = r
    dx, dy = int(round(w * pad)), int(round(h * pad))
    x0, y0 = max(0, x - dx), max(0, y - dy)
    return a[y0:y + h + dy, x0:x + w + dx], (x0, y0)


def lamp_counts(hsv, r):
    c, _ = crop(hsv, r)
    h, s, v = c[..., 0].astype(int), c[..., 1], c[..., 2]
    bright = (s >= args.s_min) & (v >= args.v_min)
    n = {k: int((HUE[k](h) & bright).sum()) for k in LAMPS}
    n["white"] = int(((v >= args.v_min) & (s < args.s_min)).sum())
    return n


def puck_stats(img, lab, hsv, r, col):
    x, y, w, h = r
    ys, xs = slice(y + int(h * 0.2), y + h - int(h * 0.2)), slice(x + int(w * 0.2), x + w - int(w * 0.2))
    big, (bx, by) = crop(lab, r, 0.5)
    mask = np.ones(big.shape[:2], bool)
    sx, sy = int(round(w * 0.1)), int(round(h * 0.1))
    mask[max(0, y - sy - by):y + h + sy - by, max(0, x - sx - bx):x + w + sx - bx] = False
    inner_lab = lab[ys, xs].reshape(-1, 3).astype(float)
    bgr = img[ys, xs].reshape(-1, 3).astype(int)
    return {"dE": float(np.linalg.norm(np.median(inner_lab, 0) - np.median(big[mask].astype(float), 0))),
            "chroma": float(np.median(bgr.max(1) - bgr.min(1))),
            "hue_ok": float(HUE[col](hsv[ys, xs].reshape(-1, 3)[:, 0].astype(int)).mean())}


per = defaultdict(list)  # (노출, gain) → 장별 결과
for r in rows:
    img = cv2.imread(str(args.sweep / "frames" / r["file"]))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    meanv = {k: float(crop(hsv, rois[f"lamp_{k}"])[0][..., 2].mean()) for k in LAMPS}
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    pucks = [(col, puck_stats(img, lab, hsv, pr, col)) for col in ("red", "green") for pr in rois.get(f"puck_{col}", [])]
    per[(float(r["exposure"]), float(r["gain"]))].append(
        {"file": r["file"], "meanv": meanv, "counts": {k: lamp_counts(hsv, rois[f"lamp_{k}"]) for k in LAMPS},
         "pucks": pucks})

out_rows = []
for (e, g), fr in sorted(per.items()):
    base = {k: min(f["meanv"][k] for f in fr) for k in LAMPS}
    row = {"exposure": e, "gain": g, "frames": len(fr)}
    sig_ok = True
    for lamp in LAMPS:
        lit = []
        for f in fr:
            d = {k: f["meanv"][k] - base[k] for k in LAMPS}
            top = max(d, key=d.get)
            if top == lamp and d[top] >= args.lit_delta:
                c = f["counts"][lamp]
                lit.append((c[lamp], sum(c[k] for k in LAMPS if k != lamp), c["white"]))
        row[f"{lamp}_lit_frames"] = len(lit)
        if lit:
            cor, wr, wh = (float(np.median([t[i] for t in lit])) for i in range(3))
            row.update({f"{lamp}_correct": cor, f"{lamp}_wrong": wr, f"{lamp}_white": wh})
            sig_ok &= cor >= args.min_correct and wr <= args.max_wrong * max(cor, 1)
        else:
            row.update({f"{lamp}_correct": "", f"{lamp}_wrong": "", f"{lamp}_white": ""})
    row["signal_ok(제안 기준)"] = sig_ok if any(row[f"{k}_lit_frames"] for k in LAMPS) else ""
    for col in ("red", "green"):
        n = len(rois.get(f"puck_{col}", []))
        if not n:
            continue
        # 퍽마다 장 중앙값 → 가장 나쁜 퍽
        med = [{k: float(np.median([f["pucks"][i][1][k] for f in fr if f["pucks"][i][0] == col] or [np.nan]))
                for k in ("dE", "chroma", "hue_ok")}
               for i in range(len(fr[0]["pucks"])) if fr[0]["pucks"][i][0] == col]
        row[f"{col}_puck_min_dE"] = round(min(m["dE"] for m in med), 1)
        row[f"{col}_puck_min_chroma"] = round(min(m["chroma"] for m in med), 1)
        row[f"{col}_puck_min_hue_ok"] = round(min(m["hue_ok"] for m in med), 2)
    out_rows.append(row)

keys = list(dict.fromkeys(k for r in out_rows for k in r))
with open(args.sweep / "summary.csv", "w", newline="") as fp:
    w = csv.DictWriter(fp, keys)
    w.writeheader()
    w.writerows(out_rows)
md = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
md += ["| " + " | ".join(str(r.get(k, "")) for k in keys) + " |" for r in out_rows]
(args.sweep / "summary.md").write_text(
    f"# 노출 · gain 훑기 요약 — {args.sweep.name}\n\n신호등 규칙 v_min {args.v_min} · s_min {args.s_min}, 켜짐 ΔV ≥ {args.lit_delta}. "
    f"signal_ok 는 제안 기준(제 색 중앙값 ≥ {args.min_correct}, 다른 색 ≤ {args.max_wrong} × 제 색) — 확인 전.\n"
    "퍽 값은 퍽마다 장 중앙값을 낸 뒤 가장 나쁜 퍽(min).\n\n" + "\n".join(md) + "\n")

# 보기용 모음: 조합마다 첫 장, 행 = 노출, 열 = gain
es = sorted({k[0] for k in per})
gs = sorted({k[1] for k in per})
tw, th = 213, 160
sheet = np.full((th * len(es), tw * len(gs), 3), 40, np.uint8)
for i, e in enumerate(es):
    for j, g in enumerate(gs):
        if (e, g) in per:
            t = cv2.resize(cv2.imread(str(args.sweep / "frames" / per[(e, g)][0]["file"])), (tw, th))
            cv2.putText(t, f"e{e:g} g{g:g}", (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            sheet[i * th:(i + 1) * th, j * tw:(j + 1) * tw] = t
cv2.imwrite(str(args.sweep / "contact.jpg"), sheet)
print(f"조합 {len(out_rows)}개 → {args.sweep}/summary.md · summary.csv · contact.jpg")
