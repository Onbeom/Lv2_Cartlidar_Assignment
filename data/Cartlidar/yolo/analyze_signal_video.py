"""노출 · WB 비교 영상 분석: run_signal_video.sh 로 찍은 영상 폴더 여러 개(조합당 1개)를 같은 관심 영역으로 재어 조합끼리 비교한다.

영상 폴더 = Cartlidar/video/<시각>/ (color/*.png, frames.csv, capture.json). 노출 · gain · WB 자동 여부 · 시리얼 · label 은 capture.json 에서 읽는다.
관심 영역(rois.json)은 처음 한 번 마우스로 고른다(신호등 빨강 · 노랑 · 초록 칸 각 1개, 빨강 퍽들, 초록 퍽들, 무채색 면 1개 — 벽판 · 바닥 등).
카메라와 장면을 고정하고 찍었으면 모든 영상에 같은 rois.json 을 쓴다(--rois). 고를 때 화면에 신호등 세 칸이 없으면 멈춘다.

신호등(analyze_sweep.py 와 같은 규칙): 칸마다 평균 V 를 재고, 그 영상 안에서 그 칸의 최솟값보다 --lit-delta 이상 밝은 칸 중 가장 밝은 칸을 '켜짐'으로 본다.
  켜진 칸 안에서 팀 perception 신호등 규칙(signal_light.hpp: S ≥ s_min, V ≥ v_min, 색상 범위)으로 제 색 · 다른 색 · 흰(V ≥ v_min, S < s_min) 화소를 센다.
  켜짐 장들의 중앙값으로 조합을 비교하고, 제 색 < 다른 색인 장(섞임 장) 수도 센다.
퍽: 영역 가운데 60 % 의 chroma(화소마다 BGR 최대 − 최소의 중앙값) · 제 색상 범위 비율, 둘레 띠와의 Lab 중앙값 거리(dE).
무채색 면: B/G · R/G 중앙값(WB 가 맞으면 1 가까이, analyze_wb_cast.py 와 같은 뜻).
출력: <out>/summary.md · summary.csv(조합별), frames.csv(장별), check.jpg(영상마다 가운데 장 + 영역).

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/analyze_signal_video.py <영상 폴더> [<영상 폴더> ...] --out <결과 폴더> [--rois rois.json] [--step 3]
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("videos", type=Path, nargs="+")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--rois", type=Path, help="이미 고른 rois.json(카메라 · 장면이 같을 때). 없으면 첫 영상에서 고름")
ap.add_argument("--step", type=int, default=3, help="몇 장마다 하나씩 잴지(30 FPS 에서 3 = 초당 10장)")
ap.add_argument("--v-min", type=int, default=200, help="신호등 V 하한(팀 perception.yaml signal.v_min)")
ap.add_argument("--s-min", type=int, default=90, help="신호등 S 하한(signal.s_min)")
ap.add_argument("--lit-delta", type=float, default=15.0, help="켜짐으로 볼 칸 평균 V 증가")
ap.add_argument("--min-correct", type=int, default=30, help="(제안값, 확인 전) 켜진 칸 제 색 화소 중앙값 하한")
ap.add_argument("--max-wrong", type=float, default=0.1, help="(제안값, 확인 전) 다른 색 화소 / 제 색 화소 상한")
args = ap.parse_args()

HUE = {"red": lambda h: (h <= 10) | (h >= 170), "yellow": lambda h: (h >= 15) & (h <= 35),
       "green": lambda h: (h >= 40) & (h <= 90)}
LAMPS = ["red", "yellow", "green"]


def frames_of(v):
    rows = list(csv.DictReader(open(v / "frames.csv")))
    return [(r, v / "color" / f"{int(r['index']):06d}.png") for r in rows][::args.step]


def info_of(v):
    c = json.loads((v / "capture.json").read_text())
    rgb = c.get("sensor_options_at_start", {}).get("RGB Camera", {})
    return {"video": v.name, "label": c.get("label"), "serial": c.get("calibration", {}).get("serial"),
            "ae": rgb.get("enable_auto_exposure"), "exposure": rgb.get("exposure"), "gain": rgb.get("gain"),
            "wb": "auto" if rgb.get("enable_auto_white_balance") == 1.0 else rgb.get("white_balance"),
            "complete": c.get("complete"), "gaps": c.get("color_frame_gaps")}


def pick_rois(img):
    show = cv2.resize(np.clip((img / 255.0) ** 0.5 * 255, 0, 255).astype(np.uint8), None, fx=2, fy=2)
    rois = {}
    for lamp in LAMPS:
        print(f"신호등 {lamp} 칸 하나를 끌어서 고르고 Enter (화면에 없으면 c 로 취소 → 멈춤)")
        x, y, w, h = cv2.selectROI(f"signal {lamp} segment", show, showCrosshair=False)
        cv2.destroyAllWindows()
        if w == 0 or h == 0:
            raise SystemExit("신호등 칸이 화면에 없음 — 카메라 방향을 고친 뒤 다시 찍어야 함")
        rois[f"lamp_{lamp}"] = [x // 2, y // 2, w // 2, h // 2]
    for col in ("red", "green"):
        print(f"{col} 퍽을 하나씩 끌어 고르고 Enter(퍽마다), 다 고르면 Esc. 없으면 바로 Esc")
        sel = cv2.selectROIs(f"{col} pucks", show, showCrosshair=False)
        cv2.destroyAllWindows()
        rois[f"puck_{col}"] = [[int(v) // 2 for v in r] for r in sel] if len(sel) else []
    print("무채색 면(흰 벽판 · 회색 바닥 등, 퍽 · 신호등 빛이 없는 곳) 하나를 고르고 Enter. 없으면 c")
    x, y, w, h = cv2.selectROI("neutral surface", show, showCrosshair=False)
    cv2.destroyAllWindows()
    rois["neutral"] = [x // 2, y // 2, w // 2, h // 2] if w and h else []
    return rois


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
    bgr = img[ys, xs].reshape(-1, 3).astype(int)
    return {"dE": float(np.linalg.norm(np.median(lab[ys, xs].reshape(-1, 3).astype(float), 0)
                                       - np.median(big[mask].astype(float), 0))),
            "chroma": float(np.median(bgr.max(1) - bgr.min(1))),
            "hue_ok": float(HUE[col](hsv[ys, xs].reshape(-1, 3)[:, 0].astype(int)).mean())}


args.out.mkdir(parents=True, exist_ok=True)
roi_path = args.out / "rois.json"
if args.rois:
    rois = json.loads(args.rois.read_text())
elif roi_path.exists():
    rois = json.loads(roi_path.read_text())
else:
    fr = frames_of(args.videos[0])
    rois = pick_rois(cv2.imread(str(fr[len(fr) // 2][1])))
roi_path.write_text(json.dumps(rois, indent=1))

summary, per_frame, checks = [], [], []
for v in args.videos:
    info = info_of(v)
    fr = frames_of(v)
    recs = []
    for row, p in fr:
        img = cv2.imread(str(p))
        if img is None:
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        rec = {"video": v.name, "index": int(row["index"]), "t_s": float(row["host_elapsed_s"]),
               "exposure_meta": row.get("color_exposure"),
               "meanv": {k: float(crop(hsv, rois[f"lamp_{k}"])[0][..., 2].mean()) for k in LAMPS},
               "counts": {k: lamp_counts(hsv, rois[f"lamp_{k}"]) for k in LAMPS},
               "pucks": [(col, puck_stats(img, lab, hsv, pr, col))
                         for col in ("red", "green") for pr in rois.get(f"puck_{col}", [])]}
        if rois.get("neutral"):
            m = np.median(crop(img, rois["neutral"])[0].reshape(-1, 3).astype(float), 0)
            rec["neutral_bg_rg"] = (m[0] / max(m[1], 1e-6), m[2] / max(m[1], 1e-6))
        recs.append(rec)
    mid = cv2.imread(str(fr[len(fr) // 2][1]))
    for k, r in rois.items():
        for b in (r if r and isinstance(r[0], list) else ([r] if r else [])):
            cv2.rectangle(mid, (b[0], b[1]), (b[0] + b[2], b[1] + b[3]), (255, 0, 255), 1)
    cv2.putText(mid, f"{info['label']} e{info['exposure']} g{info['gain']} wb{info['wb']}", (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    checks.append(cv2.resize(mid, (320, 240)))

    base = {k: min(f["meanv"][k] for f in recs) for k in LAMPS}
    row = dict(info, measured=len(recs))
    sig_ok = True
    for f in recs:
        d = {k: f["meanv"][k] - base[k] for k in LAMPS}
        top = max(d, key=d.get)
        f["lit"] = top if d[top] >= args.lit_delta else "off"
        per_frame.append({"video": v.name, "label": info["label"], "index": f["index"], "t_s": round(f["t_s"], 3),
                          "exposure_meta": f["exposure_meta"], "lit": f["lit"],
                          **{f"{k}_{c}": f["counts"][k][c] for k in LAMPS for c in LAMPS + ["white"]}})
    for lamp in LAMPS:
        lit = [(f["counts"][lamp][lamp], sum(f["counts"][lamp][k] for k in LAMPS if k != lamp), f["counts"][lamp]["white"])
               for f in recs if f["lit"] == lamp]
        row[f"{lamp}_lit"] = len(lit)
        if lit:
            cor, wr, wh = (float(np.median([t[i] for t in lit])) for i in range(3))
            row.update({f"{lamp}_correct": cor, f"{lamp}_wrong": wr, f"{lamp}_white": wh,
                        f"{lamp}_mixed_frames": sum(t[1] > t[0] for t in lit)})
            sig_ok &= cor >= args.min_correct and wr <= args.max_wrong * max(cor, 1)
        else:
            row.update({f"{lamp}_correct": "", f"{lamp}_wrong": "", f"{lamp}_white": "", f"{lamp}_mixed_frames": ""})
            sig_ok = False
    row["signal_ok(제안 기준)"] = sig_ok
    for col in ("red", "green"):
        n = [i for i, (c, _) in enumerate(recs[0]["pucks"]) if c == col] if recs else []
        if n:
            med = [{k: float(np.median([f["pucks"][i][1][k] for f in recs])) for k in ("dE", "chroma", "hue_ok")} for i in n]
            row[f"{col}_puck_min_chroma"] = round(min(m["chroma"] for m in med), 1)
            row[f"{col}_puck_min_hue_ok"] = round(min(m["hue_ok"] for m in med), 2)
            row[f"{col}_puck_min_dE"] = round(min(m["dE"] for m in med), 1)
    if recs and "neutral_bg_rg" in recs[0]:
        row["neutral_B/G"] = round(float(np.median([f["neutral_bg_rg"][0] for f in recs])), 2)
        row["neutral_R/G"] = round(float(np.median([f["neutral_bg_rg"][1] for f in recs])), 2)
    summary.append(row)
    print(f"{v.name}: {len(recs)}장, 켜짐 빨강 {row['red_lit']} · 노랑 {row['yellow_lit']} · 초록 {row['green_lit']}")

keys = list(dict.fromkeys(k for r in summary for k in r))
with open(args.out / "summary.csv", "w", newline="") as fp:
    w = csv.DictWriter(fp, keys)
    w.writeheader()
    w.writerows(summary)
with open(args.out / "frames.csv", "w", newline="") as fp:
    w = csv.DictWriter(fp, list(per_frame[0]))
    w.writeheader()
    w.writerows(per_frame)
md = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
md += ["| " + " | ".join(str(r.get(k, "")) for k in keys) + " |" for r in summary]
(args.out / "summary.md").write_text(
    f"# 노출 · WB 비교 영상 요약\n\n신호등 규칙 v_min {args.v_min} · s_min {args.s_min}, 켜짐 ΔV ≥ {args.lit_delta}, {args.step}장마다 1장. "
    f"signal_ok 는 제안 기준(세 색 모두 켜짐 장이 있고, 제 색 중앙값 ≥ {args.min_correct}, 다른 색 ≤ {args.max_wrong} × 제 색) — 확인 전.\n"
    "mixed_frames = 켜진 칸에서 다른 색 화소가 제 색보다 많은 장 수. 퍽 값은 퍽마다 장 중앙값을 낸 뒤 가장 나쁜 퍽(min).\n\n"
    + "\n".join(md) + "\n")
while len(checks) % 3:
    checks.append(np.zeros_like(checks[0]))
cv2.imwrite(str(args.out / "check.jpg"), np.vstack([np.hstack(checks[i:i + 3]) for i in range(0, len(checks), 3)]))
print(f"→ {args.out}/summary.md · summary.csv · frames.csv · check.jpg")
