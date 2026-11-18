"""render_d435.py 자료 판정 (모의): 앞 D435 영상 한 장으로 상자 윗면 중심 · 요 (dev c689a38 place_box.hpp, place_box_check).
    python3 eval_d435.py <렌더 폴더> [--overlay <그림 폴더>]
장마다 중심 오차 (mm) · 요 오차 (90° 접음, °). 기준 (검토 261111 1 확인 뒤 확정):
  D1 모든 장: 찾음 · 중심 ≤ 15 mm · 요 ≤ 4° (필요 정확도)   D2 ≥ 95 % 가 중심 ≤ 10 mm · 요 ≤ 2°
표는 θ 묶음 (±15° 안 · ±45° 안) × standoff 로 나눔. 기록: 지도 축 기준 잰 요의 부호가 참과 다른 장 (|θ| ≥ 40°, 고를 면이 바뀌는 경우).
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np

import os
TOOL = Path(os.environ.get("PB_TOOL", str(Path.home() / "git/lv2_next_module/261112_0130_u3_render/place_box_check")))
EVAL_TAG = os.environ.get("EVAL_TAG", "")
ap = argparse.ArgumentParser()
ap.add_argument("dir", type=Path)
ap.add_argument("--overlay", type=Path)
a = ap.parse_args()
meta = json.loads((a.dir / "render_u3.json").read_text())
K = meta["K"]


def fold(x):
    return (x + math.pi / 4) % (math.pi / 2) - math.pi / 4


rows = []
for sc in meta["scenes"]:
    T = np.array(sc["T_bf_cam"]).reshape(-1)
    out = subprocess.run([str(TOOL), str(a.dir / sc["image"]), *map(str, (K["fx"], K["fy"], K["cx"], K["cy"])), *map(str, T),
                          str(sc["prior"][0]), str(sc["prior"][1]), str(sc["prior_yaw"])], capture_output=True, text=True).stdout.split()
    x, y, yaw, score, ok = float(out[0]), float(out[1]), float(out[2]), float(out[3]), int(out[4])
    dc = 1000 * math.hypot(x - sc["center"][0], y - sc["center"][1])
    dy = abs(math.degrees(fold(yaw - sc["yaw"])))
    meas_map = math.degrees(fold(yaw - sc["prior_yaw"]))   # 지도 축 기준 잰 요 (±45° 접음)
    flip = abs(sc["box_yaw_deg"]) >= 40 and meas_map * sc["box_yaw_deg"] < 0
    rows.append(dict(image=sc["image"], group=sc["group"], standoff=sc["standoff"], corner=sc["corner"], light=sc["light"], theta=sc["box_yaw_deg"],
                     n_pucks=len(sc["pucks"]), ok=bool(ok), c_mm=dc, y_deg=dy, meas_map_deg=meas_map, flip=bool(flip),
                     d1=bool(ok) and dc <= 15 and dy <= 4, d2=bool(ok) and dc <= 10 and dy <= 2))
    if a.overlay:
        import cv2
        a.overlay.mkdir(parents=True, exist_ok=True)
        img = cv2.imread(str(a.dir / sc["image"]))
        Ti = np.linalg.inv(np.array(sc["T_bf_cam"]))

        def poly(cx, cy, yw, col):
            R = np.array([[math.cos(yw), -math.sin(yw)], [math.sin(yw), math.cos(yw)]])
            pts = []
            for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
                v = Ti @ np.r_[np.array([cx, cy]) + R @ np.array([sx, sy]) * meta["box"] / 2, meta["box_h"], 1.0]
                pts.append((K["fx"] * v[0] / v[2] + K["cx"], K["fy"] * v[1] / v[2] + K["cy"]))
            cv2.polylines(img, [np.round(pts).astype(int)], True, col, 1)
        poly(*sc["center"], sc["yaw"], (0, 255, 0))
        if ok:
            poly(x, y, yaw, (255, 0, 255))
        cv2.putText(img, f"{sc['image']} th {sc['box_yaw_deg']:+.0f} so {sc['standoff']:.2f} {dc:.1f}mm {dy:.2f}deg", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imwrite(str(a.overlay / sc["image"].replace(".png", ".jpg")), img)


def line(name, rs):
    if not rs:
        return
    f = [r for r in rs if r["ok"]]
    c = np.array([r["c_mm"] for r in f]) if f else np.array([np.nan])
    y = np.array([r["y_deg"] for r in f]) if f else np.array([np.nan])
    print(f"  {name}: {len(rs)} 장 · 찾음 {len(f)} · D1 {sum(r['d1'] for r in rs)}/{len(rs)} · D2 {sum(r['d2'] for r in rs)}/{len(rs)} "
          f"({100 * sum(r['d2'] for r in rs) / len(rs):.1f} %) · 중심 중앙 {np.median(c):.2f} p95 {np.percentile(c, 95):.2f} 최대 {c.max():.2f} mm · "
          f"요 중앙 {np.median(y):.2f} p95 {np.percentile(y, 95):.2f} 최대 {y.max():.2f}°")


n = len(rows)
d1 = all(r["d1"] for r in rows)
d2 = sum(r["d2"] for r in rows) / n
print(f"D1 (모든 장 찾음 · ≤ 15 mm · ≤ 4°): {'통과' if d1 else '불통과'} {sum(r['d1'] for r in rows)}/{n}")
print(f"D2 (≥ 95 % 가 ≤ 10 mm · ≤ 2°): {'통과' if d2 >= 0.95 else '불통과'} {100 * d2:.1f} %")
line("전체", rows)
for g in ("15", "45"):
    line(f"θ ±{g}° 묶음", [r for r in rows if r["group"] == g])
    for so in sorted({r["standoff"] for r in rows}):
        line(f"θ ±{g}° · standoff {so:.2f}", [r for r in rows if r["group"] == g and r["standoff"] == so])
line("꼭짓점 장", [r for r in rows if r["corner"]])
for lv in (0, 1):
    line(f"조명 L{lv}", [r for r in rows if r["light"] == lv])
fl = [r for r in rows if abs(r["theta"]) >= 40]
print(f"기록: |θ| ≥ 40° {len(fl)} 장 중 지도 축 기준 잰 요 부호가 참과 다른 장 {sum(r['flip'] for r in fl)} "
      f"(θ = ±45° 꼭짓점 {sum(r['flip'] for r in fl if abs(abs(r['theta']) - 45) < 1e-6)}/{sum(abs(abs(r['theta']) - 45) < 1e-6 for r in fl)})")
bad = [r for r in rows if not r["d1"]]
for r in bad[:20]:
    print(f"  D1 불통과: {r['image']} θ {r['theta']:+.1f} standoff {r['standoff']} 찾음 {r['ok']} 중심 {r['c_mm']:.1f} mm 요 {r['y_deg']:.2f}°")
(a.dir / f"eval_d435{EVAL_TAG}.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False) + "\n")
