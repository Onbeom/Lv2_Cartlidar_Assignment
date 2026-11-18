"""render_pick_box.py 자료를 perception place_box_check (C++, dev place_box.hpp) 로 돌려 참 상자 중심 · 요와 견줌 (모의 판정 · 도구).
    python3 eval_pick.py <렌더 폴더> <place_box_check> [--overlay <그림 폴더>]
판정 (장마다): 찾음(ok) · 중심 오차 <= 3 mm · 요 오차 <= 1°. 사전값 = 계획기 자리 (정지 오차 없는 상자 중심 · 요)."""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("dir", type=Path)
ap.add_argument("exe")
ap.add_argument("--overlay", type=Path)
a = ap.parse_args()
meta = json.loads((a.dir / "render_pick_box.json").read_text())
K = meta["K"]
rows = []
for sc in meta["scenes"]:
    T = np.array(sc["T_bf_cam"]).reshape(-1)
    out = subprocess.run([a.exe, str(a.dir / sc["image"]), *map(str, (K["fx"], K["fy"], K["cx"], K["cy"])), *map(str, T),
                          str(sc["prior"][0]), str(sc["prior"][1]), str(sc["prior_yaw"])],
                         capture_output=True, text=True, check=True).stdout.split()
    x, y, yaw, score, ok = float(out[0]), float(out[1]), float(out[2]), float(out[3]), int(out[4])
    dc = 1000 * math.hypot(x - sc["center"][0], y - sc["center"][1])
    dy = abs(math.degrees((yaw - sc["yaw"] + math.pi / 4) % (math.pi / 2) - math.pi / 4))
    nb = [p["kind"] for p in sc["pucks"] if not p["target"]]
    passed = bool(ok) and dc <= 3.0 and dy <= 1.0
    rows.append(dict(image=sc["image"], case=sc["case"], light=sc["light"], neighbor=nb[0] if nb else "none",
                     ok=bool(ok), center_mm=dc, yaw_deg=dy, score=score, passed=passed, est=[x, y, yaw]))
    print(f"{sc['image']}: {sc['case']} 옆 {rows[-1]['neighbor']:<15} 찾음 {ok} 중심 {dc:6.2f} mm 요 {dy:5.2f}° "
          f"{'통과' if passed else '불통과'}")
    if a.overlay:
        import cv2
        a.overlay.mkdir(parents=True, exist_ok=True)
        img = cv2.imread(str(a.dir / sc["image"]))
        Ti = np.linalg.inv(np.array(sc["T_bf_cam"]))

        def proj(cx, cy, yw, col):
            R = np.array([[math.cos(yw), -math.sin(yw)], [math.sin(yw), math.cos(yw)]])
            pts = []
            for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
                p = np.r_[np.array([cx, cy]) + R @ np.array([sx, sy]) * meta["box"] / 2, meta["box_h"], 1.0]
                v = Ti @ p
                pts.append((int(round(K["fx"] * v[0] / v[2] + K["cx"])), int(round(K["fy"] * v[1] / v[2] + K["cy"]))))
            cv2.polylines(img, [np.array(pts)], True, col, 2)
        proj(*sc["center"], sc["yaw"], (0, 255, 0))
        if ok:
            proj(x, y, yaw, (255, 0, 255))
        cv2.imwrite(str(a.overlay / sc["image"].replace(".png", ".jpg")), img)
n = len(rows)
p = sum(r["passed"] for r in rows)
c = np.array([r["center_mm"] for r in rows if r["ok"]])
y = np.array([r["yaw_deg"] for r in rows if r["ok"]])
print(f"통과 {p}/{n}, 찾음 {sum(r['ok'] for r in rows)}/{n}")
if len(c):
    print(f"  중심 mm: 중앙 {np.median(c):.2f} p95 {np.percentile(c, 95):.2f} 최대 {c.max():.2f}")
    print(f"  요 °: 중앙 {np.median(y):.2f} p95 {np.percentile(y, 95):.2f} 최대 {y.max():.2f}")
for key in ("case", "neighbor", "light"):
    groups = sorted({str(r[key]) for r in rows})
    print(f"  {key}: " + ", ".join(f"{g} {sum(r['passed'] for r in rows if str(r[key]) == g)}/{sum(str(r[key]) == g for r in rows)}" for g in groups))
(a.dir / "eval_pick.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False) + "\n")
