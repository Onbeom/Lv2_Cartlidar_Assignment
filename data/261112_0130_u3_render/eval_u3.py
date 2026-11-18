"""render_u3.py 자료 판정 (모의): P1 place_box (dev place_box.hpp) · T1 태그 판독 (dev TagDetector, 검출 폭 640 · 960 · 1280).
    python3 eval_u3.py <렌더 폴더> [--overlay <그림 폴더>]
P1 (장마다): 찾음(ok) · 중심 오차 <= 3 mm · 요 오차 <= 1°. 사전값 = 계획 자리 상자 중심 (0.4353, 0) · 요 0.
T1: 대상 = 서 있는 퍽 윗면 태그 중 네 꼭짓점이 화면 가장자리에서 5 px 넘게 안쪽인 것. 폭마다 읽힌 비율 · 장면에 없는 ID 수.
    누운 퍽 끝면 태그는 기록만 (화면 안 · 카메라를 향한 것 중 읽힌 수).
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
import os
PB_TOOL = os.environ.get("PB_TOOL", str(HERE / "place_box_check"))   # 11-12: 고친 place_box 회귀용
EVAL_TAG = os.environ.get("EVAL_TAG", "")   # 결과 json 이름 뒤에 붙임 (옛 결과 안 덮음)
ap = argparse.ArgumentParser()
ap.add_argument("dir", type=Path)
ap.add_argument("--overlay", type=Path)
ap.add_argument("--widths", type=int, nargs="+", default=[640, 960, 1280])
a = ap.parse_args()
meta = json.loads((a.dir / "render_u3.json").read_text())
K = meta["K"]
W, H = meta["width"], meta["height"]


def proj(Ti, p):
    v = Ti @ np.r_[p, 1.0]
    return np.array([K["fx"] * v[0] / v[2] + K["cx"], K["fy"] * v[1] / v[2] + K["cy"]]), v[:3]


def inside(uv, m=5):
    return all(m < u < W - m and m < v < H - m for u, v in uv)


# ---- T1 태그 판독
lst = a.dir / "images.txt"
lst.write_text("".join(f"{sc['image']} {a.dir / sc['image']}\n" for sc in meta["scenes"]))
reads = {}
for w in a.widths:
    out = subprocess.run([str(HERE / "tag_read"), str(lst), str(w)], capture_output=True, text=True, check=True).stdout
    reads[w] = {j["name"]: j for j in (json.loads(l) for l in out.splitlines() if l.strip())}

rows = []
for sc in meta["scenes"]:
    T = np.array(sc["T_bf_cam"])
    Ti = np.linalg.inv(T)
    cam = T[:3, 3]
    stand, lie = [], []
    for pk in sc["pucks"]:
        if "tag_top" in pk:
            uv = [proj(Ti, np.array(c))[0] for c in pk["tag_top"]]
            short = min(np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4))
            if inside(uv):
                stand.append((pk["tag_id"], float(short), pk["cell"]))
        else:
            for corners, nrm in zip(pk["tag_ends"], pk["tag_end_normals"]):
                c = np.mean(corners, 0)
                facing = float(np.dot(np.asarray(nrm), (cam - c) / np.linalg.norm(cam - c)))
                uv = [proj(Ti, np.array(p))[0] for p in corners]
                if facing > 0.2 and inside(uv):
                    lie.append((pk["tag_id"], facing))
    present = {pk["tag_id"] for pk in sc["pucks"]}
    r = dict(image=sc["image"], kind=sc.get("kind", "judge"), cam_dz=sc.get("cam_dz", 0.0), corner=sc["corner"], light=sc["light"], n_pucks=len(sc["pucks"]),
             stand=[s[0] for s in stand], stand_px=[s[1] for s in stand], lie=[l[0] for l in lie])
    for w in a.widths:
        got = set(reads[w][sc["image"]].get("ids", []))
        r[f"w{w}"] = dict(read=sorted(got & {s[0] for s in stand}), wrong=sorted(got - present),
                          lie_read=sorted(got & {l[0] for l in lie}), ms=reads[w][sc["image"]].get("ms"))
    rows.append(r)

# ---- P1 place_box
for r, sc in zip(rows, meta["scenes"]):
    T = np.array(sc["T_bf_cam"]).reshape(-1)
    out = subprocess.run([PB_TOOL, str(a.dir / sc["image"]), *map(str, (K["fx"], K["fy"], K["cx"], K["cy"])),
                          *map(str, T), str(sc["prior"][0]), str(sc["prior"][1]), str(sc["prior_yaw"])],
                         capture_output=True, text=True).stdout.split()   # 못 찾으면 종료 코드 1 · ok 0
    x, y, yaw, score, ok = float(out[0]), float(out[1]), float(out[2]), float(out[3]), int(out[4])
    dc = 1000 * math.hypot(x - sc["center"][0], y - sc["center"][1])
    dy = abs(math.degrees((yaw - sc["yaw"] + math.pi / 4) % (math.pi / 2) - math.pi / 4))
    r.update(pb_ok=bool(ok), pb_center_mm=dc, pb_yaw_deg=dy, pb_pass=bool(ok) and dc <= 3.0 and dy <= 1.0, pb_est=[x, y, yaw])
    if a.overlay:
        import cv2
        a.overlay.mkdir(parents=True, exist_ok=True)
        img = cv2.imread(str(a.dir / sc["image"]))
        Ti = np.linalg.inv(np.array(sc["T_bf_cam"]))

        def poly(cx, cy, yw, col):
            R = np.array([[math.cos(yw), -math.sin(yw)], [math.sin(yw), math.cos(yw)]])
            pts = [proj(Ti, np.r_[np.array([cx, cy]) + R @ np.array([sx, sy]) * meta["box"] / 2, meta["box_h"]])[0]
                   for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
            cv2.polylines(img, [np.round(pts).astype(int)], True, col, 2)
        poly(*sc["center"], sc["yaw"], (0, 255, 0))
        if ok:
            poly(x, y, yaw, (255, 0, 255))
        cv2.putText(img, f"{sc['image']} pb {dc:.2f}mm {dy:.2f}deg  960 {len(r['w960']['read'])}/{len(r['stand'])}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imwrite(str(a.overlay / sc["image"].replace(".png", ".jpg")), img)

allrows = rows
recs = [r for r in rows if r["kind"] != "judge"]
rows = [r for r in rows if r["kind"] == "judge"]
n = len(rows)
pb = [r for r in rows if r["pb_ok"]]
c = np.array([r["pb_center_mm"] for r in pb])
y = np.array([r["pb_yaw_deg"] for r in pb])
print(f"P1 place_box: 통과 {sum(r['pb_pass'] for r in rows)}/{n}, 찾음 {len(pb)}/{n}, 꼭짓점 장 {sum(r['pb_pass'] for r in rows if r['corner'])}/{sum(r['corner'] for r in rows)}")
if len(c):
    print(f"  중심 mm: 중앙 {np.median(c):.2f} p95 {np.percentile(c, 95):.2f} 최대 {c.max():.2f}  요 °: 중앙 {np.median(y):.2f} p95 {np.percentile(y, 95):.2f} 최대 {y.max():.2f}")
tot = sum(len(r["stand"]) for r in rows)
px = np.array([p for r in rows for p in r["stand_px"]])
if len(px):
    print(f"T1 대상 (서 있는 퍽 윗면 태그, 화면 안): {tot} 개, 투영 짧은 변(1280 기준) 최소 {px.min():.1f} 중앙 {np.median(px):.1f} px")
for w in a.widths:
    got = sum(len(r[f"w{w}"]["read"]) for r in rows)
    wrong = sum(len(r[f"w{w}"]["wrong"]) for r in rows)
    full = sum(len(r[f"w{w}"]["read"]) == len(r["stand"]) for r in rows)
    cr = sum(len(r[f"w{w}"]["read"]) for r in rows if r["corner"])
    ct = sum(len(r["stand"]) for r in rows if r["corner"])
    lie_t = sum(len(r["lie"]) for r in rows)
    lie_g = sum(len(r[f"w{w}"]["lie_read"]) for r in rows)
    ms = np.array([r[f"w{w}"]["ms"] for r in rows if r[f"w{w}"]["ms"] is not None])
    print(f"  폭 {w}: 읽음 {got}/{tot} ({100 * got / max(tot, 1):.1f} %) · 장면에 없는 ID {wrong} · 모두 읽은 장 {full}/{n} · "
          f"꼭짓점 장 {cr}/{ct} · 누운 끝면(기록) {lie_g}/{lie_t} · PC 장당 중앙 {np.median(ms):.1f} ms")
print(f"기록 장면 {len(recs)} 장 (판정 밖):")
for r in recs:
    print(f"  {r['image']} {r['kind']} dz {r['cam_dz']:+.2f}: place_box 찾음 {r['pb_ok']} 중심 {r['pb_center_mm']:.2f} mm 요 {r['pb_yaw_deg']:.2f}° · "
          + " · ".join(f"{w} {len(r[f'w{w}']['read'])}/{len(r['stand'])}" for w in a.widths))
(a.dir / f"eval_u3{EVAL_TAG}.json").write_text(json.dumps(allrows, indent=1, ensure_ascii=False) + "\n")
