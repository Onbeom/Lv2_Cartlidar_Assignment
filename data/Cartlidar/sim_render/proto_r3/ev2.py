"""시제품 2 평가: 벽 · 윗면 검출률과 헛검출 수. 사용: python3 ev2.py <render_r3 폴더> [키=값 ...] [dump=<json>] [-v]

정답: 장면마다 실제 있는 라벨 모두(옮긴 prim 의 원래 자리는 뺌). 판정 대상: 라벨 보임 ≥ 0.95 · base_footprint 수평 ≤ 1.0 m.
찾음 = 지금 라벨에서 30 mm 안, 헛검출 = 어느 라벨에서도 50 mm 넘게 떨어진 검출(판정 기준 R3).
"""
import collections
import json
import sys
import zlib
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from locate_v2 import CFG2, locate  # noqa: E402

root = Path(sys.argv[1])
cfg = dict(CFG2)
CORR = 0.0    # 잡음 공간 상관(가우시안 σ 화소, 0 이면 화소마다 따로)
SEED = 0      # 잡음 씨앗 번호(같은 장면에 다른 잡음을 넣어 우연 효과를 평균내려고)
NOISE = 0.0   # 깊이 잡음: 화소마다 σ = NOISE · z² (m), D435 이론값 ≈ 0.004 (z = 1 m 에서 4 mm)
for a in sys.argv[2:]:
    if a.startswith("corr="):
        CORR = float(a[5:]); continue
    if a.startswith("noise="):
        NOISE = float(a[6:]); continue
    if a.startswith("seed="):
        SEED = int(a[5:]); continue
    if "=" in a and not a.startswith("dump="):
        k, v = a.split("=")
        cfg[k] = (v == "1") if isinstance(CFG2[k], bool) else type(CFG2[k])(v)
rep = json.loads((root / "render_r3.json").read_text())
K = rep["d435"]["K"]
labs = {lb["id"]: lb for lb in rep["labels"]}


def run(p):
    cur = labs[p["label"]]
    present = [np.array(cur["center"])] + [np.array(lb["center"]) for lb in rep["labels"]
                                           if lb["world"] == cur["world"] and lb["source"] == "world" and lb["id"] != cur["id"]
                                           and lb["prim"] != (cur["prim"] if cur["source"] == "moved" else None)]
    bgr = cv2.imread(str(root / p["images"]["color"]))
    dep = np.load(root / p["images"]["depth"])
    if NOISE > 0:
        rng = np.random.default_rng(zlib.crc32(p["name"].encode()) + 7919 * SEED)   # hash() 는 실행마다 바뀌어 재현이 안 됨
        e = rng.normal(0, 1, dep.shape).astype(np.float32)
        if CORR > 0:
            e = cv2.GaussianBlur(e, (0, 0), CORR); e /= e.std()
        dep = (dep + e * NOISE * dep ** 2).astype(np.float32)
    Twc = np.array(p["T_world_cam"]); Twb = np.array(p["T_world_base_footprint"])
    found = locate(bgr, dep, K, np.linalg.inv(Twb) @ Twc, cfg)
    judged = p["label_visible"] >= 0.95 and p["base_hdist_m"] <= 1.0
    rows = []
    for f in found:
        w = (Twc @ np.append(f["center"], 1))[:3]   # center 는 카메라 좌표
        f2 = {k: v for k, v in f.items() if k != "center"}
        f2.update(scene=p["name"], judged=judged, where=cur["where"], dcur=float(np.linalg.norm(w - present[0])),
                  dmin=float(min(np.linalg.norm(w - c) for c in present)), world=[round(x, 3) for x in w], hdist=p["base_hdist_m"])
        rows.append(f2)
    return p["name"], judged, cur["where"], rows


if __name__ == "__main__":
    poses = [p for p in rep["poses"] if p["kind"] == "d435"]
    with Pool(12) as pool:
        res = pool.map(run, poses, chunksize=4)
    dump = [r for _, _, _, rows in res for r in rows]
    for a in sys.argv:
        if a.startswith("dump="):
            Path(a[5:]).write_text(json.dumps(dump))
    print("설정", {k: cfg[k] for k in cfg if k != "tex"})
    for thr in sorted({0.5, 0.6, 0.65, 0.7, 0.75, 0.8, cfg["thr"]}):
        if thr < cfg["thr"]:
            continue
        c = collections.Counter(); errs = []
        for name, judged, where, rows in res:
            c["false"] += sum(r["dmin"] > 0.05 and r["score"] >= thr for r in rows)
            if judged:
                c[where + "_n"] += 1
                hit = [r["dcur"] for r in rows if r["dcur"] <= 0.03 and r["score"] >= thr]
                if hit:
                    c[where + "_hit"] += 1; errs.append(min(hit))
        print(f"thr {thr:.2f}: 벽 {c['wall_hit']}/{c['wall_n']}  윗면 {c['shelf_top_hit']}/{c['shelf_top_n']}  헛 {c['false']}"
              f"  오차 최대 {max(errs) * 1000 if errs else float('nan'):.1f} mm")
    if "-v" in sys.argv:
        for r in sorted([r for r in dump if r["dmin"] > 0.05], key=lambda r: -r["score"])[:20]:
            print("헛", round(r["score"], 2), r["kind"], r["scene"], r["world"], [round(x) for x in r["px"]])
        for name, judged, where, rows in res:
            if judged and not any(r["dcur"] <= 0.03 for r in rows):
                print("놓침", name, [(round(r["dcur"] * 1000), round(r["score"], 2)) for r in rows if r["dcur"] < 0.1])
