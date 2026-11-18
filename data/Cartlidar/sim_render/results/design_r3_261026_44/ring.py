"""후보의 펴진 영상 · 둘레 띠 · 라벨 자리를 그림으로. 사용: ring.py <렌더> <장면> <noise> <seed> [키=값]"""
import sys, json, zlib
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, "/home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3")
from locate_v2 import CFG2, locate
root = Path(sys.argv[1]); name = sys.argv[2]; NOISE = float(sys.argv[3]); SEED = int(sys.argv[4])
over = {k: type(CFG2[k])(v) for k, v in (a.split("=") for a in sys.argv[5:])}
rep = json.loads((root / "render_r3.json").read_text()); K = rep["d435"]["K"]
p = [p for p in rep["poses"] if p["name"] == name][0]
dep = np.load(root / p["images"]["depth"])
if NOISE:
    rng = np.random.default_rng(zlib.crc32(p["name"].encode()) + 7919 * SEED); e = rng.normal(0, 1, dep.shape).astype(np.float32)
    e = cv2.GaussianBlur(e, (0, 0), 3); e /= e.std(); dep = (dep + e * NOISE * dep ** 2).astype(np.float32)
Twc = np.array(p["T_world_cam"]); Tbc = np.linalg.inv(np.array(p["T_world_base_footprint"])) @ Twc
lc = (np.linalg.inv(Twc) @ np.append(p["label_center"], 1))[:3]
loose = dict(CFG2, **over); loose.update(thr=0.3, ring_max=9, ring_rel_max=9, resw_min=-99, sharp_min=-99, sat_max=999, bw_min=0, side_min=-99, keep=True)
for f in locate(cv2.imread(str(root / p["images"]["color"])), dep, K, Tbc, loose):
    if np.linalg.norm(f["center"] - lc) < 0.01:
        g, rv, ml = f["crop"]
        im = cv2.cvtColor(np.clip(g, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR); ov = im.copy()
        ov[rv] = (0.5 * ov[rv] + [0, 0, 127]).astype(np.uint8); ov[ml > 0] = (0.6 * ov[ml > 0] + [100, 0, 0]).astype(np.uint8)
        out = f"ring_{name}_{NOISE}_{SEED}.png"; cv2.imwrite(out, cv2.resize(np.hstack([im, ov]), None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST))
        print(out, "score %.2f ring_c %.2f ring_std %.1f white %.0f band %.0f" % (f["score"], f["ring_c"], f["ring_std"], f["white_mean"], f["band_mean"]))
