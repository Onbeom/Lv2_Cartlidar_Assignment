"""놓친 장면 진단: 거르기를 모두 풀고 라벨 근처 후보의 특징을 본다. 사용: python3 diag.py <렌더> 장면... [noise=σ corr=px] [키=값]"""
import sys, json, zlib
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, "/home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3")
from locate_v2 import CFG2, locate, surfaces
root = Path(sys.argv[1]); rep = json.loads((root / "render_r3.json").read_text()); K = rep["d435"]["K"]
NOISE = CORR = 0.0; SEED = 0; names = []; over = {}
for a in sys.argv[2:]:
    if a.startswith("noise="): NOISE = float(a[6:])
    elif a.startswith("corr="): CORR = float(a[5:])
    elif a.startswith("seed="): SEED = int(a[5:])
    elif "=" in a: k, v = a.split("="); over[k] = type(CFG2[k])(v)
    else: names.append(a)
loose = dict(CFG2, **over); loose.update(thr=0.3, ring_max=9, ring_rel_max=9, resw_min=-99, sharp_min=-99, sat_max=999, bw_min=0, side_min=-99)
for p in rep["poses"]:
    if p["name"] not in names: continue
    dep = np.load(root / p["images"]["depth"]); bgr = cv2.imread(str(root / p["images"]["color"]))
    if NOISE:
        rng = np.random.default_rng(zlib.crc32(p["name"].encode()) + 7919 * SEED); e = rng.normal(0, 1, dep.shape).astype(np.float32)
        if CORR: e = cv2.GaussianBlur(e, (0, 0), CORR); e /= e.std()
        dep = (dep + e * NOISE * dep ** 2).astype(np.float32)
    Twc = np.array(p["T_world_cam"]); Twb = np.array(p["T_world_base_footprint"]); Tbc = np.linalg.inv(Twb) @ Twc
    lc = np.linalg.inv(Twc) @ np.append(p["label_center"], 1)
    pl, P, z = surfaces(dep, K, Tbc, loose)
    u, v = [int(round(x)) for x in p["label_px"]]
    hit = [(q["kind"], len(q["ys"]), round(q["rms"]*1000, 1), round(q["sig"]*1000, 1), round(q["ztop"], 3)) for q in pl if np.any((np.abs(q["ys"]-v) < 4) & (np.abs(q["xs"]-u) < 4))]
    print(p["name"], "px", (u, v), "incid", p["incidence_deg"], "dist", p["cam_dist_m"], "평면", hit)
    for f in locate(bgr, dep, K, Tbc, loose):
        d = np.linalg.norm(f["center"] - lc[:3])
        c = dict(CFG2, **over)
        why = [n for n, bad in [("score", f["score"] < c["thr"]), ("sat", f["sat"] > c["sat_max"]), ("bw", f["band_mean"] < c["bw_min"] * f["white_mean"]),
               ("ring_c", f["ring_c"] > c["ring_max"]), ("rrel", f["ring_std"] > c["ring_rel_max"] * max(f["white_mean"], 1)), ("resw", f["resw_lo"] < c["resw_min"]),
               ("sharp", f["sharp"] < c["sharp_min"]), ("side", f["side_lo"] < c["side_min"])] if bad]
        if d < 0.05 or (f["score"] >= 0.6 and not why):
            print("   ", "통과" if not why else "탈락:" + ",".join(why), "side %.2f" % f["side_lo"])
            print("   후보 d %.0fmm score %.2f ring_c %.2f rrel %.3f resw %.2f sharp %.3f sat %.0f bw %.2f" % (d*1000, f["score"], f["ring_c"], f["ring_std"]/f["white_mean"], f["resw_lo"], f["sharp"], f["sat"], f["band_mean"]/max(f["white_mean"],1)), f["kind"], [round(x) for x in f["px"]])
