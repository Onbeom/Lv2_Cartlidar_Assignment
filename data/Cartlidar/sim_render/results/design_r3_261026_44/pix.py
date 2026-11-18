"""라벨 화소 둘레에서 surfaces() 의 조건별로 몇 화소가 남는지."""
import sys, json, zlib
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, "/home/pa4/git/lv2_next_module/Cartlidar/sim_render/proto_r3")
from locate_v2 import CFG2
root = Path(sys.argv[1]); name = sys.argv[2]
over = {}; NOISE = 0; SEED = 0
for a in sys.argv[3:]:
    k, v = a.split("=")
    if k == "noise": NOISE = float(v)
    elif k == "seed": SEED = int(v)
    else: over[k] = type(CFG2[k])(v)
cfg = dict(CFG2, **over)
rep = json.loads((root / "render_r3.json").read_text()); K = rep["d435"]["K"]
p = [p for p in rep["poses"] if p["name"] == name][0]
dep = np.load(root / p["images"]["depth"])
if NOISE:
    rng = np.random.default_rng(zlib.crc32(p["name"].encode()) + 7919 * SEED); e = rng.normal(0, 1, dep.shape).astype(np.float32)
    e = cv2.GaussianBlur(e, (0, 0), 3); e /= e.std(); dep = (dep + e * NOISE * dep ** 2).astype(np.float32)
Tbc = np.linalg.inv(np.array(p["T_world_base_footprint"])) @ np.array(p["T_world_cam"])
H, W = dep.shape
uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
z = dep.astype(np.float32); ok = np.isfinite(z) & (z > 0.15) & (z < 4.0); z = np.where(ok, z, 0)
z = np.where(ok, cv2.medianBlur(z, cfg["median"]), 0)
P = np.dstack([(uu - K["cx"]) / K["fx"] * z, (vv - K["cy"]) / K["fy"] * z, z])
s = cfg["smooth"]; wsum = cv2.blur(ok.astype(np.float32), (s, s))
Ps = np.dstack([cv2.blur(P[..., i], (s, s)) for i in range(3)]) / np.maximum(wsum, 1e-6)[..., None]
k = cfg["nk"]; dx = np.zeros_like(Ps); dy = np.zeros_like(Ps)
dx[:, k:-k] = Ps[:, 2 * k:] - Ps[:, :-2 * k]; dy[k:-k] = Ps[2 * k:] - Ps[:-2 * k]
n = np.cross(dx, dy); n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
n *= -np.sign(np.sum(n * Ps, axis=2, keepdims=True) + 1e-12)
R, t = Tbc[:3, :3], Tbc[:3, 3]; nb = n @ R.T; Pb = P @ R.T + t
c = np.cos(np.radians(cfg["edge_deg"])); eN = np.zeros((H, W), bool); eZ = np.zeros((H, W), bool)
for sh in [(0, k), (k, 0)]:
    eN |= np.sum(n * np.roll(n, sh, axis=(0, 1)), axis=2) < c
    eZ |= np.abs(z - np.roll(z, sh, axis=(0, 1))) > 0.02 * np.maximum(z, 0.2)
u0, v0 = [int(round(x)) for x in p["label_px"]]
r = int(sys.argv[-1]) if False else 12
sl = (slice(max(v0 - r, 0), v0 + r + 1), slice(u0 - r, u0 + r + 1))
conds = dict(ok=ok, wsum=wsum > 0.99, edgeN=~eN, edgeZ=~eZ, minz=Pb[..., 2] >= cfg["min_z"], rng=np.hypot(Pb[..., 0], Pb[..., 1]) <= cfg["rng"], nz_top=nb[..., 2] >= cfg["nz_top"])
tot = np.ones((H, W), bool)
print(name, "px", (u0, v0), "창", sl, "입사", p["incidence_deg"], "거리", p["cam_dist_m"], "base 수평", p["base_hdist_m"])
for kk, m in conds.items():
    tot &= m
    print(f"  {kk:7s} 혼자 {m[sl].mean():.2f}  누적 {tot[sl].mean():.2f}")
print("  nb_z 중앙 %.3f  1%% %.3f, 라벨 base 높이 %.3f 수평 %.3f" % (np.median(nb[sl][..., 2]), np.percentile(nb[sl][..., 2], 1), Pb[v0, u0, 2], np.hypot(*Pb[v0, u0, :2])))
cnt, lab, st, _ = cv2.connectedComponentsWithStats(tot.astype(np.uint8), connectivity=8)
li = lab[v0, u0]; print("  라벨 화소 덩어리", li, "넓이", st[li, cv2.CC_STAT_AREA] if li else 0, "min_comp", cfg["min_comp"])
# 둘레 영역 단위 화소 범위 그림
bgr = cv2.imread(str(root / p["images"]["color"]))
R2 = 40; y0, x0 = max(v0 - R2, 0), max(u0 - R2, 0)
crop = bgr[y0:v0 + R2, x0:u0 + R2].copy(); m = tot[y0:v0 + R2, x0:u0 + R2]
ov = crop.copy(); ov[m] = (0.5 * ov[m] + [0, 127, 0]).astype(np.uint8)
cv2.imwrite(f"{name}_{NOISE}.png", cv2.resize(np.hstack([crop, ov]), None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
