import sys, json, zlib
from pathlib import Path
import cv2, numpy as np
root = Path(sys.argv[1]); name = sys.argv[2]; NOISE = float(sys.argv[3]) if len(sys.argv) > 3 else 0
rep = json.loads((root / "render_r3.json").read_text()); K = rep["d435"]["K"]
p = [p for p in rep["poses"] if p["name"] == name][0]
dep = np.load(root / p["images"]["depth"]).astype(np.float32)
if NOISE:
    rng = np.random.default_rng(zlib.crc32(p["name"].encode())); e = rng.normal(0, 1, dep.shape).astype(np.float32)
    e = cv2.GaussianBlur(e, (0, 0), 3); e /= e.std(); dep = (dep + e * NOISE * dep ** 2).astype(np.float32)
Tbc = np.linalg.inv(np.array(p["T_world_base_footprint"])) @ np.array(p["T_world_cam"])
H, W = dep.shape; uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
z = cv2.medianBlur(dep, 5)
P = np.dstack([(uu - K["cx"]) / K["fx"] * z, (vv - K["cy"]) / K["fy"] * z, z]); Pb = P @ Tbc[:3, :3].T + Tbc[:3, 3]
u0, v0 = [int(round(x)) for x in p["label_px"]]
h = Pb[..., 2]; h0 = h[v0, u0]
print("label h", h0)
for dv in range(-14, 15, 2):
    print("%+3d " % dv + "".join("#" if abs(h[v0+dv, u0+du]-h0) < 0.003 else ("^" if h[v0+dv,u0+du] > h0 else ".") for du in range(-30, 31)))
print("row0 h(mm):", " ".join("%.0f" % (h[v0, u0+du]*1000) for du in range(-30, 31, 2)))
print("col0 h(mm):", " ".join("%.0f" % (h[v0+dv, u0]*1000) for dv in range(-30, 31, 2)))
print("row0 depth(mm):", " ".join("%.0f" % (z[v0, u0+du]*1000) for du in range(-30, 31, 2)))
print("lab center base", np.linalg.inv(np.array(p["T_world_base_footprint"])) @ np.append(p["label_center"], 1))
