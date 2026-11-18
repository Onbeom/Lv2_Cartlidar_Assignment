"""전처리별 zbar 결과(원본 배율). 사용: python3 -I zpre.py <렌더>..."""
import json, sys, collections
from pathlib import Path
import cv2, numpy as np
sys.argv, roots = sys.argv[:1], sys.argv[1:]
exec(open(Path(__file__).with_name("zq.py")).read().split("rows = []")[0])   # zbar 준비 · read() 만 가져옴
def gam(g, k): return (255 * (g / 255.0) ** k).astype(np.uint8)
def unsharp(g, a): b = cv2.GaussianBlur(g.astype(np.float32), (0, 0), 1.0); return np.clip(g + a * (g - b), 0, 255).astype(np.uint8)
def stretch(g): lo, hi = np.percentile(g, [1, 99]); return np.clip((g - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)
clahe = cv2.createCLAHE(2.0, (8, 8))
V = {"원본": lambda g: g, "감마1.5": lambda g: gam(g, 1.5), "감마2.5": lambda g: gam(g, 2.5), "감마3": lambda g: gam(g, 3.0), "감마2": lambda g: gam(g, 2.0), "감마0.5": lambda g: gam(g, 0.5), "선명0.5": lambda g: unsharp(g, 0.5),
     "선명1": lambda g: unsharp(g, 1.0), "늘이기": stretch, "CLAHE": clahe.apply}
out = []
for root in map(Path, roots):
    rep = json.loads((root / "render_r3.json").read_text())
    for p in rep["poses"]:
        if p["kind"] != "arm": continue
        g = cv2.cvtColor(cv2.imread(str(root / p["images"]["color"])), cv2.COLOR_BGR2GRAY)
        out.append(dict(name=p["name"], text=p["text"], d=p["dist_nominal"], t=p["tilt_deg"], vis=p["label_visible"], mpx=p["module_px"],
                        v={k: read(f(g.astype(np.float32) if k.startswith("선명") else g)) for k, f in V.items()}))
json.dump(out, open("zpre_rows.json", "w"))
J = lambda r: r["d"] <= 0.05 + 1e-9 and r["t"] <= 20 and r["vis"] >= 0.95
for k in V:
    for qmin in [1, 4]:
        c = collections.Counter()
        for r in out:
            u = {t for t, q in r["v"][k] if q >= qmin}; d = u.pop() if len(u) == 1 else None
            ok = d == r["text"]; c["w"] += d is not None and not ok; c["a"] += ok
            if J(r): c["jn"] += 1; c["j"] += ok
        bq = max([q for r in out for t, q in r["v"][k] if t != r["text"]] or [0])
        print(f"{k:6s} q≥{qmin}: 판정 {c['j']}/{c['jn']} 틀림 {c['w']} 전체 {c['a']}/{len(out)} (틀린 문자열 quality 최대 {bq})")
