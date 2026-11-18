"""render_a1 결과에서 검출기 평가: 다 보이는 퍽(in_frac 1 · vis ≥ 0.9) 검출률(모양별 · 조명별 · 목표), 헛검출.
python3 a1_det_eval.py <render_a1 폴더> [old|label]"""
import json
import math
import sys
from collections import defaultdict

import cv2

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from a1_labeldet import CFG, detect  # noqa: E402

root = sys.argv[1]
aware = (sys.argv[2] if len(sys.argv) > 2 else "label") != "old"
cfg = dict(CFG)
for kv in sys.argv[3:]:   # 예: dark_win=31 dark_c=20
    k, v = kv.split("=")
    cfg[k] = type(CFG.get(k, 0))(float(v)) if k in CFG else int(float(v))
d = json.load(open(f"{root}/render_a1.json"))
st = defaultdict(lambda: [0, 0])
false, ndet = 0, 0
for sc in d["scenes"]:
    info = {p["name"]: p for p in sc["pucks"]}
    for f in sc["arm_frames"]:
        dets = detect(cv2.imread(f"{root}/{f['image']}"), c=cfg, label_aware=aware)
        ndet += len(dets)
        used = set()
        for n, p in f["pucks"].items():
            if not (p["in_frac"] == 1.0 and p["vis"] >= 0.9):
                continue
            c = 0 if info[n]["color"] == "red" else 1
            dd = [(math.hypot(x[0] - p["box"][0], x[1] - p["box"][1]), i) for i, x in enumerate(dets) if x[3] == c]
            ok = bool(dd) and min(dd)[0] <= 0.05
            if ok:
                used.add(min(dd)[1])
            pose = "서 있음" if info[n]["standing"] else "누움"
            for key in (f"{info[n]['shape']}·{pose}", f"조명{sc['light_level']:g}", "전체") + (("목표",) if n == sc["target"] else ()):
                st[key][0] += ok
                st[key][1] += 1
        # 헛검출: 어떤 보이는 퍽(일부 포함) 상자에도 0.05 안에 없는 검출
        for i, x in enumerate(dets):
            near = [math.hypot(x[0] - p["box"][0], x[1] - p["box"][1]) for p in f["pucks"].values() if p["box"]]
            if not near or min(near) > 0.05:
                false += 1
for k in sorted(st):
    print(k, f"{st[k][0]}/{st[k][1]} = {st[k][0] / st[k][1]:.3f}")
print("검출 수", ndet, "어느 퍽과도 안 맞는 검출", false)
