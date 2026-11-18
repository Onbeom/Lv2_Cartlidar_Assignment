"""D435 바코드 라벨 찾기 시제품: 밝은 덩어리 → 깊이로 3D → 크기 · 평면 · 막대 무늬. 사용: python -I locate_proto.py <render_r3 폴더>"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

CFG = dict(bg=41, delta=12.0, band_scales=(5, 7, 11), band_rel=0.08, close=5, L=0.035, S=0.020, bandL=0.0286, bandS=0.014, tol=0.2, aspect=1.45, plane_rms=0.004, s_lo=0.6, stripe=0.5, min_px=20, min_z=0.06, nz_wall=0.25, nz_top=0.9)


def locate(bgr, dep, K, cfg=CFG, debug=False, T_base_cam=None):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (cfg["close"], cfg["close"]))
    comps = []
    masks = [g > cv2.blur(g, (cfg["bg"], cfg["bg"])) + cfg["delta"]]   # 넓은 면 위(벽)
    kinds = ["label"]
    for sc in cfg["band_scales"]:   # 밝은 면(선반 윗면) 위: 막대 띠가 둘레보다 어두움 (작은 black-hat)
        cl = cv2.morphologyEx(g, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (sc, sc)))
        masks.append((cl - g) / (cl + 10.0) > cfg["band_rel"])
        kinds.append("band")
    for mm, kind in zip(masks, kinds):
        m = cv2.morphologyEx((mm * 255).astype(np.uint8), cv2.MORPH_CLOSE, k)
        n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] >= cfg["min_px"]:
                comps.append((kind, np.nonzero(lab == i)))
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    out = []
    for kind, (ys, xs) in comps:
        LL, SS = (cfg["L"], cfg["S"]) if kind == "label" else (cfg["bandL"], cfg["bandS"])
        z = dep[ys, xs]
        ok = np.isfinite(z) & (z > 0.15) & (z < 4.0)
        if ok.sum() < cfg["min_px"]:
            continue
        P = np.stack([(xs[ok] - K["cx"]) / K["fx"] * z[ok], (ys[ok] - K["cy"]) / K["fy"] * z[ok], z[ok]], 1)
        c = np.median(P, 0)
        Q = P - P.mean(0)
        w, V = np.linalg.eigh(Q.T @ Q / len(Q))
        a1, a2, nrm = V[:, 2], V[:, 1], V[:, 0]
        p1, p2 = Q @ a1, Q @ a2
        L = np.percentile(p1, 98) - np.percentile(p1, 2)
        S = np.percentile(p2, 98) - np.percentile(p2, 2)
        rms = float(np.sqrt(max(w[0], 0)))
        why = []
        Llo, Lhi = (LL * (1 - cfg["tol"]), LL * (1 + cfg["tol"])) if kind == "label" else (0.75 * LL, cfg["L"] * (1 + cfg["tol"]))
        Slo, Shi = (SS * cfg["s_lo"], SS * (1 + cfg["tol"])) if kind == "label" else (cfg["s_lo"] * SS, cfg["S"] * (1 + cfg["tol"]))
        if not (Llo <= L <= Lhi):
            why.append(f"L{L*1000:.0f}")
        if not (Slo <= S <= Shi):
            why.append(f"S{S*1000:.0f}")
        if L / max(S, 1e-6) < cfg["aspect"]:
            why.append(f"a{L / max(S, 1e-6):.2f}")
        if rms > cfg["plane_rms"]:
            why.append(f"rms{rms*1000:.1f}")
        # 막대 무늬: 긴 축 방향 화면 기울기 / 짧은 축 방향
        def img_dir(a):
            p0, p1_ = c, c + a * 0.01
            u0 = np.array([K["fx"] * p0[0] / p0[2] + K["cx"], K["fy"] * p0[1] / p0[2] + K["cy"]])
            u1 = np.array([K["fx"] * p1_[0] / p1_[2] + K["cx"], K["fy"] * p1_[1] / p1_[2] + K["cy"]])
            d = u1 - u0
            return d / np.linalg.norm(d)
        d1, d2 = img_dir(a1), img_dir(a2)
        ga = np.abs(gx[ys, xs] * d1[0] + gy[ys, xs] * d1[1]).mean()
        gb = np.abs(gx[ys, xs] * d2[0] + gy[ys, xs] * d2[1]).mean()
        stripe = ga / max(gb, 1e-6)
        if stripe < cfg["stripe"]:
            why.append(f"st{stripe:.2f}")
        if T_base_cam is not None:   # 과제 규칙: 벽(법선 수평) 또는 상자 윗면(법선 수직), 바닥은 안 됨
            zb = (T_base_cam @ np.append(c, 1.0))[2]
            nz = abs((T_base_cam[:3, :3] @ nrm)[2])
            if zb < cfg["min_z"]:
                why.append(f"z{zb:.2f}")
            if cfg["nz_wall"] < nz < cfg["nz_top"]:
                why.append(f"n{nz:.2f}")
        rec = dict(kind=kind, center=c, L=L, S=S, rms=rms, stripe=stripe, px=(float(xs.mean()), float(ys.mean())), why=why)
        if not why:
            if any(np.linalg.norm(o["center"] - c) < 0.01 for o in out if not o["why"]):
                continue
            out.append(rec)
        elif debug:
            out.append(rec)
    return out


if __name__ == "__main__":
    root = Path(sys.argv[1])
    rep = json.loads((root / "render_r3.json").read_text())
    K = rep["d435"]["K"]
    stats = {"hit": 0, "miss": 0, "false": 0}
    errs, misses = [], []
    for p in rep["poses"]:
        if p["kind"] != "d435":
            continue
        bgr = cv2.imread(str(root / p["images"]["color"]))
        dep = np.load(root / p["images"]["depth"])
        T = np.array(p["T_world_cam"])
        truth = np.linalg.inv(T) @ np.append(p["label_center"], 1.0)
        found = locate(bgr, dep, K, T_base_cam=T)
        d = [np.linalg.norm(f["center"] - truth[:3]) for f in found]
        good = [x for x in d if x <= 0.03]
        stats["false"] += sum(x > 0.05 for x in d)
        judged = p["label_visible"] >= 0.95
        if good:
            stats["hit"] += 1
            errs.append(min(good))
        elif judged:
            stats["miss"] += 1
            dbg = [(f["px"], f["why"], round(f["L"] * 1000), round(f["S"] * 1000), round(f["stripe"], 2))
                   for f in locate(bgr, dep, K, debug=True, T_base_cam=T)
                   if np.hypot(f["px"][0] - p["label_px"][0], f["px"][1] - p["label_px"][1]) < 25]
            misses.append((p["name"], p["where"], p["cam_dist_m"], dbg))
    print(stats, "다 보이는 장면", sum(p["kind"] == "d435" and p["label_visible"] >= 0.95 for p in rep["poses"]))
    if errs:
        print("오차 mm 중앙", round(np.median(errs) * 1000, 1), "최대", round(max(errs) * 1000, 1))
    for m in misses:
        print("놓침", m)
