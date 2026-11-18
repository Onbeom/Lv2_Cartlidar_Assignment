"""#128 앞 D435 로 쥔 퍽 재기 평가 (모의). 윗면 태그 네 모서리 + 색 윤곽을 퍽 모형에 맞춘다.

사용: ~/isaacsim-venv/bin/python eval_d435.py <렌더 폴더> --out 결과.json
조건 (프레임별 무작위 부호, 계산에 쓰는 값만 틀리게):
  clean      : 잡음 · 오차 없음 (렌더의 관절 오차는 그대로)
  noise      : 영상 잡음 σ 2/255
  k          : f ±0.5 %, 주점 ±2 px (D435 는 공장 보정값이 있어 작게 가정)
  mount      : D435 장착 위치 ±1 mm · 각도 ±1° (가정)
  all        : 위 전부
  blur_N     : 가우시안 σ N px (참고)
방법: plain = 관절 값(FK)으로 집게 자세 / finger = 손가락 위 표시 2개로 화면 닮음 변환 보정 (관절 · 장착 오차를 지움)
"""
import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure as M  # noqa: E402

M.W, M.H = 640, 480
DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
P = cv2.aruco.DetectorParameters()
P.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
DET.setDetectorParameters(P)
TAG_H = 0.008
TAG_MODEL = np.array([[-TAG_H, -TAG_H, M.HALF_L], [TAG_H, -TAG_H, M.HALF_L], [TAG_H, TAG_H, M.HALF_L], [-TAG_H, TAG_H, M.HALF_L]])
MARK_BIAS = np.zeros((2, 2))


def sim_apply(uv, K, shift):
    du, dv, s, ph = shift
    a, b = uv[:, 0] - K["cx"], uv[:, 1] - K["cy"]
    c_, s_ = s * math.cos(ph), s * math.sin(ph)
    return np.column_stack([K["cx"] + c_ * a - s_ * b + du, K["cy"] + s_ * a + c_ * b + dv])


def find_tag(gray, K, T, nominal, shift):
    cs, ids, _ = DET.detectMarkers(gray)
    if ids is None:
        return None
    top = sim_apply(M.project((nominal + M.HALF_L * M.Z_P)[None], K, T), K, shift)[0]
    best = None
    for c in cs:
        c = c.reshape(4, 2).astype(np.float64)
        d = np.linalg.norm(c.mean(0) - top)
        if best is None or d < best[0]:
            best = (d, c)
    return best[1] if best[0] < 60 else None


def fit_puck(ex, shape, K, T, nominal, tag, shift):
    mp = M.model_points(shape)
    cont = ex["contour"][:: max(1, len(ex["contour"]) // 150)]
    SC = np.array([1e-3, 1e-3, 1e-3, math.radians(1), math.radians(1), math.radians(1)])
    prior = np.array([0, 1.0, 0, 0, 0.0, 0.0])

    def proj(P_, ):
        return sim_apply(M.project(P_, K, T), K, shift)

    def res(x, order):
        R, c = M.pose(x * SC, nominal)
        hull, line = M.hull_outline(proj(mp @ R.T + c))
        ok = (line[:, 0] > 2) & (line[:, 0] < M.W - 3) & (line[:, 1] > 2) & (line[:, 1] < M.H - 3)
        d = M.bilinear(ex["dt"], line[ok]) if ok.any() else np.zeros(1)
        r1 = d[np.linspace(0, len(d) - 1, 80).astype(int)]
        hull32 = hull.astype(np.float32).reshape(-1, 1, 2)
        r2 = np.array([cv2.pointPolygonTest(hull32, (float(u), float(v)), True) for u, v in cont]) if len(cont) else np.zeros(1)
        rs = [r1 * 0.5, r2 * 0.5, prior * x]
        if tag is not None:
            uv = proj(TAG_MODEL[order] @ R.T + c)
            rs.append((uv - tag).ravel() * 4.0)
        return np.concatenate(rs)

    best = None
    orders = [list(np.roll([0, 1, 2, 3], k)) for k in range(4)] if tag is not None else [[0, 1, 2, 3]]
    # 태그 회전: 시작점을 잘 고르기 위해 spin 을 12 곳에서 비용만 본 뒤 가장 좋은 순서 · 각에서 푼다
    cand = []
    for order in orders:
        for sp in np.arange(-180, 180, 30.0) if shape == "cylinder" else (0.0,):
            x0 = np.array([0, 0, 0, 0, 0, sp], float)
            cand.append((float(np.sum(np.minimum(np.abs(res(x0, order)), 20.0))), order, sp))
    cand.sort(key=lambda t: t[0])
    for _, order, sp in cand[:4]:
        lo = [-14, -3, -14, -25, -8, sp - 40]
        hi = [14, 3, 14, 25, 8, sp + 40]
        x0 = np.array([0, 0, 0, 0, 0, sp], float)
        sol = least_squares(res, x0, args=(order,), bounds=(lo, hi), loss="soft_l1", f_scale=2.0, max_nfev=40,
                            x_scale=[1, 1, 1, 2, 2, 2], diff_step=None)
        if best is None or sol.cost < best.cost:
            best = sol
    return best.x * SC, best.cost


def perturb(kind, rng, K0, T_bf_cam):
    K, Tc = dict(K0), T_bf_cam.copy()
    sg = lambda: float(rng.choice([-1.0, 1.0]))  # noqa: E731
    if kind in ("k", "all"):
        s = 1 + sg() * 0.005
        K["fx"] *= s
        K["fy"] *= s
        K["cx"] += sg() * 2
        K["cy"] += sg() * 2
    if kind in ("mount", "all"):
        Tc[:3, 3] += Tc[:3, :3] @ (np.array([sg(), sg(), sg()]) * 0.001)
        a = math.radians(1.0)
        Tc[:3, :3] = Tc[:3, :3] @ M.rot("x", sg() * a) @ M.rot("y", sg() * a) @ M.rot("z", sg() * a)
    return K, Tc


def degrade(bgr, kind, rng):
    if kind in ("noise", "all"):
        return np.clip(bgr.astype(np.float32) + rng.normal(0, 2.0, bgr.shape), 0, 255).astype(np.uint8)
    if kind.startswith("blur_"):
        return cv2.GaussianBlur(bgr, (0, 0), float(kind.split("_")[1]))
    return bgr


def one(job):
    folder, fr, meta, kind, method, seed, bias = job
    rng = np.random.default_rng(seed)
    im = degrade(cv2.imread(f"{folder}/{fr['id']}.png"), kind, rng)
    K, Tbc = perturb(kind, rng, meta["K"], np.array(meta["T_BF_cam"]))
    T_bf_G = np.array(fr["T_bf_G_cmd"])          # 명령 관절로 계산한 집게 자세 (실제와 다름)
    T = np.linalg.inv(T_bf_G) @ Tbc
    nominal = np.array(meta["tool"]) - 0.015 * M.Z_P
    shift = (0.0, 0.0, 1.0, 0.0)
    if method == "finger":
        pred = M.project(np.array(meta["marks_G"]), K, T) + bias
        marks = M.fingertip_marks(im, pred, win=50, scale=0.6)
        if marks is None:
            return {"id": fr["id"], "shape": fr["shape"], "fail": "mark"}
        c = np.array([K["cx"], K["cy"]])
        zp, zm = complex(*(pred[1] - pred[0])), complex(*(marks[1] - marks[0]))
        q = zm / zp
        rc = complex(*(pred[0] - c)) * q
        d = marks[0] - c - np.array([rc.real, rc.imag])
        shift = (float(d[0]), float(d[1]), float(abs(q)), float(np.angle(q)))
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    tag = find_tag(gray, K, T, nominal, shift)
    fm = np.zeros((M.H, M.W), np.uint8)
    # extract 는 집게 좌표 ROI 를 쓰므로 닮음 변환 없이 부른다: ROI 여유를 넉넉히
    ex = M.extract(im, fr["color"], fr["shape"], K, T, nominal, fm, roi_margin=0.03)
    if ex is None or len(ex["contour"]) < 20:
        return {"id": fr["id"], "shape": fr["shape"], "fail": "mask", "tag": tag is not None}
    p, cost = fit_puck(ex, fr["shape"], K, T, nominal, tag, shift)
    T_G_p = np.array(fr["T_G_puck"])
    R_e, c_e = M.pose(p, nominal)
    ax_t = T_G_p[:3, 2] * np.sign(T_G_p[:3, 2] @ M.Z_P)
    err_c = c_e - T_G_p[:3, 3]
    err_b = (c_e - 0.03 * R_e[:, 2]) - (T_G_p[:3, 3] - 0.03 * ax_t)
    tilt_err = math.degrees(math.acos(np.clip(R_e[:, 2] @ ax_t, -1, 1)))
    return {"id": fr["id"], "shape": fr["shape"], "pose": fr["pose"], "cost": float(cost), "tag": tag is not None,
            "ex": float(err_c[0]) * 1e3, "ey": float(err_c[1]) * 1e3, "ez": float(err_c[2]) * 1e3,
            "ehor": float(err_c[0] + err_c[2]) * M.S * 1e3, "eup": float(err_c[2] - err_c[0]) * M.S * 1e3,
            "ebot": float(err_b[0] + err_b[2]) * M.S * 1e3, "enorm": float(np.linalg.norm(err_c)) * 1e3, "eth": tilt_err}


def summarize(rows):
    out = {}
    for sh in ("cylinder", "cuboid"):
        rs = [r for r in rows if r["shape"] == sh]
        ok = [r for r in rs if "fail" not in r]
        d = {"n": len(rs), "fail": len(rs) - len(ok), "tag": sum(bool(r.get("tag")) for r in ok)}
        for k in ("ex", "ez", "ehor", "ebot", "eup", "ey", "enorm", "eth"):
            a = np.abs([r[k] for r in ok]) if ok else np.array([np.nan])
            d[k] = [round(float(np.median(a)), 2), round(float(np.percentile(a, 95)), 2)]
        out[sh] = d
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--kinds", nargs="+", default=["clean", "noise", "k", "mount", "all"])
    ap.add_argument("--methods", nargs="+", default=["plain", "finger"])
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    meta = json.load(open(f"{a.folder}/meta.json"))
    frames = meta["frames"]
    # 표시 치우침: 이 폴더 앞 20 장, 실제 집게 자세로 (실물은 기준 장 한 번)
    ds = []
    for fr in frames[:20]:
        T = np.linalg.inv(np.array(fr["T_bf_G"])) @ np.array(meta["T_BF_cam"])
        pr = M.project(np.array(meta["marks_G"]), meta["K"], T)
        d = M.fingertip_marks(cv2.imread(f"{a.folder}/{fr['id']}.png"), pr, win=50, scale=0.6)
        if d is not None:
            ds.append(d - pr)
    bias = np.median(np.array(ds), axis=0) if ds else np.zeros((2, 2))
    print("표시 찾음", len(ds), "/ 20, 치우침", bias.round(2).tolist(), flush=True)
    res = {}
    for method in a.methods:
        for kind in a.kinds:
            jobs = [(a.folder, fr, meta, kind, method, 2000 + i, bias) for i, fr in enumerate(frames)]
            with ProcessPoolExecutor(a.jobs) as ex:
                rows = list(ex.map(one, jobs))
            res[f"{method}/{kind}"] = summarize(rows)
            print(method, kind, json.dumps(res[f"{method}/{kind}"], ensure_ascii=False), flush=True)
            if a.out:
                Path(a.out + f".{method}.{kind}.rows.json").write_text(json.dumps(rows))
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1, ensure_ascii=False))
