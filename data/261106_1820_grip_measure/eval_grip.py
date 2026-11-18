"""#128 평가: 렌더 폴더를 재서 깊이 · 기울기 오차를 낸다 (모의).

사용: ~/isaacsim-venv/bin/python eval_grip.py <렌더 폴더> [--jobs 6] [--out 결과.json]
조건마다 프레임별 무작위 부호로 가정 오차를 넣는다(카메라 실제 자세 · K 는 렌더 값, 측정 계산에 쓰는 값만 틀리게).
  clean      : 잡음 · 오차 없음 (A)
  noise_jpeg : 영상 잡음 σ 2/255 + JPEG 품질 80 (압축 품질은 가정)
  f          : f ±2 %        cxy : 주점 ±5 px (가정)
  mt         : 장착 위치 ±1 mm    mr : 장착 각도 ±1° (가정, 근거 측정 없음)
  all        : 위 전부 (B)
  target     : 보정 뒤 목표 가정 — f ±0.5 %, 주점 ±1.5 px, 장착 ±0.25 mm · ±0.25°, 잡음 · JPEG 포함
  blur_N     : 가우시안 σ N px (참고, 판정 제외)
방법: plain = URDF · K 그대로 / finger = 손가락 위 검은 표시 2개로 화면 닮음 변환(이동 · 배율 · 회전) 보정
"""
import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure as M  # noqa: E402


MARK_BIAS = np.array([[-0.91, 0.59], [-0.19, 0.53]])


def visibility(fr, meta, dep):
    """아랫면 앞 가장자리(폭 ±10 mm)의 점 중 화면 안 · 안 가린 비율 (정답 자세, 렌더 깊이)."""
    T_G_p = np.array(fr["T_G_puck"])
    T = np.array(meta["T_G_cam"])
    R, c = T_G_p[:3, :3], T_G_p[:3, 3]
    z_p = R[:, 2]
    f = np.cross(M.E1, z_p)   # 앞(수평) 방향: 퍽 축 · y 와 직교
    f = f / np.linalg.norm(f)
    if f @ np.array([M.S, 0, M.S]) < 0:
        f = -f
    s = np.linspace(-0.010, 0.010, 41)
    off = np.sqrt(0.015 ** 2 - s ** 2) if fr["shape"] == "cylinder" else np.full_like(s, 0.015)
    pts = c - 0.03 * z_p + np.outer(off, f) + np.outer(s, M.E1)
    uv = M.project(pts, meta["K"], T)
    q = (pts - T[:3, 3]) @ T[:3, :3]
    ok = 0
    for (u, v), z in zip(uv, q[:, 2]):
        if 0 <= u < M.W and 0 <= v < M.H:
            # 가장자리 점은 화소 경계라 배경이 나올 수 있어 둘레 5 × 5 중 하나라도 그 깊이면 보임으로 친다
            win = dep[max(0, int(v) - 2):int(v) + 3, max(0, int(u) - 2):int(u) + 3] / 1e4
            ok += bool((np.abs(win - z) < 0.006).any())
    return ok / len(pts)


def perturb(kind, rng, K0, T0):
    K, T = dict(K0), T0.copy()
    sg = lambda: float(rng.choice([-1.0, 1.0]))  # noqa: E731
    if kind == "target":
        s = 1 + sg() * 0.005
        K["fx"] *= s
        K["fy"] *= s
        K["cx"] += sg() * 1.5
        K["cy"] += sg() * 1.5
        T[:3, 3] += T[:3, :3] @ (np.array([sg(), sg(), sg()]) * 0.00025)
        a = math.radians(0.25)
        T[:3, :3] = T[:3, :3] @ M.rot("x", sg() * a) @ M.rot("y", sg() * a) @ M.rot("z", sg() * a)
        return K, T
    if kind in ("f", "all"):
        s = 1 + sg() * 0.02
        K["fx"] *= s
        K["fy"] *= s
    if kind in ("cxy", "all"):
        K["cx"] += sg() * 5
        K["cy"] += sg() * 5
    if kind in ("mt", "all"):
        T[:3, 3] += T[:3, :3] @ (np.array([sg(), sg(), sg()]) * 0.001)
    if kind in ("mr", "all"):
        a = math.radians(1.0)
        T[:3, :3] = T[:3, :3] @ M.rot("x", sg() * a) @ M.rot("y", sg() * a) @ M.rot("z", sg() * a)
    return K, T


def degrade(bgr, kind, rng):
    if kind in ("noise_jpeg", "all", "target"):
        im = np.clip(bgr.astype(np.float32) + rng.normal(0, 2.0, bgr.shape), 0, 255).astype(np.uint8)
        _, b = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return cv2.imdecode(b, cv2.IMREAD_COLOR)
    if kind.startswith("blur_"):
        return cv2.GaussianBlur(bgr, (0, 0), float(kind.split("_")[1]))
    return bgr


def one(job):
    folder, fr, meta, kind, method, seed = job
    rng = np.random.default_rng(seed)
    bgr = cv2.imread(f"{folder}/{fr['id']}.png")
    K0, T0 = meta["K"], np.array(meta["T_G_cam"])
    nominal = np.array(meta["tool"]) - 0.015 * M.Z_P
    im = degrade(bgr, kind, rng)
    K, T = perturb(kind, rng, K0, T0)
    # 손가락은 |y| ≥ 15 mm 에 있어 카메라(y 0)에서 |y| ≤ 15 mm 인 퍽을 가리지 못한다 → 윤곽 점을 빼지 않는다
    fm = np.zeros((M.H, M.W), np.uint8)
    shift = (0.0, 0.0, 1.0, 0.0)
    if method == "finger":
        # 표시 무게 중심과 투영 사이의 고정 차이는 한 번 재서 뺀 것으로 본다 (설계 자료 r_design3 중앙값, 실물은 기준 장 한 번으로 재는 값)
        pred = M.project(np.array(meta["marks_G"]), K, T) + MARK_BIAS
        marks = M.fingertip_marks(im, pred, scale=K["fx"] / 424.5)
        if marks is None:
            return {"id": fr["id"], "shape": fr["shape"], "fail": "mark"}
        c = np.array([K["cx"], K["cy"]])
        zp, zm = complex(*(pred[1] - pred[0])), complex(*(marks[1] - marks[0]))
        q = zm / zp
        s_, ph = abs(q), np.angle(q)
        rot_c = complex(*(pred[0] - c)) * q
        d = marks[0] - c - np.array([rot_c.real, rot_c.imag])
        shift = (float(d[0]), float(d[1]), float(s_), float(ph))
    ex = M.extract(im, fr["color"], fr["shape"], K, T, nominal, fm)
    if ex is None or len(ex["contour"]) < 20:
        return {"id": fr["id"], "shape": fr["shape"], "fail": "mask"}
    p, cost = M.fit(ex, fr["shape"], K, T, nominal, fm, shift)
    T_G_p = np.array(fr["T_G_puck"])
    err_c = (nominal + p[:3]) - T_G_p[:3, 3]
    R_e, c_e = M.pose(p, nominal)
    # 퍽 바닥 가운데의 수평(집게 앞) 오차: 칸에 앉을지 정하는 점
    err_b = (c_e - 0.03 * R_e[:, 2]) - (T_G_p[:3, 3] - 0.03 * T_G_p[:3, 2])
    return {"id": fr["id"], "shape": fr["shape"], "cost": float(cost),
            "ex": float(err_c[0]) * 1e3, "ey": float(err_c[1]) * 1e3, "ez": float(err_c[2]) * 1e3,
            "ehor": float(err_c[0] + err_c[2]) * M.S * 1e3, "eup": float(err_c[2] - err_c[0]) * M.S * 1e3,
            "eth": math.degrees(p[3]) - fr["tilt_y_deg"], "ebot": float(err_b[0] + err_b[2]) * M.S * 1e3, "shift": list(shift)}


def summarize(rows):
    out = {}
    for sh in ("cylinder", "cuboid"):
        rs = [r for r in rows if r["shape"] == sh]
        ok = [r for r in rs if "fail" not in r]
        d = {"n": len(rs), "fail": len(rs) - len(ok)}
        for k in ("ex", "ez", "ehor", "ebot", "eup", "ey", "eth"):
            a = np.abs([r[k] for r in ok]) if ok else np.array([np.nan])
            d[k] = [round(float(np.median(a)), 2), round(float(np.percentile(a, 95)), 2)]
        out[sh] = d
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--kinds", nargs="+", default=["clean", "noise_jpeg", "f", "cxy", "mt", "mr", "all"])
    ap.add_argument("--methods", nargs="+", default=["plain"])
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--out", default=None)
    ap.add_argument("--auto-bias", action="store_true", help="표시 치우침을 이 폴더 앞 30 장 중앙값으로 (참고 렌더)")
    a = ap.parse_args()
    meta = json.load(open(f"{a.folder}/meta.json"))
    frames = meta["frames"]
    vis = []
    for fr in frames:
        dep = cv2.imread(f"{a.folder}/{fr['id']}_depth.png", cv2.IMREAD_UNCHANGED)
        vis.append({"shape": fr["shape"], "vis": visibility(fr, meta, dep)})
    res = {"visibility": {sh: {"n": len([v for v in vis if v["shape"] == sh]),
                               "frac_ge_0.8": round(float(np.mean([v["vis"] >= 0.8 for v in vis if v["shape"] == sh])), 3),
                               "median": round(float(np.median([v["vis"] for v in vis if v["shape"] == sh])), 3)}
                          for sh in ("cylinder", "cuboid")}}
    if a.auto_bias:
        K0, T0 = meta["K"], np.array(meta["T_G_cam"])
        pr = M.project(np.array(meta["marks_G"]), K0, T0)
        ds = [M.fingertip_marks(cv2.imread(f"{a.folder}/{fr['id']}.png"), pr, scale=K0["fx"] / 424.5, win=40) for fr in frames[:30]]
        MARK_BIAS = np.median(np.array([d - pr for d in ds if d is not None]), axis=0)
        print("표시 치우침", MARK_BIAS.round(2).tolist())
    for method in a.methods:
        for kind in a.kinds:
            jobs = [(a.folder, fr, meta, kind, method, 1000 + i) for i, fr in enumerate(frames)]
            with ProcessPoolExecutor(a.jobs) as ex:
                rows = list(ex.map(one, jobs))
            res[f"{method}/{kind}"] = summarize(rows)
            print(method, kind, json.dumps(res[f"{method}/{kind}"], ensure_ascii=False), flush=True)
            if a.out:
                Path(a.out + f".{method}.{kind}.rows.json").write_text(json.dumps(rows))
    print(json.dumps(res["visibility"], ensure_ascii=False))
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1, ensure_ascii=False))
