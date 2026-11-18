"""D435 바코드 라벨 찾기 시제품 2: 깊이로 평면(벽 · 윗면) 나누기 → 평면마다 정면으로 펴기 → 라벨 모양 템플릿 정규 상관.

시제품 1(밝은 덩어리 → 크기)은 선반 윗면에서 라벨 덩어리가 선반 모서리와 붙어 못 찾았다.
여기서는 덩어리를 먼저 나누지 않고, 평면을 위에서 본 영상(mm 격자)으로 바꾼 뒤 35 × 20 mm 라벨 그림을 돌려 가며 맞춘다.
"""
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CFG2 = dict(mm=1.5, rng=1.2, min_z=0.06, nz_wall=0.25, nz_top=0.9, edge_deg=12.0, smooth=15, nk=3, median=5, tol_k=3.0, rot_step=7.5,
            thr=0.65, nms=0.03, depth_tol=0.006, valid_frac=0.90, min_comp=150, min_std=4.0, plane_rms=0.004, psf_px=0.9,
            ring_in=4.0, ring_out=10.0, ring_min_valid=0.0, ring_max=0.35, ring_drop_worst=True, rect_margin=0.06, win_erode=1, bw_min=0.4, wall_min_top=0.20, img_margin=4, sat_max=40.0, sharp_min=-0.05, resw_min=-99.0, ring_rel_max=0.06, ring_mad=False, ring_lin=True, sat_hi_max=999.0, side_min=-0.45, constrain=0, max_blur=20, tol_inc=True, local_psf=False, lift_max=9.0, split=True, split_tol=0.008, split_iter=300,
            wall_min_w=0.06, bw_max=9.0, white_ring_min=0.98, top_h=0.2, top_hk=5, side_min_frac=0.3, rescore_m=0.05,   # 44차: 좁은 틈 · 구석 · 조금 가린 윗면 라벨
            label_mm=(35.0, 20.0), tex=str(HERE.parents[1] / "sim_world/textures/barcode_P2-01.png"), use_text=True, keep=False)


def _templates(cfg):
    tex = cv2.imread(cfg["tex"], cv2.IMREAD_GRAYSCALE).astype(np.float32)
    w, h = [int(round(x / cfg["mm"])) for x in cfg["label_mm"]]
    t = cv2.resize(tex, (w, h), interpolation=cv2.INTER_AREA)
    D = int(np.ceil(np.hypot(w, h))) | 1
    out = []
    for th in np.arange(0.0, 360.0, cfg["rot_step"]):   # 글자 줄 때문에 위아래가 달라 한 바퀴 다 찾음
        M = cv2.getRotationMatrix2D(((w - 1) / 2, (h - 1) / 2), th, 1.0)
        M[:, 2] += [(D - w) / 2, (D - h) / 2]
        # 바깥은 가장자리 값으로 채움: 나중에 화소 넓이만큼 흐릴 때 0 이 흰 테두리로 번지지 않게
        tr = cv2.warpAffine(t, M, (D, D), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        mk = cv2.warpAffine(np.ones_like(t), M, (D, D), flags=cv2.INTER_NEAREST, borderValue=0)
        mk = (mk > 0.5).astype(np.float32)
        if not cfg["use_text"]:   # 글자 줄(아래 4.5 mm)은 실물 라벨마다 다를 수 있어 빼 볼 수 있게
            keep = np.zeros_like(t); keep[: int(round(15.5 / cfg["mm"])), :] = 1
            mk *= (cv2.warpAffine(keep, M, (D, D), flags=cv2.INTER_NEAREST, borderValue=0) > 0.5)
        rot = lambda a: (cv2.warpAffine(a.astype(np.float32), M, (D, D), flags=cv2.INTER_NEAREST, borderValue=0) > 0.5)
        out.append((th, tr, mk, rot(t >= 230) & (mk > 0), rot(t <= 180) & (mk > 0)))   # 흰 테두리 · 막대 띠
    return out


_TPL = {}


def _ncc(w, t, m):
    w, t = w[m], t[m]
    w = w - w.mean(); t = t - t.mean()
    return float(w @ t / max(np.sqrt((w @ w) * (t @ t)), 1e-9))


def _shape_feats(win, tpb, tp0, mk0, ker, bright=None, th_deg=0.0, lwh=None):
    """후보 창을 '밝기 a · 흐린 템플릿 + b' 로 맞춘 뒤 남는 것과, 더 흐린 템플릿과 견준 선명도."""
    m = mk0 > 0
    A = np.stack([tpb[m], np.ones(m.sum(), np.float32)], 1)
    (a, b), *_ = np.linalg.lstsq(A, win[m], rcond=None)
    span = max(abs(a) * (tpb[m].max() - tpb[m].min()), 1e-6)   # 템플릿이 예상하는 명암 폭
    res = (win[m] - (a * tpb[m] + b)) / span
    # 화소 넓이만큼 흐린 템플릿과, 그 흐림을 두 배로 한 템플릿의 상관 차: 라벨은 가장자리가 카메라 흐림만큼만 흐리다
    k2 = cv2.resize(ker, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR); k2 /= k2.sum()
    tb2 = cv2.filter2D(tp0, -1, k2, borderType=cv2.BORDER_REPLICATE)
    rw = ((win - (a * tpb + b)) / span)[bright] if bright is not None and bright.sum() > 10 else res
    side_lo = 0.0
    if bright is not None and lwh is not None:   # 흰 테두리를 네 변(위 · 아래 · 왼 · 오른)으로 나눠 가장 어두운 변의 평균 잔차
        D = win.shape[0]; th = np.radians(th_deg)
        yy, xx = np.nonzero(bright); dx, dy = xx - (D - 1) / 2, yy - (D - 1) / 2
        la = dx * np.cos(th) - dy * np.sin(th); lb = dx * np.sin(th) + dy * np.cos(th)
        R_ = (win - (a * tpb + b)) / span
        rv = R_[yy, xx]; hw, hh = lwh[0] / 2, lwh[1] / 2
        sides = [lb < -hh * 0.5, lb > hh * 0.5, la < -hw * 0.6, la > hw * 0.6]
        side_lo = float(min(rv[m].mean() for m in sides if m.sum() >= 5))
    return dict(res_lo=float(np.percentile(res, 1)), res_hi=float(np.percentile(res, 99)), resw_lo=float(np.percentile(rw, 2)), side_lo=side_lo,
                dark_frac=float(np.mean(rw < -0.5)),   # 흰 테두리 자리 중 크게 어두운 화소 비율(태그의 검은 칸은 넓게 차지)
                resw_std=float(rw.std()),
                sharp=_ncc(win, tpb, m) - _ncc(win, tb2, m), fit_a=float(a))


def surfaces(dep, K, T_base_cam, cfg=CFG2):
    """화소마다 법선(base 기준)을 구해 벽(법선 수평) · 윗면(법선 수직) 평면 덩어리로 나눈다."""
    H, W = dep.shape
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    z = dep.astype(np.float32)
    ok = np.isfinite(z) & (z > 0.15) & (z < 4.0)
    z = np.where(ok, z, 0)
    if cfg["median"]:   # 깊이 잡음(실물 D435 는 1 m 에서 수 mm) 줄이기
        z = np.where(ok, cv2.medianBlur(z, cfg["median"]), 0)
    P = np.dstack([(uu - K["cx"]) / K["fx"] * z, (vv - K["cy"]) / K["fy"] * z, z])
    s = cfg["smooth"]
    wsum = cv2.blur(ok.astype(np.float32), (s, s))
    Ps = np.dstack([cv2.blur(P[..., i], (s, s)) for i in range(3)]) / np.maximum(wsum, 1e-6)[..., None]
    k = cfg["nk"]
    dx = np.zeros_like(Ps); dy = np.zeros_like(Ps)
    dx[:, k:-k] = Ps[:, 2 * k:] - Ps[:, :-2 * k]
    dy[k:-k] = Ps[2 * k:] - Ps[:-2 * k]
    n = np.cross(dx, dy)
    n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
    n *= -np.sign(np.sum(n * Ps, axis=2, keepdims=True) + 1e-12)   # 카메라 쪽을 보게
    R, t = T_base_cam[:3, :3], T_base_cam[:3, 3]
    nb = n @ R.T
    Pb = P @ R.T + t
    c = np.cos(np.radians(cfg["edge_deg"]))
    edge = np.zeros((H, W), bool)
    for sh in [(0, k), (k, 0)]:
        n2 = np.roll(n, sh, axis=(0, 1))
        edge |= np.sum(n * n2, axis=2) < c
        z2 = np.roll(z, sh, axis=(0, 1))
        edge |= np.abs(z - z2) > 0.02 * np.maximum(z, 0.2)
    good = ok & (wsum > 0.99) & ~edge & (Pb[..., 2] >= cfg["min_z"]) & (np.hypot(Pb[..., 0], Pb[..., 1]) <= cfg["rng"])
    top_m = good & (nb[..., 2] >= cfg["nz_top"])
    if cfg["top_h"] > 0:
        # 좁은 틈(벽 · 퍽 사이 수십 mm)의 윗면은 법선 창에 옆 면이 섞여 법선이 틀어진다.
        # 수평면은 깊이 잡음(시선 방향)이 있어도 높이는 거의 그대로(비스듬할수록 덜 변함) →
        # 작은 창에서 '높이 퍼짐 / 수평 퍼짐' 이 작으면 윗면으로 더한다(벽 · 퍽 옆면은 수평 퍼짐이 작고 높이 퍼짐이 큼)
        hk = cfg["top_hk"]
        okf = ok.astype(np.float32); wk = cv2.blur(okf, (hk, hk))
        def var(a):
            m1 = cv2.blur(a * okf, (hk, hk)) / np.maximum(wk, 1e-6)
            return cv2.blur(a * a * okf, (hk, hk)) / np.maximum(wk, 1e-6) - m1 * m1
        vh = var(Pb[..., 2]); vxy = var(Pb[..., 0]) + var(Pb[..., 1])
        flat = np.sqrt(np.maximum(vh, 0) / np.maximum(vxy, 1e-12)) <= cfg["top_h"]
        jump = np.zeros((H, W), bool)
        for sh in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            jump |= np.abs(z - np.roll(z, sh, axis=(0, 1))) > 0.02 * np.maximum(z, 0.2)
        top_m |= ok & (wk > 0.99) & flat & ~cv2.dilate(jump.astype(np.uint8), np.ones((hk, hk), np.uint8)).astype(bool) \
            & (Pb[..., 2] >= cfg["min_z"]) & (np.hypot(Pb[..., 0], Pb[..., 1]) <= cfg["rng"])
    planes = []
    for kind, m in [("wall", good & (np.abs(nb[..., 2]) <= cfg["nz_wall"])), ("top", top_m)]:
        cnt, lab, st, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
        comps = []
        for i in range(1, cnt):
            if st[i, cv2.CC_STAT_AREA] >= cfg["min_comp"]:
                ys, xs = np.nonzero(lab == i)
                comps += _split(P, ys, xs, cfg) if cfg["split"] else [(ys, xs)]
        for ys, xs in comps:
            Q = P[ys, xs]
            c0 = Q.mean(0)
            w_, V = np.linalg.eigh((Q - c0).T @ (Q - c0) / len(Q))
            nrm = V[:, 0] * (-1 if V[:, 0] @ c0 > 0 else 1)
            if cfg["constrain"] and (cfg["constrain"] is True or cfg["constrain"] == 1 or (cfg["constrain"] == 2 and kind == "top")):
                # 윗면은 수평(법선 = base z축), 벽은 수직(법선이 수평): 작은 평면에서 잡음으로 법선이 틀어지지 않게 자유도를 줄임
                Qb = Pb[ys, xs]
                if kind == "top":
                    nb_ = np.array([0, 0, 1.0])
                else:
                    xy = Qb[:, :2] - Qb[:, :2].mean(0)
                    _, V2 = np.linalg.eigh(xy.T @ xy)
                    nb_ = np.array([V2[0, 0], V2[1, 0], 0.0])
                nrm = R.T @ nb_
                nrm *= -1 if nrm @ c0 > 0 else 1
                w_ = np.array([np.mean(((Q - c0) @ nrm) ** 2), 0, 0])
            r = (Q - c0) @ nrm
            sig = 1.4826 * float(np.median(np.abs(r - np.median(r))))   # 이 평면에서 잰 깊이 잡음(강건 표준편차)
            planes.append(dict(kind=kind, c=c0, n=nrm, rms=float(np.sqrt(max(w_[0], 0))), sig=sig,
                               zmed=float(np.median(Q[:, 2])), ys=ys, xs=xs,
                               ztop=float(np.percentile(Pb[ys, xs, 2], 98))))
    return planes, P, z


def _split(P, ys, xs, cfg, depth=0):
    """평면 하나로 맞지 않는 덩어리(잡음으로 모서리 너머 면과 이어진 것)를 RANSAC 으로 평면 하나씩 떼어 낸다."""
    Q = P[ys, xs]
    c0 = Q.mean(0)
    w_, V = np.linalg.eigh((Q - c0).T @ (Q - c0) / len(Q))
    if np.sqrt(max(w_[0], 0)) <= cfg["split_tol"] / 2 or depth >= 3:
        return [(ys, xs)]
    rng = np.random.default_rng(len(ys))
    best, bn = None, 0
    for _ in range(cfg["split_iter"]):
        a, b, c = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(b - a, c - a); nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        inl = np.abs((Q - a) @ (n / nn)) < cfg["split_tol"]
        if inl.sum() > bn:
            best, bn = inl, inl.sum()
    if best is None or bn < cfg["min_comp"]:
        return [(ys, xs)]
    out = []
    for keep in (best, ~best):   # 맞는 쪽 · 나머지 각각 다시 이어진 덩어리로 나눔
        m = np.zeros(P.shape[:2], np.uint8); m[ys[keep], xs[keep]] = 1
        cnt, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        for i in range(1, cnt):
            if st[i, cv2.CC_STAT_AREA] >= cfg["min_comp"]:
                y2, x2 = np.nonzero(lab == i)
                out += [(y2, x2)] if keep is best else _split(P, y2, x2, cfg, depth + 1)
    return out


def locate(bgr, dep, K, T_base_cam, cfg=CFG2, debug=False):
    key = (cfg["mm"], cfg["rot_step"], cfg["tex"], cfg["use_text"])
    if key not in _TPL:
        _TPL[key] = _templates(cfg)
    tpls = _TPL[key]
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    sat = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[..., 1].astype(np.float32)
    H, W = g.shape
    planes, P, zf = surfaces(dep, K, T_base_cam, cfg)
    R = T_base_cam[:3, :3]
    zb_cam = R.T @ np.array([0, 0, 1.0])     # base 위쪽 방향을 카메라 좌표로
    out = []
    for pl in planes:
        if pl["rms"] > cfg["plane_rms"] + 2 * pl["sig"]:   # 잡음보다 크게 휜 면은 평면이 아님
            continue
        # 과제 규칙: 라벨은 벽판 또는 상자 윗면에만(상자 옆면 · 바닥은 안 됨) → 수직면은 상자보다 높이 올라가야 벽으로 봄
        if pl["kind"] == "wall" and pl["ztop"] < cfg["wall_min_top"]:
            continue
        n, c0 = pl["n"], pl["c"]
        if pl["kind"] == "wall":
            e1 = np.cross(zb_cam, n)
        else:
            e1 = R.T @ np.array([1.0, 0, 0]); e1 -= (e1 @ n) * n
        e1 /= np.linalg.norm(e1); e2 = np.cross(e1, n)   # 카메라 쪽에서 본 오른쪽 · 아래 (거울상 방지)
        Q = P[pl["ys"], pl["xs"]] - c0
        s, t = Q @ e1, Q @ e2
        m = cfg["rect_margin"]   # 법선 다듬기로 평면 가장자리가 깎이므로 둘레까지 펴서 깊이로 평면 위인지 본다
        s0, s1, t0, t1 = s.min() - m, s.max() + m, t.min() - m, t.max() + m
        # 평면 가로 폭(벽은 e1 이 수평): 라벨은 벽판(380 mm)에만 붙고, 미로 기둥 옆면(30 ~ 36 mm)에는 없다
        pl_w = float(np.percentile(s, 99) - np.percentile(s, 1))
        if pl["kind"] == "wall" and pl_w < cfg["wall_min_w"]:
            continue
        mm = cfg["mm"] / 1000
        ns, nt = int((s1 - s0) / mm) + 1, int((t1 - t0) / mm) + 1
        if ns * nt > 4e6:
            continue
        S, Tt = np.meshgrid(s0 + mm * np.arange(ns), t0 + mm * np.arange(nt))
        X = c0 + S[..., None] * e1 + Tt[..., None] * e2
        Z = X[..., 2]
        u = (K["fx"] * X[..., 0] / Z + K["cx"]).astype(np.float32)
        v = (K["fy"] * X[..., 1] / Z + K["cy"]).astype(np.float32)
        Rg = cv2.remap(g, u, v, cv2.INTER_LINEAR, borderValue=0)
        Rs = cv2.remap(sat, u, v, cv2.INTER_LINEAR, borderValue=0)
        Dd = cv2.remap(zf, u, v, cv2.INTER_NEAREST, borderValue=0)
        # 평면 위인지: 허용치 = 고정 6 mm + 이 평면에서 잰 잡음 × tol_k (거리 제곱에 비례해 늘림)
        tol = cfg["depth_tol"] + cfg["tol_k"] * pl["sig"] * (Z / pl["zmed"]) ** 2
        if cfg["tol_inc"]:
            # sig 는 평면 법선 방향 잡음, 비교는 시선 방향 깊이 차 → 1/cos(입사각) 배(비스듬한 면일수록 큼)
            cos_i = np.abs(X @ n) / np.maximum(np.linalg.norm(X, axis=2), 1e-6)
            tol = cfg["depth_tol"] + cfg["tol_k"] * pl["sig"] * (Z / pl["zmed"]) ** 2 / np.maximum(cos_i, 0.2)
        valid = ((u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1) & (Z > 0.15)
                 & (np.abs(Dd - Z) < tol)).astype(np.float32)
        # 카메라 화소 하나가 평면에서 덮는 넓이(비스듬하면 한쪽으로 길다)만큼 템플릿을 흐리게 한다
        def proj(X_):
            return np.array([K["fx"] * X_[0] / X_[2] + K["cx"], K["fy"] * X_[1] / X_[2] + K["cy"]])
        def psf_kernel(Xp):
            J = np.stack([(proj(Xp + 1e-3 * e1) - proj(Xp)) / 1e-3, (proj(Xp + 1e-3 * e2) - proj(Xp)) / 1e-3], 1)
            Ji = np.linalg.inv(J)
            Sig = Ji @ Ji.T * cfg["psf_px"] ** 2 / mm ** 2      # 펴진 영상 화소 단위
            r_f = 3 * np.sqrt(np.linalg.eigvalsh(Sig).max())
            # 평면이 시선과 거의 나란하면 화소 하나가 평면에서 아주 길게 늘어나 흐림 커널이 수만 칸이 됨(메모리 수십 GB).
            # 그런 평면에서는 라벨(20 mm)이 몇 화소도 안 되어 찾을 수 없으므로 건너뜀
            if not np.isfinite(r_f) or r_f > cfg["max_blur"]:
                return None
            r_k = int(np.ceil(r_f))
            ky, kx = np.mgrid[-r_k:r_k + 1, -r_k:r_k + 1]
            Si = np.linalg.inv(Sig)
            k_ = np.exp(-0.5 * (Si[0, 0] * kx * kx + 2 * Si[0, 1] * kx * ky + Si[1, 1] * ky * ky)).astype(np.float32)
            return k_ / k_.sum()
        ker = psf_kernel(c0 + np.median(s) * e1 + np.median(t) * e2)
        if ker is None:
            continue
        # 라벨 창 전체가 평면 경계에서 조금 안쪽(win_erode 화소)에 있어야 함: 경계 화소는 옆 면 밝기가 섞임
        valid_w = cv2.erode(valid, np.ones((2 * cfg["win_erode"] + 1,) * 2, np.uint8)) if cfg["win_erode"] else valid
        best = np.full((nt, ns), -1.0, np.float32); bth = np.zeros((nt, ns), np.float32)
        tpb = {}   # 이 평면에서 흐린 템플릿(후보 검사에 다시 씀)
        for th, tp, mk, _, _ in tpls:
            tp = cv2.filter2D(tp, -1, ker, borderType=cv2.BORDER_REPLICATE)
            tpb[th] = tp
            D = tp.shape[0]
            if nt < D or ns < D:
                break
            sc = cv2.matchTemplate(Rg, tp, cv2.TM_CCOEFF_NORMED, mask=mk)
            sc = np.nan_to_num(sc, nan=-1, posinf=-1, neginf=-1)
            area = mk.sum()
            vf = cv2.matchTemplate(valid_w, mk, cv2.TM_CCORR) / area
            mu = cv2.matchTemplate(Rg, mk, cv2.TM_CCORR) / area
            m2 = cv2.matchTemplate(Rg * Rg, mk, cv2.TM_CCORR) / area
            sd = np.sqrt(np.maximum(m2 - mu * mu, 0))
            sc[(vf < cfg["valid_frac"]) | (sd < cfg["min_std"])] = -1
            h0 = D // 2
            reg = best[h0:h0 + sc.shape[0], h0:h0 + sc.shape[1]]
            upd = sc > reg
            reg[upd] = sc[upd]
            bth[h0:h0 + sc.shape[0], h0:h0 + sc.shape[1]][upd] = th
        # 봉우리: 둘레 nms 안 최댓값
        r = max(1, int(cfg["nms"] / mm))
        dil = cv2.dilate(best, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        lw, lh = [x / cfg["mm"] for x in cfg["label_mm"]]
        valid_er = cv2.erode(valid, np.ones((5, 5), np.uint8))   # 평면 가장자리에서 섞인 화소 빼기
        tpl_by_th = {x[0]: x[1:] for x in tpls}
        for (ty, sx) in zip(*np.nonzero((best >= cfg["thr"] - cfg["rescore_m"]) & (best >= dil))):
            # 둘레 띠: 라벨은 무늬 없는 면에 붙인 스티커 → 바깥 띠(평면 위 화소만)는 고르고, 라벨 안은 무늬가 있다
            th = np.radians(bth[ty, sx])
            a = np.array([np.cos(th), -np.sin(th)]); b = np.array([np.sin(th), np.cos(th)])
            def rect(grow):
                gw, gh = lw / 2 + grow / cfg["mm"], lh / 2 + grow / cfg["mm"]
                return np.round([np.array([sx, ty]) + i * gw * a + j * gh * b for i, j in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]).astype(np.int32)
            m_lab = np.zeros((nt, ns), np.uint8); m_in = np.zeros_like(m_lab); m_out = np.zeros_like(m_lab)
            cv2.fillPoly(m_lab, [rect(0)], 1); cv2.fillPoly(m_in, [rect(cfg["ring_in"])], 1); cv2.fillPoly(m_out, [rect(cfg["ring_out"])], 1)
            ring = (m_out > 0) & (m_in == 0)
            rv = ring & (valid_er > 0)
            lab_v = (m_lab > 0) & (valid > 0)
            ring_valid = rv.sum() / max(ring.sum(), 1)
            ring_std = float(Rg[rv].std()) if rv.sum() > 10 else 1e9
            lab_std = float(Rg[lab_v].std()) if lab_v.sum() > 10 else 0.0
            ring_ratio = ring_std / max(lab_std, 1e-6)   # 기록용
            ring_mean = float(Rg[rv].mean()) if rv.sum() > 10 else 0.0
            # 라벨 흰 테두리(템플릿 밝은 곳) 평균과 둘레 비교: 스티커 흰색은 둘레 면보다 어둡지 않다
            tp0, mk0, bright, dark = tpl_by_th[float(bth[ty, sx])]
            D = tp0.shape[0]; y0_, x0_ = ty - D // 2, sx - D // 2
            win = Rg[y0_:y0_ + D, x0_:x0_ + D] if y0_ >= 0 and x0_ >= 0 else np.zeros((0, 0))
            if win.shape == tp0.shape:
                white_mean = float(win[bright].mean()); band_mean = float(win[dark].mean())
                sat_mean = float(Rs[y0_:y0_ + D, x0_:x0_ + D][mk0 > 0].mean())
            else:
                white_mean = band_mean = 0.0; sat_mean = 1e9
            score = float(best[ty, sx])
            if win.shape != tp0.shape:
                continue
            tpb_c, ker_c = tpb[float(bth[ty, sx])], ker
            if score < cfg["thr"]:
                # 문턱 바로 아래 후보: 라벨 앞을 물체가 조금 가리면(평면 밖 화소, 창의 5 % 미만) 그 화소가 상관을 끌어내림
                # → 평면 위 화소만으로 다시 잰다
                mv = (mk0 > 0) & (valid[y0_:y0_ + D, x0_:x0_ + D] > 0)
                score = _ncc(win, tpb_c, mv)
                if score < cfg["thr"]:
                    continue
            if cfg["local_psf"]:
                # 흐림은 평면 한 점(가운데)에서 구했으므로, 후보 자리에서 다시 구해 점수 · 특징을 다시 잰다
                # (큰 평면의 가까운 쪽 후보가 실제보다 흐린 템플릿과 비교되면 부드러운 그림자와 잘 맞아 버림)
                ker_c = psf_kernel(c0 + (s0 + mm * sx) * e1 + (t0 + mm * ty) * e2)
                if ker_c is None:
                    continue
                tpb_c = cv2.filter2D(tp0, -1, ker_c, borderType=cv2.BORDER_REPLICATE)
                score = _ncc(win, tpb_c, mk0 > 0)
                if score < cfg["thr"]:
                    continue
            feat = _shape_feats(win, tpb_c, tp0, mk0, ker_c, bright, float(bth[ty, sx]), (lw, lh))
            # 라벨 창이 평면에서 떠 있는 높이(시선 방향 깊이 차의 중앙값, m): 스티커는 거의 0,
            # 평면에 닿은 원기둥 옆면(닿은 선 둘레는 깊이 허용치 안)은 몇 mm 떠 있다
            lv = lab_v if lab_v.sum() > 10 else (m_lab > 0)
            lift = float(np.median((Z - Dd)[lv]))
            if lift > cfg["lift_max"]:
                continue
            # 바코드 라벨은 흑백 인쇄 → 채도가 낮다(빨강 · 초록 퍽, 파란 기둥과 다름)
            if sat_mean > cfg["sat_max"]:
                continue
            # 평균이 낮아도 창 일부에 색 물체(퍽)가 들어오면 안 됨: 라벨 안쪽(가장자리 2 화소 뺌) 채도 상위 10 %
            m_core = cv2.erode(m_lab, np.ones((5, 5), np.uint8)) > 0
            sat_hi = float(np.percentile(Rs[m_core], 90)) if m_core.sum() > 10 else 1e9
            if sat_hi > cfg["sat_hi_max"]:
                continue
            # 막대 띠는 검은 막대와 흰 빈칸이 섞여 회색: 태그처럼 새까만 곳은 아니다
            if band_mean < cfg["bw_min"] * white_mean:
                continue
            # 흰 스티커는 같은 조명을 받는 둘레 면보다 어둡지 않다: 물체 옆 그림자는 '흰 테두리' 자리까지 둘레보다 어둡다
            if white_mean < cfg["white_ring_min"] * ring_mean:
                continue
            # 막대 띠는 흰 테두리보다 뚜렷이 어둡다(검은 막대): 흐린 그림자 · 얼룩은 명암 차가 작다
            if band_mean > cfg["bw_max"] * white_mean:
                continue
            # 둘레가 고른가: 둘레 표준편차를 라벨 안 명암(흰 테두리 − 막대 띠)에 견줌. 태그 가장자리 · 면 경계는 둘레가 얼룩짐
            # 네 변(위 · 아래 · 왼 · 오른, 모서리 제외)으로 나눠, 가장 얼룩진 한 변은 빼고 본다:
            # 라벨 옆에 기둥 · 퍽이 붙어 있어도 한 변만 얼룩짐. 태그 가장자리 등은 여러 변이 얼룩짐
            yy, xx = np.nonzero(rv)
            la = (xx - sx) * a[0] + (yy - ty) * a[1]; lb = (xx - sx) * b[0] + (yy - ty) * b[1]
            hw, hh = lw / 2, lh / 2
            sides = [(lb < -hh) & (np.abs(la) <= hw), (lb > hh) & (np.abs(la) <= hw),
                     (la < -hw) & (np.abs(lb) <= hh), (la > hw) & (np.abs(lb) <= hh)]
            vals = Rg[yy, xx]
            ok_sides = [m for m in sides if m.sum() >= 6]
            if cfg["side_min_frac"] > 0:
                # 거의 다 평면 밖인 변(남은 몇 화소는 평면 경계 — 잡음에 따라 옆 면 화소가 들고 남)은 빼고 본다
                ya, xa = np.nonzero(ring)
                la_a = (xa - sx) * a[0] + (ya - ty) * a[1]; lb_a = (xa - sx) * b[0] + (ya - ty) * b[1]
                full = [((lb_a < -hh) & (np.abs(la_a) <= hw)).sum(), ((lb_a > hh) & (np.abs(la_a) <= hw)).sum(),
                        ((la_a < -hw) & (np.abs(lb_a) <= hh)).sum(), ((la_a > hw) & (np.abs(lb_a) <= hh)).sum()]
                ok_sides = [m for m, f in zip(sides, full) if m.sum() >= 6 and m.sum() >= cfg["side_min_frac"] * f]
            if cfg["ring_mad"]:   # 중앙값 기반 퍼짐: 이웃 물체가 둘레 일부(절반 미만)를 차지해도 흔들리지 않음
                ring_std = 1.4826 * float(np.median(np.abs(vals - np.median(vals))))
            elif cfg["ring_drop_worst"] and len(ok_sides) >= 3:
                worst = max(range(len(ok_sides)), key=lambda i: vals[ok_sides[i]].std())
                keep = np.zeros(len(vals), bool)
                for i, m in enumerate(ok_sides):
                    if i != worst:
                        keep |= m
                ring_std = float(vals[keep].std())
                if cfg["ring_lin"]:   # 무늬 없는 면도 조명 때문에 밝기가 완만히 기움 → 기울기(1차 평면)를 빼고 남은 퍼짐
                    A = np.stack([np.ones(keep.sum()), la[keep], lb[keep]], 1)
                    cf, *_ = np.linalg.lstsq(A, vals[keep], rcond=None)
                    ring_std = float((vals[keep] - A @ cf).std())
            elif cfg["side_min_frac"] > 0 and ok_sides:   # 남은 변이 한두 개면 그 변들만으로(뺀 변의 경계 화소는 넣지 않음)
                keep = np.any(ok_sides, axis=0)
                ring_std = float(vals[keep].std())
            ring_c = ring_std / max(white_mean - band_mean, 1.0)
            if ring_c > cfg["ring_max"]:
                continue
            # 둘레는 무늬 없는 면이라 조명 밝기에 견줘도 고르다(태그 안쪽에 걸린 후보는 둘레가 태그 무늬라 얼룩짐)
            if ring_std > cfg["ring_rel_max"] * max(white_mean, 1.0):
                continue
            # 흰 테두리 자리에 템플릿 예상보다 아주 어두운 화소가 있으면 흰 스티커가 아님(태그의 검은 칸)
            if feat["resw_lo"] < cfg["resw_min"]:
                continue
            # 가장자리가 카메라 흐림보다 더 흐리면 인쇄물이 아님(그림자 · 음영): 두 배 흐린 템플릿이 더 잘 맞음
            if feat["sharp"] < cfg["sharp_min"]:
                continue
            # 흰 테두리 한 변 전체가 예상보다 어두우면 흰 스티커가 아님(물체 옆 접촉 그림자 · 음영 기울기)
            if feat["side_lo"] < cfg["side_min"]:
                continue
            # 라벨 창이 영상 안에 다 들어와야 함(영상 끝에 잘린 물체를 라벨로 보지 않게)
            cs = [c0 + (s0 + mm * q[0]) * e1 + (t0 + mm * q[1]) * e2 for q in rect(0)]
            if any(not (cfg["img_margin"] <= proj(X_)[0] < W - cfg["img_margin"] and cfg["img_margin"] <= proj(X_)[1] < H - cfg["img_margin"])
                   for X_ in cs):
                continue
            if ring_valid < cfg["ring_min_valid"]:
                continue
            Xc = c0 + (s0 + mm * sx) * e1 + (t0 + mm * ty) * e2
            out.append(dict(kind=pl["kind"], center=Xc, score=score, theta=float(bth[ty, sx]),
                            ring_ratio=float(ring_ratio), ring_c=float(ring_c), sat=sat_mean, ring_valid=float(ring_valid), ring_std=ring_std, lab_std=lab_std,
                            ring_mean=ring_mean, white_mean=white_mean, band_mean=band_mean, sat_hi=sat_hi, lift=lift, pl_w=pl_w, **feat,
                            crop=(Rg[max(ty - 20, 0):ty + 21, max(sx - 20, 0):sx + 21].copy(),
                                  (ring & (valid_er > 0))[max(ty - 20, 0):ty + 21, max(sx - 20, 0):sx + 21].copy(),
                                  m_lab[max(ty - 20, 0):ty + 21, max(sx - 20, 0):sx + 21].copy()) if cfg["keep"] else None,
                            px=(float(K["fx"] * Xc[0] / Xc[2] + K["cx"]), float(K["fy"] * Xc[1] / Xc[2] + K["cy"]))))
    out.sort(key=lambda d: -d["score"])
    kept = []
    for d in out:
        if all(np.linalg.norm(d["center"] - o["center"]) >= cfg["nms"] for o in kept):
            kept.append(d)
    return kept
