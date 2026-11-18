"""장소 상자 윗면 찾기 시제품 (팔 Arducam 한 장, 깊이 없음). 도구 확인 · 설계용.

1) 윗면 높이 평면 (base_footprint z = 0.07) 을 위에서 본 정사영 영상으로 펼침 (호모그래피, 1 mm / 화소). 윗면은 실제 크기 205 mm 정사각형.
2) 크기를 아는 정사각형 테두리를 (중심 x · y, 요) 로 찾음: 변마다 안쪽 · 바깥 띠 Lab 차이의 중앙값 (안팎 대비) 을 더하고,
   네 변 안쪽 색이 서로 다르면 깎음 (윗면은 한 색, 퍽 모서리가 우연히 늘어선 자리는 변마다 안쪽 색이 다름).
   놓인 퍽 · 차체에 가린 곳은 변 중앙값이 무시함. 거친 격자 (요 1° · 4 mm) -> 고운 격자 (0.25° · 1 mm -> 0.05° · 0.25 mm).
3) 요는 정사각형이라 90° 마다 같음 -> 사전 요 (로봇 기준 0) ±45° 안에서 고름.
"""
import json, math, sys
import numpy as np
import cv2

BOX, BOX_H = 0.205, 0.07
RES = 0.001  # [m / 화소]
SPAN = 0.44  # 정사영 영상 한 변 [m]


def plane_homography(K, T_bf_cam, h):
    """평면 z = h 의 (X, Y) [m, base_footprint] -> 영상 화소 호모그래피."""
    R, e = T_bf_cam[:3, :3], T_bf_cam[:3, 3]
    Rt = R.T
    M = np.column_stack((Rt[:, 0], Rt[:, 1], Rt[:, 2] * h - Rt @ e))
    return K @ M


def top_view(img, K, T_bf_cam, center, h=BOX_H):
    """center (x, y) 를 가운데로 SPAN x SPAN m 정사영 영상. 화소 (u, v) -> (X, Y) = (center + (SPAN/2 − v·RES, SPAN/2 − u·RES)) : 위쪽 = +x (앞), 왼쪽 = +y."""
    n = int(round(SPAN / RES))
    S = np.array([[0, -RES, center[0] + SPAN / 2], [-RES, 0, center[1] + SPAN / 2], [0, 0, 1]])   # (u, v, 1) -> (X, Y, 1)
    Hm = plane_homography(K, T_bf_cam, h) @ S
    tv = cv2.warpPerspective(img, Hm, (n, n), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    valid = cv2.warpPerspective(np.full(img.shape[:2], 255, np.uint8), Hm, (n, n), flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP, borderValue=0)
    return tv, valid, S


def gradients(tv):
    lab = cv2.cvtColor(tv, cv2.COLOR_BGR2LAB).astype(np.float32)
    lab = cv2.GaussianBlur(lab, (0, 0), 1.5)
    gx = sum(cv2.Sobel(lab[:, :, c], cv2.CV_32F, 1, 0, ksize=3) for c in range(3))
    gy = sum(cv2.Sobel(lab[:, :, c], cv2.CV_32F, 0, 1, ksize=3) for c in range(3))
    # 채널 부호가 서로 지우지 않게 채널별 크기로도
    mag = np.sqrt(sum(cv2.Sobel(lab[:, :, c], cv2.CV_32F, 1, 0, ksize=3) ** 2 + cv2.Sobel(lab[:, :, c], cv2.CV_32F, 0, 1, ksize=3) ** 2 for c in range(3)))
    return gx, gy, mag


def perimeter(cu, cv_, th, side_px, k=60):
    """정사영 영상 화소 좌표의 정사각형 (중심 cu, cv, 각 th, 한 변 side_px) 변 4 개 위 점 [4, k, 2] 와 변 바깥 법선 [4, 2]."""
    t = np.linspace(-0.45, 0.45, k) * side_px  # 모서리 근처 (퍽 · 둥근 모서리) 는 뺌
    c, s = math.cos(th), math.sin(th)
    ax, ay = np.array([c, s]), np.array([-s, c])
    pts, nrm = [], []
    for sign, a, b in ((1, ax, ay), (-1, ax, ay), (1, ay, ax), (-1, ay, ax)):
        mid = np.array([cu, cv_]) + sign * a * side_px / 2
        pts.append(mid[None, :] + t[:, None] * b[None, :]); nrm.append(sign * a)
    return np.array(pts), np.array(nrm)


# 퍽 색 (perception.yaml 검출기 HSV 범위와 같음): 빨강 H 0~10 · 170~179, 초록 40~85, S ≥ 80, V ≥ 35 (OpenCV 0~179 · 0~255)
PUCK_HSV = [((0, 80, 35), (10, 255, 255)), ((170, 80, 35), (179, 255, 255)), ((40, 80, 35), (85, 255, 255))]


def puck_mask(tv, grow=3):
    """퍽 색 화소 (조금 부풀림). 이 화소는 테두리 표본에서 뺀다 (놓인 퍽이 원근으로 기울어 테두리를 가림)."""
    hsv = cv2.cvtColor(tv, cv2.COLOR_BGR2HSV)
    m = sum(cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in PUCK_HSV)
    return cv2.dilate((m > 0).astype(np.uint8), np.ones((2 * grow + 1, 2 * grow + 1), np.uint8)) > 0


def lab_image(tv):
    return cv2.GaussianBlur(cv2.cvtColor(tv, cv2.COLOR_BGR2LAB).astype(np.float32), (0, 0), 1.5)


def side_scores(lab, valid, C, th, side_px, k, d=4.0, lam=2.0, pm=None, lam_in=0.0, beta=2.0):
    """중심 C [..., 2] 마다 점수. 변 4 개 위 k 점에서 안쪽 · 바깥쪽 d 화소 Lab 를 뽑아
    점수 = Σ변 |중앙값(안 − 밖)| − lam · Σ변 |변 안쪽 중앙값 − 네 변 안쪽 중앙값의 중앙값| (윗면은 한 색, 바깥은 바닥 · 차체 등 무엇이든).
    화면 밖 (valid 0) 점은 크게 깎음."""
    off, nrm = perimeter(0.0, 0.0, th, side_px, k)                  # [4, k, 2], [4, 2]
    hh, ww = valid.shape
    P = C[..., None, None, :] + off                                  # [..., 4, k, 2]
    Pin, Pout = P - nrm[:, None, :] * d, P + nrm[:, None, :] * d
    def take(Q):
        u = np.clip(np.rint(Q[..., 0]), 0, ww - 1).astype(int); v = np.clip(np.rint(Q[..., 1]), 0, hh - 1).astype(int)
        return lab[v, u], valid[v, u] > 0
    Li, vi = take(Pin); Lo, vo = take(Pout)
    ok = vi & vo
    if pm is not None:   # 퍽 색 화소 뺌 (안 · 밖 어느 쪽이든)
        ui = np.clip(np.rint(Pin[..., 0]), 0, ww - 1).astype(int); vi_ = np.clip(np.rint(Pin[..., 1]), 0, hh - 1).astype(int)
        uo = np.clip(np.rint(Pout[..., 0]), 0, ww - 1).astype(int); vo_ = np.clip(np.rint(Pout[..., 1]), 0, hh - 1).astype(int)
        ok = ok & ~pm[vi_, ui] & ~pm[vo_, uo]
    Li = np.where(ok[..., None], Li, np.nan); Lo = np.where(ok[..., None], Lo, np.nan)
    with np.errstate(all="ignore"):
        import warnings; warnings.simplefilter("ignore", RuntimeWarning)
        med_in = np.nanmedian(Li, axis=-2)                               # [..., 4, 3]
        contrast = np.linalg.norm(np.nanmedian(Li - Lo, axis=-2), axis=-1)  # [..., 4]
        ref = np.nanmedian(med_in, axis=-2, keepdims=True)
        spread = np.linalg.norm(med_in - ref, axis=-1)                   # [..., 4]
        # 안쪽 고르기: 사각형 안 7 x 7 점 (퍽 색 뺌) 이 변 안쪽 색 (ref) 과 얼마나 다른지 중앙값. 밀린 사각형은 안에 바닥이 들어옴
        c, s_ = math.cos(th), math.sin(th); t = np.linspace(-0.4, 0.4, 7) * side_px
        G = np.array([[a * c - b * s_, a * s_ + b * c] for a in t for b in t])   # [49, 2]
        Q = C[..., None, :] + G
        uq = np.clip(np.rint(Q[..., 0]), 0, ww - 1).astype(int); vq = np.clip(np.rint(Q[..., 1]), 0, hh - 1).astype(int)
        Lq = lab[vq, uq]; okq = valid[vq, uq] > 0
        if pm is not None:
            okq = okq & ~pm[vq, uq]
        dq = np.linalg.norm(Lq - ref[..., 0, :][..., None, :], axis=-1)
        inner = np.nan_to_num(np.nanmedian(np.where(okq, dq, np.nan), axis=-1), nan=50.0)
    seen = ok.mean(axis=-1)                                              # [..., 4] 쓸 수 있는 표본 비율
    contrast = np.where(seen >= 0.15, contrast, 0.0); spread = np.where(seen >= 0.15, spread, 0.0)
    contrast = np.nan_to_num(contrast); spread = np.nan_to_num(spread)
    # 가장 약한 변의 대비도 더함: 네 변 모두 실제 경계여야 높음 (세 변만 맞는 밀린 사각형을 깎음)
    return (contrast - lam * spread).sum(-1) + beta * contrast.min(-1) - lam_in * 4 * inner - 50.0 * np.clip(0.3 - seen, 0, None).sum(-1)


def detect(img, K, T_bf_cam, prior_center, prior_yaw=0.0, search_xy=0.08):
    tv, valid, S = top_view(img, K, T_bf_cam, prior_center)
    lab = lab_image(tv); pm = puck_mask(tv)
    n = tv.shape[0]; side = BOX / RES; c0 = n / 2
    g = np.arange(-search_xy, search_xy + 1e-9, 0.004) / RES
    C = np.stack(np.meshgrid(c0 + g, c0 + g, indexing="ij"), axis=-1)  # [G, G, 2]
    best = (-1e9, None, 0.0)
    for th in np.radians(np.arange(-45, 45, 1.0)) + prior_yaw:
        sc = side_scores(lab, valid, C, th, side, 24, pm=pm)
        i = np.unravel_index(np.argmax(sc), sc.shape)
        if sc[i] > best[0]:
            best = (float(sc[i]), C[i].copy(), th)
    sc, bc, bt = best
    for step_xy, step_th, r in ((0.001, 0.25, 4), (0.0005, 0.1, 3), (0.00025, 0.05, 2)):
        gg = np.arange(-r, r + 1) * step_xy / RES
        Cf = bc + np.stack(np.meshgrid(gg, gg, indexing="ij"), axis=-1)
        for th in bt + np.radians(np.arange(-r, r + 1) * step_th):
            s2 = side_scores(lab, valid, Cf, th, side, 60, pm=pm)
            i = np.unravel_index(np.argmax(s2), s2.shape)
            if s2[i] > sc:
                sc, nbc, nbt = float(s2[i]), Cf[i].copy(), th
        if 'nbc' in locals():
            bc, bt = nbc, nbt
    # 다듬기: 테두리 위 밝기 기울기 크기 (변 중앙값) 로 ±3 mm · ±2° 안에서 (자리 잡기 점수보다 테두리에 날카로움)
    _, _, mag = gradients(tv); mv = np.where((valid > 0) & ~pm, mag, np.nan)
    def gscore(Cs, th, k=80):
        off, _ = perimeter(0.0, 0.0, th, side, k)
        P = Cs[..., None, None, :] + off
        u = np.clip(np.rint(P[..., 0]), 0, n - 1).astype(int); v = np.clip(np.rint(P[..., 1]), 0, n - 1).astype(int)
        with np.errstate(all="ignore"):
            import warnings; warnings.simplefilter("ignore", RuntimeWarning)
            return np.nan_to_num(np.nanmedian(mv[v, u], axis=-1)).mean(axis=-1)
    gs = -1e9
    for step_xy, step_th, r in ((0.001, 0.25, 3), (0.0005, 0.1, 3), (0.00025, 0.05, 2)):
        gg = np.arange(-r, r + 1) * step_xy / RES
        Cf = bc + np.stack(np.meshgrid(gg, gg, indexing="ij"), axis=-1)
        cand = None
        for th in bt + np.radians(np.arange(-r, r + 1) * step_th):
            s2 = gscore(Cf, th); i = np.unravel_index(np.argmax(s2), s2.shape)
            if s2[i] > gs:
                gs, cand = float(s2[i]), (Cf[i].copy(), th)
        if cand:
            bc, bt = cand
    # 마지막: 변마다 경계점 (법선 ±5 화소 안 기울기 최대, 포물선으로 화소 이하) -> 크기 아는 정사각형에 가중 최소제곱 (Huber) 3 번
    for _ in range(3):
        off, nrm = perimeter(0.0, 0.0, bt, side, 80)
        tang = np.array([[-nn[1], nn[0]] for nn in nrm])
        rows, rhs, wts = [], [], []
        for si in range(4):
            for kk in range(off.shape[1]):
                p0 = bc + off[si, kk]; ts = np.dot(off[si, kk], tang[si])
                prof = []
                for dlt in range(-5, 6):
                    q = p0 + nrm[si] * dlt; u, v = int(round(q[0])), int(round(q[1]))
                    prof.append(mv[v, u] if 0 <= u < n and 0 <= v < n else np.nan)
                prof = np.array(prof)
                if np.isnan(prof).any() or np.nanmax(prof) < 5:
                    continue
                j = int(np.argmax(prof))
                if 0 < j < 10:
                    a_, b_, c_ = prof[j - 1], prof[j], prof[j + 1]; den = a_ - 2 * b_ + c_
                    dd = (j - 5) + (0.5 * (a_ - c_) / den if den != 0 else 0.0)
                    rows.append([nrm[si][0], nrm[si][1], -ts]); rhs.append(dd)   # 정사각형을 dθ 돌리면 변 위 점의 법선 성분은 −t·dθ
        if len(rows) < 40:
            break
        A, y = np.array(rows), np.array(rhs); w = np.ones(len(y))
        for _ in range(5):   # Huber 1 화소
            sol = np.linalg.lstsq(A * w[:, None], y * w, rcond=None)[0]
            res = np.abs(y - A @ sol); w = np.where(res <= 1.0, 1.0, 1.0 / np.maximum(res, 1e-6)) ** 0.5
        if np.abs(sol[:2]).max() > 3 or abs(sol[2]) > math.radians(2):
            break
        bc = bc + sol[:2]; bt = bt + sol[2]
    X, Y, _ = S @ np.array([bc[0], bc[1], 1.0])
    ax_phys = np.array([-math.sin(bt), -math.cos(bt)])   # 영상 (u, v) 방향 (cos, sin) -> 물리 (X, Y) = (−sin, −cos) (u = −Y, v = −X)
    yaw = math.atan2(ax_phys[1], ax_phys[0])
    yaw = (yaw - prior_yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4 + prior_yaw
    return {"center": [float(X), float(Y)], "yaw": float(yaw), "score": sc}


def cells(center, yaw, pitch=0.06):
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return np.array([np.asarray(center) + R @ np.array([dx, dy]) * pitch for dx in (-1, 0, 1) for dy in (-1, 0, 1)])


if __name__ == "__main__":
    d = sys.argv[1]
    meta = json.load(open(f"{d}/render_place_box.json"))
    K = np.array([[meta["K"]["fx"], 0, meta["K"]["cx"]], [0, meta["K"]["fy"], meta["K"]["cy"]], [0, 0, 1]])
    T = np.array(meta["T_bf_cam"]); out = []
    for sc in meta["scenes"]:
        img = cv2.imread(f"{d}/{sc['image']}")
        r = detect(img, K, T, (0.2553, 0.0))
        dc = 1000 * math.hypot(r["center"][0] - sc["center"][0], r["center"][1] - sc["center"][1])
        dyaw = math.degrees((r["yaw"] - sc["yaw"] + math.pi / 4) % (math.pi / 2) - math.pi / 4)
        de = 1000 * np.linalg.norm(cells(r["center"], r["yaw"]) - cells(sc["center"], sc["yaw"]), axis=1).max()
        out.append((dc, abs(dyaw), de)); print(f"{sc['image']}: 중심 {dc:.1f} mm 요 {dyaw:+.2f}° 칸 최대 {de:.1f} mm 점수 {r['score']:.0f}", flush=True)
    a = np.array(out)
    for i, k in enumerate(("중심 mm", "요 °", "칸 최대 mm")):
        print(f"{k}: 중앙 {np.median(a[:, i]):.2f} p95 {np.percentile(a[:, i], 95):.2f} 최대 {a[:, i].max():.2f}")
