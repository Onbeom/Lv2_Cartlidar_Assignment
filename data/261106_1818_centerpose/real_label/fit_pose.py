#!/usr/bin/env python3
"""한 퍽의 3D 자세를 태그 코너(있으면) + 색 윤곽 + 고친 점(있으면)으로 맞춘다. 입력은 모두 2D, 내부값 K · D 로 3D 를 계산한다."""
import cv2, numpy as np
import puck3d as P

def color_hull(img, box, cls, tag_quads=()):
    """상자 안 퍽 색 가장 큰 덩어리 + 태그 → 볼록 껍질(N×2). 없으면 None."""
    H, W = img.shape[:2]; x0, y0 = np.maximum(np.floor(box[:2]).astype(int) - 3, 0); x1, y1 = np.minimum(np.ceil(box[2:]).astype(int) + 4, (W, H))
    if x1 - x0 < 4 or y1 - y0 < 4: return None
    hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    if cls in (0, 2): m = cv2.inRange(hsv, (0, 60, 40), (12, 255, 255)) | cv2.inRange(hsv, (165, 60, 40), (180, 255, 255))
    else: m = cv2.inRange(hsv, (35, 50, 35), (92, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2: pts = np.zeros((0, 2))
    else:
        k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        if st[k, cv2.CC_STAT_AREA] < 30: pts = np.zeros((0, 2))
        else:
            ys, xs = np.nonzero(lab == k); pts = np.c_[xs + x0, ys + y0].astype(np.float32)
    if len(tag_quads): pts = np.vstack([pts] + [np.asarray(q, np.float32) for q in tag_quads])
    if len(pts) < 8: return None
    return cv2.convexHull(pts.astype(np.float32)).reshape(-1, 2)

def sample_poly(poly, n=48):
    d = np.roll(poly, -1, 0) - poly; L = np.linalg.norm(d, axis=1); c = np.r_[0, np.cumsum(L)]
    if c[-1] <= 0: return np.repeat(poly[:1], n, 0)
    s = (np.arange(n) + .5) * c[-1] / n; i = np.clip(np.searchsorted(c, s, side='right') - 1, 0, len(poly) - 1)
    return poly[i] + d[i] * ((s - c[i]) / np.maximum(L[i], 1e-9))[:, None]

# 태그 자리 어긋남 한계(m): 11-09 실측 퍽 8개의 틈에서. 끝면 틈 5.4~8.5 → ±1.6, 옆면 길이 방향 틈 19.5~22.5 → ±2.5, 옆면 폭 방향 틈 6.5~7.5 → ±0.5
LIM_END, LIM_LONG, LIM_WIDE, W_OFF = .0016, .0025, .0005, 3.0   # 사전 σ = 한계 / 2, 1σ 가 태그 1 px 와 같은 무게
def off_limits(T):
    """태그 몸체 코너 T 의 면 안 두 방향(태그 x · y) 한계."""
    e1 = T[1] - T[0]; e2 = T[0] - T[3]; n = np.cross(e1, e2); n /= np.linalg.norm(n)
    if abs(n[2]) > .9: return np.array([LIM_END, LIM_END])
    return np.array([LIM_LONG if abs(e[2]) / np.linalg.norm(e) > .9 else LIM_WIDE for e in (e1, e2)])

_FN = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
def visible_corners(R, c):
    """직육면체 코너 8개 중 카메라 쪽을 보는 면에 붙은 것(bool 8)."""
    X = P.CORNERS @ R.T + c; out = np.zeros(8, bool)
    for n in _FN:
        k = (P.CORNERS @ n) > 0; h = (P.HL if n[2] else P.HW)
        if (R @ n) @ (c + R @ (n * h)) < 0: out |= k
    return out

class Obs:
    """맞출 때 쓰는 2D 관측. tags = [(코너 4×2, 몸체 좌표 코너 4×3)], pins = [(몸체 좌표 점, 2D)]."""
    def __init__(self, cls, K, D, hull=None, W=1280, H=720):
        self.cls, self.K, self.D, self.hull, self.W, self.H = cls, K, D, hull, W, H; self.tags = []; self.pins = []; self.model = P.model_pts(cls)
        self.dist = None
        self.free_off = False     # True 면 태그마다 면 안 자리 어긋남(2개)을 함께 푼다(실물: 태그를 손으로 붙임)
        if hull is not None:
            e = np.full((H, W), 255, np.uint8); cv2.polylines(e, [hull.astype(np.int32)], True, 0, 1)
            self.dist = cv2.distanceTransform(e, cv2.DIST_L2, 3)
            s = sample_poly(hull.astype(np.float64)); keep = (s[:, 0] > 2) & (s[:, 0] < W - 3) & (s[:, 1] > 2) & (s[:, 1] < H - 3)
            self.hs = s[keep]
    def proj(self, X, R, c):
        return P.project(X, R, c, self.K, self.D)
    def tag_shift(self, T, d):
        """태그 몸체 코너 T(4×3)를 면 안 두 방향(태그 x · y)으로 d(2, m) 만큼 옮긴다."""
        e1 = T[1] - T[0]; e2 = T[0] - T[3]
        return T + d[0] * e1 / np.linalg.norm(e1) + d[1] * e2 / np.linalg.norm(e2)
    def resid(self, x, w_tag=3.0, w_pin=10.0):
        R = cv2.Rodrigues(x[:3])[0]; c = x[3:6]; r = []
        for i, (q, T) in enumerate(self.tags):
            if self.free_off: T = self.tag_shift(T, x[6 + 2 * i: 8 + 2 * i])
            r.append(w_tag * (self.proj(T, R, c) - q).ravel())
        for X, u in self.pins: r.append(w_pin * (self.proj(X[None], R, c).ravel() - u))
        if self.dist is not None:
            m = self.proj(self.model, R, c)
            if np.isfinite(m).all():
                poly = cv2.convexHull(m.astype(np.float32)).reshape(-1, 2).astype(np.float64); s = sample_poly(poly)
                ix = np.round(s).astype(int); ok = (ix[:, 0] >= 0) & (ix[:, 0] < self.W) & (ix[:, 1] >= 0) & (ix[:, 1] < self.H)
                v = np.zeros(len(s)); v[ok] = self.dist[ix[ok, 1], ix[ok, 0]]; r.append(np.minimum(v, 40.))
                pf = poly.astype(np.float32)
                r.append(np.minimum(np.array([abs(cv2.pointPolygonTest(pf, (float(a), float(b)), True)) for a, b in self.hs]), 40.) if len(self.hs) else np.zeros(0))
            else: r.append(np.full(48 + len(self.hs), 40.))
        if self.free_off and self.tags: r.append(W_OFF * x[6:] / (np.concatenate([off_limits(T) for _, T in self.tags]) / 2))
        return np.concatenate(r) if r else np.zeros(0)
    def fit(self, R0, c0, it=25, huber=2.0):
        """가우스-뉴턴 + 감쇠(huber 가중, 수치 야코비안)."""
        no = 2 * len(self.tags) if self.free_off else 0
        self.lim = np.concatenate([off_limits(T) for _, T in self.tags]) if no else np.zeros(0)
        x = np.r_[cv2.Rodrigues(R0)[0].ravel(), c0, np.zeros(no)]; lam = 1e-2; step = np.r_[np.full(3, 2e-3), np.full(3, 2e-4), np.full(no, 1e-4)]; n = len(x)
        i0 = 8 * len(self.tags); i1 = i0 + 2 * len(self.pins)      # 고친 점(pins)은 줄이지 않고 그대로 따른다
        def cost(rr):
            a = np.abs(rr); c_ = np.where(a <= huber, .5 * a * a, huber * (a - .5 * huber)); c_[i0:i1] = .5 * a[i0:i1] ** 2; return float(c_.sum())
        r = self.resid(x); f = cost(r)
        for _ in range(it):
            J = np.zeros((len(r), n))
            for j in range(n):
                y = x.copy(); y[j] += step[j]; J[:, j] = (self.resid(y) - r) / step[j]
            w = np.minimum(1, huber / np.maximum(np.abs(r), 1e-9)); w[i0:i1] = 1; A = (J * w[:, None]).T @ J; g = (J * w[:, None]).T @ r
            for _t in range(6):
                try: d = np.linalg.solve(A + lam * np.diag(np.maximum(np.diag(A), 1e-6)), -g)
                except np.linalg.LinAlgError: lam *= 10; continue
                y = x + d; y[6:] = np.clip(y[6:], -self.lim, self.lim)
                if y[5] <= .02: lam *= 5; continue
                r2 = self.resid(y); f2 = cost(r2)
                if f2 < f: x, r, ok = y, r2, f - f2; f = f2; lam = max(lam / 3, 1e-5); break
                lam *= 5
            else: break
            if ok < 1e-3: break
        if no:      # 찾은 어긋남을 태그 몸체 코너에 넣어 둔다(점수 · 저장이 같은 자리를 쓰게)
            self.tags = [(q, self.tag_shift(T, x[6 + 2 * i: 8 + 2 * i])) for i, (q, T) in enumerate(self.tags)]; self.off = x[6:].reshape(-1, 2).copy()
        return cv2.Rodrigues(x[:3])[0], x[3:6], f
    def scores(self, R, c):
        m = self.proj(self.model, R, c); out = {}
        if self.hull is not None:
            a = np.zeros((self.H, self.W), np.uint8); b = a.copy()
            cv2.fillConvexPoly(a, cv2.convexHull(np.clip(m, -3000, 4000).astype(np.int32)), 1); cv2.fillConvexPoly(b, self.hull.astype(np.int32), 1)
            out['sil_iou'] = float((a & b).sum() / max(1, (a | b).sum()))
        if self.tags:
            e = np.concatenate([np.linalg.norm(self.proj(T, R, c) - q, axis=1) for q, T in self.tags]); out['tag_px'] = float(np.sqrt((e ** 2).mean()))
        return out

FACE_R = []
for n_, u_ in [((0, 0, 1), (1, 0, 0)), ((0, 0, -1), (1, 0, 0)), ((1, 0, 0), (0, 1, 0)), ((-1, 0, 0), (0, 1, 0)), ((0, 1, 0), (0, 0, 1)), ((0, -1, 0), (0, 0, 1))]:
    n_, u_ = np.array(n_, float), np.array(u_, float); v_ = np.cross(n_, u_)
    for k in range(4):
        a = k * np.pi / 2; U = np.cos(a) * u_ + np.sin(a) * v_; FACE_R.append((np.c_[U, np.cross(n_, U), n_], n_ * (P.HL if abs(n_[2]) else P.HW)))
def tag_on_body(Rf, tf): return P.OBJ_TAG @ Rf.T + tf

def assign_tags(ob, R, c, quads, first):
    """first 는 (quad, 몸체 코너) 로 정해졌고, 다른 태그는 면 후보 중 가장 잘 맞는 자리로 붙인다."""
    ob.tags = [first]
    faces = FACE_R[:8] if P.is_cyl(ob.cls) else FACE_R
    for q in quads:
        if q is first[0]: continue
        best = (1e9, None)
        for Rf, tf in faces:
            if tf @ tf > 1e-9:
                T = tag_on_body(Rf, tf); e = np.sqrt(((ob.proj(T, R, c) - q) ** 2).sum(1).mean())
                if e < best[0]: best = (e, T)
        if best[0] < 25: ob.tags.append((q, best[1]))

def assign_tags_all(ob, R, c, quads):
    """모든 태그를 지금 자세에서 가장 가까운 면 자리에 붙인다(사람이 고칠 때)."""
    ob.tags = []
    faces = FACE_R[:8] if P.is_cyl(ob.cls) else FACE_R
    for q in quads:
        e = [(np.sqrt(((ob.proj(tag_on_body(Rf, tf), R, c) - q) ** 2).sum(1).mean()), i) for i, (Rf, tf) in enumerate(faces)]
        _, i = min(e); Rf, tf = faces[i]
        if P.is_cyl(ob.cls):       # 원기둥은 태그 방향이 자유: 끝면에 붙은 채 지금 방향을 유지
            Rt, t, _ = min(P.tag_solutions(q, ob.K, ob.D), key=lambda s: np.linalg.norm(s[1] - (c + R @ tf))) ; T = (P.OBJ_TAG @ Rt.T + t - c) @ R
            T[:, 2] = tf[2]; T[:, :2] -= T[:, :2].mean(0)
        else: T = tag_on_body(Rf, tf)
        ob.tags.append((q, T))

def fit_puck(img, box, cls, quads, K, D=None, pins=(), W=1280, H=720, init=None, free_off=False):
    """→ dict(R, c, sil_iou, tag_px, box_iou, n_tags, hyp, tag_off) 또는 None. pins = [(몸체 점, 2D)]. free_off: 태그 자리 어긋남도 푼다."""
    quads = [np.asarray(q, np.float64) for q in quads]; hull = color_hull(img, box, cls, quads)
    ob = Obs(cls, K, D, hull, W, H); ob.free_off = free_off; best = None
    def consider(R0, c0, first, hyp):
        nonlocal best
        if first is not None: assign_tags(ob, R0, c0, quads, first)
        else: ob.tags = []
        ob.pins = list(pins); ob.off = np.zeros((0, 2))
        R, c, f = ob.fit(R0, c0)
        s = ob.scores(R, c); bi = P.iou(P.bbox(ob.proj(ob.model, R, c), W, H), box)
        if c[2] - P.HL <= .005: return
        q = s.get('sil_iou', 0) + .4 * bi - .02 * min(s.get('tag_px', 0), 20)
        if best is None or q > best['_q']: best = dict(_q=q, R=R, c=c, box_iou=bi, n_tags=len(ob.tags), hyp=hyp, tags=[(a.copy(), b.copy()) for a, b in ob.tags], tag_off=ob.off.tolist(), **s)
    if init is not None:
        for i, q in enumerate(quads[:1] or [None]):
            ob.tags = []; consider(init[0], init[1], (q, init[2]) if q is not None and init[2] is not None else None, 'init')
    elif quads:
        for q in quads:
            for R, t, e in P.tag_solutions(q, K, D):
                if t[2] <= 0: continue
                for name, Rp, c in P.hypotheses(R, t, cls):
                    T = (P.OBJ_TAG @ R.T + t - c) @ Rp   # 태그 코너의 몸체 좌표
                    consider(Rp, c, (q, T), name)
    else:
        f = K[0, 0]; s = max(box[2] - box[0], box[3] - box[1], 8.); cx, cy = (box[:2] + box[2:]) / 2
        cand = []
        for sc in (.6, .8, 1.0, 1.25):
            z = f * .06 / s * sc; c0 = np.array([(cx - K[0, 2]) / f * z, (cy - K[1, 2]) / f * z, z])
            for th in np.radians(np.arange(0, 360, 30)):
                for el in np.radians((0, 20, 40, 60, 80)):
                    ax = np.array([np.cos(th) * np.cos(el), np.sin(th) * np.cos(el), np.sin(el)])
                    v = np.cross(ax, [0.3, .5, .8]); v /= np.linalg.norm(v)
                    for yaw in ((0,) if P.is_cyl(cls) else (0, 22.5, 45, 67.5)):
                        a = np.radians(yaw); X = np.cos(a) * v + np.sin(a) * np.cross(ax, v); R0 = np.c_[X, np.cross(ax, X), ax]
                        ob.tags = []; ob.pins = []; cand.append((ob.scores(R0, c0).get('sil_iou', 0), len(cand), R0, c0))
        cand.sort(key=lambda t: -t[0])
        for _, _, R0, c0 in cand[:24]: consider(R0, c0, None, 'silhouette')
    if best is None: return None
    best.pop('_q'); return best

def fit_clicks(ob, clicks, quads, extra_pins=(), top=6, n_search=4):
    """사람이 순서 없이 찍은 직육면체 코너 → (R, c, {코너 번호: 2D}).
    앞 n_search 개 점을 코너 8개에 대응시키는 모든 경우를 PnP 로 풀고(보이는 코너여야 함), 윤곽까지 보아 좋은 것 몇 개를
    고정점으로 다시 맞춘다. 나머지 점은 그 자세에서 가장 가까운 보이는 코너에 붙인다."""
    import itertools
    U = np.asarray(clicks, np.float64); S = U[:n_search]; n = len(S); K, D = ob.K, ob.D; cand = []
    for perm in itertools.permutations(range(8), n):
        Ob = P.CORNERS[list(perm)]
        if n == 3:
            k, rs, ts = cv2.solveP3P(Ob, S, K, D, flags=cv2.SOLVEPNP_P3P); sols = list(zip(rs, ts)) if k else []
        else:
            ok, rv, tv = cv2.solvePnP(Ob, S, K, D, flags=cv2.SOLVEPNP_SQPNP); sols = [(rv, tv)] if ok else []
        for rv, tv in sols:
            R = cv2.Rodrigues(rv)[0]; c = tv.ravel()
            if c[2] < .03 or c[2] > 1.5: continue
            vis = visible_corners(R, c)
            if not vis[list(perm)].all(): continue
            e = np.sqrt(((P.project(Ob, R, c, K, D) - S) ** 2).sum(1).mean())
            if e > 6: continue
            cand.append((e, perm, R, c))
    if not cand: return None
    cand.sort(key=lambda x: x[0]); cand = cand[:200]
    sc = sorted(((x[0] / 3 - ob.scores(x[2], x[3]).get('sil_iou', 0), j) for j, x in enumerate(cand)))[:top]
    best = None
    for _, j in sc:
        e, perm, R, c = cand[j]
        pins = {int(i): u for i, u in zip(perm, S)}
        m = P.project(P.CORNERS, R, c, K, D); vis = np.where(visible_corners(R, c))[0]
        for u in U[n:]:
            free = [i for i in vis if i not in pins]
            if free: i = min(free, key=lambda i: np.sum((m[i] - u) ** 2)); pins[int(i)] = u
        ob.pins = list(extra_pins) + [(P.CORNERS[i], u) for i, u in pins.items()]
        if quads: assign_tags_all(ob, R, c, quads)
        else: ob.tags = []
        R2, c2, fv = ob.fit(R, c)
        s = ob.scores(R2, c2); q = s.get('sil_iou', 0) - .01 * fv ** .5
        if best is None or q > best[0]: best = (q, R2, c2, pins, list(ob.tags))
    ob.tags = best[4]; return best[1], best[2], best[3]
