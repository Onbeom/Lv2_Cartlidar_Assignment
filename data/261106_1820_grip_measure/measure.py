"""#128 손 안 퍽 윤곽 맞추기 (모의, 오프라인). 렌더 영상에서 집게 기준 퍽 자세를 푼다.

색 영역의 바깥 윤곽과 모형 윤곽(볼록 껍질)을 맞춘다. 미지수: dx · dy · dz [집게 링크 축, m], 기울기 th(y 둘레) · ep(x 둘레), 직육면체 회전 spin.
태그는 쓰지 않는다(화면 안에 다 들어오지 않음). 왜곡은 없다고 본다.
"""
import glob
import math
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

S = math.sqrt(0.5)
Z_P = np.array([-S, 0.0, S])
E1 = np.array([0.0, 1.0, 0.0])
E2 = np.cross(Z_P, E1)
R_NOM = np.column_stack([E1, E2, Z_P])
HALF_L, HALF_W = 0.03, 0.015
MESH = os.path.expanduser("~/git/Lv2_Cartlidar_Assignment_armcam/workspace/urdf_and_usd/waffle_4wd_description/meshes/arm/")
W, H = 1280, 720


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def model_points(shape):
    if shape == "cylinder":
        a = np.linspace(0, 2 * np.pi, 96, endpoint=False)
        ring = np.column_stack([HALF_W * np.cos(a), HALF_W * np.sin(a)])
    else:
        ring = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]], float) * HALF_W
    return np.vstack([np.column_stack([ring, np.full(len(ring), z)]) for z in (-HALF_L, HALF_L)])


def pose(p, nominal):
    dx, dy, dz, th, ep, spin = p
    R = rot("y", th) @ rot("x", ep) @ R_NOM @ rot("z", spin)
    return R, nominal + np.array([dx, dy, dz])


def project(P_G, K, T_G_cam):
    """집게 링크 점 → 화면. 광학 z 가 작으면 inf 로 둔다."""
    R, t = T_G_cam[:3, :3], T_G_cam[:3, 3]
    q = (P_G - t) @ R
    z = np.maximum(q[:, 2], 1e-3)
    return np.column_stack([K["fx"] * q[:, 0] / z + K["cx"], K["fy"] * q[:, 1] / z + K["cy"]])


def color_mask(bgr, color):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    if color == "red":
        m = ((h <= 12) | (h >= 168)) & (s > 90) & (v > 25)
    else:
        m = (h >= 35) & (h <= 90) & (s > 60) & (v > 20)
    return m.astype(np.uint8) * 255


def finger_polys(K, T_G_cam, q=-0.0058):
    import trimesh
    polys = []
    for name, y, sgn in (("gripper_left_link", 0.021, 1.0), ("gripper_right_link", -0.021, -1.0)):
        m = trimesh.util.concatenate([trimesh.load(f) for f in sorted(glob.glob(MESH + name + "_*.stl"))])
        v = np.asarray(m.vertices)
        v = v[::max(1, len(v) // 4000)] + np.array([0.0947, y + sgn * q, 0.0])
        uv = project(v, K, T_G_cam)
        polys.append(cv2.convexHull(uv.astype(np.float32)))
    return polys


def finger_mask(K, T_G_cam, dil=10):
    m = np.zeros((H, W), np.uint8)
    for pl in finger_polys(K, T_G_cam):
        cv2.fillConvexPoly(m, np.round(pl.reshape(-1, 2)).astype(np.int32).clip(-20000, 20000), 255)
    return cv2.dilate(m, np.ones((2 * dil + 1, 2 * dil + 1), np.uint8))


def bilinear(img, uv):
    u = np.clip(uv[:, 0], 0, img.shape[1] - 1.001)
    v = np.clip(uv[:, 1], 0, img.shape[0] - 1.001)
    x0, y0 = u.astype(int), v.astype(int)
    a, b = u - x0, v - y0
    return (img[y0, x0] * (1 - a) * (1 - b) + img[y0, x0 + 1] * a * (1 - b) + img[y0 + 1, x0] * (1 - a) * b + img[y0 + 1, x0 + 1] * a * b)


def hull_outline(uv, step=4.0):
    hull = cv2.convexHull(uv.astype(np.float32)).reshape(-1, 2)
    a, b = hull, np.roll(hull, -1, axis=0)
    out = []
    for p, q in zip(a, b):
        n = max(1, int(np.linalg.norm(q - p) / step))
        out.append(p + (q - p) * (np.arange(n) / n)[:, None])
    return hull, np.vstack(out)


def extract(bgr, color, shape, K, T_G_cam, nominal, fmask, roi_margin=0.012):
    """색 영역 → 윤곽 거리 지도와 바깥 윤곽 점. 모형이 ±roi_margin 움직인 범위 밖 색 영역은 이웃 퍽으로 보고 버린다."""
    mp = model_points(shape)
    roi = np.zeros((H, W), np.uint8)
    for d in [(a, 0, b) for a in (-roi_margin, roi_margin) for b in (-roi_margin, roi_margin)]:
        R, c = pose((*d, 0, 0, 0), nominal)
        pts = project(mp @ R.T + c, K, T_G_cam)
        cv2.fillConvexPoly(roi, cv2.convexHull(np.round(pts).astype(np.int32).clip(-20000, 20000)), 255)
    roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((201, 201), np.uint8))
    m = cv2.bitwise_and(color_mask(bgr, color), roi)
    # 손가락에 비친 가는 띠를 지운다 (7 × 7). 라벨 · 옆면 태그의 구멍과 홈은 볼록 껍질로 메운다(퍽 윤곽은 볼록)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = (lab == k).astype(np.uint8) * 255
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hull = cv2.convexHull(max(cs, key=cv2.contourArea))
    filled = np.zeros_like(m)
    cv2.fillConvexPoly(filled, hull, 255)
    solidity = (m > 0).sum() / max(1, (filled > 0).sum())
    edge_u8 = np.zeros_like(m)
    cv2.polylines(edge_u8, [hull], True, 255, 1)
    c = np.column_stack(np.nonzero(edge_u8)[::-1])
    roi_in = cv2.erode(roi, np.ones((5, 5), np.uint8))
    keep = (c[:, 1] < H - 4) & (fmask[c[:, 1], c[:, 0]] == 0) & (roi_in[c[:, 1], c[:, 0]] > 0)
    c = c[keep]
    edge = np.full_like(m, 255)
    edge[c[:, 1], c[:, 0]] = 0
    dt = cv2.distanceTransform(edge, cv2.DIST_L2, 5)
    # 영역 안쪽 가장자리: 앞 옆면(어두움)과 윗면(밝음) 경계. 윗면 앞 가장자리 모형과 맞춘다
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (0, 0), 1.0)
    inner = cv2.Canny(gray, 15, 40)
    inner[cv2.erode(filled, np.ones((9, 9), np.uint8)) == 0] = 0
    dt_in = cv2.distanceTransform(255 - inner, cv2.DIST_L2, 5)
    return {"dt": dt, "dt_in": dt_in, "contour": c.astype(np.float32), "area": int((filled > 0).sum()), "mask": filled, "solidity": float(solidity)}


def top_far_points(shape, R, c):
    """윗면 앞(집게 앞 수평 쪽) 가장자리 점 (집게 링크 좌표)."""
    z = R[:, 2]
    f = np.cross(E1, z)
    f = f / np.linalg.norm(f)
    if f @ np.array([S, 0.0, S]) < 0:
        f = -f
    y = np.cross(z, f)
    s = np.linspace(-0.012, 0.012, 25)
    off = np.sqrt(HALF_W ** 2 - s ** 2) if shape == "cylinder" else np.full_like(s, HALF_W)
    return c + HALF_L * z + np.outer(off, f) + np.outer(s, y)


def fit(ex, shape, K, T_G_cam, nominal, fmask, shift=(0.0, 0.0, 1.0, 0.0)):
    """shift = (du, dv, s, phi): 손가락 표시 보정이 있으면 모형 투영에 닮음 변환으로 더한다. 미지수 단위 mm · ° 로 풀고 m · rad 로 돌려준다."""
    mp = model_points(shape)
    cont = ex["contour"][:: max(1, len(ex["contour"]) // 200)]
    prior = np.array([0, 1 / 1.0, 0, 0, 1 / 3.0, 1 / 3.0]) * 3.0   # dy ±1 mm, ep · spin ±3° 약한 사전 (원기둥 spin 은 답에 영향 없음)
    SCALE = np.array([1e-3, 1e-3, 1e-3, math.radians(1), math.radians(1), math.radians(1)])

    def proj(x):
        R, c = pose(x * SCALE, nominal)
        return sim(project(mp @ R.T + c, K, T_G_cam))

    def proj_pts(P, x):
        return sim(project(P, K, T_G_cam))

    def sim(uv):
        du, dv, s, ph = shift
        a, b = uv[:, 0] - K["cx"], uv[:, 1] - K["cy"]
        c_, s_ = s * math.cos(ph), s * math.sin(ph)
        return np.column_stack([K["cx"] + c_ * a - s_ * b + du, K["cy"] + s_ * a + c_ * b + dv])

    def res(x):
        hull, line = hull_outline(proj(x))
        inside = (line[:, 0] > 2) & (line[:, 0] < W - 3) & (line[:, 1] > 2) & (line[:, 1] < H - 4)
        ok = line[inside].astype(int)
        vis = np.zeros(len(line), bool)
        vis[inside] = fmask[ok[:, 1], ok[:, 0]] == 0
        r1 = np.zeros(160)
        if vis.any():
            d = bilinear(ex["dt"], line[vis])
            r1 = d[np.linspace(0, len(d) - 1, 160).astype(int)]
        hull32 = hull.astype(np.float32).reshape(-1, 1, 2)
        r2 = np.array([cv2.pointPolygonTest(hull32, (float(u), float(v)), True) for u, v in cont]) if len(cont) else np.zeros(1)
        # 윗면 앞 가장자리 (퍽 윗면 중 앞쪽 절반, 화면 안 · 영역 안 점만)
        R, c = pose(x * SCALE, nominal)
        far = top_far_points(shape, R, c)
        uvf = proj_pts(far, x)
        okf = (uvf[:, 0] > 2) & (uvf[:, 0] < W - 3) & (uvf[:, 1] > 2) & (uvf[:, 1] < H - 6)
        r3 = np.zeros(40)
        if okf.sum() >= 5:
            d3 = np.minimum(bilinear(ex["dt_in"], uvf[okf]), 6.0)
            r3 = d3[np.linspace(0, len(d3) - 1, 40).astype(int)] * 0.7
        return np.concatenate([r1 * 0.7, r2, r3, prior * x])

    STEP = np.array([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])

    def jac(x):
        J = np.zeros((len(res(x)), 6))
        for i in range(6):
            e = np.zeros(6)
            e[i] = STEP[i]
            J[:, i] = (res(x + e) - res(x - e)) / (2 * STEP[i])
        return J

    best = None
    for s0 in [(0, 0, 0), (0, 0, 4), (0, 0, -4), (3, 3, 0), (-3, -3, 0), (3, -3, 0), (-3, 3, 0)]:
        x0 = np.array([s0[0], 0.0, s0[1], s0[2], 0.0, 0.0], float)
        sol = least_squares(res, x0, jac=jac, loss="soft_l1", f_scale=2.0, max_nfev=25,
                            bounds=([-12, -3, -12, -20, -6, -6], [12, 3, 12, 20, 6, 6]))
        if best is None or sol.cost < best.cost:
            best = sol
    return best.x * SCALE, best.cost


def fingertip_marks(bgr, expected, win=26, scale=1.0):
    """손가락 위 검은 표시의 무게 중심 (좌 · 우). expected = 예상 화면 자리 2 × 2. 찾지 못하면 None."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    out = []
    win = int(win * scale)
    for u, v in expected:
        x0, x1 = int(max(0, u - win)), int(min(W, u + win))
        y0, y1 = int(max(0, v - win)), int(min(H, v + win))
        roi = gray[y0:y1, x0:x1]
        hi, lo = np.percentile(roi, 80), roi.min()
        if hi - lo < 40:
            return None
        th = (hi + lo) / 2
        m = (roi < th).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        if n < 2:
            return None
        k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        if not (100 * scale ** 2 < st[k, cv2.CC_STAT_AREA] < 2500 * scale ** 2):
            return None
        w = np.clip((th - roi) / max(1.0, th - lo), 0, 1) * cv2.dilate((lab == k).astype(np.uint8), np.ones((3, 3), np.uint8))
        ys, xs = np.mgrid[y0:y1, x0:x1]
        out.append((float((w * xs).sum() / w.sum()), float((w * ys).sum() / w.sum())))
    return np.array(out)
