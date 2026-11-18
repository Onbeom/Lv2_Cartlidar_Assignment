"""퍽 모양 · 태그 PnP 가설 · 투영 공용 함수. 치수는 과제 값: 30 x 30 x 60 mm, 원기둥 지름 30, 태그 검은 테두리 16 mm."""
import cv2, numpy as np
HW, HL, TAG = .015, .03, .016
CORNERS = np.array([[x, y, z] for x in (-HW, HW) for y in (-HW, HW) for z in (-HL, HL)], np.float64)   # train_crop.py 와 같은 순서
_a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
CYL = np.vstack([np.c_[HW * np.cos(_a), HW * np.sin(_a), np.full(48, s)] for s in (-HL, HL)])
OBJ_TAG = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], np.float64) * TAG / 2
NAMES = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']
def is_cyl(c): return c in (0, 1)
def model_pts(cls): return CYL if is_cyl(cls) else CORNERS
def tag_solutions(c, K, D=None):
    """IPPE 두 해 [(R, t, 재투영 px)]. R 은 태그 → 카메라, R[:, 2] 는 보이는 면 바깥(카메라 쪽) 법선."""
    n, rs, ts, er = cv2.solvePnPGeneric(OBJ_TAG, np.asarray(c, np.float64), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return [(cv2.Rodrigues(r)[0], t.ravel(), float(e)) for r, t, e in zip(rs, ts, er.ravel())]
def hypotheses(R, t, cls):
    """태그 자세 → 퍽 자세 가설 [(이름, R_puck, 중심)]. R_puck[:, 2] = 긴 축. 끝면은 태그 법선이 긴 축, 옆면(직육면체만)은 면 안 두 축 중 하나."""
    u, v, n = R[:, 0], R[:, 1], R[:, 2]
    out = [('end', R.copy(), t - n * HL)]
    if not is_cyl(cls):
        out.append(('side_u', np.c_[v, n, u], t - n * HW))
        out.append(('side_v', np.c_[u, -n, v], t - n * HW))
    return out
def project(P, Rp, c, K, D=None):
    q, _ = cv2.projectPoints(P @ Rp.T + c, np.zeros(3), np.zeros(3), K, D)
    return q.reshape(-1, 2)
def bbox(q, W=1280, H=720):
    x0, y0 = q.min(0); x1, y1 = q.max(0)
    return np.array([max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)])
def iou(a, b):
    lo = np.maximum(a[:2], b[:2]); hi = np.minimum(a[2:], b[2:]); i = float(np.prod(np.maximum(hi - lo, 0)))
    return i / max(1e-9, float(np.prod(np.maximum(a[2:] - a[:2], 0)) + np.prod(np.maximum(b[2:] - b[:2], 0)) - i))
def read_yolo(path, W=1280, H=720):
    out = []
    for ln in open(path):
        c, x, y, w, h = ln.split(); c = int(c); x, y, w, h = float(x) * W, float(y) * H, float(w) * W, float(h) * H
        out.append((c, np.array([x - w / 2, y - h / 2, x + w / 2, y + h / 2])))
    return out
