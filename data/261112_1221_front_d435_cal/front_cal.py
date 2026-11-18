#!/usr/bin/env python3
"""앞 D435 장착 각 풀이 (PC). 바닥 태그 두 줄로 URDF 장착 값의 롤 · 피치 · 요 오차를 구한다.

배치: 좌우 바퀴 바깥 면에 곧은 자를 하나씩 붙이고(앞 · 뒤 바퀴 둘 다 닿게) 앞으로 내민다.
      tag36h11 태그를 자 안쪽 가장자리에 한 변을 붙여 바닥에 줄마다 3 장씩 놓는다.
풀이: 태그마다 IPPE 자세 → 가운데 · 꼭짓점을 URDF 값으로 base_footprint 에 옮긴다.
      (1) 모든 점의 평면 법선이 z 가 되게 돌림(롤 · 피치) (2) 두 줄 방향이 x 가 되게 돌림(요).
      이 돌림이 '실제 카메라 = 돌림 × URDF' 의 오차다(+요 = 실제가 URDF 보다 왼쪽으로 돈 것).
      높이 = 평면 z 가 0 이 되게, 옆 = 두 줄 가운데가 y 0 이 되게(바퀴가 좌우 대칭이라고 봄) 고친 값도 낸다.

front_cal.py solve <폴더> [<폴더> ...] [--tag 0.10] [--xyz x y z --rpy r p y] [--json 결과.json]
front_cal.py selftest [--n 20]           모의: 알려진 오차로 그린 영상에서 되찾는지 본다.
"""
import argparse, glob, json, math, os, sys
import cv2, numpy as np

URDF_XYZ = [-0.0480903, 0.0250244, 0.4836226]   # base_link → top_camera_front_link (robot.urdf.xacro dev)
URDF_RPY = [0.0, 0.6144098, 0.0]
BL_TO_BF_Z = 0.00675                              # base_footprint 는 base_link 보다 6.75 mm 아래 (wheel_z − r)
DEF_LINK_OPT = {'xyz': [0.0, 0.015, 0.0], 'rpy': [-math.pi / 2, 0.0, -math.pi / 2]}   # meta 에 TF 가 없을 때만 (D435 대표값)
SIZES = {**{i: 0.016 for i in range(12)}, 12: 0.048, 13: 0.048, 14: 0.048}   # 과제 태그 실측 (퍽 16 mm · 장소 L1~L3 48 mm). 그 밖 ID 는 --tag
LIM = {'roll': 0.7, 'pitch': 0.28, 'yaw': 0.7, 'rep_yaw': 0.3, 'rep_pitch': 0.2, 'rep_roll': 0.3}


def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R2rpy(R):
    return [math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1, min(1, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])]


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(R, t):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M


def T_bf_opt(xyz, rpy, link_opt):
    T_bf_bl = T(np.eye(3), [0, 0, BL_TO_BF_Z])
    T_bl_cl = T(rpy2R(*rpy), xyz)
    if 'xyzw' in link_opt:
        T_cl_o = T(q2R(*link_opt['xyzw']), link_opt['xyz'])
    else:
        T_cl_o = T(rpy2R(*link_opt['rpy']), link_opt['xyz'])
    return T_bf_bl @ T_bl_cl @ T_cl_o, T_cl_o


class _OldDet:   # OpenCV < 4.7 (PC 시스템 4.6)
    def __init__(self, d, p): self.d, self.p = d, p
    def detectMarkers(self, g): return cv2.aruco.detectMarkers(g, self.d, parameters=self.p)


def detector():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        return cv2.aruco.ArucoDetector(d, p)
    p = cv2.aruco.DetectorParameters_create(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return _OldDet(d, p)


def marker_img(d, tid, px):
    return cv2.aruco.generateImageMarker(d, tid, px) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(d, tid, px)


def size_of(i, size):
    return SIZES.get(i, size)


def tag_points(folder, size):
    """태그 ID → 광학 프레임의 가운데 · 꼭짓점 4개 (장별 중앙값), 장마다 검출 수."""
    meta = json.load(open(os.path.join(folder, 'meta.json')))
    K = np.array(meta['K'], float).reshape(3, 3)
    D = np.array(meta.get('D') or [0] * 5, float)
    det = detector()
    per = {}; counts = []
    for f in sorted(glob.glob(os.path.join(folder, '*.png'))):
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = det.detectMarkers(g)
        counts.append(0 if ids is None else len(ids))
        if ids is None:
            continue
        for c, i in zip(corners, ids.ravel()):
            s = size_of(int(i), size) / 2
            obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
            ok, rv, tv = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            R, _ = cv2.Rodrigues(rv)
            pts = np.vstack([tv.ravel(), (R @ obj.T).T + tv.ravel()])
            per.setdefault(int(i), []).append(pts)
    return {i: np.median(np.array(v), axis=0) for i, v in per.items()}, counts, meta


def rot_a_to_b(a, b):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); s = np.linalg.norm(v); c = float(a @ b)
    if s < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def solve_one(folder, size, xyz, rpy):
    pts, counts, meta = tag_points(folder, size)
    link_opt = meta.get('T_link_optical') or DEF_LINK_OPT
    Tn, T_cl_o = T_bf_opt(xyz, rpy, link_opt)
    c = Tn[:3, 3]
    P = {i: (Tn[:3, :3] @ v.T).T + Tn[:3, 3] for i, v in pts.items()}    # 명목 base_footprint
    if len(P) < 4:
        return {'folder': folder, 'error': f'태그 {len(P)} 개만 검출', 'counts': counts}
    allq = np.vstack([v for v in P.values()]) - c
    m = allq.mean(0); _, _, Vt = np.linalg.svd(allq - m, full_matrices=False); n = Vt[2] * (1 if Vt[2][2] > 0 else -1)
    R1 = rot_a_to_b(n, np.array([0, 0, 1.0]))
    cen = {i: R1 @ (v[0] - c) + c for i, v in P.items()}
    bars = {'L': [i for i in cen if cen[i][1] > 0], 'R': [i for i in cen if cen[i][1] <= 0]}
    phis = {}
    for k, ids in bars.items():
        if len(ids) < 2:
            continue
        xy = np.array([cen[i][:2] for i in ids])
        _, _, Vt2 = np.linalg.svd(xy - xy.mean(0), full_matrices=False); d = Vt2[0] * (1 if Vt2[0][0] > 0 else -1)
        phis[k] = math.atan2(d[1], d[0])
    if not phis:
        return {'folder': folder, 'error': '줄마다 태그 2 장 이상이 없음', 'counts': counts}
    phi = float(np.mean(list(phis.values())))
    R2 = rpy2R(0, 0, -phi)
    RD = R2 @ R1
    corr = {i: (RD @ (v.T - c[:, None])).T + c for i, v in P.items()}
    z0 = float(np.mean([v[:, 2] for v in corr.values()]))
    ys = {k: float(np.mean([corr[i][0][1] for i in ids])) for k, ids in bars.items() if ids}
    dy = -0.5 * (ys['L'] + ys['R']) if len(ys) == 2 else 0.0
    tD = c - RD @ c + np.array([0, 0, -z0]) + np.array([0, dy, 0])
    D4 = T(RD, tD)
    # 줄 곧음: 고친 뒤 줄마다 가운데 y 의 흩어짐 [mm], 평면 잔차 [mm]
    straight = {k: float(np.std([corr[i][0][1] for i in ids]) * 1e3) for k, ids in bars.items() if len(ids) >= 2}
    plane_rms = float(np.sqrt(np.mean(np.concatenate([v[:, 2] - z0 for v in corr.values()]) ** 2)) * 1e3)
    T_bf_bl = T(np.eye(3), [0, 0, BL_TO_BF_Z])
    T_bl_cl_new = np.linalg.inv(T_bf_bl) @ D4 @ T_bf_bl @ T(rpy2R(*rpy), xyz)
    err = [math.degrees(a) for a in R2rpy(RD)]
    return {'folder': folder, 'n_frames': len(counts), 'tags_per_frame_min': int(min(counts)), 'tags': sorted(P),
            'bars': bars, 'err_deg': {'roll': err[0], 'pitch': err[1], 'yaw': err[2]},
            'yaw_bar_deg': {k: math.degrees(-v) for k, v in phis.items()},
            'dz_mm': -z0 * 1e3, 'dy_mm': dy * 1e3, 'bar_y_mm': {k: v * 1e3 for k, v in ys.items()},
            'straight_mm': straight, 'plane_rms_mm': plane_rms,
            'tag_x_m': {i: float(corr[i][0][0]) for i in corr},
            'urdf_new': {'xyz': T_bl_cl_new[:3, 3].tolist(), 'rpy': R2rpy(T_bl_cl_new[:3, :3])},
            'link_opt_from': 'meta' if meta.get('T_link_optical') else 'default'}


def report(res):
    ok = [r for r in res if 'error' not in r]
    for r in res:
        if 'error' in r:
            print(f"{r['folder']}: 실패 — {r['error']} (장별 검출 {r['counts'][:10]})"); continue
        e = r['err_deg']
        print(f"{os.path.basename(r['folder'].rstrip('/'))}: 롤 {e['roll']:+.2f}° 피치 {e['pitch']:+.2f}° 요 {e['yaw']:+.2f}° "
              f"(줄별 요 {', '.join(f'{k} {v:+.2f}' for k, v in r['yaw_bar_deg'].items())}) "
              f"높이 {r['dz_mm']:+.1f} mm 옆 {r['dy_mm']:+.1f} mm | 태그 {len(r['tags'])} 장, 장당 최소 {r['tags_per_frame_min']}, "
              f"평면 잔차 {r['plane_rms_mm']:.1f} mm, 줄 곧음 {', '.join(f'{k} {v:.1f}' for k, v in r['straight_mm'].items())} mm, TF {r['link_opt_from']}")
    if not ok:
        return None
    E = {k: np.array([r['err_deg'][k] for r in ok]) for k in ('roll', 'pitch', 'yaw')}
    mean = {k: float(v.mean()) for k, v in E.items()}
    rng = {k: float(v.max() - v.min()) for k, v in E.items()}
    print(f"평균: 롤 {mean['roll']:+.2f}° 피치 {mean['pitch']:+.2f}° 요 {mean['yaw']:+.2f}° | 놓기 간 폭: 롤 {rng['roll']:.2f} 피치 {rng['pitch']:.2f} 요 {rng['yaw']:.2f}°")
    xyz = np.mean([r['urdf_new']['xyz'] for r in ok], 0); rpy = np.mean([r['urdf_new']['rpy'] for r in ok], 0)
    print(f"고친 URDF top_camera_front_link: xyz=\"{xyz[0]:.7f} {xyz[1]:.7f} {xyz[2]:.7f}\" rpy=\"{rpy[0]:.7f} {rpy[1]:.7f} {rpy[2]:.7f}\"")
    if len(ok) >= 2:
        rep = rng['yaw'] <= LIM['rep_yaw'] and rng['pitch'] <= LIM['rep_pitch'] and rng['roll'] <= LIM['rep_roll']
        print(f"반복성 {'통과' if rep else '불통과'} (요 ≤ {LIM['rep_yaw']} · 피치 ≤ {LIM['rep_pitch']} · 롤 ≤ {LIM['rep_roll']}°)")
    within = all(abs(mean[k]) <= LIM[k] for k in ('roll', 'pitch', 'yaw'))
    print(f"오차 한계 안 {'예' if within else '아니오'} (|요| ≤ {LIM['yaw']} · |피치| ≤ {LIM['pitch']} · |롤| ≤ {LIM['roll']}°)")
    return {'mean_deg': mean, 'range_deg': rng, 'urdf_new': {'xyz': xyz.tolist(), 'rpy': rpy.tolist()}}


# ---------- 모의 ----------
def render_scene(out, D_true, size=0.10, n=20, seed=0, K=None, layout=None, link_opt=DEF_LINK_OPT, place_err=0.0):
    rng = np.random.default_rng(seed)
    K = K if K is not None else np.array([[605.85, 0, 324.37], [0, 605.68, 245.14], [0, 0, 1]])
    W, H = 640, 480
    Tn, _ = T_bf_opt(URDF_XYZ, URDF_RPY, link_opt)
    Tt = D_true @ Tn
    Tinv = np.linalg.inv(Tt)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    layout = layout or [(20 + i, x, sgn * 0.11, size) for i, (sgn, x) in enumerate([(s, x) for s in (1, -1) for x in (0.40, 0.65, 0.90)])]
    base = (np.full((H, W), 110, np.uint8) + rng.integers(0, 25, (H, W)).astype(np.uint8))
    base = cv2.GaussianBlur(base, (5, 5), 1.5)
    img = base.astype(np.float32)
    px = 160; pad = 20
    for item in layout:
        tid, x, y = item[:3]; tsize = item[3] if len(item) > 3 else size
        x += rng.normal(0, place_err); y += rng.normal(0, place_err)
        m = marker_img(d, tid, px)
        m = cv2.copyMakeBorder(m, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255).astype(np.float32)
        h = tsize / 2 * (px + 2 * pad) / px
        # 마커 이미지 위 = +x(앞), 왼 = +y. 이미지 꼭짓점 (0,0) (w,0) (w,h) (0,h)
        world = np.array([[x + h, y + h, 0], [x + h, y - h, 0], [x - h, y - h, 0], [x - h, y + h, 0]])
        pc = (Tinv[:3, :3] @ world.T).T + Tinv[:3, 3]
        uv = (K @ pc.T).T; uv = uv[:, :2] / uv[:, 2:]
        S = m.shape[0]
        Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [S, 0], [S, S], [0, S]]), np.float32(uv))
        warped = cv2.warpPerspective(m, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=-1)
        mask = cv2.warpPerspective(np.ones_like(m), Hm, (W, H), flags=cv2.INTER_LINEAR)
        img = img * (1 - mask) + np.where(warped < 0, 0, warped) * mask
    os.makedirs(out, exist_ok=True)
    for k in range(n):
        f = np.clip(img + rng.normal(0, 2.0, img.shape), 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(out, f'{k:03d}.png'), cv2.cvtColor(f, cv2.COLOR_GRAY2BGR))
    json.dump({'K': K.ravel().tolist(), 'D': [0] * 5, 'T_link_optical': None, 'n': n}, open(os.path.join(out, 'meta.json'), 'w'))


def selftest(args):
    base = args.out
    cases = [((0, 0, 0), 0, 0), ((0.5, -0.8, 1.2), 0.004, -0.006), ((-0.3, 0.5, -0.9), -0.008, 0.003), ((0, 0, 0.7), 0, 0)]
    worst = {'roll': 0, 'pitch': 0, 'yaw': 0, 'dz': 0, 'dy': 0}
    for ci, ((r, p, y), dy, dz) in enumerate(cases):
        RD = rpy2R(*[math.radians(a) for a in (r, p, y)])
        Tn, _ = T_bf_opt(URDF_XYZ, URDF_RPY, DEF_LINK_OPT); c = Tn[:3, 3]
        Dt = T(RD, c - RD @ c + np.array([0, dy, dz]))
        f = os.path.join(base, f'case{ci}')
        render_scene(f, Dt, n=args.n, seed=ci, place_err=args.place_err)
        res = solve_one(f, 0.10, URDF_XYZ, URDF_RPY)
        if 'error' in res:
            print(f'case{ci}: {res["error"]}'); return 1
        e = res['err_deg']
        d = {'roll': e['roll'] - r, 'pitch': e['pitch'] - p, 'yaw': e['yaw'] - y,
             'dz': res['dz_mm'] - dz * 1e3, 'dy': res['dy_mm'] - dy * 1e3}
        for k in worst:
            worst[k] = max(worst[k], abs(d[k]))
        print(f"case{ci} 참 ({r:+.1f}, {p:+.1f}, {y:+.1f})° dy {dy*1e3:+.0f} dz {dz*1e3:+.0f} mm → "
              f"찾음 ({e['roll']:+.2f}, {e['pitch']:+.2f}, {e['yaw']:+.2f})° dy {res['dy_mm']:+.1f} dz {res['dz_mm']:+.1f} mm | 태그 {len(res['tags'])}")
    print('최대 차이: ' + ', '.join(f'{k} {v:.3f}' + ('°' if k in ('roll', 'pitch', 'yaw') else ' mm') for k, v in worst.items()))
    return worst


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['solve', 'selftest'])
    ap.add_argument('folders', nargs='*')
    ap.add_argument('--tag', type=float, default=0.10, help='태그 검은 사각형 한 변 [m] (실측)')
    ap.add_argument('--xyz', type=float, nargs=3, default=URDF_XYZ)
    ap.add_argument('--rpy', type=float, nargs=3, default=URDF_RPY)
    ap.add_argument('--json')
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--place_err', type=float, default=0.0, help='모의: 태그 놓기 흩어짐 σ [m]')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'selftest'))
    a = ap.parse_args()
    if a.mode == 'selftest':
        selftest(a); sys.exit(0)
    res = [solve_one(f, a.tag, a.xyz, a.rpy) for f in a.folders]
    summ = report(res)
    if a.json:
        json.dump({'runs': res, 'summary': summ, 'tag': a.tag, 'xyz': a.xyz, 'rpy': a.rpy}, open(a.json, 'w'), indent=1, default=str)
