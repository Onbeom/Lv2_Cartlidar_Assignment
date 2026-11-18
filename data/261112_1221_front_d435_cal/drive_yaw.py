#!/usr/bin/env python3
"""D435 장착 요를 주행 방향으로 잰다 (PC). 판(노트북 화면 태그)을 고정하고 로봇이 곧게 앞뒤로 움직이는 동안 찍은 장들로 푼다.

원리: 장마다 판 PnP → 명목 장착값(xyz · rpy)으로 판 좌표의 로봇 자세(원점 · 머리 방향)를 낸다.
      로봇은 자기 x 축으로만 움직이므로(옆미끄럼 없다고 봄) 움직인 방향 = 참 머리 방향이다.
      카메라 요가 ε 틀리면 계산한 머리 방향이 움직인 방향보다 ε 만큼 돈다 → ε = 계산 머리 방향 − 움직인 방향(부호는 모의로 확인).
      주행 중 로봇이 조금 돌아도 장마다 머리 방향을 같이 보므로 빠진다(구간 평균과 현 방향 비교).
구간: 장 사이 이동 > 2 mm 인 장이 이어진 묶음(≥ 0.10 m 움직인 것만). 멈춘 장은 정지 자세(롤 · 피치)에 쓸 수 있다.

drive_yaw.py solve <폴더> --black_mm 61.5 [--cam front|rear] [--rpy r p y] [--xyz x y z]
drive_yaw.py selftest [--cam front|rear]
"""
import argparse, glob, json, math, os, sys
import cv2, numpy as np
import front_cal as F, board_cal as B

CAMS = {   # base_link → <cam>_link (opencr_hardware xacro). front 피치는 11-12 실측으로 고친 값(PR #240)
    'front': ([-0.0480903, 0.0250244, 0.4836226], [0.0, 0.5692410, 0.0]),
    'rear': ([-0.2079097, -0.0250168, 0.4836226], [0.0, 0.6144098, math.pi]),
}


def per_frame(folder, scr, s, xyz, rpy):
    meta = json.load(open(os.path.join(folder, 'meta.json')))
    K = np.array(meta['K'], float).reshape(3, 3); D = np.array(meta.get('D') or [0] * 5, float)
    Tn, _ = F.T_bf_opt(xyz, rpy, meta.get('T_link_optical') or F.DEF_LINK_OPT)
    obj = B.board_obj(scr, s); det = F.detector(); rows = []
    for f in sorted(glob.glob(os.path.join(folder, '*.png'))):
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = det.detectMarkers(g)
        if ids is None:
            continue
        ob, im = [], []
        for c, i in zip(corners, ids.ravel()):
            if int(i) in obj:
                ob.append(obj[int(i)]); im.append(c.reshape(4, 2))
        if len(ob) < 3:
            continue
        ob = np.vstack(ob); im = np.vstack(im).astype(np.float64)
        ok, rv, tv = cv2.solvePnP(ob, im, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        R_cb, _ = cv2.Rodrigues(rv)
        T_B_board = Tn @ F.T(R_cb, tv.ravel())
        T_board_B = np.linalg.inv(T_B_board)
        p = T_board_B[:3, 3]; h = T_board_B[:3, :3] @ np.array([1.0, 0, 0])
        rows.append({'file': os.path.basename(f), 'n': len(ob) // 4, 'p': p, 'h': math.atan2(h[1], h[0]), 'nz': T_B_board[:3, 2].copy()})
    return rows


def segments(rows, step=0.002, min_len=0.10):
    segs, cur = [], []
    for a, b in zip(rows, rows[1:]):
        if np.linalg.norm(b['p'][:2] - a['p'][:2]) > step:
            cur = cur or [a]; cur.append(b)
        else:
            if cur: segs.append(cur); cur = []
    if cur: segs.append(cur)
    return [g for g in segs if np.linalg.norm(g[-1]['p'][:2] - g[0]['p'][:2]) >= min_len]


def stops(rows, step=0.0007, min_n=8):
    out, cur = [], [rows[0]] if rows else []
    for a, b in zip(rows, rows[1:]):
        if np.linalg.norm(b['p'][:2] - a['p'][:2]) <= step:
            cur.append(b)
        else:
            if len(cur) >= min_n: out.append(cur)
            cur = [b]
    if len(cur) >= min_n: out.append(cur)
    return out


def stop_tilt(g, tilt):
    """정지 장들의 판 법선 평균 → 롤 · 피치 오차 (board_cal 과 같은 뜻). 판 z(화면 밖)는 base 에서 위쪽."""
    n = np.mean([r['nz'] for r in g], 0); n /= np.linalg.norm(n)
    if n[2] < 0: n = -n
    n_true = F.rpy2R(math.radians(tilt[1]), -math.radians(tilt[0]), 0) @ np.array([0, 0, 1.0])
    e = F.R2rpy(F.rot_a_to_b(n, n_true))
    return math.degrees(e[0]), math.degrees(e[1])


def solve(folder, scr, s, xyz, rpy):
    rows = per_frame(folder, scr, s, xyz, rpy)
    out = []
    for g in segments(rows):
        P = np.array([r['p'][:2] for r in g]); m = P.mean(0)
        _, _, Vt = np.linalg.svd(P - m, full_matrices=False); d = Vt[0]
        if d @ (P[-1] - P[0]) < 0: d = -d                       # 움직인 쪽
        H = np.unwrap([r['h'] for r in g]); hbar = float(np.mean(H))
        mv = math.atan2(d[1], d[0])
        fwd = math.cos(mv - hbar) > 0                            # 앞으로 갔나
        mv_head = mv if fwd else mv + math.pi                   # 머리 방향으로 바꾼 움직임 방향
        eps = (hbar - mv_head + math.pi) % (2 * math.pi) - math.pi   # 모의로 부호 확인: + = 실제 카메라가 URDF 보다 왼쪽으로 돎 (board_cal 과 같은 뜻)
        resid = float(np.sqrt(np.mean(((P - m) @ np.array([-d[1], d[0]])) ** 2)) * 1e3)
        out.append({'frames': len(g), 'len_m': float(np.linalg.norm(P[-1] - P[0])), 'dir': '앞' if fwd else '뒤',
                    'yaw_deg': math.degrees(eps), 'turn_deg': math.degrees(H[-1] - H[0]), 'line_rms_mm': resid})
    return rows, out


def report(folder, rows, out, tilt=(0.0, 0.0)):
    print(f"{os.path.basename(folder.rstrip('/'))}: 판 자세 장 {len(rows)}, 주행 구간 {len(out)}")
    st = stops(rows)
    for k, g in enumerate(st):
        r_, p_ = stop_tilt(g, tilt)
        print(f"  정지 {k + 1} ({len(g)} 장, x {g[0]['p'][0]:+.3f} m): 롤 {r_:+.2f}° 피치 {p_:+.2f}°")
    for k, o in enumerate(out):
        print(f"  구간 {k + 1} ({o['dir']}, {o['len_m'] * 100:.1f} cm, {o['frames']} 장): 요 {o['yaw_deg']:+.2f}°  | 구간 중 돎 {o['turn_deg']:+.2f}°, 직선 잔차 {o['line_rms_mm']:.1f} mm")
    if out:
        y = np.array([o['yaw_deg'] for o in out])
        print(f"  평균 요 {y.mean():+.2f}°, 폭 {y.max() - y.min():.2f}° (구간 {len(y)})")


def render_drive(out, scr, s, cam, eps_deg, n=60, travel=0.25, turn_deg=0.4, x0=0.40, seed=0):
    """판을 세상에 고정하고 로봇이 뒤로 travel 만큼(앞 카메라) · 앞으로(뒤 카메라) 움직이며 turn_deg 만큼 도는 장면."""
    rng = np.random.default_rng(seed)
    xyz, rpy = CAMS[cam]
    K = np.array([[605.85, 0, 324.37], [0, 605.68, 245.14], [0, 0, 1]]); W, H = 640, 480
    SS = 4; Ks = K.copy(); Ks[:2] *= SS; Ks[:2, 2] += (SS - 1) / 2
    Tn, _ = F.T_bf_opt(xyz, rpy, F.DEF_LINK_OPT); c = Tn[:3, 3]
    RD = F.rpy2R(0, 0, math.radians(eps_deg)); Tcam_true = F.T(RD, c - RD @ c) @ Tn
    img_src = cv2.imread(os.path.splitext(scr['_path'])[0] + '.png', cv2.IMREAD_GRAYSCALE).astype(np.float32)
    Sw, Sh = scr['width'], scr['height']
    sgn = 1 if cam == 'front' else -1                           # 판이 놓인 쪽
    os.makedirs(out, exist_ok=True)
    for k in range(n):
        f = min(1.0, max(0.0, (k - 5) / (n - 10)))              # 앞뒤 5 장은 멈춤
        th = math.radians(turn_deg) * f
        mv = -sgn * travel * f                                  # 판에서 멀어지는 쪽
        T_w_B = F.T(F.rpy2R(0, 0, th), [mv * math.cos(th / 2), mv * math.sin(th / 2), 0])
        Ti = np.linalg.inv(T_w_B @ Tcam_true)
        pts = []
        for u, v in ((-0.5, -0.5), (Sw - 0.5, -0.5), (Sw - 0.5, Sh - 0.5), (-0.5, Sh - 0.5)):
            p = np.array([sgn * x0 + sgn * v * s, sgn * (u - Sw / 2) * s, 0.08])   # 화면 위 = 로봇 쪽, 화면이 위를 봄
            pc = Ti[:3, :3] @ p + Ti[:3, 3]; q = Ks @ pc; pts.append(q[:2] / q[2])
        Hm = cv2.getPerspectiveTransform(np.float32([[-0.5, -0.5], [Sw - 0.5, -0.5], [Sw - 0.5, Sh - 0.5], [-0.5, Sh - 0.5]]), np.float32(pts))
        w = cv2.warpPerspective(img_src * 0.85 + 20, Hm, (W * SS, H * SS), flags=cv2.INTER_LINEAR, borderValue=-1)
        im = cv2.resize(np.where(w < 0, 110.0, w).astype(np.float32), (W, H), interpolation=cv2.INTER_AREA)
        cv2.imwrite(os.path.join(out, f'{k:03d}.png'), np.clip(im + rng.normal(0, 2.0, im.shape), 0, 255).astype(np.uint8))
    json.dump({'K': K.ravel().tolist(), 'D': [0] * 5, 'T_link_optical': None}, open(os.path.join(out, 'meta.json'), 'w'))


def selftest(a):
    scr = json.load(open(a.screen)); scr['_path'] = a.screen; s = 61.5e-3 / scr['black_px']
    worst = 0
    for ci, (eps, turn) in enumerate([(0.0, 0.0), (1.2, 0.4), (-0.8, -0.6), (0.5, 1.5)]):
        f = os.path.join(a.out, f'{a.cam}_case{ci}')
        render_drive(f, scr, s, a.cam, eps, turn_deg=turn, seed=ci)
        xyz, rpy = CAMS[a.cam]
        rows, out = solve(f, scr, s, xyz, rpy)
        got = out[0]['yaw_deg'] if out else float('nan')
        worst = max(worst, abs(got - eps))
        print(f"{a.cam} case{ci}: 참 요 {eps:+.2f}° · 돎 {turn:+.1f}° → 찾음 {got:+.2f}° (구간 {len(out)}, 장 {len(rows)})")
    print(f'최대 차이 {worst:.3f}°')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['solve', 'selftest'])
    ap.add_argument('folders', nargs='*')
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--screen', default=os.path.join(here, 'screen_1920x1080.json'))
    ap.add_argument('--black_mm', type=float, default=61.5)
    ap.add_argument('--cam', choices=['front', 'rear'], default='front')
    ap.add_argument('--xyz', type=float, nargs=3)
    ap.add_argument('--rpy', type=float, nargs=3)
    ap.add_argument('--out', default=os.path.join(here, 'selftest_drive'))
    ap.add_argument('--tilt', type=float, nargs=2, default=[0.0, 0.0], help='화면 − 책상 수평계 차 (앞뒤 + = 로봇 앞쪽이 높음, 좌우 + = 왼쪽이 높음)')
    a = ap.parse_args()
    if a.mode == 'selftest':
        selftest(a); sys.exit(0)
    scr = json.load(open(a.screen)); s = a.black_mm * 1e-3 / scr['black_px']
    xyz, rpy = CAMS[a.cam]; xyz = a.xyz or xyz; rpy = a.rpy or rpy
    for f in a.folders:
        rows, out = solve(f, scr, s, xyz, rpy); report(f, rows, out, tuple(a.tilt))
