#!/usr/bin/env python3
"""앞 D435 장착 각 풀이 — 모니터 판 (PC). 인쇄 대신 바닥에 눕힌 모니터에 gen_screen.py 화면을 띄운다.

배치: 왼쪽 바퀴 바깥 면(앞 · 뒤 둘 다)에 곧은 자를 붙여 앞으로 내민다. 모니터를 화면이 위로 오게 바닥에 눕히고,
      화면 '위쪽' 테두리를 자에 붙인다(화면 오른쪽 = 로봇 앞). 휴대폰 수평계로 화면의 앞뒤 · 좌우 기울기를 잰다.
풀이: 모든 태그 꼭짓점을 한 판으로 solvePnP → 판 자세. URDF 값으로 base_footprint 에 옮긴 뒤
      (1) 판 법선이 잰 기울기의 법선이 되게 돌림(롤 · 피치) (2) 판 가로축(화면 오른쪽)이 x 가 되게 돌림(요).
      front_cal.py 와 같은 뜻: 돌림 = 실제 카메라 오차(+요 = 실제가 URDF 보다 왼쪽으로 돈 것).

board_cal.py solve <폴더> [<폴더> ...] --screen screen_1920x1080.json --black_mm <화면 검은 변 실측 mm>
             [--tilt <앞뒤 °> <좌우 °>]   (앞뒤 + = 화면 앞쪽(로봇 쪽에서 먼 쪽)이 높음, 좌우 + = 왼쪽이 높음)
board_cal.py selftest
"""
import argparse, glob, json, math, os, sys
import cv2, numpy as np
import front_cal as F


def board_obj(scr, s):
    """태그 ID → 판 좌표 꼭짓점 4 개 [m] (X = 화면 오른쪽, Y = 화면 위, Z = 화면 밖)."""
    return {t['id']: np.array([[(u - 0.5) * s, -(v - 0.5) * s, 0.0] for u, v in t['corners_px']]) for t in scr['tags']}   # 화소 중심 = 정수 좌표 → 경계는 −0.5


def solve_board(folder, scr, s, xyz, rpy, tilt=(0.0, 0.0)):
    meta = json.load(open(os.path.join(folder, 'meta.json')))
    K = np.array(meta['K'], float).reshape(3, 3); D = np.array(meta.get('D') or [0] * 5, float)
    obj = board_obj(scr, s); det = F.detector(); per = {}; counts = []
    for f in sorted(glob.glob(os.path.join(folder, '*.png'))):
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = det.detectMarkers(g)
        n = 0
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) in obj:
                    per.setdefault(int(i), []).append(c.reshape(4, 2)); n += 1
        counts.append(n)
    if len(per) < 3:
        return {'folder': folder, 'error': f'화면 태그 {len(per)} 장만 검출', 'counts': counts}
    ids = sorted(per)
    img = np.vstack([np.median(np.array(per[i]), 0) for i in ids]).astype(np.float64)
    ob = np.vstack([obj[i] for i in ids])
    ok, rv, tv = cv2.solvePnP(ob, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    proj, _ = cv2.projectPoints(ob, rv, tv, K, D)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, 1))))
    R_ob, _ = cv2.Rodrigues(rv)
    link_opt = meta.get('T_link_optical') or F.DEF_LINK_OPT
    Tn, _ = F.T_bf_opt(xyz, rpy, link_opt)
    c = Tn[:3, 3]
    Rb = Tn[:3, :3] @ R_ob                      # 판 축의 base 표현 (명목)
    pb = Tn[:3, :3] @ tv.ravel() + Tn[:3, 3]    # 판 원점 (화면 왼위)
    n_true = F.rpy2R(math.radians(tilt[1]), -math.radians(tilt[0]), 0) @ np.array([0, 0, 1.0])
    R1 = F.rot_a_to_b(Rb[:, 2], n_true)
    # 화면 가로 · 세로 축 중 로봇 앞(x)에 가장 가까운 축(± 포함)을 앞으로 본다 (화면을 세로 · 가로 어느 쪽으로 놓아도 됨)
    cands = [(sg * (R1 @ Rb[:, k]), f"{'+-'[sg < 0]}{'XY'[k]}") for k in (0, 1) for sg in (1, -1)]
    d, axis = max(cands, key=lambda t: t[0][0])
    phi = math.atan2(d[1], d[0])
    RD = F.rpy2R(0, 0, -phi) @ R1
    corners_b = (Rb @ ob.T).T + pb
    corr = (RD @ (corners_b - c).T).T + c
    z0 = float(np.mean(corr[:, 2]))
    e = [math.degrees(a) for a in F.R2rpy(RD)]
    # 화면 위쪽 테두리(태그 아니고 화면 끝, v = 0) 의 y : 자 자리 확인용
    top_y = float(np.mean([(RD @ (Rb @ np.array([u * s, 0, 0]) + pb - c) + c)[1] for u in (0, scr['width'])]))
    return {'folder': folder, 'n_frames': len(counts), 'tags_per_frame_min': int(min(counts)), 'tags': ids,
            'reproj_px': reproj, 'fwd_axis': axis, 'err_deg': {'roll': e[0], 'pitch': e[1], 'yaw': e[2]},
            'screen_z_mm': z0 * 1e3, 'screen_top_y_mm': top_y * 1e3,
            'screen_x_mm': [float(corr[:, 0].min() * 1e3), float(corr[:, 0].max() * 1e3)],
            'link_opt_from': 'meta' if meta.get('T_link_optical') else 'default', 'RD': RD.tolist()}


def report(res, xyz, rpy):
    ok = [r for r in res if 'error' not in r]
    for r in res:
        if 'error' in r:
            print(f"{r['folder']}: 실패 — {r['error']} (장별 {r['counts'][:10]})"); continue
        e = r['err_deg']
        print(f"{os.path.basename(r['folder'].rstrip('/'))}: 롤 {e['roll']:+.2f}° 피치 {e['pitch']:+.2f}° 요 {e['yaw']:+.2f}° | "
              f"태그 {len(r['tags'])} 장(장당 최소 {r['tags_per_frame_min']}), 재투영 {r['reproj_px']:.2f} px, 앞 축 {r['fwd_axis']}, "
              f"화면 높이 {r['screen_z_mm']:+.1f} mm, 화면 위 테두리 y {r['screen_top_y_mm']:+.1f} mm, x {r['screen_x_mm'][0]:.0f} ~ {r['screen_x_mm'][1]:.0f} mm, TF {r['link_opt_from']}")
    if not ok:
        return None
    E = {k: np.array([r['err_deg'][k] for r in ok]) for k in ('roll', 'pitch', 'yaw')}
    mean = {k: float(v.mean()) for k, v in E.items()}; rng = {k: float(v.max() - v.min()) for k, v in E.items()}
    L = F.LIM
    print(f"평균: 롤 {mean['roll']:+.2f}° 피치 {mean['pitch']:+.2f}° 요 {mean['yaw']:+.2f}° | 놓기 간 폭: 롤 {rng['roll']:.2f} 피치 {rng['pitch']:.2f} 요 {rng['yaw']:.2f}°")
    if len(ok) >= 2:
        rep = rng['yaw'] <= L['rep_yaw'] and rng['pitch'] <= L['rep_pitch'] and rng['roll'] <= L['rep_roll']
        print(f"R1-a 반복성 {'통과' if rep else '불통과'} (요 ≤ {L['rep_yaw']} · 피치 ≤ {L['rep_pitch']} · 롤 ≤ {L['rep_roll']}°)")
    within = all(abs(mean[k]) <= L[k] for k in ('roll', 'pitch', 'yaw'))
    print(f"R1-b 오차 한계 안 {'예' if within else '아니오'} (|요| ≤ {L['yaw']} · |피치| ≤ {L['pitch']} · |롤| ≤ {L['roll']}°)")
    # 고친 URDF 값 (돌림만, 위치는 그대로): 돌림은 카메라 자리 둘레
    RD = F.rpy2R(*[math.radians(mean[k]) for k in ('roll', 'pitch', 'yaw')])
    R_new = RD @ F.rpy2R(*rpy)
    print(f"고친 URDF top_camera_front_link (돌림만): xyz=\"{xyz[0]:.7f} {xyz[1]:.7f} {xyz[2]:.7f}\" rpy=\"" + ' '.join(f'{a:.7f}' for a in F.R2rpy(R_new)) + '"')
    return {'mean_deg': mean, 'range_deg': rng}


def render_board(out, D_true, scr, s, x0=0.38, y_top=0.16, tilt=(0, 0), n=10, seed=0, glare=False):
    rng = np.random.default_rng(seed)
    K = np.array([[605.85, 0, 324.37], [0, 605.68, 245.14], [0, 0, 1]]); W, H = 640, 480
    SS = 4; Ks = K.copy(); Ks[:2] *= SS; Ks[:2, 2] += (SS - 1) / 2   # 4 배로 그린 뒤 줄임 (축소 계단 현상 방지)
    Tn, _ = F.T_bf_opt(F.URDF_XYZ, F.URDF_RPY, F.DEF_LINK_OPT); Tt = D_true @ Tn; Ti = np.linalg.inv(Tt)
    scr_img = cv2.imread(os.path.splitext(scr['_path'])[0] + '.png', cv2.IMREAD_GRAYSCALE).astype(np.float32)
    Rt = F.rpy2R(math.radians(tilt[1]), -math.radians(tilt[0]), 0)
    Sw, Sh = scr['width'], scr['height']
    pts = []
    for u, v in ((-0.5, -0.5), (Sw - 0.5, -0.5), (Sw - 0.5, Sh - 0.5), (-0.5, Sh - 0.5)):
        p = Rt @ np.array([u * s, -v * s, 0]) + np.array([x0, y_top, 0.02])
        pc = Ti[:3, :3] @ p + Ti[:3, 3]; q = Ks @ pc; pts.append(q[:2] / q[2])
    Hm = cv2.getPerspectiveTransform(np.float32([[-0.5, -0.5], [Sw - 0.5, -0.5], [Sw - 0.5, Sh - 0.5], [-0.5, Sh - 0.5]]), np.float32(pts))
    wimg = cv2.warpPerspective(scr_img * 0.85 + 20, Hm, (W * SS, H * SS), flags=cv2.INTER_LINEAR, borderValue=-1)
    img = cv2.resize(np.where(wimg < 0, 110.0, wimg).astype(np.float32), (W, H), interpolation=cv2.INTER_AREA)
    if glare:
        yy, xx = np.mgrid[0:H, 0:W]; img = img + 60 * np.exp(-(((xx - 380) / 60) ** 2 + ((yy - 200) / 40) ** 2))
    os.makedirs(out, exist_ok=True)
    for k in range(n):
        cv2.imwrite(os.path.join(out, f'{k:03d}.png'), np.clip(img + rng.normal(0, 2.0, img.shape), 0, 255).astype(np.uint8))
    json.dump({'K': K.ravel().tolist(), 'D': [0] * 5, 'T_link_optical': None}, open(os.path.join(out, 'meta.json'), 'w'))


def selftest(a):
    scr = json.load(open(a.screen)); scr['_path'] = a.screen
    s = 0.277e-3                                             # 24" 1920 가로 0.277 mm/px (가정)
    Tn, _ = F.T_bf_opt(F.URDF_XYZ, F.URDF_RPY, F.DEF_LINK_OPT); c = Tn[:3, 3]
    cases = [((0, 0, 0), (0, 0), False), ((0.5, -0.8, 1.2), (0, 0), False), ((-0.3, 0.5, -0.9), (0.6, -0.4), False), ((0, 0, 0.7), (0, 0), True)]
    worst = {'roll': 0, 'pitch': 0, 'yaw': 0}
    for ci, (t, tilt, glare) in enumerate(cases):
        RD = F.rpy2R(*[math.radians(v) for v in t]); D = F.T(RD, c - RD @ c)
        f = os.path.join(a.out, f'case{ci}')
        render_board(f, D, scr, s, tilt=tilt, seed=ci, glare=glare)
        r = solve_board(f, scr, s, F.URDF_XYZ, F.URDF_RPY, tilt=tilt)
        if 'error' in r:
            print(f'case{ci}', r['error'], r['counts'][:5]); continue
        e = r['err_deg']
        for k, v in zip(('roll', 'pitch', 'yaw'), t): worst[k] = max(worst[k], abs(e[k] - v))
        print(f"case{ci} 참 {t} 기울기 {tilt} 반사 {glare} → ({e['roll']:+.2f}, {e['pitch']:+.2f}, {e['yaw']:+.2f})° 태그 {len(r['tags'])} 재투영 {r['reproj_px']:.2f} px")
    print('최대 차이: ' + ', '.join(f'{k} {v:.3f}°' for k, v in worst.items()))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['solve', 'selftest'])
    ap.add_argument('folders', nargs='*')
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--screen', default=os.path.join(here, 'screen_1920x1080.json'))
    ap.add_argument('--black_mm', type=float, help='화면에 뜬 태그 검은 변 실측 [mm]')
    ap.add_argument('--tilt', type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument('--xyz', type=float, nargs=3, default=F.URDF_XYZ)
    ap.add_argument('--rpy', type=float, nargs=3, default=F.URDF_RPY)
    ap.add_argument('--json')
    ap.add_argument('--out', default=os.path.join(here, 'selftest_board'))
    a = ap.parse_args()
    if a.mode == 'selftest':
        selftest(a); sys.exit(0)
    scr = json.load(open(a.screen)); s = a.black_mm * 1e-3 / scr['black_px']
    res = [solve_board(f, scr, s, a.xyz, a.rpy, tuple(a.tilt)) for f in a.folders]
    summ = report(res, a.xyz, a.rpy)
    if a.json:
        json.dump({'runs': res, 'summary': summ, 'mm_per_px': s * 1e3, 'tilt': a.tilt}, open(a.json, 'w'), indent=1, default=str)
