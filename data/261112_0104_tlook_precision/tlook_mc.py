#!/usr/bin/env python3
"""target_look 퍽 위치 오차 모의 (몬테카를로, 영상 렌더 없음).

세운 직육면체 퍽(30 x 30 x 60 mm) 윗면 가운데 근처의 16 mm tag36h11 을 팔 카메라(fx 865, 1280 x 720)가
target_look 자세(광축 수평 아래 theta, 카메라 -> 윗면 d)에서 본다. 태그 네 모서리 화소에 잡음을 넣고
perception 과 같은 방식(IPPE_SQUARE 두 해 + prefer_upright 1 px, 중심 = 태그 - 법선 x 0.030)으로
퍽 중심을 낸 뒤, 다른 추정 방식과 수평 오차를 비교한다.

잡음 모형
  n  : 장마다 바뀌는 모서리 잡음 sigma_n [px]  (실물 정지 흔들림 p95 로 맞춤 -> calibrate)
  b  : 한 번 볼 때(look) 고정인 모서리 치우침 sigma_b [px]  (번짐 · 조명 · 검출 치우침, 실측 없음 -> 범위로)
  off: 태그가 윗면 가운데에서 벗어난 양 [mm], 축마다 N(0, 0.8) 을 +-1.6 으로 자름 (11-09 퍽 실측 틈 5.4~8.5 mm)
  tilt: 퍽 기울기 [deg] (평면에 선 퍽은 0 에 가까움, 범위로)

추정 방식
  M0  지금: 한 장, PnP 법선으로 30 mm 내려감
  M0n M0 를 N 장 평균
  M2  법선 = 위(서 있는 퍽): 태그 위치 xy 그대로 (N 장 평균 가능)
  M3  윗면 높이 고정: 태그 중심 화소 광선 -> z = 윗면 높이 (+dh 오차)
  M4  윗면 윤곽(30 mm 정사각) PnP, 모서리 잡음 = k x sigma_n (태그 위치 어긋남 영향 없음, k 는 가정)
"""
import argparse
import json
import math

import cv2
import numpy as np

FX = 865.4
CX, CY = 584.12, 359.36
W, H_IMG = 1280, 720
K = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1.0]])
TAG = 0.016
HALF_LEN = 0.030
FACE = 0.030
TOP_Z = 0.060


def square(s):
    h = s / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


OBJ_TAG = square(TAG)
OBJ_FACE = square(FACE)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_axis(axis, a):
    return cv2.Rodrigues(np.asarray(axis, float) * a)[0]


def camera_pose(theta, d, phi, aim):
    """월드(z 위) 기준 카메라: 광축이 aim 을 지나고 수평 아래 theta. R_wc 열 = 카메라 x(오른쪽) y(아래) z(앞)."""
    v = np.array([math.cos(theta) * math.cos(phi), math.cos(theta) * math.sin(phi), -math.sin(theta)])
    x = np.cross(v, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(v, x)
    R_wc = np.stack([x, y, v], axis=1)
    return R_wc, aim - d * v


def project(Pw, R_wc, C):
    Pc = (Pw - C) @ R_wc
    return np.stack([FX * Pc[:, 0] / Pc[:, 2] + CX, FX * Pc[:, 1] / Pc[:, 2] + CY], axis=1)


def pnp_center(img, R_wc_est, C_est, K_est, gap_px=1.0, lever=HALF_LEN):
    """perception: IPPE 두 해, 오차 차 <= gap 이면 법선이 수직 · 수평에 가까운 해. 반환 (중심_w, 태그_w, 법선_w)."""
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(OBJ_TAG, img.reshape(-1, 1, 2), K_est, None,
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or len(rvecs) == 0:
        return None
    errs = np.asarray(errs).ravel()
    order = np.argsort(errs)
    sols = []
    for i in order:
        R = cv2.Rodrigues(rvecs[i])[0]
        nw = R_wc_est @ R[:, 2]
        tw = R_wc_est @ tvecs[i].ravel() + C_est
        dev = math.degrees(math.acos(min(1.0, abs(nw[2]) / np.linalg.norm(nw))))
        sols.append((errs[i], min(dev, 90 - dev), tw, nw))
    best = sols[0]
    if len(sols) > 1 and gap_px > 0 and sols[1][0] - sols[0][0] <= gap_px and sols[1][1] < sols[0][1]:
        best = sols[1]
    _, _, tw, nw = best
    return tw - nw * lever, tw, nw


def face_center(img, R_wc_est, C_est, K_est):
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(OBJ_FACE, img.reshape(-1, 1, 2), K_est, None,
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    errs = np.asarray(errs).ravel()
    sols = []
    for i in np.argsort(errs):
        R = cv2.Rodrigues(rvecs[i])[0]
        nw = R_wc_est @ R[:, 2]
        tw = R_wc_est @ tvecs[i].ravel() + C_est
        sols.append((errs[i], math.degrees(math.acos(min(1.0, abs(nw[2])))), tw, nw))
    best = sols[0]
    if len(sols) > 1 and sols[1][0] - sols[0][0] <= 1.0 and sols[1][1] < sols[0][1]:
        best = sols[1]
    return best[2] - best[3] * HALF_LEN


def ray_plane(img, R_wc_est, C_est, K_est, z_plane):
    p = img
    # 대각선 교점 = 투영 중심
    a1, a2, b1, b2 = p[0], p[2], p[1], p[3]
    da, db = a2 - a1, b2 - b1
    A = np.array([da, -db]).T
    s = np.linalg.solve(A, b1 - a1)[0]
    uv = a1 + s * da
    ray_c = np.array([(uv[0] - K_est[0, 2]) / K_est[0, 0], (uv[1] - K_est[1, 2]) / K_est[1, 1], 1.0])
    ray_w = R_wc_est @ ray_c
    t = (z_plane - C_est[2]) / ray_w[2]
    return C_est + t * ray_w


def run(cfg, rng):
    """cfg: theta_deg, d, sigma_n, sigma_b, off_sigma_mm, tilt_deg, n_frames, trials, k_face, dh_mm,
    cal: dict(rot_mrad=[rx,ry,rz] 카메라 축, trans_mm=[x,y,z] 월드, fx_pct)."""
    th = math.radians(cfg['theta_deg'])
    d = cfg['d']
    N = cfg['n_frames']
    cal = cfg.get('cal', {})
    out = {k: [] for k in ('M0', 'M0n', 'M2', 'M2n', 'M3', 'M3n', 'M4n', 'tilt_true')}
    for _ in range(cfg['trials']):
        phi = rng.uniform(-math.pi, math.pi)
        # 퍽 참 중심(수평) — 카메라 겨냥점(예상 자리)에서 +-10 mm
        p = np.array([rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01)])
        yaw = rng.uniform(-math.pi, math.pi)
        tilt = math.radians(cfg['tilt_deg'])
        tilt_axis = rot_z(rng.uniform(-math.pi, math.pi)) @ np.array([1.0, 0, 0])
        R_body = rot_axis(tilt_axis, tilt) @ rot_z(yaw)           # 퍽 몸체(긴 축 = z)
        center = np.array([p[0], p[1], TOP_Z - HALF_LEN])
        top_c = center + R_body @ np.array([0, 0, HALF_LEN])
        off = np.clip(rng.normal(0, cfg['off_sigma_mm'] / 1000, 2), -0.0016, 0.0016) if cfg['off_sigma_mm'] > 0 else np.zeros(2)
        tag_c = top_c + R_body @ np.array([off[0], off[1], 0])
        aim = np.array([0, 0, TOP_Z])
        R_wc, C = camera_pose(th, d, phi, aim)
        tag_w = tag_c + OBJ_TAG @ R_body.T
        face_w = top_c + OBJ_FACE @ R_body.T
        img_t = project(tag_w, R_wc, C)
        img_f = project(face_w, R_wc, C)
        # 추정에 쓰는 (틀릴 수 있는) 카메라 자세 · 내부값
        rx, ry, rz = [v / 1000 for v in cal.get('rot_mrad', [0, 0, 0])]
        R_err = cv2.Rodrigues(np.array([rx, ry, rz]))[0]
        R_wc_e = R_wc @ R_err
        C_e = C + np.array(cal.get('trans_mm', [0, 0, 0])) / 1000
        K_e = K.copy()
        K_e[0, 0] *= 1 + cal.get('fx_pct', 0) / 100
        K_e[1, 1] *= 1 + cal.get('fx_pct', 0) / 100
        bias_t = rng.normal(0, cfg['sigma_b'], (4, 2))
        bias_f = rng.normal(0, cfg['sigma_b'] * cfg['k_face'], (4, 2))
        z_plane = TOP_Z + cfg['dh_mm'] / 1000
        c0, c2, c3, c4 = [], [], [], []
        for _f in range(N):
            im = img_t + bias_t + rng.normal(0, cfg['sigma_n'], (4, 2))
            r = pnp_center(im, R_wc_e, C_e, K_e)
            if r is None:
                continue
            c0.append(r[0][:2])
            c2.append(r[1][:2])
            c3.append(ray_plane(im, R_wc_e, C_e, K_e, z_plane)[:2])
            imf = img_f + bias_f + rng.normal(0, cfg['sigma_n'] * cfg['k_face'], (4, 2))
            c4.append(face_center(imf, R_wc_e, C_e, K_e)[:2])
        truth = center[:2]
        out['M0'].append(c0[0] - truth)
        out['M0n'].append(np.mean(c0, 0) - truth)
        out['M2'].append(c2[0] - truth)
        out['M2n'].append(np.mean(c2, 0) - truth)
        out['M3'].append(c3[0] - truth)
        out['M3n'].append(np.mean(c3, 0) - truth)
        out['M4n'].append(np.mean(c4, 0) - truth)
    res = {}
    for k, v in out.items():
        if not v or k == 'tilt_true':
            continue
        e = np.asarray(v) * 1000
        r = np.hypot(e[:, 0], e[:, 1])
        res[k] = dict(sigma_axis=float(np.sqrt(np.mean(e ** 2))), bias=[float(x) for x in e.mean(0)],
                      r_med=float(np.median(r)), r_p95=float(np.percentile(r, 95)), r_max=float(r.max()))
    return res


def jitter_p95(theta_deg, d, sigma_n, rng, looks=40, frames=100):
    """실물 정지 흔들림 지표와 같은 것: 한 자리 · 한 번 보기에서 M0 수평 위치의 중앙값으로부터 거리 p95 [mm]."""
    vals = []
    th = math.radians(theta_deg)
    for _ in range(looks):
        phi = rng.uniform(-math.pi, math.pi)
        R_wc, C = camera_pose(th, d, phi, np.array([0, 0, TOP_Z]))
        tag_w = np.array([0, 0, TOP_Z]) + OBJ_TAG @ rot_z(rng.uniform(-math.pi, math.pi)).T
        img = project(tag_w, R_wc, C)
        pts = []
        for _f in range(frames):
            r = pnp_center(img + rng.normal(0, sigma_n, (4, 2)), R_wc, C, K)
            pts.append(r[0][:2])
        pts = np.asarray(pts)
        med = np.median(pts, 0)
        vals.append(np.percentile(np.hypot(*(pts - med).T) * 1000, 95))
    return float(np.median(vals)), float(np.percentile(vals, 10)), float(np.percentile(vals, 90))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['calibrate', 'sweep', 'sens'], required=True)
    ap.add_argument('--trials', type=int, default=2000)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    rows = []
    if a.mode == 'calibrate':
        # 실물(11-09 run5 · run3/4): 계산 렌즈 ~ 윗면 207 mm, 광축 약 75 deg(관절 j2+j3+j4 = 1.045 rad + 15 deg)
        for sn in [0.05, 0.1, 0.15, 0.2, 0.3, 0.4]:
            med, lo, hi = jitter_p95(75, 0.207, sn, rng)
            rows.append(dict(sigma_n=sn, jitter_p95_med=med, p10=lo, p90=hi))
            print(rows[-1], flush=True)
    elif a.mode == 'sweep':
        base = dict(theta_deg=70, d=0.17, sigma_n=0.1, sigma_b=0.1, off_sigma_mm=0.8, tilt_deg=0.0,
                    n_frames=10, trials=a.trials, k_face=3.0, dh_mm=0.0)
        cases = [
            ('기본', {}),
            ('잡음만(b0 off0)', dict(sigma_b=0, off_sigma_mm=0)),
            ('off0', dict(off_sigma_mm=0)),
            ('b0', dict(sigma_b=0)),
            ('b0.2', dict(sigma_b=0.2)),
            ('b0.3', dict(sigma_b=0.3)),
            ('b0.5', dict(sigma_b=0.5)),
            ('n0.05', dict(sigma_n=0.05)),
            ('tilt1', dict(tilt_deg=1.0)),
            ('tilt2', dict(tilt_deg=2.0)),
            ('tilt5', dict(tilt_deg=5.0)),
            ('dh+3', dict(dh_mm=3.0)),
            ('theta60', dict(theta_deg=60)),
            ('theta80', dict(theta_deg=80)),
            ('theta90', dict(theta_deg=89.5)),
            ('d0.15', dict(d=0.15)),
            ('d0.22', dict(d=0.22)),
            ('k_face1', dict(k_face=1.0)),
            ('k_face5', dict(k_face=5.0)),
        ]
        for name, over in cases:
            cfg = dict(base, **over)
            res = run(cfg, rng)
            rows.append(dict(case=name, cfg=over, res=res))
            print(name, {k: round(v['sigma_axis'], 2) for k, v in res.items()}, flush=True)
    else:
        base = dict(theta_deg=70, d=0.17, sigma_n=0.0, sigma_b=0.0, off_sigma_mm=0.0, tilt_deg=0.0,
                    n_frames=1, trials=a.trials, k_face=1.0, dh_mm=0.0)
        cases = [('rx+5mrad', dict(rot_mrad=[5, 0, 0])), ('ry+5mrad', dict(rot_mrad=[0, 5, 0])),
                 ('rz+5mrad', dict(rot_mrad=[0, 0, 5])), ('tz+5mm', dict(trans_mm=[0, 0, 5])),
                 ('tx+5mm', dict(trans_mm=[5, 0, 0])), ('fx+1%', dict(fx_pct=1.0))]
        for name, cal in cases:
            res = run(dict(base, cal=cal), rng)
            rows.append(dict(case=name, cal=cal, res=res))
            print(name, {k: round(v['sigma_axis'], 2) for k, v in res.items() if not k.endswith('n')}, flush=True)
    if a.out:
        json.dump(rows, open(a.out, 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
