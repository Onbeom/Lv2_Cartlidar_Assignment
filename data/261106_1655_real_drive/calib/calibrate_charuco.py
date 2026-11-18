#!/usr/bin/env python3
"""저장한 보정판 사진으로 Arducam 내부값을 구한다 (PC). 왜곡 모델 2개를 맡겨 둔 장으로 견주고 ROS camera_info yaml 을 쓴다.

실행: python3 calibrate_charuco.py <사진 폴더> [--out 폴더]
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'jetson'))
from charuco_util import SQUARES_X, Detector, object_points  # noqa: E402

MIN_CORNERS = 12
MODELS = {'plumb_bob': 0, 'rational_polynomial': cv2.CALIB_RATIONAL_MODEL}


def calibrate(views, size, flags, init=None):
    """init=(K, D) 이면 그 값에서 시작한다. 왜곡이 큰 렌즈 + 화면 끝 장이 섞이면 초깃값 없이는 엉뚱한 해(fx 19000 등)로 빠졌다(11-09)."""
    obj = [object_points(i) for _, i, _ in views]
    img = [p.astype(np.float32) for p, _, _ in views]
    K0, D0 = None, None
    if init is not None:
        K0 = init[0].copy()
        n = 8 if flags & cv2.CALIB_RATIONAL_MODEL else 5
        D0 = np.zeros((1, n)); D0[0, :min(n, init[1].size)] = init[1].ravel()[:n]
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    rms, K, D, _, _ = cv2.calibrateCamera(obj, img, size, K0, D0, flags=flags)
    return rms, K, D


def reproj(view, K, D):
    p, i, _ = view
    o = object_points(i)
    ok, r, t = cv2.solvePnP(o, p.astype(np.float32), K, D)
    q, _ = cv2.projectPoints(o, r, t, K, D)
    return np.linalg.norm(q.reshape(-1, 2) - p, axis=1)


def rms_of(views, K, D):
    e = np.concatenate([reproj(v, K, D) for v in views])
    return float(np.sqrt((e ** 2).mean())), float(np.percentile(e, 99))


def straightness(views, K, D):
    """왜곡 편 코너 줄이 곧은지. 줄마다 직선 맞춤 잔차(px)."""
    res = []
    for p, i, _ in views:
        u = cv2.undistortPoints(p.reshape(-1, 1, 2).astype(np.float32), K, D, P=K).reshape(-1, 2)
        for row in np.unique(i // (SQUARES_X - 1)):
            s = u[i // (SQUARES_X - 1) == row]
            if len(s) >= 5:
                vx, vy, x0, y0 = cv2.fitLine(s, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
                res.append(np.abs((s[:, 0] - x0) * vy - (s[:, 1] - y0) * vx))
    e = np.concatenate(res) if res else np.array([np.nan])
    return float(np.median(e)), float(np.percentile(e, 99))


def coverage(views, size, grid=(8, 5)):
    seen = np.zeros(grid[::-1], bool)
    for p, _, _ in views:
        seen[(p[:, 1] / size[1] * grid[1]).astype(int).clip(0, grid[1] - 1), (p[:, 0] / size[0] * grid[0]).astype(int).clip(0, grid[0] - 1)] = True
    return float(seen.mean())


def fx_sigma(views, size, flags):
    """최종 모델의 fx 표준편차(px). 보정판 자세가 고르지 않으면 커진다."""
    obj = [object_points(i) for _, i, _ in views]
    img = [p.astype(np.float32) for p, _, _ in views]
    si = cv2.calibrateCameraExtended(obj, img, size, None, None, flags=flags)[5]
    return float(si.ravel()[0])


def tilts_deg(views, K, D):
    """장마다 보정판 법선과 광축 사이 각(°). 카메라 고정이면 판을 기울여야 f 가 정해진다."""
    out = []
    for p, i, _ in views:
        ok, r, _t = cv2.solvePnP(object_points(i), p.astype(np.float32), K, D)
        n = cv2.Rodrigues(r)[0][:, 2]
        out.append(float(np.degrees(np.arccos(min(1.0, abs(n[2]))))))
    return np.array(out)


def write_yaml(path, size, K, D, model):
    # 11-09: OpenCV 가 14 개를 돌려줘도 rational_polynomial 은 ROS 형식대로 8 개(뒤 6 개는 플래그를 안 켜 0)
    n = 8 if model == 'rational_polynomial' else 5
    assert not np.any(np.asarray(D).ravel()[n:]), '버릴 왜곡 계수가 0 이 아님'
    D = np.asarray(D).ravel()[:n]
    f = lambda a: ', '.join(f'{v:.10g}' for v in np.asarray(a).ravel())
    P = np.hstack([K, np.zeros((3, 1))])
    path.write_text(f"""image_width: {size[0]}
image_height: {size[1]}
camera_name: arm_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [{f(K)}]
distortion_model: {model}
distortion_coefficients:
  rows: 1
  cols: {D.size}
  data: [{f(D)}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{f(P)}]
""")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', type=Path)
    ap.add_argument('--out', type=Path, default=None)
    a = ap.parse_args()
    out = a.out or a.folder / 'result'
    out.mkdir(parents=True, exist_ok=True)
    det = Detector()
    views, skipped, size = [], [], None
    for fp in sorted(a.folder.glob('frame_*.jpg')):
        g = cv2.imread(str(fp), cv2.IMREAD_GRAYSCALE)
        size = (g.shape[1], g.shape[0])
        p, i, _ = det.detect(g)
        if p is None or len(p) < MIN_CORNERS:
            skipped.append(fp.name)
            continue
        views.append((p, i, fp.name))
    print(f'사진 {len(views) + len(skipped)} 장, 코너 쓸 수 있는 장 {len(views)}, 뺀 장 {len(skipped)}')
    if len(views) < 10:
        sys.exit('장 수가 모자람(10 장 미만)')
    if size != (1280, 720):
        print('경고: 1280x720 이 아님', size)

    # 첫 맞춤에서 크게 벗어난 장을 뺀다(최대 20 %)
    _, K, D = calibrate(views, size, 0)
    frame_rms = np.array([np.sqrt((reproj(v, K, D) ** 2).mean()) for v in views])
    limit = max(1.0, 3 * float(np.median(frame_rms)))
    order = np.argsort(-frame_rms)
    drop = {int(j) for j in order[:len(views) // 5] if frame_rms[j] > limit}
    removed = [views[j][2] for j in sorted(drop)]
    views = [v for j, v in enumerate(views) if j not in drop]
    print(f'벗어난 장 뺌: {removed} (문턱 {limit:.2f} px)')

    _, K_all, D_all = calibrate(views, size, 0)  # 아래 맞춤들의 초깃값
    held = views[::5]
    train = [v for j, v in enumerate(views) if j % 5]
    table = {}
    for name, flags in MODELS.items():
        rms, K, D = calibrate(train, size, flags, (K_all, D_all))
        table[name] = dict(train_rms=rms, held_rms=rms_of(held, K, D)[0], held_p99=rms_of(held, K, D)[1], fx=float(K[0, 0]))
        print(f"{name}: 학습 RMS {rms:.3f} · 맡겨 둔 장 RMS {table[name]['held_rms']:.3f} · p99 {table[name]['held_p99']:.3f} px · fx {K[0, 0]:.1f}")
    # 맡겨 둔 장에서 10 % 넘게 좋을 때만 계수가 많은 모델을 고른다
    pick = 'rational_polynomial' if table['rational_polynomial']['held_rms'] < 0.9 * table['plumb_bob']['held_rms'] else 'plumb_bob'
    rms, K, D = calibrate(views, size, MODELS[pick], (K_all, D_all))
    med, p99 = straightness(views, K, D)
    cov = coverage(views, size)
    fx_s = fx_sigma(views, size, MODELS[pick])
    tilt = tilts_deg(views, K, D)
    tilted = float((tilt >= 15).mean())
    hfov = 2 * np.degrees(np.arctan(size[0] / 2 / K[0, 0]))
    write_yaml(out / 'arm_camera_1280x720.yaml', size, K, D, pick)
    rep = dict(views=len(views), removed=removed, model=pick, rms=float(rms), models=table, K=K.ravel().tolist(), D=D.ravel().tolist(),
               hfov_deg=float(hfov), straight_med_px=med, straight_p99_px=p99, coverage=cov,
               fx_sigma_px=fx_s, tilt_med_deg=float(np.median(tilt)), tilt_max_deg=float(tilt.max()), tilted_ge15_ratio=tilted)
    (out / 'report.json').write_text(json.dumps(rep, ensure_ascii=False, indent=1))
    print(f'고른 모델 {pick}, {len(views)} 장, RMS {rms:.3f} px, fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}, 가로 화각 {hfov:.1f}°')
    print(f'왜곡 편 줄 잔차 중앙 {med:.3f} · p99 {p99:.3f} px, 화면 덮음 {cov * 100:.0f} %')
    print(f'fx 표준편차 {fx_s:.2f} px ({fx_s / K[0, 0] * 100:.2f} %), 판 기울기 중앙 {np.median(tilt):.1f}° · 최대 {tilt.max():.1f}°, 15° 이상 장 {tilted * 100:.0f} %')
    checks = {'RMS ≤ 1 px(맡겨 둔 장도)': rms <= 1 and table[pick]['held_rms'] <= 1, '화면 덮음 ≥ 70 %': cov >= .7,
              'fx 표준편차 ≤ 1 %': fx_s / K[0, 0] <= .01, '15° 이상 기운 장 ≥ 25 %': tilted >= .25}
    for k, v in checks.items():
        print('판정:', k, '통과' if v else '불통과')
    # 눈으로 볼 그림: 왼쪽 원본, 오른쪽 왜곡 편 영상
    src = cv2.imread(str(a.folder / views[len(views) // 2][2]))
    newK = cv2.getOptimalNewCameraMatrix(K, D, size, 0.0)[0]
    und = cv2.undistort(src, K, D, None, newK)
    cv2.imwrite(str(out / 'undistort_check.jpg'), np.hstack([src, und]), [cv2.IMWRITE_JPEG_QUALITY, 85])
    print('결과:', out)


if __name__ == '__main__':
    main()
