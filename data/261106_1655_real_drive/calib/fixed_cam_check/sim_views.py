#!/usr/bin/env python3
"""도구 확인(모의, 코너 좌표만, 렌더 아님): 카메라가 고정일 때 보정판을 어떻게 옮겨야 f 를 구하나.
값 = 8 번 되풀이의 |fx 오차| p50 · p95 %, 왜곡은 make_synth 와 같은 값, 코너 잡음 0.3 px(가정).
"""
import cv2
import numpy as np

K0 = np.array([[430., 0, 655], [0, 432., 352], [0, 0, 1]])
D0 = np.array([-0.12, 0.012, 0.0006, -0.0004, 0.0])
W, H = 1280, 720
gx, gy = np.meshgrid(np.arange(1, 10), np.arange(1, 7))
OBJ = np.stack([gx.ravel(), gy.ravel(), 0 * gx.ravel()], 1).astype(np.float32)  # 칸 단위


def view(R, t, rng, obj=OBJ):
    p, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K0, D0)
    p = p.reshape(-1, 2) + rng.normal(0, .3, (len(obj), 2))
    m = (p[:, 0] > 2) & (p[:, 0] < W - 2) & (p[:, 1] > 2) & (p[:, 1] < H - 2)
    return (obj[m], p[m].astype(np.float32)) if m.sum() >= 12 else None


def eul(a, b):
    return cv2.Rodrigues(np.array([a, 0, 0.]))[0] @ cv2.Rodrigues(np.array([0, b, 0.]))[0]


def fit(vs):
    o, i = [v[0] for v in vs], [v[1] for v in vs]
    try:
        _, K, _, _, _ = cv2.calibrateCamera(o, i, (W, H), None, None)
    except cv2.error:
        return np.nan
    return abs(K[0, 0] / K0[0, 0] - 1) * 100


def moving(rng, n, tilt, zs=(8., 11., 15.)):
    """보정판을 옮김: 자리 · 거리 · 기울기가 장마다 다름"""
    vs = []
    while len(vs) < n:
        z = rng.choice(zs)
        a, b = np.radians(rng.uniform(-tilt, tilt, 2))
        v = view(eul(a, b), np.array([rng.uniform(-.55, .55) * z - 5, rng.uniform(-.35, .35) * z - 3.5, z]), rng)
        if v:
            vs.append(v)
    return vs


def same_plane(rng, n, tilt):
    """고정 모니터: 같은 평면 위에서 표시 자리 · 크기만 바꿈(카메라 · 모니터 안 움직임)"""
    R = eul(np.radians(tilt * .6), np.radians(tilt))
    vs = []
    while len(vs) < n:
        s = rng.uniform(.5, 1.)
        off = rng.uniform([-6, -4], [6, 4])
        obj = OBJ.copy()
        pl = OBJ.copy(); pl[:, 0] = (OBJ[:, 0] - 5) * s + off[0]; pl[:, 1] = (OBJ[:, 1] - 3.5) * s + off[1]
        p, _ = cv2.projectPoints(pl, cv2.Rodrigues(R)[0], np.array([0, 0, 11.]), K0, D0)
        p = p.reshape(-1, 2) + rng.normal(0, .3, (len(pl), 2))
        m = (p[:, 0] > 2) & (p[:, 0] < W - 2) & (p[:, 1] > 2) & (p[:, 1] < H - 2)
        if m.sum() >= 12:
            vs.append((obj[m], p[m].astype(np.float32)))
    return vs


cv2.setNumThreads(4)
rows = [('고정 카메라 + 고정 평면 40 장, 평면 기울기 0°', lambda r: same_plane(r, 40, 0)),
        ('고정 카메라 + 고정 평면 40 장, 평면 기울기 20°', lambda r: same_plane(r, 40, 20)),
        ('판을 옮김 40 장, 기울기 ±5°', lambda r: moving(r, 40, 5)),
        ('판을 옮김 40 장, 기울기 ±15°', lambda r: moving(r, 40, 15)),
        ('판을 옮김 40 장, 기울기 ±30°', lambda r: moving(r, 40, 30)),
        ('판을 옮김 15 장, 기울기 ±30°', lambda r: moving(r, 15, 30))]
for name, g in rows:
    e = [fit(g(np.random.default_rng(s))) for s in range(8)]
    e = np.array(e, float)
    print(f'{name}: p50 {np.nanmedian(e):.2f} % · p95 {np.nanpercentile(e, 95):.2f} % (실패 {int(np.isnan(e).sum())}/8)')
