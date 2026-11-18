#!/usr/bin/env python3
"""실물 팔 영상 바코드 읽기 시제품 (PC, 설계용 · 판정 아님). python3 proto_label.py <장 목록> [키=값 ...]

팀 BarcodeReader(전체 장 · 감마 2 · 회전 후보 ±30° · CODE128 · quality ≥ 4)에 앞서
'라벨 찾기' 단계를 둔다: 흰 라벨(밝은 볼록 사각형)을 찾아 → 긴 변이 가로가 되게 바로 세워 자르고 →
둘레에 흰 여백(조용한 구역)을 붙이고 → 라벨용 감마 → zbar (작은 회전 후보). 라벨에서 못 읽으면 결과 없음(여기서는 전체 장 길은 안 봄).
장 목록 한 줄: <이름> <png>. 이름 앞 5자가 P2-0x 면 정답으로 센다.
"""
import ctypes
import json
import sys
import time

import cv2
import numpy as np

P = dict(thr='rel', min_area=0.004, max_area=0.5, ar_min=1.8, ar_max=6.0, fill_min=0.75, max_labels=3,
         pad=0.35, gamma=0.5, angles='0,-4,4,-8,8', qmin=4, scale=1.0, inset=0.0, rel=0.8, close=0, persp=0)
for kv in sys.argv[2:]:
    k, v = kv.split('=')
    P[k] = v if k in ('thr', 'angles') else float(v)

z = ctypes.CDLL('libzbar.so.0')
for n, r in [('zbar_image_scanner_create', ctypes.c_void_p), ('zbar_image_create', ctypes.c_void_p), ('zbar_image_first_symbol', ctypes.c_void_p),
             ('zbar_symbol_next', ctypes.c_void_p), ('zbar_symbol_get_data', ctypes.c_char_p)]:
    getattr(z, n).restype = r
for n in ('zbar_image_first_symbol', 'zbar_symbol_next', 'zbar_symbol_get_data', 'zbar_symbol_get_quality', 'zbar_image_destroy'):
    getattr(z, n).argtypes = [ctypes.c_void_p]
z.zbar_image_scanner_set_config.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
z.zbar_image_set_format.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
z.zbar_image_set_size.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]
z.zbar_image_set_data.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p]
z.zbar_scan_image.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
SC = z.zbar_image_scanner_create()
z.zbar_image_scanner_set_config(SC, 0, 0, 0)      # 모두 끔
z.zbar_image_scanner_set_config(SC, 128, 0, 1)    # CODE128 켬 (ZBAR_CODE128 = 128)


def zscan(g):
    out = []
    im = z.zbar_image_create(); z.zbar_image_set_format(im, 0x30303859)
    g = np.ascontiguousarray(g); z.zbar_image_set_size(im, g.shape[1], g.shape[0])
    buf = g.tobytes(); z.zbar_image_set_data(im, buf, len(buf), None)
    z.zbar_scan_image(SC, im)
    s = z.zbar_image_first_symbol(im)
    while s:
        out.append((z.zbar_symbol_get_data(s).decode('latin1'), z.zbar_symbol_get_quality(s)))
        s = z.zbar_symbol_next(s)
    z.zbar_image_destroy(im)
    return out


LUT = np.round(255 * (np.arange(256) / 255.0) ** P['gamma']).astype(np.uint8)
ANG = [float(a) for a in P['angles'].split(',')]


def labels(gray):
    """밝은 볼록 사각형 후보 → minAreaRect 목록 (넓이 큰 순)"""
    b = cv2.GaussianBlur(gray, (5, 5), 0)
    if P['thr'] == 'otsu':
        _, bw = cv2.threshold(b, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    elif P['thr'] == 'rel':   # 가장 밝은 쪽(라벨 흰 종이) 기준: rel × 99.5 백분위
        _, bw = cv2.threshold(b, P['rel'] * float(np.percentile(b, 99.5)), 255, cv2.THRESH_BINARY)
    else:
        _, bw = cv2.threshold(b, float(P['thr']), 255, cv2.THRESH_BINARY)
    if P['close'] > 0:   # 막대 사이 검은 칸을 메워 라벨 하나로
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((int(P['close']), int(P['close'])), np.uint8))
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    A = gray.shape[0] * gray.shape[1]
    out = []
    for c in cnts:
        a = cv2.contourArea(c)
        if not (P['min_area'] * A <= a <= P['max_area'] * A):
            continue
        (cx, cy), (w, h), ang = cv2.minAreaRect(c)
        L, S = max(w, h), min(w, h)
        if S < 8 or not (P['ar_min'] <= L / S <= P['ar_max']) or a / (w * h) < P['fill_min']:
            continue
        quad = None
        if P['persp']:
            hull = cv2.convexHull(c)
            ap = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
            if len(ap) == 4:
                quad = ap.reshape(4, 2).astype(np.float32)
        out.append((a, ((cx, cy), (w, h), ang, quad)))
    out.sort(key=lambda t: -t[0])
    return [r for _, r in out[:int(P['max_labels'])]]


def upright(gray, rect):
    (cx, cy), (w, h), ang, quad = rect
    if quad is not None:   # 네 꼭짓점 → 직사각형 (원근 보정): 긴 변이 가로
        q = quad[np.argsort(np.arctan2(quad[:, 1] - quad[:, 1].mean(), quad[:, 0] - quad[:, 0].mean()))]   # 각도 순
        e = [np.linalg.norm(q[(k + 1) % 4] - q[k]) for k in range(4)]
        if e[0] + e[2] < e[1] + e[3]:
            q = np.roll(q, -1, axis=0); e = e[1:] + e[:1]
        W, H = int(round((e[0] + e[2]) / 2)), int(round((e[1] + e[3]) / 2))
        dst = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
        crop = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(q, dst), (W, H), flags=cv2.INTER_LINEAR, borderValue=255)
        if P['scale'] != 1.0:
            crop = cv2.resize(crop, None, fx=P['scale'], fy=P['scale'], interpolation=cv2.INTER_CUBIC)
        crop = cv2.LUT(crop, LUT)
        py, px = int(P['pad'] * crop.shape[0]) + 4, int(P['pad'] * crop.shape[1]) + 4
        return cv2.copyMakeBorder(crop, py, py, px, px, cv2.BORDER_CONSTANT, value=255)
    if w < h:
        w, h, ang = h, w, ang + 90
    M = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
    r = cv2.warpAffine(gray, M, (gray.shape[1], gray.shape[0]), flags=cv2.INTER_LINEAR, borderValue=255)
    i = P['inset']
    x0, x1 = int(round(cx - w / 2 * (1 - i))), int(round(cx + w / 2 * (1 - i)))
    y0, y1 = int(round(cy - h / 2 * (1 - i))), int(round(cy + h / 2 * (1 - i)))
    crop = r[max(0, y0):y1, max(0, x0):x1]
    if crop.size == 0:
        return None
    if P['scale'] != 1.0:
        crop = cv2.resize(crop, None, fx=P['scale'], fy=P['scale'], interpolation=cv2.INTER_CUBIC)
    crop = cv2.LUT(crop, LUT)
    py, px = int(P['pad'] * crop.shape[0]) + 4, int(P['pad'] * crop.shape[1]) + 4
    return cv2.copyMakeBorder(crop, py, py, px, px, cv2.BORDER_CONSTANT, value=255)


def read(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    syms, drop, nl = [], 0, 0
    for rect in labels(gray):
        lab = upright(gray, rect)
        if lab is None:
            continue
        nl += 1
        for a in ANG:
            s = lab
            if a:
                M = cv2.getRotationMatrix2D((lab.shape[1] / 2, lab.shape[0] / 2), a, 1.0)
                s = cv2.warpAffine(lab, M, (lab.shape[1], lab.shape[0]), borderValue=255)
            for t, q in zscan(s):
                if q >= P['qmin']:
                    syms.append(t)
                else:
                    drop += 1
    u = set(syms)
    return dict(ok=len(u) == 1, text=next(iter(u)) if len(u) == 1 else '', symbols=sorted(u), dropped=drop, labels=nl)


for line in open(sys.argv[1]):
    p = line.split()
    if len(p) != 2:
        continue
    t0 = time.perf_counter()
    r = read(cv2.imread(p[1]))
    r['ms'] = round((time.perf_counter() - t0) * 1000, 1)
    r['name'] = p[0]
    print(json.dumps(r, ensure_ascii=False))
