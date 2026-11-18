#!/usr/bin/env python3
"""신호등 실물 사진(cam1 D435) 측정 — 노출 비교 · 켜진 칸 높이로 색 정하기 설계 자료.

사진마다:
  1) 바닥 평면: 정렬 깊이(아래쪽 화면)에서 RANSAC 으로 수평에 가까운 평면 중 가장 낮은 것
  2) 현재 검출기(팀 perception signal_light.hpp detect_light_blobs, config/perception.yaml 값) 덩어리
  3) 넓은 밝은 덩어리(V >= v_lo): 색상과 상관없이 '켜진 것' 후보 — 위치(높이)로 칸 번호를 정하는 설계용
  4) 덩어리마다 깊이(덩어리를 3 px 넓힌 영역의 유효 깊이 중앙값)와 바닥 위 높이 [m]
출력: results/measure.json (사진별 덩어리 목록)
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np

CAM = 'cam1_261822072247'
# perception.yaml signal 기본값 (2026-10-26 feat/perception-barcode)
CUR = dict(v_min=200, s_min=90, red_h_max=10, red2_h_min=170, yellow_h_min=15, yellow_h_max=35,
           green_h_min=40, green_h_max=90, min_area_px=4)


def hue_name(h):
    c = CUR
    if h <= c['red_h_max'] or h >= c['red2_h_min']:
        return 'R'
    if c['yellow_h_min'] <= h <= c['yellow_h_max']:
        return 'Y'
    if c['green_h_min'] <= h <= c['green_h_max']:
        return 'G'
    return '-'


def current_blobs(hsv):
    c = CUR
    h, s, v = cv2.split(hsv)
    bright = (s >= c['s_min']) & (v >= c['v_min'])
    masks = {'R': ((h <= c['red_h_max']) | (h >= c['red2_h_min'])) & bright,
             'Y': (h >= c['yellow_h_min']) & (h <= c['yellow_h_max']) & bright,
             'G': (h >= c['green_h_min']) & (h <= c['green_h_max']) & bright}
    out = []
    for col, m in masks.items():
        n, lab, st, ce = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
        for i in range(1, n):
            if st[i, 4] < c['min_area_px']:
                continue
            out.append(dict(color=col, area=int(st[i, 4]), box=[int(x) for x in st[i, :4]],
                            cx=float(ce[i, 0]), cy=float(ce[i, 1]), mask=(lab == i)))
    return out


def bright_blobs(hsv, v_lo, min_area):
    """색상과 상관없이 밝은 덩어리. 색은 덩어리 둘레(채도 있는 화소)의 색상 중앙값으로 기록만 함"""
    h, s, v = cv2.split(hsv)
    m = (v >= v_lo).astype(np.uint8)
    n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        if st[i, 4] < min_area:
            continue
        mk = lab == i
        ring = cv2.dilate(mk.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
        sat = ring & (s >= 60)
        hh = h[sat]
        # 빨강은 0/180 을 넘나들므로 원형 중앙값 대신 180 근처를 음수로 접어 중앙값
        hmed = None
        if hh.size:
            hf = np.where(hh > 150, hh.astype(int) - 180, hh.astype(int))
            hmed = int(np.median(hf)) % 180
        out.append(dict(area=int(st[i, 4]), box=[int(x) for x in st[i, :4]], cx=float(ce[i, 0]), cy=float(ce[i, 1]),
                        v255=float(np.mean(v[mk] >= 250)), s_mean=float(np.mean(s[mk])),
                        hue_ring=hmed, hue_class=None if hmed is None else hue_name(hmed), mask=mk))
    return out


def deproject(u, v, z, K):
    return np.array([(u - K['ppx']) / K['fx'] * z, (v - K['ppy']) / K['fy'] * z, z])


def floor_plane(depth_m, K, rng, iters=400):
    H, W = depth_m.shape
    vv, uu = np.mgrid[H // 2:H:4, 0:W:4]
    z = depth_m[H // 2:H:4, 0:W:4]
    ok = (z > 0.15) & (z < 4.0)
    P = np.stack([(uu[ok] - K['ppx']) / K['fx'] * z[ok], (vv[ok] - K['ppy']) / K['fy'] * z[ok], z[ok]], 1)
    if len(P) < 200:
        return None
    best = None
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        n = np.cross(P[idx[1]] - P[idx[0]], P[idx[2]] - P[idx[0]])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if n[1] > 0:          # 위쪽(카메라 -y 쪽)을 향하게
            n = -n
        if -n[1] < 0.6:       # 수평 면만 (광학 y 와 53° 안)
            continue
        d = -n @ P[idx[0]]
        inl = np.abs(P @ n + d) < 0.01
        cnt = int(inl.sum())
        # 같은 수평 면 후보 중 아래쪽(바닥)을 고르기 위해: 안쪽 점 수 + 낮을수록 우선
        if best is None or cnt > best[0] * 1.0:
            best = (cnt, n, d)
    if best is None:
        return None
    # 최소 제곱 다듬기
    cnt, n, d = best
    inl = np.abs(P @ n + d) < 0.01
    Q = P[inl]
    c = Q.mean(0)
    _, _, vt = np.linalg.svd(Q - c)
    n2 = vt[2]
    if n2[1] > 0:
        n2 = -n2
    return dict(n=n2.tolist(), d=float(-n2 @ c), inliers=int(inl.sum()), frac=float(inl.mean()))


def blob_depth(depth_m, mk, grow=3):
    k = np.ones((2 * grow + 1, 2 * grow + 1), np.uint8)
    reg = cv2.dilate(mk.astype(np.uint8), k).astype(bool)
    z = depth_m[reg]
    z = z[(z > 0.1) & (z < 6)]
    zin = depth_m[mk]
    return (float(np.median(z)) if z.size else None, float(np.mean((zin > 0.1) & (zin < 6))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--photos', default=os.path.expanduser('~/git/lv2_next_module/cartlidar/photo'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--v-lo', type=int, default=180)
    ap.add_argument('--min-area', type=int, default=4)
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    res = []
    for i, d in enumerate(sorted(glob.glob(os.path.join(a.photos, '20261026_1*')))):
        cd = os.path.join(d, CAM)
        meta = json.load(open(os.path.join(cd, 'metadata.json')))
        K = meta['color_intrinsics']
        bgr = cv2.imread(os.path.join(cd, 'color.png'))
        dep = cv2.imread(os.path.join(cd, 'depth_aligned.png'), -1).astype(np.float32) * meta['depth_unit_m']
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        rgbopt = meta.get('sensor_options_at_receive', {}).get('RGB Camera', {})
        fp = floor_plane(dep, K, rng)
        rec = dict(index=i, capture=os.path.basename(d), auto_exposure=rgbopt.get('enable_auto_exposure'),
                   exposure=rgbopt.get('exposure'), gain=rgbopt.get('gain'), floor=fp, current=[], bright=[])
        for key, blobs in (('current', current_blobs(hsv)), ('bright', bright_blobs(hsv, a.v_lo, a.min_area))):
            for b in blobs:
                z, valid = blob_depth(dep, b['mask'])
                b.pop('mask')
                b['depth_m'] = z
                b['depth_valid_in_blob'] = valid
                b['height_m'] = None
                if z is not None and fp is not None:
                    p = deproject(b['cx'], b['cy'], z, K)
                    b['height_m'] = float(np.dot(fp['n'], p) + fp['d'])
                rec[key].append(b)
        res.append(rec)
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    json.dump(res, open(a.out, 'w'), ensure_ascii=False, indent=1)
    print(len(res), 'photos ->', a.out)


if __name__ == '__main__':
    main()
