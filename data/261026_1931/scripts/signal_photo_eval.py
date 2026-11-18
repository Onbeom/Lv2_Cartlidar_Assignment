#!/usr/bin/env python3
"""신호등 실물 사진 측정 M1~M5 (신호등_실물설계.md). 정답 scripts/labels_eye.json, 신호등 영역 scripts/tower_boxes.json.

M1 현재 검출기 색: 신호등 영역(사각형 + 8 px) 안 덩어리 중 가장 큰 것의 색, 없으면 off
M2 꺼짐 사진의 신호등 영역 덩어리
M3 신호등 영역 밖 덩어리 수
M4 위치 규칙: 영역 안 V >= v_lo 인 가장 큰 덩어리 중심 높이 − 신호등이 선 면 높이 → 칸 구간
    깊이 = 영역 안 유효 깊이의 20 백분위(신호등이 영역에서 가장 가까운 물체), 선 면 = 그 깊이 ±0.05 m 인
    영역 아래쪽(+80 px까지) 화소 높이의 2 백분위
M5 켜진 칸(현재 검출기 가장 큰 덩어리 → 없으면 밝은 덩어리) 화소 중 깊이 있는 비율
"""
import argparse
import collections
import csv
import glob
import json
import os

import cv2
import numpy as np

from signal_photo_measure import CAM, bright_blobs, current_blobs, floor_plane

HERE = os.path.dirname(os.path.abspath(__file__))
BANDS = [('G', 0.0689, 0.1038), ('Y', 0.1038, 0.1387), ('R', 0.1387, 0.1736)]   # 사용자 실측 2026-10-23
SEG_W, SEG_H = 0.0396, 0.0349


def in_box(b, box, m=8):
    x0, y0, x1, y1 = box
    return x0 - m <= b['cx'] <= x1 + m and y0 - m <= b['cy'] <= y1 + m


def height(n, d, u, v, z, K):
    p = np.array([(u - K['ppx']) / K['fx'] * z, (v - K['ppy']) / K['fy'] * z, z])
    return float(np.dot(n, p) + d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--photos', default=os.path.expanduser('~/git/lv2_next_module/cartlidar/photo'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--v-lo', type=int, default=180)
    a = ap.parse_args()
    labels = json.load(open(os.path.join(HERE, 'labels_eye.json')))['labels']
    groups = json.load(open(os.path.join(HERE, 'tower_boxes.json')))['groups']
    rng = np.random.default_rng(0)
    rows = []
    for i, d in enumerate(sorted(glob.glob(os.path.join(a.photos, '20261026_1*')))):
        cap = os.path.basename(d)
        cd = os.path.join(d, CAM)
        meta = json.load(open(os.path.join(cd, 'metadata.json')))
        K = meta['color_intrinsics']
        bgr = cv2.imread(os.path.join(cd, 'color.png'))
        dep = cv2.imread(os.path.join(cd, 'depth_aligned.png'), -1).astype(np.float32) * meta['depth_unit_m']
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        opt = meta.get('sensor_options_at_receive', {}).get('RGB Camera', {})
        expo = 'manual30' if opt.get('enable_auto_exposure') == 0.0 else 'auto'
        box = [b for s, e, b in groups if s <= i <= e][0]
        x0, y0, x1, y1 = box
        fp = floor_plane(dep, K, rng)
        cur = current_blobs(hsv)
        cin = [b for b in cur if in_box(b, box)]
        cout = [b for b in cur if not in_box(b, box)]
        m1 = max(cin, key=lambda b: b['area'])['color'] if cin else 'off'
        bb = [b for b in bright_blobs(hsv, a.v_lo, 4) if in_box(b, box)]
        top = max(bb, key=lambda b: b['area']) if bb else None
        # 신호등 깊이 · 선 면
        zb = dep[y0:y1, x0:x1]
        zb = zb[(zb > 0.1) & (zb < 6)]
        ztow = float(np.percentile(zb, 20)) if zb.size > 20 else None
        base = None
        if fp is not None and ztow is not None:
            n, dd = np.array(fp['n']), fp['d']
            ys, xs = np.mgrid[y0:min(y1 + 80, 480), max(x0 - 5, 0):min(x1 + 5, 640)]
            zz = dep[ys, xs]
            ok = np.abs(zz - ztow) < 0.05
            if ok.sum() > 20:
                P = np.stack([(xs[ok] - K['ppx']) / K['fx'] * zz[ok], (ys[ok] - K['ppy']) / K['fy'] * zz[ok], zz[ok]], 1)
                base = float(np.percentile(P @ n + dd, 2))
        rel = seg = area_ratio = None
        if top is not None and ztow is not None:
            exp_area = (SEG_W * K['fx'] / ztow) * (SEG_H * K['fy'] / ztow)
            area_ratio = top['area'] / exp_area
            if fp is not None and base is not None:
                rel = height(np.array(fp['n']), fp['d'], top['cx'], top['cy'], ztow, K) - base
                seg = next((c for c, lo, hi in BANDS if lo <= rel < hi), 'out')
        # M5: 켜진 칸 덩어리 깊이 유효 비율
        lit = max(cin, key=lambda b: b['area']) if cin else None
        m5 = None
        if lit is not None:
            zin = dep[lit['mask']]
            m5 = float(np.mean((zin > 0.1) & (zin < 6)))
        rows.append(dict(index=i, capture=cap, exposure=expo, label=labels[cap], m1=m1, n_in=len(cin), n_out=len(cout),
                         out_colors=''.join(sorted(b['color'] for b in cout)), bright_area=None if top is None else top['area'],
                         bright_area_ratio=area_ratio, bright_v255=None if top is None else top['v255'],
                         tower_depth_m=ztow, floor_ok=fp is not None, base_m=base, rel_h_m=rel, seg=seg, lit_depth_valid=m5))
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'per_photo.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    # 요약
    S = []
    for ex in ('manual30', 'auto'):
        R = [r for r in rows if r['exposure'] == ex]
        litR = [r for r in R if r['label'] != 'off']
        offR = [r for r in R if r['label'] == 'off']
        S.append(f'## {ex}: 사진 {len(R)} (켜짐 {len(litR)}, 꺼짐 {len(offR)})')
        for c in 'RYG':
            rc = [r for r in litR if r['label'] == c]
            if rc:
                S.append(f'M1 {c}: 맞음 {sum(r["m1"] == c for r in rc)}/{len(rc)}  나온 색 {dict(collections.Counter(r["m1"] for r in rc))}')
        S.append(f'M2 꺼짐인데 영역 안 덩어리: {sum(r["n_in"] > 0 for r in offR)}/{len(offR)}  나온 색 {dict(collections.Counter(r["m1"] for r in offR))}')
        S.append(f'M3 영역 밖 덩어리: 켜짐 사진 한 장당 평균 {np.mean([r["n_out"] for r in litR]):.2f}, 1개 이상 {sum(r["n_out"] > 0 for r in litR)}/{len(litR)}; 꺼짐 사진 평균 {np.mean([r["n_out"] for r in offR]) if offR else 0:.2f}')
        ev = [r for r in litR if r['seg'] is not None]
        S.append(f'M4 위치 규칙 칸 맞음: {sum(r["seg"] == r["label"] for r in ev)}/{len(ev)} (높이 못 잼 {len(litR) - len(ev)})')
        for c, lo, hi in BANDS:
            hs = [r['rel_h_m'] for r in ev if r['label'] == c]
            if hs:
                S.append(f'   {c} 칸({lo*1000:.1f}~{hi*1000:.1f} mm) 잰 높이 {1000*min(hs):.1f}~{1000*max(hs):.1f} mm, 중앙 {1000*np.median(hs):.1f}')
        ar_l = [r['bright_area_ratio'] for r in litR if r['bright_area_ratio'] is not None]
        ar_o = [r['bright_area_ratio'] if r['bright_area_ratio'] is not None else 0 for r in offR]
        S.append(f'   밝은 덩어리 면적 / 칸 한 개 면적: 켜짐 최소 {min(ar_l) if ar_l else None}, 꺼짐 최대 {max(ar_o) if ar_o else None}')
        m5 = [r['lit_depth_valid'] for r in litR if r['lit_depth_valid'] is not None]
        if m5:
            S.append(f'M5 켜진 칸 깊이 유효 비율: 중앙 {100*np.median(m5):.0f} %, 범위 {100*min(m5):.0f}~{100*max(m5):.0f} % ({len(m5)}장)')
    open(os.path.join(a.out, 'summary.txt'), 'w').write('\n'.join(S) + '\n')
    print('\n'.join(S))


if __name__ == '__main__':
    main()
