#!/usr/bin/env python3
"""Cartographer 실물 지도 ↔ 발제 도면 SDF 지도(map_amcl) 비교 (PC, 11-14).
  compare_sdf.py <carto map.yaml> <sdf map_amcl.yaml> <출력 png> [결과 json]
두 지도의 원점이 달라서(Cartographer 는 출발 자리), 실물 지도의 벽 칸을 SDF 지도 거리장에 맞추는 2D 자세(x · y · yaw)를
거친 격자 → 고운 격자로 찾은 뒤(잘라낸 거리 평균 최소), 그 자세에서
  ① 실물 벽 칸 → SDF 벽까지 거리(중앙값 · 90 % · 5 cm 안 비율 · 15 cm 넘는 비율 = SDF 에 없는 것: 상자 · 받침대 · 사람 등)
  ② SDF 벽 칸 중 실물 벽 5 cm 안에 있는 비율(주행으로 본 범위 안만: 실물 지도에서 아는 칸 0.3 m 안)
을 낸다. 판정 없음(관찰).
"""
import json
import math
import sys

import cv2
import numpy as np
import yaml


def load(path):
    y = yaml.safe_load(open(path))
    from pathlib import Path
    ip = Path(y['image'])
    img = cv2.imread(str(ip if ip.is_absolute() else Path(path).parent / ip), cv2.IMREAD_GRAYSCALE)
    return img, float(y['resolution']), [float(v) for v in y['origin'][:2]]


def cells_to_xy(rc, res, org, h):
    r, c = rc[:, 0], rc[:, 1]
    return np.stack([org[0] + (c + 0.5) * res, org[1] + (h - 1 - r + 0.5) * res], 1)


def main():
    cy, sy, out = sys.argv[1], sys.argv[2], sys.argv[3]
    jout = sys.argv[4] if len(sys.argv) > 4 else None
    cimg, cres, corg = load(cy)
    simg, sres, sorg = load(sy)
    H, W = simg.shape
    s_occ = simg < 100
    dist = cv2.distanceTransform((~s_occ).astype(np.uint8), cv2.DIST_L2, 5) * sres   # m
    clip = 0.20
    c_occ = np.argwhere(cimg < 100)            # 실물 벽 칸 (pbstream_to_ros_map: 벽 어두움 · 빈칸 254 · 모름 128)
    c_known = np.argwhere(cimg != 128)
    P = cells_to_xy(c_occ, cres, corg, cimg.shape[0])
    K = cells_to_xy(c_known, cres, corg, cimg.shape[0])
    rng = np.random.default_rng(0)
    Ps = P[rng.choice(len(P), min(400, len(P)), replace=False)]

    def score(pts, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        X = c * pts[:, 0] - s * pts[:, 1] + x
        Y = s * pts[:, 0] + c * pts[:, 1] + y
        col = np.floor((X - sorg[0]) / sres).astype(int)
        row = (H - 1 - np.floor((Y - sorg[1]) / sres)).astype(int)
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        d = np.full(len(pts), clip)
        d[ok] = np.minimum(dist[row[ok], col[ok]], clip)
        return d.mean()

    cx, cyy = P.mean(0)
    xs = np.arange(sorg[0], sorg[0] + W * sres, 0.10)
    ys = np.arange(sorg[1], sorg[1] + H * sres, 0.10)
    best = (9, 0, 0, 0)
    Pc = Ps - [cx, cyy]                         # 실물 벽 무게중심 기준으로 돌림
    for yaw in np.radians(np.arange(0, 360, 3)):
        for x in xs:
            for y in ys:
                v = score(Pc, x, y, yaw)
                if v < best[0]:
                    best = (v, x, y, yaw)
    _, x0, y0, a0 = best
    for step, astep in ((0.03, math.radians(1.0)), (0.01, math.radians(0.3)), (0.004, math.radians(0.1))):
        improved = True
        while improved:
            improved = False
            for dx, dy, da in ((step, 0, 0), (-step, 0, 0), (0, step, 0), (0, -step, 0), (0, 0, astep), (0, 0, -astep)):
                v = score(Pc, x0 + dx, y0 + dy, a0 + da)
                if v < best[0] - 1e-9:
                    best = (v, x0 + dx, y0 + dy, a0 + da)
                    x0, y0, a0 = best[1:]
                    improved = True
    # 전체 점으로 결과
    c, s = math.cos(a0), math.sin(a0)
    def tf(pts):
        q = pts - [cx, cyy]
        return np.stack([c * q[:, 0] - s * q[:, 1] + x0, s * q[:, 0] + c * q[:, 1] + y0], 1)
    PT = tf(P)
    col = np.floor((PT[:, 0] - sorg[0]) / sres).astype(int)
    row = (H - 1 - np.floor((PT[:, 1] - sorg[1]) / sres)).astype(int)
    ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    d1 = np.full(len(PT), np.inf)
    d1[ok] = dist[row[ok], col[ok]]
    # ② SDF 벽 → 실물 벽 (본 범위 안)
    cimg_T = np.zeros_like(simg, dtype=np.uint8)
    cr = row[ok]; cc = col[ok]
    cimg_T[cr, cc] = 1
    dist_c = cv2.distanceTransform((1 - cimg_T).astype(np.uint8), cv2.DIST_L2, 5) * sres
    KT = tf(K)
    kc = np.floor((KT[:, 0] - sorg[0]) / sres).astype(int)
    kr = (H - 1 - np.floor((KT[:, 1] - sorg[1]) / sres)).astype(int)
    kk = (kc >= 0) & (kc < W) & (kr >= 0) & (kr < H)
    seen = np.zeros_like(simg, dtype=np.uint8)
    seen[kr[kk], kc[kk]] = 1
    seen = cv2.dilate(seen, np.ones((int(0.6 / sres) | 1,) * 2, np.uint8))
    s_rc = np.argwhere(s_occ & (seen > 0))
    d2 = dist_c[s_rc[:, 0], s_rc[:, 1]]
    res = dict(
        pose_carto_to_sdf=dict(x=round(x0, 3), y=round(y0, 3), yaw_deg=round(math.degrees(a0) % 360, 2),
                               about_carto_wall_centroid=[round(cx, 3), round(cyy, 3)]),
        mean_clipped_dist_m=round(best[0], 4),
        carto_wall_cells=int(len(P)),
        carto_to_sdf=dict(median_m=round(float(np.median(d1[np.isfinite(d1)])), 4),
                          p90_m=round(float(np.percentile(d1[np.isfinite(d1)], 90)), 4),
                          within_5cm=round(float(np.mean(d1 <= 0.05)), 3),
                          over_15cm=round(float(np.mean(d1 > 0.15)), 3),
                          outside_sdf_map=round(float(np.mean(~ok)), 3)),
        sdf_wall_cells_in_seen_area=int(len(s_rc)),
        sdf_to_carto=dict(median_m=round(float(np.median(d2)), 4), within_5cm=round(float(np.mean(d2 <= 0.05)), 3),
                          over_15cm=round(float(np.mean(d2 > 0.15)), 3)),
    )
    print(json.dumps(res, ensure_ascii=False, indent=1))
    if jout:
        json.dump(res, open(jout, 'w'), ensure_ascii=False, indent=1)
    # 그림: 흰 바탕, SDF 벽 회색, 실물 벽 (5 cm 안 초록 · 5~15 cm 주황 · 15 cm 넘음 빨강)
    sc = 4
    vis = np.full((H, W, 3), 255, np.uint8)
    vis[s_occ] = (150, 150, 150)
    vis[(seen == 0) & ~s_occ] = (235, 235, 235)
    for (r_, c_), d in zip(zip(row[ok], col[ok]), d1[ok]):
        vis[r_, c_] = (0, 170, 0) if d <= 0.05 else ((0, 140, 255) if d <= 0.15 else (0, 0, 230))
    vis = cv2.resize(vis, (W * sc, H * sc), interpolation=cv2.INTER_NEAREST)
    txt = (f"carto->SDF median {res['carto_to_sdf']['median_m']*100:.1f} cm, <=5cm {res['carto_to_sdf']['within_5cm']*100:.0f}%, "
           f">15cm {res['carto_to_sdf']['over_15cm']*100:.0f}%")
    cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(vis, "gray=SDF wall  green<=5cm  orange 5-15cm  red>15cm (real map walls)", (8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(out, vis)


if __name__ == '__main__':
    main()
