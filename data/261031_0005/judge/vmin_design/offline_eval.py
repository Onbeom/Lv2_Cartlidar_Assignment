#!/usr/bin/env python3
"""신호등 후보 문턱 오프라인 비교 (PC, 모의 렌더). 팀 detect_light_blobs + 노드의 크기 · 높이 거르기를 그대로 흉내 내고,
켜진 칸이 있는 자리(세 칸 가운데 노랑 기준 0.10 m 안)에서 가장 큰 후보의 색을 그 영상의 상태로 본다(R1′ 고정 뒤 판정과 같음).
자리 밖에서 거르기를 통과한 후보는 '헛 후보'로 센다(고정되려면 점멸이 필요하므로 판정 실패는 아님 — 여유 확인용).
  offline_eval.py --cfg '<json>' <렌더 폴더> ..."""
import argparse, json, pathlib
import cv2, numpy as np

BASE = dict(v_min=200, s_min=90, red_h_max=10, red2_h_min=170, yellow_h_min=15, yellow_h_max=35,
            green_h_min=40, green_h_max=90, min_area_px=4)
SEG_W, SEG_H, RMIN, RMAX, ZMIN, ZMAX, LOCK = 0.0396, 0.0349, 0.15, 2.5, 0.05, 0.20, 0.10

def blobs(bgr, c):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    bright = (s >= c['s_min']) & (v >= c['v_min'])
    yb = (s >= c['s_min']) & (v >= c.get('yellow_v_min', c['v_min']))
    red = ((h <= c['red_h_max']) | (h >= c['red2_h_min'])) & (s >= c['s_min']) & (v >= c.get('red_v_min', c['v_min']))
    if c.get('red_warm_h_min', 0) > 0:          # 따뜻한 조명 AWB 아래 자홍으로 보이는 빨강 LED (다른 밝기 문턱)
        red |= (h >= c['red_warm_h_min']) & (h < c['red2_h_min']) & (s >= c['s_min']) & (v >= c['red_warm_v_min'])
    masks = {'red': red,
             'yellow': (h >= c['yellow_h_min']) & (h <= c['yellow_h_max']) & yb,
             'green': (h >= c['green_h_min']) & (h <= c['green_h_max']) & bright}
    for col, m in masks.items():
        n, lab, st, ce = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] >= c['min_area_px']:
                yield col, lab == i, st[i, cv2.CC_STAT_AREA], ce[i]

def run(render, c):
    r = pathlib.Path(render)
    meta = json.load(open(r / 'render_r12.json'))
    K = meta['K']
    poses = {p['name']: p for p in meta['poses']}
    ok = tot = fp_site = fp_out = 0
    conf, worst = {}, []
    for name, p in sorted(poses.items()):
        d = r / name
        Twc = np.array(p['T_world_cam'])
        site = np.array(p['segments']['yellow']['world'])
        for st in ('off', 'red', 'yellow', 'green'):
            bgr = cv2.imread(str(d / f'{st}.png'))
            dep = np.load(d / f'{st}_depth.npy')
            best = None
            for col, m, area, ctr in blobs(bgr, c):
                z = dep[m]
                z = z[np.isfinite(z) & (z >= 0.15) & (z <= 4.0)]
                if not z.size:
                    continue
                z = float(np.median(z))
                exp = (SEG_W * K['fx'] / z) * (SEG_H * K['fy'] / z)
                if area < RMIN * exp or area > RMAX * exp:
                    continue
                u, w = round(ctr[0]), round(ctr[1])
                pc = np.array([(u - K['cx']) / K['fx'] * z, (w - K['cy']) / K['fy'] * z, z, 1.0])
                pw = (Twc @ pc)[:3]
                if not (ZMIN <= pw[2] <= ZMAX):
                    continue
                if np.linalg.norm(pw - site) < LOCK:
                    if best is None or area > best[1]:
                        best = (col, area)
                else:
                    fp_out += 1
                    worst.append(f'{name}/{st}:{col}@{pw.round(2).tolist()} a{area}')
            got = best[0] if best else 'off'
            tot += 1
            ok += got == st
            if st == 'off' and best:
                fp_site += 1
            k = f'{st}->{got}'
            conf[k] = conf.get(k, 0) + 1
    return dict(render=str(r), ok=ok, tot=tot, off_lit=fp_site, outside_candidates=fp_out,
                confusion={k: v for k, v in sorted(conf.items()) if k.split('->')[0] != k.split('->')[1]}, outside_examples=worst[:6])

ap = argparse.ArgumentParser()
ap.add_argument('--cfg', default='{}')
ap.add_argument('renders', nargs='+')
a = ap.parse_args()
c = dict(BASE, **json.loads(a.cfg))
for R in a.renders:
    print(json.dumps(run(R, c), ensure_ascii=False))
