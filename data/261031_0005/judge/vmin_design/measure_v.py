#!/usr/bin/env python3
"""켜진 등 · 배경 화소의 HSV 분포 (오프라인, PC). 등 화소 = 켜짐 영상과 꺼짐 영상의 차이(채널 최대 |차| ≥ DIFF).
배경 = 네 영상 모두 등 화소가 아닌 곳(로봇 가림 포함). 배경은 팀 문턱의 S · H 조건을 통과하는 화소의 V 를 본다.
  measure_v.py <렌더 폴더> [<렌더 폴더> ...] --out <json>"""
import argparse, json, pathlib
import cv2, numpy as np

DIFF = 40
S_MIN = 90
H = {'red': lambda h: (h <= 10) | (h >= 170), 'yellow': lambda h: (h >= 15) & (h <= 35), 'green': lambda h: (h >= 40) & (h <= 90)}
PCT = [0, 1, 5, 10, 25, 50, 75, 90, 95, 99, 100]

def pct(a):
    return {str(p): (float(np.percentile(a, p)) if a.size else None) for p in PCT}

ap = argparse.ArgumentParser()
ap.add_argument('renders', nargs='+')
ap.add_argument('--out', required=True)
a = ap.parse_args()
res = {}
for r in a.renders:
    r = pathlib.Path(r)
    lamp = {c: {'h': [], 's': [], 'v': []} for c in H}
    lamp_px = {c: [] for c in H}                       # 영상마다 등 화소 수 · S·V 문턱 통과 수(후보 기준 v 별)
    bg_v = {c: [] for c in H}                           # 배경에서 S ≥ s_min · H 범위인 화소 V
    bg_max_per_img = {c: [] for c in H}
    for d in sorted(p for p in r.iterdir() if p.is_dir()):
        imgs = {k: cv2.imread(str(d / f'{k}.png')) for k in ('off', 'red', 'yellow', 'green')}
        if any(v is None for v in imgs.values()):
            continue
        off = imgs['off'].astype(np.int16)
        masks = {}
        for c in H:
            masks[c] = (np.abs(imgs[c].astype(np.int16) - off).max(axis=2) >= DIFF)
        anylamp = masks['red'] | masks['yellow'] | masks['green']
        anylamp = cv2.dilate(anylamp.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        for c in H:
            hsv = cv2.cvtColor(imgs[c], cv2.COLOR_BGR2HSV)
            m = masks[c]
            for i, k in enumerate('hsv'):
                lamp[c][k].append(hsv[..., i][m])
            hh, ss, vv = hsv[..., 0][m], hsv[..., 1][m], hsv[..., 2][m]
            ok = ss >= S_MIN
            lamp_px[c].append({'name': d.name, 'n': int(m.sum()),
                               'n_hs': int((ok & H[c](hh)).sum()),
                               'v_hs': [int(x) for x in np.sort(vv[ok & H[c](hh)])[::-1][:20]]})
        for k, img in imgs.items():                     # 배경: 등 둘레를 뺀 모든 화소, 네 영상 모두
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            for c in H:
                m = ~anylamp & (hsv[..., 1] >= S_MIN) & H[c](hsv[..., 0])
                v = hsv[..., 2][m]
                bg_v[c].append(v)
                bg_max_per_img[c].append({'name': f'{d.name}/{k}', 'max_v': int(v.max()) if v.size else None, 'n_ge_100': int((v >= 100).sum())})
    out = {}
    for c in H:
        L = {k: np.concatenate(lamp[c][k]) for k in 'hsv'}
        sel = (L['s'] >= S_MIN) & H[c](L['h'])
        B = np.concatenate(bg_v[c])
        out[c] = {'lamp_all': {k: pct(L[k]) for k in 'hsv'},
                  'lamp_hs_ok_v': pct(L['v'][sel]), 'lamp_hs_ok_frac': float(sel.mean()),
                  'lamp_h_above_hmax': int(((L['h'] > 35) & (L['h'] < 40)).sum()) if c == 'yellow' else None,
                  # 영상별: S·H 를 통과한 등 화소 중 v ≥ 후보 인 화소 수의 최소(가장 어두운 장면)
                  'per_img_n_at_v': {str(v): int(min(sum(1 for x in p['v_hs'] if x >= v) if len(p['v_hs']) < 20 else 20 for p in lamp_px[c])) for v in (120, 130, 140, 150, 160, 180, 200)},
                  'per_img_lamp_vmax_min': int(min(p['v_hs'][0] if p['v_hs'] else -1 for p in lamp_px[c])),
                  'per_img_lamp_n_hs_min': int(min(p['n_hs'] for p in lamp_px[c])),
                  'bg_v': pct(B), 'bg_n': int(B.size),
                  'bg_n_ge': {str(v): int((B >= v).sum()) for v in (100, 120, 130, 140, 150, 160, 180, 200)},
                  'bg_worst': sorted([x for x in bg_max_per_img[c] if x['max_v'] is not None], key=lambda x: -x['max_v'])[:5]}
        if c == 'yellow':
            yh = L['h'][(L['s'] >= S_MIN)]
            out[c]['lamp_s_ok_h'] = pct(yh)
    res[str(r)] = out
json.dump(res, open(a.out, 'w'), indent=1, ensure_ascii=False)
for r, out in res.items():
    print('==', r)
    for c, o in out.items():
        print(f" {c}: 등 V(S·H 통과) p0/p5/p50={o['lamp_hs_ok_v']['0']}/{o['lamp_hs_ok_v']['5']}/{o['lamp_hs_ok_v']['50']}, 통과율 {o['lamp_hs_ok_frac']:.2f},"
              f" 영상별 등 최대 V 의 최소 {o['per_img_lamp_vmax_min']}, 영상별 통과 화소 최소 {o['per_img_lamp_n_hs_min']}, H{o['lamp_all']['h']['0']}~{o['lamp_all']['h']['100']} |"
              f" 배경 n={o['bg_n']} max={o['bg_v']['100']} ≥ {o['bg_n_ge']} worst={o['bg_worst'][:2]}")
