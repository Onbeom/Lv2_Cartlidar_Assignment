"""P8 색온도 확인 (도구 확인, 모의): 같은 화면을 색온도만 바꿔 렌더한 것에 팀 HSV 범위를 대 본다.

기준 = 색온도 끔(흰빛, 지금까지의 월드). 색마다
  유지율   = 기준에서 그 색 범위였던 화소 중 T 에서도 그 범위인 비율
  새 화소  = 기준에서는 아니었는데 T 에서 그 범위가 된 화소 수(헛검출 재료)
  H 중앙값 = 기준에서 그 색이던 화소 자리의 H 중앙값(기준 → T)
  덩어리   = 그 색 마스크의 연결 영역 중 넓이 ≥ min_area 인 수 (팀 min_area_ratio 0.0004 × 640 × 480)
퍽 범위는 팀 perception.yaml detector.hsv, 신호등은 signal_light.hpp LightBlobConfig 기본값.

--awb gray: 자동 화이트밸런스 흉내(gray-world, 채널 평균을 같게)를 씌운 뒤 잰다. --awb illum: 판정용 조명 이득(Cartlidar/sim_render/awb.py). 실물 D435 기본은 AWB 켬(realsense-ros
rgb_camera.enable_auto_white_balance), 렌더에는 없다. 결과 p8_colortemp_hsv{_awb}.json

실행 (PC): python3 -I 261031_0005/p8_colortemp_hsv.py [--awb gray|illum]
"""
import argparse
import json
from pathlib import Path

import sys

import cv2
import numpy as np

TOP = Path(__file__).resolve().parents[1]
RES = TOP / 'Cartlidar/sim_render/results'
sys.path.insert(0, str(TOP / 'Cartlidar/sim_render'))
import awb  # noqa: E402
TEMPS = [0, 6500, 5000, 4000, 3000]
VIEWS = ['box1_close', 'front_d435', 'signal_close']
MIN_AREA = int(0.0004 * 640 * 480)


def puck_masks(hsv):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    sv = (s >= 80) & (v >= 35)
    return {'red': sv & ((h <= 10) | (h >= 170)), 'green': sv & (h >= 40) & (h <= 85)}


def signal_masks(hsv):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    on = (v >= 200) & (s >= 90)
    return {'sig_red': on & ((h <= 10) | (h >= 170)), 'sig_yellow': on & (h >= 15) & (h <= 35),
            'sig_green': on & (h >= 40) & (h <= 90)}


def blobs(m):
    n, _, st, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
    return int((st[1:, cv2.CC_STAT_AREA] >= MIN_AREA).sum())


def hue_median(hsv, m):
    if not m.any():
        return None
    h = hsv[..., 0][m].astype(float)
    h = np.where(h >= 90, h - 180, h) if np.median(h) < 20 or np.median(h) > 160 else h   # 빨강은 0 근처로 펴서
    return round(float(np.median(h)), 1)


def gray_world(im):
    f = im.reshape(-1, 3).astype(np.float64).mean(0)
    return np.clip(im.astype(np.float64) * (f.mean() / f), 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--awb', choices=['gray', 'illum'], default=None)
    args = ap.parse_args()
    out = {}
    for view in VIEWS:
        imgs = {T: cv2.imread(str(RES / f'p8_colortemp_seed5_T{T}/{view}.png')) for T in TEMPS}
        if args.awb == 'gray':
            imgs = {T: gray_world(im) for T, im in imgs.items()}
        elif args.awb == 'illum':
            imgs = {T: awb.apply(im, T) for T, im in imgs.items()}
        hsvs = {T: cv2.cvtColor(im, cv2.COLOR_BGR2HSV) for T, im in imgs.items()}
        base = {**puck_masks(hsvs[0]), **signal_masks(hsvs[0])}
        rows = {}
        for T in TEMPS:
            m = {**puck_masks(hsvs[T]), **signal_masks(hsvs[T])}
            mean_bgr = imgs[T].reshape(-1, 3).mean(0)
            r = {'mean_bgr': [round(float(x), 1) for x in mean_bgr]}
            for k, b in base.items():
                nb = int(b.sum())
                r[k] = {'base_px': nb, 'px': int(m[k].sum()),
                        'keep': round(float((m[k] & b).sum() / nb), 3) if nb else None,
                        'new_px': int((m[k] & ~b).sum()), 'blobs': blobs(m[k]),
                        'h_median_at_base_px': hue_median(hsvs[T], b)}
            rows[T] = r
        out[view] = rows
    (Path(__file__).parent / f"p8_colortemp_hsv{'_awb_' + args.awb if args.awb else ''}.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    for view, rows in out.items():
        print(f'== {view}')
        for T, r in rows.items():
            parts = [f"{k} 유지 {v['keep']} 새 {v['new_px']} 덩어리 {v['blobs']} H {v['h_median_at_base_px']}"
                     for k, v in r.items() if k != 'mean_bgr' and v['base_px'] > 0]
            print(f"  T={T or '끔'} BGR평균 {r['mean_bgr']} | " + ' | '.join(parts))
    if args.awb:
        return
    # 보여 줄 그림: box1_close 끔 · 6500 · 3000 나란히
    row = [cv2.putText(cv2.imread(str(RES / f'p8_colortemp_seed5_T{T}/box1_close.png')), f'{T or "off"} K', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2) for T in (0, 6500, 3000)]
    cv2.imwrite(str(TOP / 'photo/261031_색온도_박스1_끔_6500_3000.png'), np.hstack(row))


if __name__ == '__main__':
    main()
