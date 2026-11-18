"""카메라 자동 화이트밸런스(AWB) 흉내 — 렌더 영상 후처리 (모의)

렌더(Isaac RTX)에는 AWB가 없어 조명 색온도가 그대로 찍힌다. 실물 D435는 realsense-ros 기본이 AWB 켬.
판정 렌더는 월드 색온도(layout.json color_temp_k)를 알고 되돌리는 '조명 이득' 방식으로 맞춘다:
  화소(BGR, 8 bit) × 색온도별 채널 이득. 이득은 같은 화면을 색온도 끔 · T 로 렌더한 화소 비율(끔 / T)의 중앙값을
  세 화면(박스1 근접 · 앞 D435 · 신호등 앞, seed 5)에서 평균한 것(261031_0005/README.md). 사이 값은 미레드(1e6 / T)로 선형 보간.
  화면마다 실제 비율이 다르므로(3000 K 파랑 2.41~3.08) 화면별 완벽 보정이 아니다 — 고정 이득 하나인 카메라 AWB에 가깝게 둔 것.
gray-world 는 화면 대부분이 한 색(팔 근접 퍽)이면 그 색을 지우므로 판정에는 쓰지 않는다.

실행 (PC): python3 -I Cartlidar/sim_render/awb.py --in <렌더 폴더> --out <새 폴더> --temp <K>
  폴더 안 8 bit 3채널 PNG · JPG 만 바꾸고 나머지(깊이 16 bit PNG, json 등)는 그대로 복사한다. 원본은 바꾸지 않는다.
"""
import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np

# 색온도 [K] → BGR 이득 (끔 / T). 0 = 색온도 끔(이득 1)
GAIN_TABLE = {
    6500.0: (0.978, 1.011, 0.972),
    5000.0: (1.190, 1.023, 0.885),
    4000.0: (1.543, 1.048, 0.809),
    3000.0: (2.690, 1.109, 0.716),
}


def gains(temp_k):
    if not temp_k:
        return np.ones(3)
    ts = sorted(GAIN_TABLE)
    t = min(max(float(temp_k), ts[0]), ts[-1])
    mired = [1e6 / x for x in ts]
    return np.array([np.interp(1e6 / t, mired[::-1], [GAIN_TABLE[x][c] for x in ts][::-1]) for c in range(3)])


def apply(bgr, temp_k):
    return np.clip(bgr.astype(np.float64) * gains(temp_k), 0, 255).round().astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='src', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--temp', type=float, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    n = 0
    for f in sorted(args.src.rglob('*')):
        dst = args.out / f.relative_to(args.src)
        if f.is_dir():
            dst.mkdir(parents=True, exist_ok=True)
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        im = cv2.imread(str(f), cv2.IMREAD_UNCHANGED) if f.suffix.lower() in ('.png', '.jpg', '.jpeg') else None
        if im is not None and im.dtype == np.uint8 and im.ndim == 3 and im.shape[2] == 3:
            cv2.imwrite(str(dst), apply(im, args.temp))
            n += 1
        else:
            shutil.copy2(f, dst)
    (args.out / 'awb.json').write_text(json.dumps({'source': str(args.src.resolve()), 'temp_k': args.temp,
                                                   'gains_bgr': [round(float(g), 4) for g in gains(args.temp)],
                                                   'images_changed': n, 'tool': 'Cartlidar/sim_render/awb.py'}, indent=1))
    print(f'{n} 장 AWB 적용, 이득 BGR {np.round(gains(args.temp), 3).tolist()}')


if __name__ == '__main__':
    main()
