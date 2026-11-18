#!/usr/bin/env python3
"""Arducam 내부값 보정 촬영 (Jetson). 보정판을 찾으면 흔들리지 않는 장을 자리 · 기울기별로 자동 저장한다.

키: q 끝내기. 다른 키는 없다. 카메라를 천천히 옮기고 잠깐씩 멈추면 된다.
"""
import argparse
import glob
import json
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import cv2
import numpy as np

from charuco_util import N_CORNERS, Detector, object_points

HERE = Path(__file__).resolve().parent
MIN_CORNERS = 12        # 이보다 적으면 저장 안 함
STILL_PX = 1.5          # 직전 장 대비 코너 평균 이동이 이 안이면 '멈춤' (11-09 손에 든 카메라로 1.0 → 1.5)
SHARP_MIN = 40.0        # 보정판 영역 라플라시안 분산 하한
PER_BIN = 2             # 한 자리 · 크기 · 기울기 칸에 저장할 장 수
GRID = (4, 3)           # 화면을 나누는 칸
SAT_LEVEL, SAT_MAX = 250, 0.01  # 이 밝기 이상 화소가 이 비율을 넘으면 노출을 줄인다


def find_device(name):
    if name != 'auto':
        return name
    for p in sorted(glob.glob('/sys/class/video4linux/video*'), key=lambda p: int(p.rsplit('video', 1)[1])):
        if open(p + '/index').read().strip() == '0' and any(k in open(p + '/name').read() for k in ('USB 2.0 Camera', 'Arducam')):
            return '/dev/' + os.path.basename(p)
    sys.exit('Arducam 못 찾음: --device /dev/videoN 으로 주기 (v4l2-ctl --list-devices)')


def tilt_bin(ids, pts):
    """호모그래피 원근 항으로 기울기 칸을 정한다. 0 정면, 1~4 좌우 · 상하."""
    if len(ids) < 6:
        return 0
    H, _ = cv2.findHomography(object_points(ids)[:, :2], pts)
    if H is None:
        return 0
    hx, hy = H[2, 0] / H[2, 2], H[2, 1] / H[2, 2]
    t = 0.012  # 칸 단위 원근 항. 합성 영상에서 약 15° 기울기
    if max(abs(hx), abs(hy)) < t:
        return 0
    if abs(hx) >= abs(hy):
        return 1 if hx > 0 else 2
    return 3 if hy > 0 else 4


def cell(ids, pts, w, h):
    c = pts.mean(axis=0)
    gx = min(GRID[0] - 1, int(c[0] / w * GRID[0]))
    gy = min(GRID[1] - 1, int(c[1] / h * GRID[1]))
    span = float(np.ptp(pts[:, 0]))
    size = 1 if span > 0.45 * w else 0
    return gx, gy, size, tilt_bin(ids, pts)


def show_board():
    """연결된 모니터에 보정판을 가득 띄운다. 못 띄우면 False."""
    try:
        img = cv2.imread(str(HERE / 'charuco_10x7_5x5_100.png'))
        cv2.namedWindow('board', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('board', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        pad = cv2.copyMakeBorder(img, 120, 120, 120, 120, cv2.BORDER_CONSTANT, value=(255, 255, 255))
        cv2.imshow('board', pad)
        cv2.waitKey(200)
        return True
    except cv2.error as e:
        print('보정판 창을 못 띄움:', str(e).splitlines()[0])
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='auto')
    ap.add_argument('--out', type=Path, default=Path.home() / 'calib_arm' / time.strftime('%Y%m%d_%H%M%S'))
    ap.add_argument('--target', type=int, default=40, help='저장할 장 수')
    ap.add_argument('--exposure', type=float, default=None, help='고정 노출. 없으면 흰색이 안 번지게 스스로 맞춤')
    ap.add_argument('--no_board', action='store_true', help='보정판을 다른 화면에 직접 띄울 때')
    ap.add_argument('--preview', action='store_true', help='카메라 영상 · 찾은 코너 · 상태를 창으로 보여 줌(판을 든 사람용)')
    a = ap.parse_args()

    dev = find_device(a.device)
    is_file = os.path.isfile(dev)
    cap = cv2.VideoCapture(dev) if is_file else cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not is_file:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        sys.exit(f'카메라를 못 엶: {dev}')
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (w, h) != (1280, 720):
        sys.exit(f'해상도가 1280x720 이 아님: {w}x{h}')

    exposure = a.exposure
    if not is_file:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # V4L2: 1 = 수동
        if exposure is None:
            exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
            exposure = exposure if exposure > 0 else 156.0
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    auto = a.exposure is None and not is_file

    board_ok = False if a.no_board else show_board()
    a.out.mkdir(parents=True, exist_ok=True)
    print(f'장치 {dev}, {w}x{h}, 저장 {a.out}, 보정판 창 {"뜸" if board_ok else "없음(직접 띄우기)"}')
    print('카메라를 보정판 앞에서 천천히 옮기고 잠깐씩 멈추세요. 화면 가운데 · 네 모서리, 가까이 · 멀리, 좌우 · 상하 기울임. q 로 끝.')

    det = Detector()
    bins, saved, prev = {}, 0, None
    key_pressed = -1
    done_pts = []  # 11-09: 저장한 장의 코너(미리보기에 파랗게 쌓아 빈 곳을 보여 줌)
    frames, found, t_exp, t_print = 0, 0, 0.0, 0.0
    tty_in = sys.stdin.isatty()
    saved_tty = termios.tcgetattr(0) if tty_in else None
    try:
        if tty_in:
            tty.setcbreak(0)
        while saved < a.target:
            if tty_in and select.select([0], [], [], 0)[0] and os.read(0, 8)[:1].lower() == b'q':
                break
            force = key_pressed in (ord(' '), ord('s'))  # 11-09: 미리보기 창에서 스페이스 · s = 지금 장 저장(코너 12 개 이상 · 선명할 때)
            key_pressed = -1
            ok, img = cap.read()
            if not ok:
                if is_file:
                    break
                print('\n카메라에서 영상이 끊김')
                break
            frames += 1
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            now = time.monotonic()
            sat = float((gray >= SAT_LEVEL).mean())
            if auto and now - t_exp > 0.4:
                # 모니터 흰색이 번지면 마커가 뭉개져 검출이 0 이 된다(10-29 미리보기)
                if sat > SAT_MAX:
                    exposure = max(1.0, exposure * 0.75)
                    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
                elif np.median(gray) < 25:
                    exposure = min(5000.0, exposure * 1.25)
                    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
                t_exp = now
            pts, ids, nm = det.detect(gray)
            nc = 0 if pts is None else len(pts)
            status = ''
            if nc:
                found += 1
            if nc >= MIN_CORNERS:
                still = False
                if prev is not None:  # 11-09: 검출이 깜빡여 id 집합이 늘 같지는 않으므로 겹치는 코너로 비교
                    common, ia, ib = np.intersect1d(prev[1], ids, return_indices=True)
                    if len(common) >= MIN_CORNERS:
                        still = float(np.linalg.norm(prev[0][ia] - pts[ib], axis=1).mean()) < STILL_PX
                still = still or force
                prev = (pts.copy(), ids.copy())
                x0, y0 = pts.min(axis=0).astype(int)
                x1, y1 = pts.max(axis=0).astype(int)
                sharp = float(cv2.Laplacian(gray[y0:y1 + 1, x0:x1 + 1], cv2.CV_64F).var())
                key = cell(ids, pts, w, h)
                room = bins.get(key, 0) < PER_BIN
                status = '멈춤' if still else '움직임'
                if still and sharp >= SHARP_MIN and room:
                    bins[key] = bins.get(key, 0) + 1
                    saved += 1
                    cv2.imwrite(str(a.out / f'frame_{saved:03d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 98])
                    done_pts.append(pts.copy())
                    status = '저장'
                    prev = None
                elif force and not room:
                    status = '이 자리 다 찍음, 옮기세요'
                elif still and not room:
                    status = '이 자리 다 찍음, 옮기세요'
                elif still:
                    status = f'흐림({sharp:.0f})'
            else:
                prev = None
            if a.preview:
                v = img.copy()
                for q in done_pts:
                    for x, y in q.astype(int):
                        cv2.circle(v, (int(x), int(y)), 4, (255, 0, 0), -1)
                if nc:
                    for x, y in pts.astype(int):
                        cv2.circle(v, (int(x), int(y)), 6, (0, 255, 0), 2)
                cv2.putText(v, f'saved {saved}/{a.target}  corners {nc}/{N_CORNERS}  ' + {'저장': 'SAVED', '멈춤': 'still', '움직임': 'moving'}.get(status, 'full: move' if '다 찍음' in status else ('blur' if '흐림' in status else '')), (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
                cv2.imshow('arm_calib_preview', cv2.resize(v, (960, 540)))
            if board_ok or a.preview:
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    break
                key_pressed = k if k != 255 else -1
            if now - t_print > 0.5 or status == '저장':
                print(f'\r저장 {saved}/{a.target} · 코너 {nc}/{N_CORNERS} · 노출 {(exposure or 0):.0f} · {status:<14}', end='', flush=True)
                t_print = now
    finally:
        if tty_in:
            termios.tcsetattr(0, termios.TCSADRAIN, saved_tty)
        cap.release()
        cv2.destroyAllWindows()
    print()
    meta = dict(device=dev, width=w, height=h, exposure=exposure, saved=saved, frames=frames, frames_with_corners=found,
                bins={str(k): v for k, v in bins.items()})
    (a.out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    print(f'끝: {saved} 장 저장, 코너 보인 장 {found}/{frames}. 폴더 {a.out}')
    return 0 if saved else 1


if __name__ == '__main__':
    sys.exit(main())
