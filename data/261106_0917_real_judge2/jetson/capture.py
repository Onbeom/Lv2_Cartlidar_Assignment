#!/usr/bin/env python3
"""팔 카메라(Arducam) 실물 촬영: Jetson 모니터에 화면을 띄우고, ssh 터미널 키보드로 녹화를 켜고 끈다 (Jetson).

실행 (Jetson, ssh 에서):  bash ~/real_judge2/capture.sh      (DISPLAY=:0 으로 Jetson 모니터에 띄움)
터미널 입력 (Enter 로 보냄):
  <이름> <거리 cm>   녹화 시작. 이름 = 찍는 것 하나: tag00 ~ tag11(퍽 태그) · L1 · L2 · L3(장소 태그) · P2-01 ~ P2-04(바코드)
  (빈 줄)            녹화 끝
  q                  끝내기
저장: <out>/<이름>_<거리>cm_<시각>/NNNNN.png (화면 표시용 글자 · 선이 없는 원본 장) + meta.json
해상도 기본 1280 × 720 MJPG: 640 × 480 은 화면에 줄이 생김(사용자 11-04). YOLO 팔 평가 · 384 엔진(640×360 으로 줄임)도 1280 × 720 기준.
"""
import argparse
import glob
import json
import os
import queue
import sys
import threading
import time

import cv2

ap = argparse.ArgumentParser()
ap.add_argument('--device', default='auto', help='/dev/videoN, auto = 이름에 Arducam · USB 2.0 Camera 가 있는 첫 장치')
ap.add_argument('--width', type=int, default=1280)
ap.add_argument('--height', type=int, default=720)
ap.add_argument('--fps', type=int, default=30)
ap.add_argument('--out', default=os.path.expanduser('~/real_judge2/clips'))
ap.add_argument('--every', type=int, default=3, help='녹화 중 N 장마다 1 장 저장 (720p PNG 는 장당 약 1.5 MB · 저장이 느려 기본 3 = 초당 10 장)')
a = ap.parse_args()

dev = a.device
if dev == 'auto':
    cand = [f'/dev/{os.path.basename(p)}' for p in sorted(glob.glob('/sys/class/video4linux/video*'), key=lambda p: int(p.rsplit('video', 1)[1]))
            if open(p + '/index').read().strip() == '0' and any(k in open(p + '/name').read() for k in ('USB 2.0 Camera', 'Arducam'))]
    if not cand:
        sys.exit('Arducam 못 찾음: --device /dev/videoN 으로 주기 (v4l2-ctl --list-devices)')
    dev = cand[0]
cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
cap.set(cv2.CAP_PROP_FPS, a.fps)
ok, frame = cap.read()
if not ok:
    sys.exit(f'{dev} 에서 영상을 못 받음')
W, H = frame.shape[1], frame.shape[0]
print(f'카메라 {dev}: {W}×{H}, 요청 {a.width}×{a.height} @ {a.fps}', flush=True)
if (W, H) != (a.width, a.height):
    print('  ※ 요청과 다른 해상도로 열림 (그대로 기록함)', flush=True)

cmds = queue.Queue()


def read_stdin():
    for line in sys.stdin:
        cmds.put(line.strip())
    cmds.put('q')


threading.Thread(target=read_stdin, daemon=True).start()
print('입력: "<이름> <거리cm>" Enter = 녹화 시작 · 빈 Enter = 끝 · q = 종료  (예: P2-01 10)', flush=True)

win = 'arm camera (tag_barcode_check)'
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, W * 2 if W <= 800 else W, H * 2 if W <= 800 else H)
rec = None          # {'dir', 'name', 'dist', 'n', 'k', 't0'}
fps_t, fps_n, fps = time.monotonic(), 0, 0.0
while True:
    ok, frame = cap.read()
    if not ok:
        print('영상 끊김', flush=True)
        time.sleep(0.05)
        continue
    fps_n += 1
    if time.monotonic() - fps_t >= 1.0:
        fps, fps_n, fps_t = fps_n / (time.monotonic() - fps_t), 0, time.monotonic()
    if rec is not None:
        rec['k'] += 1
        if (rec['k'] - 1) % a.every == 0:
            cv2.imwrite(os.path.join(rec['dir'], f"{rec['n']:05d}.png"), frame)
            rec['n'] += 1
    view = frame.copy()
    cv2.line(view, (W // 2 - 20, H // 2), (W // 2 + 20, H // 2), (0, 255, 255), 1)
    cv2.line(view, (W // 2, H // 2 - 20), (W // 2, H // 2 + 20), (0, 255, 255), 1)
    txt = f"REC {rec['name']} {rec['dist']}cm {rec['n']}" if rec else 'ready'
    cv2.putText(view, f'{txt}  {fps:.1f} fps', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if rec else (0, 200, 0), 2)
    cv2.imshow(win, view)
    cv2.waitKey(1)
    try:
        c = cmds.get_nowait()
    except queue.Empty:
        continue
    if c == 'q':
        if rec:
            print(f"끝 {rec['name']}: {rec['n']} 장", flush=True)
        break
    if c == '':
        if rec:
            meta = dict(name=rec['name'], distance_cm=rec['dist'], frames=rec['n'], every=a.every, device=dev, width=W, height=H,
                        start=rec['t0'], end=time.time(), fps_measured=round(fps, 2),
                        auto_exposure=cap.get(cv2.CAP_PROP_AUTO_EXPOSURE), exposure=cap.get(cv2.CAP_PROP_EXPOSURE))
            json.dump(meta, open(os.path.join(rec['dir'], 'meta.json'), 'w'), ensure_ascii=False, indent=1)
            print(f"끝 {rec['name']} {rec['dist']}cm: {rec['n']} 장 → {rec['dir']}", flush=True)
            rec = None
        continue
    p = c.split()
    if rec:
        print('녹화 중: 빈 Enter 로 먼저 끝내기', flush=True)
        continue
    if len(p) != 2:
        print('형식: <이름> <거리cm>  (예: tag03 10, L1 10, P2-01 10)', flush=True)
        continue
    d = os.path.join(a.out, f"{p[0]}_{p[1]}cm_{time.strftime('%H%M%S')}")
    os.makedirs(d, exist_ok=True)
    rec = dict(dir=d, name=p[0], dist=p[1], n=0, k=0, t0=time.time())
    print(f'녹화 시작 {p[0]} {p[1]}cm → {d}', flush=True)
cap.release()
cv2.destroyAllWindows()
