#!/usr/bin/env python3
"""P2b 팔 카메라 손-눈 보정 촬영 (Jetson, 창 없음, ssh 터미널). 바닥에 고정한 ChArUco 10x7 보드를 팔 자세를 바꿔 가며 찍는다.

arm_handeye.py [--out 폴더] [--device /dev/arm_cam] [--step 0.05] [--speed 0.15] [--port auto]

키 (터미널):
  1~5   움직일 관절 고르기            w / s  고른 관절을 +step / -step rad (속도 한계 따라 천천히)
  c     지금 장 저장 (팔이 멈춰 있고 보드 코너 ≥ 20 개일 때만). 상태 줄 '폭' = 저장한 장의 관절 J1/J2/J3/J4/J5 범위 rad
  space 정지 (지금 자리에서 HOLD, 토크 유지)       r  다시 시작 (토크 ON, 지금 자세부터)
  q     끝 (HOLD 로 두고 나감 — 팔은 그 자세로 버팀)
시작하면 토크를 켜고 지금 자세를 잡는다(움직이지 않음). 팔이 벽 · 상자 · 보드 · 바닥에 닿기 전에 space.
자세 다양성(검토 조건): 20 장 넘게, J1 · J4 · J5 처럼 축이 다른 관절을 섞어 관절마다 폭 0.6 rad 이상이 목표(모니터가 보이는 범위 안에서).
촬영 동안 차체 · 모니터를 움직이지 않는다(차체는 바퀴를 막아 고정).
저장: <폴더>/caps.jsonl (관절 6 rad, 코너 px(돌리기 전 화면), id, 시각) + <폴더>/cap_NNN.jpg (돌리기 전 원본).
ROS 팔 카메라 노드 · bringup(팔 OpenCR 을 쓰는 것)은 꺼 두고 쓴다 (카메라 · 시리얼을 이 도구가 직접 엶).

자동 순서 (--auto, 키를 못 누를 때 · 보드를 PC 화면 가득 띄울 때): 지금 자세가 ALIGN 이 아니면 DRIVE → ALIGN 으로 한 관절씩
  (관절 3 → 2 → 1 → 4 → 5), ALIGN 주변 자세 26 개를 돌며 멈출 때마다 보드 코너 ≥ min_corners 면 자동 저장(안 보이면 건너뜀),
  끝나면 ALIGN → DRIVE(5 → 4 → 1 → 2 → 3) 에서 멈춤 (--end_stow 면 STOW(4 → 1 → 2 → 3 → 5) 까지). 단계가 25 s 안에 안 끝나면 목표를 지금 자리로 바꾸고 멈춤.
  자세 변화는 J5 ±0.6 · J4 −0.3~+0.1 · J1 ±0.3 rad 위주, J2 · J3 는 ±0.1 rad 만.
  --extra <자세 파일> --extra_names A,B: 손-눈 보정 뒤 ALIGN → A → B 로 한 관절씩(3 → 2 → 1 → 4 → 5) 가서, 자세마다 보드 코너가
  --extra_corners(기본 54 = 전부) 이상 2 s 이어지면 1 s 간격으로 --extra_shots 장 저장(그동안 노트북만 옮김, 최대 --extra_wait s,
  넘으면 건너뜀). 끝나면 DRIVE(3 → 2 → 1 → 4 → 5) → STOW(4 → 1 → 2 → 3 → 5). 멈추려면 터미널에서 space(보드 화면이 가리면 Esc 로 빠져나와서).

자세 사진 반자동 (--auto_pose, #14 정렬점 · P6a 가림): 팔 이동 · 촬영은 자동, 물건을 놓은 뒤 엔터만 누름.
  DRIVE → ALIGN [빈 화면 3 장 · 엔터 · 퍽을 쥘 자리에 놓고 3 장] → 엔터 → BOX_VIEW [빈 상자 3 장 · 엔터 · 퍽 있는 상자 3 장]
  → 엔터 → ALIGN → DRIVE → STOW. 보드는 필요 없음(터미널을 보며 함).

이름 자세 사진 (#14 정렬점 · P6a 가림): --poses arm_poses.yaml --any
  p     다음 이름 자세 고르기 (상태 줄에 이름과 남은 관절 차)
  g     고른 자세로 '한 관절만' 이동 (--order 순서에서 아직 안 맞는 첫 관절). 누를 때마다 한 관절씩, 지켜보다 위험하면 space
  o     이동 순서 뒤집기 (DRIVE → STOW 처럼 되돌아갈 때)
  --any 이면 보드 없이도 c 로 저장 (저장 줄에 자세 이름 · 메모 n 을 남김)
  n     다음 저장에 붙일 메모 바꾸기 (예: 퍽 없음 → 퍽 놓음): 메모 목록 --notes 를 차례로 돎
"""
import argparse
import json
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import cv2
import numpy as np

from charuco_util import Detector
from teleop_common import ROLE_ARM, find_port, tool
import arm_control

ap = argparse.ArgumentParser()
ap.add_argument('--out', default=str(Path.home() / 'real_drive/handeye' / ('run_' + time.strftime('%m%d_%H%M'))))
ap.add_argument('--device', default='/dev/arm_cam')
ap.add_argument('--step', type=float, default=0.05)
ap.add_argument('--speed', type=float, default=0.15)
ap.add_argument('--port', default='auto')
ap.add_argument('--board', default='10x7', help='보드 칸 수 가로x세로 (10x7 = 54 코너, 6x4 = 15 코너: 칸이 커서 멀리서도 읽힘)')
ap.add_argument('--cal', action='store_true', help='보정 자세 촬영: 보드를 못 보면 노트북 밀기 안내 · 다시 하기 없이 --extra_wait 뒤 바로 건너뜀 (11-14 tagfit: 안내가 헷갈리고 자세당 80 s)')
ap.add_argument('--erode_both', action='store_true', help='코너를 erode 0 과 5 로 둘 다 찾아 많은 쪽을 씀 (11-14 BOXLOOK: 같은 장에서 0 이면 14 · 7, 5 면 12 · 15 개)')
ap.add_argument('--erode', type=int, default=0, help='밝은 화면 과노출 대비: 마커를 흰색을 깎은 영상에서 찾음 (예 5, 코너는 원본에서 다듬음)')
ap.add_argument('--min_corners', type=int, default=0, help='저장할 최소 코너 (0 = 10x7 이면 20, 6x4 면 10)')
ap.add_argument('--poses', default='', help='이름 자세 파일 (arm_poses.yaml)')
ap.add_argument('--order', default='3,2,1,4,5', help='g 로 움직일 관절 순서 (1~5)')
ap.add_argument('--any', action='store_true', help='보드 없이도 저장')
ap.add_argument('--auto', action='store_true', help='자동 순서 (위 설명)')
ap.add_argument('--auto_pose', action='store_true', help='자세 사진 반자동 (위 설명)')
ap.add_argument('--sweep', type=int, default=0, help='--auto 에서 돌 자세 수 (0 = 전부 26 개)')
ap.add_argument('--no_sweep', action='store_true', help='--auto 에서 손-눈 보정 자세를 건너뜀 (--extra 자세만)')
ap.add_argument('--tol', type=float, default=0.02, help='도착 판정 관절 오차 rad')
ap.add_argument('--sag_tol', type=float, default=0.06, help='관절이 멈춘 채 --sag_s 가 지나면 이 오차 [rad] 까지 도착으로 봄 (11-13 실물: 관절 3 이 중력으로 0.024 ~ 0.026 rad 처져 0.02 를 못 넘고 25 s 정지). 찍을 때는 엔코더 q 를 쓰므로 결과에 영향 없음')
ap.add_argument('--sag_s', type=float, default=2.0)
ap.add_argument('--end_stow', action='store_true', help='끝에 STOW 까지 접음. 기본은 DRIVE 에서 멈춤 (11-13 실물: DRIVE → STOW 중 관절 1 이 팔레트 v5 첫 줄 왼쪽 끝 칸에 닿고 관절 2 가 −1.94 에서 막힘)')
ap.add_argument('--extra', default='', help='--auto 뒤에 갈 자세 파일 (예: ~/front_cal/fk_poses.yaml)')
ap.add_argument('--extra_names', default='BOXLOOK_PLACE,BOXLOOK_TAG')
ap.add_argument('--extra_shots', type=int, default=3)
ap.add_argument('--measure_max_s', type=float, default=90.0, help='--measure 에서 한 자세에 세워 두는 최대 s. 넘으면 기록 · 안내 뒤 다음 단계(→ ALIGN → DRIVE)로 (11-14 검토: 관절 3 토크 꺼짐이 PLACE 대기 중에 났음)')
ap.add_argument('--measure', action='store_true', help='추가 자세마다 사진 뒤 그 자세로 세워 두고 엔터를 기다림 — 사람이 렌즈 높이 · 앞뒤 자리를 잼 (11-14 TAG 높이). 엔터 때 관절각을 arm_log 에 기록')
ap.add_argument('--extra_corners', type=int, default=0, help='추가 자세에서 저장할 코너 수 (0 = 보드 코너 전부: 10x7 54 · 6x4 15)')
ap.add_argument('--extra_wait', type=float, default=25.0, help='추가 자세에서 보드를 기다리는 s (검토: 20~30 s). 넘으면 DRIVE 로 돌아가 10 s 쉬고 한 번 더')
ap.add_argument('--j3_max_a', type=float, default=1.2, help='관절 2 · 3 전류 [A] 가 이 값을 3 s 넘게 넘으면 정지 (HOLD). 관절 2 · 3 은 I 게인이 있어 막히면 힘이 커짐 (BOXLOOK 검토 조건)')
ap.add_argument('--notes', default='퍽 없음,퍽 놓음', help='n 으로 돌릴 메모 목록 (쉼표)')
ap.add_argument('--show_board', action='store_true', help='연결된 모니터에 보정판을 가득 띄움 (arm_handeye.sh 가 DISPLAY 를 찾아 줌)')
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

# ---- 카메라 (돌리기 전 원본 그대로; 보정 파일이 이 화면 기준) ----
cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    sys.exit(f'카메라 {a.device} 를 못 엶 (ROS 팔 카메라 노드가 쓰고 있으면 끌 것)')
BX, BY = (int(v) for v in a.board.lower().split('x'))
if a.min_corners <= 0:
    a.min_corners = 20 if (BX - 1) * (BY - 1) >= 40 else 10
det = Detector(BX, BY, a.erode)
det5 = Detector(BX, BY, 5) if a.erode_both else None
if a.extra_corners <= 0:
    a.extra_corners = (BX - 1) * (BY - 1)
board_shown = False
if a.show_board:   # 모니터에 보정판을 가득 띄운다 (11-09 내부값 보정과 같은 그림 · 여백)
    try:
        bimg = cv2.imread(str(Path(__file__).resolve().parent / f'charuco_{a.board}_5x5_100.png'))
        cv2.namedWindow('board', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('board', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow('board', cv2.copyMakeBorder(bimg, 120, 120, 120, 120, cv2.BORDER_CONSTANT, value=(255, 255, 255)))
        cv2.waitKey(300)
        board_shown = True
    except cv2.error as e:
        print('보정판 창을 못 띄움:', str(e).splitlines()[0])
cam = {'img': None, 'cc': None, 'ci': None, 't': 0.0, 'move': 99.0, 'hz': 0.0}
lock = threading.Lock()


cam_stop = threading.Event()


def cam_loop():
    prev = None
    t_live, n_frames, t_hz = 0.0, 0, time.time()
    while not cam_stop.is_set():
        ok, img = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        g_ = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cc, ci, _ = det.detect(g_)
        if det5 is not None:   # 둘 다 찾아 코너가 많은 쪽 (같은 그림에서 코너 자리 차 0.001 px, 11-13 확인)
            c5, i5, _ = det5.detect(g_)
            if c5 is not None and (cc is None or len(c5) > len(cc)):
                cc, ci = c5, i5
        move = 99.0
        if cc is not None and prev is not None:
            common = {int(i): p for i, p in zip(prev[1], prev[0])}
            d = [np.linalg.norm(p - common[int(i)]) for i, p in zip(ci, cc) if int(i) in common]
            if d:
                move = float(np.mean(d))
        prev = (cc, ci) if cc is not None else None
        n_frames += 1
        now_w = time.time()
        if now_w - t_live > 2.0:   # 지금 화면을 2 초마다 덮어써 저장 (보드가 안 잡힐 때 원인 확인용, 돌리기 전 원본)
            t_live = now_w
            cv2.imwrite(os.path.join(a.out, 'live.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with lock:
            cam.update(img=img, cc=cc, ci=ci, t=now_w, move=move, hz=n_frames / max(1e-3, now_w - t_hz))
        if now_w - t_hz > 5.0:
            n_frames, t_hz = 0, now_w


cam_th = threading.Thread(target=cam_loop, daemon=True)
cam_th.start()

# ---- 팔 ----
ctl = arm_control.ArmController(find_port(ROLE_ARM, a.port), speed=a.speed)
t = time.monotonic()
while ctl.snapshot()['last'] is None and time.monotonic() - t < 3:
    time.sleep(0.05)
ctl.request('start')
hist = []          # (시각, q) 최근 1 s
sel, n_saved = 0, 0
import yaml
if a.auto_pose:
    a.auto, a.any = True, True
if a.auto and not a.poses:
    a.poses = str(Path(__file__).resolve().parent / 'arm_poses.yaml')
POSES = yaml.safe_load(open(a.poses))['poses'] if a.poses else {}
pnames = list(POSES)
psel = 0
order = [int(v) - 1 for v in a.order.split(',')]
notes = a.notes.split(',')
nsel = 0
SWEEP = [(0, 0, 0, 0, 0), (0, 0, 0, 0, 0.3), (0, 0, 0, 0, -0.3), (0, 0, 0, 0, 0.6), (0, 0, 0, 0, -0.6),
         (0.15, 0, 0, 0, 0), (-0.15, 0, 0, 0, 0), (0.3, 0, 0, 0, 0), (-0.3, 0, 0, 0, 0),
         (0, 0, 0, -0.15, 0), (0, 0, 0, -0.3, 0), (0, 0, 0, 0.1, 0),
         (0.2, 0, 0, 0, 0.4), (0.2, 0, 0, 0, -0.4), (-0.2, 0, 0, 0, 0.4), (-0.2, 0, 0, 0, -0.4),
         (0, 0, 0, -0.2, 0.4), (0, 0, 0, -0.2, -0.4), (0.2, 0, 0, -0.2, 0), (-0.2, 0, 0, -0.2, 0),
         (0, 0.1, 0, 0, 0), (0, -0.1, 0, 0, 0), (0, 0, 0.1, 0, 0), (0, 0, -0.1, 0, 0),
         (0.15, 0, 0, -0.15, 0.3), (-0.15, 0, 0, -0.15, -0.3)]
def relieve():
    """다섯 관절 명령 · 목표를 지금 자리로 바로 맞춤 (막혀 밀던 힘을 풂, 11-13 검토). 그리퍼는 그대로"""
    s2 = ctl.snapshot()
    qq = tool.arm_pos(s2['last']) if s2['last'] else None
    if qq is None:
        return None
    with ctl.lock:
        if ctl.cmd is None or ctl.target is None:
            return None
        for j in range(5):
            ctl.cmd[j] = ctl.target[j] = min(ctl.hi[j], max(ctl.lo[j], qq[j]))
    return [round(float(v), 4) for v in qq[:5]]


plan, step_i, step_t0, cur_j, issued, t_settle, auto_on = [], 0, None, None, False, None, a.auto
t_auto_end = None   # 자동이 끝난 시각 → 2 s 뒤 스스로 끝냄 (11-14 사용자: 언제 끝나는지 모름)
sag_ok = set()   # 이 단계에서 처짐으로 도착 처리한 관절
t_full = None
nshot = 0
pending = ''
retried = set()
t_log, t_j3hi = 0.0, None
f_power = open(os.path.join(a.out, 'power.txt'), 'a', buffering=1)   # 12 V 시작 · 끝 (BOXLOOK 검토 조건: 입력 전압 오류가 1 순위 후보)
start_logged = False
last_v, last_err = None, ''
f_log = open(os.path.join(a.out, 'arm_log.jsonl'), 'a', buffering=1)   # 1 s 마다 관절각 · 전류 (온도는 STATE 에 없음)
enter = False
shown_wait = -1
t_wait0 = 0.0
saved_q = []   # 저장한 장의 관절각 — 관절별 폭을 보여 줘 자세 다양성(검토 조건 1)을 챙기게 함
f_caps = open(os.path.join(a.out, 'caps.jsonl'), 'a', buffering=1)
def save(q, img, cc, ci, pose, note):
    global n_saved
    name = f'cap_{n_saved:03d}.jpg'
    cv2.imwrite(os.path.join(a.out, name), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    f_caps.write(json.dumps(dict(t=time.time(), board=a.board, q=[round(float(v), 5) for v in q], image=name, pose=pose, note=note,
                                 corners=np.round(cc, 3).tolist() if cc is not None else [],
                                 ids=[int(i) for i in ci] if ci is not None else [])) + '\n')
    n_saved += 1
    saved_q.append(list(q[:5]))
    return f'저장 {n_saved} 장 ({name}, 코너 {0 if cc is None else len(cc)})'


old = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
msg = '시작: 토크 ON 중...'
t_print = 0.0
print(__doc__.split('\n\n')[0])
print(f'저장 폴더 {a.out}, 보정판 창 {"뜸" if board_shown else "없음"}')
try:
    while True:
        s = ctl.snapshot()
        q = tool.arm_pos(s['last']) if s['last'] else None
        now = time.monotonic()
        if q is not None:
            hist.append((now, list(q)))
            hist = [h for h in hist if now - h[0] < 1.0]
        still = len(hist) > 5 and now - hist[0][0] > 0.5 and \
            max((abs(h[1][j] - hist[-1][1][j]) for h in hist if now - h[0] < 0.5 for j in range(5)), default=1.0) < 0.002
        tgt_ok = s['target'] is not None and q is not None and max(abs(s['target'][j] - q[j]) for j in range(5)) < 0.01
        with lock:
            cc, ci, move, img = cam['cc'], cam['ci'], cam['move'], cam['img']
        ncc = 0 if cc is None else len(cc)
        if board_shown:
            cv2.waitKey(1)   # 보정판 창 유지
        eff = list(s['last']['effort'][6:12]) if s['last'] and 'effort' in s['last'] else None
        if s['last']:
            last_v, last_err = float(s['last'].get('battery', 0.0)), tool.err_text(s['last']['error'])
        if not start_logged and s['state'] == 'RUN' and q is not None and s['target'] is not None:
            # 토크를 켠 직후: 목표 = 지금 자리인지 (11-12 관절 5 가 −0.07 → −0.72 rad 로 돈 일, 원인 모름)
            start_logged = True
            dq = [round(float(s['target'][j] - q[j]), 4) for j in range(5)]
            f_power.write(json.dumps(dict(t=round(time.time(), 2), event='start', v12=round(last_v, 2), err=last_err,
                                          q=[round(float(v), 4) for v in q[:6]], target=[round(float(v), 4) for v in s['target'][:6]], target_minus_q=dq)) + '\n')
            if max(abs(v) for v in dq) > 0.05:
                ctl.request('stop')
                auto_on = False
                msg = f'시작 목표가 지금 자리와 다름 {dq} → 정지(HOLD). 알려 주세요'
        if eff is not None and now - t_log > 1.0:
            t_log = now
            f_log.write(json.dumps(dict(t=round(time.time(), 2), state=s['state'], step=step_i, q=[round(float(v), 4) for v in q],
                                        cur=[round(float(v), 3) for v in eff], v12=round(last_v, 2) if last_v is not None else None,
                                        err=last_err)) + '\n')
        hi = [j for j in (1, 2) if eff is not None and abs(eff[j]) > a.j3_max_a]
        if hi and s['state'] == 'RUN':
            t_j3hi = t_j3hi or now
            if now - t_j3hi > 3.0:
                ctl.request('stop')
                auto_on = False
                msg = f'관절 {hi[0] + 1} 전류 {abs(eff[hi[0]]):.2f} A > {a.j3_max_a} A 가 3 s 넘음 → 정지(HOLD). 팔을 받치고 확인'
        else:
            t_j3hi = None
        if auto_on and s['state'] == 'RUN' and q is not None and s['target'] is not None:
            if not plan and a.auto_pose:
                if max(abs(POSES['ALIGN'][j] - q[j]) for j in range(5)) > 0.05 and max(abs(POSES['DRIVE'][j] - q[j]) for j in range(5)) > 0.05:
                    plan.append(('joints', 'DRIVE', [2, 1, 0, 3, 4]))
                plan += [('joints', 'ALIGN', [2, 1, 0, 3, 4]),
                         ('wait', 'ALIGN: 집게 앞에 아무것도 없게 두고 엔터'), ('shots', 'ALIGN', '퍽 없음'),
                         ('wait', 'ALIGN: 집게가 쥘 자리에 퍽을 놓고 엔터'), ('shots', 'ALIGN', '퍽 놓음'),
                         ('wait', '퍽을 치우고 엔터 → BOX_VIEW 로 이동'), ('joints', 'BOX_VIEW', [2, 1, 0, 3, 4]),
                         ('wait', 'BOX_VIEW: 빈 상자를 카메라 아래에 놓고 엔터'), ('shots', 'BOX_VIEW', '빈 상자'),
                         ('wait', 'BOX_VIEW: 퍽 있는 상자로 바꾸고 엔터'), ('shots', 'BOX_VIEW', '퍽 있는 상자'),
                         ('wait', '상자를 치우고 엔터 → 접기 시작'),
                         ('joints', 'ALIGN', [4, 3, 0, 1, 2]), ('joints', 'DRIVE', [4, 3, 0, 1, 2]), ('joints', 'STOW', [3, 0, 1, 2, 4])]
            if not plan:   # 처음 RUN 이 됐을 때 지금 자세로 순서를 짬
                AL = POSES['ALIGN']
                if max(abs(AL[j] - q[j]) for j in range(5)) > 0.05:
                    if max(abs(POSES['DRIVE'][j] - q[j]) for j in range(5)) > 0.05:
                        plan.append(('joints', 'DRIVE', [2, 1, 0, 3, 4]))
                    plan.append(('joints', 'ALIGN', [2, 1, 0, 3, 4]))
                for i, off in enumerate([] if a.no_sweep else SWEEP[:a.sweep or len(SWEEP)]):
                    plan += [('all', [AL[j] + off[j] for j in range(5)]), ('shot', i)]
                plan.append(('all', list(AL)))
                if a.extra:
                    EX = yaml.safe_load(open(os.path.expanduser(a.extra)))['poses']
                    for k_nm, nm in enumerate(a.extra_names.split(',')):
                        if k_nm > 0:
                            plan.append(('joints', 'ALIGN', [4, 3, 0, 1, 2]))   # 자세 사이는 ALIGN 을 거침 (11-14, ALIGN ↔ 자세 길만 검토함)
                        plan.append(('joints', nm, [2, 1, 0, 3, 4], EX[nm]))
                        plan += [('shotx', nm, k) for k in range(a.extra_shots)]
                        if a.measure:
                            plan.append(('wait', f'{nm}: 사진 끝, 팔을 이 자세로 {a.measure_max_s:.0f} s 까지 세워 둠 — 렌즈 앞면 가운데 높이(책상 위) · 1 층 앞 끝에서 렌즈 앞면 가운데까지 앞뒤 거리를 재고 엔터. '
                                         f'팔은 만지지 말고 팔 아래에 손을 넣지 마세요. MOTOR_HW 가 뜨면 팔을 받치고 12 V 는 끄지 마세요(오류 값을 읽음). {a.measure_max_s:.0f} s 가 지나면 스스로 DRIVE 로 돌아감', 'measure'))
                    plan.append(('joints', 'ALIGN', [4, 3, 0, 1, 2]))   # 추가 자세에서 온 길(ALIGN)을 거꾸로 (11-13: 관절 3 먼저 접으면 집게가 보드에 닿음)
                plan.append(('joints', 'DRIVE', [4, 3, 0, 1, 2]))
                if a.end_stow:
                    plan.append(('joints', 'STOW', [3, 0, 1, 2, 4]))
            if step_i >= len(plan):
                auto_on = False
                msg = f'자동 끝: 저장 {n_saved} 장, {"STOW" if a.end_stow else "DRIVE"} 에서 멈춤'
                t_auto_end = now
            else:
                st = plan[step_i]
                if step_t0 is None:
                    step_t0, cur_j, issued, t_settle = now, None, False, None
                    sag_ok = set()
                done = False
                if st[0] == 'joints':
                    tgt = st[3] if len(st) > 3 else POSES[st[1]]
                    tgt_c = [min(ctl.hi[j], max(ctl.lo[j], tgt[j])) for j in range(5)]
                    left = [j for j in st[2] if abs(tgt_c[j] - q[j]) > a.tol and j not in sag_ok]
                    if left and cur_j == left[0] and still and now - step_t0 > a.sag_s and abs(tgt_c[cur_j] - q[cur_j]) <= a.sag_tol:
                        sag = round(float(q[cur_j] - tgt_c[cur_j]), 4)   # 중력 처짐: 멈췄는데 tol 밖 → 도착으로 보고 기록
                        sag_ok.add(cur_j)
                        f_log.write(json.dumps(dict(t=round(time.time(), 2), event='sag', step=step_i, pose=st[1], joint=cur_j + 1, q_minus_target=sag)) + '\n')
                        msg = f'자동 {st[1]}: 관절 {cur_j + 1} 처짐 {sag:+.3f} rad — 도착으로 봄'
                        left = [j for j in left if j != cur_j]
                    elif left and cur_j == left[0] and still and now - step_t0 > a.sag_s:
                        err_j = round(float(q[cur_j] - tgt_c[cur_j]), 4)   # 멈췄는데 처짐보다 멂 = 막힘 (11-13 boxlook_1113b 관절 3 0.069)
                        rq = relieve()
                        auto_on = False
                        f_log.write(json.dumps(dict(t=round(time.time(), 2), event='blocked', step=step_i, pose=st[1], joint=cur_j + 1, q_minus_target=err_j, q=rq)) + '\n')
                        msg = f'막힘: {st[1]} 관절 {cur_j + 1} 이 {err_j:+.3f} rad 앞에서 멈춤 → 목표를 지금 자리로(미는 힘 풂), 자동 멈춤. 닿은 곳 확인 뒤 q'
                        left = []
                    if not auto_on:
                        pass
                    elif not left:
                        done = still
                    elif cur_j is None or (cur_j not in left and still) or (cur_j != left[0] and still):
                        cur_j = left[0]
                        ctl.set_target(cur_j, tgt[cur_j])
                        step_t0 = now
                        msg = f'자동 {st[1]}: 관절 {cur_j + 1} → {tgt[cur_j]:+.2f}'
                elif st[0] == 'all':
                    if not issued:
                        for j in range(5):
                            ctl.set_target(j, st[1][j])
                        issued = True
                        msg = f'자동 자세 {(step_i - 1) // 2 + 1}'
                    err = max(abs(s['target'][j] - q[j]) for j in range(5))
                    done = still and (err < a.tol or (now - step_t0 > a.sag_s and err <= a.sag_tol))
                    if not done and still and now - step_t0 > a.sag_s and err > a.sag_tol:
                        rq = relieve()
                        auto_on = False
                        f_log.write(json.dumps(dict(t=round(time.time(), 2), event='blocked', step=step_i, pose=f'auto_{(step_i - 1) // 2}', err=round(float(err), 4), q=rq)) + '\n')
                        msg = f'막힘: 자세 오차 {err:.3f} rad 에서 멈춤 → 목표를 지금 자리로(미는 힘 풂), 자동 멈춤. 닿은 곳 확인 뒤 q'
                    if done and err >= a.tol:
                        f_log.write(json.dumps(dict(t=round(time.time(), 2), event='sag', step=step_i, pose=f'auto_{(step_i - 1) // 2}',
                                                    q_minus_target=[round(float(q[j] - s['target'][j]), 4) for j in range(5)])) + '\n')
                elif st[0] == 'shot':
                    if t_settle is None:
                        t_settle = now
                    if now - t_settle > 0.5 and still and img is not None and ncc >= a.min_corners and move <= 1.0:
                        msg = save(q, img, cc, ci, f'auto_{st[1]:02d}', '')
                        done = True
                    elif now - t_settle > 3.0:
                        if img is not None:   # 건너뛴 자세의 화면을 남김 (보드가 왜 안 잡히는지 확인용, 돌리기 전 원본)
                            cv2.imwrite(os.path.join(a.out, f'miss_{st[1]:02d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        msg = f'자동 자세 {st[1] + 1}: 보드 코너 {ncc} — 건너뜀 (miss_{st[1]:02d}.jpg)'
                        done = True
                elif st[0] == 'wait':
                    step_t0 = now
                    if shown_wait != step_i:
                        shown_wait = step_i
                        t_wait0 = now
                        enter = False
                        sys.stdout.write(f'\n>>> {st[1]}\n')
                        msg = st[1]
                    if enter:
                        enter = False
                        done = True
                        f_log.write(json.dumps(dict(t=round(time.time(), 2), event='wait_done', step=step_i, text=st[1][:40],
                                                    q=[round(float(v), 4) for v in q[:5]])) + '\n')   # 측정 때 관절각 (--measure)
                    elif len(st) > 2 and st[2] == 'measure' and now - t_wait0 > a.measure_max_s:
                        done = True
                        f_log.write(json.dumps(dict(t=round(time.time(), 2), event='measure_timeout', step=step_i, text=st[1][:40],
                                                    q=[round(float(v), 4) for v in q[:5]])) + '\n')
                        msg = f'측정 대기 {a.measure_max_s:.0f} s 넘음 → 다음 단계로(ALIGN → DRIVE). 못 잰 값은 알려 주세요'
                        sys.stdout.write(f'\n>>> {msg}\n')
                elif st[0] == 'shots':
                    if t_settle is None:
                        t_settle, nshot = now, 0
                    if still and img is not None and now - t_settle > 0.5 * (nshot + 1):
                        msg = save(q, img, cc, ci, st[1], st[2])
                        nshot += 1
                        done = nshot >= 3
                elif st[0] == 'shotx':
                    step_t0 = now   # 노트북을 옮기는 동안은 25 s 정지에 걸지 않음 (대신 extra_wait)
                    if t_settle is None:
                        t_settle, t_full = now, None
                        if st[2] == 0 and not a.cal:
                            msg = f'{st[1]}: 보드가 다 보이게 노트북만 옆이나 로봇에서 먼 쪽으로 밀기(로봇 쪽 금지) (코너 {a.extra_corners} 개, 최대 {a.extra_wait:.0f} s). 옮겼으면 화면 높이 · 수평 다시 적기'
                    full = still and img is not None and ncc >= a.extra_corners and move <= 1.0
                    t_full = (t_full or now) if full else None
                    if t_full is not None and now - t_full > (2.0 if st[2] == 0 else 1.0):
                        msg = save(q, img, cc, ci, st[1], f'{st[1]} {st[2] + 1}/{a.extra_shots}')
                        done = True
                    elif now - t_settle > (a.extra_wait if st[2] == 0 else 10.0):
                        if img is not None:   # 못 본 화면을 남김 (11-13: 코너 5/15 인데 어느 쪽이 잘렸는지 몰랐음)
                            cv2.imwrite(os.path.join(a.out, f'miss_{st[1]}_{len(retried)}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if st[1] not in retried and not a.cal:
                            retried.add(st[1])
                            EXr = yaml.safe_load(open(os.path.expanduser(a.extra)))['poses']
                            k_end = step_i
                            while k_end + 1 < len(plan) and plan[k_end + 1][0] == 'shotx' and plan[k_end + 1][1] == st[1]:
                                k_end += 1
                            block = plan[step_i:k_end + 1]   # 이 자세의 촬영 단계 전부 (지금 것 포함)
                            plan[step_i + 1:k_end + 1] = [('joints', 'ALIGN', [4, 3, 0, 1, 2]), ('joints', 'DRIVE', [4, 3, 0, 1, 2]), ('pause', 10.0),
                                                          ('joints', 'ALIGN', [2, 1, 0, 3, 4]),
                                                          ('joints', st[1], [2, 1, 0, 3, 4], EXr[st[1]])] + block   # 온 길(ALIGN)을 거꾸로 돌아감 (11-13: 관절 3 먼저 접으면 집게가 보드에 닿음)
                            msg = f'{st[1]}: {a.extra_wait:.0f} s 안에 보드 못 봄 → DRIVE 에서 10 s 쉬고 한 번 더 (노트북 위치 고치기)'
                        else:
                            msg = f'{st[1]}: 보드 코너 {ncc} < {a.extra_corners} — 건너뜀'
                            while step_i + 1 < len(plan) and plan[step_i + 1][0] == 'shotx' and plan[step_i + 1][1] == st[1]:
                                plan.pop(step_i + 1)   # 이 자세의 남은 촬영도 건너뜀
                        done = True
                elif st[0] == 'pause':
                    if t_settle is None:
                        t_settle = now
                    step_t0 = now
                    done = now - t_settle > st[1]
                if done:
                    step_i += 1
                    step_t0 = None
                elif now - step_t0 > 25.0:
                    # 막혀서 못 간 것일 수 있음 → 목표를 지금 자리로 바꿔 미는 힘을 풂 (11-13: HOLD 가 처진 목표를 유지해 관절 3 이 0.7 A 로 8 분 밀었음)
                    relieve()
                    auto_on = False
                    msg = f'자동 멈춤: 단계 {step_i + 1}/{len(plan)} 가 25 s 안에 안 끝남 — 목표를 지금 자리로 바꿈(미는 힘 풂). 막힌 곳 확인 뒤 q'
        if t_auto_end is not None and now - t_auto_end > 2.0:
            sys.stdout.write('\a\n\n========== 끝: 자동 순서를 마쳤습니다. 팔은 지금 자리에서 HOLD. 다음 명령을 쳐도 됩니다 ==========\n')
            break
        if not pending and select.select([sys.stdin], [], [], 0.05)[0]:
            pending += os.read(sys.stdin.fileno(), 64).decode(errors='ignore')   # 버퍼 없이 읽음 (빨리 누른 키가 갇히지 않게)
        if pending:
            k, pending = pending[0], pending[1:]
            if k in '12345':
                sel = int(k) - 1
                msg = f'관절 {sel + 1} 고름'
            elif k in 'ws' and s['state'] == 'RUN' and s['target'] is not None:
                ctl.set_target(sel, s['target'][sel] + (a.step if k == 'w' else -a.step))
                msg = f'관절 {sel + 1} 목표 {s["target"][sel] + (a.step if k == "w" else -a.step):+.3f} rad'
            elif k == ' ':
                ctl.request('stop')
                auto_on = False
                msg = '정지 (HOLD)' + (' · 자동 멈춤' if a.auto else '')
            elif k == 'r':
                ctl.request('start')
                msg = '다시 시작'
            elif k == 'p' and pnames:
                psel = (psel + 1) % len(pnames)
                msg = f'자세 {pnames[psel]} 고름'
            elif k == 'o':
                order.reverse()
                msg = '순서 ' + ','.join(str(j + 1) for j in order)
            elif k == 'n':
                nsel = (nsel + 1) % len(notes)
                msg = f'메모 {notes[nsel]}'
            elif k == 'g' and pnames and s['state'] == 'RUN' and s['target'] is not None and q is not None:
                tgt = POSES[pnames[psel]]
                left = [j for j in order if abs(tgt[j] - q[j]) > 0.01]
                if not left:
                    msg = f'{pnames[psel]} 도착'
                else:
                    ctl.set_target(left[0], tgt[left[0]])
                    msg = f'{pnames[psel]}: 관절 {left[0] + 1} → {tgt[left[0]]:+.3f}'
            elif k == 'c':
                if not (still and (tgt_ok or s['state'] != 'RUN')):
                    msg = '저장 안 함: 팔이 아직 움직임'
                elif ncc < a.min_corners and not a.any:
                    msg = f'저장 안 함: 보드 코너 {ncc} < {a.min_corners}'
                elif img is None:
                    msg = '저장 안 함: 카메라 영상 없음'
                elif move > 1.0 and not a.any:
                    msg = f'저장 안 함: 영상 흔들림 {move:.1f} px'
                else:
                    msg = save(q, img, cc, ci, pnames[psel] if pnames else '', notes[nsel] if a.any else '')
            elif k == '\n':
                enter = True
            elif k == 'q':
                break
        if now - t_print > 0.5:
            t_print = now
            qs = ' '.join(f'{v:+.2f}' for v in q[:5]) if q is not None else '-'
            why = '' if s['state'] == 'RUN' else f' ({s["msg"][:40]})'
            sys.stdout.write(f'\r[{s["state"]}{why}] q {qs} | 관절 {sel + 1} | 영상 {cam['hz']:4.1f} Hz · 보드 코너 {ncc:2d} 흔들림 {min(move, 99):4.1f}px | ' +
                             (f'{pnames[psel]} 차 ' + '/'.join(f'{POSES[pnames[psel]][j] - q[j]:+.1f}' for j in range(5)) + f' {notes[nsel] if a.any else ""} | ' if pnames and q is not None else '') +
                             f'{"멈춤" if still else "움직임"} | 저장 {n_saved} 폭 ' +
                             ('/'.join(f'{np.ptp([v[j] for v in saved_q]):.1f}' for j in range(5)) if saved_q else '-') + f' | {msg[:34]:34s}')
            sys.stdout.flush()
except KeyboardInterrupt:
    print('\nCtrl-C: 목표를 지금 자리로 바꾸고 HOLD')
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    q_end = relieve()   # 비상 = Ctrl-C → 목표 = 지금 자리 (모터 전원을 끄면 팔이 떨어짐, 11-13 검토)
    time.sleep(0.3)     # 지금 자리 명령이 몇 번 나간 뒤
    ctl.request('stop')
    time.sleep(0.5)
    with ctl.lock:
        cmd_end = [round(float(v), 4) for v in ctl.cmd[:5]] if ctl.cmd else None
    ctl.close()
    cam_stop.set(); cam_th.join(timeout=2); cap.release()
    f_power.write(json.dumps(dict(t=round(time.time(), 2), event='end', v12=round(last_v, 2) if last_v is not None else None, err=last_err,
                                  q=q_end, cmd=cmd_end)) + '\n')
    print(f'12 V 시작 · 끝 → {os.path.join(a.out, "power.txt")} (끝 {last_v} V, 오류 {last_err or "-"})')
    print(f'\n끝: 저장 {n_saved} 장 → {a.out} (팔은 HOLD 로 버팀)')
