#!/usr/bin/env python3
"""자동 직진 주행 (Jetson, 키 누를 필요 없음). 바퀴 OpenCR 에 직접 명령 (~/real_drive/robot_teleop.py 의 WheelBoard, 같은 가속 한계).

drive_auto.py test            멈춤 확인: 0.05 m/s 로 1 초 뒤로 → 멈춤 (약 5 cm)
drive_auto.py run front|rear  촬영용: 3 s 멈춤 → 판에서 멀어지는 쪽 5 s → 3 s 멈춤 → 돌아오는 쪽 5 s → 3 s 멈춤 → 한 번 더 → 멈춤
                              (front = 판이 앞 → 먼저 뒤로, rear = 판이 뒤 → 먼저 앞으로). 0.05 m/s 라 한 번에 25 cm, 끝나면 처음 자리 근처.
회전 명령은 없다(각속도 0). Ctrl-C · ssh 끊김(HUP) · TERM 이면 바로 비상 정지하고 바퀴 토크를 끈다. 바퀴 보드 오류 · STATE 끊김도 비상 정지.
"""
import os, signal, sys, time
sys.path.insert(0, os.environ.get('REAL_DRIVE', os.path.expanduser('~/real_drive')))   # 시험 때만 바꿈
from robot_teleop import WheelBoard            # noqa: E402
from teleop_common import ROLE_WHEEL, find_port  # noqa: E402

V = 0.05   # m/s (robot_teleop 속도 단계 1 과 같음)
mode = sys.argv[1] if len(sys.argv) > 1 else 'test'
side = sys.argv[2] if len(sys.argv) > 2 else 'front'
if mode == 'test':
    plan = [(0.0, 1.0), (-V, 1.0), (0.0, 1.5)]
else:
    away = -V if side == 'front' else V
    plan = [(0.0, 3), (away, 5), (0.0, 3), (-away, 5), (0.0, 3), (away, 5), (0.0, 3), (-away, 5), (0.0, 3)]

SCALE = float(os.environ.get('DRIVE_TIME_SCALE', '1'))   # 모의 시험 때만 시간을 줄임
plan = [(v, d * SCALE) for v, d in plan]
wheel = WheelBoard(find_port(ROLE_WHEEL, os.environ.get('WHEEL_PORT', 'auto')), 0.033, 0.317, 1.5)
print(f'바퀴 보드 {wheel.port}', flush=True)
stopped = {'why': None}


def bail(sig, _f):
    stopped['why'] = f'신호 {sig}'
    wheel.estop('사용자 멈춤')


for s in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
    signal.signal(s, bail)
try:
    time.sleep(0.5)
    wheel.start()
    time.sleep(0.2)
    if wheel.state != 'RUN':
        print('시작 못 함:', wheel.msg, flush=True); sys.exit(2)
    t0 = time.monotonic()
    for v, dur in plan:
        print(f'{time.monotonic() - t0:5.1f} s  {"멈춤" if v == 0 else ("앞으로" if v > 0 else "뒤로")} {dur} s', flush=True)
        wheel.set_goal(v, 0.0)
        end = time.monotonic() + dur
        while time.monotonic() < end:
            if wheel.state != 'RUN' or stopped['why']:
                raise SystemExit(f'비상 정지: {stopped["why"] or wheel.msg}')
            time.sleep(0.02)
    wheel.set_goal(0.0, 0.0); time.sleep(0.5)
    print(f'{time.monotonic() - t0:5.1f} s  끝 (멈춤)', flush=True)
finally:
    wheel.halt(); wheel.close()
    print('바퀴 토크 꺼짐', flush=True)
