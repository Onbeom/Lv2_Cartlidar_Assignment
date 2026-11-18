#!/usr/bin/env python3
"""추적 손실 시험용 팔 키보드 프로그램. 팔 보드가 붙은 컴퓨터의 터미널에서 실행한다. ROS 를 쓰지 않는다.

t 로 토크를 켜면 그때 자세가 기본 자세가 된다. 숫자 키를 누르면 한 관절만 왕복하고 기본 자세로 돌아온다.
왕복하는 동안 팔 카메라 영상을 JPEG 원본 그대로 저장한다(180° 회전은 비교할 때 한다).
안전 규칙은 arm_control.ArmController 를 그대로 쓴다: 추종 오차 감시, 오류 비트 정지, STATE 끊김 정지.
"""
import argparse
import glob
import json
import math
import os
import select
import shutil
import signal
import sys
import termios
import threading
import time
import tty

from teleop_common import ROLE_ARM, find_port, tool
import arm_control

MAX_SPEED, MAX_AMP = 0.6, 0.35   # rad/s, rad. 이보다 큰 값은 실행을 거부한다
LIMIT_MARGIN = 0.01              # rad. 왕복 끝이 관절 한계에서 이만큼 안쪽이어야 한다
LEAD_S = 0.3                     # 녹화를 먼저 시작하고 이만큼 뒤에 움직인다
ARRIVE_RAD, ARRIVE_TIMEOUT = 0.02, 2.0
BACK_SPEED = 0.15                # b 키로 돌아올 때 rad/s


class Path:
    """점들을 차례로 잇는 사다리꼴 속도 궤적. 점마다 잠깐 멈춰 방향을 바꾼다."""

    def __init__(self, points, speed, acc, dwell=0.15):
        self.legs, t = [], 0.0
        for a, b in zip(points, points[1:]):
            dist = abs(b - a)
            vp = min(speed, math.sqrt(dist * acc))
            ta = vp / acc if vp > 0 else 0.0
            tc = (dist - vp * ta) / vp if vp > 0 else 0.0
            self.legs.append((t, a, b, vp, ta, tc, acc))
            t += 2 * ta + tc + dwell
        self.duration = t - dwell if self.legs else 0.0
        self.end = points[-1]

    def pos(self, t):
        for t0, a, b, vp, ta, tc, acc in reversed(self.legs):
            if t < t0:
                continue
            u = t - t0
            d = 1.0 if b >= a else -1.0
            if u < ta:
                s = 0.5 * acc * u * u
            elif u < ta + tc:
                s = 0.5 * vp * ta + vp * (u - ta)
            elif u < 2 * ta + tc:
                r = 2 * ta + tc - u
                s = vp * ta + vp * tc - 0.5 * acc * r * r
            else:
                return b
            return a + d * s
        return self.legs[0][1] if self.legs else self.end


def sweep_points(base, amp, cycles):
    pts = [base]
    for _ in range(cycles):
        pts += [base + amp, base - amp]
    return pts + [base]


def find_camera(name):
    if name != 'auto':
        return name
    for p in sorted(glob.glob('/sys/class/video4linux/video*'), key=lambda p: int(p.rsplit('video', 1)[1])):
        if open(p + '/index').read().strip() == '0' and any(k in open(p + '/name').read() for k in ('USB 2.0 Camera', 'Arducam')):
            return '/dev/' + os.path.basename(p)
    sys.exit('Arducam 못 찾음: --cam /dev/videoN 으로 주거나 --cam none')


def open_camera(dev):
    """팀 arm_camera_node.py 와 같은 설정으로 열고, MJPEG 를 풀지 않은 채 받는다."""
    import cv2
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if not cap.isOpened():
        sys.exit(f'카메라를 못 엶: {dev}')
    return cap


class Recorder:
    """계속 읽어 두고(자동 노출이 맞춰지도록) begin ~ end 사이의 장만 폴더에 쓴다."""

    def __init__(self, cap, out, probe):
        self.cap, self.out, self.probe = cap, out, probe
        self.lock = threading.Lock()
        self.dir, self.csv, self.n, self.bad = None, None, 0, 0
        self.latest = None
        self.alive = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.alive:
            ok, frame = self.cap.read()
            t = time.monotonic()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            data = frame.tobytes()
            with self.lock:
                self.latest = data
                if self.dir is None:
                    continue
                if data[:2] != b'\xff\xd8':
                    self.bad += 1
                    continue
                with open(os.path.join(self.dir, f'{self.n:06d}.jpg'), 'wb') as f:
                    f.write(data)
                q, cmd = self.probe()
                self.csv.write(f'{self.n},{t:.4f},{q:.4f},{cmd:.4f}\n')
                self.n += 1

    def begin(self, name, meta):
        d = os.path.join(self.out, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'run.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        with self.lock:
            self.csv = open(os.path.join(d, 'frames.csv'), 'w', encoding='utf-8')
            self.csv.write('frame,t,q,cmd\n')
            self.dir, self.n, self.bad = d, 0, 0
        return d

    def end(self, result):
        with self.lock:
            if self.dir is None:
                return None
            d, n, bad = self.dir, self.n, self.bad
            self.csv.close()
            self.dir = None
        with open(os.path.join(d, 'result.json'), 'w', encoding='utf-8') as f:
            json.dump(dict(result, frames=n, bad_frames=bad), f, ensure_ascii=False, indent=1)
        return d, n

    def close(self):
        self.end({'result': '프로그램 끝'})
        self.alive = False
        self.thread.join(timeout=1.0)
        self.cap.release()


def check_tags(data):
    """마지막 장(180° 돌림)에서 tag36h11 을 찾아 번호 · 긴 변 px · 가운데 x 를 돌려준다. 녹화와 상관없는 확인용."""
    import cv2
    import numpy as np
    if not data:
        return '영상 없음'
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return '영상을 못 풂'
    img = cv2.rotate(img, cv2.ROTATE_180)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    corners, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=cv2.aruco.DetectorParameters_create())
    if ids is None:
        return f'태그 없음 ({img.shape[1]}×{img.shape[0]})'
    out = []
    for c, i in zip(corners, ids.ravel()):
        q = c.reshape(4, 2)
        side = max(float(np.linalg.norm(q[k] - q[(k + 1) % 4])) for k in range(4))
        out.append(f'ID {i} 긴 변 {side:.0f} px 가운데 x {q[:, 0].mean():.0f}')
    return ' / '.join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='auto', help='팔 보드 포트')
    ap.add_argument('--joint', type=int, default=1, choices=[1, 2, 3, 4, 5], help='왕복할 관절')
    ap.add_argument('--amp', type=float, default=0.20, help='기본 자세에서 한쪽으로 가는 양 rad')
    ap.add_argument('--cycles', type=int, default=3, help='왕복 횟수')
    ap.add_argument('--speeds', default='0.1,0.25,0.5', help='숫자 키 1 2 3 의 속도 rad/s')
    ap.add_argument('--acc', type=float, default=1.5, help='가감속 rad/s²')
    ap.add_argument('--cam', default='auto', help="카메라 장치, 'none' 이면 녹화 안 함")
    ap.add_argument('--out', default='~/real_drive/track_loss/runs', help='녹화 폴더')
    ap.add_argument('--label', default='', help='폴더 이름에 붙일 조건, 예 px100')
    ap.add_argument('--log', default=None, help='상태 기록 파일(JSON 줄)')
    a = ap.parse_args()
    speeds = [float(x) for x in a.speeds.split(',')]
    if not sys.stdin.isatty():
        print('터미널에서 실행해야 합니다.', file=sys.stderr)
        return 2
    if not speeds or len(speeds) > 9 or any(not 0 < v <= MAX_SPEED for v in speeds):
        print(f'--speeds 는 0 초과 {MAX_SPEED} 이하, 9 개까지', file=sys.stderr)
        return 2
    if not 0 < a.amp <= MAX_AMP or a.cycles < 1 or a.acc <= 0:
        print(f'--amp 는 0 초과 {MAX_AMP} 이하, --cycles 는 1 이상, --acc 는 양수', file=sys.stderr)
        return 2

    j = a.joint - 1
    ctl = arm_control.ArmController(find_port(ROLE_ARM, a.port), speed=BACK_SPEED)

    def probe():
        s = ctl.snapshot()
        q = tool.arm_pos(s['last'])[j] if s['last'] else float('nan')
        return q, (s['cmd'][j] if s['cmd'] else float('nan'))

    rec = None
    if a.cam != 'none':
        dev = find_camera(a.cam)
        rec = Recorder(open_camera(dev), os.path.expanduser(a.out), probe)
        print(f'카메라 {dev}, 녹화 폴더 {os.path.expanduser(a.out)}')
    log = open(a.log, 'w', encoding='utf-8', buffering=1) if a.log else None

    stop = {'flag': False}

    def on_signal(_s, _f):
        stop['flag'] = True

    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)

    keys = '   '.join(f'{i + 1} {v:g} rad/s 왕복' for i, v in enumerate(speeds))
    print(f"""
[토크]  t 켜기 · 끄기 (켤 때 자세가 기본 자세, 끄면 팔이 떨어지니 받칠 것)
[동작]  {keys}  (J{a.joint} ±{a.amp:g} rad × {a.cycles}회, 끝나면 기본 자세로)
[확인]  c 지금 화면의 태그 번호 · 긴 변 px · 가운데 x (자리 맞출 때)
[멈춤]  space 그 자리에 멈춤   b 기본 자세로 천천히   e 비상 정지   r 비상 정지 뒤 다시 켜기(토크 유지)
[끝]    q   (영문 입력 상태로)
""")

    def note(text):
        print('\n' + text)
        if log:
            log.write(json.dumps({'t': round(time.monotonic(), 3), 'note': text}, ensure_ascii=False) + '\n')

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    base, motion, prev_state = None, None, None
    t_show, t_log, shown = 0.0, 0.0, ''

    def finish_motion(result):
        nonlocal motion
        if motion is None:
            return
        info = {'result': result, 'speed': motion['speed']}
        motion = None
        ctl.speed = BACK_SPEED
        if rec:
            r = rec.end(info)
            if r:
                note(f'녹화 {r[1]} 장: {r[0]}')

    def halt():
        s = ctl.snapshot()
        if s['state'] == 'RUN':
            for i in range(arm_control.N_MOVE):
                ctl.set_target(i, s['cmd'][i])

    def begin(path, speed, now, record):
        nonlocal motion
        ctl.speed = speed * 1.15
        motion = {'path': path, 't0': now + (LEAD_S if record and rec else 0.0), 'speed': speed, 'done_at': None}
        if record and rec:
            name = time.strftime('%H%M%S') + f'_J{a.joint}_v{speed:g}' + (f'_{a.label}' if a.label else '')
            d = rec.begin(name, {'joint': a.joint, 'speed': speed, 'amp': a.amp, 'cycles': a.cycles, 'acc': a.acc, 'base': base[j]})
            note(f'왕복 시작: {speed:g} rad/s, 녹화 {d}')

    try:
        tty.setcbreak(fd)
        print('토크가 꺼져 있습니다. 팔을 원하는 자세로 두고 t 를 누르세요.')
        while not stop['flag']:
            r, _, _ = select.select([fd], [], [], 0.005)
            now = time.monotonic()
            snap = ctl.snapshot()
            st = snap['state']
            if st == 'RUN' and prev_state != 'RUN':
                base = list(snap['target'])
                note(f'토크 켜짐. 기본 자세 J{a.joint} = {base[j]:+.3f} rad')
            if st != prev_state and st == 'STOPPED':
                note(snap['msg'])
            prev_state = st
            if motion is not None and st != 'RUN':
                finish_motion('중단: ' + snap['msg'])
                note('왕복 중단: ' + snap['msg'])
            if r:
                data = os.read(fd, 64)
                if not data:
                    break
                for ch in data.decode(errors='ignore'):
                    c = ch.lower()
                    if c == 'q':
                        stop['flag'] = True
                    elif c == 'e':
                        finish_motion('비상 정지')
                        ctl.request('stop')
                    elif c == ' ':
                        if motion is not None:
                            finish_motion('space 로 멈춤')
                            note('멈춤. b 로 기본 자세로 돌아갑니다.')
                        halt()
                    elif c == 't':
                        if motion is not None:
                            note('움직이는 중에는 토크를 바꾸지 않습니다. space 로 먼저 멈추세요.')
                        elif st == 'IDLE':
                            ctl.request('start')
                        elif st in ('RUN', 'STOPPED'):
                            ctl.request('off')
                            base = None
                            note('토크 끄는 중: 팔이 떨어집니다.')
                    elif c == 'c':
                        note(check_tags(rec.latest) if rec else '카메라 없이 실행 중')
                    elif c == 'r':
                        if st == 'STOPPED':
                            ctl.request('start')
                    elif c == 'b':
                        if st == 'RUN' and motion is None and base is not None:
                            begin(Path([snap['cmd'][j], base[j]], BACK_SPEED, a.acc), BACK_SPEED, now, False)
                    elif c.isdigit() and 1 <= int(c) <= len(speeds):
                        if st != 'RUN' or base is None:
                            note('토크를 먼저 켜세요 (t).')
                        elif motion is not None:
                            note('움직이는 중입니다.')
                        elif abs(snap['cmd'][j] - base[j]) > ARRIVE_RAD:
                            note('기본 자세가 아닙니다. b 로 먼저 돌아가세요.')
                        elif base[j] - a.amp < ctl.lo[j] + LIMIT_MARGIN or base[j] + a.amp > ctl.hi[j] - LIMIT_MARGIN:
                            note(f'왕복 범위가 관절 한계를 넘습니다: {base[j] - a.amp:+.3f} ~ {base[j] + a.amp:+.3f}, '
                                 f'한계 {ctl.lo[j]:+.3f} ~ {ctl.hi[j]:+.3f}. 자세를 바꾸거나 --amp 를 줄이세요.')
                        else:
                            v = speeds[int(c) - 1]
                            begin(Path(sweep_points(base[j], a.amp, a.cycles), v, a.acc), v, now, True)
                    elif ord(c) > 127:
                        note('영문 입력 상태로 바꾸세요.')
            if motion is not None and st == 'RUN':
                u = now - motion['t0']
                path = motion['path']
                if u >= 0:
                    ctl.set_target(j, path.pos(u) if u < path.duration else path.end)
                if u >= path.duration:
                    q = tool.arm_pos(snap['last'])[j] if snap['last'] else float('nan')
                    motion['done_at'] = motion['done_at'] or now
                    if abs(q - path.end) <= ARRIVE_RAD or now - motion['done_at'] > ARRIVE_TIMEOUT:
                        ok = abs(q - path.end) <= ARRIVE_RAD
                        speed = motion['speed']
                        finish_motion('완료' if ok else f'도착 못함 오차 {q - path.end:+.3f}')
                        note(f'{speed:g} rad/s 끝: 기본 자세 오차 {q - path.end:+.4f} rad' if ok else f'도착 못함: 오차 {q - path.end:+.4f} rad')
            if log and now - t_log > 0.02:
                t_log = now
                log.write(json.dumps({'t': round(now, 3), 'state': st, 'moving': motion is not None,
                                      'q': [round(x, 4) for x in tool.arm_pos(snap['last'])] if snap['last'] else None,
                                      'cmd': [round(x, 4) for x in snap['cmd']] if snap['cmd'] else None,
                                      'base': round(base[j], 4) if base else None,
                                      'err': tool.err_text(snap['last']['error']) if snap['last'] else None}, ensure_ascii=False) + '\n')
            if now - t_show > 0.2:
                t_show = now
                q = tool.arm_pos(snap['last'])[j] if snap['last'] else float('nan')
                line = f"{st} | J{a.joint} {q:+.3f}" + (f" | 기본 {base[j]:+.3f}" if base else '') + (' | 동작 중' if motion else '')
                if st not in ('RUN', 'IDLE'):
                    line += ' | ' + snap['msg']
                if line != shown:
                    width = shutil.get_terminal_size((100, 24)).columns
                    sys.stdout.write('\r\x1b[K' + line[:max(20, width - 2)])
                    sys.stdout.flush()
                    shown = line
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        if motion is not None:
            finish_motion('프로그램 끝')
        if rec:
            rec.close()
        ctl.close()
        if log:
            log.write(json.dumps({'t': round(time.monotonic(), 3), 'state': 'END'}) + '\n')
            log.close()
    print('\n끝. 토크를 켠 채 끝냈으면 팔은 현재 자리에서 HOLD(토크 유지)입니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
