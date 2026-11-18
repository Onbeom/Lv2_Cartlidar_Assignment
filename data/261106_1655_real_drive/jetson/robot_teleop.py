#!/usr/bin/env python3
"""키보드로 바퀴와 팔을 함께 조종한다. ROS · colcon 빌드 없이 OpenCR 에 직접 명령한다. 실행은 바퀴 보드가 붙은 PC 에서.

바퀴 보드: 이 컴퓨터의 시리얼. 팔 보드: --arm_ssh 로 다른 컴퓨터의 arm_agent.py 를 띄워 키를 넘긴다.
"""
import argparse
import json
import os
import select
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import termios
import threading
import time
import tty

from teleop_common import ROLE_WHEEL, find_port, tool

HELP = """
[바퀴]  w 전진  s 후진  a 왼쪽 회전  d 오른쪽 회전  z 왼쪽 원  c 오른쪽 원 (누르는 동안만)   1 2 3 속도 단계   space 멈춤
[팔]    관절       +     -
        J1 허리     y     h
        J2 어깨     u     j
        J3 팔꿈치   i     k
        J4 손목굽힘 o     l
        J5 손목롤   p     ;
        그리퍼     [ 닫기 ' 열기        n 한 번 양 줄이기   m 한 번 양 늘리기
[공통]  r 시작(토크 ON)   e 비상 정지   q 끝내기   (영문 입력 상태로)
"""
LEVELS = {'1': (0.05, 0.25), '2': (0.10, 0.50), '3': (0.20, 1.00)}  # m/s, rad/s
MOVES = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1), 'z': (1, 1), 'c': (1, -1)}  # z · c: 전진하며 회전, 반지름 = 단계의 m/s ÷ rad/s = 0.20 m
ARM_KEYS = {'y': (0, 1), 'h': (0, -1), 'u': (1, 1), 'j': (1, -1), 'i': (2, 1), 'k': (2, -1),
            'o': (3, 1), 'l': (3, -1), 'p': (4, 1), ';': (4, -1), '[': (5, 1), "'": (5, -1)}
STEPS = [0.02, 0.05, 0.10]  # 키 한 번에 움직이는 양 rad
JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5', '그리퍼']


class WheelBoard:
    """바퀴 보드 v5 CMD 100 Hz. 모터 축 rad/s = 바퀴 rad/s / 기어비 (OpenCRHardware 와 같은 변환)."""

    def __init__(self, port, radius, separation, gear, accel=0.5, ang_accel=2.0):
        self.link = tool.Link(port)
        self.port, self.r, self.sep, self.gear = port, radius, separation, gear
        self.accel, self.ang_accel = accel, ang_accel
        self.state, self.msg = 'IDLE', '대기'
        self.goal = (0.0, 0.0)
        self.v, self.w, self.seq = 0.0, 0.0, 0
        self.t_state = time.monotonic()
        self.count = 0
        self.last = None
        self.alive = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _send(self, wheels, flags):
        pl = struct.pack(tool.CMD_FMT, ROLE_WHEEL, *wheels, *([0.0] * 7), flags, self.seq & 0xFF)  # 바퀴 6 · 팔 6 · 그리퍼 1
        self.link.ser.write(tool.frame(tool.TYPE_CMD, pl))
        self.seq += 1

    def _wheels(self):
        left = (self.v - self.w * self.sep / 2) / self.r / self.gear
        right = (self.v + self.w * self.sep / 2) / self.r / self.gear
        return [left, left, right, right]

    def start(self):
        with self.lock:
            s = self.last
            if self.state == 'RUN':
                return
            if s is None or time.monotonic() - self.t_state > 0.3:
                self.msg = '시작 안 함: 바퀴 보드 STATE 없음'
            elif s['role'] != ROLE_WHEEL or s['fw'] != tool.PROTOCOL_VERSION:
                self.msg = f"시작 안 함: 바퀴 보드 v{tool.PROTOCOL_VERSION} 가 아님 (role={s['role']} fw={s['fw']})"
            elif s['error'] & tool.ERR_ABORT:
                self.msg = f"시작 안 함: 오류 {tool.err_text(s['error'])}"
            else:
                self.v, self.w, self.goal = 0.0, 0.0, (0.0, 0.0)
                self.state, self.msg = 'RUN', '바퀴 움직일 수 있음'

    def set_goal(self, v, w):
        with self.lock:
            self.goal = (v, w)

    def halt(self):
        with self.lock:
            self.goal = (0.0, 0.0)
            self.v, self.w = 0.0, 0.0

    def estop(self, why='비상 정지'):
        with self.lock:
            if self.state == 'RUN':
                self.state, self.msg = 'ESTOP', why
                self.v, self.w, self.goal = 0.0, 0.0, (0.0, 0.0)
                self.t_e = time.monotonic()

    def close(self):
        self.estop('끝')
        t = time.monotonic() + 0.5
        while self.state == 'ESTOP' and time.monotonic() < t:
            time.sleep(0.01)
        self.alive = False
        self.thread.join(1.0)
        try:
            for _ in range(5):  # TORQUE_ON 없는 명령 = 바퀴 토크 끔
                self._send([0.0] * 6, 0)
                time.sleep(0.01)
            self.link.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def _loop(self):
        dt = 0.01
        while self.alive:
            t0 = time.monotonic()
            try:
                s = self.link.poll()
                if self.link.count != self.count:
                    self.count, self.t_state = self.link.count, t0
                with self.lock:
                    self.last = s
                    if self.state == 'RUN':
                        if t0 - self.t_state > 0.3:
                            self.state, self.msg, self.t_e = 'ESTOP', '바퀴 보드 STATE 가 0.3 s 넘게 없음', t0
                        elif s and s['error'] & tool.ERR_ABORT:
                            self.state, self.msg, self.t_e = 'ESTOP', f"바퀴 보드 오류 {tool.err_text(s['error'])}", t0
                        if self.state == 'ESTOP':
                            self.v, self.w, self.goal = 0.0, 0.0, (0.0, 0.0)
                    if self.state == 'RUN':
                        gv, gw = self.goal
                        self.v += max(-self.accel * dt, min(self.accel * dt, gv - self.v))
                        self.w += max(-self.ang_accel * dt, min(self.ang_accel * dt, gw - self.w))
                        self._send(self._wheels() + [0.0, 0.0], tool.FLAG_TORQUE_ON)
                    elif self.state == 'ESTOP':
                        self._send([0.0] * 6, tool.FLAG_TORQUE_ON | tool.FLAG_ESTOP)
                        if t0 - self.t_e > 0.3:
                            self.state = 'STOPPED'  # 송신 중단. 펌웨어가 200 ms 뒤 바퀴 0
            except Exception as e:  # noqa: BLE001
                with self.lock:
                    self.state, self.msg = 'STOPPED', f'바퀴 포트 오류 {type(e).__name__}'
                return
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))


class ArmRemote:
    """arm_agent.py 프로세스(ssh 또는 이 컴퓨터)에 한 줄 명령을 보낸다."""

    def __init__(self, cmd, log_path=None):
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.status, self.ready, self.notes = None, False, []
        self.log = open(log_path, 'a', encoding='utf-8') if log_path else None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            try:
                m = json.loads(line)
            except ValueError:
                self.notes.append(line.rstrip())
                continue
            if self.log:
                self.log.write(line)
                self.log.flush()
            if m.get('ready'):
                self.ready = True
                self.notes.append(f"팔 보드 {m.get('port')}")
            else:
                if self.status is None or m.get('msg') != self.status.get('msg'):
                    self.notes.append('팔: ' + str(m.get('msg')))
                self.status = m

    def send(self, line):
        try:
            self.p.stdin.write(line + '\n')
            self.p.stdin.flush()
        except (BrokenPipeError, ValueError, OSError):
            pass

    def alive(self):
        return self.p.poll() is None

    def close(self):
        self.send('quit')
        try:
            self.p.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass
        try:
            self.p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.p.terminate()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wheel_port', default='auto', help="바퀴 보드 포트, 'none' 이면 바퀴 없이")
    ap.add_argument('--arm_ssh', default=None, help='팔 보드가 붙은 컴퓨터, 예 cart11@cart11.local')
    ap.add_argument('--arm_port', default='auto', help="팔 보드 포트, --arm_ssh 없이 쓰면 이 컴퓨터. 'none' 이면 팔 없이")
    ap.add_argument('--arm_dir', default='~/real_drive', help='--arm_ssh 쪽 도구 폴더')
    ap.add_argument('--arm_speed', type=float, default=0.2, help='관절 명령 변화 상한 rad/s')
    ap.add_argument('--arm_cmd', default=None, help='시험용: 팔 프로세스 명령을 직접 줌')
    ap.add_argument('--gear', type=float, default=1.5, help='바퀴 / 모터 (drive_belt 1.5, 직결 1.0)')
    ap.add_argument('--radius', type=float, default=0.033, help='바퀴 반지름 m (controllers_4wd.yaml)')
    ap.add_argument('--separation', type=float, default=0.317, help='바퀴 간격 m (drive_belt.yaml)')
    ap.add_argument('--hold_s', type=float, default=0.6, help='키를 뗀 뒤 바퀴가 멈출 때까지 s')
    ap.add_argument('--log', default=None, help='팔 상태 기록 파일')
    a = ap.parse_args()
    if not sys.stdin.isatty():
        print('터미널에서 실행해야 합니다.', file=sys.stderr)
        return 2

    arm = None
    if a.arm_cmd:
        arm = ArmRemote(shlex.split(a.arm_cmd), a.log)
    elif a.arm_port != 'none':
        here = os.path.dirname(os.path.abspath(__file__))
        args = f'--port {shlex.quote(a.arm_port)} --speed {a.arm_speed}'
        if a.arm_ssh:
            remote = f'cd {a.arm_dir} && exec python3 -u arm_agent.py {args}'
            # 비밀번호는 ssh 가 터미널에서 직접 묻는다. 연결이 끊기면 3 s 안에 끝나 팔이 HOLD 로 간다.
            arm = ArmRemote(['ssh', '-o', 'ServerAliveInterval=1', '-o', 'ServerAliveCountMax=3', a.arm_ssh, remote], a.log)
        else:
            arm = ArmRemote([sys.executable, '-u', os.path.join(here, 'arm_agent.py')] + shlex.split(args), a.log)
    if arm:
        print('팔 쪽 연결 중... (비밀번호를 물으면 입력)')
        t = time.monotonic() + 90
        while not arm.ready and arm.alive() and time.monotonic() < t:
            time.sleep(0.05)
        if not arm.ready:
            for n in arm.notes:
                print(n)
            print('팔 쪽 시작 실패', file=sys.stderr)
            arm.close()
            return 3

    wheel = None
    if a.wheel_port != 'none':
        wheel = WheelBoard(find_port(ROLE_WHEEL, a.wheel_port), a.radius, a.separation, a.gear)
        print(f'바퀴 보드 {wheel.port}')
    if not wheel and not arm:
        print('바퀴도 팔도 없음', file=sys.stderr)
        return 2

    stop = {'flag': False}

    def on_signal(_s, _f):
        stop['flag'] = True

    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)

    print(HELP)
    print('토크가 꺼져 있습니다. r 로 시작하세요.')
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    level, step_i = '1', 1
    move_until, move = 0.0, (0, 0)
    t_hb, t_show = 0.0, 0.0
    t_last = ''
    warned = False
    try:
        tty.setcbreak(fd)
        while not stop['flag']:
            r, _, _ = select.select([fd], [], [], 0.02)
            now = time.monotonic()
            if r:
                data = os.read(fd, 64)
                if not data:
                    break
                for ch in data.decode(errors='ignore'):
                    c = ch.lower()
                    if c == 'q':
                        stop['flag'] = True
                    elif c == 'r':
                        if wheel:
                            wheel.start()
                        if arm:
                            arm.send('start')
                    elif c == 'e':
                        move_until = 0.0
                        if wheel:
                            wheel.estop()
                        if arm:
                            arm.send('stop')
                    elif c == ' ':
                        move_until = 0.0
                        if wheel:
                            wheel.halt()
                        if arm:
                            arm.send('halt')
                    elif c in LEVELS:
                        level = c
                    elif c in MOVES:
                        move, move_until = MOVES[c], now + a.hold_s
                    elif c in ARM_KEYS:
                        if arm:
                            i, sign = ARM_KEYS[c]
                            arm.send(f'step {i} {sign:+d} {STEPS[step_i]}')
                    elif c == 'n':
                        step_i = max(0, step_i - 1)
                    elif c == 'm':
                        step_i = min(len(STEPS) - 1, step_i + 1)
                    elif ord(c) > 127 and not warned:
                        print('\n영문 입력 상태로 바꾸세요.')
                        warned = True
            if wheel:
                lin, ang = LEVELS[level]
                if now < move_until:
                    wheel.set_goal(move[0] * lin, move[1] * ang)
                else:
                    wheel.set_goal(0.0, 0.0)
            if arm:
                if now - t_hb > 0.25:
                    arm.send('hb')
                    t_hb = now
                if not arm.alive():
                    print('\n팔 쪽 연결이 끊김: 팔은 HOLD 로 갑니다. 끝냅니다.')
                    break
                while arm.notes:
                    print('\n' + arm.notes.pop(0))
            if now - t_show > 0.2:
                t_show = now
                parts = [f'단계 {level} · 한 번 {STEPS[step_i]:.2f} rad']
                if wheel:
                    parts.append(f'바퀴 {wheel.state} {wheel.v:+.2f} m/s {wheel.w:+.2f} rad/s')
                    if wheel.state != 'RUN' and wheel.msg not in ('대기', '바퀴 움직일 수 있음'):
                        parts.append(wheel.msg)
                if arm and arm.status and arm.status.get('q'):
                    q = arm.status['q']
                    parts.append(f"팔 {arm.status['state']} " + ' '.join(f'{x:+.2f}' for x in q))
                line = ' | '.join(parts)
                if line != t_last:  # 바뀔 때만 다시 쓴다. 줄을 지우고 써서 좁은 터미널에서도 겹치지 않게
                    width = shutil.get_terminal_size((100, 24)).columns
                    sys.stdout.write('\r\x1b[K' + line[:max(20, width - 2)])
                    sys.stdout.flush()
                    t_last = line
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        if wheel:
            wheel.close()
        if arm:
            arm.close()
    print('\n끝. 바퀴 토크 꺼짐, 팔은 현재 자리에서 HOLD(토크 유지). 팔 토크를 끄려면 팔을 받친 뒤 모터 전원을 끄세요.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
