#!/usr/bin/env python3
"""키보드 조종: 조이스틱 대신 /joy 를 내는 가상 조이스틱.

조이스틱과 같은 길(/joy → teleop_twist_joy → /cmd_vel_teleop → twist_mux)로 가므로
E-stop 잠금 · heartbeat · 리셋도 조이스틱과 똑같이 동작한다.

실행 (Jetson, ssh 터미널): bringup 을 use_teleop:=true 로 띄우고, 실제 조이스틱은 뽑은 뒤
    python3 keyboard_teleop.py

키
    w / s   전진 / 후진         a / d   왼쪽 / 오른쪽 회전
    1 2 3   속도 단계 (기본 1)   space   바로 멈춤
    e       E-stop (잠김)       r       리셋 (잠금 풀기)
    q       끝내기 (heartbeat 가 끊겨 E-stop 이 잠긴다)

키를 떼면 hold_s 뒤 스스로 멈춘다. 끝내거나 ssh 가 끊기면 /joy 가 멈춰 joy_timeout(0.5 s) 뒤 잠긴다.
미션 시작 · 정지 버튼은 일부러 내지 않는다.
"""
import argparse
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from sensor_msgs.msg import Joy

# 축 값 비율. teleop.yaml 의 scale 0.2 m/s · 1.0 rad/s 와 곱해져 실제 속도가 된다.
LEVELS = {'1': (0.25, 0.25), '2': (0.5, 0.5), '3': (1.0, 1.0)}
MOVES = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1)}
PULSE_S = 0.5  # E-stop · 리셋 버튼을 누르고 있는 시간
MIN_AXES, MIN_BUTTONS = 8, 12  # 보통 게임패드와 같은 길이, 받는 쪽이 번호 범위를 벗어나지 않게
FRAME_ID = 'keyboard'  # 내가 낸 /joy 와 진짜 조이스틱을 가르는 표시
FOREIGN_MSG = '다른 곳에서 /joy 가 들어옵니다(조이스틱). 조이스틱을 뽑거나 키보드 조종을 쓰지 마세요.'


def parse_args():
    p = argparse.ArgumentParser(description='키보드로 /joy 를 내는 가상 조이스틱')
    p.add_argument('--rate', type=float, default=20.0, help='/joy 발행 주기 (Hz), teleop.yaml autorepeat 과 같게')
    p.add_argument('--hold_s', type=float, default=0.7, help='키를 뗀 뒤 멈출 때까지 시간 (s), 키 반복 시작 지연보다 길게')
    p.add_argument('--axis_linear', type=int, default=1)
    p.add_argument('--axis_angular', type=int, default=0)
    p.add_argument('--enable_button', type=int, default=4)
    p.add_argument('--estop_button', type=int, default=6)
    p.add_argument('--reset_button', type=int, default=7)
    p.add_argument('--scale_linear', type=float, default=0.2, help='표시용, teleop.yaml 값')
    p.add_argument('--scale_angular', type=float, default=1.0, help='표시용, teleop.yaml 값')
    return p.parse_args()


class Teleop:
    """키 입력을 /joy 내용으로 바꾼다. 시각을 인자로 받아 ROS 없이도 시험할 수 있다."""

    def __init__(self, a):
        self.a = a
        self.level = '1'
        self.move = (0, 0)
        self.move_until = 0.0
        self.estop_until = 0.0
        self.reset_until = 0.0

    def key(self, ch, now):
        """키 하나를 처리한다. 끝내는 키면 False."""
        if ch == 'q' or ch == '\x03':
            return False
        if ch in LEVELS:
            self.level = ch
        elif ch in MOVES:
            self.move = MOVES[ch]
            self.move_until = now + self.a.hold_s
        elif ch == ' ':
            self.move_until = 0.0
        elif ch == 'e':
            self.move_until = 0.0
            self.estop_until = now + PULSE_S
        elif ch == 'r':
            self.move_until = 0.0
            self.reset_until = now + PULSE_S
        return True

    def joy(self, now):
        """지금 내야 할 (axes, buttons)."""
        a = self.a
        axes = [0.0] * max(a.axis_linear + 1, a.axis_angular + 1, MIN_AXES)
        buttons = [0] * max(a.enable_button + 1, a.estop_button + 1, a.reset_button + 1, MIN_BUTTONS)
        if now < self.estop_until:  # E-stop 이 다른 모든 입력보다 먼저
            buttons[a.estop_button] = 1
            return axes, buttons
        if now < self.reset_until:
            buttons[a.reset_button] = 1
        if now < self.move_until:
            lin, ang = LEVELS[self.level]
            axes[a.axis_linear] = self.move[0] * lin
            axes[a.axis_angular] = self.move[1] * ang
            buttons[a.enable_button] = 1
        return axes, buttons

    def status(self, now):
        lin, ang = LEVELS[self.level]
        return (f'단계 {self.level}: 최대 {lin * self.a.scale_linear:.2f} m/s · '
                f'{ang * self.a.scale_angular:.2f} rad/s')


def main():
    a = parse_args()
    if not sys.stdin.isatty():
        print('터미널에서 실행해야 합니다(키 입력을 받지 못함).', file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    # joy_node 와 같은 QoS(reliable). teleop_twist_joy 가 reliable 로 받을 수 있어 맞춘다.
    pub = node.create_publisher(Joy, '/joy', 10)

    # 진짜 조이스틱과 같이 내면 버튼 값이 번갈아 바뀌어 E-stop 이 무시될 수 있다.
    # joy_node 는 조이스틱이 없어도 발행자로 잡히므로, 발행자 수가 아니라 실제 메시지로 판단한다.
    foreign = {'seen': False}

    def on_joy(msg):
        if msg.header.frame_id != FRAME_ID:
            foreign['seen'] = True

    node.create_subscription(Joy, '/joy', on_joy, 10)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not foreign['seen']:
        rclpy.spin_once(node, timeout_sec=0.05)
    if foreign['seen']:
        print(FOREIGN_MSG, file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return 3

    stop = {'flag': False}

    def on_signal(_sig, _frame):
        stop['flag'] = True

    for s in (signal.SIGTERM, signal.SIGHUP):  # ssh 가 끊기면 SIGHUP
        signal.signal(s, on_signal)

    t = Teleop(a)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    print(__doc__.split('키\n')[1].split('키를 떼면')[0])
    print('시작할 때는 잠겨 있습니다. r 로 리셋한 뒤 움직이세요.')
    print(t.status(time.monotonic()))
    period = 1.0 / a.rate
    next_pub = time.monotonic()
    code = 0
    try:
        tty.setcbreak(fd)
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.0)
            if foreign['seen']:
                print(FOREIGN_MSG, file=sys.stderr)
                code = 3
                break
            now = time.monotonic()
            r, _, _ = select.select([fd], [], [], max(0.0, next_pub - now))
            now = time.monotonic()
            if r:
                data = os.read(fd, 32)
                if not data:  # 입력이 닫힘
                    break
                alive = True
                for ch in data.decode(errors='ignore').lower():
                    before = t.level
                    alive = t.key(ch, now) and alive
                    if t.level != before:
                        print(t.status(now))
                    elif ch == 'e':
                        print('E-stop')
                    elif ch == 'r':
                        print('리셋')
                if not alive:
                    break
            if now >= next_pub:
                axes, buttons = t.joy(now)
                msg = Joy()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = FRAME_ID
                msg.axes = axes
                msg.buttons = buttons
                pub.publish(msg)
                next_pub = max(next_pub + period, now)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        # 마지막으로 정지를 한 번 내고 끝낸다. 그 뒤 heartbeat 가 끊겨 잠긴다.
        last = Joy()
        last.header.stamp = node.get_clock().now().to_msg()
        last.header.frame_id = FRAME_ID
        last.axes, last.buttons = Teleop(a).joy(0.0)
        pub.publish(last)
        node.destroy_node()
        rclpy.try_shutdown()
    print('끝. 0.5 s 뒤 E-stop 이 잠깁니다.')
    return code


if __name__ == '__main__':
    sys.exit(main())
