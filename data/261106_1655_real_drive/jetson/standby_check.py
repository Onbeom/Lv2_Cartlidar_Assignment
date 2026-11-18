#!/usr/bin/env python3
"""팔 대기 자세 확인 (읽기만, 명령을 내지 않음). 토크가 꺼진 팔을 손으로 옮기며 보면 된다. Ctrl+C 로 끝.

범위는 설치된 opencr.ros2_control.xacro 의 arm_standby_min_tick · arm_standby_max_tick (bringup 이 같은 값으로 시작을 거부한다).
"""
import math
import os
import re
import shutil
import sys
import time

from teleop_common import ROLE_ARM, find_port, tool

XACRO = os.path.expanduser('~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/urdf/opencr.ros2_control.xacro')
NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', '그리퍼']


def ranges():
    text = open(XACRO, encoding='utf-8').read()
    lo = re.search(r'arm_standby_min_tick">([\d,\s]+)<', text)
    hi = re.search(r'arm_standby_max_tick">([\d,\s]+)<', text)
    if not lo or not hi:
        sys.exit(f'대기 자세 범위를 못 찾음: {XACRO}')
    return [int(x) for x in lo.group(1).split(',')], [int(x) for x in hi.group(1).split(',')]


def tick(rad):
    return round(rad * 4096 / (2 * math.pi) + 2048)


def deg(t):
    return (t - 2048) * 360 / 4096


def main():
    lo, hi = ranges()
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    link = tool.Link(find_port(ROLE_ARM, args[0] if args else 'auto'))
    once = '--once' in sys.argv
    try:
        while True:
            s = link.wait_state(1.0)
            if not s:
                print('STATE 없음')
                continue
            q = tool.arm_pos(s)
            if s['error'] & (tool.ERR_ABORT | 0x10) or all(abs(x + math.pi) < 1e-3 for x in q):
                line = f"모터 응답 없음 ({tool.err_text(s['error'])}) - 12 V 를 켠 뒤 팔 OpenCR 리셋. 자세 값은 의미 없음"
                if once:
                    print(line)
                    return 2
                width = shutil.get_terminal_size((100, 24)).columns
                sys.stdout.write('\r\x1b[K' + line[:max(20, width - 2)])
                sys.stdout.flush()
                time.sleep(0.5)
                continue
            parts, bad = [], 0
            for k in range(6):
                t = tick(q[k])
                if lo[k] <= t <= hi[k]:
                    parts.append(f'{NAMES[k]} OK')
                else:
                    bad += 1
                    target = lo[k] if t < lo[k] else hi[k]
                    parts.append(f'{NAMES[k]} {deg(t):+.0f}° → {deg(lo[k]):+.0f}~{deg(hi[k]):+.0f}° ({deg(target) - deg(t):+.0f}°)')
            line = ('모두 대기 자세 안' if not bad else f'{bad} 칸 밖') + ' | ' + ' · '.join(parts)
            if once:
                print(line)
                return 0 if not bad else 1
            width = shutil.get_terminal_size((100, 24)).columns
            sys.stdout.write('\r\x1b[K' + line[:max(20, width - 2)])
            sys.stdout.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        link.ser.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
