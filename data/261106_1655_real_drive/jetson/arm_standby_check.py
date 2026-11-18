#!/usr/bin/env python3
"""팔 대기 자세 미리 확인 (Jetson, 11-15). bringup 전에 팔 보드 STATE 를 읽기만 해서 대기 범위(설치본 xacro) 안인지 본다.
  python3 arm_standby_check.py   → 통과면 종료 코드 0, 아니면 1 (관절마다 지금 각 · 범위 · 몇 도 옮길지)
포트에는 쓰지 않음 (opencr_arm_tool.py monitor 와 같음). bringup 이 떠 있으면 쓰지 말 것 (같은 포트를 같이 읽게 됨).
"""
import glob
import re
import subprocess
import sys

TOOL = '/home/cart11/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py'
XACRO = '/home/cart11/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/urdf/opencr.ros2_control.xacro'
NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', '집게']


def deg(t):
    return (t - 2048) * 360.0 / 4096.0


x = open(XACRO).read()
lo = [int(v) for v in re.search(r'arm_standby_min_tick">([^<]+)<', x).group(1).split(',')]
hi = [int(v) for v in re.search(r'arm_standby_max_tick">([^<]+)<', x).group(1).split(',')]
arm = None
for p in sorted(glob.glob('/dev/ttyACM*')):
    out = subprocess.run(['python3', TOOL, 'monitor', '--port', p, '--seconds', '1'], capture_output=True, text=True, timeout=10).stdout
    rows = [l for l in out.splitlines() if 'role=2' in l and 'arm[deg]' in l]
    if rows:
        arm = (p, [float(v) for v in rows[-1].split('arm[deg]')[1].split()])
        break
if arm is None:
    print('팔 보드(role 2)를 못 찾음 — 12 V · USB 확인')
    sys.exit(1)
p, q = arm
ok_all = True
print(f'팔 보드 {p}')
for i, n in enumerate(NAMES):
    a, b = deg(lo[i]), deg(hi[i])
    ok = a - 0.05 <= q[i] <= b + 0.05
    ok_all &= ok
    mid = (a + b) / 2
    tip = '' if ok else f'  → 가운데 {mid:+.1f}° 쪽으로 {mid - q[i]:+.1f}° 옮기기' + ('  (360° 어긋남: 12 V 껐다 켜기)' if abs(q[i] - mid) > 180 else '')
    print(f'  {n}: {q[i]:+7.1f}°  범위 {a:+.1f} ~ {b:+.1f}°  {"안" if ok else "밖"}{tip}')
print('판정:', '통과 — bringup 해도 됨' if ok_all else '불통과 — 팔을 옮긴 뒤 다시')
sys.exit(0 if ok_all else 1)
