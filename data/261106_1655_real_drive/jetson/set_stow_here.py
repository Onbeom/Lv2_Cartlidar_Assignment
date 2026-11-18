#!/usr/bin/env python3
"""지금 팔 자리를 대기 자세로 (Jetson, 11-16, 지도 녹화용 임시 · 사용자 지시, 저장소에 안 올림).
  python3 ~/real_drive/set_stow_here.py
- 팔 보드 STATE 를 읽기만 해서(포트에 안 씀) J1 ~ J5 각을 얻고,
  설치본 opencr.ros2_control.xacro 의 arm_standby_min/max_tick 을 그 각 가운데 · 원래 폭 그대로로,
  manipulator.yaml 의 STOW J1 ~ J5 를 그 각(rad)으로 바꿈. 집게(6 번째)는 그대로.
- bringup 이 떠 있으면 하지 않음(포트를 같이 읽게 됨). 처음 원본은 ~/real_drive/backup_1116_stow/ (있으면 덮지 않음).
"""
import math
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.expanduser('~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src')
TOOL = f'{ROOT}/opencr_hardware/scripts/opencr_arm_tool.py'
XACRO = f'{ROOT}/opencr_hardware/urdf/opencr.ros2_control.xacro'
YAML = f'{ROOT}/control/config/manipulator.yaml'
BK = os.path.expanduser('~/real_drive/backup_1116_stow')

if subprocess.run(['pgrep', '-f', 'lib/controller_manager/ros2_control_node'], capture_output=True).returncode == 0:
    sys.exit('bringup(ros2_control_node) 이 떠 있음 — 끈 뒤 다시')
q = None
for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'):
    if not os.path.exists(p):
        continue
    out = subprocess.run(['python3', TOOL, 'monitor', '--port', p, '--seconds', '1'], capture_output=True, text=True, timeout=10).stdout
    rows = [l for l in out.splitlines() if 'role=2' in l and 'arm[deg]' in l]
    if rows:
        q = [float(v) for v in rows[-1].split('arm[deg]')[1].split()]
        break
if q is None:
    sys.exit('팔 보드(role 2)를 못 찾음 — 12 V · USB 확인')
os.makedirs(BK, exist_ok=True)
for f in (XACRO, YAML):
    if not os.path.exists(f'{BK}/{os.path.basename(f)}'):
        shutil.copy(f, BK)
tick = lambda d: round(2048 + d * 4096 / 360)   # noqa: E731
s = open(XACRO).read()
lo = [int(v) for v in re.search(r'arm_standby_min_tick">([^<]+)<', s).group(1).split(',')]
hi = [int(v) for v in re.search(r'arm_standby_max_tick">([^<]+)<', s).group(1).split(',')]
nlo, nhi = list(lo), list(hi)
for i in range(5):
    half = (hi[i] - lo[i]) // 2
    t = tick(q[i])
    nlo[i], nhi[i] = t - half, t + half
s = re.sub(r'(arm_standby_min_tick">)[^<]+(<)', lambda m: m.group(1) + ','.join(map(str, nlo)) + m.group(2), s)
s = re.sub(r'(arm_standby_max_tick">)[^<]+(<)', lambda m: m.group(1) + ','.join(map(str, nhi)) + m.group(2), s)
open(XACRO, 'w').write(s)
m = open(YAML).read()
old = re.search(r'      STOW: \[[^\]]*\][^\n]*', m).group(0)
grip = old.split('[')[1].split(']')[0].split(',')[5].strip()
new = ('      STOW: [' + ', '.join(f'{math.radians(v):.4f}' for v in q[:5]) + ', ' + grip +
       ']   # 지도 녹화용 임시 (사용자 지시): J1 ~ J5 = set_stow_here.py 로 읽은 실물, 원본 ~/real_drive/backup_1116_stow/')
open(YAML, 'w').write(m.replace(old, new))
print('지금 각 J1 ~ J5:', ' · '.join(f'{v:+.1f}°' for v in q[:5]))
print('대기 범위 min', nlo, '\n          max', nhi)
print('STOW', new.strip())
subprocess.run(['python3', os.path.expanduser('~/real_drive/arm_standby_check.py')])
