#!/usr/bin/env python3
"""시연용 라이다 두 대 (11-14 Jetson 판 + 11-15 포트 인자 + 11-15 20:3x URDF 보정 자동 판단).

LDS-03 두 대는 기준 모델 (옛 URDF) 보다 자기 축으로 180° 돌아가 있음 (11-14 실측). 고치는 방법은 둘 중 하나만:
  가. 옛 URDF (laser_front rpy = π 0 0)          → 이 스크립트가 스캔을 180° 돌려 /scan_front · /scan_rear 로 냄
  나. 새 URDF (dev 1b9266c · PR #304, yaw +π)    → 돌리지 않고 드라이버가 바로 /scan_front · /scan_rear 로 냄
  둘 다 하면 두 번 돌아 다시 180° 틀어짐. 설치된 robot.urdf.xacro 를 읽어 자동으로 고름. 강제: SCAN_ROT=1 (돌림) · SCAN_ROT=0 (안 돌림)

  python3 ~/demo_v2/scan_rotate.py [앞 장치] [뒤 장치]
    기본 = 팀 udev 이름 /dev/lidar_front · /dev/lidar_rear. 없으면 ls -l /dev/serial/by-path/ 를 보고 인자로
    (11-15 20 시: 허브 2.1.x — 앞 …usb-0:2.1.2:1.0-port0 · 뒤 …usb-0:2.1.1:1.0-port0 추정, 가려서 확인)
  Ctrl+C = 끝 (드라이버도 같이 끔)
"""
import math
import os
import re
import subprocess
import sys

FRONT = sys.argv[1] if len(sys.argv) > 1 else "/dev/lidar_front"
REAR = sys.argv[2] if len(sys.argv) > 2 else "/dev/lidar_rear"
URDF = os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/opencr_hardware/share/opencr_hardware/urdf/robot.urdf.xacro")
for n, p in (("앞", FRONT), ("뒤", REAR)):
    if not os.path.exists(p):
        print(f"✗ {n} 라이다 포트 {p} 없음. 지금 있는 것:", flush=True)
        os.system("ls -l /dev/serial/by-path/ 2>/dev/null | sed 's/^/   /'")
        sys.exit(1)


def urdf_yaw_fixed():
    """설치된 robot.urdf.xacro 의 laser_front yaw 가 0 이 아니면 True (PR #304 보정 있음), 못 읽으면 None"""
    try:
        t = open(URDF, encoding="utf-8").read()
    except OSError:
        return None
    m = re.search(r'name="laser_front"[^>]*rpy="([^"]*)"', t)
    if not m:
        return None
    parts = m.group(1).split(" ", 2)
    return len(parts) == 3 and parts[2].strip() not in ("0", "0.0", "0.000000")


env = os.environ.get("SCAN_ROT", "").strip()
fixed = urdf_yaw_fixed()
if env in ("0", "1"):
    ROT = env == "1"
    why = f"SCAN_ROT={env} 로 정함"
elif fixed is None:
    ROT = True
    why = f"URDF 를 못 읽음 ({URDF}) → 예전처럼 돌림. 맞는지 RViz 로 확인"
else:
    ROT = not fixed
    why = "URDF 에 라이다 yaw 보정 있음 (PR #304) → 돌리지 않음" if fixed else "URDF 에 보정 없음 (옛 판) → 180° 돌림"
OUT_F, OUT_R = ("/scan_front_raw", "/scan_rear_raw") if ROT else ("/scan_front", "/scan_rear")

Y = os.path.expanduser("~/demo_v2/lidar_raw.yaml")
open(Y, "w").write(f"""lidar_node:
  ros__parameters:
    lidar_list: [lidar_front, lidar_rear]
    lidar_front: {{port: {FRONT}, frame_id: laser_front, baudrate: 230400, version: 4, topic_name: {OUT_F}, reverse: false, warmup_time: 5}}
    lidar_rear: {{port: {REAR}, frame_id: laser_rear, baudrate: 230400, version: 4, topic_name: {OUT_R}, reverse: false, warmup_time: 5}}
""")
drv = subprocess.Popen(["ros2", "run", "coin_d4_driver", "multi_coin_d4_node", "--ros-args", "-r", "__node:=lidar_node", "--params-file", Y])
print(f"라이다 {'180° 돌려' if ROT else '그대로 (돌리지 않음)'} 내보내는 중 — 앞 {FRONT} · 뒤 {REAR}  [{why}] (Ctrl+C 끝)", flush=True)

if not ROT:
    try:
        drv.wait()
    except KeyboardInterrupt:
        pass
    finally:
        drv.terminate()
    sys.exit(0)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402


class R(Node):
    def __init__(s):
        super().__init__("scan_rotate180")
        for a, b in (("/scan_front_raw", "/scan_front"), ("/scan_rear_raw", "/scan_rear")):
            p = s.create_publisher(LaserScan, b, 10)
            s.create_subscription(LaserScan, a, lambda m, p=p: s.cb(m, p), qos_profile_sensor_data)

    def cb(s, m, p):
        m.angle_min += math.pi
        m.angle_max += math.pi
        p.publish(m)


rclpy.init()
n = R()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    drv.terminate()
