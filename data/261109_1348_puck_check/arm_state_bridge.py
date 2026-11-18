#!/usr/bin/env python3
"""팔 OpenCR 의 STATE 만 읽어 /joint_states 로 낸다 (시험 전용, 쓰기 없음 → 토크 · 동작 명령을 보내지 않는다).
팀 opencr_hardware 는 바퀴 · 팔 보드 두 개를 모두 요구해서, 팔 보드 하나로 TF 만 만들 때 쓴다.
프로토콜: opencr_protocol.hpp v5. 관절 슬롯: opencr.ros2_control.xacro."""
import argparse, struct, sys, time
import serial

NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_roll']
ROLE_ARM, VERSION, STATE_LEN = 2, 5, 192
BAD = {0x01: '모터 읽기 실패', 0x08: '모터 하드웨어 오류', 0x10: '부팅 점검 실패', 0x20: '역할 불일치', 0x80: '래치 오류'}
TABLE = []
for i in range(256):
    c = i << 8
    for _ in range(8):
        c = ((c << 1) ^ 0x8005 if c & 0x8000 else c << 1) & 0xFFFF
    TABLE.append(c)


def crc16(b):
    c = 0
    for x in b:
        c = ((c << 8) ^ TABLE[((c >> 8) ^ x) & 0xFF]) & 0xFFFF
    return c


class Reader:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=.05)
        self.b = bytearray()

    def states(self):
        """쌓인 바이트에서 CRC 가 맞는 STATE 를 모두 꺼낸다."""
        self.b += self.s.read(max(1, self.s.in_waiting))
        out = []
        while True:
            i = self.b.find(b'\xff\xfd')
            if i < 0:
                del self.b[:-1]
                return out
            del self.b[:i]
            if len(self.b) < 4:
                return out
            n = self.b[3]
            if len(self.b) < 6 + n:
                return out
            fr = bytes(self.b[:6 + n])
            if crc16(fr[:4 + n]) != struct.unpack('<H', fr[4 + n:])[0]:
                del self.b[:2]
                continue
            del self.b[:6 + n]
            if fr[2] == 0x81 and n == STATE_LEN:
                p = fr[4:4 + n]
                f = struct.unpack('<47f', p[1:189])
                out.append(dict(role=p[0], pos=f[6:12], vel=f[18:24], eff=f[30:36], batt=f[46], err=p[189], ver=p[190]))


def problems(st):
    """관절값을 믿을 수 없는 이유 목록."""
    r = []
    if st['role'] != ROLE_ARM:
        r.append(f"팔 보드가 아님 (role {st['role']})")
    if st['ver'] != VERSION:
        r.append(f"펌웨어 v{st['ver']} (v{VERSION} 이어야 함)")
    r += [t for bit, t in BAD.items() if st['err'] & bit]
    return r


ap = argparse.ArgumentParser()
ap.add_argument('--port', default='/dev/ttyACM0')
ap.add_argument('--j1_offset', type=float, default=0.0, help='시험용: 발행할 때 joint1 에서 빼는 값 [rad] (joint1 영점 수리 전 임시)')
ap.add_argument('--check', action='store_true', help='2 초 읽고 상태만 출력 (ROS 안 씀)')
a = ap.parse_args()
rd = Reader(a.port)

if a.check:
    t, last, n = time.time(), None, 0
    while time.time() - t < 2.0:
        for st in rd.states():
            last, n = st, n + 1
    if last is None:
        print('중단: STATE 를 받지 못함')
        sys.exit(1)
    p = problems(last)
    print(f"STATE {n / 2:.0f} Hz, role {last['role']}, fw v{last['ver']}, error {last['err']:#04x}, 전원 {last['batt']:.2f} V")
    print('관절 [rad]:', ' '.join(f'{x:.3f}' for x in last['pos']))
    print('중단: ' + ', '.join(p) if p else '통과: 팔 관절값 정상')
    sys.exit(1 if p else 0)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
rclpy.init()
n = Node('arm_state_bridge')
pub = n.create_publisher(JointState, '/joint_states', 10)
said = None
while rclpy.ok():
    for st in rd.states():
        p = problems(st)
        if p != said:
            print('관절값 정상, 발행' if not p else '발행 안 함: ' + ', '.join(p), flush=True)
            said = p
        if p:
            continue
        m = JointState()
        m.header.stamp = n.get_clock().now().to_msg()
        m.name = NAMES
        m.position = [float(x) for x in st['pos']]
        m.position[0] -= a.j1_offset
        m.velocity = [float(x) for x in st['vel']]
        m.effort = [float(x) for x in st['eff']]
        pub.publish(m)
