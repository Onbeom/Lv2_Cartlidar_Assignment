#!/usr/bin/env python3
"""바퀴 보드(아래 OpenCR) IMU 를 읽기만 한다. 명령을 보내지 않아 모터는 움직이지 않는다. ROS · 빌드 불필요.

처음 1 s 는 로봇을 가만히 둔다(기준 잡기). 그 뒤 앞쪽을 들거나 제자리에서 돌린다. Ctrl-C 또는 --seconds 뒤 끝.
"""
import argparse
import struct
import sys
import time

from teleop_common import ROLE_WHEEL, find_port, tool

QUAT, GYRO, ACCEL = 37, 41, 44  # STATE_FMT 의 위치: role · pos 12 · vel 12 · effort 12 · quat 4 · gyro 3 · accel 3


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='auto')
    ap.add_argument('--seconds', type=float, default=60.0)
    a = ap.parse_args()
    port = find_port(ROLE_WHEEL, a.port)
    link = tool.Link(port)
    print(f'바퀴 보드 {port}. 1 s 동안 가만히 두세요.', flush=True)
    t0 = time.monotonic()
    base_g, base_a = [], []
    bias = zero = None
    yaw, t_prev, t_show, n = 0.0, None, 0.0, 0
    peak = [0.0, 0.0, 0.0]
    try:
        while time.monotonic() - t0 < a.seconds:
            got = False
            for b in link.ser.read(4096):
                r = link.parser.push(b)
                if not (r and r[0] == tool.TYPE_STATE and len(r[1]) == tool.MAX_PAYLOAD):
                    continue
                v = struct.unpack(tool.STATE_FMT, r[1])
                gyro, accel = v[GYRO:GYRO + 3], v[ACCEL:ACCEL + 3]
                now, got = time.monotonic(), True
                n += 1
                if now - t0 < 1.0:
                    base_g.append(gyro)
                    base_a.append(accel)
                    t_prev = now
                    continue
                if bias is None:
                    if not base_g:
                        sys.exit('처음 1 s 동안 STATE 를 못 받음')
                    bias = [sum(x[i] for x in base_g) / len(base_g) for i in range(3)]
                    zero = [sum(x[i] for x in base_a) / len(base_a) for i in range(3)]
                    print(f'기준 가속 x {zero[0]:+.2f} y {zero[1]:+.2f} z {zero[2]:+.2f} m/s² · 표본 {len(base_a)} · 이제 움직이세요', flush=True)
                yaw += (gyro[2] - bias[2]) * (now - t_prev)
                t_prev = now
                d = [accel[i] - zero[i] for i in range(3)]
                for i in range(3):
                    if abs(d[i]) > abs(peak[i]):
                        peak[i] = d[i]
                if now - t_show > 0.2:
                    t_show = now
                    sys.stdout.write(f'\r\x1b[K가속 변화 x {d[0]:+.2f} y {d[1]:+.2f} z {d[2]:+.2f} | ωz {gyro[2] - bias[2]:+.2f} rad/s | yaw 합 {yaw * 57.2958:+.1f}°')
                    sys.stdout.flush()
            if not got:
                time.sleep(0.003)
    except KeyboardInterrupt:
        pass
    link.ser.close()
    print()
    if bias is None:
        print('기준을 잡기 전에 끝남')
        return 1
    i = max(range(3), key=lambda k: abs(peak[k]))
    print(f'가속 변화 최대: x {peak[0]:+.2f} y {peak[1]:+.2f} z {peak[2]:+.2f} m/s² → 가장 크게 변한 축 {"xyz"[i]} ({peak[i]:+.2f})')
    print(f'yaw 합 {yaw * 57.2958:+.1f}° (자이로 z 적분, 반시계가 + 면 맞음) · 표본 {n}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
