#!/usr/bin/env python3
"""TFmini Plus 출력 켜기 시험 (Jetson, ROS 없이 시리얼 직접). 설정 저장 명령(5A 04 11 6F)은 보내지 않음 → 전원을 끄면 원래 설정으로 돌아감.

tfmini_enable.py [--port auto] [--baud 115200] [--out ~/real_drive/tfmini/enable_<시각>]
  (bringup · tfmini_node 가 같은 포트를 쓰고 있으면 끌 것)
auto = /dev/tfmini, 없으면 USB 경로 2.3.2(11-12 가리기 시험으로 정한 TFmini 자리)의 ttyUSB.
단계: ① 아무것도 안 보내고 2 s 듣기 ② 버전 요청 ③ 단일 측정 요청(트리거, 5A 04 04 62) ④ 출력 켜기(5A 05 07 01 67) 뒤 2 s
      ⑤ 그래도 없으면 100 Hz(5A 06 03 64 00 C7) 뒤 2 s ⑥ 그래도 없으면 표준 9 바이트 cm 형식(5A 05 05 01 65) 뒤 2 s
단계마다 응답 바이트 · 9 바이트 측정 프레임 수 · Hz · 거리 · 세기를 화면과 <out>/enable.log 에 남김.
"""
import argparse, glob, os, sys, time

import serial

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='auto'); ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--out', default=os.path.expanduser('~/real_drive/tfmini/enable_' + time.strftime('%m%d_%H%M')))
ap.add_argument('--listen', type=float, default=2.0)
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
log = open(os.path.join(a.out, 'enable.log'), 'a', buffering=1)


def say(s):
    print(s); log.write(s + '\n')


def find_port():
    if os.path.exists('/dev/tfmini'):
        return '/dev/tfmini'
    for p in sorted(glob.glob('/dev/serial/by-path/*')):
        if p.endswith('usb-0:2.3.2:1.0-port0'):
            return os.path.realpath(p)
    say('USB 경로 2.3.2 장치 없음. 지금 보이는 시리얼 경로:')
    for p in sorted(glob.glob('/dev/serial/by-path/*')):
        say(f'  {os.path.basename(p)} -> {os.path.realpath(p)}')
    sys.exit('--port /dev/ttyUSBn 으로 직접 지정 (허브 포트를 바꿨으면 sensor_cover.py 로 다시 확인)')


def cmd(*b):
    return bytes(list(b) + [sum(b) & 0xFF])


def frames(b):
    """9 바이트 측정 프레임(59 59 …, 합 검사) 목록: (dist, strength)."""
    out, i = [], 0
    while i + 9 <= len(b):
        if b[i] == 0x59 and b[i + 1] == 0x59 and (sum(b[i:i + 8]) & 0xFF) == b[i + 8]:
            out.append((b[i + 2] | b[i + 3] << 8, b[i + 4] | b[i + 5] << 8)); i += 9
        else:
            i += 1
    return out


def step(ser, name, send=None, secs=None):
    secs = a.listen if secs is None else secs
    ser.reset_input_buffer()
    if send:
        ser.write(send)
    t0 = time.time(); got = bytearray()
    while time.time() - t0 < secs:
        got += ser.read(4096)
    fr = frames(got)
    head = got[:24].hex(' ')
    ack = ''
    if send and send[0] == 0x5A:
        k = got.find(bytes([0x5A]))
        ack = f' · 응답(5A…) {"있음" if k >= 0 else "없음"}'
    summary = f'{name}: {len(got)} B{ack} · 측정 프레임 {len(fr)} ({len(fr) / secs:.0f} Hz)'
    if fr:
        d = [x for x, _ in fr]; s = [y for _, y in fr]
        summary += f' · 거리 {min(d)}~{max(d)} · 세기 {min(s)}~{max(s)}'
    say(f'{summary}\n    앞 24 B: {head or "(없음)"}')
    return len(fr)


port = find_port() if a.port == 'auto' else a.port
say(f'{time.strftime("%F %T")} 포트 {port} {a.baud} bps · 결과 {a.out}')
ser = serial.Serial(port, a.baud, timeout=0.05)
n = step(ser, '① 듣기(보낸 것 없음)')
step(ser, '② 버전 요청 5A 04 01 5F', cmd(0x5A, 0x04, 0x01), 0.5)
step(ser, '③ 단일 측정 요청 5A 04 04 62', cmd(0x5A, 0x04, 0x04), 0.5)
if n == 0:
    n = step(ser, '④ 출력 켜기 5A 05 07 01 67 뒤 듣기', cmd(0x5A, 0x05, 0x07, 0x01))
if n == 0:
    n = step(ser, '⑤ 100 Hz 5A 06 03 64 00 C7 뒤 듣기', cmd(0x5A, 0x06, 0x03, 0x64, 0x00))
if n == 0:
    n = step(ser, '⑥ 9 바이트 cm 형식 5A 05 05 01 65 뒤 듣기', cmd(0x5A, 0x05, 0x05, 0x01))
say('결과: ' + ('측정 프레임 나옴 — 원래 설정은 꺼져 있었음(저장은 안 함, 전원 끄면 원래대로)' if n else
               '여전히 측정 프레임 없음 — 위 응답 바이트를 그대로 가져올 것'))
