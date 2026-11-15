#!/usr/bin/env python3
"""주행 OpenCR 의 STATE 만 읽어 RC-100 값 (pos[6] 버튼 · pos[7] 패킷 뒤 ms + 1) 을 찍음 — 포트에 아무것도 쓰지 않음 (읽기만).
펌웨어 업로드 직후 노트북에서: 리모컨 버튼을 누르면 버튼 값이 바뀌는지 (BT-410 짝 · 커넥터 · 펌웨어 확인).

  python3 rc100_state_watch.py /dev/ttyACM0            # Ctrl+C 로 끝
프레임: FF FD | TYPE | LEN | PAYLOAD | CRC16-LE (CRC-16/BUYPASS, 헤더 ~ PAYLOAD), STATE 0x81 192 B (opencr_protocol.h)
"""
import struct
import sys
import time

BTN = {'U': 0x001, 'D': 0x002, 'L': 0x004, 'R': 0x008, '1': 0x010, '2': 0x020, '3': 0x040, '4': 0x080, '5': 0x100, '6': 0x200}
STATE_LEN = 192


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frames(buf):
    """buf (bytearray) 에서 STATE 프레임 페이로드를 꺼냄 (쓴 만큼 buf 에서 지움)"""
    out = []
    while True:
        i = buf.find(b'\xff\xfd')
        if i < 0:
            del buf[:-1]
            return out
        del buf[:i]
        if len(buf) < 4:
            return out
        n = buf[3]
        if len(buf) < 4 + n + 2:
            return out
        body, crc = bytes(buf[:4 + n]), buf[4 + n] | (buf[5 + n] << 8)
        if crc16(body) == crc and buf[2] == 0x81 and n == STATE_LEN:
            out.append(body[4:])
            del buf[:6 + n]
        else:
            del buf[:2]


def decode(pl):
    role = pl[0]
    pos = struct.unpack_from('<12f', pl, 1)
    err, ver, seq = pl[STATE_LEN - 3], pl[STATE_LEN - 2], pl[STATE_LEN - 1]
    return role, int(pos[6]), int(pos[7]), err, ver, seq


def names(bits):
    return '+'.join(k for k, v in BTN.items() if bits & v) or '없음'


def main():
    import serial
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    ser = serial.Serial(port, 115200, timeout=0.05)
    buf, last, n, t0 = bytearray(), None, 0, time.monotonic()
    print(f'{port} 읽기만 — 리모컨 버튼을 눌러 보세요 (Ctrl+C 끝)')
    try:
        while True:
            buf += ser.read(512)
            for pl in frames(buf):
                n += 1
                role, bits, age, err, ver, seq = decode(pl)
                if n == 1:
                    print(f'role={role} (1 = 주행) · 프로토콜 v{ver} · error=0x{err:02x}')
                    if role != 1:
                        print('⚠ 주행 보드가 아님 — 리모컨은 주행 보드에만 있음')
                state = (bits, age > 0)
                if state != last:
                    msg = '받은 적 없음 (0) — 리모컨 · BT-410 짝 · 펌웨어 확인' if age == 0 else f'버튼 {names(bits)} (0x{bits:03x}) · 마지막 패킷 {age - 1} ms 전'
                    print(f'{time.strftime("%H:%M:%S")} {msg}', flush=True)
                    last = state
                elif bits and n % 20 == 0:
                    print(f'   누르는 중 {names(bits)} · 마지막 패킷 {age - 1} ms 전 (계속 작으면 리모컨이 되풀이해 보냄)', flush=True)
    except KeyboardInterrupt:
        pass
    dt = time.monotonic() - t0
    print(f'\nSTATE {n} 개 / {dt:.1f} s ({n / dt if dt else 0:.0f} Hz)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
