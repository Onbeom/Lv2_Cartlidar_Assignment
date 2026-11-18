"""신호등 OpenCR 시리얼을 PC 시각과 함께 기록. sig_truth.py <파일> <초> [--new = 새 순서로 다시 시작]"""
import serial, sys, time
out = open(sys.argv[1], 'w', buffering=1); secs = float(sys.argv[2])
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1); t0 = time.time(); buf = b''
if '--new' in sys.argv: s.write(b'n')   # 새 순서로 처음부터 (SEQ 줄이 나옴)
while time.time() - t0 < secs:
    buf += s.read(256)
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        out.write(f'{time.time():.3f} {line.decode(errors="ignore").strip()}\n')
s.close()
