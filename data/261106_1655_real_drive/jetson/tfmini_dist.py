#!/usr/bin/env python3
"""#87 TFmini Plus 거리 보정 실측 (Jetson, ROS 없이 시리얼 직접). 실제 박스 앞면을 줄자 거리마다 재어 원값 · 신호세기를 남긴다.

tfmini_dist.py <결과 폴더> [--port /dev/tfmini] [--secs 5] [--cm]   (bringup · tfmini_node 가 같은 포트를 쓰고 있으면 끌 것)
켤 때 출력을 mm 로 바꾸는 명령(5A 05 05 06 6A)을 보내고 센서 응답을 기록한다(--cm 이면 보내지 않음, 1 cm 단위).
순서(거리마다): 줄자 거리 cm(렌즈 → 박스 앞면) · 조건 메모(예: 정면 · 15° 기울임 · 검은 면) 입력 → 엔터로 secs 초 기록 → 요약 한 줄
결과: <폴더>/results.tsv (거리, 조건, 프레임 수, 평균 · 표준편차 · 최소 · 최대 mm, 오차 = 평균 − 줄자 mm, 신호세기 평균 · 최소, 무효 수),
      <폴더>/raw.jsonl (프레임마다 dist_mm · strength · temp_c)
무효 = 신호세기 < 100(tfmini.yaml min_strength) 또는 거리 0 · 65535 등 범위 밖. 판정 없이 값만 남김(보정표 calib_raw_m · calib_true_m · min_strength 를 이 값으로 정함).
"""
import argparse, json, os, select, statistics, sys, time

import serial

ap = argparse.ArgumentParser()
ap.add_argument('out'); ap.add_argument('--port', default='auto', help='auto = /dev/tfmini, 없으면 /dev/ttyUSB* 중 TFmini 프레임이 나오는 곳'); ap.add_argument('--secs', type=float, default=5.0)
ap.add_argument('--cm', action='store_true'); ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--min_strength', type=int, default=100)
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
def find_port():
    import glob
    cands = ['/dev/tfmini'] + sorted(glob.glob('/dev/ttyUSB*'))
    for p in cands:
        if not os.path.exists(p):
            continue
        try:
            s = serial.Serial(p, a.baud, timeout=0.3); s.reset_input_buffer(); time.sleep(0.4); b = s.read(600); s.close()
        except Exception:
            continue
        if any(b[i] == 0x59 and b[i + 1] == 0x59 and (sum(b[i:i + 8]) & 0xFF) == b[i + 8] for i in range(len(b) - 8)):
            return p
        print(f'  {p}: TFmini 프레임 없음 ({len(b)} B)')
    sys.exit('TFmini 를 못 찾음 — TFmini 전원(5 V) · USB 변환기 연결 확인')


if a.port == 'auto':
    a.port = find_port()
print('TFmini 포트', a.port)
ser = serial.Serial(a.port, a.baud, timeout=0.05)
unit = 1.0 if not a.cm else 10.0   # 프레임 거리 값 → mm
log = open(os.path.join(a.out, 'run.log'), 'a', buffering=1)
if not a.cm:
    ser.reset_input_buffer()
    ser.write(bytes([0x5A, 0x05, 0x05, 0x06, 0x6A]))
    time.sleep(0.3)
    got = ser.read(4096)
    ack = bytes([0x5A, 0x05, 0x05, 0x06, 0x6A]) in got
    print(f'mm 출력 명령 보냄, 응답 {"받음" if ack else "못 받음(값이 1 mm 단위로 바뀌는지 확인)"}')
    log.write(f'{time.strftime("%F %T")} port {a.port} mm 명령 응답 {"받음" if ack else "못 받음"}\n')
buf = bytearray()


def frames():
    """읽힌 만큼 9 바이트 측정 프레임(59 59 …, 합 검사)을 낸다: (dist, strength, temp_c)."""
    global buf
    buf += ser.read(512)
    out = []
    while len(buf) >= 9:
        if buf[0] != 0x59 or buf[1] != 0x59:
            del buf[0]
            continue
        f = buf[:9]
        if (sum(f[:8]) & 0xFF) != f[8]:
            del buf[0]
            continue
        out.append((f[2] | f[3] << 8, f[4] | f[5] << 8, (f[6] | f[7] << 8) / 8.0 - 256))
        del buf[:9]
    return out


res = open(os.path.join(a.out, 'results.tsv'), 'a', buffering=1)
if res.tell() == 0:
    res.write('시각\t줄자_mm\t조건\t프레임\t평균_mm\t표준편차_mm\t최소_mm\t최대_mm\t오차_mm\t세기_평균\t세기_최소\t무효\n')
raw = open(os.path.join(a.out, 'raw.jsonl'), 'a', buffering=1)
print(__doc__.split('\n\n')[0])
cond = '정면'
while True:
    d = input('\n줄자 거리 cm (렌즈 → 박스 앞면, 엔터 = 끝): ').strip()
    if not d:
        break
    c = input(f'조건 메모 (엔터 = {cond}): ').strip()
    cond = c or cond
    print('맞추는 중 — 지금 값이 나옴. 자리 잡으면 엔터')
    while True:
        fr = frames()
        if fr:
            dd, st, _ = fr[-1]
            sys.stdout.write(f'\r  거리 {dd * unit:7.0f} mm  세기 {st:5d}   '); sys.stdout.flush()
        if select.select([sys.stdin], [], [], 0.05)[0]:
            sys.stdin.readline(); break
    t0 = time.time(); got = []
    ser.reset_input_buffer(); buf.clear()
    while time.time() - t0 < a.secs:
        got += frames()
    true_mm = float(d) * 10
    ok = [(x * unit, s) for x, s, _ in got if s >= a.min_strength and 0 < x < 65535]
    for x, s, tc in got:
        raw.write(json.dumps(dict(true_mm=true_mm, cond=cond, dist_mm=x * unit, strength=s, temp_c=round(tc, 1))) + '\n')
    if ok:
        v = [x for x, _ in ok]; ss = [st for _, st, _ in got]
        m = statistics.mean(v); sd = statistics.pstdev(v)
        line = (f'{time.strftime("%H:%M:%S")}\t{true_mm:.0f}\t{cond}\t{len(got)}\t{m:.1f}\t{sd:.1f}\t{min(v):.0f}\t{max(v):.0f}\t{m - true_mm:+.1f}\t'
                f'{statistics.mean(ss):.0f}\t{min(ss)}\t{len(got) - len(ok)}')
        print(f'\n  {true_mm:.0f} mm ({cond}): 프레임 {len(got)}, 평균 {m:.1f} · 표준편차 {sd:.1f} mm (오차 {m - true_mm:+.1f}), 세기 평균 {statistics.mean(ss):.0f} · 최소 {min(ss)}, 무효 {len(got) - len(ok)}')
    else:
        line = f'{time.strftime("%H:%M:%S")}\t{true_mm:.0f}\t{cond}\t{len(got)}\t\t\t\t\t\t\t\t{len(got)}'
        print(f'\n  {true_mm:.0f} mm: 유효 프레임 없음 (프레임 {len(got)})')
    res.write(line + '\n')
print('끝. 결과', os.path.join(a.out, 'results.tsv'))
