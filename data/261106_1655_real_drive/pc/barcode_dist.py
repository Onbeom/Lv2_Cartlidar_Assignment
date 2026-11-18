#!/usr/bin/env python3
"""바코드 거리별 판독 (PC, barcode_pc.sh 스택 위에서). 상자 위 바코드 라벨을 Arducam 에서 줄자로 x cm 떨어뜨리고 거리마다 재운다.

barcode_dist.py <결과 폴더> [--secs 10]
순서(거리마다): 거리 cm · 정답 라벨 입력 → 맞추기(한 줄로 읽은 문자열이 계속 나옴, 엔터로 기록 시작) → secs 초 기록 → 요약 한 줄
결과: <폴더>/results.tsv (거리, 정답, READ 메시지 수, 읽음, 정답 수, 틀린 문자열 수 · 목록, 정답 비율 %), <폴더>/raw.jsonl, <폴더>/d<거리>cm.jpg(팔 카메라 한 장, 받으면)
기준(기존 실물 바코드 판정과 같음): 거리마다 정답 비율 ≥ 90 %, 틀린 문자열 0.
"""
import argparse, collections, json, os, select, sys, threading, time
import numpy as np, cv2, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_interfaces.msg import Barcode
from sensor_msgs.msg import Image

ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('--secs', type=float, default=10.0); a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('barcode_dist')
buf = collections.deque(maxlen=2000); img = {}
n.create_subscription(Barcode, '/perception/barcode', lambda m: buf.append((time.time(), m)) if m.source == Barcode.READ else None,
                      QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(Image, '/arm_camera/image_raw', lambda m: img.__setitem__('m', m), qos_profile_sensor_data)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
res = open(os.path.join(a.out, 'results.tsv'), 'a', buffering=1)
if res.tell() == 0:
    res.write('시각\t거리_cm\t정답\tREAD\t읽음\t정답수\t틀림\t틀린문자열\t정답비율%\t판정\n')
raw = open(os.path.join(a.out, 'raw.jsonl'), 'a', buffering=1)
truth = ''
print(__doc__.split('\n\n')[0])
while True:
    d = input('\n거리 cm (엔터 = 끝): ').strip()
    if not d:
        break
    t_in = input(f'정답 라벨 문자열 (엔터 = {truth or "없음"}): ').strip()
    truth = t_in or truth
    print('맞추는 중 — 마지막 읽은 결과가 나옴. 자리 잡으면 엔터')
    while True:
        last = buf[-1][1] if buf else None
        s = '-' if last is None else (f'읽음 {last.text}' if last.read_ok else f'못 읽음 (기호 {last.symbols})')
        sys.stdout.write(f'\r  {s[:60]:60s}'); sys.stdout.flush()
        if select.select([sys.stdin], [], [], 0.3)[0]:
            sys.stdin.readline(); break
    t0 = time.time(); time.sleep(a.secs); t1 = time.time()
    sel = [m for t, m in list(buf) if t0 <= t < t1]
    ok = [m for m in sel if m.read_ok]
    good = [m for m in ok if m.text == truth]
    wrong = collections.Counter(m.text for m in ok if m.text != truth)
    pct = 100 * len(good) / max(1, len(sel))
    verdict = '통과' if sel and pct >= 90 and not wrong else '불통과'
    for m in sel:
        raw.write(json.dumps(dict(dist_cm=d, truth=truth, read_ok=m.read_ok, text=m.text, symbols=m.symbols, stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)) + '\n')
    if 'm' in img:
        m = img['m']
        arr = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
        cv2.imwrite(os.path.join(a.out, f'd{d}cm.jpg'), cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else arr)
    line = f'{time.strftime("%H:%M:%S")}\t{d}\t{truth}\t{len(sel)}\t{len(ok)}\t{len(good)}\t{sum(wrong.values())}\t{dict(wrong)}\t{pct:.1f}\t{verdict}'
    res.write(line + '\n')
    print(f'\n  {d} cm: READ {len(sel)} 개({len(sel) / a.secs:.1f} Hz), 읽음 {len(ok)}, 정답 {len(good)} = {pct:.1f} %, 틀림 {sum(wrong.values())} {dict(wrong) if wrong else ""} → {verdict}')
print('끝. 결과', os.path.join(a.out, 'results.tsv'))
