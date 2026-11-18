#!/usr/bin/env python3
"""P5 판정 bag → mp4 (앞 D435 영상 + 인지 판단 · 정답 글자). PC 에서:
bag_to_video.py <bag 폴더> <결과 폴더(truth_k.log · reset_k.txt)> <출력 mp4>

글자 줄: 판 k · 정답 등(연습 신호등 시리얼) · 인지 판단(/perception/signal light) · 상태 · 순서. 판정과 같은 시각 맞춤(촬영 시각, PC 하나라 시계 차이 0).
"""
import glob
import os
import re
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from robot_interfaces.msg import SignalSequence
from sensor_msgs.msg import CompressedImage

bag, D, out = sys.argv[1], sys.argv[2], sys.argv[3]
ST = {0: 'SEARCHING', 1: 'COUNTING', 2: 'CANDIDATE', 3: 'CONFIRMED', 4: 'MISMATCH'}
LT = {0: 'OFF', 1: 'R', 2: 'Y', 3: 'G'}
BGR = {'R': (40, 40, 255), 'Y': (0, 220, 255), 'G': (60, 220, 60), 'OFF': (180, 180, 180), '?': (180, 180, 180)}

# 정답: 판마다 ON 구간 (보드 ms → PC 시각), 판 시작 시각
trials = []
for f in sorted(glob.glob(os.path.join(D, 'reset_*.txt')), key=lambda p: int(re.findall(r'(\d+)\.txt', p)[0])):
    k = int(re.findall(r'(\d+)\.txt', f)[0])
    t0 = float(open(f).read().split()[-1])
    ons = []
    seq = '?'
    for l in open(os.path.join(D, f'truth_{k}.log')):
        m = re.match(r'(\d+\.\d+) ON \d+ ([RYG]) (\d+)\s*$', l.strip())
        if m:
            ons.append((float(m.group(1)), m.group(2), int(m.group(3))))
        m2 = re.search(r'SEQ ([RYG]{12})', l)
        if m2:
            seq = m2.group(1)
    base = min(pc - ms / 1000 for pc, _, ms in ons)
    trials.append((k, t0, seq, [(base + ms / 1000, base + ms / 1000 + 1.0, c) for _, c, ms in ons]))


def truth_at(t):
    cur = None
    for tr in trials:
        if t >= tr[1]:
            cur = tr
    if cur is None:
        return None, '-', '-'
    for s, e, c in cur[3]:
        if s <= t < e:
            return cur[0], c, cur[2]
    return cur[0], 'OFF', cur[2]


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
vw = None
sig = None
n = 0
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == '/perception/signal':
        sig = deserialize_message(data, SignalSequence)
        continue
    m = deserialize_message(data, CompressedImage)
    img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        continue
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if vw is None:
        h, w = img.shape[:2]
        vw = cv2.VideoWriter(out + '.raw.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (w, h + 90))
    k, tc, tseq = truth_at(t)
    canvas = np.zeros((img.shape[0] + 90, img.shape[1], 3), np.uint8)
    canvas[90:] = img
    pl = '?'
    st = '-'
    seq = '-'
    if sig is not None:
        pl = LT.get(sig.light, '?') if sig.light_visible else '?'
        st = ST.get(sig.status, str(sig.status))
        seq = sig.sequence or '-'
    cv2.putText(canvas, f'trial {k if k else "-"}   truth: {tc}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, BGR.get(tc, (255, 255, 255)), 2)
    cv2.putText(canvas, f'perception: {pl}', (330, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, BGR.get(pl, (255, 255, 255)), 2)
    cv2.putText(canvas, f'{st}  seq {seq}', (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    ok = '' if seq == '-' or tseq == '-' else ('  = truth' if seq == tseq else '  != truth')
    cv2.putText(canvas, f'truth seq {tseq}{ok}', (8, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    vw.write(canvas)
    n += 1
vw.release()
os.system(f'ffmpeg -y -loglevel error -i "{out}.raw.mp4" -c:v libx264 -pix_fmt yuv420p -crf 28 "{out}" && rm "{out}.raw.mp4"')
print('장', n, '→', out)
