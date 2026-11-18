#!/usr/bin/env python3
"""Jetson 부하 측정(주행 없음): 카메라 3대 + YOLO(TensorRT FP32, D 엔진 하나를 세 카메라가 나눠 씀). ROS 없이 인지 노드와 비슷한 일을 한다.
- 앞 · 뒤 D435: 컬러 640×480 BGR + 깊이 640×480 Z16, 30 fps, 깊이를 컬러에 맞춤(rs.align — realsense2_camera align_depth 와 같은 일). 카메라마다 받는 스레드 하나.
- Arducam: V4L2 MJPG 1280×720 30 fps, 받는 스레드 하나.
- YOLO: 추론 스레드 하나가 카메라마다 15 Hz(팀 perception.yaml publish_rate 15) 로 가장 최근 장을 골라 letterbox → TensorRT → 후처리(conf 0.25, NMS 0.45).
  같은 장을 두 번 쓰지 않음(새 장이 없으면 건너뛰고 'stale' 로 셈).
- 기록: results/<시각>/frames.csv(카메라 · 받은 시각 · 추론 시작 · 끝 · 지연 ms · 결과 나이 ms · 검출 수), cams.csv(카메라별 받은 장수 · 끊김), tegrastats.log(1 초), summary.json.
  결과 나이 = 추론 끝 시각 − 그 장을 받은 시각(같은 monotonic 시계). 카메라 노출 · USB 전송 시간은 빠짐(받은 뒤부터).
사용: python3 load_test.py --engine <D 엔진> --seconds 1800 [--front 261822072247 --rear 261822079801 --arm /dev/video0]
pyrealsense2 가 없는 파이썬이면 ~/capture/.venv 의 site-packages 를, tensorrt 가 없으면 /usr/lib/python3/dist-packages 를 찾아 붙임."""
import argparse, csv, glob, json, os, subprocess, sys, threading, time
from datetime import datetime


def need(mod, extra):
    try:
        __import__(mod)
    except ImportError:
        for p in extra:
            for q in glob.glob(os.path.expanduser(p)):
                if q not in sys.path: sys.path.append(q)
        __import__(mod)


need('tensorrt', ['/usr/lib/python3/dist-packages', '/usr/lib/python3.*/dist-packages'])
need('pyrealsense2', ['~/capture/.venv/lib/python3*/site-packages'])
import numpy as np, cv2, tensorrt as trt, pyrealsense2 as rs
sys.path.insert(0, os.path.expanduser('~/yolo_D')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trt_run import Detector, letterbox, decode  # 같은 전후처리(trt_run.py 를 ~/yolo_D 또는 같은 폴더에)

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--engine', required=True); ap.add_argument('--seconds', type=float, default=1800)
ap.add_argument('--front', default='261822072247'); ap.add_argument('--rear', default='261822079801'); ap.add_argument('--arm', default='auto')
ap.add_argument('--rate', type=float, default=15.0, help='카메라마다 YOLO 주기(Hz)'); ap.add_argument('--out', default=os.path.expanduser('~/load_test/results'))
a = ap.parse_args()
OUT = os.path.join(a.out, datetime.now().strftime('%Y%m%d_%H%M%S')); os.makedirs(OUT)
latest = {}; lock = threading.Lock(); stop = threading.Event(); stats = {}; errors = []


def rs_thread(name, serial):
    try:
        pipe, cfg = rs.pipeline(), rs.config(); cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30); cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipe.start(cfg); align = rs.align(rs.stream.color); st = stats.setdefault(name, {'frames': 0, 'gaps': 0}); last = None
        while not stop.is_set():
            fs = pipe.wait_for_frames(5000); t = time.monotonic(); fs = align.process(fs); c = fs.get_color_frame(); d = fs.get_depth_frame()
            n = c.get_frame_number()
            if last is not None and n > last + 1: st['gaps'] += n - last - 1
            last = n; st['frames'] += 1
            img = np.asanyarray(c.get_data()).copy(); _ = np.asanyarray(d.get_data())  # 깊이도 실제로 꺼냄
            with lock: latest[name] = (t, st['frames'], img)
        pipe.stop()
    except Exception as e:
        errors.append(f'{name}: {e}'); stop.set()


def uvc_thread(name, dev):
    try:
        if dev == 'auto':
            cand = [f'/dev/{os.path.basename(p)}' for p in sorted(glob.glob('/sys/class/video4linux/video*'), key=lambda p: int(p.rsplit('video', 1)[1]))
                    if open(p + '/index').read().strip() == '0' and any(k in open(p + '/name').read() for k in ('USB 2.0 Camera', 'Arducam'))]
            assert cand, 'Arducam 못 찾음(--arm /dev/videoN)'; dev = cand[0]
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2); cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720); cap.set(cv2.CAP_PROP_FPS, 30)
        st = stats.setdefault(name, {'frames': 0, 'gaps': 0, 'device': dev})
        while not stop.is_set():
            ok, img = cap.read(); t = time.monotonic()
            if not ok: st['gaps'] += 1; time.sleep(0.01); continue
            st['frames'] += 1
            with lock: latest[name] = (t, st['frames'], img)
        cap.release()
    except Exception as e:
        errors.append(f'{name}: {e}'); stop.set()


det = Detector(a.engine)
teg = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=open(os.path.join(OUT, 'tegrastats.log'), 'w'), stderr=subprocess.STDOUT)
th = [threading.Thread(target=rs_thread, args=('front', a.front), daemon=True), threading.Thread(target=rs_thread, args=('rear', a.rear), daemon=True),
      threading.Thread(target=uvc_thread, args=('arm', a.arm), daemon=True)]
for t in th: t.start()
time.sleep(3.0)  # 카메라 켜짐 · 자동 노출 안정
cams = ['front', 'rear', 'arm']; period = 1.0 / a.rate; nxt = {c: time.monotonic() for c in cams}; used = {c: 0 for c in cams}
f = open(os.path.join(OUT, 'frames.csv'), 'w', newline=''); w = csv.writer(f)
w.writerow(['cam', 'frame_no', 't_recv', 't_start', 't_end', 'infer_ms', 'age_ms', 'n_det', 'stale'])
t_begin = time.monotonic(); print(f'측정 시작 {OUT}, {a.seconds:.0f} 초', flush=True); last_print = t_begin
while time.monotonic() - t_begin < a.seconds and not stop.is_set():
    now = time.monotonic(); c = min(cams, key=lambda k: nxt[k])
    if nxt[c] > now: time.sleep(min(nxt[c] - now, 0.005)); continue
    nxt[c] += period
    with lock: item = latest.get(c)
    if item is None or item[1] == used[c]:
        w.writerow([c, item[1] if item else -1, '', '', '', '', '', '', 1]); continue
    t_recv, fno, img = item; used[c] = fno; t0 = time.monotonic()
    x, s, px, py = letterbox(img); raw = det.infer(x); D = decode(raw, s, px, py); t1 = time.monotonic()
    w.writerow([c, fno, f'{t_recv:.4f}', f'{t0:.4f}', f'{t1:.4f}', f'{(t1 - t0) * 1000:.2f}', f'{(t1 - t_recv) * 1000:.2f}', len(D), 0])
    if t1 - last_print > 30:
        f.flush(); last_print = t1; print(f'{t1 - t_begin:.0f} 초 지남 | 받은 장 ' + ' '.join(f'{k} {stats.get(k, {}).get("frames", 0)}' for k in cams), flush=True)
stop.set()
for t in th: t.join(timeout=7.0)  # 카메라 스레드가 pipeline.stop · cap.release 를 끝낸 뒤 종료(11-03 30분 소크 뒤 종료 때 segfault — 스레드가 돌던 채로 인터프리터가 내려감)
teg.terminate(); f.close()
dur = time.monotonic() - t_begin
json.dump({'engine': a.engine, 'seconds': round(dur, 1), 'rate_target_hz': a.rate, 'cams': stats, 'errors': errors,
           'tensorrt': trt.__version__, 'note': '결과 나이는 받은 뒤부터(노출 · USB 전송 제외)'}, open(os.path.join(OUT, 'summary.json'), 'w'), ensure_ascii=False, indent=1)
print('끝', OUT, '오류' if errors else '', errors, flush=True)
