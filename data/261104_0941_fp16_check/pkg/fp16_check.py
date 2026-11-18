#!/usr/bin/env python3
"""Jetson: 모델 D 의 FP16 엔진을 만들고 FP32(--noTF32) 배포 엔진과 같은 입력 · 같은 전처리 · 후처리로 비교한다(11-04).
- 엔진: trtexec --fp16 (FP16 허용 · 혼합 정밀도). FP32 엔진은 이미 배포한 것(--fp32 로 경로 지정).
- 속도: trtexec 순수 GPU 계산 시간(워밍업 2 s, 15 s) + 파이썬 한 장 처리(letterbox · infer · decode) 시간.
- 검출: images/<카메라>/*.png 모두 conf 0.10 · 클래스별 NMS 0.45(PC dets.py 와 같은 저장 꼴) → dets_J32.json · dets_J16.json. 판정은 PC judge.py.
시스템 python3(tensorrt · cv2 · numpy)로 실행: python3 fp16_check.py [--fp32 <엔진>]"""
import argparse, hashlib, json, re, statistics, subprocess, sys, time
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent; OUT = HERE / 'results'; OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
from trt_run import Detector, letterbox, decode  # noqa: E402  (배포 실행기와 같은 코드)

TRTEXEC = '/usr/src/tensorrt/bin/trtexec'
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--fp32', default=str(Path.home() / 'yolo_D/deploy/yolo11n_task4_D_fp32_notf32.engine'))
a = ap.parse_args()
sh = lambda c: subprocess.run(c, shell=True, capture_output=True, text=True)
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
onnx = HERE / 'yolo11n_task4_D.onnx'
assert sha(onnx).startswith('eeb0d6bcb6ef7063'), 'ONNX 해시가 PC D 와 다름'
assert Path(a.fp32).exists(), f'FP32 엔진 없음: {a.fp32}'
board = {'nvpmodel': sh('nvpmodel -q').stdout.strip(), 'trtexec': sh(f'{TRTEXEC} --version 2>&1 | head -2').stdout.strip()}

e16 = OUT / 'yolo11n_task4_D_fp16.engine'
if not e16.exists():
    b = sh(f'{TRTEXEC} --onnx={onnx} --fp16 --saveEngine={e16} --skipInference')
    (OUT / 'build_fp16.log').write_text(b.stdout + b.stderr)
    assert b.returncode == 0, 'FP16 엔진 빌드 실패(results/build_fp16.log)'
engines = {'J32': Path(a.fp32), 'J16': e16}
rep = {'board': board, 'engines': {k: {'path': str(p), 'sha256': sha(p), 'bytes': p.stat().st_size} for k, p in engines.items()}}

# 1) trtexec 순수 GPU 계산 시간
for k, p in engines.items():
    r = sh(f'{TRTEXEC} --loadEngine={p} --warmUp=2000 --duration=15')
    (OUT / f'trtexec_{k}.log').write_text(r.stdout + r.stderr)
    m = re.search(r'GPU Compute Time: min = ([\d.]+) ms, max = ([\d.]+) ms, mean = ([\d.]+) ms, median = ([\d.]+) ms, percentile\(90%\) = ([\d.]+) ms, '
                  r'percentile\(95%\) = ([\d.]+) ms, percentile\(99%\) = ([\d.]+) ms', r.stdout)
    t = re.search(r'Throughput: ([\d.]+) qps', r.stdout)
    rep['engines'][k]['trtexec_gpu_ms'] = dict(zip(['min', 'max', 'mean', 'median', 'p90', 'p95', 'p99'], map(float, m.groups()))) if m else None
    rep['engines'][k]['trtexec_qps'] = float(t.group(1)) if t else None
    print(k, 'trtexec', rep['engines'][k]['trtexec_gpu_ms'], flush=True)

# 2) 같은 장 검출 + 파이썬 한 장 처리 시간(사진 읽기 제외)
imgs = sorted((HERE / 'images').glob('*/*.png'))
print('장 수', len(imgs), flush=True)
for k, p in engines.items():
    det = Detector(str(p)); x0 = np.zeros((1, 3, 640, 640), np.float32)
    for _ in range(30): det.infer(x0)  # 워밍업
    D, t_pre, t_inf, t_post = {}, [], [], []
    for f in imgs:
        img = cv2.imread(str(f))
        t0 = time.perf_counter(); x, s, px, py = letterbox(img); t1 = time.perf_counter()
        raw = det.infer(x); t2 = time.perf_counter()
        out = decode(raw, s, px, py, conf=0.10, nms=0.45); t3 = time.perf_counter()
        D[f'{f.parent.name}/{f.name}'] = [[int(c), float(sc), *map(float, b)] for c, sc, b in out]
        t_pre.append((t1 - t0) * 1e3); t_inf.append((t2 - t1) * 1e3); t_post.append((t3 - t2) * 1e3)
    json.dump(D, open(OUT / f'dets_{k}.json', 'w'))
    q = lambda v: {'median': round(statistics.median(v), 3), 'p95': round(float(np.percentile(v, 95)), 3), 'max': round(max(v), 3)}
    rep['engines'][k]['python_ms'] = {'letterbox': q(t_pre), 'infer': q(t_inf), 'decode': q(t_post), 'n': len(imgs)}
    print(k, 'python', rep['engines'][k]['python_ms'], flush=True)
    del det
json.dump(rep, open(OUT / 'summary.json', 'w'), ensure_ascii=False, indent=1)
print('끝 →', OUT)
