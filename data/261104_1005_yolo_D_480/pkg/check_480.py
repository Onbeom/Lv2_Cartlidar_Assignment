#!/usr/bin/env python3
"""Jetson: D 가중치를 카메라 화면 비율에 맞춘 입력(D435 480×640, Arducam 384×640)으로 만든 엔진(FP32 --noTF32 · FP16)을 시험한다(11-04).
- 이미지: FP16 시험 때 복사한 ~/fp16_check/images 를 다시 씀. 480 엔진 → D435 · 옛 평가 장, 384 엔진 → video_arm.
- 저장: dets_N32.json · dets_N16.json(두 엔진 결과를 합친 것, conf 0.10 · NMS 0.45), summary.json(trtexec · 파이썬 시간).
시스템 python3 로 실행: python3 check_480.py"""
import hashlib, json, re, statistics, subprocess, sys, time
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent; OUT = HERE / 'results'; OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
from trt_run import Detector, letterbox, decode  # noqa: E402  (입력 크기를 엔진에서 읽는 판)

TRTEXEC = '/usr/src/tensorrt/bin/trtexec'; IMG = Path.home() / 'fp16_check/images'
sh = lambda c: subprocess.run(c, shell=True, capture_output=True, text=True)
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
ONNX = {'480': ('yolo11n_task4_D_480.onnx', '6f2c873893c5410c'), '384': ('yolo11n_task4_D_384.onnx', '96ea26472d5c9333')}
assert IMG.is_dir(), f'{IMG} 없음 — FP16 시험 꾸러미의 images 가 필요'
rep = {'nvpmodel': sh('nvpmodel -q').stdout.strip(), 'engines': {}}
engines = {}
for size, (name, h) in ONNX.items():
    assert sha(HERE / name).startswith(h), f'{name} 해시가 PC 와 다름'
    for prec, flag in (('32', '--noTF32'), ('16', '--fp16')):
        e = OUT / f'yolo11n_task4_D_{size}_{"fp32_notf32" if prec == "32" else "fp16"}.engine'
        if not e.exists():
            b = sh(f'{TRTEXEC} --onnx={HERE / name} {flag} --saveEngine={e} --skipInference')
            (OUT / f'build_{size}_{prec}.log').write_text(b.stdout + b.stderr)
            assert b.returncode == 0, f'{e.name} 빌드 실패(results/build_{size}_{prec}.log)'
        engines[f'{size}_{prec}'] = e
        print('엔진', e.name, flush=True)

for k, p in engines.items():
    r = sh(f'{TRTEXEC} --loadEngine={p} --warmUp=2000 --duration=15')
    (OUT / f'trtexec_{k}.log').write_text(r.stdout + r.stderr)
    m = re.search(r'GPU Compute Time: min = ([\d.]+) ms, max = ([\d.]+) ms, mean = ([\d.]+) ms, median = ([\d.]+) ms', r.stdout)
    rep['engines'][k] = {'path': str(p), 'sha256': sha(p), 'bytes': p.stat().st_size,
                         'trtexec_gpu_ms': dict(zip(['min', 'max', 'mean', 'median'], map(float, m.groups()))) if m else None}
    print(k, 'trtexec', rep['engines'][k]['trtexec_gpu_ms'], flush=True)

imgs = sorted(IMG.glob('*/*.png'))
for prec in ('32', '16'):
    D = {}
    for size in ('480', '384'):
        k = f'{size}_{prec}'; det = Detector(str(engines[k])); x0 = np.zeros((1, 3, *det.hw), np.float32)
        for _ in range(30): det.infer(x0)
        sel = [f for f in imgs if (f.parent.name == 'video_arm') == (size == '384')]
        t_pre, t_inf, t_post = [], [], []
        for f in sel:
            img = cv2.imread(str(f))
            t0 = time.perf_counter(); x, s, px, py = letterbox(img, det.hw); t1 = time.perf_counter()
            raw = det.infer(x); t2 = time.perf_counter()
            out = decode(raw, s, px, py, conf=0.10, nms=0.45); t3 = time.perf_counter()
            D[f'{f.parent.name}/{f.name}'] = [[int(c), float(sc), *map(float, b)] for c, sc, b in out]
            t_pre.append((t1 - t0) * 1e3); t_inf.append((t2 - t1) * 1e3); t_post.append((t3 - t2) * 1e3)
        q = lambda v: {'median': round(statistics.median(v), 3), 'p95': round(float(np.percentile(v, 95)), 3)}
        rep['engines'][k]['python_ms'] = {'letterbox': q(t_pre), 'infer': q(t_inf), 'decode': q(t_post), 'n': len(sel)}
        print(k, 'python', rep['engines'][k]['python_ms'], flush=True)
        del det
    json.dump(D, open(OUT / f'dets_N{prec}.json', 'w'))
json.dump(rep, open(OUT / 'summary.json', 'w'), ensure_ascii=False, indent=1)
print('끝 →', OUT)
