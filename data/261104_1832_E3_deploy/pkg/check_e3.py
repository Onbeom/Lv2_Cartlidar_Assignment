#!/usr/bin/env python3
"""Jetson: E3 가중치로 D435용 480×640 · Arducam용 384×640 FP16 엔진(trtexec --fp16)을 만들고 시험한다(11-04 E3, 배포 전환 전 확인 — check_e2.py 와 같은 방식).
- 이미지: FP16 시험 때 복사한 ~/fp16_check/images 1478장 + E2 시험 때 보낸 ~/e2_check/images/arena_*(경기장 판정 686장). 이름이 *_arm 인 폴더는 384 엔진, 나머지는 480 엔진.
- 저장: results/dets_E3J16.json(conf 0.10 · NMS 0.45), results/summary.json(trtexec GPU 시간 · 파이썬 한 장 시간). 판정 · 비교는 PC.
시스템 python3 로 실행: python3 check_e2.py"""
import hashlib, json, re, statistics, subprocess, sys, time
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent; OUT = HERE / 'results'; OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(HERE))
from trt_run import Detector, letterbox, decode  # noqa: E402  (입력 크기를 엔진에서 읽는 판)

TRTEXEC = '/usr/src/tensorrt/bin/trtexec'
sh = lambda c: subprocess.run(c, shell=True, capture_output=True, text=True)
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
ONNX = {'480': ('yolo11n_task4_E3_480.onnx', 'd82ab8670d014722'), '384': ('yolo11n_task4_E3_384.onnx', 'd0ced66803458092')}
dirs = sorted(p for p in (Path.home() / 'fp16_check/images').iterdir() if p.is_dir()) + sorted(p for p in (Path.home() / 'e2_check/images').iterdir() if p.is_dir())
rep = {'nvpmodel': sh('nvpmodel -q').stdout.strip(), 'engines': {}}; engines = {}
for size, (name, h) in ONNX.items():
    assert sha(HERE / name).startswith(h), f'{name} 해시가 PC 와 다름'
    e = OUT / f'yolo11n_task4_E3_{size}_fp16.engine'
    if not e.exists() or e.stat().st_size == 0:  # 11-04: 중간에 끊기면 빈 엔진 파일이 남음 → 다시 빌드
        b = sh(f'{TRTEXEC} --onnx={HERE / name} --fp16 --saveEngine={e} --skipInference')
        (OUT / f'build_{size}.log').write_text(b.stdout + b.stderr)
        if b.returncode != 0 or not e.exists() or e.stat().st_size == 0:
            e.unlink(missing_ok=True); sys.exit(f'{e.name} 빌드 실패 — results/build_{size}.log 끝:\n' + (b.stdout + b.stderr)[-1500:])
    engines[size] = e; print('엔진', e.name, flush=True)
    r = sh(f'{TRTEXEC} --loadEngine={e} --warmUp=2000 --duration=15')
    (OUT / f'trtexec_{size}.log').write_text(r.stdout + r.stderr)
    m = re.search(r'GPU Compute Time: min = ([\d.]+) ms, max = ([\d.]+) ms, mean = ([\d.]+) ms, median = ([\d.]+) ms', r.stdout)
    if not m: print('trtexec 시간 재기 실패 — results/trtexec_' + size + '.log 끝:\n' + (r.stdout + r.stderr)[-800:], flush=True)
    rep['engines'][size] = {'path': str(e), 'sha256': sha(e), 'trtexec_gpu_ms': dict(zip(['min', 'max', 'mean', 'median'], map(float, m.groups()))) if m else None}
    print(size, 'trtexec', rep['engines'][size]['trtexec_gpu_ms'], flush=True)
D = {}
for size in ('480', '384'):
    det = Detector(str(engines[size])); x0 = np.zeros((1, 3, *det.hw), np.float32)
    for _ in range(30): det.infer(x0)
    t_inf = []
    for d in dirs:
        if d.name.endswith('_arm') != (size == '384'):
            continue
        for f in sorted(d.glob('*.png')):
            x, s, px, py = letterbox(cv2.imread(str(f)), det.hw); t1 = time.perf_counter(); raw = det.infer(x); t_inf.append((time.perf_counter() - t1) * 1e3)
            D[f'{d.name}/{f.name}'] = [[int(c), float(sc), *map(float, b)] for c, sc, b in decode(raw, s, px, py, conf=0.10, nms=0.45)]
    rep['engines'][size]['python_infer_ms'] = {'median': round(statistics.median(t_inf), 3), 'n': len(t_inf)}
    print(size, 'python infer', rep['engines'][size]['python_infer_ms'], flush=True)
    del det
json.dump(D, open(OUT / 'dets_E3J16.json', 'w')); json.dump(rep, open(OUT / 'summary.json', 'w'), ensure_ascii=False, indent=1)
print('끝 →', OUT, len(D), '장')
