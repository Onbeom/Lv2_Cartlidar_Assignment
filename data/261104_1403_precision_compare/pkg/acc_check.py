#!/usr/bin/env python3
"""Jetson: 같은 D 가중치 · 같은 정사각 640 입력의 FP32 · FP16 · INT8(Q/DQ) · INT8+FP16 엔진을 같은 장에 돌려 검출을 저장한다(11-04, 정밀도별 정확도 비교).
- INT8+FP16: 교안 INT8 Q/DQ ONNX 를 --int8 --fp16 으로 빌드(Q/DQ 밖 층을 FP16 으로). 안 되면 --fp16 만.
- 장: ~/fp16_check/images(1478) + ~/e2_check/images/arena_*(경기장 686, 있으면).
- 저장: results/dets_<이름>.json(conf 0.10 · NMS 0.45), results/acc_summary.json(엔진 · trtexec GPU 시간). 사용: python3 acc_check.py"""
import hashlib, json, re, subprocess, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent; OUT = HERE / 'results'; OUT.mkdir(exist_ok=True); sys.path.insert(0, str(HERE))
from trt_run import Detector, letterbox, decode  # noqa: E402
TRTEXEC = '/usr/src/tensorrt/bin/trtexec'; H = Path.home()
sh = lambda c: subprocess.run(c, shell=True, capture_output=True, text=True)
E = {'P32': H / 'yolo_D/deploy/yolo11n_task4_D_fp32_notf32.engine', 'P16': H / 'fp16_check/results/yolo11n_task4_D_fp16.engine',
     'I8': H / 'fp16_check/int8/yolo11n_task4_D_int8_qdq.engine', 'I8F16': OUT / 'yolo11n_task4_D_int8_qdq_fp16.engine'}
if not E['I8F16'].exists() or E['I8F16'].stat().st_size == 0:
    q = H / 'fp16_check/int8/yolo11n_task4_D_int8_qdq.onnx'; b = sh(f'{TRTEXEC} --onnx={q} --int8 --fp16 --saveEngine={E["I8F16"]} --skipInference')
    if b.returncode != 0:
        b = sh(f'{TRTEXEC} --onnx={q} --fp16 --saveEngine={E["I8F16"]} --skipInference')
    (OUT / 'build_I8F16.log').write_text(b.stdout + b.stderr); assert b.returncode == 0, 'INT8+FP16 빌드 실패(results/build_I8F16.log)'
for k, p in E.items(): assert p.exists(), f'{k} 엔진 없음: {p}'
dirs = sorted(p for p in (H / 'fp16_check/images').iterdir() if p.is_dir()) + sorted((H / 'e2_check/images').glob('arena_*'))
rep = {}
for k, p in E.items():
    r = sh(f'{TRTEXEC} --loadEngine={p} --warmUp=2000 --duration=15'); (OUT / f'trtexec_{k}.log').write_text(r.stdout + r.stderr)
    m = re.search(r'GPU Compute Time: min = ([\d.]+) ms, max = ([\d.]+) ms, mean = ([\d.]+) ms, median = ([\d.]+) ms', r.stdout)
    rep[k] = {'engine': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'trtexec_gpu_median_ms': float(m.group(4)) if m else None}
    det = Detector(str(p)); D = {}
    for d in dirs:
        for f in sorted(d.glob('*.png')):
            x, s, px, py = letterbox(cv2.imread(str(f)), det.hw)
            D[f'{d.name}/{f.name}'] = [[int(c), float(sc), *map(float, b)] for c, sc, b in decode(det.infer(x), s, px, py, conf=0.10, nms=0.45)]
    json.dump(D, open(OUT / f'dets_{k}.json', 'w')); rep[k]['images'] = len(D); print(k, rep[k], flush=True); del det
json.dump(rep, open(OUT / 'acc_summary.json', 'w'), ensure_ascii=False, indent=1); print('정확도 끝')
