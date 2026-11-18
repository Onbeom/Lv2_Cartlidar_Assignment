"""도구 확인(PC, CPU): 480×640 ONNX 를 ONNX Runtime 으로 FP16 시험과 같은 1478장에 돌려 dets_P480.json(conf 0.10 · NMS 0.45) — Jetson 전에 결과를 미리 봄."""
import json, sys
from pathlib import Path
import cv2, numpy as np, onnxruntime as ort
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE / 'pkg'))
import importlib.util
spec = importlib.util.spec_from_file_location('tr', HERE / 'pkg/trt_run.py')  # letterbox · decode 만 씀(tensorrt 는 import 안 되게 아래에서 막음)
sys.modules['tensorrt'] = type(sys)('tensorrt'); tr = importlib.util.module_from_spec(spec); spec.loader.exec_module(tr)
so = ort.SessionOptions(); so.intra_op_num_threads = 4
S = ort.InferenceSession(str(HERE / 'yolo11n_task4_D_480.onnx'), so, providers=['CPUExecutionProvider'])
IMG = Path('/home/pa4/git/lv2_next_module/261104_0941_fp16_check/pkg/images'); D = {}
for f in sorted(IMG.glob('*/*.png')):
    x, s, px, py = tr.letterbox(cv2.imread(str(f)), (480, 640))
    raw = S.run(None, {'images': x})[0]
    D[f'{f.parent.name}/{f.name}'] = [[int(c), float(sc), *map(float, b)] for c, sc, b in tr.decode(raw, s, px, py, conf=0.10, nms=0.45)]
json.dump(D, open(HERE / 'dets_P480.json', 'w')); print(len(D))
