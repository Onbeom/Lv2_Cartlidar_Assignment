"""후보 C ONNX → PC TensorRT FP32(--noTF32) 엔진 vs ONNX Runtime CPU 동등성(203장). GPU 잠금은 run_trt.sh 에서.
사용: ~/yolo-venv/bin/python check_trt.py"""
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path('/home/pa4/git/lv2_next_module')
HERE = Path(__file__).resolve().parent
OUT = HERE / 'trt_results'
OUT.mkdir(exist_ok=False)
ONNX = HERE / 'yolo_export/yolo11n_task4_C.onnx'
DECODE = ROOT / '261030_1325/yolo_cpp/decode_cli'
TRTEXEC = '/usr/src/tensorrt/bin/trtexec'
ENGINE = OUT / 'yolo11n_task4_C_fp32_notf32.engine'
build = subprocess.run([TRTEXEC, f'--onnx={ONNX}', '--noTF32', f'--saveEngine={ENGINE}', '--skipInference'], capture_output=True, text=True)
(OUT / 'trtexec_build.log').write_text(build.stdout + build.stderr)
assert build.returncode == 0, 'trtexec 빌드 실패'
opt = ort.SessionOptions()
opt.intra_op_num_threads = 2
opt.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
sess = ort.InferenceSession(str(ONNX), sess_options=opt, providers=['CPUExecutionProvider'])
iname = sess.get_inputs()[0].name


def decode(raw, w, h, s, px, py):
    r = subprocess.run([str(DECODE), str(raw.shape[1]), str(raw.shape[2]), str(w), str(h), str(s), str(px), str(py)],
                       input=raw.astype(np.float32).tobytes(), capture_output=True, check=True)
    return json.loads(r.stdout)


rows = []
tmp = OUT / 'tmp'
tmp.mkdir()
for p in sorted((ROOT / '261030_1325/B_negative/images/eval').glob('*.png')):
    img = cv2.imread(str(p))
    h, w = img.shape[:2]
    s = min(640 / w, 640 / h)
    nw, nh = int(w * s), int(h * s)
    px, py = (640 - nw) // 2, (640 - nh) // 2
    canvas = np.full((640, 640, 3), 114, np.uint8)
    canvas[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh))
    x = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
    ref = sess.run(None, {iname: x})[0]
    (tmp / 'in.bin').write_bytes(x.tobytes())
    run = subprocess.run([TRTEXEC, f'--loadEngine={ENGINE}', f"--loadInputs='{iname}':{tmp / 'in.bin'}", f'--exportOutput={tmp / "out.json"}',
                          '--iterations=1', '--warmUp=0', '--duration=0', '--avgRuns=1'], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout[-2000:] + run.stderr[-2000:]
    o = json.loads((tmp / 'out.json').read_text())[0]
    trt = np.array(o['values'], np.float32).reshape(ref.shape)
    da, db = decode(ref, w, h, s, px, py), decode(trt, w, h, s, px, py)
    same = len(da) == len(db) and sorted(d['class'] for d in da) == sorted(d['class'] for d in db)
    box, score = 0.0, 0.0
    if same:
        used = set()
        for a in da:
            j = min((k for k in range(len(db)) if k not in used and db[k]['class'] == a['class']),
                    key=lambda k: max(abs(u - v) for u, v in zip(a['box'], db[k]['box'])))
            used.add(j)
            box = max(box, max(abs(u - v) for u, v in zip(a['box'], db[j]['box'])))
            score = max(score, abs(a['score'] - db[j]['score']))
    rows.append({'image': p.name, 'n_onnx': len(da), 'n_trt': len(db), 'same_classes_counts': same, 'max_box_px': box, 'max_score': score,
                 'raw_max_abs': float(np.abs(ref - trt).max()), 'raw_name': o.get('name')})
summary = {'pc_only_not_jetson': True, 'onnx': str(ONNX), 'engine': str(ENGINE), 'n_images': len(rows),
           'same_classes_counts': sum(r['same_classes_counts'] for r in rows),
           'max_box_px': max(r['max_box_px'] for r in rows), 'max_score': max(r['max_score'] for r in rows),
           'raw_max_abs': max(r['raw_max_abs'] for r in rows), 'detections_onnx': sum(r['n_onnx'] for r in rows),
           'criteria': {'same_all': True, 'box_px_le': 0.5, 'score_le': 1e-3}}
summary['pass'] = (summary['same_classes_counts'] == len(rows) and summary['max_box_px'] <= 0.5 and summary['max_score'] <= 1e-3)
(OUT / 'rows.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1))
(OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
print(json.dumps(summary, ensure_ascii=False))
