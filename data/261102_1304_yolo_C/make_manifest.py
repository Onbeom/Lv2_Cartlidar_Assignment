"""후보 C manifest: 해시 · 입출력 모양 · 유한값(실물 203장 ONNX 출력) · 동등성 결과 모음. PC, CPU."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import onnx
import onnxruntime as ort

ROOT = Path('/home/pa4/git/lv2_next_module'); HERE = Path(__file__).resolve().parent
EX = HERE / 'yolo_export'; ONNX = EX / 'yolo11n_task4_C.onnx'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
opt = ort.SessionOptions(); opt.intra_op_num_threads = 2; opt.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
sess = ort.InferenceSession(str(ONNX), sess_options=opt, providers=['CPUExecutionProvider'])
nonfinite = 0; n = 0
for p in sorted((ROOT / '261030_1325/B_negative/images/eval').glob('*.png')):
    img = cv2.imread(str(p)); h, w = img.shape[:2]; s = min(640 / w, 640 / h); nw, nh = int(w * s), int(h * s); px, py = (640 - nw) // 2, (640 - nh) // 2
    c = np.full((640, 640, 3), 114, np.uint8); c[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh))
    y = sess.run(None, {sess.get_inputs()[0].name: np.ascontiguousarray(c[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255})[0]
    nonfinite += int((~np.isfinite(y)).sum()); n += 1
g = onnx.load(str(ONNX)); pe = json.loads((EX / 'pt_onnx_equivalence.json').read_text()); tr = json.loads((HERE / 'trt_results/summary.json').read_text())
m = {'model': 'YOLO 후보 C(yolo11n, 4클래스, B 자료 + 튜터 154225 1500장, 80 epoch, last.pt)', 'created': '2026-11-02',
     'source_checkpoint': str(ROOT / '261102_1141_tutor_yolo/runs/C_tutor/weights/last.pt'),
     'classes': (EX / 'classes.txt').read_text().split(), 'class_order_same_as_B': True,
     'input': {'name': sess.get_inputs()[0].name, 'shape': sess.get_inputs()[0].shape, 'dtype': 'float32', 'layout': 'NCHW RGB /255, 정사각 letterbox 114'},
     'output': {'name': sess.get_outputs()[0].name, 'shape': sess.get_outputs()[0].shape, 'dtype': 'float32', 'layout': '[cx,cy,w,h(640 px), 4 클래스 점수], NMS 없음'},
     'onnx': {'opset': [o.version for o in g.opset_import if o.domain in ('', 'ai.onnx')][0], 'dynamic': False, 'nms': False, 'fp': 32, 'checker': True},
     'thresholds_used_in_eval': {'score': 0.25, 'nms_iou': 0.45, 'note': '설정 yolo_task4_C.yaml(B 와 같음). 등록부 D435 문턱 0.6 은 인지 쪽 값'},
     'finite_check': {'images': n, 'nonfinite_values': nonfinite, 'pass': nonfinite == 0},
     'pt_onnx_cpu': {k: pe[k] for k in ('n_images', 'n_pass', 'pass', 'max_box_px', 'max_score_abs')} | {'criteria': 'box ≤ 0.01 px · score ≤ 1e-4 · 개수 · 클래스 순서 · IoU ≥ 0.999'},
     'trt_pc_fp32_notf32': {k: tr[k] for k in ('n_images', 'same_classes_counts', 'max_box_px', 'max_score', 'raw_max_abs', 'detections_onnx', 'criteria', 'pass')}
                          | {'tensorrt': '10.16.1', 'gpu': 'RTX 5060 Laptop(PC)', 'jetson': '미검증(대상 GPU 에서 엔진 다시 빌드 · 같은 방식 확인 필요)', 'fp16': '미시험'},
     'sha256': {str(p.relative_to(HERE)): sha(p) for p in [EX / 'yolo11n_task4_C.pt', ONNX, EX / 'classes.txt', HERE / 'yolo_task4_C.yaml',
                                                          HERE / 'trt_results/yolo11n_task4_C_fp32_notf32.engine', HERE / 'export_c.py', HERE / 'check_trt.py', HERE / 'make_manifest.py']},
     'cautions': ['실물 평가 203장에서 맞힌 상자 중 등록부 문턱 0.6 밑 2개(0.538 · 0.549, B 는 0개)',
                  '0.6 넘는 헛검출 1개(0.814, B 는 2개 0.697 · 0.72)',
                  '실물 1.0 m 안쪽 · 팔 카메라(Arducam) 성능 미검증 — 11-02 새 실물 근접 사진으로 평가 예정',
                  'PC 엔진은 Jetson 에서 쓸 수 없음']}
(HERE / 'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2))
print(json.dumps({'finite': m['finite_check'], 'onnx_sha': m['sha256']['yolo_export/yolo11n_task4_C.onnx']}, ensure_ascii=False))
