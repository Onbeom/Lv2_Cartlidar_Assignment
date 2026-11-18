"""팀 TensorRTDetector(이식한 후처리, PC 엔진) ↔ ONNX Runtime CPU + decode_cli(YOLO 세션 후처리) 동등성, 실물 203장.
기준은 261031_0100_trt 와 같음: 장마다 클래스 · 개수 같음 203/203, 짝 지은 상자 차 ≤ 0.5 px, 점수 차 ≤ 1e-3.
사용: ~/yolo-venv/bin/python -I compare.py <trt_team.jsonl> <출력 폴더>"""
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path('/home/pa4/git/lv2_next_module')
ONNX = ROOT / '261030_1325/prototype/yolo_models/yolo11n_task4_B.onnx'
DECODE = ROOT / '261030_1325/yolo_cpp/decode_cli'
team = {json.loads(l)['image']: json.loads(l)['dets'] for l in Path(sys.argv[1]).read_text().splitlines()}
out = Path(sys.argv[2])
opt = ort.SessionOptions()
opt.intra_op_num_threads = 2
opt.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
sess = ort.InferenceSession(str(ONNX), sess_options=opt, providers=['CPUExecutionProvider'])
iname = sess.get_inputs()[0].name
rows = []
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
    r = subprocess.run([str(DECODE), str(ref.shape[1]), str(ref.shape[2]), str(w), str(h), str(s), str(px), str(py)],
                       input=ref.astype(np.float32).tobytes(), capture_output=True, check=True)
    da, db = json.loads(r.stdout), team[str(p)]
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
    rows.append({'image': p.name, 'n_ref': len(da), 'n_team': len(db), 'same_classes_counts': same,
                 'max_box_px': box, 'max_score': score})
summary = {'pc_only_not_jetson': True, 'onnx': str(ONNX), 'n_images': len(rows),
           'same_classes_counts': sum(r['same_classes_counts'] for r in rows),
           'max_box_px': max(r['max_box_px'] for r in rows), 'max_score': max(r['max_score'] for r in rows),
           'detections_ref': sum(r['n_ref'] for r in rows), 'detections_team': sum(r['n_team'] for r in rows),
           'criteria': {'same_all': True, 'box_px_le': 0.5, 'score_le': 1e-3}}
summary['pass'] = summary['same_classes_counts'] == len(rows) and summary['max_box_px'] <= 0.5 and summary['max_score'] <= 1e-3
(out / 'rows.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1))
(out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
print(json.dumps(summary, ensure_ascii=False))
