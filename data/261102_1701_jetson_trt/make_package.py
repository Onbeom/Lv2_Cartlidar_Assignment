"""PC: Jetson TensorRT 동등성 확인 꾸러미(pkg/)를 만든다(11-02, 실물 주행 없음).
- 실물 평가 203장을 PC 에서 letterbox(114) 한 640×640 PNG(무손실)로 저장 → Jetson 은 이 화소를 그대로 /255 해서 넣음(양쪽 cv2 판 차이 배제).
- 모델마다 ONNX(그대로 복사) + ONNX Runtime CPU(그래프 최적화 끔) 기준 출력 raw [1,8,8400] float32 를 ref_<모델>.npz 로 저장.
사용: ~/yolo-venv/bin/python make_package.py B=<onnx> C=<onnx> [D=<onnx>]"""
import hashlib, json, shutil, sys
from pathlib import Path
import cv2, numpy as np, onnxruntime as ort

HERE = Path(__file__).resolve().parent; PKG = HERE / 'pkg'
EVAL = Path('/home/pa4/git/lv2_next_module/261030_1325/B_negative/images/eval')
(PKG / 'inputs').mkdir(parents=True, exist_ok=True)
meta = {'images': []}
for p in sorted(EVAL.glob('*.png')):
    img = cv2.imread(str(p)); h, w = img.shape[:2]; s = min(640 / w, 640 / h); nw, nh = int(w * s), int(h * s); px, py = (640 - nw) // 2, (640 - nh) // 2
    canvas = np.full((640, 640, 3), 114, np.uint8); canvas[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh))
    cv2.imwrite(str(PKG / 'inputs' / p.name), canvas)
    meta['images'].append({'name': p.name, 'w': w, 'h': h, 's': s, 'px': px, 'py': py})
sha = lambda f: hashlib.sha256(open(f, 'rb').read()).hexdigest()
meta['models'] = {}
for arg in sys.argv[1:]:
    name, onnx = arg.split('=', 1)
    dst = PKG / f'yolo11n_task4_{name}.onnx'; shutil.copyfile(onnx, dst)
    opt = ort.SessionOptions(); opt.intra_op_num_threads = 2; opt.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    sess = ort.InferenceSession(str(dst), sess_options=opt, providers=['CPUExecutionProvider']); iname = sess.get_inputs()[0].name
    ref = {}
    for im in meta['images']:
        c = cv2.imread(str(PKG / 'inputs' / im['name']))
        x = np.ascontiguousarray(c[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
        ref[im['name']] = sess.run(None, {iname: x})[0]
    np.savez_compressed(PKG / f'ref_{name}.npz', **ref)
    meta['models'][name] = {'onnx': dst.name, 'onnx_sha256': sha(dst), 'source': onnx, 'input_name': iname, 'ref': f'ref_{name}.npz',
                            'ref_runtime': f'onnxruntime {ort.__version__} CPU, graph optimization off'}
json.dump(meta, open(PKG / 'meta.json', 'w'), ensure_ascii=False, indent=1)
shutil.copyfile(HERE / 'jetson_check.py', PKG / 'jetson_check.py')
print({k: v['onnx_sha256'][:16] for k, v in meta['models'].items()}, len(meta['images']))
