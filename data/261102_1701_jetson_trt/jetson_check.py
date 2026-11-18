"""Jetson: 꾸러미의 ONNX 마다 이 보드에서 TensorRT FP32(--noTF32) 엔진을 만들고, 203장 출력을 PC ONNX Runtime 기준과 비교한다(11-02).
판정 기준(B · C PC 때와 같음): 모든 장에서 검출 수 · 클래스 같음, 상자 차 ≤ 0.5 px, 점수 차 ≤ 1e-3. 후처리는 양쪽 같은 함수(conf 0.25, 클래스별 NMS 0.45).
보드 정보(버전 · 전력 모드)와 엔진 SHA256 을 result_<모델>.json · board.json 에 남긴다. 실행: python3 jetson_check.py [모델 이름 ...]"""
import hashlib, json, os, subprocess, sys
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent; OUT = HERE / 'jetson_results'; OUT.mkdir(exist_ok=True)
TRTEXEC = '/usr/src/tensorrt/bin/trtexec'
meta = json.load(open(HERE / 'meta.json'))
sh = lambda c: subprocess.run(c, shell=True, capture_output=True, text=True)
board = {k: sh(c).stdout.strip() + sh(c).stderr.strip()[:300] for k, c in {
    'uname_m': 'uname -m', 'os': 'grep PRETTY_NAME /etc/os-release', 'l4t': 'cat /etc/nv_tegra_release', 'nvpmodel': 'nvpmodel -q',
    'trtexec_version': f'{TRTEXEC} --version 2>&1 | head -5', 'python_tensorrt': 'python3 -c "import tensorrt as t; print(t.__version__)"',
    'jetpack': 'dpkg-query -W nvidia-jetpack 2>/dev/null', 'cv2': 'python3 -c "import cv2; print(cv2.__version__)"'}.items()}
help_txt = sh(f'{TRTEXEC} --help').stdout + sh(f'{TRTEXEC} --help').stderr
board['has_noTF32'] = '--noTF32' in help_txt
json.dump(board, open(OUT / 'board.json', 'w'), ensure_ascii=False, indent=1)
print(json.dumps(board, ensure_ascii=False, indent=1))
if not board['has_noTF32']:
    sys.exit('이 trtexec 에 --noTF32 가 없음 → 멈춤(TensorRT 판이 달라 명령을 바꿔야 함, board.json 을 PC 로 가져와 주세요)')


def decode(raw, im):
    p = raw[0].T; cls = p[:, 4:].argmax(1); sc = p[:, 4:].max(1); k = sc >= 0.25
    p, cls, sc = p[k], cls[k], sc[k]
    xyxy = np.stack([p[:, 0] - p[:, 2] / 2, p[:, 1] - p[:, 3] / 2, p[:, 0] + p[:, 2] / 2, p[:, 1] + p[:, 3] / 2], 1)
    out = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]; idx = idx[np.argsort(-sc[idx])]
        while len(idx):
            i = idx[0]; out.append((int(c), float(sc[i]), xyxy[i]))
            b = xyxy[idx[1:]]; ix = np.clip(np.minimum(b[:, 2], xyxy[i, 2]) - np.maximum(b[:, 0], xyxy[i, 0]), 0, None)
            iy = np.clip(np.minimum(b[:, 3], xyxy[i, 3]) - np.maximum(b[:, 1], xyxy[i, 1]), 0, None); inter = ix * iy
            u = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1]) + (xyxy[i, 2] - xyxy[i, 0]) * (xyxy[i, 3] - xyxy[i, 1]) - inter
            idx = idx[1:][inter / np.maximum(u, 1e-9) <= 0.45]
    return [(c, s, [(b[0] - im['px']) / im['s'], (b[1] - im['py']) / im['s'], (b[2] - im['px']) / im['s'], (b[3] - im['py']) / im['s']]) for c, s, b in out]


names = sys.argv[1:] or list(meta['models'])
for name in names:
    m = meta['models'][name]; onnx = HERE / m['onnx']
    assert hashlib.sha256(open(onnx, 'rb').read()).hexdigest() == m['onnx_sha256'], 'ONNX 해시가 PC 와 다름'
    engine = OUT / f'yolo11n_task4_{name}_fp32_notf32.engine'
    b = sh(f'{TRTEXEC} --onnx={onnx} --noTF32 --saveEngine={engine} --skipInference')
    (OUT / f'build_{name}.log').write_text(b.stdout + b.stderr)
    assert b.returncode == 0, f'{name} 엔진 빌드 실패(build_{name}.log)'
    ref = np.load(HERE / m['ref']); rows = []
    for im in meta['images']:
        c = cv2.imread(str(HERE / 'inputs' / im['name']), cv2.IMREAD_UNCHANGED)
        x = np.ascontiguousarray(c[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255
        (OUT / 'in.bin').write_bytes(x.tobytes())
        r = sh(f"{TRTEXEC} --loadEngine={engine} --loadInputs='{m['input_name']}':{OUT / 'in.bin'} --exportOutput={OUT / 'out.json'} "
               f"--iterations=1 --warmUp=0 --duration=0 --avgRuns=1")
        assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-1500:]
        o = json.load(open(OUT / 'out.json'))[0]; a = ref[im['name']]; t = np.array(o['values'], np.float32).reshape(a.shape)
        da, db = decode(a, im), decode(t, im)
        same = len(da) == len(db) and sorted(d[0] for d in da) == sorted(d[0] for d in db); box = score = 0.0
        if same:
            used = set()
            for ca, sa, ba in da:
                j = min((k for k in range(len(db)) if k not in used and db[k][0] == ca), key=lambda k: max(abs(u - v) for u, v in zip(ba, db[k][2])))
                used.add(j); box = max(box, max(abs(u - v) for u, v in zip(ba, db[j][2]))); score = max(score, abs(sa - db[j][1]))
        rows.append({'image': im['name'], 'n_ref': len(da), 'n_trt': len(db), 'same': same, 'max_box_px': box, 'max_score': score, 'raw_max_abs': float(np.abs(a - t).max())})
    s = {'model': name, 'onnx_sha256': m['onnx_sha256'], 'engine_sha256': hashlib.sha256(open(engine, 'rb').read()).hexdigest(),
         'n_images': len(rows), 'same': sum(r['same'] for r in rows), 'detections_ref': sum(r['n_ref'] for r in rows),
         'max_box_px': max(r['max_box_px'] for r in rows), 'max_score': max(r['max_score'] for r in rows), 'raw_max_abs': max(r['raw_max_abs'] for r in rows),
         'criteria': {'same_all': True, 'box_px_le': 0.5, 'score_le': 1e-3}, 'ref_runtime': m['ref_runtime']}
    s['pass'] = s['same'] == len(rows) and s['max_box_px'] <= 0.5 and s['max_score'] <= 1e-3
    json.dump({'summary': s, 'rows': rows}, open(OUT / f'result_{name}.json', 'w'), ensure_ascii=False, indent=1)
    print(json.dumps(s, ensure_ascii=False))
os.remove(OUT / 'in.bin') if (OUT / 'in.bin').exists() else None
