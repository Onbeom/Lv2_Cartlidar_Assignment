#!/usr/bin/env python3
"""Jetson: TensorRT 엔진(.engine, FP32 · FP16, 정적 1×3×H×W → 1×8×N, NMS 없음 — 입력 크기는 엔진에서 읽음(11-04: 480×640 엔진 추가))으로 YOLO 퍽 검출을 돌린다. torch · pycuda 없이 tensorrt + libcudart(ctypes) + OpenCV 만 씀.
전처리 · 후처리는 학습 · 평가와 같음: letterbox(114) · RGB · /255, conf 0.25(표시 문턱 --show 0.6) · 클래스별 NMS 0.45.
입력(--source): 이미지 폴더(*.png · *.jpg), 영상 장치 번호/경로(V4L2, Arducam 등), 'rs:<시리얼>'(pyrealsense2 컬러, .venv 파이썬으로 실행).
화면 창에 상자 · 점수 · fps 를 그림(q 끝). --no-window 면 창 없이 장마다 결과 한 줄 + 끝에 평균 지연 출력.
--save <폴더> 면 상자 그린 사진을 저장(yolo predict 와 같은 쓰임).
예: python3 trt_run.py --engine ~/trt_check/jetson_results/yolo11n_task4_D_fp32_notf32.engine --source ~/capture/video_judge/<폴더>/color"""
import argparse, ctypes, ctypes.util, glob, os, sys, time
import cv2, numpy as np
import tensorrt as trt

NAMES = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']; AB = ['RC', 'GC', 'RB', 'GB']
COL = [(0, 0, 255), (0, 200, 0), (0, 0, 160), (0, 120, 0)]


class Cuda:
    def __init__(self):
        cands = sorted(glob.glob('/usr/local/cuda/lib64/libcudart.so*')) + [ctypes.util.find_library('cudart')]
        path = next((p for p in cands if p), None)
        assert path, 'libcudart 를 못 찾음(/usr/local/cuda/lib64)'
        self.lib = ctypes.CDLL(path)
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]

    def check(self, r, what):
        if r != 0:
            raise RuntimeError(f'{what} 실패(cuda 오류 {r})')

    def malloc(self, n):
        p = ctypes.c_void_p(); self.check(self.lib.cudaMalloc(ctypes.byref(p), n), 'cudaMalloc'); return p

    def h2d(self, d, a):
        self.check(self.lib.cudaMemcpy(d, a.ctypes.data_as(ctypes.c_void_p), a.nbytes, 1), 'cudaMemcpy H2D')

    def d2h(self, a, d):
        self.check(self.lib.cudaMemcpy(a.ctypes.data_as(ctypes.c_void_p), d, a.nbytes, 2), 'cudaMemcpy D2H')


class Detector:
    def __init__(self, engine_path):
        self.cu = Cuda(); log = trt.Logger(trt.Logger.WARNING)
        self.engine = trt.Runtime(log).deserialize_cuda_engine(open(engine_path, 'rb').read())
        assert self.engine is not None, '엔진 역직렬화 실패(이 보드 · TensorRT 판에서 만든 엔진인지 확인)'
        self.ctx = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.inp = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.out = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
        self.ishape, self.oshape = tuple(self.engine.get_tensor_shape(self.inp)), tuple(self.engine.get_tensor_shape(self.out))
        assert self.ishape[:2] == (1, 3) and self.oshape[:2] == (1, 8), (self.ishape, self.oshape)
        self.hw = self.ishape[2:]  # (H, W): letterbox 크기
        self.hout = np.empty(self.oshape, np.float32)
        self.din, self.dout = self.cu.malloc(int(np.prod(self.ishape)) * 4), self.cu.malloc(self.hout.nbytes)
        self.ctx.set_tensor_address(self.inp, self.din.value); self.ctx.set_tensor_address(self.out, self.dout.value)
        self.stream = ctypes.c_void_p(); self.cu.check(self.cu.lib.cudaStreamCreate(ctypes.byref(self.stream)), 'cudaStreamCreate')

    def infer(self, x):
        self.cu.h2d(self.din, np.ascontiguousarray(x, np.float32))
        assert self.ctx.execute_async_v3(self.stream.value), 'execute_async_v3 실패'
        self.cu.check(self.cu.lib.cudaStreamSynchronize(self.stream), 'cudaStreamSynchronize')
        self.cu.d2h(self.hout, self.dout); return self.hout


def letterbox(img, hw=(640, 640)):
    H, W = hw; h, w = img.shape[:2]; s = min(W / w, H / h); nw, nh = int(w * s), int(h * s); px, py = (W - nw) // 2, (H - nh) // 2
    c = np.full((H, W, 3), 114, np.uint8); c[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh))
    return np.ascontiguousarray(c[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255, s, px, py


def decode(raw, s, px, py, conf=0.25, nms=0.45):
    p = raw[0].T; cls = p[:, 4:].argmax(1); sc = p[:, 4:].max(1); k = sc >= conf; p, cls, sc = p[k], cls[k], sc[k]
    xyxy = np.stack([p[:, 0] - p[:, 2] / 2, p[:, 1] - p[:, 3] / 2, p[:, 0] + p[:, 2] / 2, p[:, 1] + p[:, 3] / 2], 1); out = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]; idx = idx[np.argsort(-sc[idx])]
        while len(idx):
            i = idx[0]; out.append((int(c), float(sc[i]), xyxy[i])); b = xyxy[idx[1:]]
            ix = np.clip(np.minimum(b[:, 2], xyxy[i, 2]) - np.maximum(b[:, 0], xyxy[i, 0]), 0, None); iy = np.clip(np.minimum(b[:, 3], xyxy[i, 3]) - np.maximum(b[:, 1], xyxy[i, 1]), 0, None)
            u = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1]) + (xyxy[i, 2] - xyxy[i, 0]) * (xyxy[i, 3] - xyxy[i, 1]) - ix * iy
            idx = idx[1:][ix * iy / np.maximum(u, 1e-9) <= nms]
    return [(c, sc_, [(b[0] - px) / s, (b[1] - py) / s, (b[2] - px) / s, (b[3] - py) / s]) for c, sc_, b in out]


def frames(src):
    if os.path.isdir(src):
        for f in sorted(glob.glob(os.path.join(src, '*.png')) + glob.glob(os.path.join(src, '*.jpg'))):
            yield os.path.basename(f), cv2.imread(f)
    elif src.startswith('rs:'):
        try:
            import pyrealsense2 as rs
        except ImportError:  # 시스템 파이썬에 없으면 촬영용 가상환경(~/capture/.venv)의 것을 뒤에 붙임(시스템 numpy · cv2 는 그대로)
            sys.path += glob.glob(os.path.expanduser('~/capture/.venv/lib/python3*/site-packages')); import pyrealsense2 as rs
        pipe, cfg = rs.pipeline(), rs.config(); cfg.enable_device(src[3:]); cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30); pipe.start(cfg)
        try:
            n = 0
            while True:
                f = pipe.wait_for_frames().get_color_frame(); n += 1; yield f'rs{n}', np.asanyarray(f.get_data()).copy()
        finally:
            pipe.stop()
    else:
        cap = cv2.VideoCapture(int(src) if src.isdigit() else src, cv2.CAP_V4L2); cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720); n = 0
        while True:
            ok, img = cap.read()
            if not ok: break
            n += 1; yield f'cam{n}', img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--engine', required=True); ap.add_argument('--source', required=True)
    ap.add_argument('--show', type=float, default=0.6, help='창 · 출력에 쓸 점수 문턱(등록부 문턱 0.6)')
    ap.add_argument('--save', help='상자 그린 결과 사진을 이 폴더에 저장(yolo predict 처럼). 창 없이도 됨')
    ap.add_argument('--every', type=float, default=1.0, help='이 초마다 터미널에 처리 시간 · fps · 검출 요약 한 줄(0 = 안 함)')
    ap.add_argument('--no-window', action='store_true'); ap.add_argument('--scale', type=float, default=1.0, help='창 크기 배율(ssh -X 면 0.5 권장)')
    t_start = time.monotonic(); a = ap.parse_args(); det = Detector(a.engine); print(f'TensorRT {trt.__version__} 엔진 {a.engine} 입력 {det.ishape} 출력 {det.oshape}', flush=True)
    lat, t_prev, ndet = [], time.monotonic(), 0
    print(f'엔진 준비 {time.monotonic() - t_start:.2f} 초', flush=True); win, t_rep, first = [], time.monotonic(), True
    gen = frames(a.source)
    try:
      for name, img in gen:
          t0 = time.monotonic(); x, s, px, py = letterbox(img, det.hw); raw = det.infer(x); D = [d for d in decode(raw, s, px, py) if d[1] >= a.show]; t1 = time.monotonic()
          lat.append((t1 - t0) * 1000); ndet += len(D); win.append(lat[-1])
          if first:
              print(f'첫 장 결과까지 {t1 - t_start:.2f} 초(카메라 켜짐 포함)', flush=True); first = False
          if a.every and t1 - t_rep >= a.every:
              cnt = {}
              for c, _, _ in D: cnt[AB[c]] = cnt.get(AB[c], 0) + 1
              print(f'{time.strftime("%H:%M:%S")} | {len(win) / (t1 - t_rep):5.1f} fps | 한 장 처리 ms 중간 {np.median(win):5.1f} · 최대 {max(win):5.1f} | 지금 장 {len(D)}개 '
                    + ' '.join(f'{k}{v}' for k, v in sorted(cnt.items())), flush=True)
              win, t_rep = [], t1
          if a.no_window and not a.save:
              print(name, ' '.join(f'{AB[c]}{sc:.2f}' for c, sc, _ in D), flush=True); continue
          for c, sc, b in D:
              cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), COL[c], 2)
              cv2.putText(img, f'{AB[c]} {sc:.2f}', (int(b[0]), max(14, int(b[1]) - 4)), 0, 0.5, COL[c], 2)
          if a.save:
              os.makedirs(a.save, exist_ok=True); cv2.imwrite(os.path.join(a.save, os.path.splitext(name)[0] + '.jpg'), img)
              if a.no_window:
                  print(name, ' '.join(f'{AB[c]}{sc:.2f}' for c, sc, _ in D), flush=True); continue
          fps = 1 / max(t1 - t_prev, 1e-6); t_prev = t1
          cv2.putText(img, f'{name} | {len(D)} pucks >= {a.show} | pre+infer+post {lat[-1]:.1f} ms | {fps:.1f} fps | q quit', (8, img.shape[0] - 10), 0, 0.5, (0, 255, 255), 1)
          if a.scale != 1.0: img = cv2.resize(img, None, fx=a.scale, fy=a.scale)
          cv2.imshow('YOLO TensorRT', img)
          if cv2.waitKey(1 if not os.path.isdir(a.source) else 30) & 0xFF in (ord('q'), 27): break
    finally:  # 카메라를 프로그램이 끝나기 전에 닫음(인터프리터 종료 중 pipeline.stop 이 segfault 낼 수 있음)
        gen.close()
        if not a.no_window: cv2.destroyAllWindows()
    if lat:
        L = np.array(lat[5:] or lat); print(f'장 {len(lat)} | {a.show} 이상 검출 {ndet}개 | 전처리+추론+후처리 ms p50 {np.percentile(L, 50):.1f} p95 {np.percentile(L, 95):.1f} max {L.max():.1f}')


if __name__ == '__main__':
    sys.exit(main())
