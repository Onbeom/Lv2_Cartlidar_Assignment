#!/usr/bin/env python3
"""도구 확인용 합성 영상: 값을 아는 렌즈로 보정판을 찍은 것처럼 만든다 (실물 아님)."""
import sys
from pathlib import Path
import cv2
import numpy as np
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
K = np.array([[430., 0, 655], [0, 432., 352], [0, 0, 1]])
# 화면 모서리까지 반지름이 단조 증가하는 값. 처음 쓴 k1 -0.31 은 모서리에 대응하는 광선이 없어 렌즈로 성립하지 않았다.
D = np.array([-0.12, 0.012, 0.0006, -0.0004, 0.0])
W, H = 1280, 720
board = cv2.imread(str(Path(__file__).resolve().parent.parent / 'jetson' / 'charuco_10x7_5x5_100.png'), 0)
bh, bw = board.shape
sq = bw / 10.0
board = cv2.GaussianBlur(board, (0, 0), 1.0)
u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
# 기본 undistortPoints 는 5 번만 되풀이해 렌즈 가장자리에서 틀린다. 합성 영상의 참값이 흔들리지 않게 충분히 되풀이한다.
crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
norm = cv2.undistortPointsIter(np.stack([u, v], -1).reshape(-1, 1, 2).astype(np.float64), K, D, None, None, crit).reshape(H, W, 2)
rng = np.random.default_rng(7)
def render(rvec, t):
    R = cv2.Rodrigues(np.array(rvec, float))[0]
    n = R[:, 2]; d = np.stack([norm[..., 0], norm[..., 1], np.ones((H, W))], -1)
    s = (n @ np.array(t)) / (d @ n)
    P = (d * s[..., None] - np.array(t)) @ R  # 보정판 좌표(칸 단위)
    m1 = (P[..., 0] * sq).astype(np.float32); m2 = (P[..., 1] * sq).astype(np.float32)
    im = cv2.remap(board, m1, m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=60)
    im = cv2.GaussianBlur(im, (0, 0), 0.7).astype(np.float32) * 0.8
    return np.clip(im + rng.normal(0, 2.5, im.shape), 0, 255).astype(np.uint8)
poses = []
for z in (8.0, 11.0, 15.0):
    for cx in (-0.55, 0.0, 0.55):
        for cy in (-0.35, 0.35):
            for rv in ((0, 0, 0), (0.45, 0, 0), (-0.45, 0, 0), (0, 0.45, 0), (0, -0.45, 0)):
                poses.append((rv, (cx * z - 5, cy * z - 3.5, z)))
rng.shuffle(poses)
poses = poses[:60]
vw = cv2.VideoWriter(str(out / 'synth.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 30, (W, H))
for j, (rv, t) in enumerate(poses):
    im = render(rv, t); bgr = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    for _ in range(4):
        vw.write(bgr)  # 같은 자리 4 장 = 멈춘 장면
vw.release()
print('자세', len(poses), '참값 K', K.ravel().tolist(), 'D', D.tolist())
