"""추가 촬영이 필요한지: 28 장을 다시 뽑아(부트스트랩 30 회) 맞춘 모형끼리 화면 각 자리 투영이 얼마나 흔들리는지(px).
값 = 같은 광선이 모형마다 화면 어디에 찍히는지의 흔들림. 가운데(코너 있던 곳)와 끝(외삽)을 나눠 본다."""
import glob, sys
import cv2
import numpy as np
sys.path.insert(0, '/home/pa4/git/lv2_next_module/261106_1655_real_drive/jetson')
from charuco_util import Detector, object_points
det = Detector(); V = []
DIR = sys.argv[1] if len(sys.argv) > 1 else 'frames'
FLAGS = cv2.CALIB_RATIONAL_MODEL if len(sys.argv) > 2 and sys.argv[2] == 'rational' else 0
for f in sorted(glob.glob(DIR + '/frame_*.jpg')):
    p, i, _ = det.detect(cv2.imread(f, 0))
    if p is not None and len(p) >= 12:
        V.append((object_points(i), p.astype(np.float32)))
seen = np.concatenate([v[1] for v in V])
INIT = None
def fit(vs):
    # 11-09: 전체 맞춤 값을 초깃값으로(초깃값 없으면 끝 장이 섞일 때 엉뚱한 해로 빠짐)
    if INIT is None:
        _, K, D, _, _ = cv2.calibrateCamera([v[0] for v in vs], [v[1] for v in vs], (1280, 720), None, None)
        if FLAGS:
            Dz = np.zeros((1, 8)); Dz[0, :5] = D.ravel()[:5]
            _, K, D, _, _ = cv2.calibrateCamera([v[0] for v in vs], [v[1] for v in vs], (1280, 720), K, Dz, flags=FLAGS | cv2.CALIB_USE_INTRINSIC_GUESS)
        return K, D
    _, K, D, _, _ = cv2.calibrateCamera([v[0] for v in vs], [v[1] for v in vs], (1280, 720), INIT[0].copy(), INIT[1].copy(), flags=FLAGS | cv2.CALIB_USE_INTRINSIC_GUESS)
    return K, D
K0, D0 = fit(V)
INIT = (K0, D0)
print('모형', 'rational' if FLAGS else 'plumb_bob', '장', len(V), 'fx', round(K0[0, 0], 1))
us, vs_ = np.meshgrid(np.linspace(5, 1275, 33), np.linspace(5, 715, 19))
P = np.stack([us.ravel(), vs_.ravel()], 1)
ray = cv2.undistortPoints(P.reshape(-1, 1, 2), K0, D0).reshape(-1, 2)  # 기준 모형의 광선
obj = np.hstack([ray, np.ones((len(ray), 1))])
rng = np.random.default_rng(0); proj = []
for _ in range(30):
    K, D = fit([V[j] for j in rng.integers(0, len(V), len(V))])
    proj.append(cv2.projectPoints(obj, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2))
proj = np.array(proj); spread = np.linalg.norm(proj - proj.mean(0), axis=2)  # 30 x N
# 코너가 실제 있던 곳과의 거리
d = np.min(np.linalg.norm(P[:, None, :] - seen[None, ::7, :], axis=2), axis=1)
inside = d < 40
p95 = np.percentile(spread, 95, axis=0)
print(f'코너 있던 곳(40 px 안) {inside.mean()*100:.0f} %: 흔들림 p95 중앙 {np.median(p95[inside]):.2f} · 최대 {p95[inside].max():.2f} px')
print(f'외삽(그 밖): 흔들림 p95 중앙 {np.median(p95[~inside]):.2f} · 최대 {p95[~inside].max():.2f} px')
for name, m in [('왼쪽 끝 u<100', P[:, 0] < 100), ('오른쪽 끝 u>1180', P[:, 0] > 1180), ('아래 끝 v>650', P[:, 1] > 650), ('위 끝 v<70', P[:, 1] < 70)]:
    print(f'{name}: 흔들림 p95 최대 {p95[m].max():.2f} px, 코너 있던 비율 {inside[m].mean()*100:.0f} %')
# perception 의 펴기(alpha 0) 화각 흔들림
fx = [cv2.getOptimalNewCameraMatrix(*fit([V[j] for j in rng.integers(0, len(V), len(V))]), (1280, 720), 0.0)[0][0, 0] for _ in range(15)]
print(f'펼친 영상 newK fx: {np.mean(fx):.1f} ± {np.std(fx):.1f} px (15 회)')
