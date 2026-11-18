"""장소 태그(tag36h11 id 12, 검은 사각 48 mm) 박스 옆면을 앞 D435 컬러(640×480, f 605.9, 높이 0.490 m, 아래 35.2°)로 본 합성 영상 (도구 확인, 모의).
태그 가운데 높이 0.035 m(205×205×70 박스 옆면), 둘레 흰 여백 10 mm, 그 밖 박스 면(회색 150).
모드 a: 로봇이 태그 법선 위에서 θ 만큼 돌아 있음(태그가 화면 옆으로 감). 모드 b: 카메라가 태그를 정면으로 보되 법선에서 θ 만큼 옆 자리(비스듬함).
D = 카메라 · 태그 가운데 수평 거리. 밝기 1.0 · 0.4, 흐림 σ 0.7 px, 잡음 σ 2.
실행: python3 -I synth.py <출력 폴더>  → list.txt (이름 png 모드 θ D 밝기 + 참 법선(카메라 좌표))"""
import math, sys
from pathlib import Path
import cv2, numpy as np
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
W, H, FX, FY, CX, CY = 640, 480, 605.850769, 605.67987, 324.365814, 245.139175
CAM_H, PITCH = 0.48362259779 + 0.00675, 0.61440981034
TAG, MARGIN, TC_Z = 0.048, 0.010, 0.035
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
tex = cv2.aruco.drawMarker(dic, 12, 800) if hasattr(cv2.aruco, 'drawMarker') else cv2.aruco.generateImageMarker(dic, 12, 800)
def tex_val(u, v):   # u, v [m] 태그 가운데 기준(u 수평, v 위)
    half = TAG / 2
    out = np.full(u.shape, 150.0)
    lab = (np.abs(u) <= half + MARGIN) & (np.abs(v) <= half + MARGIN) & (v + TC_Z >= 0) & (v + TC_Z <= 0.07)
    out[lab] = 245.0
    inn = (np.abs(u) < half) & (np.abs(v) < half)
    iu = np.clip(((u[inn] + half) / TAG * 800).astype(int), 0, 799); iv = np.clip(((half - v[inn]) / TAG * 800).astype(int), 0, 799)
    out[inn] = np.where(tex[iv, iu] > 127, 245.0, 15.0)
    return out
def render(cam_xy, yaw, bright, seed):
    # 세계: 태그 면 x = 0, 법선 +x, 태그 가운데 (0, 0, TC_Z). 로봇 진행 방향 yaw(세계), 카메라는 아래로 PITCH
    cy_, sy_ = math.cos(yaw), math.sin(yaw); cp, sp = math.cos(PITCH), math.sin(PITCH)
    f = np.array([cy_ * cp, sy_ * cp, -sp]); r = np.array([sy_, -cy_, 0.0]); d = np.cross(f, r)   # 광학 z, x(오른쪽), y(아래)
    R = np.stack([r, d, f])   # 세계 → 광학
    C = np.array([cam_xy[0], cam_xy[1], CAM_H])
    ss = 3
    xs, ys = np.meshgrid((np.arange(W * ss) + 0.5) / ss - 0.5, (np.arange(H * ss) + 0.5) / ss - 0.5)
    rays = np.stack([(xs - CX) / FX, (ys - CY) / FY, np.ones_like(xs)], -1) @ R   # 광학 → 세계
    t = -C[0] / rays[..., 0]
    P = C + rays * t[..., None]
    img = np.full(xs.shape, 90.0)                 # 바닥 · 배경
    ok = (t > 0) & (P[..., 2] >= 0) & (P[..., 2] <= 0.07) & (np.abs(P[..., 1]) <= 0.1025)
    img[ok] = tex_val(P[..., 1][ok], P[..., 2][ok] - TC_Z)
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA) * bright
    img = cv2.GaussianBlur(img, (0, 0), 0.7) + np.random.default_rng(seed).normal(0, 2, img.shape)
    n_cam = R @ np.array([1.0, 0, 0])
    return np.clip(img, 0, 255).astype(np.uint8), n_cam
rows = []
i = 0
for mode in 'ab':
    for th in [0, 10, 15, 20, 25, 30, 35]:
        for D in [0.30, 0.40, 0.50, 0.60, 0.70]:
            for b in [1.0, 0.4]:
                t = math.radians(th)
                if mode == 'a':
                    cam, yaw = (D, 0.0), math.pi + t
                else:
                    cam, yaw = (D * math.cos(t), D * math.sin(t)), math.pi + t
                g, n = render(cam, yaw, b, i)
                name = f'{mode}_t{th:02d}_d{int(D*100):03d}_b{int(b*10)}'
                cv2.imwrite(str(out / f'{name}.png'), cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
                rows.append(f'{name} {out / (name + ".png")} {mode} {th} {D} {b} {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}')
                i += 1
(out / 'list.txt').write_text('\n'.join(rows) + '\n')
print(len(rows))
