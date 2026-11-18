"""상자 앞면(상자) 요 · 가운데를 잴 센서 후보 비교 (계산 · 몬테카를로, 11-12 미션 파트 261112 1, 검토 261111 1 조건 ①).

앞 라이다는 스캔 면 z 0.082 > 상자 0.07 이라 상자를 못 봄 (dev URDF, 바른 FK) → 빼고 셋을 비교.
장면 가정: 상자 지도 칸 ±0.05 m · 요 ±15°, Nav2 도착 xy ±0.05 · 요 ±0.1 rad (균등). 로봇은 대기 자리 (차체 앞 끝 → 지도 상자 면 0.45 m).
  T  TFmini 훑기: 제자리 회전 (ICR) 하며 1° 마다 한 번, 앞면에 맞은 점만 써서 직선 맞춤 → 요. 거리 잡음 σ_r (정규) + 1 cm 양자화 (사양 해상도 1 cm, 가정).
     odom 요 잡음 σ 0.2° (가정). 빔 퍼짐 · 모서리 걸침 · 바깥 점 거르기 오류는 넣지 않음 (좋은 쪽).
  D  앞 D435 컬러 (640 x 480, fx ≈ 615, sensors.yaml 대표값): 상자 윗면 네 모서리 화소 + 잡음 σ_px → 크기 아는 정사각형 IPPE → 요 · 가운데.
  A  팔 카메라 1 단계 (newK 1280 x 720): 대기 자리에서 box_look (지도 상자 중심, 후보 목록) 로 넓게 → D 와 같은 계산.
     place_box 렌더 판정 (U3 · 261111_1130) 이 모서리 위치를 약 0.5 px 안으로 맞춘 것에 맞춰 σ_px 0.5 · 1.0 을 봄.
  카메라 장착 · FK 보정 오차는 따로 표 (회전 1° 일 때 요 · 가운데 치우침).
    python3 face_sensor.py
"""
import itertools
import math
import os
import subprocess
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
AV = os.path.expanduser("~/git/lv2_next_module/261111_1700_align_view")
sys.argv = ["align_search.py", os.path.join(HERE, "waffle_4wd_dev_c689a38.urdf"), "--pose", "-1.37", "-0.01", "1.85"]
ns = {"__name__": "x"}
_o = sys.stdout
sys.stdout = open(os.devnull, "w")
exec(open(os.path.join(AV, "align_search.py")).read(), ns)
sys.stdout = _o
fk = ns["fk"]
LOOK = os.path.expanduser("~/git/lv2_next_module/261111_2240_box_view/look_tool")
FRONT, HALF, ZTOP = 0.1328, 0.1025, 0.07
ICR = np.array([-0.0658, 0.0])
STANDOFF = 0.45
NOM = np.array([FRONT + STANDOFF + HALF, 0.0])
TF_X = 0.13   # tfmini_link x (dev URDF), y 0, z 0.031
rng = np.random.default_rng(1)


def R2(t):
    return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])


def scene():
    d = rng.uniform(-0.05, 0.05, 2)
    by = math.radians(rng.uniform(-15, 15))
    nav = np.r_[rng.uniform(-0.05, 0.05, 2), rng.uniform(-0.1, 0.1)]
    # 로봇 좌표 상자 중심 · 요
    bc = R2(-nav[2]) @ (NOM + d - nav[:2])
    return bc, by - nav[2]


def wrap45(a):
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


# ---------------- T: TFmini 훑기
def front_face(bc, byaw):
    """로봇에서 보이는 앞면 (상자 네 면 중 로봇 쪽 바깥 법선이 −x 에 가장 가까운 면) 두 끝점."""
    best = None
    for k in range(4):
        n = R2(byaw + k * math.pi / 2) @ np.array([-1.0, 0.0])
        c = bc + n * HALF
        t = R2(byaw + k * math.pi / 2) @ np.array([0.0, 1.0])
        if best is None or n[0] < best[0]:
            best = (n[0], c - t * HALF, c + t * HALF, n)
    return best[1], best[2], best[3]


def tfmini_sweep(bc, byaw, span_deg, sig_r, quant=0.01, sig_odom_deg=0.2, step_deg=1.0):
    p0, p1, n = front_face(bc, byaw)
    pts = []
    for ph in np.arange(-span_deg, span_deg + 1e-9, step_deg):
        psi = math.radians(ph)
        o = ICR + R2(psi) @ (np.array([TF_X, 0.0]) - ICR)   # 돈 뒤 TFmini 위치 (돌기 전 로봇 좌표)
        dvec = R2(psi) @ np.array([1.0, 0.0])
        # 빔 ∩ 앞면 선분
        A = np.c_[dvec, p0 - p1]
        if abs(np.linalg.det(A)) < 1e-9:
            continue
        r, s = np.linalg.solve(A, p0 - o)
        if r <= 0 or not (0.0 <= s <= 1.0):
            continue
        rm = r + rng.normal(0, sig_r)
        if quant:
            rm = quant * round(rm / quant)
        psm = psi + math.radians(rng.normal(0, sig_odom_deg))
        om = ICR + R2(psm) @ (np.array([TF_X, 0.0]) - ICR)
        pts.append(om + R2(psm) @ np.array([rm, 0.0]))
    if len(pts) < 3:
        return None
    P = np.array(pts)
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c)
    t = vt[0]
    yaw_face = math.atan2(t[1], t[0]) - math.pi / 2   # 면 접선 → 상자 요 (90° 마다 같음)
    return wrap45(yaw_face - byaw), len(P)


# ---------------- D · A: 카메라로 윗면 정사각형
OBJ = np.array([[-HALF, HALF, 0], [HALF, HALF, 0], [HALF, -HALF, 0], [-HALF, -HALF, 0]], float)


def cam_measure(T_bc, K, size, bc, byaw, sig_px):
    """T_bc: base_footprint ← 광학. 윗면 네 모서리 투영 + 잡음 → IPPE → (요 오차 deg, 가운데 오차 mm, 최소 화면 여유 px, mm/px)."""
    Ti = np.linalg.inv(T_bc)
    Pw = np.array([[*(bc + R2(byaw) @ p[:2]), ZTOP] for p in OBJ])
    k = np.c_[Pw, np.ones(4)] @ Ti.T
    if (k[:, 2] <= 0.05).any():
        return None
    uv = np.c_[K[0, 0] * k[:, 0] / k[:, 2] + K[0, 2], K[1, 1] * k[:, 1] / k[:, 2] + K[1, 2]]
    W, H = size
    marg = float(min(uv[:, 0].min(), W - uv[:, 0].max(), uv[:, 1].min(), H - uv[:, 1].max()))
    img = uv + rng.normal(0, sig_px, uv.shape)
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(OBJ, img.reshape(-1, 1, 2), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    best = None
    for rv, tv in zip(rvecs, tvecs):   # 두 해 중 법선이 위 (바닥과 평행한 윗면) 에 가까운 것
        R = cv2.Rodrigues(rv)[0]
        Rw = T_bc[:3, :3] @ R
        up = abs(Rw[2, 2])
        if best is None or up > best[0]:
            best = (up, Rw, T_bc[:3, :3] @ tv.ravel() + T_bc[:3, 3])
    _, Rw, tw = best
    yaw = math.atan2(Rw[1, 0], Rw[0, 0])
    side = float(np.mean([np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4)]))
    return (math.degrees(wrap45(yaw - byaw)), 1000 * float(np.linalg.norm(tw[:2] - bc)), marg, 1000 * 2 * HALF / side)


def summary(name, vals):
    v = np.array(vals)
    return f"{name}: 요 오차 RMS {np.sqrt(np.mean(v[:, 0] ** 2)):.2f}° · p95 {np.percentile(np.abs(v[:, 0]), 95):.2f}°"


N = 2000
print("가정: 상자 ±0.05 m · ±15°, Nav2 ±0.05 · ±0.1 rad, 대기 자리 면 0.45 m. 시행", N)
print("\n[T] TFmini 훑기 (제자리 회전, 1° 마다, 앞면에 맞은 점만)")
for span, sr in itertools.product((10, 20), (0.005, 0.01, 0.02)):
    vals, npts, miss = [], [], 0
    for _ in range(N):
        bc, by = scene()
        r = tfmini_sweep(bc, by, span, sr)
        if r is None:
            miss += 1
            continue
        vals.append((math.degrees(r[0]), 0))
        npts.append(r[1])
    print(f"  훑기 ±{span}° · σ_r {sr * 100:.1f} cm + 1 cm 양자화: {summary('', vals)} · 앞면 점 중앙 {np.median(npts):.0f} · 3 점 못 됨 {miss}/{N}")

print("\n[D] 앞 D435 컬러 (640 x 480, fx 615 대표값, 장착 = dev URDF top_camera_front_color_optical_frame)")
Td = fk("top_camera_front_color_optical_frame", {})
Kd = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]])
for sp in (0.5, 1.0, 2.0):
    vals = []
    for _ in range(N):
        bc, by = scene()
        r = cam_measure(Td, Kd, (640, 480), bc, by, sp)
        if r:
            vals.append(r)
    v = np.array(vals)
    print(f"  σ_px {sp}: {summary('', vals)} · 가운데 RMS {np.sqrt(np.mean(v[:, 1] ** 2)):.1f} mm · 화면 여유 최소 {v[:, 2].min():.0f} px · "
          f"윗면 {np.median(v[:, 3]):.2f} mm/px (중앙)")

print("\n[A] 팔 카메라 1 단계 (대기 자리, 보는 점 = 지도 상자 중심, newK 1280 x 720)")
Ka = np.array([[602.35, 0, 530.64], [0, 793.36, 359.53], [0, 0, 1]])
T_ab = fk("arm_base_link", {})
LO = np.array([-3.14159, -2.14, -1.6, -1.92, -2.8])
HI = np.array([3.14159, 1.6, 1.6, 2.0, 2.8])
best_pose = None
for d, an in itertools.product((0.40, 0.45, 0.50, 0.55, 0.60), (40, 45, 50, 55, 60)):
    aim = np.linalg.inv(T_ab) @ np.array([NOM[0], NOM[1], ZTOP, 1.0])
    out = subprocess.run([LOOK], input=f"{aim[0]} {aim[1]} {aim[2]} 0.04 1 {d} 1 {an}\n", capture_output=True, text=True).stdout.split()
    if len(out) < 8 or out[2] != "1":
        continue
    q = [float(x) for x in out[3:8]]
    qs = [q[0], q[1] - 0.02, q[2], q[3] + 0.02, q[4]]
    jm = min(min(np.min(np.array(x) - LO), np.min(HI - np.array(x))) for x in (q, qs))
    Ta = fk("arm_camera_optical_frame", {f"joint{i + 1}": q[i] for i in range(5)})
    # 화면 여유 (꼭짓점 격자: 상자 ±0.05 · ±15°, Nav2 ±0.05 · ±0.1, 모서리 + 15 mm)
    worst = 1e9
    for dx, dy, b, nx, ny, nyw in itertools.product((-0.05, 0.05), (-0.05, 0.05), (-15, 15), (-0.05, 0.05), (-0.05, 0.05), (-0.1, 0.1)):
        bc = R2(-nyw) @ (NOM + np.array([dx, dy]) - np.array([nx, ny]))
        by = math.radians(b) - nyw
        Pw = np.array([[*(bc + R2(by) @ np.array([sx, sy]) * (HALF + 0.015)), ZTOP] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        k = np.c_[Pw, np.ones(4)] @ np.linalg.inv(Ta).T
        uv = np.c_[Ka[0, 0] * k[:, 0] / k[:, 2] + Ka[0, 2], Ka[1, 1] * k[:, 1] / k[:, 2] + Ka[1, 2]]
        worst = min(worst, float(min(uv[:, 0].min(), 1280 - uv[:, 0].max(), uv[:, 1].min(), 720 - uv[:, 1].max())))
    print(f"  {d:.2f} m {an}°: q {np.round(q, 3).tolist()} 관절 여유 {jm:.2f} · 화면 여유(꼭짓점) {worst:.0f} px")
    if worst > 30 and jm > 0.1 and (best_pose is None or d < best_pose[0]):
        best_pose = (d, an, Ta)
if best_pose:
    d, an, Ta = best_pose
    print(f"  → 화면 · 관절을 지키는 가장 가까운 자세 {d:.2f} m {an}° 로 정확도:")
    for sp in (0.5, 1.0, 2.0):
        vals = []
        for _ in range(N):
            bc, by = scene()
            r = cam_measure(Ta, Ka, (1280, 720), bc, by, sp)
            if r:
                vals.append(r)
        v = np.array(vals)
        print(f"    σ_px {sp}: {summary('', vals)} · 가운데 RMS {np.sqrt(np.mean(v[:, 1] ** 2)):.1f} mm · 윗면 {np.median(v[:, 3]):.2f} mm/px")

print("\n[보정 민감도] 카메라 장착이 1° 틀릴 때 (잡음 0, 상자 계획 자리) 요 · 가운데 치우침")
for name, T0, K, size in (("D435", Td, Kd, (640, 480)), ("팔 1 단계", best_pose[2] if best_pose else None, Ka, (1280, 720))):
    if T0 is None:
        continue
    for ax in range(3):
        rv = np.zeros(3)
        rv[ax] = math.radians(1.0)
        Tt = T0.copy()
        Tt[:3, :3] = T0[:3, :3] @ cv2.Rodrigues(rv)[0]   # 실제 카메라가 1° 돌아 있음, 추정은 T0
        Ti = np.linalg.inv(Tt)
        bc, by = NOM.copy(), 0.0
        Pw = np.array([[*(bc + p[:2]), ZTOP] for p in OBJ])
        k = np.c_[Pw, np.ones(4)] @ Ti.T
        uv = np.c_[K[0, 0] * k[:, 0] / k[:, 2] + K[0, 2], K[1, 1] * k[:, 1] / k[:, 2] + K[1, 2]]
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(OBJ, uv.reshape(-1, 1, 2), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        best = None
        for rv2, tv in zip(rvecs, tvecs):
            Rw = T0[:3, :3] @ cv2.Rodrigues(rv2)[0]
            if best is None or abs(Rw[2, 2]) > best[0]:
                best = (abs(Rw[2, 2]), Rw, T0[:3, :3] @ tv.ravel() + T0[:3, 3])
        _, Rw, tw = best
        yaw = math.degrees(wrap45(math.atan2(Rw[1, 0], Rw[0, 0])))
        print(f"  {name} 광학축 {'xyz'[ax]} 1°: 요 {yaw:+.2f}° · 가운데 {1000 * (tw[0] - bc[0]):+.1f}, {1000 * (tw[1] - bc[1]):+.1f} mm")
