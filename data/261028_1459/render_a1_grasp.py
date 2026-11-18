"""A1 팔 정렬 목표 유지 렌더 (검토 문서 6번 노드 수준 판정, 모의): 앞 D435 한 장 + 팔 카메라 연속 영상.

장면(월드마다 --scenes 개, 퍽이 2개 이상인 선반을 돌아가며 · 못 뽑으면 다음 선반): 로봇을 선반 앞 바닥에 둔다. base_footprint → 선반 가운데 수평 U(--dist) m,
  선반이 정면 ±15°, 카메라 흔들림 ±1°(render_r4.py 와 같은 조건 · 같은 뽑기 방법).
  추적 목표 예상 = 화면 안 · 시선이 막히지 않고 차체 제외 상자(target.exclude_*) 밖인 퍽 중 base_link 수평 거리가 가장
  가까운 것(TargetTracker 와 같은 규칙). 예상 목표가 노린 선반 위 퍽이 아니면 자세를 다시 뽑는다.
  재생기는 실제 추적 결과(object_pose)로 목표를 다시 확인한다.
D435 영상: 로봇 몸체 · 접은 팔 S0 포함, 컬러 + 깊이 한 장(로봇이 서 있는 동안 같은 영상을 계속 낸다).
팔 영상(640 × 480, f 283 px 가정, 왜곡 0): 목표 퍽 윗면 위에서 내려다본다.
  실제 팔 자세(arm_pose.py, 2026-10-27 저녁 사용자 결정): 장면마다 차체 자리 하나(D435 장면 자리에서 선반 앞으로 옮긴 자리)를 정하고
  장마다 팔 관절만 풀어 계획 카메라의 위치 · 광축을 맞춘다(광축 둘레 회전은 팔이 정함). 충돌(여유 5 mm) · 관절 한계로 못 가면
  광축 방향으로 5 mm 씩 물림(최대 0.05 m, backoff_m). 로봇 · 집게(열림 --gripper) 보임, 가까운 클리핑 2.5 cm.
  로봇 모델 = 기준 모델(robot_usd.py, 261027_1711).
  높이(퍽 윗면 위, 2026-10-27 저녁 실제 팔로 바꿈 — 새 브라켓은 수직 아래를 보는 카메라가 바닥 위 약 0.28~0.31 m 까지만 가서
  예전 0.25 m 시작은 못 감): --in-h(0.17 → 0.11 m), --pan-h(0.13 m).
  in  (20장): 높이 0.17 → 0.11 m 로 다가가며 가로 ±0.02 m · 기울기 ≤ 5° · 방향 ±3° 천천히 흔들림.
  out (38장): 0.17 → 0.13 m 6장 → 카메라 자리는 두고 시선을 이웃 쪽으로 12장에 걸쳐 돌려 목표를 화면 밖(겉면 점이 모두
              화면 밖 + 3°)으로 → 3장 머묾 → 12장에 걸쳐 돌아옴 → 0.13 → 0.11 m 5장. (2026-10-27 밤: 예전처럼 0.13 m 높이에서
              옆으로 0.12~0.16 m 옮기면 실제 팔이 못 가서 돌리기로 바꿈) 이웃 = 0.15 m 안 퍽(같은 색 먼저, 가까운 순) 중
              돌린 끝에서 중심이 화면 안(여백 10 px)인 첫 퍽. 실제 자세로 머묾 3장에서 목표가 다 화면 밖인지 다시 보고,
              아니면 돌리는 각을 3 · 6 · 10 · 15° 더한다.
  grasp (20장): 집기 직전 — 집게 수직 아래 · 손목 roll 0, TCP 를 목표 윗면 위 --grasp-h(0.05 → 0.015 m)로 내리며 가로 ±0.01 m 천천히 흔들림
              (정렬 보정 흉내). 카메라 광축이 집게에서 15° 기울어 목표가 화면 아래쪽(세로 약 316~400 px)으로 가고
              끝에는 화면 아래로 빠지거나 집게에 가린다. 못 가면 TCP 를 5 mm 씩 올림.
정답(팔 영상마다 퍽마다, 렌더 깊이로 가림 반영): 카메라를 향한 겉면 점(모서리 포함 격자) 중
  in_frac = 화면 안 비율(가림 무시), vis = 화면 안 · 깊이로 보이는 비율, box = 보이는 점 투영의 외접 상자 중심 · 폭
  (정규화 = image_offset 과 같은 뜻: x · y 화면 중심 대비 −1~1, w = 폭 / 화면 폭).
결과: <out>/<scene>_L<조명>/d435.png · d435_depth.npy · arm_XX.png, <out>/render_a1.json(replay_a1.py 입력)
실행 (PC, GPU 잠금):
  flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_a1.py \
      --worlds Cartlidar/sim_world/worlds/b_dice/seed_{1,2,3} \
      --out <결과 폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path("/home/pa4/git/lv2_next_module/Cartlidar/sim_render")
sys.path.insert(0, str(HERE))
from r12_poses import K, S0, W, H, Scene, _point_in_poly  # noqa: E402
from urdf_chain import planar_pose, rpy_matrix  # noqa: E402
from arm_pose import ArmRealizer  # noqa: E402

TOP = HERE.parents[1]
MODEL = "261027_1711"   # robot_usd.py 와 같게

ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--urdf", type=Path, default=TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf")
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--scenes", type=int, default=4, help="월드마다 장면 수 (in · out 번갈아)")
ap.add_argument("--kinds", nargs="+", default=["in", "out", "grasp"])
ap.add_argument("--gripper", type=float, default=0.02, help="[m] 팔 장면 집게 열림(gripper_left_joint, manipulator.yaml open_position 과 같은 값)")
ap.add_argument("--near", type=float, default=0.025, help="[m] 팔 카메라 가까운 클리핑")
ap.add_argument("--in-h", type=float, nargs=2, default=[0.17, 0.11], help="[m] 다가갈 때 퍽 윗면 위 카메라 높이 시작 · 끝")
ap.add_argument("--pan-h", type=float, default=0.13, help="[m] out 장면에서 옆으로 옮길 때 높이 (높으면 화면이 넓어 목표만 밖으로 못 냄)")
ap.add_argument("--grasp-h", type=float, nargs=2, default=[0.05, 0.015],
                help="[m] grasp 장면 TCP 높이 시작 · 끝 (집게 수직 TCP 는 바닥 위 약 0.22 m 까지, 서 있는 퍽 윗면 0.14 m)")
ap.add_argument("--dist", type=float, nargs=2, default=[0.5, 0.8])
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0])
ap.add_argument("--arm-f", type=float, default=283.0)
ap.add_argument("--seed", type=int, default=20261027)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--exclude-min", type=float, nargs=3, default=[-0.30, -0.25, -0.10])   # perception.yaml target.exclude_*
ap.add_argument("--exclude-max", type=float, nargs=3, default=[0.35, 0.25, 0.60])
ap.add_argument("--save-plan", type=Path)
ap.add_argument("--load-plan", type=Path)
ap.add_argument("--only-plan", action="store_true")
args, _ = ap.parse_known_args()
PAN_H = args.pan_h
H0, H1 = args.in_h

sys.path.insert(0, str(Path(__file__).resolve().parent / "a1_design"))
from a1_pose_tilt import TiltArmRealizer as ArmRealizer

LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
ROBOT = "/World/waffle_4wd"
HALF_XY, HALF_Z = 0.015, 0.03
PAN_N = 12     # 옮기는 데 쓰는 장 수 (15 Hz 에서 한 장에 화면 폭의 약 1/10 ~ 1/7)
BASE_LINK_Z = 0.00675
K_ARM = {"fx": args.arm_f, "fy": args.arm_f, "cx": W / 2, "cy": H / 2}
pkg_dir = args.urdf.parent.parent
drawing = json.loads(args.drawing.read_text())


def puck_surface(p, n=9):
    """퍽 겉면 점 · 바깥 법선 (월드), 모서리 포함 격자. 직육면체 면마다 n × n, 원기둥 옆면 32 × n · 끝면 원 안 점."""
    R, c = np.array(p["R"]), np.array(p["center"])
    g = np.linspace(-1.0, 1.0, n)
    pts, nrm = [], []
    if p["shape"] == "cuboid":
        h = np.array([HALF_XY, HALF_XY, HALF_Z])
        for ax in range(3):
            o1, o2 = [k for k in range(3) if k != ax]
            for s in (-1, 1):
                for a in g:
                    for b in g:
                        q = np.zeros(3)
                        q[ax], q[o1], q[o2] = s * h[ax], a * h[o1], b * h[o2]
                        e = np.zeros(3)
                        e[ax] = s
                        pts.append(q)
                        nrm.append(e)
    else:
        for th in np.linspace(0, 2 * math.pi, 32, endpoint=False):
            for zz in g:
                pts.append([HALF_XY * math.cos(th), HALF_XY * math.sin(th), zz * HALF_Z])
                nrm.append([math.cos(th), math.sin(th), 0.0])
        for s in (-1, 1):
            for a in g:
                for b in g:
                    if a * a + b * b <= 1.0:
                        pts.append([a * HALF_XY, b * HALF_XY, s * HALF_Z])
                        nrm.append([0.0, 0.0, s])
    return np.asarray(pts) @ R.T + c, np.asarray(nrm, float) @ R.T


def top_z(p):
    R = np.array(p["R"])
    return p["center"][2] + (HALF_Z if abs(R[2, 2]) > 0.7 else HALF_XY)


def cam_points(pts, nrm, T_w_cam):
    """카메라를 향한 점(cos > 0.05)의 카메라 좌표."""
    view = T_w_cam[:3, 3] - pts
    front = np.einsum("ij,ij->i", nrm, view) / np.linalg.norm(view, axis=1) > 0.05
    Ti = np.linalg.inv(T_w_cam)
    return (Ti[:3, :3] @ pts[front].T).T + Ti[:3, 3]


def in_frac(pts, nrm, T_w_cam, Kc):
    """카메라를 향한 점 중 화면 안 비율 (가림 무시)."""
    q = cam_points(pts, nrm, T_w_cam)
    if not len(q):
        return 0.0
    z = q[:, 2]
    ok = z > 1e-3
    u = Kc["fx"] * q[ok, 0] / z[ok] + Kc["cx"]
    v = Kc["fy"] * q[ok, 1] / z[ok] + Kc["cy"]
    return float(((u >= 0) & (u < W) & (v >= 0) & (v < H)).sum() / len(q))


def truth_box(pts, nrm, T_w_cam, Kc, dep, tol=0.004):
    """보이는 비율과 보이는 점 투영의 외접 상자(정규화 중심 x · y, 폭 w). 안 보이면 상자 None."""
    q = cam_points(pts, nrm, T_w_cam)
    n = len(q)
    if not n:
        return 0.0, None
    q = q[q[:, 2] > 1e-3]
    u = Kc["fx"] * q[:, 0] / q[:, 2] + Kc["cx"]
    v = Kc["fy"] * q[:, 1] / q[:, 2] + Kc["cy"]
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, z = u[inside], v[inside], q[inside, 2]
    seen = np.array([np.isfinite(dep[int(b), int(a)]) and dep[int(b), int(a)] > c - tol for a, b, c in zip(u, v, z)], bool)
    if not seen.any():
        return 0.0, None
    u, v = u[seen], v[seen]
    cx, cy = (u.min() + u.max()) / 2, (v.min() + v.max()) / 2
    return float(seen.sum() / n), [round(float((cx - W / 2) / (W / 2)), 4), round(float((cy - H / 2) / (H / 2)), 4),
                                   round(float((u.max() - u.min()) / W), 4)]


def look_down(pos, tilt_deg, az, yaw):
    """pos 에서 아래를 보는 광학 자세 (z 앞 = 아래로 tilt 만큼 기울임, x = yaw 방향)."""
    tilt = math.radians(tilt_deg)
    z = np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt)])
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x -= z * (x @ z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, pos
    return T


def project(T_w_cam, Kc, p):
    q = np.linalg.inv(T_w_cam) @ np.append(np.asarray(p, float), 1.0)
    return Kc["fx"] * q[0] / q[2] + Kc["cx"], Kc["fy"] * q[1] / q[2] + Kc["cy"], q[2]


def excluded(b):
    return all(args.exclude_min[i] <= b[i] <= args.exclude_max[i] for i in range(3))


def predict_target(sc, lay, T_w_bf, T_w_cam):
    """TargetTracker 규칙(제외 상자 밖 · base_link 수평 거리 최소)으로 고를 퍽 — 화면 안 · 시선 열린 것만."""
    T_bl = T_w_bf.copy()
    T_bl[:3, 3] += T_bl[:3, :3] @ np.array([0, 0, BASE_LINK_Z])
    Ti = np.linalg.inv(T_bl)
    best = None
    for p in lay["pucks"]:
        u, v, zc = project(T_w_cam, K, p["center"])
        if not (zc > 0 and 0 <= u < W and 0 <= v < H) or sc.los_blocked(T_w_cam[:3, 3], p["center"], end_skip=0.05):
            continue
        b = (Ti @ np.append(p["center"], 1.0))[:3]
        if excluded(b):
            continue
        d = math.hypot(b[0], b[1])
        if best is None or d < best[0]:
            best = (d, p["name"])
    return best[1] if best else None


def grasp_path(rng, tgt):
    """집기 직전 TCP 점 20개 (윗면 위 --grasp-h, 가로 ±0.01 m)."""
    c = np.array(tgt["center"][:2])
    zt = top_z(tgt)
    ph = rng.uniform(0, 2 * math.pi, 2)
    return [np.array([*(c + 0.01 * np.array([math.sin(0.29 * k + ph[0]), math.sin(0.21 * k + ph[1])])),
                      zt + args.grasp_h[0] - (args.grasp_h[0] - args.grasp_h[1]) * k / 19]) for k in range(20)]


def arm_path(rng, kind, tgt, nb, surf_t, extra=0.0):
    """팔 카메라 자세 목록과 장마다 단계 이름. extra: out 장면에서 더 돌릴 각 [deg]."""
    c = np.array(tgt["center"][:2])
    zt = top_z(tgt)
    yaw0, az = rng.uniform(0, 2 * math.pi), rng.uniform(0, 2 * math.pi)
    ph = rng.uniform(0, 2 * math.pi, 4)
    tilt_max = rng.uniform(2, 5)
    u = np.zeros(2)

    def pose(k, h, pan_deg=0.0, sway=0.02):
        """pan_deg: 카메라 자리는 두고 시선을 이웃 쪽(u)으로 돌린 각 (작은 흔들림 기울기와 합침)."""
        lat = sway * np.array([math.sin(0.31 * k + ph[0]), math.sin(0.23 * k + ph[1])])
        t0 = tilt_max * (0.5 + 0.5 * math.sin(0.17 * k + ph[2]))
        tv = t0 * np.array([math.cos(az), math.sin(az)]) + pan_deg * u
        yaw = yaw0 + math.radians(3) * math.sin(0.13 * k + ph[3])
        return look_down(np.array([*(c + lat), zt + h]), float(np.hypot(*tv)), math.atan2(tv[1], tv[0]), yaw)

    if kind == "in":
        return [pose(k, H0 - (H0 - H1) * k / 19) for k in range(20)], ["approach"] * 20, None
    u = np.array(nb["center"][:2]) - c
    u /= np.linalg.norm(u)
    k_hold = 6 + PAN_N
    th = 0.0
    while th < 80.0:   # 목표 겉면 점이 모두 화면 밖이 될 때까지 시선을 이웃 쪽으로 돌림 (흔들림 작게 0.005 m)
        if in_frac(*surf_t, pose(k_hold, PAN_H, th, 0.005), K_ARM) == 0.0:
            break
        th += 1.0
    th += 3.0 + extra
    seq, stage = [], []
    for k in range(6):
        seq.append(pose(k, H0 - (H0 - PAN_H) * k / 5))
        stage.append("approach")
    for j in range(1, PAN_N + 1):
        seq.append(pose(len(seq), PAN_H, th * j / PAN_N, 0.005))
        stage.append("pan_out")
    for _ in range(3):
        seq.append(pose(len(seq), PAN_H, th, 0.005))
        stage.append("hold_out")
    for j in range(1, PAN_N + 1):
        seq.append(pose(len(seq), PAN_H, th * (1 - j / PAN_N), 0.005))
        stage.append("pan_back")
    for j in range(1, 6):
        seq.append(pose(len(seq), PAN_H - (PAN_H - H1) * j / 5))
        stage.append("approach")
    return seq, stage, {"dir": u.tolist(), "pan_deg": round(th, 1)}


# ---------- 장면 뽑기 (렌더 전, 같은 seed 면 같은 결과) ----------
rng = np.random.default_rng(args.seed)
plan = []
if args.load_plan:
    plan = json.loads(args.load_plan.read_text())["plan"]
else:
    for wdir in args.worlds:
        lay = json.loads((wdir / "layout.json").read_text())
        sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, shelf_top=lay["assumptions"]["shelf_size"][2])
        ar = ArmRealizer(args.urdf, pkg_dir, sc, lay, gripper=args.gripper)
        pucks = {p["name"]: p for p in lay["pucks"]}
        floor = [np.array(p["center"][:2]) for p in lay["pucks"] if p["location"] == "floor"]
        shelves = [s for s in lay["shelves"] if sum(q["location"] == s["name"] for q in lay["pucks"]) >= 2]   # 이웃이 있는 선반
        for k in range(args.scenes):
            kind = args.kinds[k % len(args.kinds)]
            done = False
            for sh in shelves[k % len(shelves):] + shelves[:k % len(shelves)]:   # 안 되면 다음 선반
                c = np.array(sh["center"][:2])
                fails, got = 0, False
                for _ in range(20000):
                    if fails >= 3:   # 실제 팔 자세를 세 번 못 만들면 다음 선반
                        break
                    d = rng.uniform(*args.dist)
                    phi = rng.uniform(0, 2 * math.pi)
                    bearing = math.radians(rng.uniform(-15, 15))
                    pos = c + d * np.array([math.cos(phi), math.sin(phi)])
                    T_w_bf = planar_pose(pos[0], pos[1], phi + math.pi - bearing)
                    fp = sc.footprint_world(T_w_bf)
                    x0, x1, y0, y1 = sc.bounds
                    if fp[:, 0].min() < x0 or fp[:, 0].max() > x1 or fp[:, 1].min() < y0 or fp[:, 1].max() > y1:
                        continue
                    ring = np.vstack([fp[i] + (fp[(i + 1) % len(fp)] - fp[i]) * s for i in range(len(fp))
                                      for s in np.linspace(0, 1, 15, endpoint=False)])
                    if sc.clearance(ring) < 0.02:
                        continue
                    if any(_point_in_poly(f, fp) or np.min(np.linalg.norm(ring - f, axis=1)) < 0.03 for f in floor):
                        continue
                    J = np.eye(4)
                    J[:3, :3] = rpy_matrix(*[math.radians(rng.uniform(-1, 1)) for _ in range(3)])
                    T_w_cam = T_w_bf @ sc.T_bf_cam @ J
                    tname = predict_target(sc, lay, T_w_bf, T_w_cam)
                    if tname is None or pucks[tname]["location"] != sh["name"]:
                        continue
                    tgt = pucks[tname]
                    cand = [q for q in lay["pucks"] if q["name"] != tname and
                            np.hypot(*(np.array(q["center"][:2]) - tgt["center"][:2])) < 0.15]
                    if not cand:
                        continue
                    # 이웃: 같은 색 먼저, 가까운 순. out 은 돌린 끝에서 이웃 중심이 화면 안(여백 10 px)인 첫 이웃
                    cand.sort(key=lambda q: (q["color"] != tgt["color"], np.hypot(*(np.array(q["center"][:2]) - tgt["center"][:2]))))
                    st = rng.bit_generator.state
                    for nb in cand:
                        rng.bit_generator.state = st
                        seq, stage, pan = arm_path(rng, kind, tgt, nb, puck_surface(tgt))
                        if kind in ("in", "grasp"):
                            break
                        u, v, zc = project(seq[stage.index("hold_out")], K_ARM, nb["center"])
                        if zc > 0 and 10 <= u < W - 10 and 10 <= v < H - 10:
                            break
                    else:
                        continue
                    ar.view_hint = (np.array(nb["center"][:2]) - np.array(tgt["center"][:2])) if kind == "out" else None
                    # 실제 팔 자세
                    real = None
                    if kind == "grasp":
                        stage = ["grasp"] * 20
                        pan = None
                        grasp_state = rng.bit_generator.state
                        for tcp_tilt in (30.0, 45.0):
                            rng.bit_generator.state = grasp_state
                            ar.tcp_tilt = tcp_tilt
                            real = ar.realize_tcp(grasp_path(rng, tgt), floor)
                            if real is not None:
                                break
                    elif kind == "in":
                        real = ar.realize(seq, floor)
                    else:
                        st2 = rng.bit_generator.state
                        for extra in (0.0, 3.0, 6.0, 10.0, 15.0):
                            rng.bit_generator.state = st2
                            seq, stage, pan = arm_path(rng, kind, tgt, nb, puck_surface(tgt), extra)
                            real = ar.realize(seq, floor)
                            if real is None:
                                break
                            hold = [np.array(f["T_world_cam"]) for f, s_ in zip(real["frames"], stage) if s_ == "hold_out"]
                            if all(in_frac(*puck_surface(tgt), T, K_ARM) == 0.0 for T in hold):
                                pan["extra_m"] = extra
                                break
                        else:
                            real = None
                    if real is None:
                        fails += 1
                        print(f"w{lay['seed']} {kind} {sh['name']} {tname} 실제 팔 자세 없음 ({fails}), 가장 멀리 간 장 {ar.last_reach}",
                              flush=True)
                        continue
                    plan.append({"id": f"w{lay['seed']}_{kind}{k:02d}", "kind": kind, "world": str(wdir), "seed": int(lay["seed"]),
                                 "shelf": sh["name"], "target": tname, "neighbor": nb["name"],
                                 "neighbor_same_color": nb["color"] == tgt["color"], "pan": pan,
                                 "tcp_tilt_deg": getattr(ar, "tcp_tilt", None),
                                 "dist_nominal": round(float(d), 4), "bearing_deg": round(math.degrees(bearing), 3),
                                 "T_world_base_footprint": T_w_bf.tolist(), "T_world_d435": T_w_cam.tolist(),
                                 "T_world_base_footprint_arm": real["T_world_base_footprint"],
                                 "arm": [{"T_world_cam": f["T_world_cam"], "stage": s_, "backoff_m": f["backoff_m"],
                                          "arm_q": {**f["q"], "gripper_left_joint": args.gripper}}
                                         for f, s_ in zip(real["frames"], stage)]})
                    got = True
                    break
                if not got:
                    continue
                done = True
                break
            if not done:
                raise RuntimeError(f"{wdir.name} {kind} 장면을 못 뽑음")
if args.save_plan:
    args.save_plan.write_text(json.dumps({"plan":plan},ensure_ascii=False))
print(f"장면 {len(plan)}개 (" + ", ".join(f"{k} {sum(p['kind'] == k for p in plan)}" for k in args.kinds) + ")",
      flush=True)
if args.only_plan:
    for p in plan:
        print(p["id"], p["shelf"], "목표", p["target"], "이웃", p["neighbor"], "같은 색" if p["neighbor_same_color"] else "다른 색",
              "옮김", p["pan"], "장", len(p["arm"]), "물림 최대", max(f["backoff_m"] for f in p["arm"]))
    sys.exit(0)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

from robot_usd import pose_robot, use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def to_gf(T):
    return Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist())


def set_xform(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(to_gf(T))


def set_camera(stage, path, T_w_opt, Kc, near=0.01):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세로 USD 카메라(−Z 를 보고 +Y 가 위)를 둔다. 주점 부호: 가로 −, 세로 + (render_r4.py 와 같음)."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = Kc["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, 50.0))
    R, e = T_w_opt[:3, :3], T_w_opt[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def find_link(stage, name):
    hits = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ROBOT}/Geometry")) if p.GetName() == name and
            p.GetParent().GetName() != name]
    if len(hits) != 1:
        raise RuntimeError(f"링크 {name} 이 {len(hits)}개")
    return hits[0]


def set_lighting(stage, level):
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(float(rect))


def grab(a_rgb, a_dep):
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    return np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8), np.asarray(a_dep.get_data()).astype(np.float32)


report = {"urdf": str(args.urdf), "robot_usd": None, "gripper": args.gripper, "in_h": args.in_h, "pan_h": PAN_H, "grasp_h": args.grasp_h, "width": W, "height": H, "lights": args.lights, "seed": args.seed, "arm_q": S0,
          "d435": {"frame": args.frame, "K": K}, "arm": {"K": K_ARM}, "dist": args.dist,
          "exclude": [args.exclude_min, args.exclude_max], "scenes": []}
t0 = time.time()
for wdir in args.worlds:
    todo = [p for p in plan if Path(p["world"]).resolve() == wdir.resolve()]
    lay = json.loads((wdir / "layout.json").read_text())
    pucks = {p["name"]: p for p in lay["pucks"]}
    surf = {n: puck_surface(p) for n, p in pucks.items()}
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    report["robot_usd"] = use_robot_usd(stage)
    updates(10)
    cams = {}
    for kind, Kc in (("d435", K), ("arm", K_ARM)):
        path = f"/World/RenderA1/{kind}"
        set_camera(stage, path, np.array(todo[0]["T_world_d435"]), Kc, 0.01 if kind == "d435" else args.near)
        rp = rep.create.render_product(path, (W, H))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach(rp)
        a_dep.attach(rp)
        cams[kind] = (path, rp, a_rgb, a_dep)
    for level in args.lights:
        set_lighting(stage, level)
        for sc_ in todo:
            name = f"{sc_['id']}_L{level:g}"
            (out / name).mkdir()
            T_w_bf = np.array(sc_["T_world_base_footprint"])
            # D435: 로봇 · 접은 팔 S0 (두 카메라가 함께 렌더되지만 각자 영상만 쓴다)
            pose_robot(stage, args.urdf, T_w_bf, S0)
            path, rp, a_rgb, a_dep = cams["d435"]
            set_camera(stage, path, np.array(sc_["T_world_d435"]), K)
            updates(4)
            rgb, dep = grab(a_rgb, a_dep)
            cv2.imwrite(str(out / name / "d435.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            np.save(out / name / "d435_depth.npy", dep)
            d435_vis = {}
            for n, p in pucks.items():
                v, _ = truth_box(*surf[n], np.array(sc_["T_world_d435"]), K, dep)
                d435_vis[n] = round(v, 3)
            # 팔: 선반 앞 차체 자리 · 장마다 팔 관절
            path, rp, a_rgb, a_dep = cams["arm"]
            frames = []
            T_bf_arm = np.array(sc_["T_world_base_footprint_arm"])
            for k, f in enumerate(sc_["arm"]):
                T = np.array(f["T_world_cam"])
                pose_robot(stage, args.urdf, T_bf_arm, f["arm_q"])
                set_camera(stage, path, T, K_ARM, args.near)
                updates(4)
                rgb, dep = grab(a_rgb, a_dep)
                fn = f"{name}/arm_{k:02d}.png"
                cv2.imwrite(str(out / fn), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                tr = {}
                for n, p in pucks.items():
                    fr = in_frac(*surf[n], T, K_ARM)
                    if fr == 0.0:
                        continue
                    v, box = truth_box(*surf[n], T, K_ARM, dep)
                    tr[n] = {"in_frac": round(fr, 3), "vis": round(v, 3), "box": box}
                frames.append({"k": k, "stage": f["stage"], "T_world_cam": f["T_world_cam"], "backoff_m": f["backoff_m"],
                               "arm_q": f["arm_q"], "image": fn, "pucks": tr})
            e = {k: v for k, v in sc_.items() if k != "arm"}
            e.update({"name": name, "light_level": level, "d435_images": {"color": f"{name}/d435.png",
                                                                           "depth": f"{name}/d435_depth.npy"},
                      "d435_visible": d435_vis, "arm_frames": frames,
                      "pucks": [{k: p[k] for k in ("name", "tag_id", "color", "shape", "standing", "location", "center")}
                                for p in lay["pucks"]]})
            report["scenes"].append(e)
            tf = [f["pucks"].get(sc_["target"], {}).get("in_frac", 0.0) for f in frames]
            print(name, sc_["kind"], "목표", sc_["target"], "D435 보임", d435_vis[sc_["target"]],
                  "팔 목표 화면 안 비율", tf, flush=True)
    for kind in cams:
        cams[kind][2].detach()
        cams[kind][3].detach()
        cams[kind][1].destroy()
    updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_a1.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"scenes": len(report["scenes"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
