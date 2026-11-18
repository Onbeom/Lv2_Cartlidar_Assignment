"""B0′ 시작 상자 측량 렌더 (2026-10-31, 판정 계획 `261031_0005/판정계획_새미로_261031.md` B0′ · P4): 시작 상자 6개 · 퍽 12개를
앞 D435 와 팔 카메라로 찍어 실제 인지 노드 → 퍽 등록부(perception_survey) 로 넣을 영상을 만든다. 모의(Isaac 렌더).

측량 순서(이 도구의 가정 — 실제 경로는 mission 몫): 상자마다
  d435 장면 --d435-per-box(2) 장: 차체가 상자 가운데를 보고(방향 ±10°) 수평 거리 U(--d435-dist 0.45~0.70 m), 팔 = 주행 자세(--arm-params drive).
    차체 윤곽이 미로 안 · 장애물에서 0.02 m 넘게, 다른 상자의 퍽을 밟지 않음, 상자 가운데가 화면 안(여백 30 px), 시선이 막히지 않음.
  arm 장면: 그 상자 퍽마다 --views-per-puck(2) 장. 계획 카메라 = 퍽 윗면 --arm-height(0.10 m) 위, 가로 ±0.01 m · 기울기 ≤ 5° · 방향 무작위로
    (못 가면 보기 방향을 다시 뽑음 --view-attempts 10, 뒤 절반은 기울기 15~30° · 차체가 목표를 비껴 섬(0 · ±45 · ±90°, 팔 joint1 이 돌림)
     — 벽에 붙은 퍽은 바로 위 · 정면 자리에서 못 봄)
    내려다보고, 윗면 가운데가 화면 세로 --aim-v(240 px = 가운데)에 오게 카메라를 옮긴다(못 가면 --aim-v-max 320 px 까지 받음).
    새 모델 팔 카메라에서 집게는 화면 세로 420~477 px(아래 12 %)에만 보인다(URDF 충돌 형상 투영, 2026-10-31) — 이전 모델 R4 의
    145 px(집게가 아래 40 % 를 가림)는 새 모델에 맞지 않음.
    퍽마다 차체 한 자리에서 그 퍽의 장을 모두(arm_pose.ArmRealizer, 충돌 여유 5 mm · 관절 한계 · 못 가면 5 mm 씩 최대 0.05 m 물림).
    (처음엔 상자마다 한 자리였으나 벽에 붙은 상자 2 · 6(seed 101)은 한 자리에서 두 퍽을 다 못 봐서 퍽마다로 바꿈, 2026-10-31)
  두 장은 서로 다른 시각 관측(등록부 min_observations 2)이 되게 카메라 자리가 다르다.
윗면: 퍽 겉면 중 법선이 가장 위를 향한 면(누운 직육면체 = 옆면 30 × 60, 그 면 가운데 태그). 원기둥은 끝면.
카메라: 앞 D435 = Scene(K, 640 × 480), 팔 = 640 × 480 · f 283 px 가정(R4 · R6 와 같음), 왜곡 0. 로봇 · 집게(gripper 0) 보임, 팔 가까운 클리핑 2.5 cm.
렌더 확인: 한 번 더 찍은 영상과 화소 평균 차가 --settle-tol 이하일 때까지(--settle-retries, R1′ 덜 켜짐 결함 2026-10-31 과 같은 방식).
  d435 = 퍽마다 카메라를 향한 겉면 점이 깊이로 보이는 비율(visible), arm = 목표 퍽 윗면 태그 9 × 9 점이 보이는 비율(top_tag_visible).
결과: <out>/<name>/frame.png(d435 는 frame_depth.npy 도), <out>/render_b0.json(replay_b0.py 입력, 장면 순서 = 측량 순서)
실행 (PC): 계획은 CPU 만(--only-plan --save-plan), 렌더는 --load-plan(+ --plan-world-map 으로 색온도만 다른 월드).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim_world"))
from r12_poses import K, W, H, Scene, _point_in_poly  # noqa: E402
from urdf_chain import planar_pose, rpy_matrix  # noqa: E402
from arm_pose import ArmRealizer  # noqa: E402

TOP = HERE.parents[1]
M = TOP / "261029_1030/urdf_261028"

ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--urdf", type=Path, default=M / "isaac/urdf/waffle_4wd_light.urdf")
ap.add_argument("--robot-usd", type=Path, default=M / "isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda")
ap.add_argument("--arm-params", type=Path, default=M / "isaac/robot_params.json")
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_dice_mission_maze261030.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--d435-per-box", type=int, default=2)
ap.add_argument("--d435-dist", type=float, nargs=2, default=[0.45, 0.70])
ap.add_argument("--views-per-puck", type=int, default=2)
ap.add_argument("--arm-height", type=float, default=0.10)
ap.add_argument("--view-attempts", type=int, default=10, help="퍽 하나의 보기 방향을 다시 뽑는 최대 횟수 (뒤 절반은 비스듬히)")
ap.add_argument("--oblique-tilt", type=float, nargs=2, default=[15.0, 30.0], help="[°] 뒤 절반 시도의 기울기 범위")
ap.add_argument("--aim-v", type=float, default=240.0, help="[px] 목표 윗면 가운데를 둘 화면 세로 위치 (새 모델: 가운데)")
ap.add_argument("--aim-v-max", type=float, default=320.0, help="[px] aim-v 로 못 옮기면 받아들이는 목표 세로 위치 상한")
ap.add_argument("--arm-f", type=float, default=283.0)
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0])
ap.add_argument("--seed", type=int, default=20261031)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--settle-retries", type=int, default=6)
ap.add_argument("--settle-tol", type=float, default=1.0, help="[8 bit] 두 장 화소 평균 |차| 허용")
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--arm-frame", default="arm_camera_optical_frame")
ap.add_argument("--near", type=float, default=0.025)
ap.add_argument("--only-plan", action="store_true")
ap.add_argument("--save-plan", type=Path)
ap.add_argument("--load-plan", type=Path, nargs="+")
ap.add_argument("--plan-world-map", nargs=2, action="append", default=[], metavar=("FROM", "TO"))
args, _ = ap.parse_known_args()

DRIVE_Q = json.loads(args.arm_params.read_text())["poses"]["drive"]["value"]
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
ROBOT = "/World/waffle_4wd"
CUBOID_HALF = (0.015, 0.015, 0.03)
CYL_R, CYL_HALF = 0.015, 0.03
TAG = 0.016
K_ARM = {"fx": args.arm_f, "fy": args.arm_f, "cx": W / 2, "cy": H / 2}
pkg_dir = args.urdf.parent.parent
drawing = json.loads(args.drawing.read_text())


def puck_surface(p):
    """퍽 겉면 점 · 바깥 법선 (월드). render_r4.py 와 같음."""
    R, c = np.array(p["R"]), np.array(p["center"])
    pts, nrm = [], []
    g = np.linspace(-0.85, 0.85, 7)
    if p["shape"] == "cuboid":
        h = np.array(CUBOID_HALF)
        for ax in range(3):
            o1, o2 = [k for k in range(3) if k != ax]
            for s in (-1, 1):
                for a in g:
                    for b in g:
                        q = np.zeros(3)
                        q[ax], q[o1], q[o2] = s * h[ax], a * h[o1], b * h[o2]
                        n = np.zeros(3)
                        n[ax] = s
                        pts.append(q)
                        nrm.append(n)
    else:
        for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            for zz in g:
                pts.append([CYL_R * math.cos(th), CYL_R * math.sin(th), zz * CYL_HALF])
                nrm.append([math.cos(th), math.sin(th), 0.0])
        for s in (-1, 1):
            for a in g:
                for b in g:
                    if a * a + b * b <= 0.85 ** 2:
                        pts.append([a * CYL_R, b * CYL_R, s * CYL_HALF])
                        nrm.append([0.0, 0.0, s])
    return np.asarray(pts) @ R.T + c, np.asarray(nrm, float) @ R.T


def top_face(p):
    """위를 향한 면 (가운데, 면 안 두 축 u · v). 직육면체는 6면 중 법선이 가장 위, 원기둥은 끝면."""
    R, c = np.array(p["R"]), np.array(p["center"])
    half = CUBOID_HALF if p["shape"] == "cuboid" else (CYL_R, CYL_R, CYL_HALF)
    axes = range(3) if p["shape"] == "cuboid" else [2]
    ax = max(axes, key=lambda k: abs(R[2, k]))
    s = 1.0 if R[2, ax] > 0 else -1.0
    o1, o2 = [k for k in range(3) if k != ax]
    return c + s * R[:, ax] * half[ax], R[:, o1], R[:, o2]


def project(T_w_cam, Kc, p):
    q = np.linalg.inv(T_w_cam) @ np.append(np.asarray(p, float), 1.0)
    return Kc["fx"] * q[0] / q[2] + Kc["cx"], Kc["fy"] * q[1] / q[2] + Kc["cy"], q[2]


def down_view(rng, top, height, tilt_deg=(0.0, 5.0)):
    """윗면을 높이 height 에서 기울기 U(tilt_deg) · 방향 무작위로 보는 광학 자세(시선 위 윗면 가운데 ± 0.01 m 가로).
    기울면 카메라가 시선을 따라 옆으로 물러난다(벽에 붙은 퍽, 2026-10-31)."""
    tilt, az, yaw = math.radians(rng.uniform(*tilt_deg)), rng.uniform(0, 2 * math.pi), rng.uniform(0, 2 * math.pi)
    z = np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt)])
    cam = np.asarray(top, float) + [rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), 0.0] - z * height / math.cos(tilt)
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x -= z * (x @ z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, cam
    return T


def aim_frame(ar, T_bf, T_des, target, seeds, iters=4, tol_px=5.0):
    """차체 T_bf 에서 한 장: 풀고, 목표가 화면 세로 aim-v 에 오게 카메라 세로축으로 옮겨 다시 푼다(arm_pose.realize_aim 과 같은 식).
    옮긴 자리를 팔이 못 가면 직전 자세를 쓴다 — 단 목표 세로가 --aim-v-max 이하(집게 영역 420 px 위)일 때만."""
    T, f, last = np.asarray(T_des, float).copy(), None, None
    for _ in range(iters):
        f = ar.frame(T_bf, T, seeds)
        if f is None:
            return last
        Ta = f[2]
        q = np.linalg.inv(Ta) @ np.append(target, 1.0)
        v = K_ARM["fy"] * q[1] / q[2] + K_ARM["cy"]
        if abs(v - args.aim_v) <= tol_px:
            return f
        last = f if v <= args.aim_v_max else None
        T = Ta.copy()
        T[:3, 3] = Ta[:3, 3] + Ta[:3, 1] * (v - args.aim_v) * q[2] / K_ARM["fy"]
    return last


def plan_world(wdir, rng):
    lay = json.loads((wdir / "layout.json").read_text())
    sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, arm_q=DRIVE_Q, shelf_top=lay["assumptions"]["box_small"][2])
    ar = ArmRealizer(args.urdf, pkg_dir, sc, lay, gripper=0.0)
    if ar.cam_frame != args.arm_frame:
        raise ValueError(f"--arm-frame {args.arm_frame} 이 URDF 팔 카메라 {ar.cam_frame} 와 다름")
    start = [p for p in lay["pucks"] if p["location"].startswith("start_box_")]
    other_xy = [np.array(p["center"][:2]) for p in lay["pucks"]]
    plan, unreachable = [], []
    for box in [b for b in lay["boxes"] if b["kind"] == "start"]:
        bi = int(box["name"].rsplit("_", 1)[1])
        c = np.array(box["center"][:2])
        pucks = sorted([p for p in start if p["box"] == bi], key=lambda p: p["side"])
        # 앞 D435
        for k in range(args.d435_per_box):
            for _ in range(20000):
                d = rng.uniform(*args.d435_dist)
                phi = rng.uniform(0, 2 * math.pi)
                bearing = math.radians(rng.uniform(-10, 10))
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
                if any(_point_in_poly(f, fp) or np.min(np.linalg.norm(ring - f, axis=1)) < 0.03 for f in other_xy):
                    continue
                T_w_cam = T_w_bf @ sc.T_bf_cam
                ok = True
                for p in pucks:
                    u, v, zc = project(T_w_cam, K, p["center"])
                    if not (zc > 0 and 30 <= u < W - 30 and 30 <= v < H - 30) or \
                            sc.los_blocked(T_w_cam[:3, 3], np.array(p["center"]), end_skip=0.05):
                        ok = False
                if not ok:
                    continue
                plan.append({"kind": "d435", "id": f"w{lay['seed']}_b{bi}_d{k}", "world": str(wdir), "seed": int(lay["seed"]),
                             "box": bi, "dist_nominal": round(float(d), 4), "bearing_deg": round(math.degrees(bearing), 2),
                             "T_world_base_footprint": T_w_bf.tolist(), "T_world_cam": T_w_cam.tolist(), "arm_q": DRIVE_Q,
                             "min_clearance_m": round(sc.clearance(ring), 4)})
                break
            else:
                raise RuntimeError(f"{wdir.name} 상자 {bi} D435 장면을 못 뽑음")
        # 팔: 퍽마다 차체 한 자리에서 views-per-puck 장
        floor = [np.array(p["center"][:2]) for p in lay["pucks"] if p["location"] == "floor"]
        r_ik = np.random.default_rng(0)   # 고정 무작위 시작값 12개 (render_r3.py MissionArmRealizer 와 같은 방식, 한계 근처 해)
        seeds0 = ar.prof["seeds"] + [r_ik.uniform([l for l, _ in ar.lim], [h for _, h in ar.lim]) for _ in range(12)]
        rows = []
        for p in pucks:
            top, _, _ = top_face(p)
            found = None
            for attempt in range(args.view_attempts):   # 못 가면 보기 방향을 다시 뽑는다 (R3 · R4 와 같은 방식)
                tilt = (0.0, 5.0) if attempt < args.view_attempts // 2 else tuple(args.oblique_tilt)
                views = [(p, k, top, down_view(rng, top, args.arm_height, tilt)) for k in range(args.views_per_puck)]
                heads = (0.0,) if attempt < args.view_attempts // 2 else tuple(math.radians(h) for h in (0, 45, -45, 90, -90))
                for T_bf in ar.base_candidates(views[0][3], floor, heads):
                    if ar.body_hits(T_bf):
                        continue
                    frames, prev = [], None
                    for _, _, _, T_des in views:
                        f = aim_frame(ar, T_bf, T_des, top, ([prev] if prev is not None else []) + seeds0)
                        if f is None:
                            break
                        frames.append(f)
                        prev = f[0]
                    if len(frames) == len(views):
                        found = (T_bf, frames)
                        break
                if found is not None:
                    break
            if found is None:   # 같은 자리에서 두 장이 안 되면 장마다 차체 자리 (seed 103 상자 5 puck_02, 2026-10-31)
                each = []
                for k in range(args.views_per_puck):
                    fk = None
                    for attempt in range(args.view_attempts):
                        T_des = down_view(rng, top, args.arm_height)
                        for T_bf in ar.base_candidates(T_des, floor, tuple(math.radians(h) for h in (0, 45, -45, 90, -90))):
                            if ar.body_hits(T_bf):
                                continue
                            f = aim_frame(ar, T_bf, T_des, top, seeds0)
                            if f is not None:
                                fk = ((p, k, top, T_des), f, T_bf)
                                break
                        if fk is not None:
                            break
                    if fk is None:
                        break
                    each.append(fk)
                if len(each) == args.views_per_puck:
                    rows += each
                    print(wdir.name, "상자", bi, p["name"], "장마다 차체 자리", flush=True)
                    continue
            if found is None:
                unreachable.append({"world": str(wdir), "box": bi, "puck": p["name"], "reason": "한 차체 자리에서 그 퍽의 팔 장면 자세를 못 찾음"})
                print(wdir.name, "상자", bi, p["name"], "팔 자세 없음", flush=True)
                continue
            rows += [(v, f, found[0]) for v, f in zip(views, found[1])]
        frames = [f for _, f, _ in rows]
        for (p, k, top, T_des), (q, b, Ta), T_bf in rows:
            u, v, zc = project(Ta, K_ARM, top)
            plan.append({"kind": "arm", "id": f"w{lay['seed']}_b{bi}_a_{p['name']}_{k}", "world": str(wdir),
                         "seed": int(lay["seed"]), "box": bi, "target": p["name"], "backoff_m": b,
                         "T_world_base_footprint": T_bf.tolist(), "T_world_cam": Ta.tolist(),
                         "arm_q": {**dict(zip(ar.arm_j, [round(float(x), 6) for x in q])), "gripper_left_joint": 0.0,
                                   "gripper_right_joint": 0.0},
                         "aim_px": [round(float(u), 1), round(float(v), 1)], "cam_height_m": round(float(Ta[2, 3] - top[2]), 4),
                         "tilt_deg": round(math.degrees(math.acos(min(1.0, -Ta[2, 2]))), 1)})
        print(wdir.name, "상자", bi, "D435", args.d435_per_box, "팔", len(frames), "물림", [f[1] for f in frames], flush=True)
    truth = [{k: p[k] for k in ("name", "tag_id", "color", "shape", "standing", "box", "side", "center", "R")} for p in start]
    return plan, unreachable, truth


# ---------- 계획 ----------
rng = np.random.default_rng(args.seed)
plan, unreachable, truths = [], [], {}
if args.load_plan is None:
    for wdir in args.worlds:
        p, u, t = plan_world(wdir, rng)
        plan += p
        unreachable += u
        truths[str(wdir)] = t
else:
    wmap = {str(Path(a).resolve()): str(Path(b)) for a, b in args.plan_world_map}
    for f in args.load_plan:
        d = json.loads(Path(f).read_text())
        for x in d["plan"]:
            x["world"] = wmap.get(str(Path(x["world"]).resolve()), x["world"])
        plan += d["plan"]
        unreachable += d["unreachable"]
        for w, t in d["truth"].items():
            truths[wmap.get(str(Path(w).resolve()), w)] = t
if args.save_plan:
    Path(args.save_plan).write_text(json.dumps({"plan": plan, "unreachable": unreachable, "truth": truths}, ensure_ascii=False))
print(f"장면 {len(plan)}개 (D435 {sum(p['kind'] == 'd435' for p in plan)}, 팔 {sum(p['kind'] == 'arm' for p in plan)}), "
      f"팔 자세 없음 {len(unreachable)}", flush=True)
if args.only_plan:
    sys.exit(0 if not unreachable else 1)
if unreachable:
    raise SystemExit("팔 자세 없는 상자가 있어 렌더하지 않음")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from robot_usd import pose_robot, use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def set_camera(stage, path, T_w_opt, Kc, w, h, near):
    """광학 좌표 자세로 USD 카메라를 둔다 (render_r3.py 와 같음)."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = Kc["fx"] * ha / w
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * h / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - w / 2) / w * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - h / 2) / w * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, 50.0))
    R, e = T_w_opt[:3, :3], T_w_opt[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


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


def grab_settled(a_rgb, a_dep):
    rgb, dep = grab(a_rgb, a_dep)
    diffs = []
    for _ in range(args.settle_retries):
        updates(8)
        rgb2, dep2 = grab(a_rgb, a_dep)
        d = float(np.abs(rgb2.astype(np.int16) - rgb.astype(np.int16)).mean())
        diffs.append(round(d, 2))
        rgb, dep = rgb2, dep2
        if d <= args.settle_tol:
            break
    return rgb, dep, diffs


def depth_visible(pts, T_w_cam, Kc, dep, tol):
    Ti = np.linalg.inv(T_w_cam)
    q = (Ti[:3, :3] @ np.asarray(pts).T).T + Ti[:3, 3]
    vis = 0
    for x, y, z in q:
        if z <= 0:
            continue
        u, v = Kc["fx"] * x / z + Kc["cx"], Kc["fy"] * y / z + Kc["cy"]
        if 0 <= u < W and 0 <= v < H and np.isfinite(dep[int(v), int(u)]) and dep[int(v), int(u)] > z - tol:
            vis += 1
    return vis / len(q)


def d435_visible(p, T_w_cam, dep):
    pts, nrm = puck_surface(p)
    view = T_w_cam[:3, 3] - pts
    front = np.einsum("ij,ij->i", nrm, view) / np.linalg.norm(view, axis=1) > 0.15
    return depth_visible(pts[front], T_w_cam, K, dep, 0.004) if front.any() else 0.0


def tag_visible(p, T_w_cam, dep):
    c, u, v = top_face(p)
    g = np.linspace(-TAG / 2, TAG / 2, 9)
    return depth_visible([c + a * u + b * v for a in g for b in g], T_w_cam, K_ARM, dep, 0.003)


report = {"urdf": str(args.urdf), "robot_usd": str(args.robot_usd), "width": W, "height": H, "lights": args.lights,
          "seed": args.seed, "d435": {"frame": args.frame, "K": K}, "arm": {"frame": args.arm_frame, "K": K_ARM},
          "truth": truths, "poses": []}
t0 = time.time()
worlds = list(dict.fromkeys(p["world"] for p in plan))
for wstr in worlds:
    wdir = Path(wstr)
    if not any(Path(w).resolve() == wdir.resolve() for w in args.worlds):
        continue
    truth = truths[wstr]
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    use_robot_usd(stage, args.robot_usd)
    updates(10)
    scenes = [p for p in plan if p["world"] == wstr]
    prods = {}
    for kind, Kc in (("d435", K), ("arm", K_ARM)):
        path = f"/World/RenderB0/{kind}"
        first = next(p for p in scenes if p["kind"] == kind)
        set_camera(stage, path, np.array(first["T_world_cam"]), Kc, W, H, args.near if kind == "arm" else 0.01)
        rp = rep.create.render_product(path, (W, H))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach(rp)
        a_dep.attach(rp)
        prods[kind] = (path, Kc, rp, a_rgb, a_dep)
    for level in args.lights:
        set_lighting(stage, level)
        for pose in scenes:
            path, Kc, rp, a_rgb, a_dep = prods[pose["kind"]]
            T_w_cam = np.array(pose["T_world_cam"])
            set_camera(stage, path, T_w_cam, Kc, W, H, args.near if pose["kind"] == "arm" else 0.01)
            pose_robot(stage, args.urdf, np.array(pose["T_world_base_footprint"]), pose["arm_q"])
            updates(4)
            rgb, dep, diffs = grab_settled(a_rgb, a_dep)
            name = f"{pose['id']}_L{level:g}"
            (out / name).mkdir()
            cv2.imwrite(str(out / name / "frame.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            images = {"color": f"{name}/frame.png"}
            entry = dict(pose)
            if pose["kind"] == "d435":
                np.save(out / name / "frame_depth.npy", dep)
                images["depth"] = f"{name}/frame_depth.npy"
                entry["visible"] = {p["name"]: round(d435_visible(p, T_w_cam, dep), 3) for p in truth if p["box"] == pose["box"]}
            else:
                tp = next(p for p in truth if p["name"] == pose["target"])
                entry["top_tag_visible"] = round(tag_visible(tp, T_w_cam, dep), 3)
            entry.update({"name": name, "light_level": level, "images": images, "settle": diffs})
            report["poses"].append(entry)
            print(name, "보임", entry.get("visible", entry.get("top_tag_visible")), "settle", diffs, flush=True)
    for path, Kc, rp, a_rgb, a_dep in prods.values():
        a_rgb.detach()
        a_dep.detach()
        rp.destroy()
    updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_b0.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"poses": len(report["poses"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
