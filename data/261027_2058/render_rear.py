"""후방 장애물 판정 렌더 (모의, 계획_후방장애물.md 2장): 뒤 D435 깊이 + 차체 뒤 통로의 시험 물체 하나.

월드: 과제 미로(b_dice seed), 로봇 USD 는 기준 모델 261027_1711 로 바꿔(sim_render/robot_usd.py) 몸체 · 접은 팔 S0 포함.
자세: 차체 윤곽이 벽 · 기둥 · 선반에서 0.02 m 넘게, 뒤 통로(뒤 끝 ~ 0.70 m, 반폭 + 0.05)에 월드 물체가 없는 곳.
물체: 벽판 · 기둥 · 서 있는 원기둥 퍽 · 누운 원기둥 퍽 · 누운 직육면체 퍽 · 상자. 거리(뒤 끝 → 통로 안 가장 가까운 점)를
  안쪽 U(0.25, 0.39) · 바깥 U(0.50, 0.60) 로 뽑고, 빈 통로 장면도 둔다.
정답(base_footprint): 물체 겉면 점(5 mm)의 통로 · 높이 띠 안 최소 거리. 보이는 화소 수 = 렌더 깊이 화소 중 물체 안(5 mm 여유) ·
  통로 · 높이 띠 안인 화소 수(≥ 80 이면 '보이는').
결과: <out>/<id>/depth.npy(32F m) · color.png, <out>/render_rear.json
실행 (PC): ~/isaacsim-venv/bin/python 261027_2058/render_rear.py --out <결과 폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

TOP = Path(__file__).resolve().parents[1]
SR = TOP / "Cartlidar/sim_render"
sys.path.insert(0, str(SR))
from r12_poses import K, S0, W, H, Scene  # noqa: E402
from urdf_chain import planar_pose, rpy_matrix  # noqa: E402

MODEL = "261027_1711"
ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+",
                default=[TOP / f"Cartlidar/sim_world/worlds/b_dice/seed_{i}" for i in (1, 2, 3)])
ap.add_argument("--urdf", type=Path, default=TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf")
ap.add_argument("--drawing", type=Path, default=TOP / "Cartlidar/sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--n-in", type=int, default=4, help="종류 · 월드마다 안쪽 장면 수")
ap.add_argument("--n-out", type=int, default=2, help="종류 · 월드마다 바깥 장면 수")
ap.add_argument("--n-empty", type=int, default=6, help="월드마다 빈 통로 장면 수")
ap.add_argument("--seed", type=int, default=20261027)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--frame", default="top_camera_rear_color_optical_frame")
ap.add_argument("--light", type=float, default=1.0)
ap.add_argument("--plan-only", action="store_true", help="장면 뽑기만(도구 확인)")
args, _ = ap.parse_known_args()

X_EDGE, Y_HALF, Z_MIN, Z_MAX = -0.2285, 0.2176, 0.02, 0.45   # perception.yaml rear_obstacle 와 같게
CLEAR_LEN = 0.70
VIS_PX = 80
IN_RANGE, OUT_RANGE = (0.25, 0.39), (0.50, 0.60)
# 종류: (모양, 반크기 (x, y, z) — 원기둥은 (반지름, 반지름, 반높이), 축, 가로 위치 범위, 방향 흔들림 [°] (None = 무작위))
KINDS = {
    "wall": ("box", (0.01, 0.20, 0.19), "z", 0.08, 20.0),
    "post": ("box", (0.018, 0.018, 0.20), "z", 0.12, None),
    "puck_standing": ("cyl", (0.015, 0.015, 0.03), "z", 0.12, None),
    "puck_lying_cyl": ("cyl", (0.015, 0.015, 0.03), "y", 0.12, None),
    "puck_lying_cuboid": ("box", (0.03, 0.015, 0.015), "z", 0.12, None),
    "box": ("box", (0.075, 0.075, 0.075), "z", 0.12, None),
}
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음


def local_R(axis, yaw):
    """물체 자세: yaw(바닥 위 방향) 뒤, 원기둥 축이 y 면 x 축 둘레 90° 눕힘."""
    R = rpy_matrix(0.0, 0.0, yaw)
    return R @ (rpy_matrix(math.pi / 2, 0.0, 0.0) if axis == "y" else np.eye(3))


def surface(shape, half, step=0.005):
    """물체 겉면 점 (물체 좌표, 바닥 = z −half_z)."""
    hx, hy, hz = half
    pts = []
    if shape == "box":
        for ax in range(3):
            o = [k for k in range(3) if k != ax]
            g = [np.linspace(-half[k], half[k], max(2, int(2 * half[k] / step) + 1)) for k in o]
            a, b = np.meshgrid(*g, indexing="ij")
            for s in (-1, 1):
                q = np.zeros((a.size, 3))
                q[:, ax], q[:, o[0]], q[:, o[1]] = s * half[ax], a.ravel(), b.ravel()
                pts.append(q)
    else:
        r = hx
        th = np.linspace(0, 2 * math.pi, max(12, int(2 * math.pi * r / step)), endpoint=False)
        zz = np.linspace(-hz, hz, max(2, int(2 * hz / step) + 1))
        t, z = np.meshgrid(th, zz, indexing="ij")
        pts.append(np.stack([r * np.cos(t).ravel(), r * np.sin(t).ravel(), z.ravel()], 1))
        rr = np.linspace(0, r, max(2, int(r / step) + 1))
        t, rad = np.meshgrid(th, rr, indexing="ij")
        for s in (-1, 1):
            pts.append(np.stack([(rad * np.cos(t)).ravel(), (rad * np.sin(t)).ravel(), np.full(t.size, s * hz)], 1))
    return np.vstack(pts)


def in_band(P):
    return (P[:, 0] < X_EDGE) & (np.abs(P[:, 1]) <= Y_HALF) & (P[:, 2] >= Z_MIN) & (P[:, 2] <= Z_MAX)


def place(kind, d, y, yaw):
    """base_footprint 기준 물체 자세: 통로 안 가장 가까운 점의 거리가 d 가 되게 x 를 맞춤. (중심, R, 정답 거리)"""
    shape, half, axis, _, _ = KINDS[kind]
    R = local_R(axis, yaw)
    lo_z = np.abs(R[2, :]) @ np.array(half)          # 바닥에 놓는 높이 = 회전한 반높이
    S = surface(shape, half) @ R.T
    c = np.array([X_EDGE - d - 0.2, y, lo_z])
    for _ in range(3):
        P = S + c
        m = in_band(P)
        if not m.any():
            return None
        c[0] += (X_EDGE - P[m, 0].max()) - d
    P = S + c
    m = in_band(P)
    return c, R, float(X_EDGE - P[m, 0].max())


# ---------- 장면 뽑기 (렌더 전) ----------
rng = np.random.default_rng(args.seed)
plan, tries = [], 0
for wdir in args.worlds:
    lay = json.loads((wdir / "layout.json").read_text())
    sc = Scene(lay, json.loads(args.drawing.read_text()), args.urdf, args.urdf.parent.parent, frame=args.frame)
    floor = np.array([p["center"][:2] for p in lay["pucks"] if p["location"] == "floor"]).reshape(-1, 2)
    light = np.array([lay["signal_light"]["x"], lay["signal_light"]["y"]])
    gx, gy = np.meshgrid(np.linspace(X_EDGE - CLEAR_LEN, X_EDGE, 36), np.linspace(-Y_HALF - 0.05, Y_HALF + 0.05, 14))
    corridor = np.stack([gx.ravel(), gy.ravel()], 1)
    cases = [(k, "in") for k in KINDS for _ in range(args.n_in)] + [(k, "out") for k in KINDS for _ in range(args.n_out)]
    cases += [("none", "empty")] * args.n_empty
    for ci, (kind, zone) in enumerate(cases):
        for _ in range(50000):
            tries += 1
            x0, x1, y0, y1 = sc.bounds
            T_w_bf = planar_pose(rng.uniform(x0, x1), rng.uniform(y0, y1), rng.uniform(-math.pi, math.pi))
            fp = sc.footprint_world(T_w_bf)
            if fp[:, 0].min() < x0 or fp[:, 0].max() > x1 or fp[:, 1].min() < y0 or fp[:, 1].max() > y1:
                continue
            ring = np.vstack([fp[i] + (fp[(i + 1) % len(fp)] - fp[i]) * s for i in range(len(fp))
                              for s in np.linspace(0, 1, 15, endpoint=False)])
            if sc.clearance(ring) < 0.02:
                continue
            cw = corridor @ T_w_bf[:2, :2].T + T_w_bf[:2, 3]
            if cw[:, 0].min() < x0 or cw[:, 0].max() > x1 or cw[:, 1].min() < y0 or cw[:, 1].max() > y1:
                continue
            if sc.clearance(cw) < 0.01:
                continue
            if len(floor) and np.min(np.linalg.norm(cw[:, None, :] - floor[None], axis=2)) < 0.05:
                continue
            if np.min(np.linalg.norm(cw - light, axis=1)) < 0.10:
                continue
            obj = None
            if kind != "none":
                _, _, _, yr, jit = KINDS[kind]
                d = rng.uniform(*(IN_RANGE if zone == "in" else OUT_RANGE))
                yaw = math.radians(rng.uniform(-jit, jit)) if jit is not None else rng.uniform(-math.pi, math.pi)
                r = place(kind, d, rng.uniform(-yr, yr), yaw)
                if r is None:
                    continue
                c, R, dt = r
                obj = {"kind": kind, "center_bf": c.tolist(), "R_bf": R.tolist(), "truth_m": round(dt, 4)}
            plan.append({"id": f"w{lay['seed']}_{ci:02d}_{kind}_{zone}", "world": str(wdir), "zone": zone,
                         "T_world_base_footprint": T_w_bf.tolist(), "T_world_cam": (T_w_bf @ sc.T_bf_cam).tolist(),
                         "T_bf_cam": sc.T_bf_cam.tolist(), "obj": obj, "min_clearance_m": round(sc.clearance(ring), 4)})
            break
        else:
            raise RuntimeError(f"{wdir.name} {kind} {zone} 장면을 못 뽑음")
print(f"장면 {len(plan)}개, 뽑기 시도 {tries}", flush=True)
if args.plan_only:
    for p in plan:
        print(p["id"], p["obj"]["truth_m"] if p["obj"] else "-", p["min_clearance_m"])
    raise SystemExit(0)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from robot_usd import pose_robot, use_robot_usd  # noqa: E402

args.out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def set_matrix(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist()))


def set_camera(stage, path, T_w_opt, Kc, near=0.01):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세로 USD 카메라를 둔다 (sim_render/render_r4.py 와 같은 식)."""
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


def obj_pixels(dep, T_bf_cam, obj):
    """렌더 깊이 화소 중 물체 안(5 mm 여유) · 통로 · 높이 띠 안인 화소 수."""
    v, u = np.mgrid[0:H, 0:W]
    z = dep.astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05)
    x = (u - K["cx"]) / K["fx"] * z
    y = (v - K["cy"]) / K["fy"] * z
    P = np.stack([x[ok], y[ok], z[ok]], 1) @ T_bf_cam[:3, :3].T + T_bf_cam[:3, 3]
    m = in_band(P)
    if obj is None:
        return 0, int(m.sum())
    shape, half, _, _, _ = KINDS[obj["kind"]]
    L = (P[m] - np.array(obj["center_bf"])) @ np.array(obj["R_bf"])
    e = 0.005
    if shape == "box":
        inside = np.all(np.abs(L) <= np.array(half) + e, axis=1)
    else:
        inside = (np.hypot(L[:, 0], L[:, 1]) <= half[0] + e) & (np.abs(L[:, 2]) <= half[2] + e)
    return int(inside.sum()), int(m.sum())


report = {"urdf": str(args.urdf), "model": MODEL, "K": K, "width": W, "height": H, "frame": args.frame,
          "corridor": {"x_edge": X_EDGE, "y_half": Y_HALF, "z": [Z_MIN, Z_MAX]}, "vis_px": VIS_PX, "seed": args.seed,
          "light": args.light, "scenes": []}
t0 = time.time()
for wdir in args.worlds:
    todo = [p for p in plan if p["world"] == str(wdir)]
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    report["robot_usd"] = use_robot_usd(stage)
    lvl = args.light
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(
        float(LIGHT_RANGE["dome"][0] + lvl * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])))
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(
            float(LIGHT_RANGE["rect"][0] + lvl * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])))
    shapes = {}
    for kind, (shape, half, axis, _, _) in KINDS.items():
        path = f"/World/RearTest/{kind}"
        if shape == "box":
            g = UsdGeom.Cube.Define(stage, path + "/geom")
            g.CreateSizeAttr(2.0)
            UsdGeom.Xformable(g).AddScaleOp().Set(Gf.Vec3f(*half))
        else:
            g = UsdGeom.Cylinder.Define(stage, path + "/geom")
            g.CreateRadiusAttr(half[0])
            g.CreateHeightAttr(2 * half[2])
            g.CreateAxisAttr("Z")
        g.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.55, 0.5)])
        root = UsdGeom.Xform.Define(stage, path)
        root.MakeInvisible()
        shapes[kind] = root
    cam_path = "/World/RearTest/cam"
    set_camera(stage, cam_path, np.array(todo[0]["T_world_cam"]), K, 0.01)
    rp = rep.create.render_product(cam_path, (W, H))
    a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    a_rgb.attach(rp)
    a_dep.attach(rp)
    for sc_ in todo:
        T_w_bf = np.array(sc_["T_world_base_footprint"])
        set_camera(stage, cam_path, np.array(sc_["T_world_cam"]), K, 0.01)
        pose_robot(stage, args.urdf, T_w_bf, S0)
        for kind, root in shapes.items():
            root.MakeInvisible()
        if sc_["obj"]:
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = sc_["obj"]["R_bf"], sc_["obj"]["center_bf"]
            set_matrix(shapes[sc_["obj"]["kind"]].GetPrim(), T_w_bf @ T)
            shapes[sc_["obj"]["kind"]].MakeVisible()
        updates(4)
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=args.subframes)
        rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
        dep = np.asarray(a_dep.get_data()).astype(np.float32)
        d = args.out / sc_["id"]
        d.mkdir()
        np.save(d / "depth.npy", dep)
        cv2.imwrite(str(d / "color.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        n_obj, n_band = obj_pixels(dep, np.array(sc_["T_bf_cam"]), sc_["obj"])
        e = dict(sc_, obj_px=n_obj, band_px=n_band, visible=bool(sc_["obj"] and n_obj >= VIS_PX),
                 depth="%s/depth.npy" % sc_["id"])
        report["scenes"].append(e)
        print(sc_["id"], "정답", sc_["obj"]["truth_m"] if sc_["obj"] else "-", "물체 화소", n_obj, "띠 화소", n_band, flush=True)
    a_rgb.detach()
    a_dep.detach()
    rp.destroy()
    updates(4)
report["total_s"] = round(time.time() - t0, 1)
(args.out / "render_rear.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"scenes": len(report["scenes"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
