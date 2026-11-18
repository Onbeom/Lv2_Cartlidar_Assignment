"""R1 · R2 신호등 장면 렌더 (설계: R1R2_설계_가설_261024.md).

월드마다 거리마다 자세 n 개를 r12_poses.py 로 뽑고(벽 · 시선 · 화면 조건, 카메라 흔들림 ±1°),
자세마다 신호등 yaw 무작위 · 방해 퍽 1~3개를 두고, 조명 2단계 × 상태 4개(off · red · yellow · green)를 렌더한다.
로봇 몸체 · 접은 팔 S0 포함(Physics variant none, 링크 변환에 관절 값 직접). 로봇 마스크는 자세마다 한 번(로봇 숨긴 깊이와 비교).

결과: <out>/<name>/<state>.png · <state>_depth.npy · robot_mask.png, <out>/render_r12.json(K · frame · poses[] — replay_r12.py 입력)
  name = w<seed>_d<거리>_p<번호>_L<조명>

실행 (PC):
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r12.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_1 \
      --out <결과 폴더> \
      --dists 0.5 0.9 1.15 --poses 2 --lights 0 1 --emissive 1 --/crashreporter/url=
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
from r12_poses import K, S0, W, H, Scene  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True, help="task_world.usda · layout.json 폴더들")
ap.add_argument("--urdf", type=Path,
                default=HERE.parents[1] / "261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf")
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--dists", type=float, nargs="+", default=[0.5, 0.9, 1.15])
ap.add_argument("--start-pose", type=float, nargs=3, help="출발 x[m] y[m] yaw[deg]; 화면 밖도 그대로 시험")
ap.add_argument("--pos-jit", type=float, default=0.02)
ap.add_argument("--yaw-jit", type=float, default=3.0)
ap.add_argument("--only-plan", action="store_true")
ap.add_argument("--poses", type=int, nargs="+", default=[2], help="월드마다 거리마다 자세 수 (월드 수만큼 주거나 하나)")
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0], help="light_level (0 = 낮조명off, 1 = 낮조명on)")
ap.add_argument("--emissive", type=float, default=1.0, help="켜진 칸 emissiveColor 배율 — 미정(실물 사진 뒤)")
ap.add_argument("--seed", type=int, default=20261024)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--ghost-retries", type=int, default=6, help="상태 영상에 앞 상태 칸이 남으면 다시 찍는 최대 횟수 (0 = 검사 안 함)")
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--robot-usd", type=Path, default=None, help="렌더에 쓸 로봇 USD (기본 robot_usd.ROBOT_USD = 261027_1711)")
ap.add_argument("--arm-params", type=Path, default=None,
                help="robot_params.json — poses.drive 를 팔 자세로 (새 모델 joint1~5 이름). 없으면 S0(RMX52_ 이름)")
args, _ = ap.parse_known_args()
ARM_Q = json.loads(args.arm_params.read_text())["poses"]["drive"]["value"] if args.arm_params else S0
if len(args.poses) == 1:
    args.poses = args.poses * len(args.worlds)

STATES = ("off", "red", "yellow", "green")
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
ROBOT = "/World/waffle_4wd"
pkg_dir = args.urdf.parent.parent
drawing = json.loads(args.drawing.read_text())

# ---------- 자세 · 퍽 뽑기 (렌더 전, 같은 seed 면 같은 결과) ----------
rng = np.random.default_rng(args.seed)
plan = []
t_plan = time.time()
for wi, wdir in enumerate(args.worlds):
    lay = json.loads((wdir / "layout.json").read_text())
    sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, arm_q=ARM_Q)
    distances = [float(np.linalg.norm(np.array(args.start_pose[:2]) - sc.light))] if args.start_pose else args.dists
    for dist in distances:
        poses = sc.sample_start_poses(rng, args.start_pose, args.poses[wi], args.pos_jit, args.yaw_jit) if args.start_pose else sc.sample_poses(rng, dist, args.poses[wi])
        for k, pose in enumerate(poses):
            pose["pucks"] = [] if args.start_pose else sc.sample_pucks(rng, pose)
            pose["light_yaw_deg"] = float(lay["signal_light"].get("yaw_deg", 0)) if args.start_pose else float(rng.uniform(0, 360))
            pose["world"] = str(wdir)
            pose["seed"] = int(lay["seed"])
            pose["id"] = f"w{lay['seed']}_d{dist:.2f}_p{k}"
            plan.append(pose)
print(f"자세 {len(plan)}개 뽑음 ({time.time() - t_plan:.1f} s)", flush=True)

if args.only_plan:
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out / "plan.json").write_text(json.dumps({"plan": plan}, indent=1, ensure_ascii=False))
    for p in plan:
        print(p["id"], p.get("geometry_check"), flush=True)
    sys.exit(0)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdShade  # noqa: E402

from robot_usd import arm_link_locals, root_from_footprint, use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def to_gf(T):
    return Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist())


def set_xform(stage, prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(to_gf(T))


def set_camera(stage, path, T_w_opt):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세로 USD 카메라(−Z 를 보고 +Y 가 위)를 둔다."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = K["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / K["fy"])
    cam.CreateFocalLengthAttr(focal)
    # 주점: 오프셋이 양수면 화면 창이 오른쪽 · 위로 가서 주점은 왼쪽 · 아래로 감 → 가로는 −, 세로는 +
    # (2026-10-24 고침: 이전 부호는 반대여서 렌더 주점이 실물 K 와 가로 약 9 · 세로 약 10 px 어긋났음)
    cam.CreateHorizontalApertureOffsetAttr(-(K["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((K["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
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


def ghost_lit(rgb, off_rgb, px):
    # 칸 중심 7×7 에서 꺼짐 영상과의 채널 최대 |차| 평균 > 30 이면 켜진 칸 (261031_0005/judge/vmin_design/ghost_check.py 와 같음)
    u, v = (int(round(c)) for c in px)
    a = rgb[max(v - 3, 0):v + 4, max(u - 3, 0):u + 4].astype(np.int16)
    b = off_rgb[max(v - 3, 0):v + 4, max(u - 3, 0):u + 4].astype(np.int16)
    return float(np.abs(a - b).max(axis=2).mean()) > 30


def set_signal(stage, on, scale):
    for name in ("green", "yellow", "red"):
        sh = UsdShade.Shader(stage.GetPrimAtPath(f"/World/Looks/signal_{name}/PBR"))
        col = stage.GetPrimAtPath(f"/World/SignalLight/{name}").GetAttribute("signal:on_color").Get()
        sh.GetInput("emissiveColor").Set(Gf.Vec3f(*[float(c) * scale for c in col]) if name == on else Gf.Vec3f(0, 0, 0))


def set_lighting(stage, level):
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
    for i in range(6):
        p = stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}")
        if not p.IsValid():
            raise RuntimeError("천장 조명 없음 (월드를 다시 만들 것)")
        p.GetAttribute("inputs:intensity").Set(float(rect))
    return dome, rect


def place_pucks(stage, pucks):
    if stage.GetPrimAtPath("/World/R12Pucks").IsValid():
        stage.RemovePrim("/World/R12Pucks")
    UsdGeom.Xform.Define(stage, "/World/R12Pucks")
    for i, pk in enumerate(pucks):
        path = f"/World/R12Pucks/p{i}"
        if pk["shape"] == "cylinder":
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(0.015)
            g.CreateHeightAttr(0.06)
            g.CreateAxisAttr("Z")
        else:
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp().Set(Gf.Vec3d(pk["xy"][0], pk["xy"][1], pk["half_h"]))
        xf.AddRotateZOp().Set(math.degrees(pk["yaw"]))
        if not pk["standing"]:
            xf.AddRotateXOp().Set(90.0)   # 누움: 긴 축(Z)을 바닥과 나란히
        if pk["shape"] == "cuboid":
            xf.AddScaleOp().Set(Gf.Vec3f(0.03, 0.03, 0.06))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(
            UsdShade.Material(stage.GetPrimAtPath(f"/World/Looks/puck_{pk['color']}")))


def grab(a_rgb, a_dep):
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    return (np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8), np.asarray(a_dep.get_data()).astype(np.float32))


report = {"urdf": str(args.urdf), "frame": args.frame, "width": W, "height": H, "K": K, "arm_q": ARM_Q, "robot_usd": str(args.robot_usd) if args.robot_usd else None,
          "start_pose": args.start_pose, "pos_jit": args.pos_jit, "yaw_jit": args.yaw_jit, "extra_pucks": not bool(args.start_pose),
          "emissive": args.emissive, "lights": args.lights, "seed": args.seed, "dists": args.dists, "poses": []}
t0 = time.time()
for wdir in args.worlds:
    todo = [p for p in plan if p["world"] == str(wdir)]
    if not todo:
        continue
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    lay = json.loads((wdir / "layout.json").read_text())
    if args.robot_usd:   # 새 모델: 그 USD 로, Physics variant none
        use_robot_usd(stage, args.robot_usd)
    else:
        use_robot_usd(stage)   # 월드의 로봇 USD 참조를 기준 모델(261027_1711)로 바꾸고 Physics variant none (2026-10-27 밤)
    updates(10)
    for prim, T in arm_link_locals(stage, args.urdf, ARM_Q).values():   # USD 부모 링크 기준(새 모델은 arm_base_joint 가 합쳐짐, 2026-10-31)
        set_xform(stage, prim, T)
    T_bf_root = root_from_footprint(args.urdf)   # 새 모델 USD 뿌리 = base_link (base_footprint 위 6.75 mm)
    cam_path = "/World/RenderR12/d435"
    set_camera(stage, cam_path, np.array(todo[0]["T_world_cam"]))
    rp = rep.create.render_product(cam_path, (W, H))
    a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    a_rgb.attach(rp)
    a_dep.attach(rp)
    light_prim = stage.GetPrimAtPath("/World/SignalLight")
    sl = lay["signal_light"]
    A = lay["assumptions"]["signal_light"]
    seg_z = {n: A["first_led_z"] + (i + 0.5) * A["segment_length"] for i, n in enumerate(("green", "yellow", "red"))}
    for pose in todo:
        T_w_bf = np.array(pose["T_world_base_footprint"])
        T_w_cam = np.array(pose["T_world_cam"])
        set_camera(stage, cam_path, T_w_cam)
        set_xform(stage, stage.GetPrimAtPath(ROBOT), T_w_bf @ T_bf_root)
        light_prim.GetAttribute("xformOp:rotateZ").Set(float(pose["light_yaw_deg"]))
        place_pucks(stage, pose["pucks"])
        # 로봇 마스크: 첫 조명 · 꺼짐에서 로봇 있음 / 없음 깊이 비교
        set_lighting(stage, args.lights[0])
        set_signal(stage, None, args.emissive)
        updates(4)
        _, dep_with = grab(a_rgb, a_dep)
        UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT)).MakeInvisible()
        updates(4)
        _, dep_without = grab(a_rgb, a_dep)
        UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT)).MakeVisible()
        updates(4)
        mask = np.isfinite(dep_with) & ((dep_with < dep_without - 0.005) | ~np.isfinite(dep_without))
        Tinv = np.linalg.inv(T_w_cam)
        segs = {}
        for n, z in seg_z.items():
            q = Tinv @ np.array([sl["x"], sl["y"], z, 1.0])
            u, v = K["fx"] * q[0] / q[2] + K["cx"], K["fy"] * q[1] / q[2] + K["cy"]
            inside = bool(0 <= u < W and 0 <= v < H and q[2] > 0)
            ui, vi = int(np.clip(round(u), 0, W - 1)), int(np.clip(round(v), 0, H - 1))
            segs[n] = {"world": [sl["x"], sl["y"], z], "px": [round(u, 1), round(v, 1)], "cam_z": round(float(q[2]), 4),
                       "in_image": inside, "robot_occluded": bool(inside and mask[vi, ui]),
                       "depth_at_px": round(float(dep_without[vi, ui]), 4) if inside else None}
        for pk in pose["pucks"]:
            u, v = int(round(pk["px"][0])), int(round(pk["px"][1]))
            pk["robot_occluded"] = bool(mask[v, u])
        for level in args.lights:
            name = f"{pose['id']}_L{level:g}"
            (out / name).mkdir()
            cv2.imwrite(str(out / name / "robot_mask.png"), mask.astype(np.uint8) * 255)
            dome, rect = set_lighting(stage, level)
            images, hsv, retries = {}, {}, {}
            for state in STATES:
                set_signal(stage, state if state != "off" else None, args.emissive)
                updates(4)
                rgb, dep = grab(a_rgb, a_dep)
                # 잔상: 바꾼 emissive 가 늦게 반영되면 앞 상태 칸이 켜진 채 찍힌다(3000 K L1 에서 렌더마다 몇 장).
                # 화면 안 칸마다 꺼짐 영상과 차이로 켜진 칸을 보고, 이 상태 칸 하나가 아니면 더 기다렸다 다시 찍는다
                for k in range(args.ghost_retries if state != "off" else 0):
                    lit = [n for n, sg in segs.items() if sg["in_image"] and ghost_lit(rgb, off_rgb, sg["px"])]
                    if lit == [state] or not segs[state]["in_image"]:
                        break
                    retries[state] = k + 1
                    updates(8)
                    rgb, dep = grab(a_rgb, a_dep)
                if state == "off":
                    off_rgb = rgb
                cv2.imwrite(str(out / name / f"{state}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                np.save(out / name / f"{state}_depth.npy", dep)
                images[state] = {"color": f"{name}/{state}.png", "depth": f"{name}/{state}_depth.npy"}
                if state != "off" and segs[state]["in_image"]:
                    u, v = (int(round(c)) for c in segs[state]["px"])
                    patch = cv2.cvtColor(rgb[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3], cv2.COLOR_RGB2HSV).reshape(-1, 3)
                    hsv[state] = [int(np.median(patch[:, k])) for k in range(3)]
            set_signal(stage, None, args.emissive)
            entry = {k: v for k, v in pose.items() if k != "world"}
            entry.update({"name": name, "world": pose["world"], "light_level": level, "dome": dome, "rect": rect,
                          "segments": segs, "robot_mask_fraction": round(float(mask.mean()), 4),
                          "images": images, "hsv_at_lit_segment": hsv, "ghost_retries": retries})
            report["poses"].append(entry)
            print(name, json.dumps({n: [s["px"], s["in_image"], s["robot_occluded"]] for n, s in segs.items()}),
                  "pucks", len(pose["pucks"]), "hsv", json.dumps(hsv), flush=True)
    a_rgb.detach()
    a_dep.detach()
    rp.destroy()
    updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_r12.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"poses": len(report["poses"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
