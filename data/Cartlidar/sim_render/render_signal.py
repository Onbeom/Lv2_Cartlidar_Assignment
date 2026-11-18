"""신호등 상태 영상 렌더 (R1 · R2 준비) — 로봇 자세에서 실제 D435 장착 위치(URDF)로 찍는다.

로봇 자세: 신호등에서 그 칸 가운데 쪽으로 --dist [m](차체 원점 base_footprint 에서 신호등 중심까지 수평 거리)만큼 떨어져
신호등을 정면으로 본다(--bearing 으로 좌우로 돌림). 카메라 자세 = 로봇 자세 × URDF base_footprint → --frame.
--robot-body 가 없으면 로봇 몸체 · 팔은 렌더에 넣지 않는다(월드의 로봇은 출발 칸에 그대로 있음).
--robot-body 를 주면 월드의 로봇을 그 자세로 옮기고 팔을 --arm-q(기본 접은 주행 자세 S0)로 둔다. 물리는 돌리지 않고
USD 링크 변환에 관절 값을 직접 넣는다(로봇 Physics variant 는 none). 자세마다 로봇을 숨긴 깊이와 비교해 로봇이 가린 화소(robot_mask.png)와
신호등 칸 중심이 가려졌는지(segments.*.robot_occluded)를 기록한다.

상태 4개(off · red · yellow · green)를 장마다 렌더해 저장한다. 시간 흐름은 replay_ros.py 가 이 그림들을 시간표대로 이어 만든다.
결과: <out>/<pose>/<state>.png · <state>_depth.npy(32FC1, m, 영상면까지 거리), <out>/render_signal.json(자세 · 내부값 · 정답)

실행 (PC): ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_signal.py --world Cartlidar/sim_world/worlds/b_dice/seed_1 \
    --out <결과 폴더> --dist 0.6 --emissive 1 \
    --/crashreporter/url=
"""
import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from urdf_chain import fixed_transform, joint_locals, planar_pose  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--world", type=Path, required=True, help="task_world.usda · layout.json 이 있는 폴더")
ap.add_argument("--urdf", type=Path,
                default=HERE.parents[1] / "261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--dist", type=float, nargs="+", default=[0.6], help="차체 원점 → 신호등 수평 거리 [m]")
ap.add_argument("--bearing", type=float, nargs="+", default=[0.0], help="신호등이 정면에서 왼쪽(+)으로 벗어난 각 [deg]")
ap.add_argument("--emissive", type=float, nargs="+", default=[1.0], help="켜진 칸 emissiveColor 배율 — 가정")
ap.add_argument("--subframes", type=int, default=8, help="장마다 RT 누적 횟수")
ap.add_argument("--robot-body", action="store_true", help="로봇 몸체 · 팔을 렌더 자세에 둔다")
ap.add_argument("--arm-q", type=float, nargs=5, default=[0.0, 1.4, -1.0, -1.3, 0.0],
                help="팔 5관절 [rad] yaw · shoulder · elbow · pitch · roll (기본 S0, 변경요약_시뮬레이션_261023.md)")
ap.add_argument("--gripper", type=float, default=0.0, help="gripper_left_joint [m] (오른쪽은 mimic)")
args, _ = ap.parse_known_args()   # 나머지(--/crashreporter/url= 등)는 Kit 설정으로

W, H = 640, 480
D435 = {"fx": 605.850769, "fy": 605.67987, "cx": 324.365814, "cy": 245.139175}   # 실물 261822072247 컬러 640x480
STATES = ("off", "red", "yellow", "green")
T_bf_cam = fixed_transform(str(args.urdf), args.frame)
T_bf_bl = fixed_transform(str(args.urdf), "base_link")
ARM_JOINTS = ("RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed")
ARM_Q = dict(zip(ARM_JOINTS, args.arm_q), gripper_left_joint=args.gripper)
ROBOT = "/World/waffle_4wd"
CHECK_LINKS = ("arm_link_5", "gripper_left_link", "gripper_right_link", "camera_optical_frame", args.frame)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdShade  # noqa: E402

from robot_usd import use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
lay = json.loads((args.world / "layout.json").read_text())
ctx = omni.usd.get_context()
t0 = time.time()
ok = ctx.open_stage(str((args.world / "task_world.usda").resolve()))
for _ in range(60):
    app.update()
stage = ctx.get_stage()

sl = lay["signal_light"]
A = lay["assumptions"]["signal_light"]
seg_z = {n: A["first_led_z"] + (i + 0.5) * A["segment_length"] for i, n in enumerate(("green", "yellow", "red"))}
cell = next(c for c in lay["cells"] if c["id"] == sl["cell"])
toward = np.array(cell["center"]) - np.array([sl["x"], sl["y"]])
toward /= np.linalg.norm(toward)
light_xy = np.array([sl["x"], sl["y"]])


def set_camera(path, T_w_opt):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세로 USD 카메라를 둔다. USD 카메라는 -Z 를 보고 +Y 가 위."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = D435["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / D435["fy"])
    cam.CreateFocalLengthAttr(focal)
    # 주점 부호(2026-10-24 고침, render_r12.py 와 같음): 가로 −, 세로 +
    cam.CreateHorizontalApertureOffsetAttr(-(D435["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((D435["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    R, e = T_w_opt[:3, :3], T_w_opt[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    return path


def project(T_w_opt, p):
    q = np.linalg.inv(T_w_opt) @ np.append(np.asarray(p, float), 1.0)
    return D435["fx"] * q[0] / q[2] + D435["cx"], D435["fy"] * q[1] / q[2] + D435["cy"], q[2]


def to_gf(T):
    """열 벡터 4 × 4 → USD(행 벡터) Matrix4d."""
    return Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist())


def from_gf(m):
    return np.array([[m[r][c] for c in range(4)] for r in range(4)]).T


def set_xform(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(to_gf(T))


def find_link(name):
    hits = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ROBOT}/Geometry")) if p.GetName() == name and
            p.GetParent().GetName() != name]   # 링크 Xform 아래 같은 이름의 메시 묶음은 건너뜀
    if len(hits) != 1:
        raise RuntimeError(f"링크 {name} 이 {len(hits)}개")
    return hits[0]


def pose_robot_arm():
    """팔 · 집게 링크의 부모 기준 변환을 관절 값으로 바꾼다. 바퀴는 0."""
    done = {}
    for link, (jname, T) in joint_locals(str(args.urdf), ARM_Q).items():
        if jname.startswith(("RMX52_", "gripper_")):
            set_xform(find_link(link), T)
            done[jname] = ARM_Q.get(jname, None)
    return done


def check_robot(T_w_bf):
    """USD 가 계산한 링크 월드 변환 = URDF 기구학 × 로봇 자세 인지 확인(위치 m · 회전 deg 최대 차)."""
    cache = UsdGeom.XformCache()
    out = {}
    for link in CHECK_LINKS:
        hits = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ROBOT}/Geometry")) if p.GetName() == link]
        if not hits:
            out[link] = "USD 에 없음"
            continue
        T_usd = from_gf(cache.GetLocalToWorldTransform(hits[0]))
        T_ref = T_w_bf @ fixed_transform(str(args.urdf), link, q=ARM_Q)
        dR = T_usd[:3, :3].T @ T_ref[:3, :3]
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
        out[link] = {"pos_err_m": float(np.linalg.norm(T_usd[:3, 3] - T_ref[:3, 3])), "rot_err_deg": ang}
    return out


def set_signal(on, scale):
    for name in ("green", "yellow", "red"):
        sh = UsdShade.Shader(stage.GetPrimAtPath(f"/World/Looks/signal_{name}/PBR"))
        col = stage.GetPrimAtPath(f"/World/SignalLight/{name}").GetAttribute("signal:on_color").Get()
        val = Gf.Vec3f(*[float(c) * scale for c in col]) if name == on else Gf.Vec3f(0, 0, 0)
        sh.GetInput("emissiveColor").Set(val)
    for _ in range(4):
        app.update()


cam_path = "/World/RenderSignal/d435"
rp = a_rgb = a_dep = None
report = {"world": str(args.world), "urdf": str(args.urdf), "frame": args.frame, "open_ok": bool(ok), "width": W,
          "height": H, "K": D435, "light_level": lay["light_level"], "signal_light": sl,
          "T_base_footprint_cam": T_bf_cam.tolist(), "robot_body": args.robot_body,
          "arm_q": ARM_Q if args.robot_body else None, "poses": []}
if args.robot_body:
    # 렌더 한 장마다 타임라인이 돌아 PhysX 가 링크 변환을 덮어쓴다(첫 실행에서 팔이 180° 뒤집힘,
    # results/render_body_s0_261024_fail_physics). 로봇만 Physics variant 를 none(형상만)으로 바꾼다.
    use_robot_usd(stage)   # 월드의 로봇 USD 참조를 기준 모델(261027_1711)로 바꾸고 Physics variant none (2026-10-27 밤)
    for _ in range(10):
        app.update()
    report["robot_physics_variant"] = "none"
    report["arm_joints_set"] = pose_robot_arm()
for i, (dist, bearing, em) in enumerate(itertools.product(args.dist, args.bearing, args.emissive)):
    # 신호등이 정면에서 bearing 만큼 왼쪽에 보이도록: 로봇 위치는 신호등 → 칸 쪽 방향, yaw 는 신호등 방향에서 -bearing
    pos = light_xy + dist * toward
    yaw = math.atan2(-toward[1], -toward[0]) - math.radians(bearing)
    T_w_bf = planar_pose(pos[0], pos[1], yaw)
    T_w_cam = T_w_bf @ T_bf_cam
    set_camera(cam_path, T_w_cam)
    if args.robot_body:
        set_xform(stage.GetPrimAtPath(ROBOT), T_w_bf)
        UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT)).MakeVisible()
    if rp is None:
        rp = rep.create.render_product(cam_path, (W, H))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach(rp)
        a_dep.attach(rp)
    name = f"p{i:02d}_d{dist:.2f}_b{bearing:+.0f}_e{em:g}"
    (out / name).mkdir()
    T_bl_w = np.linalg.inv(T_w_bf @ T_bf_bl)
    seg = {}
    for n, z in seg_z.items():
        pw = [sl["x"], sl["y"], z]
        u, v, depth = project(T_w_cam, pw)
        seg[n] = {"world": pw, "base_link": (T_bl_w @ np.append(pw, 1.0))[:3].round(4).tolist(),
                  "px": [round(u, 1), round(v, 1)], "cam_z": round(float(depth), 4),
                  "in_image": bool(0 <= u < W and 0 <= v < H and depth > 0)}
    pose = {"name": name, "dist": dist, "bearing_deg": bearing, "emissive": em,
            "robot_xy_yaw": [float(pos[0]), float(pos[1]), float(yaw)], "T_world_base_footprint": T_w_bf.tolist(),
            "T_world_cam": T_w_cam.tolist(), "segments": seg, "images": {}}
    if args.robot_body:
        pose["robot_check"] = check_robot(T_w_bf)
    for state in STATES:
        set_signal(state if state != "off" else None, em)
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=args.subframes)
        rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
        dep = np.asarray(a_dep.get_data()).astype(np.float32)
        cv2.imwrite(str(out / name / f"{state}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        np.save(out / name / f"{state}_depth.npy", dep)
        hsv = {}
        for n, s in seg.items():
            if s["in_image"]:
                u, v = int(round(s["px"][0])), int(round(s["px"][1]))
                patch = cv2.cvtColor(rgb[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3], cv2.COLOR_RGB2HSV).reshape(-1, 3)
                hsv[n] = [int(np.median(patch[:, k])) for k in range(3)]
        pose["images"][state] = {"color": f"{name}/{state}.png", "depth": f"{name}/{state}_depth.npy", "hsv_at_segments": hsv}
    set_signal(None, em)
    if args.robot_body:
        # 로봇을 숨기고 꺼짐 상태 깊이를 다시 찍어, 로봇이 있을 때 더 가까운 화소 = 로봇이 가린 화소
        dep_with = np.load(out / name / "off_depth.npy")
        UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT)).MakeInvisible()
        for _ in range(4):
            app.update()
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=args.subframes)
        dep_without = np.asarray(a_dep.get_data()).astype(np.float32)
        UsdGeom.Imageable(stage.GetPrimAtPath(ROBOT)).MakeVisible()
        mask = np.isfinite(dep_with) & ((dep_with < dep_without - 0.005) | ~np.isfinite(dep_without))
        cv2.imwrite(str(out / name / "robot_mask.png"), mask.astype(np.uint8) * 255)
        np.save(out / name / "off_depth_no_robot.npy", dep_without)
        rows = np.where(mask.any(axis=1))[0]
        pose["robot_mask"] = {"fraction": round(float(mask.mean()), 4),
                              "top_row": int(rows.min()) if rows.size else None,
                              "near_pixels_lt_0.05m": int((dep_with < 0.05).sum())}
        for s in seg.values():
            u, v = int(round(s["px"][0])), int(round(s["px"][1]))
            s["robot_occluded"] = bool(s["in_image"] and mask[v, u])
    report["poses"].append(pose)
    print(name, json.dumps({n: [s["px"], s["in_image"], s.get("robot_occluded")] for n, s in seg.items()}),
          json.dumps(pose.get("robot_mask")), flush=True)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_signal.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({k: v for k, v in report.items() if k not in ("poses", "signal_light")}, ensure_ascii=False))
app.close()
