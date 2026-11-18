"""RealSense 앞·뒤 아래 기울기 35.2°(지금 URDF) vs 10° 비교 렌더(모의, 판정 아님). 같은 로봇 자리에서 컬러 640×480을 두 기울기로 찍는다.

장면: 인지 시험용 과제 미로 Cartlidar/sim_world/worlds/b_dice/seed_1(퍽·선반·신호등 그대로). 로봇은 물리 없이 자세만 놓고 팔은 S0.
렌더 방식은 Cartlidar/sim_render/render_r4.py와 같고, 그 폴더의 urdf_chain.py · r12_poses.py를 불러 쓴다(고치지 않음).
10°는 URDF 카메라 링크(top_camera_*_link) pitch만 0.6144 → 10°로 바꾼 것이다(위치·광학 프레임 관계는 그대로).
실행(PC): ~/isaacsim-venv/bin/python 261027_1057/tools/render_tilt.py --out 261027_1057/results/tilt --/crashreporter/url=
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

TOP = Path(__file__).resolve().parents[2]
SR = TOP / "Cartlidar/sim_render"
sys.path.insert(0, str(SR))
from r12_poses import K, S0, W, H  # noqa: E402
from urdf_chain import fixed_transform, joint_locals, planar_pose, rpy_matrix  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--world", type=Path, default=TOP / "Cartlidar/sim_world/worlds/b_dice/seed_1")
ap.add_argument("--urdf", type=Path, default=TOP / "261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf")
ap.add_argument("--light", type=float, default=0.6)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--tilts", type=float, nargs="+", default=[35.20, 10.0])
ap.add_argument("--arm", choices=["s0", "zero"], default="s0", help="팔 자세: s0 = 접은 자세, zero = URDF 기본(관절 모두 0)")
args, _ = ap.parse_known_args()

TILTS = args.tilts
ARM_Q = S0 if args.arm == "s0" else {k: 0.0 for k in S0}
# (이름, 카메라, base_footprint x, y, yaw[deg], 설명). 거리는 카메라 → 대상 수평 거리
SCENES = [
    ("A_shelf2_floorpuck_0.6m", "front", 0.902, 0.6405, -90, "앞: 선반 2(퍽 5개)까지 0.6 m, 바닥 퍽 0.43 m"),
    ("B_shelf2_0.9m", "front", 0.902, 0.9405, -90, "앞: 선반 2까지 0.9 m"),
    ("C_shelf0_0.5m", "front", -2.0108, 1.8235, 90, "앞: 선반 0(퍽 4개)까지 0.5 m"),
    ("D_signal_0.9m", "front", -0.146, 0.969, 0, "앞: 신호등까지 0.9 m"),
    ("E_rear_shelf0_0.5m", "rear", -2.0108, 1.542, -90, "뒤: 선반 0까지 0.5 m"),
]
ROBOT = "/World/waffle_4wd"
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}


def cam_transform(which, tilt_deg):
    """base_footprint → 컬러 광학 프레임. 카메라 링크 pitch만 tilt로 바꾼다."""
    link = f"top_camera_{which}_link"
    opt = f"top_camera_{which}_color_optical_frame"
    T_bl = fixed_transform(str(args.urdf), link)
    T_bo = fixed_transform(str(args.urdf), opt)
    T_lo = np.linalg.inv(T_bl) @ T_bo
    yaw = 0.0 if which == "front" else math.pi
    T_new = np.eye(4)
    T_new[:3, :3] = rpy_matrix(0.0, math.radians(tilt_deg), yaw)
    T_new[:3, 3] = T_bl[:3, 3]
    return T_new @ T_lo


from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402


def updates(n):
    for _ in range(n):
        app.update()


def to_gf(T):
    return Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist())


def set_xform(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(to_gf(T))


def set_camera(stage, path, T_w_opt, Kc):
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = Kc["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - H / 2) / W * ha)
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


args.out.mkdir(parents=True, exist_ok=True)
ctx = omni.usd.get_context()
ctx.open_stage(str((args.world / "task_world.usda").resolve()))
updates(60)
stage = ctx.get_stage()
stage.GetPrimAtPath(ROBOT).GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
updates(10)
for link, (jname, T) in joint_locals(str(args.urdf), ARM_Q).items():
    if jname.startswith(("RMX52_", "gripper_")):
        set_xform(find_link(stage, link), T)
dome = LIGHT_RANGE["dome"][0] + args.light * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
rect = LIGHT_RANGE["rect"][0] + args.light * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
for i in range(6):
    p = stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}")
    if p.IsValid():
        p.GetAttribute("inputs:intensity").Set(float(rect))
cam_path = "/World/TiltCam"
set_camera(stage, cam_path, np.eye(4), K)
rp = rep.create.render_product(cam_path, (W, H))
a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
a_rgb.attach(rp)
meta = {"world": str(args.world), "light": args.light, "K": K, "tilts_deg": TILTS, "arm": args.arm, "arm_q": ARM_Q,
        "scenes": []}
for name, which, x, y, yaw, desc in SCENES:
    T_w_bf = planar_pose(x, y, math.radians(yaw))
    set_xform(stage.GetPrimAtPath(ROBOT), T_w_bf)
    for tilt in TILTS:
        T_w_cam = T_w_bf @ cam_transform(which, tilt)
        set_camera(stage, cam_path, T_w_cam, K)
        updates(4)
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=args.subframes)
        rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
        fn = f"{name}_tilt{tilt:g}_{args.arm}.png"
        cv2.imwrite(str(args.out / fn), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        meta["scenes"].append({"name": name, "camera": which, "tilt_deg": tilt, "base_footprint": [x, y, yaw],
                               "cam_height_m": float(T_w_cam[2, 3]), "desc": desc, "image": fn})
        print("saved", fn, flush=True)
(args.out / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
app.close()
