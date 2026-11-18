"""팔 카메라 브라켓 비교 — 팔 자세를 옆에서 본 렌더 (모의, 판정 아님). render_armcam_bracket.py 결과 json 의 관절 값을 그대로 쓴다.

옆 카메라: 로봇 오른쪽(로봇 기준 −y)에서 팔 · 선반을 수평으로 본다(로봇 · 선반 · 퍽 그대로 보임). 640 × 480, f 900 px(팔 부분만 크게).
그림마다 팔 카메라 광축(녹색 선, 0.15 m)과 집게 가운데(TCP, 노란 점)를 그린다.
결과: <out>/side_<자세>_<각도>.png
실행 (PC): ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_armcam_bracket_side.py \
    --render Cartlidar/sim_render/results/armcam_bracket_261027 --out <폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from r12_poses import S0, W, H  # noqa: E402
from urdf_chain import fixed_transform, joint_locals  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--render", type=Path, required=True)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--f", type=float, default=900.0)
ap.add_argument("--side-dist", type=float, default=0.85, help="[m] 옆 카메라 → 팔 기준점 수평 거리")
ap.add_argument("--subframes", type=int, default=8)
args, _ = ap.parse_known_args()

meta = json.loads((args.render / "render_armcam_bracket.json").read_text())
URDF = meta["urdf"]
T_w_bf = np.array(meta["T_world_base_footprint"])
TCP = np.array([0.0, 0.0, -0.08295, 1.0])
KS = {"fx": args.f, "fy": args.f, "cx": W / 2, "cy": H / 2}
ROBOT = "/World/waffle_4wd"

# 옆 카메라: 로봇 기준 (x 0.25, y −dist, z 0.30) 에서 +y 를 보고, 그 점을 화면 가운데로
look_bf = np.array([0.24, 0.0, 0.25, 1.0])
eye_bf = np.array([0.24, -args.side_dist, 0.27, 1.0])
look, eye = (T_w_bf @ look_bf)[:3], (T_w_bf @ eye_bf)[:3]
z = look - eye
z /= np.linalg.norm(z)
x = np.cross(z, [0, 0, 1.0])
x /= np.linalg.norm(x)
T_side = np.eye(4)
T_side[:3, 0], T_side[:3, 1], T_side[:3, 2], T_side[:3, 3] = x, np.cross(z, x), z, eye


def qfull(q):
    d = {k: 0.0 for k in S0}
    d.update(q)
    return d


def proj(p):
    v = np.linalg.inv(T_side) @ np.append(p, 1.0)
    return int(round(KS["fx"] * v[0] / v[2] + KS["cx"])), int(round(KS["fy"] * v[1] / v[2] + KS["cy"]))


from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def set_xform(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist()))


def set_camera(stage, path, T_w_opt, Kc):
    """render_r4.py 와 같음."""
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


ctx.open_stage(str((Path(meta["world"]) / "task_world.usda").resolve()))
updates(60)
stage = ctx.get_stage()
stage.GetPrimAtPath(ROBOT).GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
updates(10)
set_xform(stage.GetPrimAtPath(ROBOT), T_w_bf)
stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(600.0)
for i in range(6):
    stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(20000.0)
cam_path = "/World/RenderBracket/side"
set_camera(stage, cam_path, T_side, KS)
rp = rep.create.render_product(cam_path, (W, H))
a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
a_rgb.attach(rp)
done = set()
for e in meta["renders"]:
    key = (e["pose"], e["angle"])
    if key in done:
        continue
    done.add(key)
    q = qfull(e["q"])
    for link, (jname, T) in joint_locals(URDF, q).items():
        if jname.startswith(("RMX52_", "gripper_")):
            set_xform(find_link(stage, link), T)
    updates(10)
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    img = cv2.cvtColor(np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
    T_cam = np.array(e["T_world_cam"])
    p0, p1 = T_cam[:3, 3], T_cam[:3, 3] + 0.15 * T_cam[:3, 2]
    cv2.line(img, proj(p0), proj(p1), (40, 200, 40), 3)
    cv2.circle(img, proj(p0), 5, (40, 200, 40), -1)
    tcp = (T_w_bf @ fixed_transform(URDF, "arm_link_5", q=q) @ TCP)[:3]
    cv2.circle(img, proj(tcp), 6, (0, 230, 255), -1)
    name = f"side_{e['pose']}_{e['angle']:g}.png"
    cv2.imwrite(str(out / name), img)
    print(name, flush=True)
a_rgb.detach()
rp.destroy()
app.close()
