"""#128 손 안 퍽 렌더 (모의): 집기 끝(쥐고 3 cm 든 자세)의 팔 카메라 영상과 정답 자세를 저장한다.

카메라는 집게에 고정이라 영상의 퍽 위치는 손 안 위치로만 정해지고, 차체 · 팔 자세는 배경만 바꾼다.
  집게 45° 아래(q2 + q3 + q4 = 0.7854, 손목 roll 0). 퍽 중심 = tool_link − 15 mm × 위 + (dx, dy, dz) [집게 링크 축].
  dx · dz ±5 mm, dy ±0.3 mm, 기울기 집게 y 둘레 ±5° · x 둘레 ±1°. 직육면체는 면이 손가락과 나란(±2°), 원기둥은 둘레 회전 무작위.
결과: <out>/<id>.png, <id>_depth.png(16 bit, 0.1 mm), <out>/meta.json
실행 (PC): flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python render_grip.py --worlds <월드> --n 40 --seed 1 --out <폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

TOP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOP / "Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

REPO = Path.home() / "git/Lv2_Cartlidar_Assignment_armcam/workspace/urdf_and_usd"
ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--n", type=int, default=40, help="월드당 장 수")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--urdf", type=Path, default=REPO / "waffle_4wd_description/urdf/waffle_4wd.urdf")
ap.add_argument("--usd", type=Path, default=REPO / "isaac/usd/full/waffle_4wd_full/waffle_4wd_full.usda")
ap.add_argument("--f", type=float, default=424.5)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--cam-pitch", type=float, default=None, help="참고: 카메라 광축 기울기를 URDF 대신 이 값[°]으로 (자리는 그대로)")
ap.add_argument("--max-d", type=float, default=0.005, help="손 안 dx · dz 범위 [m]")
ap.add_argument("--max-tilt", type=float, default=5.0, help="y 둘레 기울기 범위 [°]")
args, _ = ap.parse_known_args()
W, H = 1280, 720
KC = {"fx": args.f, "fy": args.f, "cx": W / 2, "cy": H / 2}
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
FINGER_Q = -0.0058    # 손가락 안쪽 면(링크 y −0.0002)이 퍽 면 ±15 mm 에 닿는 값 (URDF 0.021 + q, 메시에서)
S = math.sqrt(0.5)
URDF = str(args.urdf)
Z_P = np.array([-S, 0.0, S])   # 집게 45° 아래일 때 월드 위를 집게 링크 축으로
def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


TOOL = fixed_transform(URDF, "tool_link", root="gripper_roll_link")[:3, 3]
T_G_CAM = fixed_transform(URDF, "arm_camera_optical_frame", root="gripper_roll_link")
if args.cam_pitch is not None:
    # 광학 프레임을 집게 y 둘레로 더 숙인다 (URDF 15° = 0.261788 rad 기준)
    T_G_CAM[:3, :3] = rot("y", math.radians(args.cam_pitch) - 0.261788) @ T_G_CAM[:3, :3]
# 손가락 윗면에 붙인 검은 표시 (집게 링크 좌표, 한 변 6 mm). 손가락은 퍽을 쥔 폭으로 고정
MARKS_G = [[0.1489, 0.02398, 0.01342], [0.1489, -0.02398, 0.01342]]   # 메시 윗면(법선 x 0.113, z 0.994)에서 0.2 mm 위
MARK_TILT = math.atan2(0.113, 0.994)


def arm_q(q2, q3):
    return {"joint1": 0.0, "joint2": q2, "joint3": q3, "joint4": 0.7854 - q2 - q3, "joint5": 0.0,
            "gripper_left_joint": FINGER_Q, "gripper_right_joint": FINGER_Q}


def t_bf_g(q):
    return fixed_transform(URDF, "gripper_roll_link", root="base_footprint", q=q)


def nominal_center():
    return TOOL - 0.015 * Z_P


def solve_q(rng, z_target):
    """퍽 중심(공칭)이 base_footprint 위 z_target 이 되는 (q2, q3). 관절 한계 안에서 q3 를 뽑고 q2 를 푼다."""
    c = np.append(nominal_center(), 1.0)
    for _ in range(50):
        q3 = float(rng.uniform(0.3, 1.2))
        f = lambda q2: (t_bf_g(arm_q(q2, q3)) @ c)[2] - z_target  # noqa: E731
        try:
            q2 = brentq(f, -1.4, 0.6, xtol=1e-5)
        except ValueError:
            continue
        q4 = 0.7854 - q2 - q3
        reach = (t_bf_g(arm_q(q2, q3)) @ c)[0]
        if -1.9 < q4 < 2.0 and reach > 0.28:
            return q2, q3
    raise RuntimeError("자세를 못 풀었다")


def box_clear(xy, boxes, r=0.22):
    return all(np.hypot(*(xy - np.array(b["center"][:2]))) > r + 0.15 for b in boxes)


from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdShade, Vt  # noqa: E402

from robot_usd import pose_robot, set_xform, use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()
rng = np.random.default_rng(args.seed)
meta = {"K": KC, "W": W, "H": H, "marks_G": MARKS_G, "max_d": args.max_d, "max_tilt": args.max_tilt, "urdf": URDF, "usd": str(args.usd), "T_G_cam": T_G_CAM.tolist(), "tool": TOOL.tolist(),
        "seed": args.seed, "frames": []}


def updates(n):
    for _ in range(n):
        app.update()


def set_camera(stage, path, T_w_opt, near=0.012):
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = KC["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / KC["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, 50.0))
    R, e = T_w_opt[:3, :3], T_w_opt[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def smooth_cylinders(stage, n=128):
    """렌더 기본 Cylinder 는 면이 각져(눈으로 약 10각형) 윤곽 오차가 생겨, 같은 치수 · 재질의 128각 메시로 바꾼다 (모양 오차를 시험에서 뺀다)."""
    for puck in stage.GetPrimAtPath("/World/Pucks").GetChildren():
        body = puck.GetChild("body")
        if not body or body.GetTypeName() != "Cylinder":
            continue
        cyl = UsdGeom.Cylinder(body)
        r, h = cyl.GetRadiusAttr().Get(), cyl.GetHeightAttr().Get()
        pts, counts, idx = [], [], []
        for i in range(n):
            a = 2 * math.pi * i / n
            pts += [(r * math.cos(a), r * math.sin(a), -h / 2), (r * math.cos(a), r * math.sin(a), h / 2)]
        for i in range(n):
            j = (i + 1) % n
            counts.append(4)
            idx += [2 * i, 2 * j, 2 * j + 1, 2 * i + 1]
        counts += [n, n]
        idx += [2 * i + 1 for i in range(n)] + [2 * i for i in reversed(range(n))]
        mesh = UsdGeom.Mesh.Define(stage, puck.GetPath().AppendChild("body_hi"))
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*q) for q in pts]))
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-r, -r, -h / 2), Gf.Vec3f(r, r, h / 2)]))
        mat = UsdShade.MaterialBindingAPI(body).ComputeBoundMaterial()[0]
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        UsdGeom.Imageable(body).MakeInvisible()


def set_lighting(stage, level):
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(float(rect))


for wdir in args.worlds:
    lay = json.loads((wdir / "layout.json").read_text())
    pucks = [p for p in lay["pucks"] if p["standing"] and abs(p["center"][2] - 0.1) < 0.002]
    orig = {p["name"]: p for p in pucks}
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    use_robot_usd(stage, args.usd)
    smooth_cylinders(stage)
    updates(10)
    path = "/World/GripCam"
    set_camera(stage, path, np.eye(4))
    mark_prims = []
    for i in range(2):
        c = UsdGeom.Cube.Define(stage, f"/World/GripMark_{i}")
        c.CreateSizeAttr(1.0)
        c.CreateDisplayColorAttr([Gf.Vec3f(0.01, 0.01, 0.01)])
        mark_prims.append(c.GetPrim())
    rp = rep.create.render_product(path, (W, H))
    a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    a_rgb.attach(rp)
    a_dep.attach(rp)
    for k in range(args.n):
        # 모양을 번갈아 뽑는다
        want = "cylinder" if k % 2 == 0 else "cuboid"
        cand = [p for p in pucks if p["shape"] == want]
        p = cand[int(rng.integers(len(cand)))]
        level = float(rng.uniform(0.2, 1.0))
        set_lighting(stage, level)
        d = np.array([rng.uniform(-args.max_d, args.max_d), rng.uniform(-.0003, .0003), rng.uniform(-args.max_d, args.max_d)])
        th, ep = float(np.radians(rng.uniform(-args.max_tilt, args.max_tilt))), float(np.radians(rng.uniform(-1, 1)))
        e1 = np.array([0.0, 1.0, 0.0])
        e2 = np.cross(Z_P, e1)
        R_nom = np.column_stack([e1, e2, Z_P])   # 퍽 x · y · z (집게 링크 축으로)
        spin = np.radians(rng.uniform(-2, 2)) + (int(rng.integers(4)) * math.pi / 2) if want == "cuboid" else rng.uniform(0, 2 * math.pi)
        R_G_p = rot("y", th) @ rot("x", ep) @ R_nom @ rot("z", float(spin))
        c_G = nominal_center() + d
        T_G_p = np.eye(4)
        T_G_p[:3, :3], T_G_p[:3, 3] = R_G_p, c_G
        # 차체: 퍽 원래 자리 3 cm 위에 공칭 퍽 중심이 오게
        q2, q3 = solve_q(rng, 0.13)
        q = arm_q(q2, q3)
        T_bf_G = t_bf_g(q)
        pos = np.array(p["center"]) + [0, 0, 0.03]
        c_bf = (T_bf_G @ np.append(nominal_center(), 1.0))[:3]
        for _ in range(100):
            yaw = float(rng.uniform(-math.pi, math.pi))
            Rz = rot("z", yaw)
            xy = pos[:2] - (Rz @ c_bf)[:2]
            if box_clear(xy, [b for b in lay["boxes"] if b["name"] != p["location"]]):
                break
        T_w_bf = np.eye(4)
        T_w_bf[:3, :3], T_w_bf[:3, 3] = Rz, [xy[0], xy[1], 0.0]
        T_w_G = T_w_bf @ T_bf_G
        T_w_cam = T_w_G @ T_G_CAM
        pose_robot(stage, URDF, T_w_bf, q)
        set_camera(stage, path, T_w_cam)
        for n, o in orig.items():    # 다른 퍽은 제자리
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = np.array(o["R"]), o["center"]
            set_xform(stage.GetPrimAtPath(f"/World/Pucks/{n}"), T)
        set_xform(stage.GetPrimAtPath(f"/World/Pucks/{p['name']}"), T_w_G @ T_G_p)
        for mp_, m_ in zip(mark_prims, MARKS_G):
            T_m = np.eye(4)
            T_m[:3, :3] = rot("y", MARK_TILT) @ np.diag([0.006, 0.006, 0.0002])
            T_m[:3, 3] = m_
            set_xform(mp_, T_w_G @ T_m)
        updates(4)
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=args.subframes)
        rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
        dep = np.asarray(a_dep.get_data()).astype(np.float32)
        fid = f"{wdir.name}_{k:03d}"
        cv2.imwrite(str(out / f"{fid}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(out / f"{fid}_depth.png"), np.clip(np.nan_to_num(dep, posinf=6.5) * 1e4, 0, 65535).astype(np.uint16))
        meta["frames"].append({"id": fid, "world": str(wdir), "puck": p["name"], "shape": want, "color": p["color"], "light": level,
                               "T_G_puck": T_G_p.tolist(), "d": d.tolist(), "tilt_y_deg": math.degrees(th), "tilt_x_deg": math.degrees(ep),
                               "q": q, "T_w_bf": T_w_bf.tolist()})
    (out / "meta.json").write_text(json.dumps(meta, indent=0))
app.close()
