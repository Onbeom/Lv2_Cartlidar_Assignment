"""팔 카메라 브라켓 각도 비교 렌더 (모의, 판정 아님): 같은 로봇 · 같은 퍽 앞에서 브라켓만 바꿔 팔 카메라 640 × 480 을 찍는다.

브라켓: 지금 URDF(기준 자세 = 관절 모두 0 에서 광축이 수평 아래 40.83°)와 비교 각도들. 비교 각도는 URDF camera_link 를
  광학 x 축(오른쪽) 둘레로 (40.83 − 각도)° 위로 돌린 것(카메라 원점 그대로 — 브라켓 판이 돌면서 생기는 수 mm 위치 차는 뺌).
  카메라는 arm_link_5(손목 roll 뒤)에 붙어 집게와 함께 움직인다.
자세 (팔 4관절 Base_Yaw · Shoulder · Elbow · Wrist_Pitch, roll 0 · 집게 0):
  zero   : 기준 자세(관절 모두 0)
  grasp  : 집기 직전 — 집게를 수직 아래로, TCP(arm_link_5 (0, 0, −0.08295), 두 손가락 가운데)가 목표 퍽 윗면 0.03 m 위.
           집게 자세가 정해지므로 브라켓이 달라도 팔 자세는 같다 → 브라켓에 따라 퍽이 화면 어디에 보이는지 비교
  look   : 내려다보기(READ_TAG 판정 렌더와 같은 꼴) — 카메라 광축을 수직 아래로, 카메라가 퍽 윗면 0.10 m 위.
           브라켓마다 팔 자세가 달라진다(얕은 브라켓일수록 손목을 더 꺾어야 함) → 관절 값 · 한계를 같이 적음
  자세는 FK(urdf_chain.fixed_transform) 위 수치 최소제곱(scipy least_squares)으로 구함. 충돌 검사는 하지 않음.
장면: Cartlidar/sim_world/worlds/b_dice/seed_1, 로봇은 선반 2 를 정면으로 보고 base_footprint → 선반 가운데 0.32 m(차체 앞 끝과 선반 사이 약 1 cm), 목표 puck_08.
그림에 집게 가운데(TCP) 투영점(노란 십자)을 그린다. 가까운 클리핑 --near(2.5 cm)로 렌즈 둘레 카메라 몸체 메시를 잘라낸다
  (비교 각도는 광학 프레임만 돌려 몸체가 시야에 들어오는데, 실제로는 브라켓과 함께 돌아 보이지 않음).
결과: <out>/<자세>_<각도>.png · _depth.npy(영상면까지 거리 m), <out>/render_armcam_bracket.json
실행 (PC, GPU 잠금):
  flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_armcam_bracket.py --out <폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
TOP = HERE.parents[1]
sys.path.insert(0, str(HERE))
from r12_poses import S0, W, H  # noqa: E402
from urdf_chain import fixed_transform, joint_locals, planar_pose  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--world", type=Path, default=TOP / "Cartlidar/sim_world/worlds/b_dice/seed_1")
ap.add_argument("--urdf", type=Path, default=TOP / "261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf")
ap.add_argument("--angles", type=float, nargs="+", default=[40.83, 15.0], help="기준 자세 광축의 수평 아래 각 [deg] (지금 URDF · 출력한 브라켓)")
ap.add_argument("--target", default="puck_08")
ap.add_argument("--shelf", default="shelf_2")
ap.add_argument("--dist", type=float, default=0.32)
ap.add_argument("--arm-f", type=float, default=283.0)
ap.add_argument("--light", type=float, default=1.0)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--near", type=float, default=0.025, help="[m] 가까운 클리핑 — 렌즈 둘레 카메라 몸체 메시(1.1~1.4 cm)를 잘라냄")
ap.add_argument("--only-plan", action="store_true")
args, _ = ap.parse_known_args()

URDF = str(args.urdf)
BASE_ANGLE = 40.83
JOINTS = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch"]
LIMITS = {"RMX52_Base_Yaw": (-math.pi, math.pi), "RMX52_Shoulder": (-1.5, 1.5), "RMX52_Elbow": (-1.4, 1.5),
          "RMX52_Wrist_Pitch": (-1.97, 1.7)}   # URDF 261022_2119
TCP = np.array([0.0, 0.0, -0.08295, 1.0])
K_ARM = {"fx": args.arm_f, "fy": args.arm_f, "cx": W / 2, "cy": H / 2}
LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}
ROBOT = "/World/waffle_4wd"


def qdict(v):
    q = {k: 0.0 for k in S0}
    q.update(dict(zip(JOINTS, map(float, v))))
    return q


def rx(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[1:3, 1:3] = [[c, -s], [s, c]]
    return T


def cam_bf(q, angle):
    """base_footprint → 브라켓 각도 angle 의 팔 카메라 광학 프레임 (광학 x 둘레로 위로 돌림)."""
    return fixed_transform(URDF, "camera_optical_frame", q=q) @ rx(math.radians(BASE_ANGLE - angle))


def solve(res, x0):
    lo = [LIMITS[j][0] for j in JOINTS]
    hi = [LIMITS[j][1] for j in JOINTS]
    best = None
    for k in range(12):   # 시작점 몇 개
        s = np.clip(np.asarray(x0) + (np.random.default_rng(k).uniform(-0.6, 0.6, 4) if k else 0), lo, hi)
        r = least_squares(res, s, bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
    return best


lay = json.loads((args.world / "layout.json").read_text())
sh = next(s for s in lay["shelves"] if s["name"] == args.shelf)
puck = next(p for p in lay["pucks"] if p["name"] == args.target)
side = {"bottom": 1.0, "top": -1.0}[sh["wall_side"]]   # 벽 반대쪽에서 다가감 (seed_1 선반 2 는 +y 쪽)
c = np.array(sh["center"][:2])
T_w_bf = planar_pose(c[0], c[1] + side * args.dist, -side * math.pi / 2)
Ti = np.linalg.inv(T_w_bf)
top_w = np.array([puck["center"][0], puck["center"][1], puck["center"][2] + 0.03])
top = (Ti @ np.append(top_w, 1))[:3]   # base_footprint 기준 퍽 윗면 가운데

poses = {"zero": {"q": [0.0] * 4, "per_angle": False}}
# 집기 직전: TCP 를 윗면 0.03 m 위, arm_link_5 z = 위(집게 접근 방향 −z 가 아래)
r = solve(lambda v: np.r_[(fixed_transform(URDF, "arm_link_5", q=qdict(v)) @ TCP)[:3] - (top + [0, 0, 0.03]),
                          0.1 * (fixed_transform(URDF, "arm_link_5", q=qdict(v))[:3, 2] - [0, 0, 1])],
          [math.atan2(top[1], top[0]), 0.3, 0.8, 0.5])
poses["grasp"] = {"q": r.x.tolist(), "per_angle": False, "residual": float(np.sqrt(2 * r.cost))}
for ang in args.angles:   # 내려다보기: 브라켓마다
    r = solve(lambda v, ang=ang: np.r_[cam_bf(qdict(v), ang)[:3, 3] - (top + [0, 0, 0.10]),
                                       0.1 * (cam_bf(qdict(v), ang)[:3, 2] - [0, 0, -1])],
              poses["grasp"]["q"])
    poses[f"look_{ang:g}"] = {"q": r.x.tolist(), "angle": ang, "per_angle": True, "residual": float(np.sqrt(2 * r.cost))}

plan = []
for name, p in poses.items():
    for ang in ([p["angle"]] if p.get("angle") is not None else args.angles):
        q = qdict(p["q"])
        T_cam = T_w_bf @ cam_bf(q, ang)
        tcp_w = T_w_bf @ fixed_transform(URDF, "arm_link_5", q=q) @ TCP
        v = np.linalg.inv(T_cam) @ tcp_w
        tv = np.linalg.inv(T_cam) @ np.append(top_w, 1)
        uv = lambda v: [round(K_ARM["fx"] * v[0] / v[2] + K_ARM["cx"], 1), round(K_ARM["fy"] * v[1] / v[2] + K_ARM["cy"], 1)] if v[2] > 0 else None  # noqa: E731
        lim = {j: round(min(q[j] - LIMITS[j][0], LIMITS[j][1] - q[j]), 3) for j in JOINTS}
        plan.append({"pose": name.split("_")[0], "angle": ang, "q": {j: round(q[j], 4) for j in JOINTS},
                     "q_deg": {j: round(math.degrees(q[j]), 1) for j in JOINTS}, "limit_margin_rad": lim,
                     "residual": round(p.get("residual", 0.0), 5), "T_world_cam": T_cam.tolist(),
                     "tcp_px": uv(v), "puck_top_px": uv(tv), "cam_height_above_top_m": round(float(T_cam[2, 3] - top_w[2]), 3)})
for e in plan:
    print(e["pose"], e["angle"], "관절[deg]", e["q_deg"], "잔차", e["residual"], "TCP px", e["tcp_px"], "퍽 윗면 px", e["puck_top_px"],
          "한계 여유", min(e["limit_margin_rad"].values()))
if args.only_plan:
    sys.exit(0)

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


def to_gf(T):
    return Gf.Matrix4d(*np.asarray(T, float).T.reshape(-1).tolist())


def set_xform(prim, T):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(to_gf(T))


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
    cam.CreateClippingRangeAttr(Gf.Vec2f(args.near, 50.0))
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


ctx.open_stage(str((args.world / "task_world.usda").resolve()))
updates(60)
stage = ctx.get_stage()
stage.GetPrimAtPath(ROBOT).GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
updates(10)
set_xform(stage.GetPrimAtPath(ROBOT), T_w_bf)
lv = args.light
stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(
    float(LIGHT_RANGE["dome"][0] + lv * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])))
for i in range(6):
    stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(
        float(LIGHT_RANGE["rect"][0] + lv * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])))
cam_path = "/World/RenderBracket/arm"
set_camera(stage, cam_path, np.array(plan[0]["T_world_cam"]), K_ARM)
rp = rep.create.render_product(cam_path, (W, H))
a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
a_rgb.attach(rp)
a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
a_dep.attach(rp)
for e in plan:
    for link, (jname, T) in joint_locals(URDF, qdict(list(e["q"].values()))).items():
        if jname.startswith(("RMX52_", "gripper_")):
            set_xform(find_link(stage, link), T)
    set_camera(stage, cam_path, np.array(e["T_world_cam"]), K_ARM)
    updates(10)
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if e["tcp_px"] and 0 <= e["tcp_px"][0] < W and 0 <= e["tcp_px"][1] < H:
        cv2.drawMarker(img, tuple(int(v) for v in e["tcp_px"]), (0, 230, 255), cv2.MARKER_CROSS, 24, 2)
    e["image"] = f"{e['pose']}_{e['angle']:g}.png"
    cv2.imwrite(str(out / e["image"]), img)
    np.save(out / e["image"].replace(".png", "_depth.npy"), np.asarray(a_dep.get_data()).astype(np.float32))
    print(e["image"], flush=True)
a_rgb.detach()
a_dep.detach()
rp.destroy()
(out / "render_armcam_bracket.json").write_text(json.dumps({"urdf": URDF, "world": str(args.world), "target": args.target,
                                                            "T_world_base_footprint": T_w_bf.tolist(), "K": K_ARM,
                                                            "renders": plan}, indent=1, ensure_ascii=False))
app.close()
