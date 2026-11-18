"""집는 상자 (시작 박스 205 x 205 x 70) 윗면을 누운 퍽 스킬 자리에서 팔 Arducam 으로 본 렌더 (Isaac Sim 6.1, 모의 자료).

로봇 = B안 light USD · URDF, 원점 = base_footprint. 자리 = 미션 계획기 회전 접근 자리 (앞 끝 -> 면 0.1272, 로봇 중심 둘레 turn):
경우 L (turn +17.06°) · R (−17.06°) · I (inline 0°). 정지 오차 앞뒤 · 옆 ±0.01 m, 방향 ±3° (균등).
팔 = case_poses.py 로 고른 보기 자세 joint [j1, −1.80, 0.50, 1.90, 0], j1 = L −0.300 · R +0.305 · I 0.
카메라 = 인지 노드가 편 영상의 바늘구멍 (가로로 넓게 그린 뒤 가로만 줄임, 정사각 화소 렌더 한계) (실측 K · D 를 180° 돌린 뒤 newK: fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53, 1280 x 720).
상자 위 퍽: 집을 퍽 = 누운 직육면체 (자리 ±0.01 m, 요 아무 방향), 옆 퍽 = 없음 · 선 원기둥 · 선 직육면체 · 누운 직육면체 · 누운 원기둥.
조명 두 단계. 색은 발제 월드 (바닥 0.45 회색, 상자 0.8 · 0.68 · 0.48). 실물 상자 인쇄 · 잡음 · 왜곡 잔차는 없음.

    ~/isaacsim-venv/bin/python render_pick_box.py --out <폴더> --n 60 --seed 1
결과: <폴더>/scene_XXX_L{0,1}.png, <폴더>/render_pick_box.json (장면마다 K, T_bf_cam, 상자 참 중심 · 요, 사전 중심 · 요, 퍽)
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--n", type=int, default=60)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--q234", type=float, nargs=3, default=[-1.80, 0.50, 1.90], help="보기 자세 joint2 3 4")
ap.add_argument("--j1", type=float, nargs=3, default=[-0.300, 0.305, 0.0], help="보기 자세 joint1: L R I")
args, _ = ap.parse_known_args()
B = Path.home() / "git/lv2_next_module/261102_1440_urdf_B/isaac"
URDF = B / "urdf/waffle_4wd_light.urdf"
USD = B / "usd/light/waffle_4wd_light/waffle_4wd_light.usda"
sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

W, H = 1280, 720
K = {"fx": 602.35, "fy": 793.36, "cx": 530.64, "cy": 359.53}
BOX, BOX_H = 0.205, 0.07
C, PRE, PUCK_DX = -0.04035, 0.3625, 0.040
CASES = {"L": (17.06, args.j1[0]), "R": (-17.06, args.j1[1]), "I": (0.0, args.j1[2])}
Q234 = tuple(args.q234)
NEIGHBORS = ["none", "stand_cylinder", "stand_cuboid", "lie_cuboid", "lie_cylinder"]


def to_robot(w, psi):
    """돌기 전 로봇 좌표 w (2) -> 로봇 중심 둘레로 psi 돈 뒤 로봇 좌표."""
    R = np.array([[math.cos(-psi), -math.sin(-psi)], [math.sin(-psi), math.cos(-psi)]])
    return R @ (np.asarray(w) - np.array([C, 0.0])) + np.array([C, 0.0])


def q_of(case):
    return {"joint1": CASES[case][1], "joint2": Q234[0], "joint3": Q234[1], "joint4": Q234[2], "joint5": 0.0}


rng = np.random.default_rng(args.seed)
scenes = []
for i in range(args.n):
    case = "LRI"[i % 3]
    turn = math.radians(CASES[case][0])
    dx, dy, dth = rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), math.radians(rng.uniform(-3, 3))
    # 돌기 전 로봇 좌표에서 상자 = (PRE, 0), 퍽 두 자리: ±Y 접근은 옆 ±0.04, inline 은 앞뒤 ±0.04
    slots = [np.array([PRE, PUCK_DX]), np.array([PRE, -PUCK_DX])] if case != "I" else [np.array([PRE - PUCK_DX, 0.0]), np.array([PRE + PUCK_DX, 0.0])]
    ti = 0 if case == "L" else 1 if case == "R" else int(rng.integers(2))
    psi = turn + dth
    off = np.array([dx, dy])
    center = to_robot(np.array([PRE, 0.0]) + off, psi)
    yaw = -psi
    Rb = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    pucks = []
    for k, kind in ((ti, "lie_cuboid"), (1 - ti, str(rng.choice(NEIGHBORS)))):
        if kind == "none":
            continue
        local = (slots[k] - np.array([PRE, 0.0])) + rng.uniform(-0.01, 0.01, 2)
        local = np.clip(local, -0.06, 0.06)   # 누운 퍽 (반길이 0.03) 이 상자 밖으로 나가지 않게
        xy = center + Rb @ local
        pucks.append({"kind": kind, "xy": xy.tolist(), "yaw": float(rng.uniform(0, math.pi)),
                      "color": str(rng.choice(["red", "green"])), "target": k == ti})
    nominal_c = to_robot(np.array([PRE, 0.0]), turn)
    scenes.append({"id": f"scene_{i:03d}", "case": case, "q": q_of(case), "stop_err": [dx, dy, math.degrees(dth)],
                   "center": center.tolist(), "yaw": yaw, "prior": nominal_c.tolist(), "prior_yaw": -turn, "pucks": pucks})

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True, "width": int(round(W * K["fy"] / K["fx"])), "height": H, "anti_aliasing": 0, "multi_gpu": False})
import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade  # noqa: E402
from robot_usd import arm_link_locals, root_from_footprint, set_xform  # noqa: E402

args.out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)


def updates(n):
    for _ in range(n):
        app.update()


def material(name, rgb):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    s = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PBR")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m


def cube(path, size, xyz, yaw, mat):
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(g)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    return g


def cylinder(path, xyz, yaw, lying, mat):
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(0.015)
    g.CreateHeightAttr(0.06)
    g.CreateAxisAttr("X" if lying else "Z")
    xf = UsdGeom.Xformable(g)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)


mats = {"floor": material("floor", (0.45, 0.45, 0.45)), "box": material("box", (0.8, 0.68, 0.48)),
        "red": material("red", (0.85, 0.1, 0.1)), "green": material("green", (0.1, 0.65, 0.25)),
        "white": material("white", (1.0, 1.0, 1.0))}
cube("/World/floor", (4.0, 4.0, 0.02), (0.5, 0.0, -0.01), 0.0, mats["floor"])
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25, 15, 0))
robot = stage.DefinePrim("/World/waffle_4wd", "Xform")
robot.GetReferences().AddReference(str(USD))
updates(30)
vs = robot.GetVariantSets()
if vs.HasVariantSet("Physics"):
    vs.GetVariantSet("Physics").SetVariantSelection("none")
updates(10)
set_xform(robot, root_from_footprint(str(URDF)))

cam = UsdGeom.Camera.Define(stage, "/World/armcam")
# Isaac 렌더는 정사각 화소 (fx = fy) 라 fx 602 · fy 793 을 바로 못 냄 -> 가로 WR 화소로 fx' = fy 로 그린 뒤 가로만 W 로 줄임
WR = int(round(W * K["fy"] / K["fx"]))
SX = W / WR
ha = 20.955
focal = (K["fx"] / SX) * ha / WR
va = focal * H / K["fy"]
cam.CreateHorizontalApertureAttr(ha)
cam.CreateVerticalApertureAttr(va)
cam.CreateFocalLengthAttr(focal)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
off_h = cam.CreateHorizontalApertureOffsetAttr(0.0)
off_v = cam.CreateVerticalApertureOffsetAttr(0.0)
rp = rep.create.render_product("/World/armcam", (WR, H))
a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
a_rgb.attach(rp)


def grab():
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=8)
    img = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


def pose_arm(q):
    for prim, T in arm_link_locals(stage, str(URDF), q).values():
        set_xform(prim, T)
    T = fixed_transform(str(URDF), "arm_camera_optical_frame", root="base_footprint", q=q)
    Rc, e = T[:3, :3], T[:3, 3]
    m = Gf.Matrix4d(*Rc[:, 0].tolist(), 0, *(-Rc[:, 1]).tolist(), 0, *(-Rc[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    return T


def project(T, p):
    v = np.linalg.inv(T) @ np.r_[p, 1.0]
    return K["fx"] * v[0] / v[2] + K["cx"], K["fy"] * v[1] / v[2] + K["cy"]


# 주점 확인: 흰 얇은 판 (12 mm) 을 화면 왼쪽 위 · 오른쪽 아래에 두고 무게중심 화소가 투영과 맞는 aperture offset 부호를 고름
dome.CreateIntensityAttr(600.0)
sun.CreateIntensityAttr(2.0)
T0 = pose_arm(q_of("I"))
cal = []
for sign in (1.0, -1.0):
    off_h.Set(sign * (K["cx"] / SX - WR / 2) * ha / WR)
    off_v.Set(sign * (K["cy"] - H / 2) * va / H)
    errs = []
    for k, p in enumerate((np.array([0.30, 0.06, 0.0705]), np.array([0.42, -0.08, 0.0705]))):
        # 얇은 흰 판 (윗면 = 상자 윗면 높이 0.071): 옆면이 안 보여 무게중심 = 판 중심 투영
        cube(f"/World/cal{k}", (0.012, 0.012, 0.001), tuple(p), 0.0, mats["white"])
        updates(4)
        img = grab().astype(float).sum(axis=2)
        stage.RemovePrim(f"/World/cal{k}")
        updates(2)
        base = grab().astype(float).sum(axis=2)
        d = (np.clip(img - base, 0, None) > 60).astype(np.uint8)   # 렌더 잡음은 빼고 흰 상자만
        n, lab, st, cen = cv2.connectedComponentsWithStats(d)
        pu, pv = project(T0, p)
        if n < 2:
            errs.append(1e9)
            print("  흰 상자 못 찾음", k, "투영", round(pu), round(pv), flush=True)
            continue
        j = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        u, v = cen[j]
        print(f"  흰 상자 {k}: 찾음 ({u:.1f}, {v:.1f}) 면적 {st[j, cv2.CC_STAT_AREA]} 투영 ({pu:.1f}, {pv:.1f})", flush=True)
        errs.append(math.hypot(u - pu, v - pv))
    cal.append((max(errs), sign))
    print("주점 확인 부호", sign, "최대 오차 px", round(max(errs), 2), flush=True)
cal.sort()
if cal[0][0] > 3.0:
    print(f"주점 확인 실패: {cal}", flush=True)
    app.close()
    sys.exit(1)
sign = cal[0][1]
off_h.Set(sign * (K["cx"] / SX - WR / 2) * ha / WR)
off_v.Set(sign * (K["cy"] - H / 2) * va / H)

report = {"urdf": str(URDF), "usd": str(USD), "K": K, "width": W, "height": H, "box": BOX, "box_h": BOX_H,
          "frame": "base_footprint", "lights": [0, 1], "seed": args.seed, "aperture_sign": sign, "render_width": WR,
          "principal_check_px": cal, "q234": list(Q234), "j1": list(args.j1), "scenes": []}
for sc in scenes:
    T = pose_arm(sc["q"])
    cube("/World/box", (BOX, BOX, BOX_H), (sc["center"][0], sc["center"][1], BOX_H / 2), sc["yaw"], mats["box"])
    if stage.GetPrimAtPath("/World/pucks").IsValid():
        stage.RemovePrim("/World/pucks")
    for i, pk in enumerate(sc["pucks"]):
        path, (x, y), mat = f"/World/pucks/p{i}", pk["xy"], mats[pk["color"]]
        if pk["kind"] == "stand_cuboid":
            cube(path, (0.03, 0.03, 0.06), (x, y, BOX_H + 0.03), pk["yaw"], mat)
        elif pk["kind"] == "lie_cuboid":
            cube(path, (0.06, 0.03, 0.03), (x, y, BOX_H + 0.015), pk["yaw"], mat)
        elif pk["kind"] == "stand_cylinder":
            cylinder(path, (x, y, BOX_H + 0.03), pk["yaw"], False, mat)
        else:
            cylinder(path, (x, y, BOX_H + 0.015), pk["yaw"], True, mat)
    for lv in (0, 1):
        dome.CreateIntensityAttr(300.0 + 700.0 * lv)
        sun.CreateIntensityAttr(1.0 + 2.0 * lv)
        updates(4)
        img = grab()
        fn = f"{sc['id']}_L{lv}.png"
        cv2.imwrite(str(args.out / fn), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        report["scenes"].append({**sc, "light": lv, "image": fn, "T_bf_cam": T.tolist()})
    print(sc["id"], sc["case"], "옆 퍽", [p["kind"] for p in sc["pucks"] if not p["target"]], flush=True)
(args.out / "render_pick_box.json").write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n")
app.close()
