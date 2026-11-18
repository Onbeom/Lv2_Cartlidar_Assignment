"""(261112_0600 사본: --robot_yaw · --box_yaw 인자 추가, 11-12 보기 0.40 m · 50° · face 5 단계 뒤 어긋남 판정용) 5번 통합안 U3: 장소 · 퍽 상자(205 x 205 x 70)를 상자 면 0.20 m 자리에서 box_look 한 번으로 (자세는 --q, 11-12 후보 0.40 m · 55°) 본 렌더 (Isaac Sim 6.1, 모의 자료).

로봇 = B안 light USD · URDF, 원점 = base_footprint. 계획 자리: 앞 끝(0.1328) → 상자 면 0.20 m, 상자 중심 x = 0.4353, y 0, 요 0.
정지 오차 · 밀림: 로봇 요 ±0.05 rad (회전 중심 icr_x −0.0658), 앞뒤 ±0.01 (place_box 는 면까지 재서 섬) · 옆 ±0.03 m (밀림 + 정지). 상자 요 ±10°.
  앞 16 장면 = 꼭짓점 (요 ±0.05 × 상자 ±10° × 앞뒤 ±0.01 × 옆 ±0.03), 나머지 = 범위 안 균등. --record 면 기록 장면 20 개 더.
팔 = box_look 이 계획 상자 중심을 겨눠 낸 자세 (look_tool, 면 0.20, 바른 FK 로 낸 보는 점) 고정. 노드는 계획 자리를 겨누므로 정지 오차는 상자가 움직인 것으로 보임.
카메라 = 인지 노드가 편 영상의 바늘구멍 (newK fx 602.35 · fy 793.36 · cx 530.64 · cy 359.53, 1280 x 720, 가로로 넓게 그린 뒤 가로만 줄임).
상자 위 퍽: 0 · 2 · 5 · 9 개를 3 x 3 칸 (간격 0.0683) 중심 ±3 mm 에, 서 있는 것 (원기둥 · 직육면체) 이 주, 누운 것도 섞음.
  라벨 = 퍽 윗면 · 아랫면 20 mm 흰 라벨 안 16 mm tag36h11 (발제 월드와 같은 텍스처), ID 0 ~ 11 겹치지 않게.
조명 두 단계. 색은 발제 월드 (바닥 0.45 회색, 상자 0.8 · 0.68 · 0.48). 실물 인쇄 · 잡음 · 왜곡 잔차 없음.

    ~/isaacsim-venv/bin/python render_u3.py --out <폴더> --n 60 --seed 1 --q j1 j2 j3 j4 j5
결과: <폴더>/scene_XXX_L{0,1}.png, <폴더>/render_u3.json (장면마다 K, T_bf_cam, 상자 참 중심 · 요, 사전값, 퍽 · 라벨 꼭짓점)
"""
import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--n", type=int, default=60)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--q", type=float, nargs=5, required=True, help="보기 자세 joint1 ~ 5")
ap.add_argument("--shift_x", type=float, default=0.01, help="판정 꼭짓점 · 범위 앞뒤 [m]: place_box 는 approach_box 가 TFmini 로 면까지 재서 섬 → 정지 허용 ±0.01")
ap.add_argument("--shift_y", type=float, default=0.03, help="판정 꼭짓점 · 범위 옆 [m]: 상자 밀림 + 정지 ±0.03")
ap.add_argument("--robot_yaw", type=float, default=0.05, help="로봇 요 범위 ± [rad]")
ap.add_argument("--box_yaw", type=float, default=10.0, help="상자 요 범위 ± [deg]")
ap.add_argument("--record", action="store_true", help="판정 120 장 뒤에 기록 장면 추가: 옆 ±0.05 꼭짓점 8 · survey 밀림 (앞뒤 · 옆 ±0.03) 꼭짓점 8 · 카메라 높이 ±0.03 4")
args, _ = ap.parse_known_args()
B = Path.home() / "git/lv2_next_module/261102_1440_urdf_B/isaac"
URDF = B / "urdf/waffle_4wd_light.urdf"
USD = B / "usd/light/waffle_4wd_light/waffle_4wd_light.usda"
TEX = Path.home() / "git/lv2_next_module/Cartlidar/sim_world/textures"
sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

W, H = 1280, 720
K = {"fx": 602.35, "fy": 793.36, "cx": 530.64, "cy": 359.53}
BOX, BOX_H = 0.205, 0.07
FRONT, FACE = 0.1328, 0.20
PRIOR = np.array([FRONT + FACE + BOX / 2, 0.0])
ICR = np.array([-0.0658, 0.0])
CELL = 0.0683
TAG, LABEL = 0.016, 0.020
Q = {f"joint{i + 1}": v for i, v in enumerate(args.q)}


def robot_from_world(p, yaw, d):
    """로봇이 계획 자리에서 (d 만큼 옮기고) ICR 둘레로 yaw 돈 뒤, 계획 좌표 점 p 의 로봇 좌표."""
    p = np.asarray(p, float) - np.asarray(d)
    c, s = math.cos(-yaw), math.sin(-yaw)
    v = p - ICR
    return ICR + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


rng = np.random.default_rng(args.seed)
SX, SY = args.shift_x, args.shift_y
RY, BYD = args.robot_yaw, args.box_yaw
corners = list(itertools.product((-RY, RY), (-BYD, BYD), (-SX, SX), (-SY, SY)))
scenes = []
rec = []
if args.record:
    rec = [("side5", c, 0.0) for c in list(itertools.product((-0.05, 0.05), (-10.0, 10.0), (-SX, SX), (-0.05, 0.05)))[::2]]
    rec += [("survey3", c, 0.0) for c in list(itertools.product((-0.05, 0.05), (-10.0, 10.0), (-0.03, 0.03), (-0.03, 0.03)))[1::2]]
    rec += [("cam_dz", (0.0, 0.0, 0.0, 0.0), dz) for dz in (-0.03, -0.03, 0.03, 0.03)]
for i in range(args.n + len(rec)):
    kind, cam_dz = "judge", 0.0
    if i < len(corners):
        ryaw, byaw_deg, dx, dy = corners[i]
    elif i < args.n:
        ryaw, byaw_deg, dx, dy = rng.uniform(-RY, RY), rng.uniform(-BYD, BYD), rng.uniform(-SX, SX), rng.uniform(-SY, SY)
    else:
        kind, (ryaw, byaw_deg, dx, dy), cam_dz = rec[i - args.n]
    center = robot_from_world(PRIOR + np.array([dx, dy]), ryaw, (0.0, 0.0))   # 상자가 계획 자리에서 (dx, dy) 밀리고 로봇이 ryaw 돎
    yaw = math.radians(byaw_deg) - ryaw
    Rb = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    k = int(rng.choice([0, 2, 5, 9]))
    cells = list(itertools.product((-1, 0, 1), repeat=2))
    pick = rng.choice(9, size=k, replace=False) if k else []
    ids = rng.permutation(12)[:k]
    pucks = []
    for j, ci in enumerate(pick):
        local = np.array(cells[ci], float) * CELL + rng.uniform(-0.003, 0.003, 2)
        xy = center + Rb @ local
        pkind = str(rng.choice(["stand_cylinder", "stand_cuboid", "stand_cylinder", "stand_cuboid", "lie_cuboid", "lie_cylinder"]))
        pucks.append({"kind": pkind, "xy": xy.tolist(), "yaw": float(yaw + rng.uniform(-math.pi, math.pi)),
                      "color": str(rng.choice(["red", "green"])), "tag_id": int(ids[j]), "cell": [int(c) for c in cells[ci]]})
    scenes.append({"id": f"scene_{i:03d}", "kind": kind, "cam_dz": cam_dz, "corner": i < len(corners), "stop_err": [dx, dy, ryaw], "box_yaw_deg": byaw_deg,
                   "center": center.tolist(), "yaw": yaw, "prior": PRIOR.tolist(), "prior_yaw": 0.0, "pucks": pucks})

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True, "width": int(round(W * K["fy"] / K["fx"])), "height": H, "anti_aliasing": 0, "multi_gpu": False})
import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt  # noqa: E402
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


def material(name, rgb, texture=None):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    s = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PBR")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8 if texture else 0.7)
    if texture:
        rd = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/st_reader")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
        tx = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/texture")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    else:
        s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def cube(path, size, xyz, yaw, mat):
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(g)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    bind(g.GetPrim(), mat)
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
    bind(g.GetPrim(), mat)


def label(path, center, u, v, mat):
    """정사각 라벨 (한 변 LABEL), 앞면 = u x v. 텍스처 s -> u, t -> v. 꼭짓점 (태그 검은 사각형) 을 돌려줌."""
    c, u, v = (np.asarray(a, float) for a in (center, u, v))
    h = LABEL / 2
    pts = [c - h * u - h * v, c + h * u - h * v, c + h * u + h * v, c - h * u + h * v]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    bind(m.GetPrim(), mat)
    t = TAG / 2
    return [(c + a * t * u + b * t * v).tolist() for a, b in ((-1, 1), (1, 1), (1, -1), (-1, -1))]


mats = {"floor": material("floor", (0.45, 0.45, 0.45)), "box": material("box", (0.8, 0.68, 0.48)),
        "red": material("red", (0.85, 0.1, 0.1)), "green": material("green", (0.1, 0.65, 0.25)),
        "white": material("white", (1.0, 1.0, 1.0))}
for t in range(12):
    mats[f"tag{t}"] = material(f"tag{t:02d}", (1, 1, 1), TEX / f"tag36h11_{t:02d}.png")
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


def pose_arm(q, dz=0.0):
    for prim, T in arm_link_locals(stage, str(URDF), q).values():
        set_xform(prim, T)
    T = fixed_transform(str(URDF), "arm_camera_optical_frame", root="base_footprint", q=q)
    Rc, e = T[:3, :3], T[:3, 3] + np.array([0, 0, dz])   # dz: 그리는 카메라만 옮김 (FK 높이 오차 기록), 돌려주는 T 는 정기구학
    m = Gf.Matrix4d(*Rc[:, 0].tolist(), 0, *(-Rc[:, 1]).tolist(), 0, *(-Rc[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    return T


def project(T, p):
    v = np.linalg.inv(T) @ np.r_[p, 1.0]
    return K["fx"] * v[0] / v[2] + K["cx"], K["fy"] * v[1] / v[2] + K["cy"]


# 주점 확인 (render_pick_box.py 와 같음): 흰 얇은 판 두 개를 상자 윗면 높이에, 무게중심 화소가 투영과 맞는 aperture offset 부호
dome.CreateIntensityAttr(600.0)
sun.CreateIntensityAttr(2.0)
T0 = pose_arm(Q)
cal = []
cal_pts = (np.array([PRIOR[0] - 0.06, 0.06, 0.0705]), np.array([PRIOR[0] + 0.06, -0.07, 0.0705]))
for sign in (1.0, -1.0):
    off_h.Set(sign * (K["cx"] / SX - WR / 2) * ha / WR)
    off_v.Set(sign * (K["cy"] - H / 2) * va / H)
    errs = []
    for k, p in enumerate(cal_pts):
        cube(f"/World/cal{k}", (0.012, 0.012, 0.001), tuple(p), 0.0, mats["white"])
        updates(4)
        img = grab().astype(float).sum(axis=2)
        stage.RemovePrim(f"/World/cal{k}")
        updates(2)
        base = grab().astype(float).sum(axis=2)
        d = (np.clip(img - base, 0, None) > 60).astype(np.uint8)
        n, lab, st, cen = cv2.connectedComponentsWithStats(d)
        pu, pv = project(T0, p)
        if n < 2:
            errs.append(1e9)
            print("  흰 판 못 찾음", k, "투영", round(pu), round(pv), flush=True)
            continue
        j = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        u, v = cen[j]
        print(f"  흰 판 {k}: 찾음 ({u:.1f}, {v:.1f}) 투영 ({pu:.1f}, {pv:.1f})", flush=True)
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

report = {"urdf": str(URDF), "usd": str(USD), "K": K, "width": W, "height": H, "box": BOX, "box_h": BOX_H, "face": FACE,
          "frame": "base_footprint", "lights": [0, 1], "seed": args.seed, "aperture_sign": sign, "render_width": WR,
          "principal_check_px": cal, "q": args.q, "scenes": []}
for sc in scenes:
    T = pose_arm(Q, sc["cam_dz"])
    cube("/World/box", (BOX, BOX, BOX_H), (sc["center"][0], sc["center"][1], BOX_H / 2), sc["yaw"], mats["box"])
    if stage.GetPrimAtPath("/World/pucks").IsValid():
        stage.RemovePrim("/World/pucks")
    for i, pk in enumerate(sc["pucks"]):
        path, (x, y), mat, yw = f"/World/pucks/p{i}", pk["xy"], mats[pk["color"]], pk["yaw"]
        cu, su = math.cos(yw), math.sin(yw)
        ax, ay = np.array([cu, su, 0.0]), np.array([-su, cu, 0.0])
        e = 0.0003
        tm = mats[f"tag{pk['tag_id']}"]
        if pk["kind"].startswith("stand"):
            zc = BOX_H + 0.03
            if pk["kind"] == "stand_cuboid":
                cube(path, (0.03, 0.03, 0.06), (x, y, zc), yw, mat)
            else:
                cylinder(path, (x, y, zc), yw, False, mat)
            pk["tag_top"] = label(path + "_lt", (x, y, zc + 0.03 + e), ax, ay, tm)   # 윗면 (앞면 위)
            pk["tag_top_normal"] = [0, 0, 1]
        else:
            zc = BOX_H + 0.015
            if pk["kind"] == "lie_cuboid":
                cube(path, (0.06, 0.03, 0.03), (x, y, zc), yw, mat)
            else:
                cylinder(path, (x, y, zc), yw, True, mat)
            z = np.array([0, 0, 1.0])
            c0 = np.array([x, y, zc])
            pk["tag_ends"] = [label(path + "_l0", c0 + ax * (0.03 + e), ay, z, tm), label(path + "_l1", c0 - ax * (0.03 + e), -ay, z, tm)]
            pk["tag_end_normals"] = [ax.tolist(), (-ax).tolist()]
    for lv in (0, 1):
        dome.CreateIntensityAttr(300.0 + 700.0 * lv)
        sun.CreateIntensityAttr(1.0 + 2.0 * lv)
        updates(4)
        img = grab()
        fn = f"{sc['id']}_L{lv}.png"
        cv2.imwrite(str(args.out / fn), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        report["scenes"].append({**sc, "light": lv, "image": fn, "T_bf_cam": T.tolist()})
    print(sc["id"], "꼭짓점" if sc["corner"] else "", "퍽", [(p["kind"], p["tag_id"]) for p in sc["pucks"]], flush=True)
(args.out / "render_u3.json").write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n")
app.close()
