"""상자 범위 ±5 cm · ±45° 가정에서 앞 D435 컬러로 장소 · 퍽 상자(205 x 205 x 70) 윗면 자세를 재는 렌더 (Isaac Sim 6.1, 모의 자료).

로봇 = B안 light USD · URDF, 원점 = base_footprint, 팔 = DRIVE 자세 (manipulator.yaml dev c689a38). 로봇은 대기 자리 (차체 앞 끝 → 지도 상자 면 = standoff 0.45 또는 0.40,
  dev 접근 자세표에서 쓰는 두 값). 장면은 모두 로봇 좌표로 둠:
  Nav2 도착 오차 n = (x, y ±0.05 m, 요 ±0.1 rad), 상자 밀림 d = ±0.05 m, 상자 요 θ (지도 축, 가정 ±45°).
  로봇 좌표 상자 중심 = R(−n_yaw)(NOM + d − n_xy), 요 = θ − n_yaw. 사전값 (perception place_box prior) = 지도 상자 칸 중심의 로봇 좌표 R(−n_yaw)(NOM − n_xy),
  사전 요 = −n_yaw (지도 축, 위치 추정 오차 0 가정). 그래서 사전값 대비 어긋남은 상자 밀림 d 와 θ 뿐이고, 화면 안 위치는 d − n 으로 정해짐.
  꼭짓점 장면: θ ∈ {−45, −15, +15, +45} × n_yaw ±0.1 × d 대각 4 (n_xy = −d, 화면 안 어긋남이 가장 큼) × standoff 2 = 64.
  나머지는 standoff 를 번갈아, θ 는 ±15° 안 · ±45° 안 반씩 균등. 퍽 0 · 2 (시작 박스처럼 상자 축 ±40 mm) · 6 (3 x 3 칸 중 6) 개.
카메라 = 앞 D435 컬러 top_camera_front_color_optical_frame, 640 x 480, K = 실물 값 (fx 605.85 · fy 605.68 · cx 324.37 · cy 245.14, 10-23 기록).
조명 두 단계. 색은 발제 월드. 실물 인쇄 · 잡음 · 왜곡 잔차 없음.

    ~/isaacsim-venv/bin/python render_d435.py --out <폴더> --n 120 --seed 2
결과: <폴더>/scene_XXX_L{0,1}.png, <폴더>/render_u3.json (eval 과 같은 형식: 장면마다 K, T_bf_cam, 상자 참 중심 · 요, 사전값)
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
ap.add_argument("--n", type=int, default=120)
ap.add_argument("--seed", type=int, default=2)
ap.add_argument("--q", type=float, nargs=5, default=[-0.0245, -1.6122, 1.2901, 1.1888, 0.0], help="팔 자세 (기본 DRIVE)")
ap.add_argument("--standoffs", type=float, nargs="+", default=[0.45, 0.40])
ap.add_argument("--corners", type=int, default=64, help="앞에서 몇 개를 꼭짓점 장면으로 (도구 확인 때 줄임)")
args, _ = ap.parse_known_args()
B = Path.home() / "git/lv2_next_module/261102_1440_urdf_B/isaac"
URDF = B / "urdf/waffle_4wd_light.urdf"
USD = B / "usd/light/waffle_4wd_light/waffle_4wd_light.usda"
TEX = Path.home() / "git/lv2_next_module/Cartlidar/sim_world/textures"
sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

W, H = 640, 480
K = {"fx": 605.85, "fy": 605.68, "cx": 324.37, "cy": 245.14}
CAM_FRAME = "top_camera_front_color_optical_frame"
BOX, BOX_H = 0.205, 0.07
FRONT = 0.1328
CELL = 0.06
TAG, LABEL = 0.016, 0.020
Q = {f"joint{i + 1}": v for i, v in enumerate(args.q)}


def R2(t):
    return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])


rng = np.random.default_rng(args.seed)
corners = [(so, th, ny, sx, sy) for so in args.standoffs for th in (-45.0, -15.0, 15.0, 45.0) for ny in (-0.1, 0.1)
           for sx, sy in itertools.product((-1, 1), repeat=2)]
corners = corners[:: max(1, len(corners) // max(args.corners, 1))][: args.corners] if args.corners < len(corners) else corners
scenes = []
for i in range(args.n):
    if i < len(corners):
        so, th, nyaw, sx, sy = corners[i]
        d = np.array([sx, sy]) * 0.05
        nxy = -d
        group = "45" if abs(th) > 15 else "15"
    else:
        so = args.standoffs[i % len(args.standoffs)]
        group = "15" if (i // len(args.standoffs)) % 2 == 0 else "45"
        lim = 15.0 if group == "15" else 45.0
        th = rng.uniform(-lim, lim)
        nyaw = rng.uniform(-0.1, 0.1)
        d = rng.uniform(-0.05, 0.05, 2)
        nxy = rng.uniform(-0.05, 0.05, 2)
    NOM = np.array([FRONT + so + BOX / 2, 0.0])
    center = R2(-nyaw) @ (NOM + d - nxy)
    prior = R2(-nyaw) @ (NOM - nxy)
    yaw = math.radians(th) - nyaw
    Rb = R2(yaw)
    k = int(rng.choice([0, 2, 6]))
    pucks = []
    if k == 2:
        locs = [np.array([-0.04, 0.0]), np.array([0.04, 0.0])]
    elif k == 6:
        cells = list(itertools.product((-1, 0, 1), repeat=2))
        locs = [np.array(cells[c], float) * CELL for c in rng.choice(9, size=6, replace=False)]
    else:
        locs = []
    ids = rng.permutation(12)[:k]
    for j, loc in enumerate(locs):
        xy = center + Rb @ (loc + rng.uniform(-0.003, 0.003, 2))
        pkind = str(rng.choice(["stand_cylinder", "stand_cuboid", "stand_cylinder", "stand_cuboid", "lie_cuboid", "lie_cylinder"]))
        pucks.append({"kind": pkind, "xy": xy.tolist(), "yaw": float(yaw + rng.uniform(-math.pi, math.pi)),
                      "color": str(rng.choice(["red", "green"])), "tag_id": int(ids[j]), "cell": [0, 0]})
    scenes.append({"id": f"scene_{i:03d}", "kind": "judge", "cam_dz": 0.0, "corner": i < len(corners), "group": group, "standoff": so,
                   "box_yaw_deg": th, "nav": [float(nxy[0]), float(nxy[1]), float(nyaw)], "shift": [float(d[0]), float(d[1])],
                   "center": center.tolist(), "yaw": yaw, "prior": prior.tolist(), "prior_yaw": -nyaw, "pucks": pucks})

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

cam = UsdGeom.Camera.Define(stage, "/World/d435")
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
rp = rep.create.render_product("/World/d435", (WR, H))
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
    T = fixed_transform(str(URDF), CAM_FRAME, root="base_footprint", q=q)
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
P0 = FRONT + args.standoffs[0] + BOX / 2
cal_pts = (np.array([P0 - 0.08, 0.10, 0.0705]), np.array([P0 + 0.08, -0.10, 0.0705]))
for sign in ((1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)):   # (가로, 세로) 부호를 따로: D435 는 cy 도 가운데에서 5 px 떨어짐
    off_h.Set(sign[0] * (K["cx"] / SX - WR / 2) * ha / WR)
    off_v.Set(sign[1] * (K["cy"] - H / 2) * va / H)
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
off_h.Set(sign[0] * (K["cx"] / SX - WR / 2) * ha / WR)
off_v.Set(sign[1] * (K["cy"] - H / 2) * va / H)

report = {"urdf": str(URDF), "usd": str(USD), "K": K, "width": W, "height": H, "box": BOX, "box_h": BOX_H, "camera": CAM_FRAME, "standoffs": args.standoffs,
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
    print(sc["id"], sc["group"], sc["standoff"], f"{sc['box_yaw_deg']:+.1f}", "꼭짓점" if sc["corner"] else "", "퍽", [(p["kind"], p["tag_id"]) for p in sc["pucks"]], flush=True)
(args.out / "render_u3.json").write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n")
app.close()
