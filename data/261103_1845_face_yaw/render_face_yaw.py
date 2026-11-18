"""#81 상자 퍽 면 요 정확도용 렌더 (Isaac Sim 6.1, 모의 자료). 로봇 = B안 light USD · URDF(팀 dev 와 같음).

로봇은 최종 자리 (원점 = base_footprint), 팔 = view_pose.py 로 고른 내려다보기 자세. 상자 205 x 205 x 70 을 요 ±30° · 옆 ±5 cm 로 두고,
앞뒤는 TFmini 빔 (y 0, 바닥 위 31 mm) 이 닿는 면이 차체 앞 끝 + 0.02 m 가 되게 맞춤 (주행이 TFmini 로 다가간다고 가정).
상자 위에는 퍽 0 ~ 9 개 (이미 놓인 퍽, 참 상자 기준 칸). 조명 두 단계. 카메라 = 바늘구멍 1280 x 720 · 가로 113° (sensors.yaml 가정).

    ~/isaacsim-venv/bin/python render_place_box.py --out <폴더> --n 60 --seed 1
결과: <폴더>/scene_XXX_L{0,1}.png, <폴더>/render_place_box.json (K, 카메라 자세, 상자 참 중심 · 요 · 꼭짓점, 퍽)
"""
import argparse, json, math, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True); ap.add_argument("--n", type=int, default=60); ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--q", type=float, nargs=3, default=[-0.98, 0.42, 1.54], help="관찰 자세 joint2 3 4 (joint1 = 퍽 방향, joint5 0) — 261103_1821_issue82_view/observe_pose.py")
args, _ = ap.parse_known_args()
B = Path.home() / "git/lv2_next_module/261102_1440_urdf_B/isaac"
URDF = B / "urdf/waffle_4wd_light.urdf"
USD = B / "usd/light/waffle_4wd_light/waffle_4wd_light.usda"
sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

W, H, HFOV = 1280, 720, 1.9721
fx = W / 2 / math.tan(HFOV / 2)
K = {"fx": fx, "fy": fx, "cx": W / 2, "cy": H / 2}
BOX, BOX_H = 0.205, 0.07
ROBOT_FRONT, GAP = 0.1328, 0.06   # 박스 앞 간격 0.06 (#81)
J1 = fixed_transform(str(URDF), "arm_base_link", root="base_footprint", q={})[:3, 3]
qof = lambda j1: {"joint1": float(j1), "joint2": args.q[0], "joint3": args.q[1], "joint4": args.q[2], "joint5": 0.0}


def box_corners(c, yaw):
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return [(c + R @ np.array([sx, sy]) * BOX / 2).tolist() for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def front_hit(c, yaw):
    """y = 0 직선 (+x 방향) 이 상자 테두리와 처음 만나는 x (없으면 None)."""
    P = np.array(box_corners(c, yaw)); xs = []
    for i in range(4):
        a, b = P[i], P[(i + 1) % 4]
        if (a[1] - 0) * (b[1] - 0) <= 0 and a[1] != b[1]:
            xs.append(a[0] + (0 - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
    return min(xs) if xs else None


rng = np.random.default_rng(args.seed); scenes = []
BOX_C = np.array([ROBOT_FRONT + GAP + BOX / 2, 0.0])
while len(scenes) < args.n:
    d, lat = rng.uniform(0.195, 0.215), rng.uniform(-0.06, 0.06)
    xy = np.array([J1[0] + math.sqrt(d * d - lat * lat), J1[1] + lat])
    if np.abs(xy - BOX_C).max() > BOX / 2 - 0.02: continue
    j1 = math.atan2(xy[1] - J1[1], xy[0] - J1[0])
    yaw = j1 + rng.uniform(-math.radians(10), math.radians(10))     # 면 요 참값 = fold(yaw)
    tgt = {"xy": xy.tolist(), "yaw": float(yaw), "tag_id": int(rng.integers(0, 12)), "color": str(rng.choice(["red", "green"])), "shape": "cuboid"}
    others = []
    for _ in range(int(rng.integers(0, 3))):   # 이웃 퍽 (다른 칸, 6 cm 이상 떨어짐)
        o = xy + rng.choice([-0.06, 0.06], 2) * rng.integers(0, 2, 2)
        if np.linalg.norm(o - xy) < 0.059 or np.abs(o - BOX_C).max() > BOX / 2 - 0.02 or any(np.linalg.norm(o - np.array(p["xy"])) < 0.059 for p in others): continue
        others.append({"xy": o.tolist(), "yaw": float(rng.uniform(-math.pi, math.pi)), "tag_id": int(rng.integers(0, 12)),
                       "color": str(rng.choice(["red", "green"])), "shape": str(rng.choice(["cylinder", "cuboid"]))})
    others = [p for p in others if p["tag_id"] != tgt["tag_id"]]
    scenes.append({"id": f"scene_{len(scenes):03d}", "j1": j1, "q": qof(j1), "target": tgt, "others": others})
from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})
import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade  # noqa: E402
from robot_usd import arm_link_locals, root_from_footprint, set_xform  # noqa: E402

args.out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)


def updates(n):
    for _ in range(n):
        app.update()


def material(name, rgb):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    s = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PBR"); s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface"); return m


def cube(path, size, xyz, yaw, mat):
    g = UsdGeom.Cube.Define(stage, path); g.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(g); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz)); xf.AddRotateZOp().Set(math.degrees(yaw)); xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat); return g


# 바닥 · 상자 색은 발제 월드 (dice_mission.sdf: 바닥 0.45 회색, 장소 상자 0.8 · 0.68 · 0.48)
mats = {"floor": material("floor", (0.45, 0.45, 0.45)), "box": material("box", (0.8, 0.68, 0.48)),
        "red": material("red", (0.85, 0.1, 0.1)), "green": material("green", (0.1, 0.65, 0.25))}
cube("/World/floor", (4.0, 4.0, 0.02), (0.5, 0.0, -0.01), 0.0, mats["floor"])
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun"); UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25, 15, 0))
robot = stage.DefinePrim("/World/waffle_4wd", "Xform"); robot.GetReferences().AddReference(str(USD))
updates(30)
vs = robot.GetVariantSets()
if vs.HasVariantSet("Physics"):
    vs.GetVariantSet("Physics").SetVariantSelection("none")
updates(10)
set_xform(robot, root_from_footprint(str(URDF)))

cam = UsdGeom.Camera.Define(stage, "/World/armcam"); ha = 20.955; focal = K["fx"] * ha / W
cam.CreateHorizontalApertureAttr(ha); cam.CreateVerticalApertureAttr(focal * H / K["fy"]); cam.CreateFocalLengthAttr(focal)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
def set_cam(T):
    Rc, e = T[:3, :3], T[:3, 3]
    m = Gf.Matrix4d(*Rc[:, 0].tolist(), 0, *(-Rc[:, 1]).tolist(), 0, *(-Rc[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(m)
def tex_material(name, png):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    s = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PBR"); s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    rd = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/st_reader"); rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    tx = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/texture"); tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(png)))
    for w in ("wrapS", "wrapT"): tx.CreateInput(w, Sdf.ValueTypeNames.Token).Set("clamp")
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result"); tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface"); return m
TEX = Path.home() / "git/lv2_next_module/Cartlidar/sim_world/textures"
tag_mats = {i: tex_material(f"tag_{i:02d}", TEX / f"tag36h11_{i:02d}.png") for i in range(12)}
LAB = 0.016 * 10 / 8 / 2   # 라벨 반폭 (태그 16 mm + 흰 여백 한 칸, build_mission_world 와 같음)
def puck(path, pk):
    x = UsdGeom.Xform.Define(stage, path); x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(pk["xy"][0], pk["xy"][1], BOX_H + 0.03)); x.AddRotateZOp().Set(math.degrees(pk["yaw"]))
    if pk["shape"] == "cuboid":
        g = UsdGeom.Cube.Define(stage, path + "/body"); g.CreateSizeAttr(1.0); UsdGeom.Xformable(g).AddScaleOp().Set(Gf.Vec3f(0.03, 0.03, 0.06))
    else:
        g = UsdGeom.Cylinder.Define(stage, path + "/body"); g.CreateRadiusAttr(0.015); g.CreateHeightAttr(0.06); g.CreateAxisAttr("Z")
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mats[pk["color"]])
    m = UsdGeom.Mesh.Define(stage, path + "/label_top"); z = 0.03 + 2e-4
    m.CreatePointsAttr([Gf.Vec3f(-LAB, -LAB, z), Gf.Vec3f(LAB, -LAB, z), Gf.Vec3f(LAB, LAB, z), Gf.Vec3f(-LAB, LAB, z)])
    m.CreateFaceVertexCountsAttr([4]); m.CreateFaceVertexIndicesAttr([0, 1, 2, 3]); m.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(tag_mats[pk["tag_id"]])
rp = rep.create.render_product("/World/armcam", (W, H)); a_rgb = rep.AnnotatorRegistry.get_annotator("rgb"); a_rgb.attach(rp)


def grab():
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=8)
    return np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)


report = {"urdf": str(URDF), "usd": str(USD), "q_obs": args.q, "K": K, "width": W, "height": H, "box": BOX, "box_h": BOX_H,
          "box_center": BOX_C.tolist(), "frame": "base_footprint", "lights": [0, 1], "seed": args.seed, "scenes": []}
cube("/World/box", (BOX, BOX, BOX_H), (BOX_C[0], BOX_C[1], BOX_H / 2), 0.0, mats["box"])
for sc in scenes:
    for prim, T in arm_link_locals(stage, str(URDF), sc["q"]).values():
        set_xform(prim, T)
    T_bf_cam = fixed_transform(str(URDF), "arm_camera_optical_frame", root="base_footprint", q=sc["q"]); set_cam(T_bf_cam)
    if stage.GetPrimAtPath("/World/pucks").IsValid(): stage.RemovePrim("/World/pucks")
    for i, pk in enumerate([sc["target"]] + sc["others"]): puck(f"/World/pucks/p{i}", pk)
    for lv in (0, 1):
        dome.CreateIntensityAttr(300.0 + 700.0 * lv); sun.CreateIntensityAttr(1.0 + 2.0 * lv)
        updates(4)
        img = grab(); fn = f"{sc['id']}_L{lv}.png"
        cv2.imwrite(str(args.out / fn), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        report["scenes"].append({**sc, "light": lv, "image": fn, "T_bf_cam": T_bf_cam.tolist()})
    print(sc["id"], "j1", round(math.degrees(sc["j1"]), 1), "yaw", round(math.degrees(sc["target"]["yaw"]), 1), "others", len(sc["others"]), flush=True)
(args.out / "render_face_yaw.json").write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n")
app.close()
