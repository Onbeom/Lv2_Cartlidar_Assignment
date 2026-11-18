"""CenterPose 비교용 팔 카메라 합성 자료 (Isaac Sim 6.1, 모의). render_face_yaw.py(#81)의 장면을 넓힌 것.

로봇 = B안 light USD · URDF, 카메라 = 바늘구멍 1280 x 720 · 가로 113°(sensors.yaml 가정). 팔 = 관찰 자세 둘레로 흔든 자세(충돌 · 도달 계획은 안 함).
상자 205 x 205 x 70 위 퍽 1~4개. 원기둥은 서 있음, 직육면체는 누울 수 있음(발제 · 사용자). 태그: 원기둥 위 · 아래,
직육면체 6면(build_mission_world). 누운 직육면체 일부는 위 · 아래 끝면만 태그 = 위를 보는 넓은 면에 태그 없음(실물에서 본 경우, 11-06 YOLO 통보).
    ~/isaacsim-venv/bin/python render_cp.py --out <폴더> --n 400 --seed 1003
결과: <폴더>/images/XXXXX.png, <폴더>/truth.json. 퍽 축: 열 x · y · z, z = 긴 축, 위치 · 회전은 카메라 광학 좌표(m).
"""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True); ap.add_argument("--n", type=int, default=20); ap.add_argument("--seed", type=int, default=1)
args, _ = ap.parse_known_args()
HOME = Path.home() / "git/lv2_next_module"
B = HOME / "261102_1440_urdf_B/isaac"; URDF = B / "urdf/waffle_4wd_light.urdf"; USD = B / "usd/light/waffle_4wd_light/waffle_4wd_light.usda"
sys.path.insert(0, str(HOME / "Cartlidar/sim_render"))
from urdf_chain import fixed_transform  # noqa: E402

W, H, HFOV = 1920, 1080, 2 * math.atan(960 / 649.)  # 실측 렌즈 화면을 덮는 바늘구멍(warp_sim.py 가 실제 왜곡으로 바꿈)
fx = W / 2 / math.tan(HFOV / 2); K = np.array([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1.0]])
BOX, BOX_H = 0.205, 0.07
ROBOT_FRONT = 0.1328; Q0 = np.array([-0.98, 0.42, 1.54])
J1 = fixed_transform(str(URDF), "arm_base_link", root="base_footprint", q={})[:3, 3]
LAB = 0.016 * 10 / 8 / 2
rng = np.random.default_rng(args.seed)

def Rz(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
LIE = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])  # 몸체 z(긴 축) → 세계 -y, 몸체 y → 세계 +z (py 면이 위)

def project(p):
    u = K @ p; return u[:2] / u[2]
def make_puck(xy, yaw, shape, color, lying, tag_id, tag6):
    R = Rz(yaw) @ (LIE if lying else np.eye(3)); z = BOX_H + (0.015 if lying else 0.03)
    return {"xy": list(map(float, xy)), "yaw": float(yaw), "shape": shape, "color": color, "lying": bool(lying), "tag_id": int(tag_id), "tag6": bool(tag6),
            "R_w": R.tolist(), "c_w": [float(xy[0]), float(xy[1]), z]}
def rand_kind():
    shape = str(rng.choice(["cylinder", "cuboid"])); lying = shape == "cuboid" and rng.random() < 0.5
    # 서 있는 직육면체는 6면(발제 월드). 누운 직육면체는 절반이 위 넓은 면에 태그 없음
    tag6 = shape == "cuboid" and (not lying or rng.random() < 0.5)
    return shape, lying, tag6
scenes = []
while len(scenes) < args.n:
    q3 = Q0 + rng.uniform(-0.35, 0.35, 3); j1 = rng.uniform(-0.45, 0.45); gap = rng.uniform(0.04, 0.30)
    q = {"joint1": float(j1), "joint2": float(q3[0]), "joint3": float(q3[1]), "joint4": float(q3[2]), "joint5": float(rng.uniform(-0.3, 0.3))}
    T_bf_cam = fixed_transform(str(URDF), "arm_camera_optical_frame", root="base_footprint", q=q); T = np.linalg.inv(T_bf_cam)
    BC = np.array([ROBOT_FRONT + gap + BOX / 2, rng.uniform(-0.06, 0.06)]); byaw = rng.uniform(-0.2, 0.2)
    def on_box(xy, m=0.025):
        d = Rz(-byaw)[:2, :2] @ (np.asarray(xy) - BC); return np.abs(d).max() <= BOX / 2 - m
    tgt_xy = BC + Rz(byaw)[:2, :2] @ rng.uniform(-BOX / 2 + 0.025, BOX / 2 - 0.025, 2)
    pc = T[:3, :3] @ np.array([*tgt_xy, BOX_H + 0.03]) + T[:3, 3]
    if pc[2] < 0.07 or pc[2] > 0.50: continue
    u = project(pc)
    if not (60 < u[0] < W - 60 and 50 < u[1] < H - 50): continue
    ids = list(rng.permutation(12)); pucks = []; s, l, t6 = rand_kind()
    pucks.append(make_puck(tgt_xy, rng.uniform(-math.pi, math.pi), s, str(rng.choice(["red", "green"])), l, ids.pop(), t6))
    for _ in range(int(rng.integers(0, 4))):
        o = tgt_xy + Rz(byaw)[:2, :2] @ (rng.choice([-0.08, -0.06, 0.0, 0.06, 0.08], 2))
        s, l, t6 = rand_kind(); mn = 0.075 if l else 0.058
        if not on_box(o) or any(np.linalg.norm(o - np.array(p["xy"])) < (0.075 if p["lying"] else mn) for p in pucks): continue
        pucks.append(make_puck(o, rng.uniform(-math.pi, math.pi), s, str(rng.choice(["red", "green"])), l, ids.pop(), t6))
    scenes.append({"id": f"{len(scenes):05d}", "q": q, "box_c": BC.tolist(), "box_yaw": float(byaw), "pucks": pucks, "T_bf_cam": T_bf_cam.tolist(),
                   "dome": float(rng.uniform(250, 1100)), "sun": float(rng.uniform(0.5, 3.0)), "floor": int(rng.integers(0, 8)), "boxm": int(rng.integers(0, 4)),
                   "shade": [int(rng.integers(0, 3)) for _ in pucks]})

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})
import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade  # noqa: E402
from robot_usd import arm_link_locals, root_from_footprint, set_xform  # noqa: E402
(args.out / "images").mkdir(parents=True, exist_ok=False); (args.out / "floors").mkdir()
ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
def updates(n):
    for _ in range(n): app.update()
def material(name, rgb):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}"); s = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PBR"); s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb)); s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface"); return m
def tex_material(name, png):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}"); s = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PBR"); s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    rd = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/st_reader"); rd.CreateIdAttr("UsdPrimvarReader_float2"); rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    tx = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/texture"); tx.CreateIdAttr("UsdUVTexture"); tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(png)))
    for w in ("wrapS", "wrapT"): tx.CreateInput(w, Sdf.ValueTypeNames.Token).Set("repeat")
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result"); tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface"); return m
def quad_mesh(path, c, u, v, h, rep_st, mat):
    m = UsdGeom.Mesh.Define(stage, path); c, u, v = map(np.array, (c, u, v))
    P = [c - u * h - v * h, c + u * h - v * h, c + u * h + v * h, c - u * h + v * h]
    m.CreatePointsAttr([Gf.Vec3f(*p.tolist()) for p in P]); m.CreateFaceVertexCountsAttr([4]); m.CreateFaceVertexIndicesAttr([0, 1, 2, 3]); m.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(rep_st, 0), Gf.Vec2f(rep_st, rep_st), Gf.Vec2f(0, rep_st)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat); return m
# 바닥: 민무늬 4 + 카펫 비슷한 점무늬 4 (실물 바닥과 같게 만든 것은 아님)
floors = []
frng = np.random.default_rng(7)
for i, g in enumerate((0.30, 0.40, 0.50, 0.60)):
    floors.append(material(f"floor{i}", (g, g, g)))
for i, (g, a) in enumerate(((0.30, 0.10), (0.40, 0.14), (0.50, 0.12), (0.35, 0.18))):
    n = frng.normal(0, 1, (512, 512)).astype(np.float32); n = cv2.GaussianBlur(n, (0, 0), 1.2); n = n / n.std()
    img = np.clip((g + a * n) * 255, 0, 255).astype(np.uint8); p = args.out / f"floors/f{i}.png"; cv2.imwrite(str(p), img); floors.append(tex_material(f"floort{i}", p))
floor = quad_mesh("/World/floor", (0.5, 0, 0), (1, 0, 0), (0, 1, 0), 2.0, 24, floors[0])
boxms = [material(f"box{i}", c) for i, c in enumerate(((0.8, 0.68, 0.48), (0.72, 0.6, 0.42), (0.85, 0.78, 0.62), (0.62, 0.55, 0.45)))]
COL = {"red": [material(f"red{i}", c) for i, c in enumerate(((0.85, 0.1, 0.1), (0.75, 0.07, 0.08), (0.9, 0.16, 0.12)))],
       "green": [material(f"green{i}", c) for i, c in enumerate(((0.1, 0.65, 0.25), (0.08, 0.55, 0.2), (0.16, 0.7, 0.3)))]}
TEX = HOME / "Cartlidar/sim_world/textures"
tag_mats = {i: tex_material(f"tag_{i:02d}", TEX / f"tag36h11_{i:02d}.png") for i in range(12)}
for m_ in tag_mats.values(): pass
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome"); sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun"); UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25, 15, 0))
robot = stage.DefinePrim("/World/waffle_4wd", "Xform"); robot.GetReferences().AddReference(str(USD)); updates(30)
vs = robot.GetVariantSets()
if vs.HasVariantSet("Physics"): vs.GetVariantSet("Physics").SetVariantSelection("none")
updates(10); set_xform(robot, root_from_footprint(str(URDF)))
cam = UsdGeom.Camera.Define(stage, "/World/armcam"); ha = 20.955; focal = fx * ha / W
cam.CreateHorizontalApertureAttr(ha); cam.CreateVerticalApertureAttr(focal * H / fx); cam.CreateFocalLengthAttr(focal); cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))
def set_cam(T):
    Rc, e = T[:3, :3], T[:3, 3]; m = Gf.Matrix4d(*Rc[:, 0].tolist(), 0, *(-Rc[:, 1]).tolist(), 0, *(-Rc[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(m)
def xfm(prim, R, t):
    xf = UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(*R[:, 0].tolist(), 0, *R[:, 1].tolist(), 0, *R[:, 2].tolist(), 0, *list(map(float, t)), 1))
FACES = {"top": ((0, 0, 0.03 + 2e-4), (1, 0, 0), (0, 1, 0)), "bottom": ((0, 0, -0.03 - 2e-4), (1, 0, 0), (0, -1, 0)),
         "px": ((0.015 + 2e-4, 0, 0), (0, 1, 0), (0, 0, 1)), "nx": ((-0.015 - 2e-4, 0, 0), (0, -1, 0), (0, 0, 1)),
         "py": ((0, 0.015 + 2e-4, 0), (-1, 0, 0), (0, 0, 1)), "ny": ((0, -0.015 - 2e-4, 0), (1, 0, 0), (0, 0, 1))}
def build_puck(path, pk, shade):
    x = UsdGeom.Xform.Define(stage, path); xfm(x, np.array(pk["R_w"]), pk["c_w"]); mat = COL[pk["color"]][shade]
    if pk["shape"] == "cuboid":
        g = UsdGeom.Cube.Define(stage, path + "/body"); g.CreateSizeAttr(1.0); UsdGeom.Xformable(g).AddScaleOp().Set(Gf.Vec3f(0.03, 0.03, 0.06))
    else:
        g = UsdGeom.Cylinder.Define(stage, path + "/body"); g.CreateRadiusAttr(0.015); g.CreateHeightAttr(0.06); g.CreateAxisAttr("Z")
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    for fn in (("top", "bottom", "px", "nx", "py", "ny") if pk["tag6"] else ("top", "bottom")):
        c, u, v = FACES[fn]; quad_mesh(f"{path}/label_{fn}", c, u, v, LAB, 1.0, tag_mats[pk["tag_id"]])
def surface_points(pk):
    """몸체 좌표 겉면 표본 점 · 바깥 법선"""
    P, N = [], []
    g = np.linspace(-1, 1, 9)
    if pk["shape"] == "cuboid":
        for a, b in [(x, y) for x in g for y in g]:
            for s in (-1, 1):
                P.append([a * .015, b * .015, s * .03]); N.append([0, 0, s]); P.append([s * .015, a * .015, b * .03]); N.append([s, 0, 0]); P.append([a * .015, s * .015, b * .03]); N.append([0, s, 0])
    else:
        for a, b in [(x, y) for x in g for y in g]:
            if a * a + b * b <= 1:
                for s in (-1, 1): P.append([a * .015, b * .015, s * .03]); N.append([0, 0, s])
        for th in np.linspace(0, 2 * math.pi, 36, endpoint=False):
            for b in g: P.append([.015 * math.cos(th), .015 * math.sin(th), b * .03]); N.append([math.cos(th), math.sin(th), 0])
    return np.array(P), np.array(N, float)
def visible_of(P_c, N_c, dep, tol=0.004):
    """법선이 카메라를 보는 점 중 화면 안에서 깊이가 맞는 비율"""
    view = -P_c / np.linalg.norm(P_c, axis=1, keepdims=True); fac = (N_c * view).sum(1) > 0.05
    if not fac.any(): return 0.0, 0
    Q = P_c[fac]; uv = Q @ K.T; uv = uv[:, :2] / uv[:, 2, None]; ok = 0
    for (u, v), z in zip(uv, Q[:, 2]):
        i, j = int(round(u)), int(round(v))
        if 0 <= i < W and 0 <= j < H and abs(float(dep[j, i]) - z) <= tol: ok += 1
    return ok / len(Q), int(fac.sum())
rp = rep.create.render_product("/World/armcam", (W, H)); a_rgb = rep.AnnotatorRegistry.get_annotator("rgb"); a_rgb.attach(rp)
a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); a_dep.attach(rp)
box_prim = UsdGeom.Cube.Define(stage, "/World/box"); box_prim.CreateSizeAttr(1.0)
truth = {"K": K.tolist(), "width": W, "height": H, "urdf": str(URDF), "seed": args.seed, "synthetic_only": True, "images": []}
t0 = time.time()
for sc in scenes:
    for prim, T in arm_link_locals(stage, str(URDF), sc["q"]).values(): set_xform(prim, T)
    T_bf_cam = np.array(sc["T_bf_cam"]); set_cam(T_bf_cam); Tc = np.linalg.inv(T_bf_cam)
    xf = UsdGeom.Xformable(box_prim); xf.ClearXformOpOrder(); xf.AddTranslateOp().Set(Gf.Vec3d(sc["box_c"][0], sc["box_c"][1], BOX_H / 2)); xf.AddRotateZOp().Set(math.degrees(sc["box_yaw"])); xf.AddScaleOp().Set(Gf.Vec3f(BOX, BOX, BOX_H))
    UsdShade.MaterialBindingAPI.Apply(box_prim.GetPrim()).Bind(boxms[sc["boxm"]]); UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(floors[sc["floor"]])
    if stage.GetPrimAtPath("/World/pucks").IsValid(): stage.RemovePrim("/World/pucks")
    for i, pk in enumerate(sc["pucks"]): build_puck(f"/World/pucks/p{i}", pk, sc["shade"][i])
    dome.CreateIntensityAttr(sc["dome"]); sun.CreateIntensityAttr(sc["sun"]); updates(3)
    for _ in range(2): rep.orchestrator.step(rt_subframes=8)
    img = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8); dep = np.asarray(a_dep.get_data())
    fn = f"images/{sc['id']}.png"; cv2.imwrite(str(args.out / fn), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    rows = []
    for pk in sc["pucks"]:
        Rw = np.array(pk["R_w"]); cw = np.array(pk["c_w"]); Rc = Tc[:3, :3] @ Rw; c = Tc[:3, :3] @ cw + Tc[:3, 3]
        P, N = surface_points(pk); vis, nf = visible_of(P @ Rc.T + c, N @ Rc.T, dep)
        tags = {}
        for fn_ in (("top", "bottom", "px", "nx", "py", "ny") if pk["tag6"] else ("top", "bottom")):
            cc, u, v = map(np.array, FACES[fn_]); g = np.linspace(-LAB, LAB, 9)
            Q = np.array([cc + a * u + b * v for a in g for b in g]); Nn = np.tile(np.cross(u, v).astype(float), (len(Q), 1))
            tv, _ = visible_of(Q @ Rc.T + c, Nn @ Rc.T, dep); tags[fn_] = round(tv, 3)
        corners = np.array([[x, y, z] for x in (-.015, .015) for y in (-.015, .015) for z in (-.03, .03)])
        uv = (corners @ Rc.T + c) @ K.T; uv = uv[:, :2] / uv[:, 2, None]
        rows.append({"tag_id": pk["tag_id"], "shape": pk["shape"], "color": pk["color"], "lying": pk["lying"], "tag6": pk["tag6"], "center_cam": c.tolist(), "R_cam": Rc.tolist(),
                     "visible": round(vis, 3), "tags_visible": tags, "tag_visible_max": max(tags.values()), "corners_px": uv.tolist(), "dist_m": float(np.linalg.norm(c))})
    truth["images"].append({"id": sc["id"], "image": fn, "T_bf_cam": sc["T_bf_cam"], "q": sc["q"], "dome": sc["dome"], "sun": sc["sun"], "floor": sc["floor"], "pucks": rows})
    if len(truth["images"]) % 100 == 0:
        (args.out / "truth.json").write_text(json.dumps(truth)); print(len(truth["images"]), round(time.time() - t0), flush=True)
truth["render_s"] = time.time() - t0
(args.out / "truth.json").write_text(json.dumps(truth)); print("끝", len(truth["images"]), round(time.time() - t0), flush=True)
app.close()
