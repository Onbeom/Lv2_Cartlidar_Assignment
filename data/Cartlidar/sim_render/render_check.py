"""렌더 도구 확인 (R 시험 준비, 판정 시험 아님) — Isaac Sim 6.1 헤드리스로 월드를 열어 카메라 영상 · 깊이를 저장한다.

확인 항목(가설은 결과 폴더 README 에 실행 전에 적는다):
  V1 신호등 앞 0.5 m 수평 시점(D435 내부값): 꺼짐 · 빨강 · 노랑 · 초록 4장. 칸 가운데 화소 HSV, 가운데 깊이
  V2 선반 위 선 퍽 윗면 0.10 m 위에서 내려다봄(팔 카메라 640 x 480, f 283 가정): tag36h11 번호 검출, 가운데 깊이
  V3 경기장 전체(보기용), V4 선반 앞 D435 높이 시점(보기용)
결과: <out>/<view>.png, <view>_depth.npy, render_check.json

실행 (PC): ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_check.py \
    --world Cartlidar/sim_world/worlds/b_dice/seed_1 --out <결과 폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--world", type=Path, required=True, help="task_world.usda · layout.json 이 있는 폴더")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--emissive", type=float, default=8.0, help="켜진 칸 emissiveColor 배율(on_color × 이 값) — 가정")
ap.add_argument("--subframes", type=int, default=8, help="장마다 RT 누적 횟수")
args, _ = ap.parse_known_args()   # 나머지(--/crashreporter/url= 등)는 Kit 설정으로

W, H = 640, 480
D435 = {"fx": 605.850769, "fy": 605.67987, "cx": 324.365814, "cy": 245.139175}   # 실물 261822072247 컬러 640x480
ARM_F = 283.0   # 가정: Arducam B0201 640 x 480(16:9 센서 가운데 4:3을 줄인 것이라면)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdShade  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
lay = json.loads((args.world / "layout.json").read_text())
ctx = omni.usd.get_context()
t0 = time.time()
ok = ctx.open_stage(str((args.world / "task_world.usda").resolve()))
for _ in range(60):
    app.update()
stage = ctx.get_stage()
report = {"world": str(args.world), "open_ok": bool(ok), "open_s": round(time.time() - t0, 1), "views": {},
          "emissive_scale": args.emissive, "d435": D435, "arm_f": ARM_F}


def look_at(eye, target, up_hint):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye
    f /= np.linalg.norm(f)
    right = np.cross(f, up_hint)
    right /= np.linalg.norm(right)
    down = np.cross(f, right)
    return right, down, f


def make_camera(name, eye, target, fx, cx, cy, up_hint=(0, 0, 1)):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 기준 핀홀 카메라. USD 카메라는 -Z 를 보고 +Y 가 위."""
    right, down, f = look_at(eye, target, np.asarray(up_hint, float))
    path = f"/World/RenderCheck/{name}"
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(ha * H / W)
    cam.CreateFocalLengthAttr(fx * ha / W)
    # 주점: 화면 가운데에서 어긋난 만큼 (USD 오프셋은 조리개 단위, +x 오른쪽 · +y 위)
    # 주점 부호(2026-10-24 고침, render_r12.py 와 같음): 가로 −, 세로 +
    cam.CreateHorizontalApertureOffsetAttr(-(cx - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((cy - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    m = Gf.Matrix4d(*right.tolist(), 0, *(-down).tolist(), 0, *(-f).tolist(), 0, *np.asarray(eye, float).tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    Rwc = np.stack([right, down, f], 1)   # 광학 → 월드
    return path, {"eye": list(map(float, eye)), "R_world_optical": Rwc.tolist(), "fx": fx, "fy": fx, "cx": cx, "cy": cy}


def project(pose, p):
    R, e = np.array(pose["R_world_optical"]), np.array(pose["eye"])
    q = R.T @ (np.asarray(p, float) - e)
    return pose["fx"] * q[0] / q[2] + pose["cx"], pose["fy"] * q[1] / q[2] + pose["cy"], q[2]


products = {}


def render(cam_path, name):
    if cam_path not in products:
        rp = rep.create.render_product(cam_path, (W, H))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach(rp)
        a_dep.attach(rp)
        products[cam_path] = (rp, a_rgb, a_dep)
    rp, a_rgb, a_dep = products[cam_path]
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
    dep = np.asarray(a_dep.get_data()).astype(np.float32)
    cv2.imwrite(str(out / f"{name}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    np.save(out / f"{name}_depth.npy", dep)
    return rgb, dep


def patch_hsv(rgb, u, v, r=2):
    u, v = int(round(u)), int(round(v))
    hsv = cv2.cvtColor(rgb[v - r:v + r + 1, u - r:u + r + 1], cv2.COLOR_RGB2HSV).reshape(-1, 3)
    return [int(np.median(hsv[:, k])) for k in range(3)]


def set_signal(on):
    for name in ("green", "yellow", "red"):
        sh = UsdShade.Shader(stage.GetPrimAtPath(f"/World/Looks/signal_{name}/PBR"))
        col = stage.GetPrimAtPath(f"/World/SignalLight/{name}").GetAttribute("signal:on_color").Get()
        val = Gf.Vec3f(*[float(c) * args.emissive for c in col]) if name == on else Gf.Vec3f(0, 0, 0)
        sh.GetInput("emissiveColor").Set(val)
    for _ in range(4):
        app.update()


# ---------- V1 신호등 ----------
sl = lay["signal_light"]
A = lay["assumptions"]["signal_light"]
seg_z = {n: A["first_led_z"] + (i + 0.5) * A["segment_length"] for i, n in enumerate(("green", "yellow", "red"))}
cell = next(c for c in lay["cells"] if c["id"] == sl["cell"])
d = np.array(cell["center"]) - np.array([sl["x"], sl["y"]])
d /= np.linalg.norm(d)
eye = [sl["x"] + 0.5 * d[0], sl["y"] + 0.5 * d[1], seg_z["yellow"]]
target = [sl["x"], sl["y"], seg_z["yellow"]]
cam, pose = make_camera("v1_signal", eye, target, D435["fx"], D435["cx"], D435["cy"])
v1 = {"pose": pose, "expected_center_depth_m": round(0.5 - A["head_diameter"] / 2, 4), "states": {}}
for state in ("off", "red", "yellow", "green"):
    set_signal(state if state != "off" else None)
    rgb, dep = render(cam, f"v1_signal_{state}")
    row = {}
    for n in ("green", "yellow", "red"):
        u, v, _ = project(pose, [sl["x"] + A["head_diameter"] / 2 * d[0], sl["y"] + A["head_diameter"] / 2 * d[1], seg_z[n]])
        row[n] = {"px": [round(u, 1), round(v, 1)], "hsv_median": patch_hsv(rgb, u, v)}
    u, v, _ = project(pose, target)
    row["center_depth_m"] = round(float(dep[int(round(v)), int(round(u))]), 4)
    v1["states"][state] = row
report["views"]["V1"] = v1
set_signal(None)

# ---------- V2 팔 카메라로 선 퍽 윗면 ----------
standing = [p for p in lay["pucks"] if p["location"] != "floor" and abs(p["R"][2][2]) > 0.99]
v2 = {"candidates": len(standing)}
if standing:
    pk = standing[0]
    top = np.array(pk["center"]) + np.array([0, 0, 0.03 + 0.0002])
    cam, pose = make_camera("v2_arm_top", top + [0, 0, 0.10], top, ARM_F, W / 2, H / 2, up_hint=(0, 1, 0))
    rgb, dep = render(cam, "v2_arm_top")
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                  cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    found = [] if ids is None else [int(i) for i in ids.ravel()]
    side_px = None
    if ids is not None and pk["tag_id"] in found:
        c = corners[found.index(pk["tag_id"])][0]
        side_px = round(float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)])), 1)
    v2.update({"puck": pk["name"], "tag_id": pk["tag_id"], "color": pk["color"], "shape": pk["shape"], "pose": pose,
               "found_ids": found, "tag_side_px": side_px, "expected_side_px": round(ARM_F * 0.016 / 0.10, 1),
               "center_depth_m": round(float(dep[H // 2, W // 2]), 4), "expected_center_depth_m": 0.10})
report["views"]["V2"] = v2

# ---------- V3 · V4 보기용 ----------
xs = [c["center"][0] for c in lay["cells"]]
cam, pose = make_camera("v3_overview", [np.mean(xs), -1.2, 3.0], [np.mean(xs), 1.6, 0.0], 380.0, W / 2, H / 2)
render(cam, "v3_overview")
report["views"]["V3"] = {"pose": pose}
sh0 = lay["shelves"][0]
sc = np.array(sh0["center"])
cell = next(c for c in lay["cells"] if c["id"] == sh0["cell"])
d = np.array(cell["center"]) - sc[:2]
d /= np.linalg.norm(d)
eye = [sc[0] + 0.7 * d[0], sc[1] + 0.7 * d[1], 0.49]
cam, pose = make_camera("v4_shelf_d435", eye, [sc[0], sc[1], 0.10], D435["fx"], D435["cx"], D435["cy"])
render(cam, "v4_shelf_d435")
report["views"]["V4"] = {"pose": pose, "shelf": sh0["name"]}

report["total_s"] = round(time.time() - t0, 1)
(out / "render_check.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({k: v for k, v in report.items() if k != "views"}, ensure_ascii=False))
print("V1", json.dumps(report["views"]["V1"]["states"], ensure_ascii=False))
print("V2", json.dumps({k: v for k, v in report["views"]["V2"].items() if k != "pose"}, ensure_ascii=False))
app.close()
