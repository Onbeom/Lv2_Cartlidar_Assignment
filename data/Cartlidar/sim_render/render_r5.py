"""R5 장소 태그 렌더 (인지 판정 기준 v1 R5): 48 mm tag36h11 ID 12~14, D435 거리 · 기울기 · 조명.

카메라는 로봇 장착 자세가 아닌 자유 카메라다(태그 검출 · 위치 정확도만 본다). 높이는 앞 D435 장착 높이(URDF, 약 0.49 m),
광학축은 태그 가운데를 향하고 roll 0. 거리 = 카메라 → 태그 가운데 3D 거리, 기울기 = 태그 면 법선에서 수평으로 돌린 각.
시선이 벽 · 기둥 · 선반(0.08 m)에 막히거나 카메라가 벽에 너무 가까우면 반대쪽(−각)을 쓰고, 둘 다 안 되면 '불가'로 기록한다.
로봇은 숨긴다. replay_r12.py marker 로 재생한다(재생기는 base_footprint = 카메라 자세 × URDF 장착 역변환으로 TF 를 낸다).

결과: <out>/<name>/frame.png · frame_depth.npy, <out>/render_r5.json
실행 (PC):
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r5.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_{1,2,3} \
      --urdf 261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf --out <결과 폴더> --/crashreporter/url=
"""
import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from r12_poses import K, W, H, Scene  # noqa: E402
from urdf_chain import fixed_transform  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--urdf", type=Path, required=True)
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--dists", type=float, nargs="+", default=[0.5, 1.0, 1.5])
ap.add_argument("--tilts", type=float, nargs="+", default=[0.0, 30.0, 60.0])
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0])
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
args, _ = ap.parse_known_args()

LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}
drawing = json.loads(args.drawing.read_text())
pkg_dir = args.urdf.parent.parent
T_bf_cam = fixed_transform(str(args.urdf), args.frame)
CAM_Z = float(T_bf_cam[2, 3])


def look_at(cam, target):
    """광학 좌표(z 앞 · x 오른쪽 · y 아래) 자세, roll 0."""
    z = target - cam
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, cam
    return T


plan = []
for wdir in args.worlds:
    lay = json.loads((wdir / "layout.json").read_text())
    # 선반은 실제 높이(0.08 m)로만 막음을 계산한다. 장소 태그가 선반 바로 위(0.13 m~)라 퍽까지 넣으면 늘 막힘.
    # 선반 위 퍽이 태그를 실제로 가리는지는 렌더 깊이로 따로 잰다(tag_visible)
    sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, shelf_top=lay["assumptions"]["shelf_size"][2])
    for tag, d, tilt in itertools.product(lay["place_tags"], args.dists, args.tilts):
        c = np.array(tag["center"])
        nrm = np.array(tag["normal"], float)
        dz = CAM_Z - c[2]
        h = math.sqrt(max(d * d - dz * dz, 0.0))
        chosen, why = None, []
        for sgn in ((1, -1) if tilt else (1,)):
            a = math.radians(sgn * tilt)
            dirh = np.array([nrm[0] * math.cos(a) - nrm[1] * math.sin(a), nrm[0] * math.sin(a) + nrm[1] * math.cos(a)])
            cam = np.array([c[0] + h * dirh[0], c[1] + h * dirh[1], CAM_Z])
            x0, x1, y0, y1 = sc.bounds
            if not (x0 < cam[0] < x1 and y0 < cam[1] < y1):
                why.append(f"{sgn:+d}: 미로 밖")
                continue
            if sc.clearance([cam[:2]]) < 0.05:
                why.append(f"{sgn:+d}: 벽에 가까움")
                continue
            if sc.los_blocked(cam, c, end_skip=0.02):
                why.append(f"{sgn:+d}: 시선 막힘")
                continue
            chosen = (sgn, cam)
            break
        entry = {"id": f"w{lay['seed']}_t{tag['id']}_d{d:.1f}_a{tilt:.0f}", "world": str(wdir), "seed": int(lay["seed"]),
                 "tag": tag, "dist_nominal": d, "tilt_deg": tilt}
        if chosen is None:
            entry["infeasible"] = why
        else:
            T_w_cam = look_at(chosen[1], c)
            entry.update({"tilt_sign": chosen[0], "T_world_cam": T_w_cam.tolist(),
                          "T_world_base_footprint": (T_w_cam @ np.linalg.inv(T_bf_cam)).tolist()})
        plan.append(entry)
print(f"장면 {len(plan)}개, 불가 {sum('infeasible' in p for p in plan)}개", flush=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def set_camera(stage, path, T_w_opt):
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = K["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / K["fy"])
    cam.CreateFocalLengthAttr(focal)
    # 주점 부호: 가로 −, 세로 + (render_r12.py 와 같음)
    cam.CreateHorizontalApertureOffsetAttr(-(K["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((K["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    R, e = T_w_opt[:3, :3], T_w_opt[:3, 3]
    m = Gf.Matrix4d(*R[:, 0].tolist(), 0, *(-R[:, 1]).tolist(), 0, *(-R[:, 2]).tolist(), 0, *e.tolist(), 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def set_lighting(stage, level):
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(float(rect))


report = {"urdf": str(args.urdf), "frame": args.frame, "width": W, "height": H, "K": K, "lights": args.lights,
          "infeasible": [p for p in plan if "infeasible" in p], "poses": []}
t0 = time.time()
for wdir in args.worlds:
    todo = [p for p in plan if p["world"] == str(wdir) and "infeasible" not in p]
    if not todo:
        continue
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    UsdGeom.Imageable(stage.GetPrimAtPath("/World/waffle_4wd")).MakeInvisible()
    cam_path = "/World/RenderR5/d435"
    set_camera(stage, cam_path, np.array(todo[0]["T_world_cam"]))
    rp = rep.create.render_product(cam_path, (W, H))
    a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    a_rgb.attach(rp)
    a_dep.attach(rp)
    for level in args.lights:
        set_lighting(stage, level)
        for pose in todo:
            set_camera(stage, cam_path, np.array(pose["T_world_cam"]))
            updates(4)
            for _ in range(2):
                rep.orchestrator.step(rt_subframes=args.subframes)
            rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
            dep = np.asarray(a_dep.get_data()).astype(np.float32)
            name = f"{pose['id']}_L{level:g}"
            (out / name).mkdir()
            cv2.imwrite(str(out / name / "frame.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            np.save(out / name / "frame_depth.npy", dep)
            # 태그 면 9 × 9 점이 깊이로 보이는지(렌더 깊이가 기대 깊이보다 5 mm 넘게 가깝지 않음) → 보이는 비율
            Tc = np.array(pose["T_world_cam"])
            Ti = np.linalg.inv(Tc)
            tg = pose["tag"]
            nrm = np.array(tg["normal"], float)
            side = np.cross(nrm, [0.0, 0.0, 1.0])
            vis = tot = 0
            for a in np.linspace(-0.45, 0.45, 9):
                for b in np.linspace(-0.45, 0.45, 9):
                    pw = np.array(tg["center"]) + side * a * tg["size"] + np.array([0, 0, 1.0]) * b * tg["size"]
                    q = Ti @ np.append(pw, 1.0)
                    u, v = K["fx"] * q[0] / q[2] + K["cx"], K["fy"] * q[1] / q[2] + K["cy"]
                    tot += 1
                    if 0 <= u < W and 0 <= v < H and q[2] > 0:
                        vis += bool(dep[int(v), int(u)] > q[2] - 0.005)
            e = {k: v for k, v in pose.items()}
            e["tag_visible"] = round(vis / tot, 3)
            e.update({"name": name, "light_level": level,
                      "images": {"frame": {"color": f"{name}/frame.png", "depth": f"{name}/frame_depth.npy"}}})
            report["poses"].append(e)
        print(f"{wdir.name} 조명 {level:g}: {len(todo)}장", flush=True)
    a_rgb.detach()
    a_dep.detach()
    rp.destroy()
    updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_r5.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"poses": len(report["poses"]), "infeasible": len(report["infeasible"]), "total_s": report["total_s"]}))
app.close()
