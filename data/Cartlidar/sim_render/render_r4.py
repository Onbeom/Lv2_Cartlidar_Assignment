"""R4 퍽 렌더 (인지 판정 기준 v1 R4): 앞 D435 장면과 팔 카메라 내려다보기 장면. 퍽은 월드(layout.json)에 있는 그대로 쓴다.

로봇 모델: 기준 모델(robot_usd.py, 2026-10-27 저녁부터 261027_1711). 월드의 로봇 USD 참조를 렌더할 때 바꾼다.
d435 장면: 로봇 장착 앞 D435, 로봇 몸체 · 접은 팔 S0 포함. 월드마다 --d435-scenes 개, 선반을 돌아가며 고른다.
  base_footprint → 선반 가운데 수평 거리 U(0.5, 1.5) m, 방향 무작위, 선반이 정면에서 ±15° 안, 카메라 흔들림 ±1°.
  조건: 차체 바닥 윤곽이 미로 안 · 벽 · 기둥 · 선반에서 0.02 m 넘게, 바닥 퍽을 밟지 않음, 선반 가운데가 화면 안(여백 30 px),
  카메라 → 선반 위 시선이 벽 · 기둥 · 다른 선반에 막히지 않음. 안 되면 다시 뽑는다.
arm 장면: 선반 위 퍽마다 하나. 계획 카메라 = 퍽 윗면 0.10 m 위, 가로 ±0.02 m · 기울기 ≤ 5° · 방향 무작위로 내려다봄.
  실제 팔 자세(arm_pose.py, 2026-10-27 저녁 사용자 결정): 차체 자리 + 팔 관절을 풀어 카메라 위치 · 광축을 맞추고(광축 둘레 회전은 팔이 정함),
  충돌(여유 5 mm) · 관절 한계로 못 가면 광축 방향으로 5 mm 씩 물림(최대 0.05 m, backoff_m 기록). 못 만든 장면은 json unreachable.
  보기 자세(2026-10-27 밤 설계 변경, R4 팔 재판정 37/38 실패 뒤): 카메라를 수평으로 옮겨 목표 윗면 가운데가 화면 세로 --aim-v(145 px)에
  오게 한다 — 내려다보면 손목 · 집게가 화면 아래 약 40 % 를 가려 손가락이 태그 테두리를 덮었음.
  로봇 · 집게(gripper 0) 보임, 가까운 클리핑 2.5 cm(렌즈 둘레 카메라 몸체 메시를 잘라냄).
  팔 카메라 640 × 480, f 283 px 가정(render_check.py 와 같음), 왜곡 0. 재생기는 장면의 팔 관절(arm_q) /joint_states 와
  실제 차체 자리 base_footprint TF 를 낸다.
보이는 정도(렌더 깊이): d435 = 퍽 겉면 점 중 카메라를 향한 점이 화면 안에서 깊이로 보이는 비율(visible),
  arm = 위를 향한 라벨의 태그(16 mm) 9 × 9 점이 보이는 비율(top_tag_visible).

결과: <out>/<name>/frame.png · frame_depth.npy(arm 은 깊이 없음), <out>/render_r4.json(replay_r4.py 입력)
실행 (PC):
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r4.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_{1,2,3} \
      --out <결과 폴더> --/crashreporter/url=
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from r12_poses import K, S0, W, H, Scene, _point_in_poly  # noqa: E402
from urdf_chain import fixed_transform, planar_pose, rpy_matrix  # noqa: E402
from arm_pose import ArmRealizer  # noqa: E402

TOP = HERE.parents[1]
MODEL = "261027_1711"   # robot_usd.py 와 같게 (그 파일은 Isaac Sim 실행 뒤에만 import)

ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--urdf", type=Path, default=TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf")
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--d435-scenes", type=int, default=20, help="월드마다 D435 장면 수")
ap.add_argument("--dist", type=float, nargs=2, default=[0.5, 1.5])
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0])
ap.add_argument("--arm-f", type=float, default=283.0)
ap.add_argument("--arm-height", type=float, default=0.10)
ap.add_argument("--seed", type=int, default=20261025)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--arm-frame", default="camera_optical_frame")
ap.add_argument("--near", type=float, default=0.025, help="[m] 팔 카메라 가까운 클리핑")
ap.add_argument("--aim-v", type=float, default=145.0, help="[px] 보기 자세: 목표 윗면 가운데가 올 화면 세로 위치(0 = 옮기지 않음)")
args, _ = ap.parse_known_args()

LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
ROBOT = "/World/waffle_4wd"
CUBOID_HALF = (0.015, 0.015, 0.03)
CYL_R, CYL_HALF = 0.015, 0.03
TAG = 0.016
K_ARM = {"fx": args.arm_f, "fy": args.arm_f, "cx": W / 2, "cy": H / 2}
pkg_dir = args.urdf.parent.parent
drawing = json.loads(args.drawing.read_text())
T_bf_arm = fixed_transform(str(args.urdf), args.arm_frame, q=S0)
unreachable = []


def puck_surface(p):
    """퍽 겉면 점 · 바깥 법선 (월드). 직육면체 면마다 7 × 7, 원기둥 옆면 24 × 7 · 끝면 원 안 점."""
    R, c = np.array(p["R"]), np.array(p["center"])
    pts, nrm = [], []
    g = np.linspace(-0.85, 0.85, 7)
    if p["shape"] == "cuboid":
        h = np.array(CUBOID_HALF)
        for ax in range(3):
            o1, o2 = [k for k in range(3) if k != ax]
            for s in (-1, 1):
                for a in g:
                    for b in g:
                        q = np.zeros(3)
                        q[ax], q[o1], q[o2] = s * h[ax], a * h[o1], b * h[o2]
                        n = np.zeros(3)
                        n[ax] = s
                        pts.append(q)
                        nrm.append(n)
    else:
        for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            for zz in g:
                pts.append([CYL_R * math.cos(th), CYL_R * math.sin(th), zz * CYL_HALF])
                nrm.append([math.cos(th), math.sin(th), 0.0])
        for s in (-1, 1):
            for a in g:
                for b in g:
                    if a * a + b * b <= 0.85 ** 2:
                        pts.append([a * CYL_R, b * CYL_R, s * CYL_HALF])
                        nrm.append([0.0, 0.0, s])
    return np.asarray(pts) @ R.T + c, np.asarray(nrm, float) @ R.T


def up_label(p):
    """위를 향한 끝면(라벨) 가운데 · 면 안 두 축. 없으면(누움) None."""
    R, c = np.array(p["R"]), np.array(p["center"])
    z = R[:, 2]
    if abs(z[2]) < 0.7:
        return None
    s = 1.0 if z[2] > 0 else -1.0
    return c + s * z * CYL_HALF, R[:, 0], R[:, 1]


def visible_fraction(pts, nrm, T_w_cam, Kc, dep, tol=0.004):
    """카메라를 향한 점(법선 · 시선 cos > 0.15) 중 화면 안에서 렌더 깊이로 보이는 비율. (비율, 향한 점 수)"""
    Ti = np.linalg.inv(T_w_cam)
    cam = T_w_cam[:3, 3]
    view = cam - pts
    cosv = np.einsum("ij,ij->i", nrm, view) / np.linalg.norm(view, axis=1)
    front = cosv > 0.15
    if not front.any():
        return 0.0, 0
    q = (Ti[:3, :3] @ pts[front].T).T + Ti[:3, 3]
    vis = 0
    for x, y, z in q:
        if z <= 0:
            continue
        u, v = Kc["fx"] * x / z + Kc["cx"], Kc["fy"] * y / z + Kc["cy"]
        if 0 <= u < W and 0 <= v < H and np.isfinite(dep[int(v), int(u)]) and dep[int(v), int(u)] > z - tol:
            vis += 1
    return vis / int(front.sum()), int(front.sum())


def project(T_w_cam, Kc, p):
    q = np.linalg.inv(T_w_cam) @ np.append(np.asarray(p, float), 1.0)
    return Kc["fx"] * q[0] / q[2] + Kc["cx"], Kc["fy"] * q[1] / q[2] + Kc["cy"], q[2]


# ---------- 장면 뽑기 (렌더 전, 같은 seed 면 같은 결과) ----------
rng = np.random.default_rng(args.seed)
plan = []
for wdir in args.worlds:
    lay = json.loads((wdir / "layout.json").read_text())
    sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, shelf_top=lay["assumptions"]["shelf_size"][2])
    ar = ArmRealizer(args.urdf, pkg_dir, sc, lay, gripper=0.0)
    floor = [np.array(p["center"][:2]) for p in lay["pucks"] if p["location"] == "floor"]
    shelves = lay["shelves"]
    tries = 0
    for k in range(args.d435_scenes):
        sh = shelves[k % len(shelves)]
        c = np.array(sh["center"][:2])
        target = np.array([c[0], c[1], sh["top_z"] + 0.03])
        for _ in range(20000):
            tries += 1
            d = rng.uniform(*args.dist)
            phi = rng.uniform(0, 2 * math.pi)
            bearing = math.radians(rng.uniform(-15, 15))
            pos = c + d * np.array([math.cos(phi), math.sin(phi)])
            T_w_bf = planar_pose(pos[0], pos[1], phi + math.pi - bearing)
            fp = sc.footprint_world(T_w_bf)
            x0, x1, y0, y1 = sc.bounds
            if fp[:, 0].min() < x0 or fp[:, 0].max() > x1 or fp[:, 1].min() < y0 or fp[:, 1].max() > y1:
                continue
            ring = np.vstack([fp[i] + (fp[(i + 1) % len(fp)] - fp[i]) * s for i in range(len(fp))
                              for s in np.linspace(0, 1, 15, endpoint=False)])
            if sc.clearance(ring) < 0.02:
                continue
            if any(_point_in_poly(f, fp) or np.min(np.linalg.norm(ring - f, axis=1)) < 0.03 for f in floor):
                continue
            J = np.eye(4)
            J[:3, :3] = rpy_matrix(*[math.radians(rng.uniform(-1, 1)) for _ in range(3)])
            T_w_cam = T_w_bf @ sc.T_bf_cam @ J
            u, v, zc = project(T_w_cam, K, target)
            if not (zc > 0 and 30 <= u < W - 30 and 30 <= v < H - 30):
                continue
            if sc.los_blocked(T_w_cam[:3, 3], target, end_skip=0.05):
                continue
            plan.append({"kind": "d435", "id": f"w{lay['seed']}_d{k:02d}", "world": str(wdir), "seed": int(lay["seed"]),
                         "shelf": sh["name"], "dist_nominal": round(float(d), 4), "bearing_deg": math.degrees(bearing),
                         "T_world_base_footprint": T_w_bf.tolist(), "T_world_cam": T_w_cam.tolist(),
                         "min_clearance_m": round(sc.clearance(ring), 4)})
            break
        else:
            raise RuntimeError(f"{wdir.name} {sh['name']} D435 장면을 못 뽑음")
    for p in lay["pucks"]:
        if p["location"] == "floor":
            continue
        R = np.array(p["R"])
        top = p["center"][2] + (CYL_HALF if abs(R[2, 2]) > 0.7 else CUBOID_HALF[0])
        cam = np.array([p["center"][0] + rng.uniform(-0.02, 0.02), p["center"][1] + rng.uniform(-0.02, 0.02),
                        top + args.arm_height])
        tilt, az, yaw = math.radians(rng.uniform(0, 5)), rng.uniform(0, 2 * math.pi), rng.uniform(0, 2 * math.pi)
        z = np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt)])
        x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        x -= z * (x @ z)
        x /= np.linalg.norm(x)
        T_w_cam = np.eye(4)
        T_w_cam[:3, 0], T_w_cam[:3, 1], T_w_cam[:3, 2], T_w_cam[:3, 3] = x, np.cross(z, x), z, cam
        sid = f"w{lay['seed']}_a_{p['name']}"
        r = ar.realize_aim(T_w_cam, floor, [p["center"][0], p["center"][1], top], v_px=args.aim_v) if args.aim_v else \
            ar.realize([T_w_cam], floor)
        if r is None:
            unreachable.append({"id": sid, "target": p["name"], "T_world_cam_nominal": T_w_cam.tolist()})
            print(sid, "실제 팔 자세 없음", flush=True)
            continue
        f = r["frames"][0]
        plan.append({"kind": "arm", "id": sid, "world": str(wdir), "seed": int(lay["seed"]),
                     "target": p["name"], "tilt_deg": math.degrees(tilt), "T_world_cam": f["T_world_cam"],
                     "T_world_cam_nominal": T_w_cam.tolist(), "backoff_m": f["backoff_m"], "aim_v_px": r.get("aim_v_px"),
                     "arm_q": {**f["q"], "gripper_left_joint": 0.0}, "T_world_base_footprint": r["T_world_base_footprint"]})
print(f"장면 {len(plan)}개 (D435 {sum(p['kind'] == 'd435' for p in plan)}, 팔 {sum(p['kind'] == 'arm' for p in plan)},"
      f" 팔 자세 없음 {len(unreachable)}), 뽑기 시도 {tries}", flush=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

from robot_usd import pose_robot, use_robot_usd  # noqa: E402

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


def set_camera(stage, path, T_w_opt, Kc, near=0.01):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세로 USD 카메라(−Z 를 보고 +Y 가 위)를 둔다. 주점 부호: 가로 −, 세로 +."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = Kc["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, 50.0))
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


def set_lighting(stage, level):
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(float(rect))


def grab(a_rgb, a_dep):
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    return np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8), np.asarray(a_dep.get_data()).astype(np.float32)


def puck_truth(p):
    return {k: p[k] for k in ("name", "tag_id", "color", "shape", "standing", "location", "center", "R")}


report = {"urdf": str(args.urdf), "robot_usd": None, "width": W, "height": H, "lights": args.lights, "seed": args.seed,
          "arm_q": S0, "unreachable": unreachable,
          "d435": {"frame": args.frame, "K": K}, "arm": {"frame": args.arm_frame, "K": K_ARM, "height": args.arm_height},
          "poses": []}
t0 = time.time()
for wdir in args.worlds:
    todo = [p for p in plan if p["world"] == str(wdir)]
    lay = json.loads((wdir / "layout.json").read_text())
    pucks = {p["name"]: p for p in lay["pucks"]}
    surf = {n: puck_surface(p) for n, p in pucks.items()}
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    report["robot_usd"] = use_robot_usd(stage)
    updates(10)
    for kind, Kc in (("d435", K), ("arm", K_ARM)):
        scenes = [p for p in todo if p["kind"] == kind]
        if not scenes:
            continue
        near = 0.01 if kind == "d435" else args.near
        cam_path = f"/World/RenderR4/{kind}"
        set_camera(stage, cam_path, np.array(scenes[0]["T_world_cam"]), Kc, near)
        rp = rep.create.render_product(cam_path, (W, H))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach(rp)
        a_dep.attach(rp)
        for li, level in enumerate(args.lights):
            set_lighting(stage, level)
            for pose in scenes:
                T_w_cam = np.array(pose["T_world_cam"])
                T_w_bf = np.array(pose["T_world_base_footprint"])
                set_camera(stage, cam_path, T_w_cam, Kc, near)
                pose_robot(stage, args.urdf, T_w_bf, S0 if kind == "d435" else pose["arm_q"])
                updates(4)
                rgb, dep = grab(a_rgb, a_dep)
                name = f"{pose['id']}_L{level:g}"
                (out / name).mkdir()
                cv2.imwrite(str(out / name / "frame.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                images = {"color": f"{name}/frame.png"}
                if kind == "d435":
                    np.save(out / name / "frame_depth.npy", dep)
                    images["depth"] = f"{name}/frame_depth.npy"
                if li == 0:   # 보이는 정도는 기하만 따르므로 첫 조명에서 한 번
                    truth = []
                    for n, p in pucks.items():
                        e = puck_truth(p)
                        e["visible"], e["front_points"] = visible_fraction(*surf[n], T_w_cam, Kc, dep)
                        e["visible"] = round(e["visible"], 3)
                        u, v, zc = project(T_w_cam, Kc, p["center"])
                        e["px"] = [round(float(u), 1), round(float(v), 1)] if zc > 0 else None
                        e["cam_dist_m"] = round(float(np.linalg.norm(np.array(p["center"]) - T_w_cam[:3, 3])), 4)
                        e["base_hdist_m"] = round(float(np.hypot(*(np.array(p["center"][:2]) - T_w_bf[:2, 3]))), 4)
                        lab = up_label(p) if kind == "arm" else None
                        if lab is not None:
                            c, a1, a2 = lab
                            pts = np.array([c + a1 * a * TAG + a2 * b * TAG for a in np.linspace(-0.45, 0.45, 9)
                                            for b in np.linspace(-0.45, 0.45, 9)])
                            nr = np.tile(np.array([0, 0, 1.0]), (len(pts), 1))
                            e["top_tag_visible"] = round(visible_fraction(pts, nr, T_w_cam, Kc, dep, tol=0.003)[0], 3)
                        truth.append(e)
                    pose["pucks"] = truth
                entry = {k: v for k, v in pose.items()}
                entry.update({"name": name, "light_level": level, "images": images})
                report["poses"].append(entry)
                vis = [(t["name"], t["visible"]) for t in pose["pucks"] if t["visible"] > 0]
                print(name, "보이는 퍽", len(vis), json.dumps(vis), flush=True)
        a_rgb.detach()
        a_dep.detach()
        rp.destroy()
        updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_r4.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"poses": len(report["poses"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
