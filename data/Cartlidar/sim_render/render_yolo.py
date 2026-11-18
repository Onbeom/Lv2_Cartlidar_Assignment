"""YOLO 퍽 학습 자료 렌더 (색 × 모양 6클래스): 앞 D435 장면, 퍽은 월드(layout.json)에 있는 그대로 쓴다.

클래스 번호 = 모양 × 3 + 색. 색 0 red · 1 green · 2 blue(팀 perception class_id 와 같은 순서, 지금 월드에는 파랑 없음),
  모양 0 cylinder · 1 cuboid. → 0 red_cylinder · 1 green_cylinder · 2 blue_cylinder · 3 red_cuboid · 4 green_cuboid · 5 blue_cuboid
장면 뽑기는 render_r4.py 와 같은 방식(차체 윤곽이 미로 안 · 벽에서 0.02 m 넘게, 바닥 퍽 안 밟음, 목표가 화면 안, 시선 안 막힘).
  목표 = 선반 가운데(돌아가며) 또는 --floor-prob 확률로 바닥 퍽. base_footprint → 목표 수평 --dist, 정면 ±--bearing°,
  카메라 흔들림 ±--jitter°. 로봇 몸체 · 접은 팔 S0 포함.
조명: --lights 를 주면 장면마다 그 값들로 모두 렌더(평가용, R4 와 같게 0 · 1), --light-random 이면 장면마다 U(0, 1) 하나.
--color-jitter j: 프레임마다 퍽 재질 밝기 × U(1 − j, 1 + j), 채널마다 ± j/3, 바닥 밝기 × U(1 − j, 1 + j) (학습 자료 다양화).
라벨: 인스턴스 분할(instance_id_segmentation_fast)에서 퍽마다 보이는 화소의 상자. 화소 ≥ --min-px 이고
  보이는 정도(render_r4.py 와 같은 계산: 카메라를 향한 겉면 점 중 렌더 깊이로 보이는 비율) ≥ --min-vis 인 퍽만 라벨에 넣는다.
  그보다 덜 보이는 조각은 라벨 없이 두고 truth 에 unlabeled 로 센다.

결과: <out>/images/<split>/<name>.png, <out>/labels/<split>/<name>.txt(YOLO 형식),
      <out>/truth_<split>_<part>.json(장면 · 퍽 정답: 색 · 모양 · 자세 · 상자 · 화소 수 · 보이는 정도 · base_footprint 수평 거리)
실행 (PC):
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_yolo.py --worlds <월드 폴더들> --split train --out <자료 폴더> \
      --urdf 261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf --scenes 60 --light-random \
      --color-jitter 0.2 --seed <seed> --/crashreporter/url=
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
from urdf_chain import joint_locals, planar_pose, rpy_matrix  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--split", required=True, choices=["train", "val", "eval"])
ap.add_argument("--urdf", type=Path, required=True)
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--scenes", type=int, default=60, help="월드마다 장면 수")
ap.add_argument("--dist", type=float, nargs=2, default=[0.3, 1.6])
ap.add_argument("--bearing", type=float, default=25.0)
ap.add_argument("--jitter", type=float, default=2.0)
ap.add_argument("--floor-prob", type=float, default=0.25)
ap.add_argument("--lights", type=float, nargs="+", default=None)
ap.add_argument("--light-random", action="store_true")
ap.add_argument("--color-jitter", type=float, default=0.0)
ap.add_argument("--min-px", type=int, default=12)
ap.add_argument("--min-vis", type=float, default=0.2)
ap.add_argument("--seed", type=int, required=True)
ap.add_argument("--part", default="0", help="같은 split 을 여러 번 나눠 렌더할 때 truth 파일 이름 구분")
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--plan-only", action="store_true", help="장면만 뽑고 끝냄(GPU 안 씀)")
args, _ = ap.parse_known_args()
if (args.lights is None) == (not args.light_random):
    sys.exit("--lights 와 --light-random 중 하나만 준다")

LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
COLORS = {"red": (0.75, 0.08, 0.07), "green": (0.10, 0.55, 0.18)}  # build_task_world.py 와 같음
CARPET = (0.30, 0.31, 0.32)
ROBOT = "/World/waffle_4wd"
CUBOID_HALF = (0.015, 0.015, 0.03)
CYL_R, CYL_HALF = 0.015, 0.03
COLOR_IDX = {"red": 0, "green": 1, "blue": 2}
SHAPE_IDX = {"cylinder": 0, "cuboid": 1}
CLASS_NAMES = [f"{c}_{s}" for s in SHAPE_IDX for c in COLOR_IDX]
pkg_dir = args.urdf.parent.parent
drawing = json.loads(args.drawing.read_text())


def cls_of(p):
    return SHAPE_IDX[p["shape"]] * 3 + COLOR_IDX[p["color"]]


def puck_surface(p):
    """render_r4.py 와 같음: 퍽 겉면 점 · 바깥 법선 (월드)."""
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


def visible_fraction(pts, nrm, T_w_cam, Kc, dep, tol=0.004):
    """render_r4.py 와 같음: 카메라를 향한 점 중 화면 안에서 렌더 깊이로 보이는 비율."""
    Ti = np.linalg.inv(T_w_cam)
    cam = T_w_cam[:3, 3]
    view = cam - pts
    cosv = np.einsum("ij,ij->i", nrm, view) / np.linalg.norm(view, axis=1)
    front = cosv > 0.15
    if not front.any():
        return 0.0
    q = (Ti[:3, :3] @ pts[front].T).T + Ti[:3, 3]
    vis = 0
    for x, y, z in q:
        if z <= 0:
            continue
        u, v = Kc["fx"] * x / z + Kc["cx"], Kc["fy"] * y / z + Kc["cy"]
        if 0 <= u < W and 0 <= v < H and np.isfinite(dep[int(v), int(u)]) and dep[int(v), int(u)] > z - tol:
            vis += 1
    return vis / int(front.sum())


def project(T_w_cam, Kc, p):
    q = np.linalg.inv(T_w_cam) @ np.append(np.asarray(p, float), 1.0)
    return Kc["fx"] * q[0] / q[2] + Kc["cx"], Kc["fy"] * q[1] / q[2] + Kc["cy"], q[2]


# ---------- 장면 뽑기 (렌더 전, 같은 seed 면 같은 결과) ----------
rng = np.random.default_rng(args.seed)
plan = []
tries = 0
for wdir in args.worlds:
    lay = json.loads((wdir / "layout.json").read_text())
    sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, shelf_top=lay["assumptions"]["shelf_size"][2])
    floor_p = [p for p in lay["pucks"] if p["location"] == "floor"]
    floor = [np.array(p["center"][:2]) for p in floor_p]
    shelves = lay["shelves"]
    for k in range(args.scenes):
        if floor_p and rng.random() < args.floor_prob:
            fp_ = floor_p[int(rng.integers(len(floor_p)))]
            c, target, tname = np.array(fp_["center"][:2]), np.array(fp_["center"]), fp_["name"]
        else:
            sh = shelves[k % len(shelves)]
            c = np.array(sh["center"][:2])
            target, tname = np.array([c[0], c[1], sh["top_z"] + 0.03]), sh["name"]
        for _ in range(20000):
            tries += 1
            d = rng.uniform(*args.dist)
            phi = rng.uniform(0, 2 * math.pi)
            bearing = math.radians(rng.uniform(-args.bearing, args.bearing))
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
            J[:3, :3] = rpy_matrix(*[math.radians(rng.uniform(-args.jitter, args.jitter)) for _ in range(3)])
            T_w_cam = T_w_bf @ sc.T_bf_cam @ J
            u, v, zc = project(T_w_cam, K, target)
            if not (zc > 0 and 30 <= u < W - 30 and 30 <= v < H - 30):
                continue
            if sc.los_blocked(T_w_cam[:3, 3], target, end_skip=0.05):
                continue
            levels = list(args.lights) if args.lights else [float(rng.uniform(0, 1))]
            jit = None
            if args.color_jitter > 0:
                j = args.color_jitter
                jit = {n: (float(rng.uniform(1 - j, 1 + j)), rng.uniform(-j / 3, j / 3, 3).tolist()) for n in COLORS}
                jit["carpet"] = (float(rng.uniform(1 - j, 1 + j)), [0.0, 0.0, 0.0])
            plan.append({"id": f"{args.split}_w{lay['seed']}_d{k:03d}", "world": str(wdir), "seed": int(lay["seed"]),
                         "target": tname, "dist_nominal": round(float(d), 4), "bearing_deg": math.degrees(bearing),
                         "T_world_base_footprint": T_w_bf.tolist(), "T_world_cam": T_w_cam.tolist(),
                         "levels": levels, "color_jitter": jit})
            break
        else:
            raise RuntimeError(f"{wdir.name} {tname} D435 장면을 못 뽑음")
print(f"{args.split}: 장면 {len(plan)}개, 영상 {sum(len(p['levels']) for p in plan)}장, 뽑기 시도 {tries}", flush=True)

if args.plan_only:
    sys.exit(0)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

img_dir, lab_dir = args.out / "images" / args.split, args.out / "labels" / args.split
truth_path = args.out / f"truth_{args.split}_{args.part}.json"
if truth_path.exists():
    sys.exit(f"{truth_path} 이 이미 있음")
img_dir.mkdir(parents=True, exist_ok=True)
lab_dir.mkdir(parents=True, exist_ok=True)
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
    """render_r4.py 와 같음: 광학 좌표 자세로 USD 카메라를 둔다."""
    cam = UsdGeom.Camera.Define(stage, path)
    ha = 20.955
    focal = Kc["fx"] * ha / W
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * H / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - W / 2) / W * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - H / 2) / W * ha)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
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


def set_colors(stage, jit):
    for name, base in list(COLORS.items()) + [("carpet", CARPET)]:
        path = f"/World/Looks/{'carpet_gray' if name == 'carpet' else 'puck_' + name}/PBR"
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        s, n = jit[name] if jit else (1.0, [0.0, 0.0, 0.0])
        col = [min(max(base[i] * s + n[i], 0.0), 1.0) for i in range(3)]
        prim.GetAttribute("inputs:diffuseColor").Set(Gf.Vec3f(*col))


def grab(a_rgb, a_dep, a_seg):
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=args.subframes)
    seg = a_seg.get_data()
    return (np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8), np.asarray(a_dep.get_data()).astype(np.float32),
            np.asarray(seg["data"]).reshape(H, W), seg["info"]["idToLabels"])


def puck_masks(seg, id_to_path, names):
    """퍽 이름 → 보이는 화소 (ys, xs). 퍽 아래 메시(body · label_*) id 를 모두 합친다."""
    ids = {n: [] for n in names}
    for i, path in id_to_path.items():
        parts = str(path).split("/")
        if len(parts) >= 4 and parts[2] == "Pucks" and parts[3] in ids:
            ids[parts[3]].append(int(i))
    return {n: np.nonzero(np.isin(seg, v)) if v else (np.array([]), np.array([])) for n, v in ids.items()}


report = {"split": args.split, "urdf": str(args.urdf), "width": W, "height": H, "K": K, "frame": args.frame,
          "class_names": CLASS_NAMES, "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()
                                               if k != "worlds"},
          "worlds": [str(w) for w in args.worlds], "images": []}
report["args"]["urdf"] = str(args.urdf)
t0 = time.time()
n_lab = n_unlab = 0
for wdir in args.worlds:
    todo = [p for p in plan if p["world"] == str(wdir)]
    lay = json.loads((wdir / "layout.json").read_text())
    pucks = {p["name"]: p for p in lay["pucks"]}
    surf = {n: puck_surface(p) for n, p in pucks.items()}
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    stage.GetPrimAtPath(ROBOT).GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
    updates(10)
    for link, (jname, T) in joint_locals(str(args.urdf), S0).items():
        if jname.startswith(("RMX52_", "gripper_")):
            set_xform(find_link(stage, link), T)
    cam_path = "/World/RenderYolo/d435"
    set_camera(stage, cam_path, np.array(todo[0]["T_world_cam"]), K)
    rp = rep.create.render_product(cam_path, (W, H))
    a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    a_seg = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation_fast", init_params={"colorize": False})
    for a in (a_rgb, a_dep, a_seg):
        a.attach(rp)
    for pose in todo:
        T_w_cam = np.array(pose["T_world_cam"])
        T_w_bf = np.array(pose["T_world_base_footprint"])
        set_camera(stage, cam_path, T_w_cam, K)
        set_xform(stage.GetPrimAtPath(ROBOT), T_w_bf)
        set_colors(stage, pose["color_jitter"])
        geo = None
        for level in pose["levels"]:
            set_lighting(stage, level)
            updates(4)
            rgb, dep, seg, id_to_path = grab(a_rgb, a_dep, a_seg)
            name = f"{pose['id']}_L{level:.2f}"
            if geo is None:   # 보이는 정도 · 화소는 기하만 따르므로 장면마다 한 번
                masks = puck_masks(seg, id_to_path, list(pucks))
                geo = []
                for n, p in pucks.items():
                    ys, xs = masks[n]
                    e = {k: p[k] for k in ("name", "tag_id", "color", "shape", "standing", "location", "center")}
                    e["cls"] = cls_of(p)
                    e["px"] = int(len(xs))
                    e["bbox"] = ([int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]
                                 if len(xs) else None)
                    e["visible"] = round(visible_fraction(*surf[n], T_w_cam, K, dep), 3)
                    e["cam_dist_m"] = round(float(np.linalg.norm(np.array(p["center"]) - T_w_cam[:3, 3])), 4)
                    e["base_hdist_m"] = round(float(np.hypot(*(np.array(p["center"][:2]) - T_w_bf[:2, 3]))), 4)
                    e["labeled"] = bool(e["px"] >= args.min_px and e["visible"] >= args.min_vis)
                    geo.append(e)
            cv2.imwrite(str(img_dir / f"{name}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            lines = []
            for e in geo:
                if e["labeled"]:
                    x0, y0, x1, y1 = e["bbox"]
                    lines.append(f"{e['cls']} {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} "
                                 f"{(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}")
                    n_lab += 1
                elif e["px"] > 0:
                    n_unlab += 1
            (lab_dir / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            entry = {k: v for k, v in pose.items() if k != "levels"}
            entry.update({"name": name, "light_level": level, "image": f"images/{args.split}/{name}.png",
                          "pucks": [e for e in geo if e["px"] > 0 or e["visible"] > 0]})
            report["images"].append(entry)
        if len(report["images"]) % 50 < len(pose["levels"]):
            print(f"{len(report['images'])}장 {time.time() - t0:.0f} s, 라벨 {n_lab} · 라벨 없는 조각 {n_unlab}", flush=True)
    for a in (a_rgb, a_dep, a_seg):
        a.detach()
    rp.destroy()
    updates(4)

report.update({"total_s": round(time.time() - t0, 1), "labels": n_lab, "unlabeled_fragments": n_unlab})
truth_path.write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"split": args.split, "images": len(report["images"]), "labels": n_lab, "unlabeled": n_unlab,
                  "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
