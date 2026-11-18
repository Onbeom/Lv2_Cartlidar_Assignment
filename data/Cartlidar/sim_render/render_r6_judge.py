"""R6 주사위 확인 렌더 (인지 판정 기준 v1 R6): 선반 위 3 × 3 배치를 팔 카메라로 위에서 본다.

배치: 선반 가운데에 칸 간격 --pitch(0.05 m) 3 × 3, 칸 번호 0~8 행 우선(행 = 월드 −y 쪽으로, 열 = +x 쪽으로).
  눈 모양: 1 {4} · 2 {0,8} · 3 {0,4,8} · 4 {0,2,6,8} · 5 {0,2,4,6,8} · 6 {0,2,3,5,6,8}.
  맞는 배치 18 = 눈 1~6 × 주사위 색 R · G · Y(노랑 = 색 무관, 모양 = 그 월드에 많은 모양).
  틀린 배치 6종 × --wrong-reps(3): WRONG_COLOR(R · G 주사위의 한 칸 색 바꿈) · WRONG_SHAPE(Y 주사위 한 칸 다른 모양) ·
  MISSING(한 칸 비움) · UNEXPECTED(빈 칸 하나에 퍽) · MULTIPLE(한 칸에 퍽 2개, 축 방향 ±16 mm) · OFFSET(한 칸 대각선 24 mm 벗어남).
  퍽은 모두 서 있음. 놓는 오차 ≤ 4 mm, 직육면체 방향 ±10°(MULTIPLE · OFFSET 칸은 0°), 원기둥 방향 무작위. 퍽끼리 겹치지 않고
  선반 안(±0.078 m)인 배치만 쓴다.
  퍽은 그 월드의 퍽 prim 을 옮겨 쓰고(번호 · 모양은 prim 그대로), 색은 몸통 재질을 바꿔 맞춘다. 쓰지 않는 퍽은 바닥 아래(z −1 m)로 옮긴다.
  렌더 확인: 렌더 영상에서 태그를 찾아 정답 라벨 중심 투영과 비교하고, 3 px 넘게 다르면 갱신을 더 돌려 다시 렌더한다(최대 3번, 결과에 기록).
  (2026-10-25 첫 렌더는 숨김 · 보임을 바꾸며 옮긴 퍽 일부가 이전 변환으로 렌더되어 25 px 어긋났음 → results/render_r6_261025_fail_render)
  장면마다 seed 1 · 2 · 3 의 선반 3개를 돌아가며 쓴다.
카메라: 계획 = 선반 윗면 --height(0.20 m) 위, 가로 ±0.01 m · 기울기 ≤ 3° · 방향 무작위, 640 × 480, f 283 px 가정.
  실제 팔 자세(arm_pose.py, 2026-10-27 저녁 사용자 결정): 차체 자리 + 팔 관절로 카메라 위치 · 광축을 맞춤(광축 둘레 회전은 팔이 정함),
  충돌(옮긴 퍽 · 선반 · 벽, 여유 5 mm) · 관절 한계로 못 가면 광축 방향으로 5 mm 씩 물림(최대 0.05 m, backoff_m). 못 만든 배치는 다시 뽑는다.
  보기 자세(2026-10-27 밤 설계 변경, R6 재판정 14/72 실패 뒤): 카메라를 수평으로 옮겨 3 × 3 가운데가 화면 세로 --aim-v(145 px)에 오게 한다
  — 내려다보면 손목 · 집게가 화면 아래 약 40 % 를 가려 아래 줄이 안 보였음.
  로봇 · 집게(gripper 0) 보임, 로봇 모델 = 기준 모델(robot_usd.py, 261027_1711), 가까운 클리핑 2.5 cm.
결과: <out>/<name>/frame.png, <out>/render_r6.json(replay_r6.py 입력 — 칸 좌표(월드 = odom) · 기대값 · 정답 판정 포함)
실행 (PC):
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r6.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_{1,2,3} \
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
from r12_poses import S0, W, H, Scene  # noqa: E402
from r6_views_judge import plan_views,prime_cache
from arm_pose import ArmRealizer  # noqa: E402

TOP = HERE.parents[1]
MODEL = "261027_1711"   # robot_usd.py 와 같게

ap = argparse.ArgumentParser()
ap.add_argument("--lying-prob",type=float,default=0.0,help="직육면체 눕힘 모의 확률, 원기둥은 항상 섬")
ap.add_argument("--view-cache",type=Path)
ap.add_argument("--plan-file", type=Path)
ap.add_argument("--load-plan", type=Path)
ap.add_argument("--case-limit", type=int, default=0)
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--urdf", type=Path, default=TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf")
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--near", type=float, default=0.025, help="[m] 팔 카메라 가까운 클리핑")
ap.add_argument("--aim-v", type=float, default=145.0, help="[px] 보기 자세: 3 × 3 가운데(퍽 윗면 높이)가 올 화면 세로 위치(0 = 옮기지 않음)")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--pitch", type=float, default=0.06)
ap.add_argument("--height", type=float, default=0.18)
ap.add_argument("--wrong-reps", type=int, default=3)
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0])
ap.add_argument("--arm-f", type=float, default=283.0)
ap.add_argument("--tolerance", type=float, default=0.010, help="판정 허용 오차(노드 dice.tolerance_m 과 같게)")
ap.add_argument("--seed", type=int, default=20261026)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--arm-frame", default="camera_optical_frame")
ap.add_argument("--only-plan", action="store_true", help="배치만 뽑고 렌더하지 않음")
args, _ = ap.parse_known_args()
if args.view_cache:prime_cache(json.loads(args.view_cache.read_text())["plan"],args.height,args.aim_v)

LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
K_ARM = {"fx": args.arm_f, "fy": args.arm_f, "cx": W / 2, "cy": H / 2}
PIPS = {1: [4], 2: [0, 8], 3: [0, 4, 8], 4: [0, 2, 6, 8], 5: [0, 2, 4, 6, 8], 6: [0, 2, 3, 5, 6, 8]}
EXPECT = {"EMPTY": 0, "R": 1, "G": 2, "Y": 3}
ERR = {"OK": 0, "MISSING": 1, "UNEXPECTED": 2, "MULTIPLE": 3, "OFFSET": 4, "WRONG_COLOR": 5, "WRONG_SHAPE": 6}
SHAPE_ID = {"cylinder": 1, "cuboid": 2}
COLOR_ID = {"red": 1, "green": 2}
HALF = 0.015
drawing = json.loads(args.drawing.read_text())


def footprint(p):
    """바닥 윤곽 점(2D). 원기둥 24각형, 직육면체 회전 정사각형."""
    c = np.array(p["xy"])
    if p["shape"] == "cylinder":
        th = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        return c + HALF * np.stack([np.cos(th), np.sin(th)], 1)
    a = p["yaw"]
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    hx=.030 if not p.get("standing",True) else HALF
    return c + np.array([[-hx, -HALF], [hx, -HALF], [hx, HALF], [-hx, HALF]]) @ R.T


def inside_convex(pt, poly):
    s = None
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        cr = (b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0])
        if s is None:
            s = cr > 0
        elif (cr > 0) != s:
            return False
    return True


def overlap(p, q, gap=0.001):
    """두 퍽 윤곽이 gap 보다 가깝게 겹치는지 (윤곽 점을 바깥으로 gap 만큼 키워 서로 안에 드는지)."""
    fp, fq = footprint(p), footprint(q)
    cp, cq = np.array(p["xy"]), np.array(q["xy"])
    gp = fp + gap * (fp - cp) / np.linalg.norm(fp - cp, axis=1, keepdims=True)
    gq = fq + gap * (fq - cq) / np.linalg.norm(fq - cq, axis=1, keepdims=True)
    return any(inside_convex(x, fq) for x in gp) or any(inside_convex(x, fp) for x in gq)


def check_dice(cells, expect, yellow_shape, pucks, tol):
    """dice_check.hpp 와 같은 판정(정답 위치로) — 장면을 만든 의도가 맞는지 확인용."""
    inn = [[] for _ in range(9)]
    for p in pucks:
        d = [math.hypot(p["xy"][0] - c[0], p["xy"][1] - c[1]) for c in cells]
        inn[int(np.argmin(d))].append((min(d), p))
    err = []
    for i in range(9):
        n = len(inn[i])
        if expect[i] == 0:
            e = "UNEXPECTED" if n else "OK"
        elif n == 0:
            e = "MISSING"
        elif n > 1:
            e = "MULTIPLE"
        else:
            d, p = inn[i][0]
            if d > tol:
                e = "OFFSET"
            elif expect[i] == 3:
                e = "OK" if SHAPE_ID[p["shape"]] == yellow_shape else "WRONG_SHAPE"
            else:
                e = "OK" if COLOR_ID[p["color"]] == expect[i] else "WRONG_COLOR"
        err.append(ERR[e])
    return err


# ---------- 배치 뽑기 ----------
rng = np.random.default_rng(args.seed)
worlds = []
for wdir in args.worlds:
    wdir=wdir.resolve()
    lay = json.loads((wdir / "layout.json").read_text())
    lay["shelves"] = [dict(s) for s in lay["shelves"]]
    shapes = [p["shape"] for p in lay["pucks"]]
    yshape = max(set(shapes), key=shapes.count)
    sc = Scene(lay, drawing, args.urdf, args.urdf.parent.parent, shelf_top=lay["assumptions"]["shelf_size"][2])
    mission_shelves=[{"name":name,"center":[*lay["assumptions"]["L"][name],0.035],"top_z":0.07,"size":[0.205,0.205,0.07]} for name in ("L1","L2","L3")]
    worlds.append({"targets":mission_shelves,"dir": wdir, "lay": lay, "yellow_shape": yshape, "scene": sc})
slots = [(w, sh) for sh in range(3) for w in range(len(worlds))]   # (월드, 선반) 9 자리를 돌아가며

configs = [{"kind": "OK", "pips": n, "die": d} for d in ("R", "G", "Y") for n in range(1, 7)]
for kind in ("WRONG_COLOR", "WRONG_SHAPE", "MISSING", "UNEXPECTED", "MULTIPLE", "OFFSET"):
    for k in range(args.wrong_reps):
        die = {"WRONG_COLOR": ("R", "G")[k % 2], "WRONG_SHAPE": "Y"}.get(kind, ("R", "G", "Y")[k % 3])
        configs.append({"kind": kind, "pips": int(rng.integers(1, 7)), "die": die})

if args.load_plan:
    plan=json.loads(args.load_plan.read_text())["plan"]
else:
    plan = []
    if args.case_limit: configs=configs[:args.case_limit]
    for ci, cfg in enumerate(configs):
        wi, si = ci % len(worlds), ci % 3
        w = worlds[wi]
        lay = w["lay"]
        sh = w["targets"][si]
        cx, cy, top = sh["center"][0], sh["center"][1], sh["top_z"]
        P = args.pitch
        cells = [[cx + (i % 3 - 1) * P, cy + (1 - i // 3) * P, top] for i in range(9)]
        pips = PIPS[cfg["pips"]]
        expect = [EXPECT[cfg["die"]] if i in pips else 0 for i in range(9)]
        yshape = w["yellow_shape"]
        other = "cylinder" if yshape == "cuboid" else "cuboid"
        for attempt in range(2000):
            target_cell = None
            want = []   # (칸, 색, 모양 또는 None, 놓을 위치 xy, yaw 고정 여부)
            for i in pips:
                col = {"R": "red", "G": "green"}.get(cfg["die"]) or str(rng.choice(["red", "green"]))
                want.append({"cell": i, "color": col, "shape": yshape if cfg["die"] == "Y" else None, "fix_yaw": False,
                             "off": rng.uniform(-1, 1, 2) * 0.004 / math.sqrt(2)})
            kind = cfg["kind"]
            if kind != "OK":
                if kind == "UNEXPECTED":
                    target_cell = int(rng.choice([i for i in range(9) if i not in pips]))
                    want.append({"cell": target_cell, "color": str(rng.choice(["red", "green"])), "shape": None,
                                 "fix_yaw": False, "off": rng.uniform(-1, 1, 2) * 0.004 / math.sqrt(2)})
                else:
                    k = int(rng.integers(len(want)))
                    target_cell = want[k]["cell"]
                    if kind == "WRONG_COLOR":
                        want[k]["color"] = "green" if want[k]["color"] == "red" else "red"
                    elif kind == "WRONG_SHAPE":
                        want[k]["shape"] = other
                    elif kind == "MISSING":
                        want.pop(k)
                    elif kind == "MULTIPLE":
                        ax = np.array([1.0, 0.0]) if rng.random() < 0.5 else np.array([0.0, 1.0])
                        want[k].update({"off": ax * 0.016, "fix_yaw": True})
                        want.append({**want[k], "off": -ax * 0.016})
                    elif kind == "OFFSET":
                        sx, sy = rng.choice([-1, 1], 2)
                        want[k].update({"off": np.array([sx, sy]) * 0.024 / math.sqrt(2), "fix_yaw": True})
            # prim 고르기: 모양이 정해진 것부터, 그 월드의 퍽 prim 에서 겹치지 않게
            free = list(lay["pucks"])
            rng.shuffle(free)
            placed, ok = [], True
            for wnt in sorted(want, key=lambda x: x["shape"] is None):
                cand = [p for p in free if wnt["shape"] is None or p["shape"] == wnt["shape"]]
                if not cand:
                    ok = False
                    break
                p = cand[0]
                free.remove(p)
                xy = np.array(cells[wnt["cell"]][:2]) + wnt["off"]
                yaw = 0.0 if wnt["fix_yaw"] else (math.radians(rng.uniform(-10, 10)) if p["shape"] == "cuboid"
                                                    else rng.uniform(0, 2 * math.pi))
                standing=p["shape"]=="cylinder" or rng.random()>=args.lying_prob
                if not standing:yaw=float(rng.choice([0.,math.pi/2])+math.radians(rng.uniform(-5,5)))
                placed.append({"name": p["name"], "tag_id": p["tag_id"], "shape": p["shape"], "color": wnt["color"],
                               "cell": wnt["cell"], "xy": xy.tolist(), "yaw": float(yaw), "standing":standing,"z": top + (0.03 if standing else .015)})
            if not ok:
                raise RuntimeError(f"{cfg} 에 맞는 퍽 prim 이 모자람")
            if any(overlap(placed[a], placed[b]) for a in range(len(placed)) for b in range(a + 1, len(placed))):
                continue
            if any(np.abs(footprint(p) - [cx, cy]).max() > 0.1025 for p in placed):
                continue
            # 카메라 계획 · 실제 팔 자세 (충돌 상자 = 이 배치의 퍽, 다른 퍽은 바닥 아래로 옮겨 렌더하므로 뺌)
            T_w_cam=np.eye(4);T_w_cam[:3,:3]=np.diag([1.,-1.,-1.]);T_w_cam[:3,3]=[cx,cy,top+args.height]
            colliders=[]
            for q in placed:
                outline=footprint(q);lo=outline.min(0);hi=outline.max(0)
                colliders.append({"name":q["name"],"aabb":[[float(lo[0]),float(lo[1]),top],[float(hi[0]),float(hi[1]),top+(.06 if q["standing"] else .03)]]})
            lay_r=dict(lay,pucks=colliders)
            ar6 = ArmRealizer(args.urdf, args.urdf.parent.parent, w["scene"], lay_r, gripper=0.0)
            # 후보는 실제 차체 윤곽으로 고르고 팔은 새 보기 자세마다 arm_hits로 검사.
            from scipy.spatial import ConvexHull
            body_xy=np.vstack([V[:,:2] for _,V in ar6.body])
            ar6.scene.footprint=body_xy[ConvexHull(body_xy).vertices]
            real = plan_views(ar6,cells,top,height=args.height,aim_v=args.aim_v)
            if real is None:
                raise RuntimeError(f"{cfg}: 9시야 접근 자세 검색 실패, 배치 제외하지 않음")
            break
        else:
            raise RuntimeError(f"{cfg} 배치를 못 뽑음")
        exp_err = [0] * 9
        if cfg["kind"] != "OK":
            exp_err[target_cell] = ERR[cfg["kind"]]
        ys = SHAPE_ID[yshape]
        sim_err = check_dice(cells, expect, ys, placed, args.tolerance)
        if sim_err != exp_err:
            raise RuntimeError(f"{cfg}: 의도 {exp_err} 와 정답 위치 판정 {sim_err} 가 다름")
        plan.append({"id": f"c{ci:02d}_{cfg['kind']}_{cfg['die']}{cfg['pips']}_w{lay['seed']}s{si}", "world": str(w["dir"]),
                     "seed": int(lay["seed"]), "shelf": sh["name"], "kind": cfg["kind"], "die": cfg["die"], "pips": cfg["pips"],
                     "cells": cells, "expect": expect, "yellow_shape": ys, "target_cell": target_cell,
                     "expected_error": exp_err, "expected_passed": cfg["kind"] == "OK",
                     "expected_wrong": [i for i in range(9) if exp_err[i]], "pucks": placed,
                     "T_world_cam": real["frames"][0]["T_world_cam"], "T_world_cam_nominal": T_w_cam.tolist(),
                     "backoff_m": real["frames"][0]["backoff_m"], "aim_v_px": real.get("aim_v_px"), "arm_q": {**real["frames"][0]["q"], "gripper_left_joint": 0.0},
                     "T_world_base_footprint": real["T_world_base_footprint"]})
        group=plan.pop()
        for vi,frame in enumerate(real["frames"]):
            view=dict(group)
            view.update({"id":group["id"]+f"_v{vi}","group_id":group["id"],"view_index":vi,"view_cell":frame["view_cell"],"T_world_base_footprint":frame["T_world_base_footprint"],"T_world_cam":frame["T_world_cam"],"arm_q":{**frame["q"],"gripper_left_joint":0.0},"aim_v_px":frame["aim_v_px"],"backoff_m":frame["backoff_m"]})
            plan.append(view)
for p in plan: p["world"]=str(Path(p["world"]).resolve())
if not plan: raise RuntimeError("렌더 계획이 비어 있음")
print(f"배치 {len(plan)}개 (맞음 {sum(p['expected_passed'] for p in plan)}, 틀림 {sum(not p['expected_passed'] for p in plan)})",
      flush=True)
if args.plan_file:
    args.plan_file.parent.mkdir(parents=True,exist_ok=True)
    args.plan_file.write_text(json.dumps({"plan":plan},indent=1,ensure_ascii=False))
if args.only_plan:
    for p in plan:
        print(p["id"], "물림", p["backoff_m"], p["expected_wrong"], [(q["tag_id"], q["color"][0], q["shape"][:3], q["cell"]) for q in p["pucks"]])
    sys.exit(0)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H, "anti_aliasing": 0, "multi_gpu": False})

import cv2  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdShade  # noqa: E402

from robot_usd import pose_robot, use_robot_usd  # noqa: E402

out = args.out
out.mkdir(parents=True, exist_ok=False)
ctx = omni.usd.get_context()


def updates(n):
    for _ in range(n):
        app.update()


def set_camera(stage, path, T_w_opt, Kc, near=0.01):
    """광학 좌표(x 오른쪽 · y 아래 · z 앞) 자세로 USD 카메라를 둔다. 주점 부호: 가로 −, 세로 + (render_r4.py 와 같음)."""
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


def set_lighting(stage, level):
    dome = LIGHT_RANGE["dome"][0] + level * (LIGHT_RANGE["dome"][1] - LIGHT_RANGE["dome"][0])
    rect = LIGHT_RANGE["rect"][0] + level * (LIGHT_RANGE["rect"][1] - LIGHT_RANGE["rect"][0])
    stage.GetPrimAtPath("/World/Lights/dome").GetAttribute("inputs:intensity").Set(float(dome))
    for i in range(6):
        stage.GetPrimAtPath(f"/World/Lights/ceiling_{i}").GetAttribute("inputs:intensity").Set(float(rect))


def place(stage, lay, pose):
    used = {p["name"]: p for p in pose["pucks"]}
    for p in lay["pucks"]:
        prim = stage.GetPrimAtPath(f"/World/Pucks/{p['name']}")
        if p["name"] not in used:
            prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(p["center"][0], p["center"][1], -1.0))
            continue
        q = used[p["name"]]
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(q["xy"][0], q["xy"][1], q["z"]))
        a=q["yaw"]/2
        if q.get("standing",True):rotation=Gf.Quatf(math.cos(a),Gf.Vec3f(0,0,math.sin(a)))
        else:
            b=math.pi/4
            rotation=Gf.Quatf(math.cos(a)*math.cos(b),Gf.Vec3f(-math.sin(a)*math.sin(b),math.cos(a)*math.sin(b),math.sin(a)*math.cos(b)))
        prim.GetAttribute("xformOp:orient").Set(rotation)
        UsdShade.MaterialBindingAPI(prim.GetChild("body")).Bind(
            UsdShade.Material(stage.GetPrimAtPath(f"/World/Looks/puck_{q['color']}")))


DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def render_error_px(rgb, pose):
    """렌더 영상의 태그 중심과 정답 라벨 중심 투영의 차 [px] 최대값 · 못 찾은 태그 수."""
    pr = cv2.aruco.DetectorParameters()
    pr.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    c, ids, _ = cv2.aruco.ArucoDetector(DICT, pr).detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    seen = {}
    if ids is not None:
        for cc,i in zip(c,ids.ravel()):seen.setdefault(int(i),[]).append(cc.reshape(4,2).mean(0))
    Ti = np.linalg.inv(np.array(pose["T_world_cam"]))
    worst, missing = 0.0, 0
    for q in pose["pucks"]:
        v = Ti @ np.array([q["xy"][0], q["xy"][1], q["z"] + (0.0302 if q.get("standing",True) else 0.0152), 1.0])
        uv = np.array([K_ARM["fx"] * v[0] / v[2] + K_ARM["cx"], K_ARM["fy"] * v[1] / v[2] + K_ARM["cy"]])
        if q["tag_id"] in seen:
            worst = max(worst, float(min(np.linalg.norm(center-uv) for center in seen[q["tag_id"]])))
        else:
            missing += 1
    return round(worst, 2), missing


report = {"urdf": str(args.urdf), "robot_usd": None, "width": W, "height": H, "lights": args.lights, "seed": args.seed, "arm_q": S0,
          "arm": {"frame": args.arm_frame, "K": K_ARM, "height": args.height}, "pitch": args.pitch,
          "tolerance": args.tolerance, "multiview": True, "lying_prob":args.lying_prob,"poses": []}
t0 = time.time()
for w in worlds:
    todo = [p for p in plan if p["world"] == str(w["dir"])]
    print("단계: 월드 열기", w["dir"], flush=True)
    ctx.open_stage(str((w["dir"] / "task_world.usda").resolve()))
    print("단계: 월드 열기 완료", flush=True)
    updates(60)
    print("단계: 월드 초기 갱신 완료", flush=True)
    stage = ctx.get_stage()
    report["robot_usd"] = use_robot_usd(stage)
    updates(10)
    print("단계: 로봇 참조 갱신 완료", flush=True)
    cam_path = "/World/RenderR6/arm"
    set_camera(stage, cam_path, np.array(todo[0]["T_world_cam"]), K_ARM, args.near)
    rp = rep.create.render_product(cam_path, (W, H))
    a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    a_rgb.attach(rp)
    print("단계: 렌더 제품 연결 완료", flush=True)
    for level in args.lights:
        set_lighting(stage, level)
        for pose in todo:
            print("단계: 장면",pose["id"],level,flush=True)
            place(stage, w["lay"], pose)
            pose_robot(stage, args.urdf, np.array(pose["T_world_base_footprint"]), pose["arm_q"])
            set_camera(stage, cam_path, np.array(pose["T_world_cam"]), K_ARM, args.near)
            tries = []
            for k in range(3):
                updates(10 * (k + 1))
                for _ in range(2):
                    rep.orchestrator.step(rt_subframes=args.subframes)
                rgb = np.asarray(a_rgb.get_data())[:, :, :3].astype(np.uint8)
                tries.append(render_error_px(rgb, pose))
                if tries[-1][0] <= 3.0:
                    break
            name = f"{pose['id']}_L{level:g}"
            (out / name).mkdir()
            cv2.imwrite(str(out / name / "frame.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            e = dict(pose)
            e.update({"name": name, "light_level": level, "images": {"color": f"{name}/frame.png"},
                      "render_check": {"tries": tries, "max_tag_px": tries[-1][0], "missing_tags": tries[-1][1]}})
            report["poses"].append(e)
            print(name, "기대 틀린 칸", pose["expected_wrong"], "렌더 확인(최대 px, 못 찾음)", tries, flush=True)
    a_rgb.detach()
    rp.destroy()
    updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_r6.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"poses": len(report["poses"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
