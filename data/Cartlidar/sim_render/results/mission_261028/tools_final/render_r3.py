"""R3 바코드 렌더 (인지 판정 기준 v1 R3): 팔 카메라 읽기 장면과 앞 D435 찾기 장면.

라벨: 월드(layout.json)의 바코드(벽 · 선반 윗면) + 바코드가 없는 선반마다 윗면 하나를 더 둔다(벽 쪽 가장자리 띠,
  build_task_world.py 와 같은 자리). 더한 라벨은 그 월드의 바코드 prim 하나를 그 자리로 옮겨 쓴다(문자열 = 그 prim 의 것).
  라벨 자리에 걸치는 퍽은 그 라벨 장면 동안 바닥 아래(z −1)로 치운다(결과에 기록).
arm 장면(640 × 480, f 283 px 가정 — 카메라는 모두 640×480(사용자 2026-10-23), 가까이서 읽기(사용자 2026-10-26)): 라벨마다 기울기 --arm-tilts(0 · 20 · 40°)
  × 기울기 방향 무작위, 라벨 가운데(±3 mm)를 겨눔, 카메라 가로축 힌트 = 라벨 가로(막대에 수직) ± --arm-roll(10°), 반은 180° 뒤집음.
  실제 팔 자세(arm_pose.py, 2026-10-27 저녁 사용자 결정 1 · 2): 집게 끝이 광축 방향으로 약 0.10 m 앞이라 예전 거리(0.04~0.06 m)는 갈 수 없다.
  그래서 기울기 · 방향마다 --arm-dmin-start(0.04 m)부터 광축을 따라 1 cm 씩 물려(뒤에 5 mm 다듬음) 충돌(여유 5 mm) 없이 갈 수 있는 첫 거리 d_min 을 찾고,
  거리 = d_min + --arm-extra(0 · 0.01 · 0.02 m) 세 장면을 만든다(그 거리에서 다시 못 가면 5 mm 씩 최대 0.05 m 더 물림).
  카메라 가로축(영상 회전)은 팔이 정한다 — 힌트는 차체 자리 · 첫 풀이에만 쓰이고 실제 값은 roll_actual_deg 로 남김.
  시선이 벽 · 기둥 · 선반에 막히거나 d_min 이 --arm-dmin-max(0.20 m) 안에서 없으면 그 방향은 다시 뽑는다(최대 10번, 끝내 없으면 기록).
  로봇 · 집게(gripper 0) 보임, 로봇 모델 = 기준 모델(robot_usd.py, 261027_1711), 가까운 클리핑 2.5 cm.
  (2026-10-25 첫 렌더 640 × 480 0.06 · 0.08 · 0.10 m 는 0.08 m 에서 막대 하나가 1.1 화소라 zbar 가 하나도 못 읽음 → results/render_r3_261025_640.
   40차~43차 렌더는 대체된 1080p · f 636 으로 했음 — 팔 결과는 640×480 시험이 아님)
d435 장면: 라벨마다 --d435-scenes 개. 로봇 장착 앞 D435, 몸체 · 접은 팔 S0 포함. base_footprint → 라벨 수평 거리 U(--dist),
  벽 라벨은 라벨 앞쪽 반평면(법선에서 ±80°), 라벨이 정면에서 ±15° 안, 카메라 흔들림 ±1°.
  조건은 render_r4.py 와 같음(미로 안 · 0.02 m 떨어짐 · 바닥 퍽 안 밟음 · 라벨 가운데 화면 안(여백 20 px) · 시선 안 막힘).
렌더 확인: 라벨 위 7 × 5 점이 렌더 깊이로 보이는 비율(label_visible). arm 장면은 라벨 안 화소의 밝기 표준편차도 본다
  (옮긴 라벨이 이전 변환으로 렌더되면 막대가 안 보임 — R6 첫 렌더 실패 참고). 표준편차 < 30 이면 갱신을 더 돌려 다시(최대 3번).
module_px: 라벨 가로 가운데 선의 화면 길이 / 모듈 수(조용한 영역 포함 110) — 막대 하나가 몇 화소인지.

결과: <out>/<name>/frame.png(d435 는 frame_depth.npy 도), <out>/render_r3.json(replay_r3.py 입력)
실행 (PC):
  ~/isaacsim-venv/bin/python Cartlidar/sim_render/render_r3.py --worlds Cartlidar/sim_world/worlds/b_dice/seed_{1,2,3} \
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
sys.path.insert(0, str(HERE.parent / "sim_world"))
import code128  # noqa: E402
from r12_poses import K, S0, W, H, Scene, _point_in_poly  # noqa: E402
from urdf_chain import planar_pose, rpy_matrix  # noqa: E402
from arm_pose import ArmRealizer  # noqa: E402

TOP = HERE.parents[1]
MODEL = "261027_1711"   # robot_usd.py 와 같게

ap = argparse.ArgumentParser()
ap.add_argument("--mission", action="store_true", help="발제 벽 바코드 한 장, 1080p·기울기0/15·extra0/0.02/0.04")
ap.add_argument("--worlds", type=Path, nargs="+", required=True)
ap.add_argument("--urdf", type=Path, default=TOP / f"{MODEL}/urdf/waffle_4wd_description_{MODEL}/urdf/waffle_4wd.urdf")
ap.add_argument("--drawing", type=Path, default=HERE.parent / "sim_world/data/maze_upper_drawing.json")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--arm-dmin-start", type=float, default=0.04)
ap.add_argument("--arm-dmin-max", type=float, default=0.20)
ap.add_argument("--arm-extra", type=float, nargs="+", default=[0.0, 0.01, 0.02])
ap.add_argument("--near", type=float, default=0.025, help="[m] 팔 카메라 가까운 클리핑")
ap.add_argument("--arm-tilts", type=float, nargs="+", default=[0.0, 20.0, 40.0])
ap.add_argument("--arm-roll", type=float, default=10.0)
ap.add_argument("--arm-f", type=float, default=283.0, help="B0201 640×480(16:9 센서 가운데 4:3 을 줄였다고 가정), 핀홀 근사")
ap.add_argument("--arm-size", type=int, nargs=2, default=[640, 480], help="카메라는 모두 640×480(사용자 2026-10-23)")
ap.add_argument("--d435-scenes", type=int, default=4, help="라벨마다 D435 장면 수")
ap.add_argument("--dist", type=float, nargs=2, default=[0.3, 1.0])
ap.add_argument("--lights", type=float, nargs="+", default=[0.0, 1.0])
ap.add_argument("--seed", type=int, default=20261027)
ap.add_argument("--subframes", type=int, default=8)
ap.add_argument("--frame", default="top_camera_front_color_optical_frame")
ap.add_argument("--arm-frame", default="camera_optical_frame")
ap.add_argument("--search-bases", type=int, default=4, help="거리 검색에서 조사할 차체 후보 수")
ap.add_argument("--retry-unreachable-from", type=Path, help="이 계획에서 자세를 못 찾은 라벨·기울기만 재검색")
ap.add_argument("--only-plan", action="store_true", help="장면만 뽑고 렌더하지 않음")
ap.add_argument("--save-plan", type=Path, help="뽑은 장면을 이 json 에 저장 (--only-plan 과 함께 월드마다 따로 계획할 때)")
ap.add_argument("--load-plan", type=Path, nargs="+", help="저장한 계획 json 들을 합쳐 렌더 (계획을 다시 하지 않음, --worlds 는 같은 월드를 줌)")
args, _ = ap.parse_known_args()
if args.mission:
    args.arm_size = [1920, 1080]
    args.arm_f = 636.0
    args.arm_tilts = [0.0, 15.0]
    args.arm_extra = [0.0, 0.02, 0.04]
    args.d435_scenes = 0

class MissionArmRealizer(ArmRealizer):
    """기존 B0 벽 도달 검사와 같은 넓은 초기값으로 손목 ±π 해도 찾는다."""
    def solve_arm(self, T_bf, T_des, seeds):
        if not hasattr(self, "_mission_seeds"):
            rng_ik = np.random.default_rng(0)
            lo, hi = [l for l, _ in self.lim], [h for _, h in self.lim]
            self._mission_seeds = [rng_ik.uniform(lo, hi) for _ in range(12)]
            self._mission_seeds += [np.array([0., 0.3, 0.3, 0., 0.]), np.array([0., 0.8, -0.5, -0.3, 0.])]
        yield from super().solve_arm(T_bf, T_des, list(seeds) + self._mission_seeds)


if args.mission:
    ArmRealizer = MissionArmRealizer

LIGHT_RANGE = {"dome": (300.0, 600.0), "rect": (0.0, 20000.0)}   # build_task_world.py 와 같음
ROBOT = "/World/waffle_4wd"
WALL_SIDE = {"bottom": (np.array([0.0, 1.0]), "x"), "top": (np.array([0.0, -1.0]), "x"),   # 선반 벽 쪽 → 칸 안 법선 · 벽 축
             "left": (np.array([1.0, 0.0]), "y"), "right": (np.array([-1.0, 0.0]), "y")}   # (build_task_world.py 칸 가장자리)
AW, AH = args.arm_size
K_ARM = {"fx": args.arm_f, "fy": args.arm_f, "cx": AW / 2, "cy": AH / 2, "w": AW, "h": AH}
K_D = {**K, "w": W, "h": H}
pkg_dir = args.urdf.parent.parent
drawing = json.loads(args.drawing.read_text())
unreachable = []
retry = None
if args.retry_unreachable_from:
    retry = {(p["label"], p["tilt_deg"]) for p in json.loads(args.retry_unreachable_from.read_text())["unreachable"]}


def label_frame(lb):
    """라벨 좌표 (가로 u · 세로 v · 법선 n · 가운데) → 4 × 4."""
    n, u = np.array(lb["normal"], float), np.array(lb["u"], float)
    F = np.eye(4)
    F[:3, 0], F[:3, 1], F[:3, 2], F[:3, 3] = u, np.cross(n, u), n, lb["center"]
    return F


def label_points(lb, nu=7, nv=5, lift=0.0):
    F = label_frame(lb)
    bw, bh = lb["size"]
    return np.array([F[:3, 3] + F[:3, 0] * a * bw / 2 + F[:3, 1] * b * bh / 2 + F[:3, 2] * lift
                     for a in np.linspace(-0.85, 0.85, nu) for b in np.linspace(-0.85, 0.85, nv)])


def project(T_w_cam, Kc, p):
    q = np.linalg.inv(T_w_cam) @ np.append(np.asarray(p, float), 1.0)
    return Kc["fx"] * q[0] / q[2] + Kc["cx"], Kc["fy"] * q[1] / q[2] + Kc["cy"], q[2]


def module_px(T_w_cam, Kc, lb):
    F = label_frame(lb)
    a = project(T_w_cam, Kc, F[:3, 3] - F[:3, 0] * lb["size"][0] / 2)
    b = project(T_w_cam, Kc, F[:3, 3] + F[:3, 0] * lb["size"][0] / 2)
    return math.hypot(b[0] - a[0], b[1] - a[1]) / len(code128.modules(lb["text"]))


def look_at(cam, target, right_hint):
    """광학 좌표 자세: z = 시선, x = right_hint 를 시선에 수직으로 만든 것."""
    z = np.asarray(target, float) - cam
    z /= np.linalg.norm(z)
    x = np.asarray(right_hint, float) - z * (np.asarray(right_hint, float) @ z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, cam
    return T


# ---------- 라벨 · 장면 뽑기 (렌더 전, 같은 seed 면 같은 결과) ----------
rng = np.random.default_rng(args.seed)
plan, labels = [], []
if args.load_plan is None:
    for wdir in args.worlds:
        lay = json.loads((wdir / "layout.json").read_text())
        A = lay["assumptions"]
        sc = Scene(lay, drawing, args.urdf, pkg_dir, frame=args.frame, shelf_top=A["shelf_size"][2])
        floor = [np.array(p["center"][:2]) for p in lay["pucks"] if p["location"] == "floor"]
        wl = []
        for k, bc in enumerate(lay["barcodes"]):
            wl.append({**bc, "prim": f"/World/Barcodes/barcode_{k}", "source": "world", "hide_pucks": []})
        has = {bc.get("shelf") for bc in lay["barcodes"] if bc["where"] == "shelf_top"}
        bw, bh = A["barcode_label"]
        sy = A["shelf_size"][1]
        j = 0
        for sh in ([] if args.mission else lay["shelves"]):
            if sh["name"] in has:
                continue
            nrm, axis = WALL_SIDE[sh["wall_side"]]
            c = np.array(sh["center"][:2]) - nrm * (sy / 2 - bh / 2 - 0.004)
            half = np.array([bw / 2, bh / 2]) if axis == "x" else np.array([bh / 2, bw / 2])
            hide = [p["name"] for p in lay["pucks"] if p["location"] == sh["name"] and
                    np.all(np.array(p["aabb"][0][:2]) < c + half + 0.005) and np.all(np.array(p["aabb"][1][:2]) > c - half - 0.005)]
            k = j % len(lay["barcodes"])
            j += 1
            wl.append({"text": lay["barcodes"][k]["text"], "where": "shelf_top", "shelf": sh["name"],
                       "center": [float(c[0]), float(c[1]), sh["top_z"] + 0.0005], "normal": [0.0, 0.0, 1.0],
                       "u": [1.0, 0.0, 0.0] if axis == "x" else [0.0, 1.0, 0.0], "size": [bw, bh],
                       "prim": f"/World/Barcodes/barcode_{k}", "source": "moved", "from": lay["barcodes"][k]["center"],
                       "from_normal": lay["barcodes"][k]["normal"], "from_u": lay["barcodes"][k]["u"], "hide_pucks": hide})
        for li, lb in enumerate(wl):
            lb["id"] = f"w{lay['seed']}_b{li}_{lb['where']}"
            lb["world"] = str(wdir)
            F = label_frame(lb)
            c, u, v, n = F[:3, 3], F[:3, 0], F[:3, 1], F[:3, 2]
            pts = label_points(lb, 3, 3, lift=0.002)
            # 팔 카메라: 이 라벨 동안 치우는 퍽은 충돌에서 뺀다
            lay_l = dict(lay, pucks=[p for p in lay["pucks"] if p["name"] not in lb["hide_pucks"]])
            floor_l = [np.array(p["center"][:2]) for p in lay_l["pucks"] if p["location"] == "floor"]
            ar_search = ArmRealizer(args.urdf, pkg_dir, sc, lay_l, gripper=0.0, step=0.01,
                                    backoff_max=args.arm_dmin_max - args.arm_dmin_start)   # 1 cm 로 찾고 아래에서 5 mm 다듬음
            ar = ArmRealizer(args.urdf, pkg_dir, sc, lay_l, gripper=0.0)
            if args.mission:
                ar.backoff_max = 0.0  # d_min+extra를 정확히 시험; 더 물려 다른 조건으로 바꾸지 않음
            for t in args.arm_tilts:
                if retry is not None and (lb["id"], t) not in retry:
                    continue
                tr = math.radians(t)
                found = None
                for attempt in range(10):
                    az = rng.uniform(0, 2 * math.pi)
                    aim = c + u * rng.uniform(-0.003, 0.003) + v * rng.uniform(-0.003, 0.003)
                    dirv = math.cos(tr) * n + math.sin(tr) * (math.cos(az) * u + math.sin(az) * v)
                    roll = math.radians(rng.uniform(-args.arm_roll, args.arm_roll))
                    flip = bool(rng.random() < 0.5)
                    hint = (math.cos(roll) * u + math.sin(roll) * v) * (-1.0 if flip else 1.0)
                    r0 = ar_search.realize([look_at(aim + args.arm_dmin_start * dirv, aim, hint)], floor_l, max_bases=args.search_bases)
                    if r0 is None:
                        continue
                    dmin = round(args.arm_dmin_start + r0["frames"][0]["backoff_m"], 4)
                    if dmin - 0.005 >= args.arm_dmin_start:   # 5 mm 다듬기(물림 없이 되는지)
                        ar.backoff_max = 0.0
                        if ar.realize([look_at(aim + (dmin - 0.005) * dirv, aim, hint)], floor_l, max_bases=args.search_bases) is not None:
                            dmin = round(dmin - 0.005, 4)
                        ar.backoff_max = 0.0 if args.mission else 0.05
                    rows = []
                    for extra in args.arm_extra:
                        r = ar.realize([look_at(aim + (dmin + extra) * dirv, aim, hint)], floor_l)
                        if r is None:
                            break
                        T_w_cam = np.array(r["frames"][0]["T_world_cam"])
                        if any(sc.los_blocked(T_w_cam[:3, 3], p, end_skip=0.0) for p in pts):
                            break
                        rows.append((extra, r))
                    if len(rows) == len(args.arm_extra):
                        found = (az, roll, flip, dmin, rows)
                        break
                if found is None:
                    unreachable.append({"label": lb["id"], "tilt_deg": t, "height_m": float(c[2]), "attempts": 10, "search_bases": args.search_bases, "reason": "정해진 검색에서 모든 거리 조건의 충돌 없는 팔 자세를 찾지 못함"})
                    print(lb["id"], "기울기", t, "실제 팔 자세 없음", flush=True)
                    continue
                az, roll, flip, dmin, rows = found
                for extra, r in rows:
                    f = r["frames"][0]
                    T_w_cam = np.array(f["T_world_cam"])
                    d = round(dmin + extra + f["backoff_m"], 4)
                    ux = T_w_cam[:3, 0]
                    plan.append({"kind": "arm", "id": f"{lb['id']}_a_e{int(round(extra * 100)):02d}_t{int(t):02d}",
                                 "label": lb["id"], "world": str(wdir), "seed": int(lay["seed"]),
                                 "dist_nominal": d, "d_min": dmin, "extra_m": extra, "backoff_m": f["backoff_m"],
                                 "tilt_deg": t, "azimuth_deg": round(math.degrees(az), 2),
                                 "roll_deg": round(math.degrees(roll), 2), "flip": flip,
                                 "roll_actual_deg": round(math.degrees(math.atan2(ux @ v, ux @ u)), 2),
                                 "module_px": round(module_px(T_w_cam, K_ARM, lb), 3), "T_world_cam": f["T_world_cam"],
                                 "arm_q": {**f["q"], "gripper_left_joint": 0.0},
                                 "T_world_base_footprint": r["T_world_base_footprint"]})
            # 앞 D435
            tries = 0
            for k in range(args.d435_scenes):
                for _ in range(20000):
                    tries += 1
                    d = rng.uniform(*args.dist)
                    if lb["where"] == "wall":
                        phi = math.atan2(n[1], n[0]) + math.radians(rng.uniform(-80, 80))
                    else:
                        phi = rng.uniform(0, 2 * math.pi)
                    bearing = math.radians(rng.uniform(-15, 15))
                    pos = c[:2] + d * np.array([math.cos(phi), math.sin(phi)])
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
                    pu, pv, zc = project(T_w_cam, K, c)
                    if not (zc > 0 and 20 <= pu < W - 20 and 20 <= pv < H - 20):
                        continue
                    if (T_w_cam[:3, 3] - c) @ n <= 0 or sc.los_blocked(T_w_cam[:3, 3], c + n * 0.002, end_skip=0.0):
                        continue
                    plan.append({"kind": "d435", "id": f"{lb['id']}_d{k}", "label": lb["id"], "world": str(wdir),
                                 "seed": int(lay["seed"]), "dist_nominal": round(float(d), 4),
                                 "bearing_deg": round(math.degrees(bearing), 2),
                                 "incidence_deg": round(math.degrees(math.acos(np.clip(
                                     (T_w_cam[:3, 3] - c) @ n / np.linalg.norm(T_w_cam[:3, 3] - c), -1, 1))), 2),
                                 "module_px": round(module_px(T_w_cam, K, lb), 3),
                                 "T_world_base_footprint": T_w_bf.tolist(), "T_world_cam": T_w_cam.tolist(),
                                 "min_clearance_m": round(sc.clearance(ring), 4)})
                    break
                else:
                    raise RuntimeError(f"{lb['id']} D435 장면을 못 뽑음")
            labels.append(lb)
else:   # 계획 파일에서 (여러 월드를 따로 계획해 합칠 때) — 팔 화소 크기는 이번 K 로 다시 계산
    for f in args.load_plan:
        d = json.loads(Path(f).read_text())
        plan += d["plan"]
        labels += d["labels"]
        unreachable += d["unreachable"]
    lb_by = {lb["id"]: lb for lb in labels}
    for p in plan:
        if p["kind"] == "arm":
            p["module_px"] = round(module_px(np.array(p["T_world_cam"]), K_ARM, lb_by[p["label"]]), 3)
if args.save_plan:
    Path(args.save_plan).write_text(json.dumps({"plan": plan, "labels": labels, "unreachable": unreachable}, ensure_ascii=False))
if plan and not any(Path(p["world"]).resolve() == w.resolve() for p in plan for w in args.worlds):
    raise RuntimeError("계획과 --worlds 경로가 일치하는 장면이 없음")
print(f"라벨 {len(labels)}개 (벽 {sum(lb['where'] == 'wall' for lb in labels)}, 선반 윗면 "
      f"{sum(lb['where'] == 'shelf_top' for lb in labels)}, 옮김 {sum(lb['source'] == 'moved' for lb in labels)}), "
      f"장면 {len(plan)}개 (팔 {sum(p['kind'] == 'arm' for p in plan)}, D435 {sum(p['kind'] == 'd435' for p in plan)})",
      flush=True)
if args.only_plan:
    for lb in labels:
        print(lb["id"], lb["text"], lb["source"], [round(x, 3) for x in lb["center"]], "치운 퍽", lb["hide_pucks"])
    for p in plan:
        print(p["id"], "module_px", p["module_px"], "거리", p.get("dist_nominal"), "d_min", p.get("d_min"))
    print("실제 팔 자세 없음", unreachable)
    sys.exit(0)

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
    w, h = Kc["w"], Kc["h"]
    focal = Kc["fx"] * ha / w
    cam.CreateHorizontalApertureAttr(ha)
    cam.CreateVerticalApertureAttr(focal * h / Kc["fy"])
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureOffsetAttr(-(Kc["cx"] - w / 2) / w * ha)
    cam.CreateVerticalApertureOffsetAttr((Kc["cy"] - h / 2) / w * ha)
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


def visible_fraction(pts, T_w_cam, Kc, dep, tol=0.003):
    Ti = np.linalg.inv(T_w_cam)
    q = (Ti[:3, :3] @ pts.T).T + Ti[:3, 3]
    vis = 0
    for x, y, z in q:
        if z <= 0:
            continue
        u, v = Kc["fx"] * x / z + Kc["cx"], Kc["fy"] * y / z + Kc["cy"]
        if 0 <= u < Kc["w"] and 0 <= v < Kc["h"] and np.isfinite(dep[int(v), int(u)]) and dep[int(v), int(u)] > z - tol:
            vis += 1
    return vis / len(pts)


def label_std(rgb, T_w_cam, Kc, lb):
    """라벨 안쪽(가로 · 세로 70 %) 화소 밝기 표준편차."""
    F = label_frame(lb)
    bw, bh = lb["size"]
    quad = [project(T_w_cam, Kc, F[:3, 3] + F[:3, 0] * a * bw * 0.35 + F[:3, 1] * b * bh * 0.35)[:2]
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    mask = np.zeros((Kc["h"], Kc["w"]), np.uint8)
    cv2.fillConvexPoly(mask, np.round(np.array(quad)).astype(np.int32), 1)
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)[mask > 0]
    return round(float(g.std()), 1) if g.size else 0.0


def place_label(stage, lay, lb, restore=False):
    """옮긴 라벨: prim 의 메시(월드 좌표)를 원래 자리 → 새 자리로 옮기는 변환. restore 면 원래대로."""
    if lb["source"] != "moved":
        return
    prim = stage.GetPrimAtPath(lb["prim"])
    if restore:
        set_xform(prim, np.eye(4))
    else:
        src = label_frame({"normal": lb["from_normal"], "u": lb["from_u"], "center": lb["from"]})
        set_xform(prim, label_frame(lb) @ np.linalg.inv(src))
    for p in lay["pucks"]:
        if p["name"] in lb["hide_pucks"]:
            pr = stage.GetPrimAtPath(f"/World/Pucks/{p['name']}")
            pr.GetAttribute("xformOp:translate").Set(Gf.Vec3d(p["center"][0], p["center"][1],
                                                              p["center"][2] if restore else -1.0))


report = {"mission": args.mission, "urdf": str(args.urdf), "robot_usd": None, "unreachable": unreachable, "width": W, "height": H, "lights": args.lights, "seed": args.seed, "arm_q": S0,
          "d435": {"frame": args.frame, "K": K_D}, "arm": {"frame": args.arm_frame, "K": K_ARM},
          "labels": labels, "poses": []}
t0 = time.time()
for wdir in args.worlds:
    lay = json.loads((wdir / "layout.json").read_text())
    ctx.open_stage(str((wdir / "task_world.usda").resolve()))
    updates(60)
    stage = ctx.get_stage()
    report["robot_usd"] = use_robot_usd(stage)
    updates(10)
    for kind, Kc in (("arm", K_ARM), ("d435", K_D)):   # 해상도가 달라 카메라 종류마다 render product 를 따로 만든다
        scenes = [p for p in plan if Path(p["world"]).resolve() == wdir.resolve() and p["kind"] == kind]
        if not scenes:   # --d435-scenes 0 처럼 한 종류만 렌더할 때
            continue
        cam_path = f"/World/RenderR3/{kind}"
        near = args.near if kind == "arm" else 0.01
        set_camera(stage, cam_path, np.array(scenes[0]["T_world_cam"]), Kc, near)
        rp = rep.create.render_product(cam_path, (Kc["w"], Kc["h"]))
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach(rp)
        a_dep.attach(rp)
        for level in args.lights:
            set_lighting(stage, level)
            for lb in [x for x in labels if Path(x["world"]).resolve() == wdir.resolve()]:
                place_label(stage, lay, lb)
                updates(10)
                vis_pts = label_points(lb)
                for pose in [p for p in scenes if p["label"] == lb["id"]]:
                    T_w_cam = np.array(pose["T_world_cam"])
                    set_camera(stage, cam_path, T_w_cam, Kc, near)
                    pose_robot(stage, args.urdf, np.array(pose["T_world_base_footprint"]),
                               S0 if kind == "d435" else pose["arm_q"])
                    tries = []
                    for k in range(3):
                        updates(4 + 10 * k)
                        rgb, dep = grab(a_rgb, a_dep)
                        tries.append([round(visible_fraction(vis_pts, T_w_cam, Kc, dep,
                                                             tol=0.003 if kind == "arm" else 0.004), 3),
                                      label_std(rgb, T_w_cam, Kc, lb) if kind == "arm" else None])
                        if kind == "d435" or tries[-1][1] >= 30:
                            break
                    name = f"{pose['id']}_L{level:g}"
                    (out / name).mkdir()
                    cv2.imwrite(str(out / name / "frame.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    images = {"color": f"{name}/frame.png"}
                    if kind == "d435":
                        np.save(out / name / "frame_depth.npy", dep)
                        images["depth"] = f"{name}/frame_depth.npy"
                    pu, pv, zc = project(T_w_cam, Kc, lb["center"])
                    entry = dict(pose)
                    entry.update({"name": name, "light_level": level, "images": images, "text": lb["text"],
                                  "where": lb["where"], "label_center": lb["center"],
                                  "label_px": [round(float(pu), 1), round(float(pv), 1)],
                                  "cam_dist_m": round(float(np.linalg.norm(np.array(lb["center"]) - T_w_cam[:3, 3])), 4),
                                  "base_hdist_m": round(float(np.hypot(*(np.array(lb["center"][:2]) -
                                                                         np.array(pose["T_world_base_footprint"])[:2, 3]))), 4),
                                  "label_visible": tries[-1][0], "render_check": {"tries": tries}})
                    report["poses"].append(entry)
                    print(name, lb["text"], "module_px", pose["module_px"], "보임", tries[-1][0], "확인", tries,
                          flush=True)
                place_label(stage, lay, lb, restore=True)
                updates(4)
        a_rgb.detach()
        a_dep.detach()
        rp.destroy()
        updates(4)

report["total_s"] = round(time.time() - t0, 1)
(out / "render_r3.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))
print(json.dumps({"poses": len(report["poses"]), "total_s": report["total_s"]}, ensure_ascii=False))
app.close()
