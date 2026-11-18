"""가상 과제 월드 판정 W1~W6 (모의·USD 정적 검사).

W1 만들기 종료 코드 0, 새 프로세스에서 다시 열 때 합성 오류 0·참조 파일 못 찾음 0
W2 미로 충돌체가 위쪽 반 + 가운데 벽만 (중심 y < 0 인 미로 충돌체 0개)
W3 퍽 12개: 색마다 4개, 색마다 두 모양 모두, 번호 0~11 중복 없음, 모두 선반 위
   (퍽 몸통 아랫면 - 선반 윗면 |dz| <= 1 mm, 몸통 바닥 범위가 선반 윗면 안)
W4 출발 로봇 충돌 형상과 벽·선반·퍽의 겹침 0쌍 (seed 3개)
W5 같은 seed 두 번 -> task_world.usda, layout.json 바이트 동일
W6 Aruco 마커(한 변 0.10 m ±0.5 mm, 중심 높이 0.15 m ±1 mm, 벽판 면에서 2 mm ±0.5 mm, 판 안, 출발 칸 쪽)
   출발 칸 테이프(바깥 0.60 m ±1 mm, 폭 0.03 m ±0.5 mm, 바닥 위, 왼쪽 아래 구석 칸(c00) 가운데 ±1 mm, 벽과 안 겹침)

실행 (PC): ~/isaacsim-venv/bin/python -I check_task_world.py --seeds 1 2 3
결과: results/check_<YYMMDD_HHMM>/check_result.json, check.log, seed_<n>_top.png
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PY = sys.executable


# ---------------- 새 프로세스에서 USD 읽기 ----------------
def inspect(usd_path):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: F401

    stage = Usd.Stage.Open(str(usd_path))
    out = {"composition_errors": [str(e) for e in stage.GetCompositionErrors()], "unresolved_assets": []}
    layer_stack = [usd_path]
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                v = attr.Get()
                if v and v.path and not v.resolvedPath:
                    out["unresolved_assets"].append(f"{attr.GetPath()} -> {v.path}")
    robot_geo = stage.GetPrimAtPath("/World/waffle_4wd/Geometry/base_footprint/base_link")
    out["robot_composed"] = bool(robot_geo.IsValid()) and bool(stage.GetPrimAtPath("/World/waffle_4wd/Physics").IsValid())
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def aabb(prim):
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        return [list(r.GetMin()), list(r.GetMax())]

    out["maze"] = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Maze")):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            out["maze"].append({"path": str(prim.GetPath()), "aabb": aabb(prim)})
    out["shelves"] = [{"name": p.GetName(), "aabb": aabb(p), "collide": p.HasAPI(UsdPhysics.CollisionAPI)}
                      for p in stage.GetPrimAtPath("/World/Shelves").GetChildren()]
    def exact_aabb(body):
        """몸통의 정확한 월드 범위. Cube는 꼭짓점 8개, Cylinder는 축 방향으로 계산
        (BBoxCache는 원기둥을 사각 범위로 감싸서 누운 원기둥 아랫면이 최대 6 mm 낮게 나온다)."""
        xf = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        if body.IsA(UsdGeom.Cylinder):
            cyl = UsdGeom.Cylinder(body)
            r, h = cyl.GetRadiusAttr().Get(), cyl.GetHeightAttr().Get()
            c = np.array(xf.Transform((0, 0, 0)))
            a = np.array(xf.TransformDir((0, 0, 1)))
            a = a / np.linalg.norm(a)
            ext = h / 2 * np.abs(a) + r * np.sqrt(np.clip(1 - a * a, 0, None))
            return [list(c - ext), list(c + ext)]
        pts = np.array([list(xf.Transform((x, y, z))) for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
        return [list(pts.min(0)), list(pts.max(0))]

    out["pucks"] = []
    for p in stage.GetPrimAtPath("/World/Pucks").GetChildren():
        body = p.GetChild("body")
        tags = [c.GetName() for c in p.GetChildren() if c.GetName().startswith("tag_")]
        out["pucks"].append({"name": p.GetName(), "color": p.GetAttribute("puck:color").Get(),
                             "shape": p.GetAttribute("puck:shape").Get(), "tag_id": p.GetAttribute("puck:tag_id").Get(),
                             "rigid": p.HasAPI(UsdPhysics.RigidBodyAPI), "body_type": body.GetTypeName(),
                             "body_collide": body.HasAPI(UsdPhysics.CollisionAPI), "body_aabb": exact_aabb(body),
                             "tags": tags})
    mk = [c for c in stage.GetPrimAtPath("/World/TaskMarker").GetChildren()]
    out["markers"] = []
    for m in mk:
        xf = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pts = [list(xf.Transform(p)) for p in UsdGeom.Mesh(m).GetPointsAttr().Get()]
        out["markers"].append({"name": m.GetName(), "id": m.GetAttribute("marker:id").Get(),
                               "panel": m.GetAttribute("marker:panel").Get(), "size_attr": m.GetAttribute("marker:size").Get(),
                               "points": pts})
    out["tapes"] = [{"name": p.GetName(), "aabb": aabb(p), "collide": p.HasAPI(UsdPhysics.CollisionAPI)}
                    for p in stage.GetPrimAtPath("/World/StartBox").GetChildren()]
    rp = stage.GetPrimAtPath("/World/waffle_4wd")
    ops = {op.GetOpName(): op.Get() for op in UsdGeom.Xformable(rp).GetOrderedXformOps()}
    t = ops.get("xformOp:translate")
    out["robot"] = {"translate": list(t) if t is not None else None, "rotateZ_deg": ops.get("xformOp:rotateZ"),
                    "joints": {}}
    for jp in stage.GetPrimAtPath("/World/waffle_4wd/Physics").GetChildren():
        for kind in ("angular", "linear"):
            a = jp.GetAttribute(f"state:{kind}:physics:position")
            if a and a.HasAuthoredValue():
                out["robot"]["joints"][jp.GetName()] = {"kind": kind, "value": a.Get()}
    return out


# ---------------- 판정 ----------------
def overlap(a, b, eps=1e-9):
    return all(a[0][k] < b[1][k] - eps and b[0][k] < a[1][k] - eps for k in range(3))


def judge_w2(info, layout):
    centers = [((np.array(m["aabb"][0]) + np.array(m["aabb"][1])) / 2) for m in info["maze"]]
    neg = [m["path"] for m, c in zip(info["maze"], centers) if c[1] < -1e-6]
    n_panel = sum(1 for m in info["maze"] if "/panel_" in m["path"])
    n_post = len({m["path"].rsplit("/", 1)[0] for m in info["maze"] if "/post_" in m["path"]})
    exp_panel, exp_post = len(layout["panels"]), len(layout["posts"])
    ok = not neg and n_panel == exp_panel and n_post == exp_post
    return ok, {"maze_colliders": len(info["maze"]), "center_y_negative": neg, "panels": n_panel,
                "panels_expected": exp_panel, "posts": n_post, "posts_expected": exp_post,
                "min_y": min(m["aabb"][0][1] for m in info["maze"]), "max_y": max(m["aabb"][1][1] for m in info["maze"])}


def judge_w3(info):
    pk = info["pucks"]
    shelves = {s["name"]: s for s in info["shelves"]}
    rows, fails = [], []
    for p in pk:
        lo, hi = np.array(p["body_aabb"][0]), np.array(p["body_aabb"][1])
        c = (lo + hi) / 2
        on = [s for s in shelves.values() if s["aabb"][0][0] <= c[0] <= s["aabb"][1][0]
              and s["aabb"][0][1] <= c[1] <= s["aabb"][1][1]]
        if len(on) != 1:
            fails.append(f"{p['name']}: 선반 {len(on)}개 위")
            continue
        s = on[0]
        dz = float(lo[2] - s["aabb"][1][2])
        inside = bool(lo[0] >= s["aabb"][0][0] and hi[0] <= s["aabb"][1][0] and lo[1] >= s["aabb"][0][1]
                      and hi[1] <= s["aabb"][1][1])
        rows.append({"name": p["name"], "shelf": s["name"], "dz_mm": round(dz * 1000, 4), "inside_top": inside,
                     "color": p["color"], "shape": p["shape"], "tag_id": p["tag_id"], "tags": len(p["tags"])})
        if abs(dz) > 0.001 or not inside:
            fails.append(f"{p['name']}: dz {dz * 1000:.3f} mm, 윗면 안 {inside}")
    per_color = {c: sorted(p["shape"] for p in pk if p["color"] == c) for c in ("red", "green", "blue")}
    ids = sorted(p["tag_id"] for p in pk)
    if len(pk) != 12:
        fails.append(f"퍽 {len(pk)}개")
    for c, shapes in per_color.items():
        if len(shapes) != 4 or set(shapes) != {"cuboid", "cylinder"}:
            fails.append(f"{c}: {shapes}")
    if ids != list(range(12)):
        fails.append(f"번호 {ids}")
    for p in pk:
        if not (p["rigid"] and p["body_collide"]):
            fails.append(f"{p['name']}: 강체 {p['rigid']}, 충돌 {p['body_collide']}")
    return not fails, {"fails": fails, "per_color": per_color, "tag_ids": ids,
                       "max_abs_dz_mm": max((abs(r["dz_mm"]) for r in rows), default=None), "pucks": rows}


def judge_w4(info, robot):
    from robot_geom import box_depth
    r = info["robot"]
    yaw = math.radians(r["rotateZ_deg"])
    T = np.eye(4)
    T[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    T[:3, 3] = r["translate"]
    q = {}
    for name, j in r["joints"].items():
        q[name] = math.radians(j["value"]) if j["kind"] == "angular" else j["value"]
    q = {k: v for k, v in q.items() if not robot.joint_by_name[k]["mimic"]}
    ws = robot.world_shapes(q, T)
    boxes = [(m["path"], m["aabb"]) for m in info["maze"]]
    boxes += [("/World/Shelves/" + s["name"], s["aabb"]) for s in info["shelves"]]
    boxes += [("/World/Pucks/" + p["name"], p["body_aabb"]) for p in info["pucks"]]
    hits, closest = [], None
    for name, bb in boxes:
        lo, hi = np.array(bb[0]), np.array(bb[1])
        c, h = (lo + hi) / 2, (hi - lo) / 2
        for link, V in ws:
            d = box_depth(V, c, h)
            if closest is None or d > closest[2]:
                closest = (link, name, d)
            if d >= 0:
                hits.append({"link": link, "object": name, "depth_mm": round(d * 1000, 3)})
    allv = np.vstack([V for _, V in ws])
    return not hits, {"pairs": hits, "closest": {"link": closest[0], "object": closest[1],
                                                 "gap_linf_mm": round(-closest[2] * 1000, 3)},
                      "robot_xy_hull_bounds": [allv[:, :2].min(0).round(4).tolist(), allv[:, :2].max(0).round(4).tolist()],
                      "q": q, "yaw_deg": r["rotateZ_deg"]}


def judge_w6(info, lay, layout):
    fails = []
    mk = info["markers"]
    if len(mk) != 1:
        return False, {"fails": [f"마커 {len(mk)}개"]}
    m = mk[0]
    P = np.array(m["points"])
    w = float(np.linalg.norm(P[1] - P[0]))
    hgt = float(np.linalg.norm(P[3] - P[0]))
    size = w * 6 / 8
    center = P.mean(0)
    nrm = np.cross(P[1] - P[0], P[3] - P[0])
    nrm /= np.linalg.norm(nrm)
    panel = next(x for x in info["maze"] if x["path"].endswith(f"/panel_{m['panel']:02d}"))
    plo, phi = np.array(panel["aabb"][0]), np.array(panel["aabb"][1])
    pc = (plo + phi) / 2
    axis = int(np.argmax(np.abs(nrm)))      # 판 두께 방향
    offset = float((center - pc)[axis] * np.sign(nrm[axis]))
    lateral = [k for k in (0, 1) if k != axis][0]
    inside = bool(P[:, lateral].min() >= plo[lateral] and P[:, lateral].max() <= phi[lateral]
                  and P[:, 2].min() >= plo[2] and P[:, 2].max() <= phi[2])
    sb = lay["start_box"]
    toward = float(np.dot(np.array(sb["center"] + [0.0])[:2] - center[:2], nrm[:2]))
    exp_id = lay["assumptions"]["task_marker_id"][lay["task"]]
    checks = {"id": (m["id"] == exp_id, m["id"]), "size_m": (abs(size - 0.10) <= 0.0005, round(size, 6)),
              "square": (abs(w - hgt) <= 1e-6, round(hgt, 6)),
              "center_z_m": (abs(center[2] - 0.15) <= 0.001, round(float(center[2]), 6)),
              "offset_from_panel_center_m": (abs(offset - 0.002) <= 0.0005, round(offset, 6)),
              "inside_panel": (inside, inside), "faces_start_cell": (toward > 0, round(toward, 4))}
    # 테이프
    tp = info["tapes"]
    lo = np.min([t["aabb"][0] for t in tp], 0)
    hi = np.max([t["aabb"][1] for t in tp], 0)
    widths = sorted(round(float(min(np.array(t["aabb"][1][:2]) - np.array(t["aabb"][0][:2]))), 6) for t in tp)
    # 출발 칸 = 왼쪽 아래 구석 칸(사용자 지정): x 첫 두 기둥 줄 사이, y 0 ~ 0.788 (배치 데이터 격자에서 직접 계산)
    xs = layout["grid"]["x"]
    cell = {"id": "c00", "center": [(xs[0] + xs[1]) / 2, 0.788 / 2]}
    tcen = (lo + hi) / 2
    tape_hit = [x["path"] for t in tp for x in info["maze"] if overlap(t["aabb"], x["aabb"])]
    checks.update({
        "tape_count": (len(tp) == 4, len(tp)),
        "tape_outer_m": (abs(hi[0] - lo[0] - 0.60) <= 0.001 and abs(hi[1] - lo[1] - 0.60) <= 0.001,
                         [round(float(hi[0] - lo[0]), 6), round(float(hi[1] - lo[1]), 6)]),
        "tape_width_m": (all(abs(x - 0.03) <= 0.0005 for x in widths), widths),
        "tape_on_floor": (abs(lo[2]) <= 1e-6 and hi[2] <= 0.0011, [round(float(lo[2]), 7), round(float(hi[2]), 7)]),
        "start_cell_id": (sb["cell"] == "c00", sb["cell"]),
        "tape_center_vs_cell_m": (float(np.abs(tcen[:2] - np.array(cell["center"])).max()) <= 0.001,
                                  round(float(np.abs(tcen[:2] - np.array(cell["center"])).max()), 6)),
        "tape_no_collision": (not any(t["collide"] for t in tp), [t["collide"] for t in tp]),
        "tape_wall_overlap": (not tape_hit, tape_hit)})
    fails = [k for k, (ok, _) in checks.items() if not ok]
    return not fails, {"fails": fails, "values": {k: v for k, (_, v) in checks.items()}}


def draw_top(info, layout, robot_hull_xy, path):
    from PIL import Image, ImageDraw
    xs, ys = layout["grid"]["x"], layout["grid"]["y"]
    sc, pad = 200, 0.15
    x0, y1 = xs[0] - pad, ys[-1] + pad
    W, H = int((xs[-1] - xs[0] + 2 * pad) * sc), int((ys[-1] - ys[0] + 2 * pad) * sc) + 30
    im = Image.new("RGB", (W, H), (235, 235, 235))
    g = ImageDraw.Draw(im)

    def T(x, y):
        return (int((x - x0) * sc), int((y1 - y) * sc))

    def rect(bb, fill, outline=None):
        a, b = T(bb[0][0], bb[1][1]), T(bb[1][0], bb[0][1])
        g.rectangle([a, b], fill=fill, outline=outline)

    for t in info["tapes"]:
        rect(t["aabb"], (255, 255, 255), (150, 150, 150))
    for m in info["maze"]:
        if m["path"].endswith("/head"):
            continue
        rect(m["aabb"], (60, 70, 100) if "/panel_" in m["path"] else (40, 60, 200))
    al = {f"/World/Maze/post_{p['id']:02d}/body" for p in layout["posts"] if p["type"] == "al"}
    for m in info["maze"]:
        if m["path"] in al:
            rect(m["aabb"], (170, 170, 175), (0, 0, 0))
    for s in info["shelves"]:
        rect(s["aabb"], (150, 150, 150), (0, 0, 0))
    col = {"red": (200, 30, 30), "green": (30, 150, 50), "blue": (30, 70, 210)}
    for p in info["pucks"]:
        rect(p["body_aabb"], col[p["color"]])
    for m in info["markers"]:
        P = np.array(m["points"])
        g.line([T(*P[0][:2]), T(*P[1][:2])], fill=(0, 0, 0), width=5)
    if robot_hull_xy is not None:
        g.polygon([T(*p) for p in robot_hull_xy], outline=(220, 120, 0))
    g.text((5, H - 25), "top view: +x right, +y up. center wall y=0 (bottom). orange = robot outline, black = marker",
           fill=(0, 0, 0))
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--task", default="A")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--inspect", type=Path, help="(내부용) USD를 읽어 JSON 출력")
    args = ap.parse_args()
    if args.inspect:
        print(json.dumps(inspect(args.inspect)))
        return

    sys.path.insert(0, str(HERE))
    from robot_geom import Robot
    from scipy.spatial import ConvexHull
    import build_task_world as B

    out = args.out or HERE / f"results/check_{time.strftime('%y%m%d_%H%M')}"
    out.mkdir(parents=True, exist_ok=False)
    log = open(out / "check.log", "w")

    def say(*a):
        s = " ".join(str(x) for x in a)
        print(s)
        log.write(s + "\n")

    layout = json.loads((HERE / "data/maze_upper_drawing.json").read_text())
    robot = Robot(B.URDF_PKG / "urdf/waffle_4wd.urdf", B.URDF_PKG)
    result = {"seeds": args.seeds, "task": args.task, "W": {}}
    per = {}
    for seed in args.seeds:
        d = out / f"seed_{seed}"
        b = subprocess.run([PY, "-I", str(HERE / "build_task_world.py"), "--seed", str(seed), "--task", args.task,
                            "--out", str(d)], capture_output=True, text=True)
        (out / f"seed_{seed}_build.log").write_text(b.stdout + b.stderr)
        r = subprocess.run([PY, "-I", __file__, "--inspect", str(d / "task_world.usda")], capture_output=True,
                           text=True)
        (out / f"seed_{seed}_inspect.err").write_text(r.stderr)
        info = json.loads(r.stdout.strip().splitlines()[-1]) if r.returncode == 0 else None
        lay = json.loads((d / "layout.json").read_text()) if b.returncode == 0 else None
        per[seed] = {"build_rc": b.returncode, "inspect_rc": r.returncode, "info": info, "layout": lay}

    def record(key, ok, detail):
        result["W"].setdefault(key, {"pass": True, "by_seed": {}})
        seed = str(detail.pop("_seed"))
        result["W"][key]["by_seed"][seed] = {"pass": ok, **detail}
        result["W"][key]["pass"] &= ok

    for seed, s in per.items():
        info, lay = s["info"], s["layout"]
        w1 = s["build_rc"] == 0 and s["inspect_rc"] == 0 and info is not None and not info["composition_errors"] \
            and not info["unresolved_assets"] and info["robot_composed"]
        record("W1", w1, {"_seed": seed, "build_rc": s["build_rc"], "reopen_rc": s["inspect_rc"],
                          "composition_errors": info["composition_errors"] if info else None,
                          "unresolved_assets": info["unresolved_assets"] if info else None,
                          "robot_composed": info["robot_composed"] if info else None})
        if not w1:
            continue
        ok, det = judge_w2(info, layout)
        record("W2", ok, {"_seed": seed, **det})
        ok, det = judge_w3(info)
        record("W3", ok, {"_seed": seed, **det})
        ok, w4 = judge_w4(info, robot)
        q4 = w4["q"]
        record("W4", ok, {"_seed": seed, **w4})
        ok, det = judge_w6(info, lay, layout)
        record("W6", ok, {"_seed": seed, **det})
        T = np.eye(4)
        yaw = math.radians(info["robot"]["rotateZ_deg"])
        T[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        T[:3, 3] = info["robot"]["translate"]
        allv = np.vstack([V for _, V in robot.world_shapes(q4, T)])
        hull = allv[:, :2][ConvexHull(allv[:, :2]).vertices]
        draw_top(info, layout, hull, out / f"seed_{seed}_top.png")

    # W5: 첫 seed를 한 번 더 만들어 바이트 비교
    s0 = args.seeds[0]
    d2 = out / f"seed_{s0}_again"
    b2 = subprocess.run([PY, "-I", str(HERE / "build_task_world.py"), "--seed", str(s0), "--task", args.task,
                         "--out", str(d2)], capture_output=True, text=True)
    same = {}
    for fn in ("task_world.usda", "layout.json"):
        a, c = (out / f"seed_{s0}" / fn), (d2 / fn)
        same[fn] = b2.returncode == 0 and a.read_bytes() == c.read_bytes()
    record("W5", all(same.values()), {"_seed": s0, "build_rc": b2.returncode, "identical": same})

    result["all_pass"] = all(v["pass"] for v in result["W"].values()) and len(result["W"]) == 6
    (out / "check_result.json").write_text(json.dumps(result, indent=1, ensure_ascii=False, default=float))
    for k in sorted(result["W"]):
        v = result["W"][k]
        say(f"{k}: {'성공' if v['pass'] else '실패'}  " + ", ".join(
            f"seed {sd}={'성공' if x['pass'] else '실패'}" for sd, x in v["by_seed"].items()))
    say("전체:", "성공" if result["all_pass"] else "실패", "| 결과", out)
    log.close()
    sys.exit(0 if result["all_pass"] else 1)


if __name__ == "__main__":
    main()
