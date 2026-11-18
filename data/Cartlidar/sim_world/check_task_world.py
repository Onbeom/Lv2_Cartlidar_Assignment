"""가상 과제 월드 판정 W1~W7 v2 (최종 과제 B조 주사위 판, 사용자 확정 2026-10-23). 모의 · USD 정적 검사.

W1 만들기 종료 코드 0, 새 프로세스에서 다시 열 때 합성 오류 0 · 참조 파일 못 찾음 0
W2 미로 충돌체가 위쪽 반 + 가운데 벽만, 판 · 기둥 수 = 배치 데이터. 기둥 몸통 알루미늄 30 x 30 ±0.5 mm,
   출력 36 x 9.5 ±0.5 mm(긴 변이 이어진 벽판 방향)
W3 퍽 12개 빨강 · 초록만, 번호 0~11 중복 없음, 점멸 순서와 맞음(빨강 · 초록 · 노랑 주사위용 퍽 수 = 횟수, 노랑은 모양 하나,
   각 1~6, 합 12), 라벨 윗면 · 아랫면 2장 20 x 20 ±0.2 mm. 선반 퍽: 아랫면 - 선반 윗면 |dz| <= 1 mm, 바닥 범위가 윗면 안,
   선반 바코드 자리와 안 겹침. 바닥 퍽 0~3개: 아랫면 |z| <= 1 mm, 자기 선반과 같은 칸, 벽 · 기둥 · 선반 · 다른 퍽과 안 겹침
W4 출발 로봇 충돌 형상과 벽 · 선반 · 퍽 · 신호등 겹침 0쌍 (seed 3개)
W5 같은 seed 두 번 -> task_world.usda, layout.json 바이트 동일
W6 장소 태그 3개(12 · 13 · 14, 선반마다 하나): 흰 여백 포함 0.060 ±0.5 mm, 자기 선반 뒤 벽판 면에서 0.5 ±0.5 mm, 판 안,
   중심 높이 0.13~0.30, 칸 안쪽(선반 쪽)을 향함. 바코드 1~4개: 35 x 20 ±0.5 mm, 벽판 면(판 안, 중심 높이 0.10~0.30)
   또는 선반 윗면(윗면 위 0.5 ±0.5 mm, 윗면 안) — 바닥 · 선반 옆면 없음. 출발 칸 테이프(이전과 같음)
W7 신호등 1개: 축 세로, 치수가 그림과 ±0.5 mm(전체 176.9, 머리 지름 39.6, 칸 34.9, 아래부터 초록 · 노랑 · 빨강),
   벽 · 기둥 · 선반 · 퍽과 안 겹침, 머리와 벽판 사이 0.02 ±0.001 m, 출발 칸 · 선반 칸 아님, 켜짐 · 꺼짐 · 텀 범위 안

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

    def pts_world(m):
        xf = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return [list(xf.Transform(p)) for p in UsdGeom.Mesh(m).GetPointsAttr().Get()]

    out["pucks"] = []
    for p in stage.GetPrimAtPath("/World/Pucks").GetChildren():
        body = p.GetChild("body")
        xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        inv = xf.GetInverse()
        labels = []
        for c in p.GetChildren():
            if c.GetName().startswith("label_"):
                P = np.array(pts_world(c))
                loc = np.array([list(inv.Transform(tuple(q))) for q in P])
                labels.append({"name": c.GetName(), "points": P.tolist(), "local": loc.tolist()})
        half_h = (UsdGeom.Cylinder(body).GetHeightAttr().Get() / 2 if body.IsA(UsdGeom.Cylinder)
                  else float(UsdGeom.Xformable(body).GetOrderedXformOps()[1].Get()[2]) / 2)
        out["pucks"].append({"name": p.GetName(), "color": p.GetAttribute("puck:color").Get(),
                             "shape": p.GetAttribute("puck:shape").Get(), "tag_id": p.GetAttribute("puck:tag_id").Get(),
                             "location": p.GetAttribute("puck:location").Get(),
                             "dice_role": p.GetAttribute("puck:dice_role").Get(), "half_h": half_h,
                             "rigid": p.HasAPI(UsdPhysics.RigidBodyAPI), "body_type": body.GetTypeName(),
                             "body_collide": body.HasAPI(UsdPhysics.CollisionAPI), "body_aabb": exact_aabb(body),
                             "labels": labels})
    out["post_bodies"] = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Maze")):
        if prim.GetName() == "body":
            sc = UsdGeom.Xformable(prim).GetOrderedXformOps()[1].Get()
            out["post_bodies"].append({"path": str(prim.GetPath()), "scale": [float(v) for v in sc],
                                       "type": prim.GetParent().GetAttribute("maze:post_type").Get()})
    out["place_tags"] = [{"name": m.GetName(), "id": m.GetAttribute("tag:id").Get(), "points": pts_world(m)}
                         for m in stage.GetPrimAtPath("/World/PlaceTags").GetChildren()]
    out["barcodes"] = [{"name": m.GetName(), "text": m.GetAttribute("barcode:text").Get(), "points": pts_world(m)}
                       for m in stage.GetPrimAtPath("/World/Barcodes").GetChildren()]
    for sh in out["shelves"]:
        sh["place_tag_id"] = stage.GetPrimAtPath("/World/Shelves/" + sh["name"]).GetAttribute("shelf:place_tag_id").Get()
    lp = stage.GetPrimAtPath("/World/SignalLight")
    parts = {}
    for c in lp.GetChildren():
        cyl = UsdGeom.Cylinder(c)
        xf = UsdGeom.Xformable(c).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        ax = np.array(xf.TransformDir((0, 0, 1)))
        parts[c.GetName()] = {"radius": cyl.GetRadiusAttr().Get(), "height": cyl.GetHeightAttr().Get(),
                              "center": list(xf.Transform((0, 0, 0))), "axis": (ax / np.linalg.norm(ax)).tolist(),
                              "aabb": exact_aabb(c)}
    out["signal"] = {"parts": parts, "sequence": lp.GetAttribute("signal:sequence").Get(),
                     "yellow_shape": lp.GetAttribute("signal:yellow_shape").Get(),
                     "t_on": lp.GetAttribute("signal:t_on").Get(), "t_off": lp.GetAttribute("signal:t_off").Get(),
                     "t_term": lp.GetAttribute("signal:t_term").Get()}
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
    axes = {}
    for pn in layout["panels"]:
        for k in (pn["a"], pn["b"]):
            axes.setdefault(k, set()).add(pn["axis"])
    bad_post = []
    for pb in info["post_bodies"]:
        pid = int(pb["path"].split("post_")[1].split("/")[0])
        sx, sy = pb["scale"][:2]
        if pb["type"] == "al":
            good = abs(sx - 0.030) <= 0.0005 and abs(sy - 0.030) <= 0.0005
        else:
            long_x = sx > sy
            good = abs(max(sx, sy) - 0.036) <= 0.0005 and abs(min(sx, sy) - 0.0095) <= 0.0005 and \
                axes.get(pid) == ({"x"} if long_x else {"y"})
        if not good:
            bad_post.append({"post": pid, "type": pb["type"], "scale_xy": [round(sx, 6), round(sy, 6)],
                             "panel_axes": sorted(axes.get(pid, []))})
    ok = not neg and n_panel == exp_panel and n_post == exp_post and not bad_post
    return ok, {"maze_colliders": len(info["maze"]), "center_y_negative": neg, "post_body_bad": bad_post,
                "post_bodies": len(info["post_bodies"]), "panels": n_panel,
                "panels_expected": exp_panel, "posts": n_post, "posts_expected": exp_post,
                "min_y": min(m["aabb"][0][1] for m in info["maze"]), "max_y": max(m["aabb"][1][1] for m in info["maze"])}


def cell_of(lay, x, y):
    for c in lay["cells"]:
        if c["x"][0] <= x <= c["x"][1] and c["y"][0] <= y <= c["y"][1]:
            return c["id"]
    return None


def judge_w3(info, lay):
    pk = info["pucks"]
    shelves = {s["name"]: s for s in info["shelves"]}
    sig = info["signal"]
    fails, rows = [], []
    seq = sig["sequence"] or ""
    cnt = {k: seq.count(k) for k in "RGY"}
    if len(seq) != 12 or sum(cnt.values()) != 12 or not all(1 <= v <= 6 for v in cnt.values()):
        fails.append(f"점멸 순서 {seq} {cnt}")
    if len(pk) != 12:
        fails.append(f"퍽 {len(pk)}개")
    if sorted(p["tag_id"] for p in pk) != list(range(12)):
        fails.append(f"번호 {sorted(p['tag_id'] for p in pk)}")
    if any(p["color"] not in ("red", "green") for p in pk):
        fails.append("빨강 · 초록 밖의 색")
    role = {k: [p for p in pk if p["dice_role"] == k] for k in "RGY"}
    for k in "RGY":
        if len(role[k]) != cnt[k]:
            fails.append(f"{k} 주사위용 퍽 {len(role[k])}개, 점멸 {cnt[k]}회")
    if any(p["color"] != "red" for p in role["R"]) or any(p["color"] != "green" for p in role["G"]):
        fails.append("빨강 · 초록 주사위용 퍽 색이 다름")
    if len({p["shape"] for p in role["Y"]}) > 1 or any(p["shape"] != sig["yellow_shape"] for p in role["Y"]):
        fails.append(f"노랑 주사위용 퍽 모양 {[p['shape'] for p in role['Y']]} (기대 {sig['yellow_shape']})")
    layp = {p["name"]: p for p in lay["pucks"]}
    shelf_bar = {}
    for b in info["barcodes"]:
        P = np.array(b["points"])
        if abs(P[:, 2].max() - P[:, 2].min()) < 1e-6:   # 누운 바코드 = 선반 윗면
            shelf_bar[b["name"]] = (P.min(0), P.max(0))
    obstacles = [(m["path"], m["aabb"]) for m in info["maze"]] + [(s["name"], s["aabb"]) for s in info["shelves"]]
    n_floor = 0
    for p in pk:
        lo, hi = np.array(p["body_aabb"][0]), np.array(p["body_aabb"][1])
        r = {"name": p["name"], "location": p["location"], "color": p["color"], "shape": p["shape"],
             "tag_id": p["tag_id"], "role": p["dice_role"]}
        # 라벨: 윗면 · 아랫면 2장, 20 x 20 ±0.2 mm, 퍽 축(로컬 z) 끝면 위(±0.5 mm)
        names = sorted(l["name"] for l in p["labels"])
        if names != ["label_bottom", "label_top"]:
            fails.append(f"{p['name']}: 라벨 {names}")
        for l in p["labels"]:
            L = np.array(l["local"])
            e1, e2 = np.linalg.norm(L[1] - L[0]), np.linalg.norm(L[3] - L[0])
            zc = float(L[:, 2].mean())
            if abs(e1 - 0.020) > 0.0002 or abs(e2 - 0.020) > 0.0002 or np.ptp(L[:, 2]) > 1e-6 or \
                    abs(abs(zc) - p["half_h"]) > 0.0005 or np.abs(L[:, :2].mean(0)).max() > 1e-6:
                fails.append(f"{p['name']} {l['name']}: 변 {e1 * 1000:.3f}/{e2 * 1000:.3f} mm, 면에서 {abs(zc) - p['half_h']:.5f} m")
        if p["location"] == "floor":
            n_floor += 1
            home = layp[p["name"]]["home_shelf"]
            hc = shelves[home]["aabb"]
            hcell = cell_of(lay, (hc[0][0] + hc[1][0]) / 2, (hc[0][1] + hc[1][1]) / 2)
            pcell = cell_of(lay, (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)
            hit = [n for n, bb in obstacles if overlap([lo, hi], bb)]
            hit += [q["name"] for q in pk if q is not p and overlap([lo, hi], q["body_aabb"])]
            r.update({"z_mm": round(float(lo[2]) * 1000, 4), "cell": pcell, "home_cell": hcell, "overlap": hit})
            if abs(lo[2]) > 0.001 or pcell != hcell or hit:
                fails.append(f"{p['name']}: 바닥 z {lo[2] * 1000:.3f} mm, 칸 {pcell}/{hcell}, 겹침 {hit}")
        else:
            s = shelves.get(p["location"])
            if s is None:
                fails.append(f"{p['name']}: 선반 {p['location']} 없음")
                continue
            dz = float(lo[2] - s["aabb"][1][2])
            inside = bool(lo[0] >= s["aabb"][0][0] and hi[0] <= s["aabb"][1][0] and lo[1] >= s["aabb"][0][1]
                          and hi[1] <= s["aabb"][1][1])
            on_bar = [n for n, (blo, bhi) in shelf_bar.items()
                      if lo[0] < bhi[0] and blo[0] < hi[0] and lo[1] < bhi[1] and blo[1] < hi[1]
                      and abs(blo[2] - s["aabb"][1][2]) < 0.002]
            r.update({"dz_mm": round(dz * 1000, 4), "inside_top": inside, "on_barcode": on_bar})
            if abs(dz) > 0.001 or not inside or on_bar:
                fails.append(f"{p['name']}: dz {dz * 1000:.3f} mm, 윗면 안 {inside}, 바코드 겹침 {on_bar}")
        if not (p["rigid"] and p["body_collide"]):
            fails.append(f"{p['name']}: 강체 {p['rigid']}, 충돌 {p['body_collide']}")
        rows.append(r)
    if not 0 <= n_floor <= 3:
        fails.append(f"바닥 퍽 {n_floor}개")
    return not fails, {"fails": fails, "sequence": seq, "counts": cnt, "yellow_shape": sig["yellow_shape"],
                       "floor_pucks": n_floor, "pucks": rows}


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
    boxes += [("/World/SignalLight/" + k, v["aabb"]) for k, v in info["signal"]["parts"].items()]
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


def quad_on_panel(info, P):
    """사각형 점 4개 → (가장 가까운 벽판, 판 가운데 면에서 거리(법선 쪽 +), 판 안인지, 법선, 크기 2개)."""
    P = np.array(P)
    nrm = np.cross(P[1] - P[0], P[3] - P[0])
    nrm /= np.linalg.norm(nrm)
    c = P.mean(0)
    best = None
    for m in info["maze"]:
        if "/panel_" not in m["path"]:
            continue
        lo, hi = np.array(m["aabb"][0]), np.array(m["aabb"][1])
        axis = int(np.argmin(hi - lo))
        if abs(nrm[axis]) < 0.999:
            continue
        lat = [k for k in (0, 1) if k != axis][0]
        d = float((c - (lo + hi) / 2)[axis] * np.sign(nrm[axis]))
        if not (lo[lat] - 0.05 <= c[lat] <= hi[lat] + 0.05) or abs(d) > 0.02:
            continue
        inside = bool(P[:, lat].min() >= lo[lat] and P[:, lat].max() <= hi[lat] and P[:, 2].min() >= lo[2]
                      and P[:, 2].max() <= hi[2])
        if best is None or abs(d) < abs(best[1]):
            best = (m["path"], d, inside)
    return best, nrm, c, (float(np.linalg.norm(P[1] - P[0])), float(np.linalg.norm(P[3] - P[0])))


def judge_w6(info, lay, layout):
    fails = []
    checks = {}
    # 장소 태그
    tags = info["place_tags"]
    shelves = {s["place_tag_id"]: s for s in info["shelves"]}
    checks["place_tag_ids"] = (sorted(t["id"] for t in tags) == [12, 13, 14] and
                               sorted(shelves) == [12, 13, 14], sorted(t["id"] for t in tags))
    pt = {}
    for t in tags:
        on, nrm, c, size = quad_on_panel(info, t["points"])
        s = shelves.get(t["id"])
        sc = (np.array(s["aabb"][0]) + np.array(s["aabb"][1])) / 2 if s else c
        toward = float(np.dot(sc[:2] - c[:2], nrm[:2]))
        behind = float(np.linalg.norm((sc - c)[:2] - np.dot((sc - c)[:2], nrm[:2]) * nrm[:2]))   # 벽 방향 거리
        ok = (on is not None and abs(on[1] - 0.002) <= 0.0005 and on[2] and all(abs(x - 0.060) <= 0.0005 for x in size)
              and 0.13 <= c[2] <= 0.30 and toward > 0 and toward < 0.20 and behind <= 0.08 + 1e-9)
        pt[t["id"]] = {"panel": on[0] if on else None, "offset_m": round(on[1], 6) if on else None,
                       "inside_panel": on[2] if on else None, "size_m": [round(x, 6) for x in size],
                       "center_z": round(float(c[2]), 4), "shelf_front_m": round(toward, 4), "along_m": round(behind, 4),
                       "pass": bool(ok)}
        if not ok:
            fails.append(f"장소 태그 {t['id']}")
    # 바코드
    bars = info["barcodes"]
    bt = {}
    for b in bars:
        on, nrm, c, size = quad_on_panel(info, b["points"])
        dims = sorted(size)
        size_ok = abs(dims[1] - 0.035) <= 0.0005 and abs(dims[0] - 0.020) <= 0.0005
        if abs(nrm[2]) > 0.999:   # 누움 → 선반 윗면이어야 함
            P = np.array(b["points"])
            sh = [s for s in info["shelves"] if s["aabb"][0][0] <= P[:, 0].min() and P[:, 0].max() <= s["aabb"][1][0]
                  and s["aabb"][0][1] <= P[:, 1].min() and P[:, 1].max() <= s["aabb"][1][1]]
            dz = float(c[2] - sh[0]["aabb"][1][2]) if sh else None
            ok = size_ok and nrm[2] > 0 and len(sh) == 1 and abs(dz - 0.0005) <= 0.0005
            bt[b["text"]] = {"where": "shelf_top", "shelf": sh[0]["name"] if sh else None,
                             "dz_m": round(dz, 6) if dz is not None else None, "size_m": [round(x, 6) for x in dims],
                             "pass": bool(ok)}
        else:
            ok = size_ok and on is not None and abs(on[1] - 0.002) <= 0.0005 and on[2] and 0.10 <= c[2] <= 0.30
            bt[b["text"]] = {"where": "wall", "panel": on[0] if on else None,
                             "offset_m": round(on[1], 6) if on else None, "inside_panel": on[2] if on else None,
                             "center_z": round(float(c[2]), 4), "size_m": [round(x, 6) for x in dims], "pass": bool(ok)}
        if not ok:
            fails.append(f"바코드 {b['text']}")
    checks["barcode_count"] = (1 <= len(bars) <= 4, len(bars))
    # 테이프
    sb = lay["start_box"]
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
    fails += [k for k, (ok, _) in checks.items() if not ok]
    return not fails, {"fails": fails, "values": {k: v for k, (_, v) in checks.items()}, "place_tags": pt,
                       "barcodes": bt}


def judge_w7(info, lay):
    fails = []
    sg = info["signal"]
    P = sg["parts"]
    exp = {"base": (0.0198, 0.010), "pole": (0.005, 0.0589), "green": (0.0198, 0.0349), "yellow": (0.0198, 0.0349),
           "red": (0.0198, 0.0349), "cap": (0.0198, 0.0033)}
    dims = {}
    for k, (r, h) in exp.items():
        if k not in P:
            fails.append(f"{k} 없음")
            continue
        q = P[k]
        dims[k] = [round(q["radius"], 6), round(q["height"], 6)]
        if abs(q["radius"] - r) > 0.0005 or abs(q["height"] - h) > 0.0005 or abs(q["axis"][2]) < 0.9999:
            fails.append(f"{k}: r {q['radius']}, h {q['height']}, 축 {q['axis']}")
    if fails:
        return False, {"fails": fails, "dims": dims}
    top = P["cap"]["aabb"][1][2]
    order = [P[k]["center"][2] for k in ("green", "yellow", "red")]
    if abs(top - 0.1769) > 0.0005 or not order[0] < order[1] < order[2] or abs(P["base"]["aabb"][0][2]) > 1e-6:
        fails.append(f"높이 {top}, 칸 순서 {order}")
    head = P["green"]
    cx, cy, r = head["center"][0], head["center"][1], head["radius"]
    los = np.min([np.array(v["aabb"][0]) for v in P.values()], 0)
    his = np.max([np.array(v["aabb"][1]) for v in P.values()], 0)
    hit = [m["path"] for m in info["maze"] if overlap([los, his], m["aabb"])]
    hit += [s["name"] for s in info["shelves"] if overlap([los, his], s["aabb"])]
    hit += [p["name"] for p in info["pucks"] if overlap([los, his], p["body_aabb"])]
    gap = None
    for m in info["maze"]:
        if "/panel_" not in m["path"]:
            continue
        lo, hi = np.array(m["aabb"][0]), np.array(m["aabb"][1])
        dx = max(lo[0] - cx, 0, cx - hi[0])
        dy = max(lo[1] - cy, 0, cy - hi[1])
        d = math.hypot(dx, dy) - r
        gap = d if gap is None or d < gap else gap
    cell = cell_of(lay, cx, cy)
    banned = {lay["start_box"]["cell"]} | {s["cell"] for s in lay["shelves"]}
    tm = (0.1 <= sg["t_on"] <= 1.0 and 0.1 <= sg["t_off"] <= 1.0 and max(3 * sg["t_off"], 1.0) <= sg["t_term"] <= 5.0)
    if hit:
        fails.append(f"겹침 {hit}")
    if gap is None or abs(gap - 0.02) > 0.001:
        fails.append(f"벽판까지 {gap}")
    if cell in banned or cell is None:
        fails.append(f"칸 {cell}")
    if not tm:
        fails.append(f"시간 on {sg['t_on']} off {sg['t_off']} term {sg['t_term']}")
    return not fails, {"fails": fails, "dims": dims, "top_m": round(top, 6), "segment_z": [round(z, 4) for z in order],
                       "wall_gap_m": round(gap, 6) if gap is not None else None, "cell": cell, "overlap": hit,
                       "timing": [sg["t_on"], sg["t_off"], sg["t_term"]], "sequence": sg["sequence"]}


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
    col = {"red": (200, 30, 30), "green": (30, 150, 50)}
    for p in info["pucks"]:
        rect(p["body_aabb"], col[p["color"]] if p["location"] != "floor" else None, col[p["color"]])
    for m in info["place_tags"]:
        P = np.array(m["points"])
        g.line([T(*P[0][:2]), T(*P[1][:2])], fill=(200, 0, 200), width=5)
    for m in info["barcodes"]:
        P = np.array(m["points"])
        g.polygon([T(*q[:2]) for q in P], outline=(255, 140, 0))
    for k, v in info["signal"]["parts"].items():
        if k == "green":
            rect(v["aabb"], (255, 210, 0), (0, 0, 0))
    if robot_hull_xy is not None:
        g.polygon([T(*p) for p in robot_hull_xy], outline=(220, 120, 0))
    g.text((5, H - 25), "top view: +x right, +y up. center wall y=0 (bottom). orange outline = robot, magenta = place tag, "
           "orange box = barcode, yellow = signal light, outline-only puck = floor",
           fill=(0, 0, 0))
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
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
    result = {"seeds": args.seeds, "task": "B_dice", "criteria": "W v2 (사용자 확정 2026-10-23)", "W": {}}
    per = {}
    for seed in args.seeds:
        d = out / f"seed_{seed}"
        b = subprocess.run([PY, "-I", str(HERE / "build_task_world.py"), "--seed", str(seed), "--out", str(d)], capture_output=True, text=True)
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
        ok, det = judge_w3(info, lay)
        record("W3", ok, {"_seed": seed, **det})
        ok, w4 = judge_w4(info, robot)
        q4 = w4["q"]
        record("W4", ok, {"_seed": seed, **w4})
        ok, det = judge_w6(info, lay, layout)
        record("W6", ok, {"_seed": seed, **det})
        ok, det = judge_w7(info, lay)
        record("W7", ok, {"_seed": seed, **det})
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
    b2 = subprocess.run([PY, "-I", str(HERE / "build_task_world.py"), "--seed", str(s0), "--out", str(d2)], capture_output=True, text=True)
    same = {}
    for fn in ("task_world.usda", "layout.json"):
        a, c = (out / f"seed_{s0}" / fn), (d2 / fn)
        same[fn] = b2.returncode == 0 and a.read_bytes() == c.read_bytes()
    record("W5", all(same.values()), {"_seed": s0, "build_rc": b2.returncode, "identical": same})

    result["all_pass"] = all(v["pass"] for v in result["W"].values()) and len(result["W"]) == 7
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
