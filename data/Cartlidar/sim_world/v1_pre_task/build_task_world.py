"""가상 과제 월드(USD) 만들기 — 인지 파트 시험용, 모든 과제 조건은 가정.

구성: 미로 위쪽 반(도면 방향, data/maze_upper_drawing.json) + 바닥 + 선반 3개 + 퍽 12개(6면 AprilTag)
      + 과제 Aruco 마커 1개 + 출발 칸 테이프 + 로봇(261022_2119 USD 참조, 무작위 위치·방향·팔 자세) + 조명.
같은 seed면 같은 파일이 나온다(시각·임의 순서 없음).

실행 (PC): ~/isaacsim-venv/bin/python -I build_task_world.py --seed 1
결과: worlds/seed_<seed>/task_world.usda, layout.json
"""
import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from robot_geom import ARM_LINKS, Robot, box_depth, hull_depth  # noqa: E402

MODEL = Path.home() / "git/lv2_next_module/261022_2119"
ROBOT_USD = MODEL / "usd/waffle_4wd_261022_2119/waffle_4wd/waffle_4wd.usda"
URDF_PKG = MODEL / "urdf/waffle_4wd_description_261022_2119"

# ---- 가정값 (단위 m, kg). 공개·실측되면 바꾼다 ----
A = {
    "panel_height": 0.380,        # 실측(사용자 2026-10-23)
    "panel_thickness": 0.003,     # 실측
    "panel_bottom": 0.0,          # 가정: 판 아래 틈 없음
    "post_width": 0.0355,         # 실측 몸통 폭
    "post_body_top": 0.380,       # 가정: 몸통은 판 높이까지
    "post_head_diameter": 0.060,  # 실측
    "post_top": 0.400,            # 실측 꼭대기 높이(머리 두께 0.020은 가정)
    "shelf_size": [0.20, 0.20, 0.08],   # 높이 0.08만 사실, 가로·세로는 가정(설계 문서 5절)
    "shelf_count": 3,             # 가정
    "shelf_wall_gap": 0.02,       # 가정: 선반과 벽판 사이 틈
    "pucks_per_shelf": 4,         # 가정(가상 과제 A)
    "puck_cuboid": [0.03, 0.03, 0.06],  # 사실(사용자)
    "puck_cylinder": [0.03, 0.06],      # 지름·높이, 사실(사용자)
    "puck_mass": {"cuboid": 0.030, "cylinder": 0.024},  # 가정(PLA, 채움 비율 모름)
    "puck_shapes_per_color": "색마다 원기둥 1~3개(나머지 직육면체), 무작위 — 가정",
    "puck_standing_prob": 0.5,    # 가정: 서 있을 확률
    "tag_family": "tag36h11",     # 사실(AprilTag), 계열은 가정
    "tag_size": 0.018,            # 가정: 검은 테두리 바깥 한 변(원기둥 옆 4장이 둘레 94.2 mm에 들어가게)
    "tag_faces_cylinder": "옆면 4곳(90° 간격, 휘어진 스티커) + 위·아래 — 가정",
    "aruco_dict": "DICT_4X4_50",  # 가정
    "aruco_size": 0.10,           # 가정: 검은 테두리 바깥 한 변
    "aruco_center_height": 0.15,  # 가정(벽면 0.10~0.20 m)
    "aruco_place": "출발 칸이 있는 칸의 벽판 하나, 칸 안쪽을 향함 — 가정",
    "task_marker_id": {"A": 10, "B": 11, "C": 12},  # 가정(설계 문서 5절)
    "start_box_outer": 0.60,      # 가정
    "start_tape_width": 0.03,     # 가정
    "start_tape_thickness": 0.001,
    "start_cell": "c00",          # 사용자 지정(2026-10-23): 왼쪽 아래 구석 칸(왼쪽 바깥 벽·가운데 벽·긴 줄 벽 사이)
    "start_place": "출발 칸(c00, 0.734 x 0.788 m)의 가운데 — 칸은 사용자 지정, 칸 안 위치(가운데)도 사용자 지정",
    "robot_offset_max": 0.02,     # 가정: 출발 칸 가운데에서 로봇 중심의 무작위 어긋남
    "ground_friction": [0.8, 0.6],  # 시뮬레이션 연습 세션 시험 조건(가정, 실측 대기)
    "arm_drive": {"kp": 400.0, "kd": 24.0, "gripper_kp": 1000.0, "gripper_kd": 10.0,
                  "wheel_kd": 1.0},  # 시뮬레이션 연습 세션 v8 조건
}
CELL_Y = 0.788
COLORS = {"red": (0.75, 0.08, 0.07), "green": (0.10, 0.55, 0.18), "blue": (0.08, 0.22, 0.78)}
LIGHTS = {  # 시작값(사진 낮조명off / 낮조명on과 화면으로 맞출 것, 아직 맞추지 않음)
    "dark": {"dome": 300.0, "rect": 0.0},
    "bright": {"dome": 600.0, "rect": 20000.0},
}
ARM_JOINTS = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"]


def r6(v):
    return float(round(float(v), 6))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quat_from_R(R):
    q = Gf.Matrix3d(*np.asarray(R, float).T.ravel().tolist()).ExtractRotation().GetQuat()  # Gf는 행벡터 규약
    return q


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# ---------------- USD 도우미 ----------------
class Builder:
    def __init__(self, path):
        self.path = Path(path)
        self.stage = Usd.Stage.CreateNew(str(self.path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        world = UsdGeom.Xform.Define(self.stage, "/World")
        self.stage.SetDefaultPrim(world.GetPrim())
        self.mats = {}

    def rel(self, target):
        return Path(os.path.relpath(target, self.path.parent)).as_posix()

    def material(self, name, color, roughness=0.6, metallic=0.0, texture=None):
        if name in self.mats:
            return self.mats[name]
        mp = f"/World/Looks/{name}"
        mat = UsdShade.Material.Define(self.stage, mp)
        sh = UsdShade.Shader.Define(self.stage, mp + "/PBR")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        if texture:
            rd = UsdShade.Shader.Define(self.stage, mp + "/st_reader")
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
            tx = UsdShade.Shader.Define(self.stage, mp + "/texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(self.rel(texture)))
            tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
            tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
        else:
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[name] = mat
        return mat

    def physics_material(self, name, static, dynamic):
        mat = UsdShade.Material.Define(self.stage, f"/World/Looks/{name}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        return mat

    @staticmethod
    def bind(prim, mat, purpose=None):
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        if purpose:
            api.Bind(mat, UsdShade.Tokens.weakerThanDescendants, purpose)
        else:
            api.Bind(mat)

    def xform(self, path, t=(0, 0, 0), quat=None):
        x = UsdGeom.Xform.Define(self.stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(r6, t)))
        if quat is not None:
            x.AddOrientOp().Set(Gf.Quatf(quat))
        return x

    def box(self, path, center, size, mat=None, collide=True):
        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        c.AddTranslateOp().Set(Gf.Vec3d(*map(r6, center)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(r6, size)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if mat:
            self.bind(c.GetPrim(), mat)
        return c

    def cylinder(self, path, center, radius, height, mat=None, collide=True):
        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateRadiusAttr(r6(radius))
        c.CreateHeightAttr(r6(height))
        c.CreateAxisAttr("Z")
        c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
        c.AddTranslateOp().Set(Gf.Vec3d(*map(r6, center)))
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if mat:
            self.bind(c.GetPrim(), mat)
        return c

    def mesh(self, path, points, faces, st, mat):
        m = UsdGeom.Mesh.Define(self.stage, path)
        pts = np.asarray(points, float)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(r6, p)) for p in pts]))
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        m.CreateExtentAttr([Gf.Vec3f(*map(r6, pts.min(0))), Gf.Vec3f(*map(r6, pts.max(0)))])
        m.CreateSubdivisionSchemeAttr("none")
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*map(r6, s)) for s in st]))
        self.bind(m.GetPrim(), mat)
        return m


def quad(center, u, v, half_u, half_v):
    """u·v 방향 사각형(앞면 = u x v 방향). st: u -> s, v -> t."""
    c, u, v = (np.asarray(a, float) for a in (center, u, v))
    pts = [c - half_u * u - half_v * v, c + half_u * u - half_v * v, c + half_u * u + half_v * v,
           c - half_u * u + half_v * v]
    return pts, [[0, 1, 2, 3]], [(0, 0), (1, 0), (1, 1), (0, 1)]


# ---------------- 배치 ----------------
def cells_and_walls(layout):
    xs = layout["grid"]["x"]
    posts = {p["id"]: (p["x"], p["y"]) for p in layout["posts"]}
    xpan = {(round(pn["center"][0], 3), round(pn["center"][1], 3)): pn for pn in layout["panels"] if pn["axis"] == "x"}
    ypan = {(round(pn["center"][0], 3), round(pn["center"][1], 3)): pn for pn in layout["panels"] if pn["axis"] == "y"}
    rows = [round(k * CELL_Y, 3) for k in range(5)]
    cells = []
    for i in range(len(xs) - 1):
        for k in range(4):
            x0, x1, y0, y1 = xs[i], xs[i + 1], rows[k], rows[k + 1]
            cx, cy = round((x0 + x1) / 2, 4), round((y0 + y1) / 2, 4)
            edges = []
            for side, key, normal in (("bottom", (round(cx, 3), y0), (0, 1)), ("top", (round(cx, 3), y1), (0, -1))):
                if key in xpan:
                    edges.append({"side": side, "panels": [xpan[key]["id"]], "line": key[1], "axis": "x",
                                  "normal": normal, "span": (x0, x1)})
            for side, xx, normal in (("left", x0, (1, 0)), ("right", x1, (-1, 0))):
                keys = [(round(xx, 3), round(y0 + 0.197, 3)), (round(xx, 3), round(y0 + 0.591, 3))]
                if all(kk in ypan for kk in keys):
                    edges.append({"side": side, "panels": [ypan[kk]["id"] for kk in keys], "line": xx, "axis": "y",
                                  "normal": normal, "span": (y0, y1)})
            cells.append({"id": f"c{i}{k}", "x": (x0, x1), "y": (y0, y1), "center": (cx, cy), "edges": edges})
    return cells, posts


def world_boxes(layout, shelves, pucks):
    """로봇 겹침 계산용 상자 목록 (center, half, yaw=0, 이름)."""
    out = []
    H, T, W = A["panel_height"], A["panel_thickness"], A["post_width"]
    for pn in layout["panels"]:
        L = pn["length"]
        size = (L, T, H) if pn["axis"] == "x" else (T, L, H)
        out.append(((pn["center"][0], pn["center"][1], A["panel_bottom"] + H / 2), np.array(size) / 2,
                    f"panel_{pn['id']:02d}"))
    hz = A["post_top"] - A["post_body_top"]
    for p in layout["posts"]:
        out.append(((p["x"], p["y"], A["post_body_top"] / 2), np.array([W, W, A["post_body_top"]]) / 2,
                    f"post_{p['id']:02d}_body"))
        d = A["post_head_diameter"]
        out.append(((p["x"], p["y"], A["post_body_top"] + hz / 2), np.array([d, d, hz]) / 2, f"post_{p['id']:02d}_head"))
    for s in shelves:
        out.append((tuple(s["center"]), np.array(A["shelf_size"]) / 2, s["name"]))
    for pk in pucks:
        lo, hi = np.array(pk["aabb"][0]), np.array(pk["aabb"][1])
        out.append((tuple((lo + hi) / 2), (hi - lo) / 2, pk["name"]))
    return out


def robot_hits(robot, q, T, boxes, near=0.9):
    ws = robot.world_shapes(q, T)
    base_xy = T[:2, 3]
    hits = []
    for c, h, name in boxes:
        if np.abs(np.asarray(c[:2]) - base_xy).max() > near + h[:2].max():
            continue
        for link, V in ws:
            d = box_depth(V, c, h)
            if d >= 0:
                hits.append((link, name, d))
                break
    return hits


def robot_self_ok(robot, q, T):
    ws = robot.world_shapes(q, T)
    for link, V in ws:  # 바닥 아래로 들어가면 안 됨(바퀴 제외)
        if link in ARM_LINKS and V[:, 2].min() < 0.005:
            return False
    arm = [(l, V) for l, V in ws if l in ARM_LINKS]
    for la, Va in ws:
        for lb, Vb in arm:
            if la == lb or frozenset((la, lb)) in robot.adjacent:
                continue
            if hull_depth(Va, Vb) >= 0:
                return False
    return True


def puck_pose(rng, shape):
    standing = bool(rng.random() < A["puck_standing_prob"])
    yaw = float(rng.uniform(-math.pi, math.pi))
    if standing:
        flip = bool(rng.random() < 0.5)
        R = Rz(yaw) @ (np.diag([1.0, -1.0, -1.0]) if flip else np.eye(3))
        half_z = A["puck_cuboid"][2] / 2
    else:
        roll = float(rng.integers(0, 4)) * math.pi / 2 if shape == "cuboid" else float(rng.uniform(-math.pi, math.pi))
        R = Rz(yaw) @ Ry(math.pi / 2) @ Rz(roll)
        half_z = A["puck_cuboid"][0] / 2
    return standing, R, half_z


def puck_local_corners(shape):
    if shape == "cuboid":
        s = np.array(A["puck_cuboid"]) / 2
    else:
        d, h = A["puck_cylinder"]
        s = np.array([d / 2, d / 2, h / 2])  # 원기둥은 외접 상자(보수적)
    return np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[1], s[1]) for z in (-s[2], s[2])])


def plan(seed, task, layout, robot):
    rng = np.random.default_rng(seed)
    cells, posts = cells_and_walls(layout)
    walled = [c for c in cells if c["edges"]]
    # 출발 칸(고정)
    start = next(c for c in cells if c["id"] == A["start_cell"])
    # 선반
    others = [c for c in walled if c["id"] != start["id"]]
    pick = rng.choice(len(others), size=A["shelf_count"], replace=False)
    sx, sy, sz = A["shelf_size"]
    shelves = []
    for n, idx in enumerate(sorted(int(i) for i in pick)):
        c = others[idx]
        e = c["edges"][int(rng.integers(len(c["edges"])))]
        lo, hi = e["span"]
        room = (hi - lo) / 2 - A["post_width"] / 2 - sx / 2 - 0.01
        along = (lo + hi) / 2 + float(rng.uniform(-room, room))
        inward = e["line"] + (A["panel_thickness"] / 2 + A["shelf_wall_gap"] + sy / 2) * (
            e["normal"][1] if e["axis"] == "x" else e["normal"][0])
        center = (along, inward, sz / 2) if e["axis"] == "x" else (inward, along, sz / 2)
        shelves.append({"name": f"shelf_{n}", "cell": c["id"], "wall_side": e["side"],
                        "center": [r6(v) for v in center], "size": A["shelf_size"], "top_z": sz})
    # 퍽: 색별 4개, 색마다 원기둥 1~3개
    specs = []
    for color in ("red", "green", "blue"):
        n_cyl = int(rng.integers(1, 4))
        shapes = ["cylinder"] * n_cyl + ["cuboid"] * (4 - n_cyl)
        specs += [(color, s) for s in shapes]
    order = rng.permutation(12)
    specs = [specs[i] for i in order]
    tag_ids = [int(t) for t in rng.permutation(12)]
    pucks = []
    for n, (color, shape) in enumerate(specs):
        shelf = shelves[n // A["pucks_per_shelf"]]
        for _ in range(500):
            standing, R, half_z = puck_pose(rng, shape)
            corners = puck_local_corners(shape) @ R.T
            half_xy = np.abs(corners[:, :2]).max(0)
            m = 0.003
            lim = np.array([sx, sy]) / 2 - half_xy - m
            if (lim <= 0).any():
                continue
            off = rng.uniform(-lim, lim)
            cxy = np.array(shelf["center"][:2]) + off
            ok = all(np.any(np.abs(cxy - np.array(o["center"][:2])) > half_xy + np.array(o["half_xy"]) + 0.005)
                     for o in pucks if o["shelf"] == shelf["name"])
            if ok:
                break
        else:
            raise RuntimeError(f"퍽 {n} 자리 못 찾음")
        center = [r6(cxy[0]), r6(cxy[1]), r6(shelf["top_z"] + half_z)]
        aabb_half = np.abs(corners).max(0)
        pucks.append({"name": f"puck_{tag_ids[n]:02d}", "tag_id": tag_ids[n], "color": color, "shape": shape,
                      "standing": standing, "shelf": shelf["name"], "center": center,
                      "R": [[r6(v) for v in row] for row in R], "half_xy": [r6(v) for v in half_xy],
                      "aabb": [[r6(c - h) for c, h in zip(center, aabb_half)],
                               [r6(c + h) for c, h in zip(center, aabb_half)]]})
    # 과제 마커: 출발 칸 벽 중 하나
    e = start["edges"][int(rng.integers(len(start["edges"])))]
    pid = e["panels"][int(rng.integers(len(e["panels"])))]
    pn = next(p for p in layout["panels"] if p["id"] == pid)
    nrm = np.array([*e["normal"], 0.0])
    off = A["panel_thickness"] / 2 + 0.0005
    mc = np.array([pn["center"][0], pn["center"][1], A["aruco_center_height"]]) + nrm * off
    marker = {"id": A["task_marker_id"][task], "panel": pid, "cell": start["id"], "normal": nrm.tolist(),
              "center": [r6(v) for v in mc], "size": A["aruco_size"]}
    # 로봇: 출발 칸 가운데, 무작위 방향·팔 자세. 겹치면 다시 뽑는다
    boxes = world_boxes(layout, shelves, pucks)
    ws0 = robot.world_shapes({})
    body = np.vstack([V for l, V in ws0 if l not in ARM_LINKS])
    body_c = (body[:, :2].min(0) + body[:, :2].max(0)) / 2
    lims = {j: (robot.joint_by_name[j]["lower"], robot.joint_by_name[j]["upper"]) for j in ARM_JOINTS}
    g_lo, g_hi = 0.0, robot.joint_by_name["gripper_left_joint"]["upper"]
    tries = 0
    while True:
        tries += 1
        if tries > 400:
            raise RuntimeError("로봇 출발 자세를 못 찾음")
        yaw = float(rng.uniform(-math.pi, math.pi))
        off = rng.uniform(-A["robot_offset_max"], A["robot_offset_max"], 2)
        q = {j: float(rng.uniform(*lims[j])) for j in ARM_JOINTS}
        q["gripper_left_joint"] = float(rng.uniform(g_lo, g_hi))
        R = Rz(yaw)
        pos = np.array(start["center"]) + off - R[:2, :2] @ body_c
        T = np.eye(4)
        T[:3, :3] = R
        T[:2, 3] = pos
        if not robot_self_ok(robot, q, T):
            continue
        if robot_hits(robot, q, T, boxes):
            continue
        break
    robot_pose = {"x": r6(pos[0]), "y": r6(pos[1]), "yaw": r6(yaw), "q": {k: r6(v) for k, v in q.items()},
                  "tries": tries, "start_cell": start["id"]}
    start_box = {"cell": start["id"], "center": [r6(v) for v in start["center"]], "outer": A["start_box_outer"],
                 "tape_width": A["start_tape_width"]}
    return {"cells": cells, "start_box": start_box, "shelves": shelves, "pucks": pucks, "marker": marker,
            "robot": robot_pose}


# ---------------- USD 쓰기 ----------------
def write_usd(path, layout, p, robot, light):
    b = Builder(path)
    st = b.stage
    UsdPhysics.Scene.Define(st, "/World/physicsScene").CreateGravityMagnitudeAttr(9.81)
    # 재질
    m_carpet = b.material("carpet_gray", (0.30, 0.31, 0.32), roughness=0.95)
    m_panel = b.material("acrylic_bluegray", (0.30, 0.35, 0.45), roughness=0.15)
    m_al = b.material("aluminium", (0.75, 0.76, 0.78), roughness=0.35, metallic=1.0)
    m_print = b.material("printed_blue", (0.10, 0.20, 0.75), roughness=0.5)
    m_shelf = b.material("shelf_gray", (0.55, 0.55, 0.55), roughness=0.8)
    m_tape = b.material("tape_white", (0.92, 0.92, 0.92), roughness=0.7)
    m_floor_phys = b.physics_material("floor_contact", *A["ground_friction"])
    # 바닥(무한 평면 충돌 + 보이는 판)
    xs, ys = layout["grid"]["x"], layout["grid"]["y"]
    cx, cy = (xs[0] + xs[-1]) / 2, (ys[0] + ys[-1]) / 2
    g = UsdGeom.Xform.Define(st, "/World/Ground")
    plane = UsdGeom.Plane.Define(st, "/World/Ground/CollisionPlane")
    plane.CreateAxisAttr("Z")
    plane.CreateWidthAttr(12.0)
    plane.CreateLengthAttr(12.0)
    plane.CreateExtentAttr([Gf.Vec3f(-6, -6, 0), Gf.Vec3f(6, 6, 0)])
    plane.AddTranslateOp().Set(Gf.Vec3d(r6(cx), r6(cy), 0))
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    b.bind(plane.GetPrim(), m_carpet)
    b.bind(plane.GetPrim(), m_floor_phys, "physics")
    # 미로
    H, T, W = A["panel_height"], A["panel_thickness"], A["post_width"]
    UsdGeom.Xform.Define(st, "/World/Maze")
    for pn in layout["panels"]:
        L = pn["length"]
        size = (L, T, H) if pn["axis"] == "x" else (T, L, H)
        prim = b.box(f"/World/Maze/panel_{pn['id']:02d}", (pn["center"][0], pn["center"][1], A["panel_bottom"] + H / 2),
                     size, m_panel).GetPrim()
        prim.CreateAttribute("maze:shared", Sdf.ValueTypeNames.Bool).Set(bool(pn["shared"]))
        prim.CreateAttribute("maze:kind", Sdf.ValueTypeNames.String).Set(pn["kind"])
    hz = A["post_top"] - A["post_body_top"]
    for po in layout["posts"]:
        base = f"/World/Maze/post_{po['id']:02d}"
        x = UsdGeom.Xform.Define(st, base)
        x.GetPrim().CreateAttribute("maze:post_type", Sdf.ValueTypeNames.String).Set(po["type"])
        b.box(base + "/body", (po["x"], po["y"], A["post_body_top"] / 2), (W, W, A["post_body_top"]),
              m_al if po["type"] == "al" else m_print)
        b.cylinder(base + "/head", (po["x"], po["y"], A["post_body_top"] + hz / 2), A["post_head_diameter"] / 2, hz,
                   m_print)
    # 출발 칸 테이프(충돌 없음)
    sb = p["start_box"]
    o, w, th = sb["outer"], sb["tape_width"], A["start_tape_thickness"]
    c0 = np.array(sb["center"])
    UsdGeom.Xform.Define(st, "/World/StartBox")
    for name, dc, size in (("tape_s", (0, -(o - w) / 2), (o, w)), ("tape_n", (0, (o - w) / 2), (o, w)),
                           ("tape_w", (-(o - w) / 2, 0), (w, o - 2 * w)), ("tape_e", ((o - w) / 2, 0), (w, o - 2 * w))):
        b.box(f"/World/StartBox/{name}", (c0[0] + dc[0], c0[1] + dc[1], th / 2), (size[0], size[1], th), m_tape,
              collide=False)
    # 과제 마커
    mk = p["marker"]
    tex = HERE / f"textures/aruco4x4_{mk['id']:02d}.png"
    m_mk = b.material(f"aruco_{mk['id']:02d}", (1, 1, 1), roughness=0.8, texture=tex)
    nrm = np.array(mk["normal"])
    u = np.cross([0, 0, 1.0], nrm)  # 바깥에서 볼 때 오른쪽
    half = mk["size"] * 8 / 6 / 2   # 흰 여백 1칸 포함
    pts, faces, stc = quad(mk["center"], u, [0, 0, 1.0], half, half)
    mm = b.mesh(f"/World/TaskMarker/aruco_{mk['id']:02d}", pts, faces, stc, m_mk)
    for k, v in (("marker:id", mk["id"]), ("marker:panel", mk["panel"])):
        mm.GetPrim().CreateAttribute(k, Sdf.ValueTypeNames.Int).Set(int(v))
    mm.GetPrim().CreateAttribute("marker:size", Sdf.ValueTypeNames.Double).Set(mk["size"])
    mm.GetPrim().CreateAttribute("marker:dictionary", Sdf.ValueTypeNames.String).Set(A["aruco_dict"])
    # 선반
    UsdGeom.Xform.Define(st, "/World/Shelves")
    for s in p["shelves"]:
        b.box(f"/World/Shelves/{s['name']}", s["center"], s["size"], m_shelf)
    # 퍽
    UsdGeom.Xform.Define(st, "/World/Pucks")
    tag_half = A["tag_size"] * 10 / 8 / 2
    for pk in p["pucks"]:
        base = f"/World/Pucks/{pk['name']}"
        x = b.xform(base, pk["center"], quat_from_R(pk["R"]))
        prim = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(A["puck_mass"][pk["shape"]])
        for k, v in (("puck:color", pk["color"]), ("puck:shape", pk["shape"]), ("puck:shelf", pk["shelf"])):
            prim.CreateAttribute(k, Sdf.ValueTypeNames.String).Set(v)
        prim.CreateAttribute("puck:tag_id", Sdf.ValueTypeNames.Int).Set(pk["tag_id"])
        prim.CreateAttribute("puck:standing", Sdf.ValueTypeNames.Bool).Set(pk["standing"])
        m_body = b.material(f"puck_{pk['color']}", COLORS[pk["color"]], roughness=0.85)
        m_tag = b.material(f"tag_{pk['tag_id']:02d}", (1, 1, 1), roughness=0.8,
                           texture=HERE / f"textures/tag36h11_{pk['tag_id']:02d}.png")
        if pk["shape"] == "cuboid":
            sx, sy, sz = A["puck_cuboid"]
            b.box(base + "/body", (0, 0, 0), (sx, sy, sz), m_body)
            e = 0.0002
            faces = {"px": ((sx / 2 + e, 0, 0), (0, 1, 0), (0, 0, 1)), "nx": ((-sx / 2 - e, 0, 0), (0, -1, 0), (0, 0, 1)),
                     "py": ((0, sy / 2 + e, 0), (-1, 0, 0), (0, 0, 1)), "ny": ((0, -sy / 2 - e, 0), (1, 0, 0), (0, 0, 1)),
                     "pz": ((0, 0, sz / 2 + e), (1, 0, 0), (0, 1, 0)), "nz": ((0, 0, -sz / 2 - e), (1, 0, 0), (0, -1, 0))}
            for fn, (c, u, v) in faces.items():
                pts, fc, stc = quad(c, u, v, tag_half, tag_half)
                b.mesh(f"{base}/tag_{fn}", pts, fc, stc, m_tag)
        else:
            d, h = A["puck_cylinder"]
            r = d / 2
            b.cylinder(base + "/body", (0, 0, 0), r, h, m_body)
            e = 0.0002
            # 위·아래: 원판(흰 여백은 원 안까지만)
            n = 32
            ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
            for fn, z, sgn in (("pz", h / 2 + e, 1), ("nz", -h / 2 - e, -1)):
                rim = [(r * 0.98 * math.cos(a), sgn * r * 0.98 * math.sin(a), z) for a in ang]
                pts = [(0, 0, z)] + rim
                faces = [[0, 1 + i, 1 + (i + 1) % n] for i in range(n)]
                stc = [(0.5 + q[0] / (2 * tag_half), 0.5 + sgn * q[1] / (2 * tag_half)) for q in pts]
                b.mesh(f"{base}/tag_{fn}", pts, faces, stc, m_tag)
            # 옆면 4곳: 휘어진 조각
            seg = 8
            half_ang = tag_half / r
            rr = r + e
            for k in range(4):
                a0 = k * math.pi / 2
                pts, stc = [], []
                for zi, z in enumerate((-tag_half, tag_half)):
                    for i in range(seg + 1):
                        a = a0 - half_ang + 2 * half_ang * i / seg
                        pts.append((rr * math.cos(a), rr * math.sin(a), z))
                        stc.append((i / seg, zi))
                faces = [[i, i + 1, seg + 2 + i, seg + 1 + i] for i in range(seg)]
                b.mesh(f"{base}/tag_side{k}", pts, faces, stc, m_tag)
    # 로봇
    rp = p["robot"]
    rprim = st.DefinePrim("/World/waffle_4wd", "Xform")
    rprim.GetReferences().AddReference(b.rel(ROBOT_USD))
    rprim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    rx = UsdGeom.Xformable(rprim)
    rx.AddTranslateOp().Set(Gf.Vec3d(rp["x"], rp["y"], 0.0))
    rx.AddRotateZOp().Set(r6(math.degrees(rp["yaw"])))
    root = "/World/waffle_4wd/Geometry/base_footprint/base_link"
    for w in ("front_left", "front_right", "rear_left", "rear_right"):
        tp = st.GetPrimAtPath(f"{root}/wheel_{w}/tire_cylinder")
        if not tp.IsValid():
            raise RuntimeError(f"바퀴 충돌체 없음: wheel_{w}")
        b.bind(tp, m_floor_phys, "physics")
    q = robot.full_q(rp["q"])
    ad = A["arm_drive"]
    for jp in st.GetPrimAtPath("/World/waffle_4wd/Physics").GetChildren():
        name = jp.GetName()
        if name.startswith("RMX52_"):
            deg = r6(math.degrees(q[name]))
            jp.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(deg)
            jp.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float).Set(deg)
            jp.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float).Set(
                r6(ad["kp"] * math.pi / 180))
            jp.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float).Set(r6(ad["kd"] * math.pi / 180))
        elif name.startswith("gripper_"):
            v = r6(q[name])
            jp.CreateAttribute("state:linear:physics:position", Sdf.ValueTypeNames.Float).Set(v)
            jp.CreateAttribute("drive:linear:physics:targetPosition", Sdf.ValueTypeNames.Float).Set(v)
            left = name == "gripper_left_joint"
            jp.CreateAttribute("drive:linear:physics:stiffness", Sdf.ValueTypeNames.Float).Set(
                ad["gripper_kp"] if left else 0.0)
            jp.CreateAttribute("drive:linear:physics:damping", Sdf.ValueTypeNames.Float).Set(
                ad["gripper_kd"] if left else 0.0)
        elif name.startswith("Wheel_"):
            jp.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float).Set(0.0)
            jp.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float).Set(
                r6(ad["wheel_kd"] * math.pi / 180))
            jp.CreateAttribute("drive:angular:physics:targetVelocity", Sdf.ValueTypeNames.Float).Set(0.0)
    # 조명
    L = LIGHTS[light]
    UsdLux.DomeLight.Define(st, "/World/Lights/dome").CreateIntensityAttr(L["dome"])
    if L["rect"] > 0:
        for i, (lx, ly) in enumerate([(xs[1], cy - 0.8), (xs[3], cy - 0.8), (xs[5], cy - 0.8),
                                       (xs[1], cy + 0.8), (xs[3], cy + 0.8), (xs[5], cy + 0.8)]):
            rl = UsdLux.RectLight.Define(st, f"/World/Lights/ceiling_{i}")
            rl.CreateWidthAttr(0.6)
            rl.CreateHeightAttr(0.6)
            rl.CreateIntensityAttr(L["rect"])
            rl.AddTranslateOp().Set(Gf.Vec3d(r6(lx), r6(ly), 2.6))  # RectLight는 -Z로 빛을 낸다
    st.GetRootLayer().customLayerData = {"seed": int(p["seed"]), "task": p["task"], "light": light}
    st.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--task", choices=["A", "B", "C"], default="A")
    ap.add_argument("--light", choices=sorted(LIGHTS), default="dark")
    ap.add_argument("--layout", type=Path, default=HERE / "data/maze_upper_drawing.json")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    out = args.out or HERE / f"worlds/seed_{args.seed}"
    out.mkdir(parents=True, exist_ok=True)
    layout = json.loads(args.layout.read_text())
    robot = Robot(URDF_PKG / "urdf/waffle_4wd.urdf", URDF_PKG)
    p = plan(args.seed, args.task, layout, robot)
    p.update({"seed": args.seed, "task": args.task, "light": args.light})
    usd = out / "task_world.usda"
    if usd.exists():
        usd.unlink()
    write_usd(usd, layout, p, robot, args.light)
    meta = {"seed": args.seed, "task": args.task, "light": args.light, "assumptions": A,
            "sources": {"layout": str(args.layout), "layout_sha256": sha256(args.layout),
                        "robot_usd": str(ROBOT_USD), "urdf": str(URDF_PKG / "urdf/waffle_4wd.urdf"),
                        "urdf_sha256": sha256(URDF_PKG / "urdf/waffle_4wd.urdf")},
            **{k: v for k, v in p.items() if k not in ("seed", "task", "light")}}
    (out / "layout.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
    print(f"seed {args.seed}: start {p['start_box']['cell']}, shelves {[s['cell'] for s in p['shelves']]}, "
          f"robot tries {p['robot']['tries']}, wrote {usd}")


if __name__ == "__main__":
    main()
